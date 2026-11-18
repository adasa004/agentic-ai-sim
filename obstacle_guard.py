"""Filter requested rover velocity using only lidar and camera observations."""
import math
import os
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan

from llm_recovery import DECISION_BUDGET, WAIT_SECONDS, LLMAdvisor
from make_world import AISLE_Y, CAMERA_FOV, CAMERA_X, ENTRY_X, ULTRASONIC_MOUNTS, VISION_MARKER_WIDTH

SENSOR_TIMEOUT = .75
COMMAND_TIMEOUT = .4
ROVER_RADIUS = .44


def camera_points(message):
    """Estimate the range and bearing of magenta traffic markers."""
    if message.encoding not in ("rgb8", "bgr8"):
        return None
    pixels = np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, message.step)
    pixels = pixels[:, :message.width * 3].reshape(message.height, message.width, 3)
    # Magenta has strong first and third channels in both RGB and BGR.
    mask = cv2.inRange(pixels, (150, 0, 150), (255, 120, 255))
    _, _, boxes, _ = cv2.connectedComponentsWithStats(mask)
    focal = message.width / (2 * math.tan(CAMERA_FOV / 2))
    points = []
    for left, _, width, height, area in boxes[1:]:
        if width < 5 or height < 2 or area < 12:
            continue
        depth = focal * VISION_MARKER_WIDTH / width
        lateral = (message.width / 2 - (left + width / 2)) * depth / focal
        points.append((CAMERA_X + depth, lateral))
    return points


def ultrasonic_points(side, scan):
    x, y, yaw = ULTRASONIC_MOUNTS[side]
    points = []
    for i, distance in enumerate(scan.ranges):
        if scan.range_min <= distance <= scan.range_max:
            heading = yaw + scan.angle_min + i * scan.angle_increment
            points.append((x + distance * math.cos(heading), y + distance * math.sin(heading)))
    return points


def swept_path(command, duration):
    linear, angular = command.linear.x, command.angular.z
    for i in range(11):
        t = duration * i / 10
        if abs(angular) < .001:
            yield linear * t, 0.0
        else:
            yield (linear / angular * math.sin(angular * t),
                   linear / angular * (1 - math.cos(angular * t)))


def obstacle_in_path(points, command, duration, radius):
    """Sample the rover's swept footprint for the requested motion."""
    return any((px - x)**2 + (py - y)**2 <= radius**2
               for x, y in swept_path(command, duration) for px, py in points)


def recovery_options(pose, request, observations, excluded_turns=()):
    """Enumerate feasible pull-over arcs for the model to evaluate."""
    x, y, yaw = pose
    aisle = math.copysign(AISLE_Y, y)
    offset = y - aisle
    direction = -math.copysign(1, aisle)
    side = math.copysign(1, offset)
    # Only steer off route on straight aisle sections with a known row side.
    if (not ENTRY_X + 5 < x < 13 or not 1.4 <= abs(offset) < 2.7
            or request.linear.x * math.cos(yaw) * direction <= 0):
        return []
    horizon = 1.6
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    options = []
    for speed in (.30, .45):
        for turn in (-.55, -.35, 0., .35, .55):
            if turn in excluded_turns:
                continue
            command = Twist()
            command.linear.x = math.copysign(speed, request.linear.x)
            command.angular.z = turn
            path = [(lx * cos_yaw - ly * sin_yaw, lx * sin_yaw + ly * cos_yaw)
                    for lx, ly in swept_path(command, horizon)]
            if any(not ENTRY_X + 4 < x + dx < 14 or direction * dx < -.02
                   or not abs(offset) - .02 <= (offset + dy) * side <= 2.75
                   for dx, dy in path):
                continue
            progress, lateral = path[-1]
            if lateral * side < .12:
                continue
            if any(obstacle_in_path(points, command, horizon, margin)
                   for _, points, margin in observations):
                continue
            options.append((direction * progress, lateral * side, command))
    return options


def maneuver_id(command):
    return f"arc_{command.linear.x:+.2f}_{command.angular.z:+.2f}"


class RecoveryAgent:
    """Observe, choose, execute and verify bounded pull-over maneuvers."""

    def __init__(self, advisor=None):
        self.advisor = advisor
        self.reset()

    def reset(self):
        self.progress_check = None
        self.last_turn = None
        self.rejected_turns = set()
        self.wait_until_clear = False
        self.requests = 0
        self.retry_after = 0.0
        self.advice = None
        self.advice_deadline = 0.0
        self.advised_wait_until = 0.0
        self.advised_arc = None

    def act(self, now, pose, request, observations):
        if self.progress_check is not None:
            started, previous_y = self.progress_check
            if now - started >= 1.0:
                if abs(pose[1] - previous_y) < .05:
                    self.rejected_turns.add(self.last_turn)
                    self.advised_arc = None
                self.progress_check = None

        if len(self.rejected_turns) >= 2:
            self.wait_until_clear = True
        if self.wait_until_clear:
            return Twist(), "Recovery agent: waiting after unsuccessful maneuvers"

        # Candidate IDs are rebuilt from current observations every tick.
        options = recovery_options(pose, request, observations, self.rejected_turns)
        if self.advisor is None:
            return Twist(), "Recovery agent: stopped, OpenAI recovery disabled"

        # If a previous selection no longer fits, ask again using only
        # the maneuvers that are currently feasible.
        advised = next((cmd for _, _, cmd in options
                        if maneuver_id(cmd) == self.advised_arc), None)
        if advised is None:
            self.advised_arc = None
        if (self.advice is None and self.advised_arc is None and options
                and now >= self.retry_after and now >= self.advised_wait_until
                and self.requests < 3):
            candidates = [
                {"id": maneuver_id(cmd), "forward_m": round(forward, 3),
                 "toward_row_m": round(toward_row, 3),
                 "speed_mps": abs(cmd.linear.x), "turn_rad_s": cmd.angular.z}
                for forward, toward_row, cmd in options
            ]
            evidence = {
                source: round(min((math.hypot(x, y) for x, y in points),
                                  default=99.0), 2) if points else None
                for source, points, _ in observations
            }
            self.advice = self.advisor.submit(candidates, evidence)
            if self.advice is not None:
                self.requests += 1
                self.advice_deadline = now + DECISION_BUDGET
            self.retry_after = now + DECISION_BUDGET + 1.0
        if self.advice is not None:
            if now >= self.advice_deadline:
                self.advice = None
            elif not self.advice.done():
                return Twist(), "Recovery agent: waiting for LLM ranking"
            else:
                try:
                    choice = self.advice.result()
                except (OSError, ValueError, KeyError, TypeError):
                    choice = None
                self.advice = None
                if choice == "WAIT":
                    self.advised_wait_until = now + WAIT_SECONDS
                else:
                    self.advised_arc = choice

        if now < self.advised_wait_until:
            return Twist(), "Recovery agent: LLM selected a brief wait"
        # Use the model's choice only while it is locally feasible.
        advised = next((cmd for _, _, cmd in options
                        if maneuver_id(cmd) == self.advised_arc), None)
        if advised is None:
            self.progress_check = None
            return Twist(), "Recovery agent: stopped, no valid OpenAI maneuver"
        if self.progress_check is None:
            self.progress_check = (now, pose[1])
        self.last_turn = advised.angular.z
        return advised, "Recovery agent: executing locally checked LLM choice"


class ObstacleGuard(Node):
    def __init__(self):
        super().__init__("lot10_obstacle_guard")
        self.output = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Twist, "/cmd_vel_requested", self.receive_command, 10)
        self.create_subscription(LaserScan, "/scan", self.receive_scan, qos_profile_sensor_data)
        self.create_subscription(Image, "/plate_camera/image", self.receive_image,
                                 qos_profile_sensor_data)
        self.declare_parameter("recovery_enabled", True)
        self.recovery_enabled = self.get_parameter("recovery_enabled").value
        advisor = LLMAdvisor.from_environment() if self.recovery_enabled else None
        self.recovery_agent = RecoveryAgent(advisor)
        if advisor is not None:
            self.get_logger().info(f"LLM recovery ranking enabled ({advisor.model}).")
        elif os.environ.get("LOT10_LLM") == "1" and self.recovery_enabled:
            self.get_logger().warning("OPENAI_API_KEY is missing. The rover will stop for blocked paths.")
        self.create_subscription(PoseStamped, "/patrol/rover_pose", self.receive_pose, 10)
        self.pose = None
        self.last_pose = -math.inf
        self.command = Twist()
        self.lidar = []
        self.vision = []
        self.ultrasonic = {side: (-math.inf, []) for side in ULTRASONIC_MOUNTS}
        for side in ULTRASONIC_MOUNTS:
            self.create_subscription(
                LaserScan, f"/ultrasonic/{side}",
                lambda scan, side=side: self.receive_ultrasonic(side, scan),
                qos_profile_sensor_data,
            )
        self.last_command = self.last_scan = self.last_image = -math.inf
        self.status = None
        self.create_timer(.05, self.filter_command)

    def receive_pose(self, message):
        p, q = message.pose.position, message.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = (p.x, p.y, yaw)
        self.last_pose = time.monotonic()

    def receive_command(self, command):
        self.command = command
        self.last_command = time.monotonic()

    def receive_scan(self, scan):
        self.lidar = []
        for i, distance in enumerate(scan.ranges):
            if scan.range_min <= distance <= min(scan.range_max, 5.0):
                angle = scan.angle_min + i * scan.angle_increment
                self.lidar.append((distance * math.cos(angle), distance * math.sin(angle)))
        self.last_scan = time.monotonic()

    def receive_ultrasonic(self, side, scan):
        if not scan.ranges or all(math.isnan(distance) for distance in scan.ranges):
            return
        self.ultrasonic[side] = (time.monotonic(), ultrasonic_points(side, scan))

    def receive_image(self, message):
        try:
            points = camera_points(message)
        except (ValueError, cv2.error):
            return
        if points is not None:
            self.vision = points
            self.last_image = time.monotonic()

    def filter_command(self):
        now = time.monotonic()
        command = self.command
        velocity = Twist()
        status = "Clear"
        if now - self.last_command > COMMAND_TIMEOUT:
            status = "Waiting for velocity commands"
        elif now - min(self.last_scan, self.last_image) > SENSOR_TIMEOUT:
            status = "Stopped: lidar or camera data is stale"
        elif any(now - stamp > SENSOR_TIMEOUT for stamp, _ in self.ultrasonic.values()):
            status = "Stopped: ultrasonic data is stale"
        elif command.linear.x or command.angular.z:
            observations = (
                ("lidar", self.lidar, ROVER_RADIUS),
                ("camera", self.vision, ROVER_RADIUS + .9),
                *((f"ultrasonic/{side}", points, .50)
                  for side, (_, points) in self.ultrasonic.items()),
            )
            for source, points, margin in observations:
                if obstacle_in_path(points, command, .45, margin):
                    status = f"Stopped: obstacle detected by {source}"
                    break
                if command.linear.x and obstacle_in_path(points, command, 1.5, margin):
                    status = f"Slowing: obstacle detected by {source}"
            else:
                scale = .35 if status.startswith("Slowing") else 1.0
                velocity.linear.x = command.linear.x * scale
                velocity.angular.z = command.angular.z * scale
            if status != "Clear" and self.recovery_enabled:
                velocity = Twist()
                if self.pose is not None and now - self.last_pose <= SENSOR_TIMEOUT and command.linear.x:
                    velocity, status = self.recovery_agent.act(now, self.pose, command, observations)
        if not status.startswith("Recovery agent:"):
            if self.status and self.status.startswith("Recovery agent:") and status == "Clear":
                self.get_logger().info("Recovery agent: clear path, returning control to patrol")
            self.recovery_agent.reset()
        self.output.publish(velocity)
        if status != self.status:
            self.get_logger().info(status)
            self.status = status


if __name__ == "__main__":
    cv2.setNumThreads(1)
    rclpy.init()
    node = ObstacleGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.recovery_agent.advisor is not None:
            node.recovery_agent.advisor.close()
        node.destroy_node()
        rclpy.try_shutdown()
