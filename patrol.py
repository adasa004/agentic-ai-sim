"""Drive both sides of each aisle using Gazebo's simulated rover pose."""
import json
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose

from make_world import (ACCESS_CURB_X, AISLE_Y, CONNECTOR_HALF_WIDTH, ENTRY_X,
                        IDLE_POINT, PARKED_CARS,
                        ROWS, TRAFFIC_COLORS, connector_x)
from permit_check import classify


TRAFFIC_PERIOD = 15.0
IDLE_SECONDS = 15 * 60
STARTUP_PAUSE = 3.0
SCAN_OFFSET = 2.2
ROVER_SPEED = 1.5
OUTER_BEND_SPEED = 1.25
OTHER_TURN_SPEED = 1.0
SCAN_SECONDS = 2.0
TRAFFIC_SPEED = 3.5
BEND_SPEED = 2.0
# Traffic runs between the median-side and perimeter-side rover tracks.
ROVER_CURB_INSET = 1.0
TRAFFIC_ROUTE = (
    (-33, -28), (-33, -13), (-32, -11), (-30, -10), (22, -10), (24, -9),
    (connector_x(-7, CONNECTOR_HALF_WIDTH), -7),
    (connector_x(7, CONNECTOR_HALF_WIDTH), 7),
    (31, 10), (-30, 10), (-32, 11), (-33, 13), (-33, 28),
)
OUTER_BEND = (
    (19.5, -12.2),
    (22.5, -11.8),
    (24.9, -10.8),
    (connector_x(-8, ROVER_CURB_INSET), -8),
    (connector_x(0, ROVER_CURB_INSET), 0),
    (connector_x(8, ROVER_CURB_INSET), 8),
    (connector_x(10.5, ROVER_CURB_INSET), 10.5),
    (37.1, 11.7),
    (35.7, 12.15),
    (32.5, 12.2),
)


# Round the median tip at x=23.6 without leaving the inner scan tracks.
INNER_BEND = ((20.5, -7.8), (20.5, -2.55), (22.7, -2.35),
              (24.0, -1.35), (24.35, 0), (24.0, 1.35),
              (22.7, 2.35), (21.5, 3.45), (21.3, 5.8), (20.5, 7.8))

# Cross along the lot edge, clear of the center of the access road.
CROSSOVER_X = ACCESS_CURB_X
INNER_ENTRY = (
    (ENTRY_X - .45, AISLE_Y + SCAN_OFFSET - .3),
    (CROSSOVER_X, AISLE_Y + SCAN_OFFSET - .7),
    (CROSSOVER_X, -AISLE_Y + SCAN_OFFSET + .7),
    (ENTRY_X - .45, -AISLE_Y + SCAN_OFFSET + .3),
)

OUTER_ENTRY = (
    (CROSSOVER_X, IDLE_POINT[1] - .55),
    (CROSSOVER_X, -AISLE_Y - SCAN_OFFSET + 1.0),
    (ENTRY_X - .45, -AISLE_Y - SCAN_OFFSET + .3),
)
IDLE_RETURN = (
    (ENTRY_X - .45, AISLE_Y - SCAN_OFFSET - .3),
    (CROSSOVER_X, AISLE_Y - SCAN_OFFSET - 1.0),
    (CROSSOVER_X, IDLE_POINT[1]),
)
ROLLING_TURNS = set(OUTER_BEND + INNER_BEND + INNER_ENTRY + OUTER_ENTRY + IDLE_RETURN)


def rounded_traffic_path(points):
    """Sample rounded corners while leaving the straight aisle centers intact."""
    path = [points[0]]
    for previous, corner, following in zip(points, points[1:], points[2:]):
        before, after = math.dist(previous, corner), math.dist(corner, following)
        trim = min(2.6, .45 * before, .45 * after)
        start = tuple(c + (p - c) * trim / before for p, c in zip(previous, corner))
        end = tuple(c + (n - c) * trim / after for n, c in zip(following, corner))
        for i in range(17):
            t = i / 16
            path.append(tuple((1 - t)**2 * a + 2 * (1 - t) * t * b + t*t*c
                              for a, b, c in zip(start, corner, end)))
    return (*path, points[-1])


TRAFFIC_PATH = rounded_traffic_path(TRAFFIC_ROUTE)
TRAFFIC_SEGMENTS = tuple(
    (start, end, math.dist(start, end))
    for start, end in zip(TRAFFIC_PATH, TRAFFIC_PATH[1:])
)


def traffic_pose(distance):
    for (x, y), (end_x, end_y), length in TRAFFIC_SEGMENTS:
        if distance < length:
            dx, dy = end_x - x, end_y - y
            return x + dx * distance / length, y + dy * distance / length, math.atan2(dy, dx)
        distance -= length
    (prev_x, prev_y), (x, y), _ = TRAFFIC_SEGMENTS[-1]
    return x, y, math.atan2(y - prev_y, x - prev_x)


def route():
    # Both loops enter the negative-Y aisle and leave the positive-Y
    # aisle, matching the one-way traffic route.
    for x, y in OUTER_ENTRY:
        yield x, y, None, None
    for row_index, direction in ((1, 1), (0, -1), (2, 1), (3, -1)):
        aisle, row, *_ = ROWS[row_index]
        inner_loop = row_index in (2, 3)
        far_x = 20.5 if inner_loop else (28 if aisle > 0 else 17.5)
        scan_y = aisle + math.copysign(SCAN_OFFSET, row - aisle)
        entry = ENTRY_X if direction > 0 else far_x
        exit_x = far_x if direction > 0 else ENTRY_X
        yield entry, scan_y, None, None
        cars = sorted((car for car in PARKED_CARS if car[2] == row),
                      key=lambda car: direction * car[1])
        for plate, x, y, yaw in cars:
            plate_x = x - 2.27 * math.cos(yaw)
            plate_y = y - 2.27 * math.sin(yaw)
            scan_x = max(ENTRY_X + .5, min(far_x - .35, plate_x - direction * .9))
            yield scan_x, scan_y, math.atan2(plate_y - scan_y, plate_x - scan_x), plate
        yield exit_x, scan_y, None, None
        if row_index == 2:
            for x, y in INNER_BEND[1:]:
                yield x, y, None, None
        elif row_index == 1:
            # Stay outside the motorcycle stalls, then round toward the curb.
            for x, y in OUTER_BEND:
                yield x, y, None, None
        elif row_index == 0:
            for x, y in INNER_ENTRY:
                yield x, y, None, None
        else:
            for x, y in IDLE_RETURN:
                yield x, y, None, None
    yield *IDLE_POINT, None, None


def car_clearance(x, y, pose):
    """Distance from a point to the car's oriented body rectangle."""
    car_x, car_y, yaw = pose
    dx, dy = x - car_x, y - car_y
    along = abs(dx * math.cos(yaw) + dy * math.sin(yaw))
    across = abs(-dx * math.sin(yaw) + dy * math.cos(yaw))
    return math.hypot(max(0, along - 2.25), max(0, across - .925))


class Patrol(Node):
    def __init__(self):
        super().__init__("lot10_patrol")
        self.commands = self.create_publisher(Twist, "/cmd_vel_requested", 10)
        self.create_subscription(PoseStamped, "/patrol/rover_pose", self.update_pose,
                                 qos_profile_sensor_data)
        self.goals = iter(route())
        self.goal = next(self.goals)
        self.aiming_at_plate = False
        self.scan_start = None
        self.idle_until = None
        self.startup_until = None
        self.rover = None
        self.traffic_service = self.create_client(SetEntityPose, "/world/lot10_prototype/set_pose")
        self.traffic = {}
        self.next_traffic = TRAFFIC_PERIOD
        self.drive_direction = None
        self.last_traffic_update = None
        self.last_pose = 0.0
        self.create_timer(0.25, self.stop_if_pose_missing)
        self.get_logger().info("Waiting for Gazebo. Patrol will start automatically.")

    def stop_if_pose_missing(self):
        if time.monotonic() - self.last_pose > 0.5:
            self.commands.publish(Twist())

    def update_pose(self, rover):
        self.last_pose = time.monotonic()
        p, q = rover.pose.position, rover.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        stamp = rover.header.stamp
        now = stamp.sec + stamp.nanosec * 1e-9
        self.rover = (p.x, p.y, yaw)
        if self.startup_until is None:
            self.startup_until = now + STARTUP_PAUSE
            self.get_logger().info("Rover parked beside the median. Starting in 3 seconds.")
        if now < self.startup_until:
            self.commands.publish(Twist())
            return
        if self.goal is None:
            if self.idle_until is None:
                self.idle_until = now + IDLE_SECONDS
                self.get_logger().info("Patrol complete. Idling at the curb for 15 minutes.")
            if now < self.idle_until:
                self.commands.publish(Twist())
                self.move_traffic(now)
                return
            self.goals = iter(route())
            self.goal = next(self.goals)
            self.idle_until = None
            self.get_logger().info("Starting next patrol.")
        x, y, heading, plate = self.goal
        self.move_traffic(now)
        # Roll through the bend instead of stopping to pivot at each point.
        while ((x, y) in ROLLING_TURNS and heading is None
               and math.hypot(x - p.x, y - p.y) < (.45 if (x, y) in OUTER_BEND else .22)):
            self.advance()
            x, y, heading, plate = self.goal
        distance = math.hypot(x - p.x, y - p.y)
        command = Twist()
        if heading is not None and distance <= 0.16:
            self.aiming_at_plate = True
        if self.scan_start is not None:
            elapsed = now - self.scan_start
            if elapsed >= SCAN_SECONDS:
                result = classify(json.dumps({"plate": plate, "confidence": 0.95}))
                if result["status"] == "flagged_candidate":
                    self.get_logger().warning(
                        f"PARKING VIOLATION FLAG: {plate} has no valid permit. Human review required."
                    )
                else:
                    self.get_logger().info(f"Plate {plate}: valid permit.")
                self.advance()
        elif not self.aiming_at_plate and distance > 0.16:
            turn_speed = None
            if heading is None and (x, y) in ROLLING_TURNS:
                turn_speed = OUTER_BEND_SPEED if (x, y) in OUTER_BEND else OTHER_TURN_SPEED
            command = self.drive_toward(p, yaw, x, y, turn_speed=turn_speed)
        elif self.aiming_at_plate:
            # Wheel slip during the pivot can move us beyond the waypoint
            # tolerance. Keep turning in place until aimed or interrupted.
            error = math.remainder(heading - yaw, math.tau)
            if abs(error) > 0.07:
                command.angular.z = max(-1.2, min(1.2, 2.5 * error))
            else:
                self.scan_start = now
                self.get_logger().info(f"Scanning plate {plate} for {SCAN_SECONDS:g} simulated seconds")
        else:
            self.advance()
        self.commands.publish(command)

    def drive_toward(self, position, yaw, x, y, turn_speed=None):
        dx, dy = x - position.x, y - position.y
        error = math.remainder(math.atan2(dy, dx) - yaw, math.tau)
        # Choose forward or reverse once for this move. Re-evaluating it
        # near the target can cause the rover to flip directions repeatedly.
        if self.drive_direction is None:
            self.drive_direction = -1 if abs(error) > math.pi / 2 else 1
        if self.drive_direction < 0:
            error = math.remainder(error + math.pi, math.tau)
        command = Twist()
        command.angular.z = max(-1.2, min(1.2, 2.5 * error))
        # Gradually trade speed for steering angle. Close to a scan pose
        # the distance limit still brings the rover to a controlled stop.
        angle = abs(error)
        distance = math.hypot(dx, dy)
        if turn_speed is not None and angle < .85:
            alignment = 1.0 - (angle / .85)**2
            command.linear.x = self.drive_direction * min(turn_speed, 2.2 * distance) * alignment
        elif turn_speed is None and angle < .65:
            alignment = 1.0 - (angle / .65)**2
            command.linear.x = self.drive_direction * min(ROVER_SPEED, distance) * alignment
        return command

    def move_traffic(self, now):
        elapsed = 0.0 if self.last_traffic_update is None else now - self.last_traffic_update
        self.last_traffic_update = now
        if (now >= self.next_traffic
                and self.traffic_service.service_is_ready()
                and all(car["distance"] >= 7.0 for car in self.traffic.values())):
            slot = next((i for i in range(len(TRAFFIC_COLORS)) if i not in self.traffic), None)
            if slot is not None:
                self.traffic[slot] = {"distance": 0.0, "pose": None, "request": None}
                self.get_logger().info(f"Traffic car {slot + 1} entering from the access road.")
                self.next_traffic = now + TRAFFIC_PERIOD

        leader = math.inf
        for slot in sorted(self.traffic, key=lambda slot: self.traffic[slot]["distance"],
                           reverse=True):
            car = self.traffic[slot]
            future = car["request"]
            if future is not None:
                if not future.done():
                    leader = car["distance"]
                    continue
                car["request"] = None
                if future.result().success:
                    car["pose"] = car["requested_pose"]
                    if car["pose"] is None:
                        del self.traffic[slot]
                        continue
            _, _, heading = traffic_pose(car["distance"])
            speed = TRAFFIC_SPEED if abs(math.sin(heading)) < .1 else BEND_SPEED
            distance = car["distance"]
            distance += max(0.0, min(speed * min(elapsed, .15),
                                     leader - 7.0 - distance))
            # Guard every pose update, including at curves, crossings and
            # when lidar has yet to see the car. Traffic waits for the rover.
            sweep = (traffic_pose(car["distance"] + (distance - car["distance"]) * i / 4)
                     for i in range(5))
            if any(car_clearance(*self.rover[:2], pose) < .95 for pose in sweep):
                leader = car["distance"]
                if not car.get("waiting"):
                    self.get_logger().info(f"Traffic car {slot + 1} waiting for rover clearance.")
                    car["waiting"] = True
                continue
            car["waiting"] = False
            car["distance"] = distance
            leader = distance
            x, y, heading = traffic_pose(distance)
            hiding = (x, y) == TRAFFIC_ROUTE[-1]
            request = SetEntityPose.Request()
            request.entity.name = f"passing_car_{slot}"
            request.entity.type = Entity.MODEL
            request.pose.position.x = float(x)
            request.pose.position.y = float(y)
            request.pose.position.z = float(-5 - slot * 2) if hiding else 0.0
            request.pose.orientation.w = math.cos(heading / 2)
            request.pose.orientation.z = math.sin(heading / 2)
            car["requested_pose"] = None if hiding else (x, y, heading)
            car["request"] = self.traffic_service.call_async(request)

    def advance(self):
        self.aiming_at_plate = False
        self.drive_direction = None
        self.scan_start = None
        self.goal = next(self.goals, None)


def main():
    rclpy.init()
    patrol = Patrol()
    try:
        rclpy.spin(patrol)
    except KeyboardInterrupt:
        pass
    finally:
        patrol.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
