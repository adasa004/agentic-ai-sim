"""Estimate planar rover pose from wheel rotation and gyro yaw rate."""
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState

from make_world import IDLE_POINT, IDLE_YAW, WHEEL_RADIUS

MAX_SENSOR_GAP = .25
PUBLISH_PERIOD = .04
WHEEL_JOINTS = ("front_left_joint", "rear_left_joint",
                "front_right_joint", "rear_right_joint")


def timestamp(message):
    stamp = message.header.stamp
    return stamp.sec + stamp.nanosec * 1e-9


class DeadReckoning:
    def __init__(self):
        self.x, self.y = IDLE_POINT
        self.yaw = IDLE_YAW
        self.previous = None
        self.previous_rate = None

    def update(self, now, wheels, yaw_rate):
        """Use all four encoder deltas for travel and gyro Z for heading."""
        previous, rate = self.previous, self.previous_rate
        self.previous = (now, wheels)
        self.previous_rate = yaw_rate
        if previous is None or not 0 < now - previous[0] <= MAX_SENSOR_GAP:
            return False
        travel = WHEEL_RADIUS * sum(new - old for new, old in zip(wheels, previous[1])) / 4
        turn = (rate + yaw_rate) * .5 * (now - previous[0])
        midpoint = self.yaw + turn / 2
        self.x += travel * math.cos(midpoint)
        self.y += travel * math.sin(midpoint)
        self.yaw = math.remainder(self.yaw + turn, math.tau)
        return True


class EncoderGyroPose(Node):
    def __init__(self):
        super().__init__("lot10_encoder_gyro_pose")
        self.estimate = DeadReckoning()
        self.gyro = None
        self.last_publish = -math.inf
        self.pose = self.create_publisher(PoseStamped, "/patrol/rover_pose", 10)
        self.create_subscription(Imu, "/imu/data", self.receive_imu, qos_profile_sensor_data)
        self.create_subscription(JointState, "/wheel_encoders", self.receive_wheels,
                                 qos_profile_sensor_data)

    def receive_imu(self, message):
        rate = message.angular_velocity.z
        self.gyro = (timestamp(message), rate) if math.isfinite(rate) else None

    def receive_wheels(self, message):
        now = timestamp(message)
        if self.gyro is None or abs(now - self.gyro[0]) > MAX_SENSOR_GAP:
            self.estimate.previous = None
            return
        positions = {name.rsplit("::", 1)[-1]: position
                     for name, position in zip(message.name, message.position)}
        if not all(name in positions and math.isfinite(positions[name]) for name in WHEEL_JOINTS):
            self.estimate.previous = None
            return
        wheels = tuple(positions[name] for name in WHEEL_JOINTS)
        if not self.estimate.update(now, wheels, self.gyro[1]):
            return
        if now - self.last_publish < PUBLISH_PERIOD:
            return
        pose = PoseStamped()
        pose.header.stamp = message.header.stamp
        pose.header.frame_id = "lot"
        pose.pose.position.x = self.estimate.x
        pose.pose.position.y = self.estimate.y
        pose.pose.orientation.z = math.sin(self.estimate.yaw / 2)
        pose.pose.orientation.w = math.cos(self.estimate.yaw / 2)
        self.pose.publish(pose)
        self.last_publish = now


if __name__ == "__main__":
    rclpy.init()
    node = EncoderGyroPose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
