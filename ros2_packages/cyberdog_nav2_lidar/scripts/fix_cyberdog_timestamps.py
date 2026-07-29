#!/usr/bin/env python3
"""
Normalize CyberDog sensor data on Jetson.

Subscribes to the robot's raw odometry/IMU topics, republishes them with Jetson
wall-clock timestamps and isolated frame IDs for Nav2/Cartographer:

  odom_fixed -> base_footprint_fixed -> base_link_fixed -> laser_frame_fixed
                                                        -> imu_fixed

It also republishes /scan as /scan_fixed with the fixed laser frame id so the
local sensor pipeline stays on a self-consistent TF tree.
"""

import copy

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan
from tf2_ros import TransformBroadcaster


class TimestampFixer(Node):
    def __init__(self):
        super().__init__("cyberdog_timestamp_fixer")

        self.declare_parameter("cyberdog_ns", "mi1035085")
        self.declare_parameter("input_odom_topic", "")
        self.declare_parameter("output_odom_topic", "")
        self.declare_parameter("input_scan_topic", "/scan")
        self.declare_parameter("output_scan_topic", "/scan_fixed")
        self.declare_parameter("odom_frame", "odom_fixed")
        self.declare_parameter("base_frame", "base_footprint_fixed")
        self.declare_parameter("laser_frame", "laser_frame_fixed")
        self.declare_parameter("input_imu_topic", "")
        self.declare_parameter("output_imu_topic", "/imu/data_fixed")
        self.declare_parameter("imu_frame", "imu_fixed")

        ns = self.get_parameter("cyberdog_ns").get_parameter_value().string_value
        input_odom_topic = self.get_parameter("input_odom_topic").get_parameter_value().string_value
        output_odom_topic = self.get_parameter("output_odom_topic").get_parameter_value().string_value
        input_scan_topic = self.get_parameter("input_scan_topic").get_parameter_value().string_value
        output_scan_topic = self.get_parameter("output_scan_topic").get_parameter_value().string_value
        input_imu_topic = self.get_parameter("input_imu_topic").get_parameter_value().string_value
        output_imu_topic = self.get_parameter("output_imu_topic").get_parameter_value().string_value

        self.odom_frame = self.get_parameter("odom_frame").get_parameter_value().string_value
        self.base_frame = self.get_parameter("base_frame").get_parameter_value().string_value
        self.laser_frame = self.get_parameter("laser_frame").get_parameter_value().string_value
        self.imu_frame = self.get_parameter("imu_frame").get_parameter_value().string_value

        self.input_odom_topic = input_odom_topic or f"/{ns}/odom_out"
        self.output_odom_topic = output_odom_topic or f"/{ns}/odom_out_fixed"
        self.input_imu_topic = input_imu_topic or f"/{ns}/camera/imu"

        imu_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
        )

        self.odom_pub = self.create_publisher(Odometry, self.output_odom_topic, 20)
        self.scan_pub = self.create_publisher(LaserScan, output_scan_topic, qos_profile_sensor_data)
        self.imu_pub = self.create_publisher(Imu, output_imu_topic, imu_qos)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(Odometry, self.input_odom_topic, self._on_odom, 20)
        self.create_subscription(LaserScan, input_scan_topic, self._on_scan, qos_profile_sensor_data)
        self.create_subscription(Imu, self.input_imu_topic, self._on_imu, imu_qos)

        self.get_logger().info(
            f"Fixing odom: {self.input_odom_topic} -> {self.output_odom_topic}, "
            f"scan: {input_scan_topic} -> {output_scan_topic}, "
            f"imu: {self.input_imu_topic} -> {output_imu_topic}"
        )

    # Quadruped odometry covariance — CyberDog firmware reports all-zero,
    # which makes Cartographer over-trust odom.  Inject realistic values
    # based on CHAMP quadruped benchmarks and legged-robot literature.
    _POSE_COV = [0.0] * 36
    _POSE_COV[0]  = 0.05   # x        (std ≈ 0.22 m)
    _POSE_COV[7]  = 0.05   # y        (std ≈ 0.22 m)
    _POSE_COV[14] = 1e6    # z        (unmeasured)
    _POSE_COV[21] = 1e6    # roll     (unmeasured)
    _POSE_COV[28] = 1e6    # pitch    (unmeasured)
    _POSE_COV[35] = 0.04   # yaw      (std ≈ 0.2 rad ≈ 11.5°)

    _TWIST_COV = [0.0] * 36
    _TWIST_COV[0]  = 0.01  # vx       (std ≈ 0.1 m/s)
    _TWIST_COV[7]  = 0.04  # vy       (std ≈ 0.2 m/s, lateral worse)
    _TWIST_COV[14] = 1e6   # vz       (unmeasured)
    _TWIST_COV[21] = 1e6   # wx       (unmeasured)
    _TWIST_COV[28] = 1e6   # wy       (unmeasured)
    _TWIST_COV[35] = 0.09  # wz       (std ≈ 0.3 rad/s, heading worst)

    def _on_odom(self, msg: Odometry) -> None:
        now = self.get_clock().now().to_msg()

        fixed_msg = copy.deepcopy(msg)
        fixed_msg.header.stamp = now
        fixed_msg.header.frame_id = self.odom_frame
        fixed_msg.child_frame_id = self.base_frame
        fixed_msg.pose.covariance = list(self._POSE_COV)
        fixed_msg.twist.covariance = list(self._TWIST_COV)
        self.odom_pub.publish(fixed_msg)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = now
        tf_msg.header.frame_id = self.odom_frame
        tf_msg.child_frame_id = self.base_frame
        tf_msg.transform.translation.x = msg.pose.pose.position.x
        tf_msg.transform.translation.y = msg.pose.pose.position.y
        tf_msg.transform.translation.z = msg.pose.pose.position.z
        tf_msg.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(tf_msg)

    def _on_scan(self, msg: LaserScan) -> None:
        fixed_scan = copy.deepcopy(msg)
        fixed_scan.header.frame_id = self.laser_frame
        self.scan_pub.publish(fixed_scan)

    def _on_imu(self, msg: Imu) -> None:
        fixed_imu = copy.deepcopy(msg)
        fixed_imu.header.frame_id = self.imu_frame

        # Camera IMU uses optical frame convention:
        #   optical X = right, Y = down, Z = forward
        # ROS body frame (imu_fixed, aligned with base_link_fixed):
        #   body X = forward, Y = left, Z = up
        # Transform: body_x = opt_z, body_y = -opt_x, body_z = -opt_y
        ax = msg.angular_velocity
        fixed_imu.angular_velocity.x = ax.z
        fixed_imu.angular_velocity.y = -ax.x
        fixed_imu.angular_velocity.z = -ax.y

        la = msg.linear_acceleration
        fixed_imu.linear_acceleration.x = la.z
        fixed_imu.linear_acceleration.y = -la.x
        fixed_imu.linear_acceleration.z = -la.y

        self.imu_pub.publish(fixed_imu)


def main() -> None:
    rclpy.init()
    node = TimestampFixer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
