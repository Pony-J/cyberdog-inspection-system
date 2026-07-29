#!/usr/bin/env python3
"""
Bridge: /cmd_vel (geometry_msgs/Twist) → /<cyberdog_ns>/body_cmd (motion_msgs/SE3VelocityCMD)

Translates standard Nav2 velocity commands into CyberDog's proprietary motion format.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from motion_msgs.msg import SE3VelocityCMD, SE3Velocity, Frameid


class CmdVelBridge(Node):
    def __init__(self):
        super().__init__('cmd_vel_bridge')
        self.declare_parameter('cyberdog_ns', 'mi1035085')
        ns = self.get_parameter('cyberdog_ns').get_parameter_value().string_value

        self.pub = self.create_publisher(
            SE3VelocityCMD, f'/{ns}/body_cmd', 10
        )
        self.sub = self.create_subscription(
            Twist, '/cmd_vel', self._on_cmd_vel, 10
        )
        self.get_logger().info(f'Bridging /cmd_vel -> /{ns}/body_cmd')

    def _on_cmd_vel(self, msg: Twist):
        cmd = SE3VelocityCMD()
        cmd.sourceid = SE3VelocityCMD.REMOTEC
        cmd.velocity.timestamp = self.get_clock().now().to_msg()
        cmd.velocity.frameid.id = Frameid.BODY_FRAME
        cmd.velocity.linear_x = float(msg.linear.x)
        cmd.velocity.linear_y = float(msg.linear.y)
        cmd.velocity.linear_z = 0.0
        cmd.velocity.angular_x = 0.0
        cmd.velocity.angular_y = 0.0
        cmd.velocity.angular_z = float(msg.angular.z)
        self.pub.publish(cmd)


def main():
    rclpy.init()
    node = CmdVelBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
