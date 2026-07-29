#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node

from motion_msgs.msg import ActionRequest


class OfficialVisionWakeController(Node):
    """Wake and suspend the upstream CyberDog vision stack."""

    def __init__(self):
        super().__init__("official_vision_wake_controller")

        self.declare_parameter("robot_namespace", "/mi1035085")
        self.declare_parameter("wake_on_start", True)
        self.declare_parameter("sleep_on_shutdown", True)
        self.declare_parameter("wake_delay_sec", 1.0)
        self.declare_parameter("warmup_wait_sec", 5.0)
        self.declare_parameter("command_timeout_sec", 30)
        self.declare_parameter("request_id_start", 8800)
        self.declare_parameter("track_control_mode", 15)
        self.declare_parameter("track_mode_type", 1)
        self.declare_parameter("manual_control_mode", 3)
        self.declare_parameter("manual_mode_type", 0)
        self.declare_parameter("shutdown_flush_sec", 1.0)

        self.robot_namespace = self._normalize_namespace(
            self.get_parameter("robot_namespace").value
        )
        self.wake_on_start = bool(self.get_parameter("wake_on_start").value)
        self.sleep_on_shutdown = bool(self.get_parameter("sleep_on_shutdown").value)
        self.wake_delay_sec = max(0.0, float(self.get_parameter("wake_delay_sec").value))
        self.warmup_wait_sec = max(0.0, float(self.get_parameter("warmup_wait_sec").value))
        self.command_timeout_sec = int(self.get_parameter("command_timeout_sec").value)
        self.request_id = int(self.get_parameter("request_id_start").value)
        self.track_control_mode = int(self.get_parameter("track_control_mode").value)
        self.track_mode_type = int(self.get_parameter("track_mode_type").value)
        self.manual_control_mode = int(self.get_parameter("manual_control_mode").value)
        self.manual_mode_type = int(self.get_parameter("manual_mode_type").value)
        self.shutdown_flush_sec = max(
            0.0, float(self.get_parameter("shutdown_flush_sec").value)
        )

        self.action_pub = self.create_publisher(
            ActionRequest, self._topic_name("cyberdog_action"), 10
        )

        self._wake_timer = None
        self._warmup_timer = None
        self._wake_sent = False
        self._sleep_sent = False

        self.get_logger().info(
            "Official vision controller ready on %s"
            % self._topic_name("cyberdog_action")
        )

        if self.wake_on_start:
            self._wake_timer = self.create_timer(
                self.wake_delay_sec, self._handle_wake_timer
            )
            self.get_logger().info(
                "Wake command scheduled in %.1f seconds" % self.wake_delay_sec
            )
        else:
            self.get_logger().info(
                "wake_on_start is disabled; waiting for manual shutdown only"
            )

    def _normalize_namespace(self, value):
        text = str(value).strip()
        if not text:
            return ""
        return text if text.startswith("/") else f"/{text}"

    def _topic_name(self, suffix):
        if not self.robot_namespace:
            return f"/{suffix}"
        return f"{self.robot_namespace}/{suffix}"

    def _next_request_id(self):
        request_id = self.request_id
        self.request_id += 1
        return request_id

    def _publish_mode_change(self, control_mode, mode_type, label):
        request = ActionRequest()
        request.type = 1
        request.request_id = self._next_request_id()
        request.mode.control_mode = control_mode
        request.mode.mode_type = mode_type
        request.timeout = self.command_timeout_sec
        self.action_pub.publish(request)
        self.get_logger().info(
            "Published %s mode change: control_mode=%d mode_type=%d request_id=%d"
            % (label, control_mode, mode_type, request.request_id)
        )

    def _handle_wake_timer(self):
        if self._wake_timer is not None:
            self._wake_timer.cancel()
            self._wake_timer = None

        self.force_wake_vision()

        if self.warmup_wait_sec > 0.0:
            self.get_logger().info(
                "Waiting %.1f seconds for the official vision stack to warm up"
                % self.warmup_wait_sec
            )
            self._warmup_timer = self.create_timer(
                self.warmup_wait_sec, self._handle_warmup_complete
            )

    def _handle_warmup_complete(self):
        if self._warmup_timer is not None:
            self._warmup_timer.cancel()
            self._warmup_timer = None
        self.get_logger().info(
            "Official vision stack should now be ready for bridge consumers"
        )

    def force_wake_vision(self):
        if self._wake_sent:
            self.get_logger().warn("Wake request already sent; ignoring duplicate call")
            return

        self._publish_mode_change(
            self.track_control_mode, self.track_mode_type, "wake"
        )
        self._wake_sent = True
        self._sleep_sent = False

    def sleep_vision_on_exit(self):
        if not self.sleep_on_shutdown:
            self.get_logger().info("sleep_on_shutdown is disabled; skipping suspend")
            return

        if self._sleep_sent:
            return

        self._publish_mode_change(
            self.manual_control_mode, self.manual_mode_type, "suspend"
        )
        self._sleep_sent = True
        self._wake_sent = False

        end_time = time.monotonic() + self.shutdown_flush_sec
        while time.monotonic() < end_time and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)


def main(args=None):
    rclpy.init(args=args)
    node = OfficialVisionWakeController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.sleep_vision_on_exit()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
