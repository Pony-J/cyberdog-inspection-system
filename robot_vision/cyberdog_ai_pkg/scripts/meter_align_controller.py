#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node

from cyberdog_ai_pkg.msg import MeterInfo
from motion_msgs.msg import ActionRequest, Frameid, SE3VelocityCMD


def clamp(value, low, high):
    return max(low, min(high, value))


def ramp(current, target, step):
    if target > current:
        return min(target, current + step)
    if target < current:
        return max(target, current - step)
    return target


class MeterAlignController(Node):
    def __init__(self):
        super().__init__("meter_align_controller")

        self.declare_parameter("robot_namespace", "/mi1035085")
        self.declare_parameter("meter_topic", "/mi1035085/meter")
        self.declare_parameter("image_width", 1920.0)
        self.declare_parameter("image_height", 1080.0)
        self.declare_parameter("target_center_tolerance_x", 0.08)
        self.declare_parameter("target_min_z", 0.35)
        self.declare_parameter("target_max_z", 0.55)
        self.declare_parameter("target_min_width_ratio", 0.18)
        self.declare_parameter("target_max_width_ratio", 0.40)
        self.declare_parameter("search_yaw_rate", 0.20)
        self.declare_parameter("max_linear_speed", 0.16)
        self.declare_parameter("max_angular_speed", 0.45)
        self.declare_parameter("yaw_kp", 0.90)
        self.declare_parameter("forward_kp", 0.60)
        self.declare_parameter("control_period_sec", 0.10)
        self.declare_parameter("detection_timeout_sec", 0.8)
        self.declare_parameter("stable_cycles_required", 8)
        self.declare_parameter("linear_accel_step", 0.03)
        self.declare_parameter("angular_accel_step", 0.08)
        self.declare_parameter("auto_prepare_robot", False)
        self.declare_parameter("request_id_start", 20000)

        self.robot_namespace = self._normalize_namespace(
            self.get_parameter("robot_namespace").value
        )
        self.meter_topic = str(self.get_parameter("meter_topic").value)
        self.image_width = float(self.get_parameter("image_width").value)
        self.image_height = float(self.get_parameter("image_height").value)
        self.target_center_tolerance_x = float(
            self.get_parameter("target_center_tolerance_x").value
        )
        self.target_min_z = float(self.get_parameter("target_min_z").value)
        self.target_max_z = float(self.get_parameter("target_max_z").value)
        self.target_min_width_ratio = float(
            self.get_parameter("target_min_width_ratio").value
        )
        self.target_max_width_ratio = float(
            self.get_parameter("target_max_width_ratio").value
        )
        self.search_yaw_rate = float(self.get_parameter("search_yaw_rate").value)
        self.max_linear_speed = float(self.get_parameter("max_linear_speed").value)
        self.max_angular_speed = float(self.get_parameter("max_angular_speed").value)
        self.yaw_kp = float(self.get_parameter("yaw_kp").value)
        self.forward_kp = float(self.get_parameter("forward_kp").value)
        self.control_period_sec = float(self.get_parameter("control_period_sec").value)
        self.detection_timeout_sec = float(
            self.get_parameter("detection_timeout_sec").value
        )
        self.stable_cycles_required = int(
            self.get_parameter("stable_cycles_required").value
        )
        self.linear_accel_step = float(self.get_parameter("linear_accel_step").value)
        self.angular_accel_step = float(self.get_parameter("angular_accel_step").value)
        self.auto_prepare_robot = bool(self.get_parameter("auto_prepare_robot").value)
        self.request_id = int(self.get_parameter("request_id_start").value)

        self.vel_pub = self.create_publisher(
            SE3VelocityCMD, self._topic_name("body_cmd"), 10
        )
        self.action_pub = self.create_publisher(
            ActionRequest, self._topic_name("cyberdog_action"), 10
        )
        self.meter_sub = self.create_subscription(
            MeterInfo, self.meter_topic, self._on_meter, 10
        )

        self.current_lx = 0.0
        self.current_az = 0.0
        self.target_lx = 0.0
        self.target_az = 0.0
        self.publish_zero_count = 0

        self.latest_meter = None
        self.last_meter_time = None
        self.stable_cycles = 0
        self.state = "SEARCH"

        if self.auto_prepare_robot:
            self._prepare_robot()

        self.control_timer = self.create_timer(
            self.control_period_sec, self._control_loop
        )
        self.get_logger().info(
            "meter_align_controller ready, meter_topic=%s, cmd_topic=%s/body_cmd"
            % (self.meter_topic, self.robot_namespace)
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
        value = self.request_id
        self.request_id += 1
        return value

    def _send_action(self, action_type, control_mode=0, mode_type=0, gait_id=0, order_id=0):
        req = ActionRequest()
        req.type = action_type
        req.request_id = self._next_request_id()
        req.mode.control_mode = control_mode
        req.mode.mode_type = mode_type
        req.gait.gait = gait_id
        req.order.id = order_id
        req.order.para = 0.0
        req.timeout = 30
        self.action_pub.publish(req)

    def _prepare_robot(self):
        self.get_logger().info("preparing robot: switch manual -> stand -> trot")
        self._send_action(action_type=1, control_mode=3)
        time.sleep(2.0)
        self._send_action(action_type=3, order_id=9)
        time.sleep(3.0)
        self._send_action(action_type=2, gait_id=4)
        time.sleep(1.0)

    def _select_target(self, msg):
        if msg.count == 0 or not msg.infos:
            return None

        def score(meter):
            roi = meter.roi
            area = float(roi.width * roi.height)
            center_x = roi.x_offset + roi.width / 2.0
            center_error = abs(center_x - self.image_width / 2.0) / max(self.image_width, 1.0)
            return area - center_error * 10000.0

        return max(msg.infos, key=score)

    def _on_meter(self, msg):
        target = self._select_target(msg)
        if target is None:
            return
        self.latest_meter = target
        self.last_meter_time = time.monotonic()

    def _has_recent_meter(self):
        if self.latest_meter is None or self.last_meter_time is None:
            return False
        return (time.monotonic() - self.last_meter_time) <= self.detection_timeout_sec

    def _publish_velocity(self):
        self.current_lx = ramp(self.current_lx, self.target_lx, self.linear_accel_step)
        self.current_az = ramp(self.current_az, self.target_az, self.angular_accel_step)

        if (
            abs(self.current_lx) < 0.01
            and abs(self.current_az) < 0.01
            and self.target_lx == 0.0
            and self.target_az == 0.0
        ):
            if self.publish_zero_count < 5:
                self.publish_zero_count += 1
                self.current_lx = 0.0
                self.current_az = 0.0
            else:
                return
        else:
            self.publish_zero_count = 0

        cmd = SE3VelocityCMD()
        cmd.sourceid = SE3VelocityCMD.REMOTEC
        cmd.velocity.frameid.id = Frameid.BODY_FRAME
        cmd.velocity.timestamp = self.get_clock().now().to_msg()
        cmd.velocity.linear_x = float(self.current_lx)
        cmd.velocity.linear_y = 0.0
        cmd.velocity.linear_z = 0.0
        cmd.velocity.angular_x = 0.0
        cmd.velocity.angular_y = 0.0
        cmd.velocity.angular_z = float(self.current_az)
        self.vel_pub.publish(cmd)

    def _set_stop(self):
        self.target_lx = 0.0
        self.target_az = 0.0

    def _control_loop(self):
        if not self._has_recent_meter():
            if self.state != "SEARCH":
                self.get_logger().info("state -> SEARCH (meter lost)")
            self.state = "SEARCH"
            self.stable_cycles = 0
            self.target_lx = 0.0
            self.target_az = self.search_yaw_rate
            self._publish_velocity()
            return

        meter = self.latest_meter
        roi = meter.roi
        meter_cx = roi.x_offset + roi.width / 2.0
        center_error_x = (meter_cx - self.image_width / 2.0) / max(self.image_width, 1.0)
        width_ratio = roi.width / max(self.image_width, 1.0)

        need_turn = abs(center_error_x) > self.target_center_tolerance_x
        valid_depth = bool(meter.valid_depth)
        too_far = (valid_depth and meter.z > self.target_max_z) or (
            not valid_depth and width_ratio < self.target_min_width_ratio
        )
        too_close = (valid_depth and meter.z < self.target_min_z) or (
            not valid_depth and width_ratio > self.target_max_width_ratio
        )

        if need_turn:
            if self.state != "ALIGN":
                self.get_logger().info(
                    "state -> ALIGN (center_error_x=%.3f, width_ratio=%.3f)"
                    % (center_error_x, width_ratio)
                )
            self.state = "ALIGN"
            self.stable_cycles = 0
            self.target_lx = 0.0
            self.target_az = clamp(
                -self.yaw_kp * center_error_x,
                -self.max_angular_speed,
                self.max_angular_speed,
            )
        elif too_far:
            if self.state != "APPROACH":
                if valid_depth:
                    self.get_logger().info(
                        "state -> APPROACH (z=%.2f, width_ratio=%.3f)"
                        % (meter.z, width_ratio)
                    )
                else:
                    self.get_logger().info(
                        "state -> APPROACH (no depth, width_ratio=%.3f)" % width_ratio
                    )
            self.state = "APPROACH"
            self.stable_cycles = 0
            if valid_depth:
                forward_error = meter.z - self.target_max_z
                self.target_lx = clamp(
                    self.forward_kp * forward_error, 0.0, self.max_linear_speed
                )
            else:
                ratio_error = self.target_min_width_ratio - width_ratio
                self.target_lx = clamp(
                    self.forward_kp * ratio_error, 0.05, self.max_linear_speed
                )
            self.target_az = 0.0
        elif too_close:
            if self.state != "BACKUP":
                self.get_logger().info(
                    "state -> BACKUP (z=%.2f, width_ratio=%.3f)"
                    % (meter.z if valid_depth else -1.0, width_ratio)
                )
            self.state = "BACKUP"
            self.stable_cycles = 0
            if valid_depth:
                back_error = self.target_min_z - meter.z
                self.target_lx = -clamp(
                    self.forward_kp * back_error, 0.0, self.max_linear_speed * 0.7
                )
            else:
                self.target_lx = -0.05
            self.target_az = 0.0
        else:
            self.stable_cycles += 1
            self._set_stop()
            if self.stable_cycles >= self.stable_cycles_required:
                if self.state != "READY":
                    self.get_logger().info(
                        "state -> READY (z=%s, width_ratio=%.3f, stable_cycles=%d)"
                        % (
                            ("%.2f" % meter.z) if valid_depth else "N/A",
                            width_ratio,
                            self.stable_cycles,
                        )
                    )
                self.state = "READY"
            else:
                if self.state != "HOLD":
                    self.get_logger().info(
                        "state -> HOLD (waiting stable frames, current=%d/%d)"
                        % (self.stable_cycles, self.stable_cycles_required)
                    )
                self.state = "HOLD"

        self._publish_velocity()

    def destroy_node(self):
        self._set_stop()
        self.publish_zero_count = 0
        for _ in range(5):
            self._publish_velocity()
            time.sleep(0.05)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MeterAlignController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
