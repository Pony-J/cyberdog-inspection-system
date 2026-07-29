#!/usr/bin/env python3
"""单点表盘读数执行器 — 发导航目标 → 等到达 → 检测 → 读数 → 输出结果

前置条件：
  - Nav2 已通过前端启动（mode:=nav）
  - 狗已在 navigation 模式
  - meter_reading_node 运行中
  - camera topic 正常发布

用法：
  ros2 run cyberdog_ai_pkg meter_inspect_task --ros-args -p point_id:=meter_01
"""
import json
import math
import os
import statistics
import time
from collections import deque
from datetime import datetime
from typing import Any, Deque, Dict, List, Optional, Tuple

import cv2
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import Image
from nav2_msgs.action import NavigateToPose

from cyberdog_ai_pkg.msg import MeterInfo

# ── 失败码 ──
FAIL_NAV_FAILED = "NAV_FAILED"
FAIL_DETECT_TIMEOUT = "DETECT_TIMEOUT"
FAIL_READ_TIMEOUT = "READ_TIMEOUT"
FAIL_DETECT_UNSTABLE = "DETECT_UNSTABLE"
FAIL_READ_UNSTABLE = "READ_UNSTABLE"
FAIL_NO_VALID_VALUE = "NO_VALID_VALUE"
FAIL_VALUE_OUT_OF_RANGE = "VALUE_OUT_OF_RANGE"


class MeterInspectTask(Node):
    def __init__(self):
        super().__init__("meter_inspect_task")

        # ── 参数 ──
        self.declare_parameter("point_id", "")
        self.declare_parameter("name", "")
        self.declare_parameter("meter_type", "pressure")
        self.declare_parameter("meter_topic", "/mi1035085/meter")
        self.declare_parameter("image_topic", "/mi1035085/camera/color/image_raw")
        self.declare_parameter("output_dir", os.path.expanduser("~/cyberdog_ws/alarm_logs"))
        self.declare_parameter("timeout_sec", 45.0)
        self.declare_parameter("settle_sec", 2.0)
        self.declare_parameter("detect_confirm_frames", 5)
        self.declare_parameter("detect_pass_ratio", 0.7)
        self.declare_parameter("read_frames", 7)
        self.declare_parameter("max_retry", 2)
        self.declare_parameter("alarm_low", -1.0)
        self.declare_parameter("alarm_high", -1.0)
        self.declare_parameter("image_width", 1920.0)
        self.declare_parameter("image_height", 1080.0)
        self.declare_parameter("max_center_jitter_ratio", 0.05)
        self.declare_parameter("max_value_range", 0.30)
        # 导航相关
        self.declare_parameter("points_config", os.path.expanduser(
            "~/cyberdog_ws/src/cyberdog_ai_pkg/config/meter_points.json"))
        self.declare_parameter("nav_timeout_sec", 90.0)
        self.declare_parameter("nav_goal_x", -1000.0)
        self.declare_parameter("nav_goal_y", -1000.0)
        self.declare_parameter("nav_goal_yaw", -1000.0)

        # ── 读参数 ──
        self.point_id = str(self.get_parameter("point_id").value)
        self.name = str(self.get_parameter("name").value or self.point_id)
        self.meter_type = str(self.get_parameter("meter_type").value)
        self.timeout_sec = float(self.get_parameter("timeout_sec").value)
        self.settle_sec = float(self.get_parameter("settle_sec").value)
        self.detect_confirm_frames = max(1, int(self.get_parameter("detect_confirm_frames").value))
        self.detect_pass_ratio = float(self.get_parameter("detect_pass_ratio").value)
        self.read_frames = max(1, int(self.get_parameter("read_frames").value))
        self.max_retry = max(0, int(self.get_parameter("max_retry").value))
        self.image_width = float(self.get_parameter("image_width").value)
        self.image_height = float(self.get_parameter("image_height").value)
        self.max_center_jitter_ratio = float(self.get_parameter("max_center_jitter_ratio").value)
        self.max_value_range = float(self.get_parameter("max_value_range").value)
        self.output_dir = str(self.get_parameter("output_dir").value)
        self.nav_timeout_sec = float(self.get_parameter("nav_timeout_sec").value)

        alarm_low = float(self.get_parameter("alarm_low").value)
        alarm_high = float(self.get_parameter("alarm_high").value)
        self.alarm_low = None if alarm_low < 0 else alarm_low
        self.alarm_high = None if alarm_high < 0 else alarm_high

        os.makedirs(self.output_dir, exist_ok=True)

        # ── 从配置文件加载点位参数 ──
        self._load_point_config()

        # 参数覆盖配置文件
        nav_x = float(self.get_parameter("nav_goal_x").value)
        nav_y = float(self.get_parameter("nav_goal_y").value)
        nav_yaw = float(self.get_parameter("nav_goal_yaw").value)
        if nav_x > -999:
            self.nav_goal_x = nav_x
        if nav_y > -999:
            self.nav_goal_y = nav_y
        if nav_yaw > -999:
            self.nav_goal_yaw = nav_yaw

        # ── 状态 ──
        self.bridge = CvBridge()
        self.latest_meter_msg: Optional[MeterInfo] = None
        self.latest_image = None
        self.latest_meter_time = 0.0

        # ── 订阅 ──
        meter_topic = str(self.get_parameter("meter_topic").value)
        image_topic = str(self.get_parameter("image_topic").value)
        self.create_subscription(MeterInfo, meter_topic, self._on_meter, 10)
        self.create_subscription(Image, image_topic, self._on_image, 10)

        # ── Action Client: NavigateToPose ──
        self.nav_action_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        self.get_logger().info(
            f"meter_inspect_task ready  point={self.point_id}  "
            f"meter_type={self.meter_type}  "
            f"nav_goal=({self.nav_goal_x:.3f}, {self.nav_goal_y:.3f}, {self.nav_goal_yaw:.3f})"
        )

    # ── 配置文件 ──

    def _load_point_config(self):
        config_path = str(self.get_parameter("points_config").value)
        self.nav_goal_x = 0.0
        self.nav_goal_y = 0.0
        self.nav_goal_yaw = 0.0

        if not config_path or not os.path.isfile(config_path):
            self.get_logger().warn(f"points config not found: {config_path}")
            return
        try:
            with open(config_path, "r", encoding="utf-8") as f:
                data = json.load(f)
        except Exception as e:
            self.get_logger().error(f"load points config failed: {e}")
            return

        for pt in data.get("points", []):
            if pt.get("point_id") != self.point_id:
                continue
            pose = pt.get("pose", {})
            self.nav_goal_x = float(pose.get("x", 0.0))
            self.nav_goal_y = float(pose.get("y", 0.0))
            self.nav_goal_yaw = float(pose.get("yaw", 0.0))
            if not self.name or self.name == self.point_id:
                self.name = pt.get("name", self.point_id)
            if self.meter_type == "pressure":
                self.meter_type = pt.get("meter_type", "pressure")
            self.detect_confirm_frames = max(1, int(pt.get("detect_confirm_frames", self.detect_confirm_frames)))
            self.detect_pass_ratio = float(pt.get("detect_pass_ratio", self.detect_pass_ratio))
            self.read_frames = max(1, int(pt.get("read_frames", self.read_frames)))
            self.max_retry = max(0, int(pt.get("max_retry", self.max_retry)))
            if pt.get("alarm_low") is not None:
                self.alarm_low = float(pt["alarm_low"])
            if pt.get("alarm_high") is not None:
                self.alarm_high = float(pt["alarm_high"])
            self.get_logger().info(
                f"loaded point {self.point_id}: ({self.nav_goal_x:.3f}, {self.nav_goal_y:.3f})")
            return

        self.get_logger().warn(f"point_id={self.point_id} not found in config")

    # ── 回调 ──

    def _on_meter(self, msg: MeterInfo) -> None:
        self.latest_meter_msg = msg
        self.latest_meter_time = time.time()

    def _on_image(self, msg: Image) -> None:
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warn(f"image convert failed: {exc}")

    # ── 导航 (NavigateToPose action) ──

    def _navigate_to_point(self) -> bool:
        """通过 NavigateToPose action server 导航到点位"""
        self.get_logger().info(
            f"navigating to ({self.nav_goal_x:.3f}, {self.nav_goal_y:.3f}, "
            f"{self.nav_goal_yaw:.3f})")

        # 等 action server 就绪
        if not self.nav_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                "navigate_to_pose action server not available. "
                "Is Nav2 running? (start via frontend or ros2 launch)")
            return False

        # 构建 goal
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = self.nav_goal_x
        goal.pose.pose.position.y = self.nav_goal_y
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation.z = math.sin(self.nav_goal_yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(self.nav_goal_yaw / 2.0)

        # 异步发送 goal
        self.get_logger().info("sending NavigateToPose goal...")
        send_future = self.nav_action_client.send_goal_async(goal)
        self._nav_goal_handle = None
        self._nav_done = False
        self._nav_success = False
        send_future.add_done_callback(self._on_goal_response)

        # 等待导航完成
        deadline = time.time() + self.nav_timeout_sec
        while not self._nav_done and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.5)

        if not self._nav_done:
            # 超时，尝试取消
            self.get_logger().error(f"nav timeout after {self.nav_timeout_sec}s")
            if self._nav_goal_handle:
                self.nav_action_client._cancel_goal_async(self._nav_goal_handle)
            return False

        return self._nav_success

    def _on_goal_response(self, future):
        """goal 被接受后的回调"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("NavigateToPose goal REJECTED by server")
            self._nav_done = True
            self._nav_success = False
            return
        self._nav_goal_handle = goal_handle
        self.get_logger().info("goal accepted, waiting for result...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_nav_result)

    def _on_nav_result(self, future):
        """导航结果回调"""
        result = future.result()
        status = result.status
        # Nav2 action status: 4=SUCCEEDED, 5=CANCELED, 6=ABORTED
        if status == 4:
            self.get_logger().info("navigation SUCCEEDED")
            self._nav_success = True
        elif status == 5:
            self.get_logger().warn("navigation CANCELED")
            self._nav_success = False
        elif status == 6:
            self.get_logger().error("navigation ABORTED")
            self._nav_success = False
        else:
            self.get_logger().error(f"navigation status={status}")
            self._nav_success = False
        self._nav_done = True

    # ── 主流程 ──

    def run(self) -> Dict[str, Any]:
        # Step 1: 导航到点位
        nav_ok = self._navigate_to_point()
        if not nav_ok:
            return self._build_failure_result(FAIL_NAV_FAILED, 0)

        # 到达后等狗稳定
        self.get_logger().info(f"arrived! waiting {self.settle_sec}s for settle...")
        time.sleep(self.settle_sec)

        # Step 2: 读数（独立 timeout，不包含导航时间）
        started_at = time.time()
        attempts = 0
        last_failure = FAIL_DETECT_TIMEOUT

        while attempts <= self.max_retry and (time.time() - started_at) < self.timeout_sec:
            attempts += 1
            self.get_logger().info(f"read attempt {attempts}/{self.max_retry + 1}")

            ok, detect_data = self._confirm_detection(started_at)
            if not ok:
                last_failure = detect_data.get("fail_reason", FAIL_DETECT_TIMEOUT)
                continue

            ok, read_data = self._read_samples(started_at)
            if not ok:
                last_failure = read_data.get("fail_reason", FAIL_READ_TIMEOUT)
                continue

            result = self._validate_result(read_data, detect_data, attempts)
            if result["success"]:
                return result
            last_failure = result.get("fail_reason", FAIL_READ_UNSTABLE)

        return self._build_failure_result(last_failure, attempts)

    # ── 检测确认 ──

    def _confirm_detection(self, started_at: float) -> Tuple[bool, Dict[str, Any]]:
        frames: Deque[Tuple[bool, Optional[Tuple[float, float]]]] = deque(
            maxlen=self.detect_confirm_frames)
        phase_deadline = min(started_at + self.timeout_sec, time.time() + max(5.0, self.settle_sec + 4.0))

        while time.time() < phase_deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            found, center = self._extract_detection_center()
            frames.append((found, center))
            if len(frames) < self.detect_confirm_frames:
                continue

            success_count = sum(1 for f, _ in frames if f)
            pass_ratio = success_count / float(len(frames))
            centers = [c for f, c in frames if f and c is not None]
            jitter_ok = self._check_center_jitter(centers)

            if pass_ratio >= self.detect_pass_ratio and jitter_ok:
                return True, {
                    "pass_ratio": pass_ratio,
                    "centers": centers,
                    "fail_reason": None,
                }

        return False, {
            "fail_reason": FAIL_DETECT_TIMEOUT
            if len(frames) < self.detect_confirm_frames
            else FAIL_DETECT_UNSTABLE,
        }

    # ── 读数采样 ──

    def _read_samples(self, started_at: float) -> Tuple[bool, Dict[str, Any]]:
        samples: List[float] = []
        last_unit = ""
        phase_deadline = min(
            started_at + self.timeout_sec,
            time.time() + max(8.0, self.read_frames * 1.2),
        )

        while time.time() < phase_deadline and len(samples) < self.read_frames:
            rclpy.spin_once(self, timeout_sec=0.2)
            reading = self._extract_meter_value()
            if reading is None:
                continue
            value, unit = reading
            if value is None or value <= 0:
                continue
            samples.append(float(value))
            if unit:
                last_unit = unit

        if not samples:
            return False, {"fail_reason": FAIL_NO_VALID_VALUE}
        if len(samples) < self.read_frames:
            return False, {"fail_reason": FAIL_READ_TIMEOUT, "samples": samples, "unit": last_unit}
        return True, {"samples": samples, "unit": last_unit}

    # ── 结果校验 ──

    def _validate_result(
        self,
        read_data: Dict[str, Any],
        detect_data: Dict[str, Any],
        attempts: int,
    ) -> Dict[str, Any]:
        samples = [float(v) for v in read_data.get("samples", [])]
        median_value = statistics.median(samples)
        mean_value = statistics.mean(samples)
        std_value = statistics.pstdev(samples) if len(samples) > 1 else 0.0
        range_value = max(samples) - min(samples) if samples else 0.0

        if range_value > self.max_value_range:
            return self._build_failure_result(
                FAIL_READ_UNSTABLE,
                attempts,
                samples=samples,
                stability={
                    "median": median_value, "mean": mean_value,
                    "std": std_value, "range": range_value,
                    "valid_count": len(samples), "target_count": self.read_frames,
                },
            )

        alarm = False
        if self.alarm_low is not None and median_value < self.alarm_low:
            alarm = True
        if self.alarm_high is not None and median_value > self.alarm_high:
            alarm = True

        screenshot_path = self._save_snapshot(alarm=alarm)
        result = {
            "success": True,
            "point_id": self.point_id,
            "name": self.name,
            "meter_type": self.meter_type,
            "value": median_value,
            "unit": read_data.get("unit", ""),
            "stability": {
                "median": median_value, "mean": mean_value,
                "std": std_value, "range": range_value,
                "samples": samples,
                "valid_count": len(samples), "target_count": self.read_frames,
                "detect_pass_ratio": detect_data.get("pass_ratio", 0.0),
            },
            "screenshot_path": screenshot_path,
            "timestamp": datetime.now().isoformat(timespec="seconds"),
            "alarm": alarm,
            "fail_reason": FAIL_VALUE_OUT_OF_RANGE if alarm else None,
            "attempts": attempts,
        }
        self._save_result_json(result)
        return result

    # ── 提取 ──

    def _extract_detection_center(self) -> Tuple[bool, Optional[Tuple[float, float]]]:
        msg = self.latest_meter_msg
        if msg is None or msg.count == 0 or not msg.infos:
            return False, None
        meter = msg.infos[0]
        cx = float(meter.roi.x_offset) + float(meter.roi.width) / 2.0
        cy = float(meter.roi.y_offset) + float(meter.roi.height) / 2.0
        return True, (cx, cy)

    def _extract_meter_value(self) -> Optional[Tuple[Optional[float], str]]:
        msg = self.latest_meter_msg
        if msg is None or msg.count == 0 or not msg.infos:
            return None
        for meter in msg.infos:
            if self.meter_type == "pressure" and meter.meter_type != 0:
                continue
            if self.meter_type == "thermo" and meter.meter_type != 1:
                continue
            return float(meter.value), str(meter.unit)
        return None

    def _check_center_jitter(self, centers: List[Tuple[float, float]]) -> bool:
        if len(centers) < 2:
            return True
        xs = [c[0] for c in centers]
        ys = [c[1] for c in centers]
        x_std = statistics.pstdev(xs) if len(xs) > 1 else 0.0
        y_std = statistics.pstdev(ys) if len(ys) > 1 else 0.0
        max_jitter = self.image_width * self.max_center_jitter_ratio
        return x_std <= max_jitter and y_std <= max_jitter

    # ── 保存 ──

    def _save_snapshot(self, alarm: bool) -> str:
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        suffix = "alarm" if alarm else "meter"
        filename = f"{self.point_id or 'meter'}_{suffix}_{ts}.jpg"
        path = os.path.join(self.output_dir, filename)
        if self.latest_image is not None:
            cv2.imwrite(path, self.latest_image)
        return path

    def _save_result_json(self, result: Dict[str, Any]) -> None:
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"{self.point_id or 'meter'}_result_{ts}.json"
        path = os.path.join(self.output_dir, filename)
        with open(path, "w", encoding="utf-8") as f:
            json.dump(result, f, ensure_ascii=False, indent=2)
            f.write("\n")

    def _build_failure_result(
        self,
        reason: str,
        attempts: int,
        samples: Optional[List[float]] = None,
        stability: Optional[Dict[str, Any]] = None,
    ) -> Dict[str, Any]:
        screenshot_path = self._save_snapshot(alarm=False)
        result = {
            "success": False,
            "point_id": self.point_id,
            "name": self.name,
            "meter_type": self.meter_type,
            "value": None,
            "unit": "",
            "stability": stability or {
                "samples": samples or [],
                "valid_count": len(samples or []),
                "target_count": self.read_frames,
            },
            "screenshot_path": screenshot_path,
            "timestamp": datetime.now().isoformat(timespec="seconds"),
            "alarm": False,
            "fail_reason": reason,
            "attempts": attempts,
        }
        self._save_result_json(result)
        return result


def main(args=None):
    rclpy.init(args=args)
    node = MeterInspectTask()
    try:
        result = node.run()
        print(json.dumps(result, ensure_ascii=False, indent=2))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
