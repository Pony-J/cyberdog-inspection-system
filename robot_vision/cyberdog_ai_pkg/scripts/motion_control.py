#!/usr/bin/env python3
"""
CyberDog 运动控制工具 — 正确的准备+移动+退出流程

功能:
  prepare:  切手动 → 站立 → TROT（建图/导航前必须）
  forward:  前进 N 米
  back:     后退 N 米
  turn_left:  左转 N 弧度
  turn_right: 右转 N 弧度
  stop:     急停
  sit:      趴下
  stand:    站立
  gait:     切步态 (8=TROT, 1=WALK, 3=RUN, 0=IDLE)
  mode:     切模式 (manual/mapping/navigation)
  cleanup:  急停 → 趴下 → 默认模式（安全退出）

用法:
  ros2 run cyberdog_ai_pkg motion_control -- prepare
  ros2 run cyberdog_ai_pkg motion_control -- forward --distance 1.0 --speed 0.3
  ros2 run cyberdog_ai_pkg motion_control -- turn_left --angle 1.57 --speed 0.5
  ros2 run cyberdog_ai_pkg motion_control -- stop
  ros2 run cyberdog_ai_pkg motion_control -- sit
  ros2 run cyberdog_ai_pkg motion_control -- cleanup
  ros2 run cyberdog_ai_pkg motion_control -- gait --gait-id 1
  ros2 run cyberdog_ai_pkg motion_control -- mode --mode-name mapping
"""

import argparse
import re
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node

from motion_msgs.msg import ActionRequest, Frameid, SE3VelocityCMD


# ── Namespace ──────────────────────────────────────────────────────

def get_namespace():
    try:
        with open("/sys/firmware/devicetree/base/serial-number", "rb") as f:
            raw = f.read()
        digits = re.sub(r"[^0-9]", "", raw.decode("ascii", errors="ignore"))
        if len(digits) >= 8:
            return "mi" + digits[-8:]
        if digits:
            return "mi" + digits
        return "mi1035085"
    except Exception:
        return "mi1035085"


# ── Node ───────────────────────────────────────────────────────────

class MotionControl(Node):
    def __init__(self, namespace):
        super().__init__("motion_control")
        self.ns = namespace
        self.body_cmd_topic = f"/{self.ns}/body_cmd"
        self.action_topic = f"/{self.ns}/cyberdog_action"

        self.vel_pub = self.create_publisher(SE3VelocityCMD, self.body_cmd_topic, 10)
        self.action_pub = self.create_publisher(ActionRequest, self.action_topic, 10)
        self.request_id = 1000

        self.get_logger().info(f"body_cmd: {self.body_cmd_topic}")
        self.get_logger().info(f"action:   {self.action_topic}")

    # ── Low-level ────────────────────────────────────────────────

    def _next_id(self):
        self.request_id += 1
        return self.request_id

    def _send_action(self, action_type, control_mode=0, mode_type=0,
                     order_id=0, gait_id=0):
        """发布 ActionRequest 到 cyberdog_action topic."""
        req = ActionRequest()
        req.type = action_type
        req.request_id = self._next_id()
        req.mode.control_mode = control_mode
        req.mode.mode_type = mode_type
        req.gait.gait = gait_id
        req.order.id = order_id
        req.order.para = 0.0
        req.timeout = 30
        self.action_pub.publish(req)
        # spin 一下确保发出
        rclpy.spin_once(self, timeout_sec=0.1)

    def _send_velocity(self, vx, vy=0.0, wz=0.0):
        """发布速度指令到 body_cmd topic."""
        cmd = SE3VelocityCMD()
        cmd.sourceid = SE3VelocityCMD.REMOTEC
        cmd.velocity.timestamp = self.get_clock().now().to_msg()
        cmd.velocity.frameid.id = Frameid.BODY_FRAME
        cmd.velocity.linear_x = float(vx)
        cmd.velocity.linear_y = float(vy)
        cmd.velocity.linear_z = 0.0
        cmd.velocity.angular_x = 0.0
        cmd.velocity.angular_y = 0.0
        cmd.velocity.angular_z = float(wz)
        self.vel_pub.publish(cmd)
        rclpy.spin_once(self, timeout_sec=0.1)

    def _stop(self):
        """连发 5 次零速度确保急停."""
        for _ in range(5):
            self._send_velocity(0.0)
            time.sleep(0.05)

    def _change_gait(self, gait_id):
        """用 ros2 action send_goal 切步态 (必须用 action, 不是 topic)."""
        ns = self.ns
        ts = int(time.time())
        cmd = (
            f"timeout 10 ros2 action send_goal /{ns}/checkout_gait "
            f"motion_msgs/action/ChangeGait "
            f"'{{motivation: 253, gaitstamped: "
            f"{{timestamp: {{sec: {ts}, nanosec: 0}}, gait: {gait_id}}}}}'"
        )
        r = subprocess.run(
            ["bash", "-c", cmd],
            capture_output=True, timeout=15,
        )
        if r.returncode != 0:
            self.get_logger().warn(
                f"checkout_gait gait={gait_id} returned {r.returncode}: "
                f"{r.stderr.decode(errors='replace')[:200]}"
            )
        return r.returncode == 0

    # ── High-level commands ──────────────────────────────────────

    def prepare(self):
        """切手动 → 站立 → TROT. 建图/导航/遥控前必须执行."""
        print("[1/3] 切换手动模式 (control_mode=3)...")
        self._send_action(action_type=1, control_mode=3, mode_type=0)
        time.sleep(2.0)

        print("[2/3] 站立 (order_id=9)...")
        self._send_action(action_type=3, order_id=9)
        time.sleep(3.0)

        print("[3/3] 切 TROT 步态 (gait=8)...")
        self._change_gait(8)
        time.sleep(1.0)

        print("准备完成，可以发速度指令了。")

    def cleanup(self):
        """急停 → 趴下 → 默认模式. 安全退出用."""
        print("急停...")
        self._stop()
        time.sleep(0.5)

        print("趴下 (order_id=10)...")
        self._send_action(action_type=3, order_id=10)
        time.sleep(2.0)

        print("切回默认模式 (control_mode=0)...")
        self._send_action(action_type=1, control_mode=0, mode_type=0)
        time.sleep(0.5)

        print("清理完成。")

    def stand(self):
        self._send_action(action_type=3, order_id=9)

    def sit(self):
        """趴下."""
        self._send_action(action_type=3, order_id=10)

    def move_distance(self, distance, speed=0.3):
        """前进/后退指定距离 (米). 正=前, 负=后."""
        if distance == 0:
            return
        direction = 1.0 if distance > 0 else -1.0
        duration = abs(distance) / abs(speed)
        vx = direction * abs(speed)

        print(f"移动 {distance:+.2f}m, 速度 {abs(speed):.2f}m/s, "
              f"预计 {duration:.1f}s...")
        t_start = time.time()
        rate = 20  # Hz
        while time.time() - t_start < duration:
            self._send_velocity(vx)
            time.sleep(1.0 / rate)

        self._stop()
        print("移动完成。")

    def turn_angle(self, angle, speed=0.5):
        """旋转指定角度 (弧度). 正=左转(逆时针), 负=右转(顺时针)."""
        if angle == 0:
            return
        direction = 1.0 if angle > 0 else -1.0
        duration = abs(angle) / abs(speed)
        wz = direction * abs(speed)

        print(f"旋转 {angle:+.2f}rad ({angle*180/3.14159:+.0f}°), "
              f"角速 {abs(speed):.2f}rad/s, 预计 {duration:.1f}s...")
        t_start = time.time()
        rate = 20
        while time.time() - t_start < duration:
            self._send_velocity(0.0, wz=wz)
            time.sleep(1.0 / rate)

        self._stop()
        print("旋转完成。")

    def set_mode(self, mode_name):
        """切狗的模式."""
        modes = {
            "manual":    (3, 0),
            "mapping":   (14, 5),
            "navigation": (14, 3),
            "default":   (0, 0),
            "lock":      (1, 0),
        }
        if mode_name not in modes:
            print(f"未知模式: {mode_name}, 可选: {list(modes.keys())}")
            return
        cm, mt = modes[mode_name]
        print(f"切模式: {mode_name} (control_mode={cm}, mode_type={mt})")
        self._send_action(action_type=1, control_mode=cm, mode_type=mt)
        time.sleep(1.0)

    def set_gait(self, gait_id):
        """切步态."""
        names = {0: "IDLE", 1: "WALK", 3: "RUN", 8: "TROT"}
        print(f"切步态: {names.get(gait_id, gait_id)} (gait={gait_id})")
        self._change_gait(gait_id)


# ── CLI ────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="CyberDog 运动控制")
    parser.add_argument("command", choices=[
        "prepare", "cleanup", "stand", "sit",
        "forward", "back", "turn_left", "turn_right",
        "stop", "mode", "gait",
    ])
    parser.add_argument("--distance", type=float, default=1.0,
                        help="移动距离(米), forward/back 用")
    parser.add_argument("--angle", type=float, default=1.5708,
                        help="旋转角度(弧度), turn_left/turn_right 用, 默认 π/2")
    parser.add_argument("--speed", type=float, default=0.3,
                        help="线速度(m/s)或角速度(rad/s)")
    parser.add_argument("--mode-name", type=str, default="manual",
                        choices=["manual", "mapping", "navigation", "default", "lock"],
                        help="模式名称, mode 命令用")
    parser.add_argument("--gait-id", type=int, default=8,
                        help="步态 ID, gait 命令用 (0=IDLE 1=WALK 3=RUN 8=TROT)")
    parser.add_argument("--namespace", type=str, default=None,
                        help="机器人 namespace, 默认自动检测")
    args = parser.parse_args()

    rclpy.init()
    ns = args.namespace or get_namespace()
    node = MotionControl(ns)

    try:
        cmd = args.command
        if cmd == "prepare":
            node.prepare()
        elif cmd == "cleanup":
            node.cleanup()
        elif cmd == "stand":
            node.stand()
        elif cmd == "sit":
            node.sit()
        elif cmd == "forward":
            node.move_distance(abs(args.distance), args.speed)
        elif cmd == "back":
            node.move_distance(-abs(args.distance), args.speed)
        elif cmd == "turn_left":
            node.turn_angle(abs(args.angle), args.speed)
        elif cmd == "turn_right":
            node.turn_angle(-abs(args.angle), args.speed)
        elif cmd == "stop":
            node._stop()
            print("已急停。")
        elif cmd == "mode":
            node.set_mode(args.mode_name)
        elif cmd == "gait":
            node.set_gait(args.gait_id)
    except KeyboardInterrupt:
        print("\n中断，急停...")
        node._stop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
