#!/usr/bin/env python3
"""
Prepare CyberDog into a movable state. Matches workspace-zhc/scripts/start_mapping.sh:
- Namespace: NS=mi + digits from serial-number (last 8 chars, same as start_mapping.sh).
- Sequence: switch to manual (type=1, control_mode=3) -> sleep 2 -> stand (type=3, order.id=9) -> sleep 3.
- Topic: /$NS/cyberdog_action.
"""

import argparse
import re
import sys
import time

import rclpy
from rclpy.node import Node

from motion_msgs.msg import ActionRequest


def get_namespace():
    """Same as start_mapping.sh: digits from serial-number, last 8 chars, prefix 'mi'."""
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


def _normalize_namespace(value):
    text = str(value).strip()
    if not text:
        return ""
    return text if text.startswith("/") else f"/{text}"


class PrepareRobotNode(Node):
    def __init__(self, namespace, topic_override=None):
        super().__init__("prepare_robot_node")
        ns = _normalize_namespace(namespace) if namespace else ""
        self.ns = ns if ns else "/"
        if topic_override:
            self.action_topic = topic_override if topic_override.startswith("/") else f"/{topic_override}"
        else:
            self.action_topic = "/cyberdog_action" if self.ns == "/" else f"{self.ns}/cyberdog_action"
        self.pub = self.create_publisher(ActionRequest, self.action_topic, 10)
        self.request_id = 1000
        self.get_logger().info("Publishing to: %s" % self.action_topic)

    def _next_request_id(self):
        val = self.request_id
        self.request_id += 1
        return val

    def send_action(
        self,
        action_type,
        control_mode=0,
        mode_type=0,
        gait_id=0,
        order_id=0,
        request_id=None,
        repeat=1,
    ):
        # Format matches start_mapping.sh ros2 topic pub (no nested timestamps)
        rid = int(request_id) if request_id is not None else self._next_request_id()
        for i in range(repeat):
            req = ActionRequest()
            req.type = int(action_type)
            req.request_id = rid + i
            req.mode.control_mode = int(control_mode)
            req.mode.mode_type = int(mode_type)
            req.gait.gait = int(gait_id)
            req.order.id = int(order_id)
            req.order.para = 0.0
            req.timeout = 30
            self.pub.publish(req)
            rclpy.spin_once(self, timeout_sec=0.05)
            if i < repeat - 1:
                time.sleep(0.25)


def prepare_manual_stand(node):
    # Same sequence as start_mapping.sh: manual (1000) -> sleep 2 -> stand (1001, order 9) -> sleep 3
    node.request_id = 1000
    print(">>> 切换到手动模式...")
    node.send_action(action_type=1, request_id=1000, control_mode=3, mode_type=0)
    time.sleep(2.0)

    print(">>> 发送站立命令...")
    node.send_action(action_type=3, request_id=1001, order_id=9)
    print(">>> 等待站立完成 (3秒)...")
    time.sleep(3.0)


def switch_mode(node, mode):
    if mode == "mapping":
        print(">>> 切换到建图模式 (EXPLOR + MAP_NEW)...")
        node.send_action(action_type=1, control_mode=14, mode_type=5)
        time.sleep(2.0)
    elif mode == "navigation":
        print(">>> 切换到导航模式 (EXPLOR + NAV_AB)...")
        node.send_action(action_type=1, control_mode=14, mode_type=3)
        time.sleep(2.0)


def main():
    parser = argparse.ArgumentParser(
        description="Prepare CyberDog for motion control."
    )
    parser.add_argument(
        "--namespace",
        default=get_namespace(),
        help="Robot namespace, e.g. mi1035085",
    )
    parser.add_argument(
        "--mode",
        choices=["manual", "mapping", "navigation"],
        default="manual",
        help="Optional mode switch after standing up",
    )
    parser.add_argument("--request-id-start", type=int, default=1000, help="First request_id (stand sequence uses 1000, 1001)")
    parser.add_argument(
        "--topic",
        default=None,
        help="Override action topic (e.g. /cyberdog_action or /mi1035085/cyberdog_action)",
    )
    args = parser.parse_args()

    rclpy.init()
    node = PrepareRobotNode(args.namespace, topic_override=args.topic)
    node.request_id = args.request_id_start

    try:
        prepare_manual_stand(node)
        switch_mode(node, args.mode)
        ns = (args.namespace.strip("/") or "mi1035085").strip()
        print(">>> 准备完成。现在可以发送 /%s/body_cmd" % ns)
        print(">>> 查看动作结果: ros2 topic echo /%s/cyberdog_action_result" % ns)
    except Exception as e:
        print("prepare_robot failed: %s" % e, file=sys.stderr)
        sys.exit(1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
