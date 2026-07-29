#!/usr/bin/env python3

import asyncio
import json
import threading
import time
from typing import Any, Dict, Optional, Set
from uuid import uuid4

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from cyberdog_ai_pkg.srv import RuntimeSetDetectors

import websockets
from websockets.server import WebSocketServerProtocol


class GatewayState:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._latest: Dict[str, Dict[str, Any]] = {}

    def update(self, msg_type: str, payload: Dict[str, Any]) -> None:
        with self._lock:
            self._latest[msg_type] = dict(payload.get("data", {}))

    def snapshot(self) -> Dict[str, Dict[str, Any]]:
        with self._lock:
            return dict(self._latest)


class VisionWsGatewayNode(Node):
    def __init__(self) -> None:
        super().__init__("vision_ws_gateway")

        self.declare_parameter("ws_host", "0.0.0.0")
        self.declare_parameter("ws_port", 9091)
        self.declare_parameter("ws_path", "/vision")
        self.declare_parameter("detections_topic", "/vision/detections")
        self.declare_parameter("status_topic", "/vision/status")
        self.declare_parameter("runtime_status_topic", "/vision/runtime_status")
        self.declare_parameter("annotated_ws_topic", "/vision/annotated_ws")
        self.declare_parameter("include_annotated", True)
        self.declare_parameter("alarm_ws_topic", "/vision/alarm_ws")

        self.ws_host = str(self.get_parameter("ws_host").value)
        self.ws_port = int(self.get_parameter("ws_port").value)
        self.ws_path = str(self.get_parameter("ws_path").value)
        self.detections_topic = str(self.get_parameter("detections_topic").value)
        self.status_topic = str(self.get_parameter("status_topic").value)
        self.runtime_status_topic = str(self.get_parameter("runtime_status_topic").value)
        self._include_annotated = bool(self.get_parameter("include_annotated").value)

        self.state = GatewayState()
        self._clients: Set[WebSocketServerProtocol] = set()
        self._ws_loop: Optional[asyncio.AbstractEventLoop] = None

        self.create_subscription(
            String, self.detections_topic, lambda msg: self._on_json_payload("detections", msg), 10
        )
        self.create_subscription(
            String, self.status_topic, lambda msg: self._on_json_payload("status", msg), 10
        )
        self.create_subscription(
            String, self.runtime_status_topic, lambda msg: self._on_json_payload("runtime", msg), 10
        )
        self.get_logger().info("Runtime forwarding enabled: %s" % self.runtime_status_topic)

        if self._include_annotated:
            annotated_ws_topic = str(self.get_parameter("annotated_ws_topic").value)
            self.create_subscription(
                String, annotated_ws_topic, lambda msg: self._on_json_payload("annotated", msg), 10
            )
            self.get_logger().info(
                "Annotated image forwarding enabled: %s" % annotated_ws_topic
            )

        alarm_ws_topic = str(self.get_parameter("alarm_ws_topic").value)
        self.create_subscription(
            String, alarm_ws_topic, lambda msg: self._on_json_payload("alarm", msg), 10
        )
        self.get_logger().info("Alarm forwarding enabled: %s" % alarm_ws_topic)

        self.get_logger().info(
            "vision_ws_gateway ready: ws=ws://%s:%d%s detections=%s status=%s"
            % (
                self.ws_host,
                self.ws_port,
                self.ws_path,
                self.detections_topic,
                self.status_topic,
            )
        )

    def set_ws_loop(self, loop: asyncio.AbstractEventLoop) -> None:
        self._ws_loop = loop

    def _schedule_broadcast(self, payload: Dict[str, Any]) -> None:
        self.state.update(str(payload.get("type", "unknown")), payload)
        loop = self._ws_loop
        if loop is None or loop.is_closed():
            return
        asyncio.run_coroutine_threadsafe(self._broadcast(payload), loop)

    def _on_json_payload(self, msg_type: str, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            self.get_logger().warn("%s payload is not valid JSON" % msg_type)
            return

        payload = {
            "type": msg_type,
            "timestamp": time.time(),
            "data": data,
        }
        self._schedule_broadcast(payload)

    async def _handle_set_detectors(self, websocket, detectors, request_id: str) -> None:
        client = self.create_client(RuntimeSetDetectors, "/vision/runtime/set_detectors")
        if not client.wait_for_service(timeout_sec=3.0):
            payload = {
                "type": "set_detectors_result",
                "timestamp": time.time(),
                "request_id": request_id,
                "data": {
                    "success": False,
                    "message": "RuntimeSetDetectors service not available",
                    "desired": [],
                    "running": [],
                    "available": [],
                    "unavailable": [],
                    "failed": list(detectors or []),
                },
            }
            await websocket.send(json.dumps(payload, ensure_ascii=True))
            self.get_logger().warn("RuntimeSetDetectors service not available")
            return

        req = RuntimeSetDetectors.Request()
        req.detectors = [str(d) for d in detectors]
        future = client.call_async(req)

        while rclpy.ok() and not future.done():
            await asyncio.sleep(0.05)

        try:
            result = future.result()
        except Exception as exc:
            payload = {
                "type": "set_detectors_result",
                "timestamp": time.time(),
                "request_id": request_id,
                "data": {
                    "success": False,
                    "message": "RuntimeSetDetectors call failed: %s" % exc,
                    "desired": [],
                    "running": [],
                    "available": [],
                    "unavailable": [],
                    "failed": list(detectors or []),
                },
            }
            await websocket.send(json.dumps(payload, ensure_ascii=True))
            self.get_logger().warn("RuntimeSetDetectors call failed: %s" % exc)
            return

        payload = {
            "type": "set_detectors_result",
            "timestamp": time.time(),
            "request_id": request_id,
            "data": {
                "success": bool(result.success),
                "message": str(result.message),
                "desired": list(result.desired),
                "running": list(result.running),
                "available": list(result.available),
                "unavailable": list(result.unavailable),
                "failed": list(result.failed),
            },
        }
        await websocket.send(json.dumps(payload, ensure_ascii=True))
        self.get_logger().info(
            "RuntimeSetDetectors result: success=%s running=%s failed=%s"
            % (result.success, list(result.running), list(result.failed))
        )

    async def _broadcast(self, payload: Dict[str, Any]) -> None:
        if not self._clients:
            return

        message = json.dumps(payload, ensure_ascii=True)
        dead_clients = []
        for client in list(self._clients):
            try:
                await client.send(message)
            except Exception:
                dead_clients.append(client)

        for client in dead_clients:
            self._clients.discard(client)

    async def ws_handler(self, websocket: WebSocketServerProtocol, path: str) -> None:
        if websocket.path != self.ws_path:
            await websocket.close(code=1008, reason="unsupported path")
            return

        self._clients.add(websocket)
        self.get_logger().info("WS client connected: %s" % (websocket.remote_address,))
        try:
            snapshot = self.state.snapshot()
            if snapshot:
                await websocket.send(
                    json.dumps(
                        {
                            "type": "snapshot",
                            "timestamp": time.time(),
                            "data": snapshot,
                        },
                        ensure_ascii=True,
                    )
                )

            async for raw_msg in websocket:
                try:
                    req = json.loads(raw_msg)
                except (json.JSONDecodeError, TypeError):
                    continue
                action = req.get("action")
                if action == "set_detectors":
                    detectors = req.get("detectors", [])
                    request_id = str(req.get("request_id") or uuid4().hex)
                    await self._handle_set_detectors(websocket, detectors, request_id)
        finally:
            self._clients.discard(websocket)
            self.get_logger().info("WS client disconnected: %s" % (websocket.remote_address,))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VisionWsGatewayNode()

    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    node.set_ws_loop(loop)

    async def spin_ros() -> None:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            await asyncio.sleep(0.0)

    async def run_all() -> None:
        async with websockets.serve(node.ws_handler, node.ws_host, node.ws_port):
            node.get_logger().info(
                "Vision WS server listening on ws://%s:%d%s"
                % (node.ws_host, node.ws_port, node.ws_path)
            )
            await spin_ros()

    try:
        loop.run_until_complete(run_all())
    except KeyboardInterrupt:
        node.get_logger().info("vision_ws_gateway interrupted")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        loop.stop()
        loop.close()


if __name__ == "__main__":
    main()
