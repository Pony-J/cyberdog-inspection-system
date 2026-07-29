import asyncio
import base64
import json
import math
import os
import re
import signal
import subprocess
import threading
import time
import urllib.error
import urllib.request
from pathlib import Path
from typing import Any, Dict, List, Optional

import cv2
import numpy as np
import rclpy
import uvicorn
from ament_index_python.packages import get_package_share_directory
from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse, Response
from fastapi.staticfiles import StaticFiles
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path as NavPath
from nav_msgs.msg import OccupancyGrid
from tf2_msgs.msg import TFMessage
from pydantic import BaseModel
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

try:
    import websocket
    HAS_WS_CLIENT = True
except ImportError:
    HAS_WS_CLIENT = False

try:
    from motion_msgs.msg import ActionRequest, ControlState, Frameid, SE3VelocityCMD
    HAS_MOTION_MSGS = True
except ImportError:
    HAS_MOTION_MSGS = False


# ── Constants ────────────────────────────────────────────────────────────────

MODE_PRESETS = {
    "default": (0, 0),
    "lock": (1, 0),
    "manual": (3, 0),
    "semi": (13, 0),
    "explor_nav": (14, 3),
    "map_update": (14, 4),
    "map_new": (14, 5),
    "track_f": (15, 1),
    "track_s": (15, 2),
}

ORDER_PRESETS = {
    "stand": 9,
    "sit": 18,
    "prostrate": 10,
    "back": 12,
    "turn": 13,
}

MODE_NAMES = {
    (0, 0): "默认", (1, 0): "锁定", (3, 0): "手动",
    (13, 0): "半自主", (14, 3): "探索导航", (14, 4): "地图更新",
    (14, 5): "新建地图", (15, 1): "跟随(前)", (15, 2): "跟随(侧)",
}

GAIT_NAMES = {0: "IDLE", 1: "WALK", 3: "RUN", 8: "TROT"}


def quaternion_to_yaw(z: float, w: float) -> float:
    return 2.0 * math.atan2(z, w)


# ── Pydantic Models ─────────────────────────────────────────────────────────

class InitializeInspectionRequest(BaseModel):
    scene_name: str = "map"

class SetInitialPoseRequest(BaseModel):
    x: float
    y: float
    yaw: float
    covariance_xy: float = 0.25
    covariance_yaw: float = 0.1

class SetNavigationGoalRequest(BaseModel):
    x: float
    y: float
    yaw: float = 0.0

class StartNav2Request(BaseModel):
    mode: str = "slam"
    pbstream_file: str = ""
    map_yaml: str = ""

class RobotModeRequest(BaseModel):
    preset: str = ""
    control_mode: int = -1
    mode_type: int = -1

class RobotOrderRequest(BaseModel):
    preset: str = ""
    order_id: int = -1

class RobotGaitRequest(BaseModel):
    gait: int = 8

class RobotVelocityRequest(BaseModel):
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0

class CameraRequest(BaseModel):
    enable: bool = True

class SetDetectorsRequest(BaseModel):
    detectors: list

class SaveMapRequest(BaseModel):
    filename: str = "cyberdog_map"
    export_pgm: bool = True

class RecordMeterPointRequest(BaseModel):
    point_id: str = ""
    name: str = ""
    meter_type: str = "pressure"
    detect_confirm_frames: int = 5
    detect_pass_ratio: float = 0.7
    read_frames: int = 7
    max_retry: int = 2
    alarm_low: Optional[float] = None
    alarm_high: Optional[float] = None

class RenameMeterPointRequest(BaseModel):
    point_id: str
    name: str

class RecordMeterReadingRequest(BaseModel):
    point_id: str = ""
    point_name: str = ""
    meter_type: str = "pressure"
    reading_value: Optional[float] = None
    reading_unit: str = ""
    reading_text: str = ""
    route_index: int = -1
    source: str = "manual_confirm"
    pose: Optional[Dict[str, float]] = None


# ── Service Manager ─────────────────────────────────────────────────────────

class ServiceManager:
    """Manages ROS2 launch subprocesses (nav2, inspection, etc.)."""

    def __init__(self, logger):
        self._logger = logger
        self._services: Dict[str, Dict[str, Any]] = {}
        self._lock = threading.Lock()
        self._output_lines: Dict[str, List[str]] = {}

    def start(self, name: str, cmd: List[str], env: Optional[Dict] = None,
              extra_info: Optional[Dict] = None) -> Dict[str, Any]:
        with self._lock:
            if name in self._services:
                proc = self._services[name]["process"]
                if proc.poll() is None:
                    raise RuntimeError(f"{name} already running (pid={proc.pid})")
                self._cleanup(name)

            launch_env = env or os.environ.copy()
            process = subprocess.Popen(
                cmd, env=launch_env,
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                preexec_fn=os.setsid,
            )
            self._services[name] = {
                "process": process,
                "started_at": time.time(),
                **(extra_info or {}),
            }
            self._output_lines[name] = []
            threading.Thread(
                target=self._read_output, args=(name, process), daemon=True
            ).start()

        self._logger.info(f"Started {name} (pid={process.pid})")
        return {"pid": process.pid, "name": name}

    def stop(self, name: str, timeout: int = 15) -> None:
        with self._lock:
            info = self._services.get(name)
            if not info:
                return
            proc = info["process"]
            if proc.poll() is not None:
                self._cleanup(name)
                return
            pid = proc.pid

        self._logger.info(f"Stopping {name} (pid={pid})...")
        try:
            os.killpg(os.getpgid(pid), signal.SIGINT)
            proc.wait(timeout=timeout)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            try:
                os.killpg(os.getpgid(pid), signal.SIGKILL)
                proc.wait(timeout=5)
            except Exception:
                pass
        except Exception:
            pass

        with self._lock:
            self._cleanup(name)
        self._logger.info(f"Stopped {name}")

    def _cleanup(self, name: str) -> None:
        self._services.pop(name, None)
        self._output_lines.pop(name, None)

    def _read_output(self, name: str, process: subprocess.Popen) -> None:
        try:
            for raw in iter(process.stdout.readline, b""):
                line = raw.decode("utf-8", errors="replace").rstrip()
                with self._lock:
                    buf = self._output_lines.get(name)
                    if buf is not None:
                        buf.append(line)
                        if len(buf) > 200:
                            buf[:] = buf[-100:]
        except Exception:
            pass

    def get_service_status(self, name: str) -> Optional[Dict[str, Any]]:
        with self._lock:
            info = self._services.get(name)
            if not info:
                return None

            proc = info["process"]
            if proc.poll() is not None:
                self._cleanup(name)
                return None

            started_at = info.get("started_at", time.time())
            return {
                "name": name,
                "running": True,
                "pid": proc.pid,
                "started_at": started_at,
                "uptime": time.time() - started_at,
                "mode": info.get("mode", ""),
                "recent_output": list(self._output_lines.get(name, [])[-8:]),
            }

    def get_status(self) -> Dict[str, Any]:
        with self._lock:
            result = {}
            dead = []
            for name, info in self._services.items():
                running = info["process"].poll() is None
                if not running:
                    dead.append(name)
                result[name] = {
                    "running": running,
                    "pid": info["process"].pid if running else None,
                    "started_at": info.get("started_at"),
                    "uptime": time.time() - info["started_at"] if running else 0,
                    "mode": info.get("mode", ""),
                    "recent_output": list(self._output_lines.get(name, [])[-8:]),
                }
            for k in dead:
                self._cleanup(k)
            return result

    def is_running(self, name: str) -> bool:
        with self._lock:
            if name not in self._services:
                return False
            return self._services[name]["process"].poll() is None

    def stop_all(self) -> None:
        for name in list(self._services.keys()):
            self.stop(name)


# ── Inspection HTTP Client ──────────────────────────────────────────────────

class InspectionHttpClient:
    def __init__(self, base_url: str):
        self._base_url = base_url.rstrip("/")

    def get_maps(self) -> Dict[str, Any]:
        return self._request("GET", "/inspection/internal/maps")

    def get_status(self) -> Dict[str, Any]:
        return self._request("GET", "/inspection/internal/status")

    def get_planned_path(self) -> Dict[str, Any]:
        return self._request("GET", "/inspection/internal/planned_path")

    def get_map_artifacts(self) -> Dict[str, Any]:
        return self._request("GET", "/inspection/internal/map_artifacts")

    def start_initialization(self, scene_name: str) -> Dict[str, Any]:
        return self._request(
            "POST", "/inspection/internal/start_initialization",
            {"scene_name": scene_name or "map"},
        )

    def start(self) -> Dict[str, Any]:
        return self._request("POST", "/inspection/internal/start_inspection")

    def pause(self) -> Dict[str, Any]:
        return self._request("POST", "/inspection/internal/pause_inspection")

    def resume(self) -> Dict[str, Any]:
        return self._request("POST", "/inspection/internal/resume_inspection")

    def stop(self) -> Dict[str, Any]:
        return self._request("POST", "/inspection/internal/stop_inspection")

    def _request(self, method: str, path: str,
                 payload: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        body_bytes = None
        headers = {}
        if payload is not None:
            body_bytes = json.dumps(payload).encode("utf-8")
            headers["Content-Type"] = "application/json"
        req = urllib.request.Request(
            url=self._base_url + path, data=body_bytes,
            headers=headers, method=method,
        )
        try:
            with urllib.request.urlopen(req, timeout=3.0) as resp:
                body = resp.read().decode("utf-8")
        except urllib.error.HTTPError as exc:
            body = exc.read().decode("utf-8", errors="replace")
            raise RuntimeError(f"{method} {path} → {exc.code}: {body}") from exc
        except urllib.error.URLError as exc:
            raise RuntimeError(f"{method} {path} → {exc.reason}") from exc
        try:
            return json.loads(body) if body else {}
        except json.JSONDecodeError:
            raise RuntimeError(f"{method} {path} returned invalid JSON")


# ── Bridge State ────────────────────────────────────────────────────────────

class BridgeState:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._robot_pose: Optional[Dict[str, Any]] = None
        self._global_path: List[Dict[str, float]] = []
        self._local_path: List[Dict[str, float]] = []
        self._inspection_status: Dict[str, Any] = {}
        self._planned_path: List[List[float]] = []
        self._map_artifacts: Dict[str, Any] = {}
        self._last_ros_update: float = 0.0
        self._last_inspection_update: float = 0.0
        self._global_costmap: Optional[Dict[str, Any]] = None
        self._local_costmap: Optional[Dict[str, Any]] = None
        self._live_map: Optional[Dict[str, Any]] = None
        self._live_map_version: int = 0
        self._robot_status: Dict[str, Any] = {}
        self._service_status: Dict[str, Any] = {}
        self._task_status: Dict[str, Dict[str, Any]] = {}
        self._sensor_data: Optional[Dict[str, Any]] = None
        self._vision_data: Dict[str, Any] = {
            "status": {}, "runtime": {}, "detections": {}, "annotated": {},
        }
        self._latest_frame_bytes: Optional[bytes] = None
        self._frame_version: int = 0
        self._alarm_history: List[Dict[str, Any]] = []
        self._alarm_lock = threading.Lock()
        self._meter_history: List[Dict[str, Any]] = []
        self._meter_lock = threading.Lock()

    def update_sensor_data(self, data: Dict[str, Any]) -> None:
        with self._lock:
            self._sensor_data = data

    def update_vision_data(self, msg_type: str, data: Dict[str, Any]) -> None:
        with self._lock:
            self._vision_data[msg_type] = data

    def clear_vision_detections(self) -> None:
        with self._lock:
            self._vision_data["detections"] = {}
            self._vision_data.pop("annotated", None)

    def update_vision_snapshot(self, snapshot: Dict[str, Any]) -> None:
        with self._lock:
            for key in ("status", "runtime", "detections", "annotated"):
                if key in snapshot:
                    self._vision_data[key] = snapshot[key]

    def update_frame(self, jpeg_bytes: bytes) -> None:
        with self._lock:
            self._latest_frame_bytes = jpeg_bytes
            self._frame_version += 1

    def get_frame(self) -> Optional[bytes]:
        with self._lock:
            return self._latest_frame_bytes

    def add_alarm(self, alarm_entry: Dict[str, Any]) -> None:
        with self._alarm_lock:
            self._alarm_history.insert(0, alarm_entry)
            if len(self._alarm_history) > 100:
                self._alarm_history = self._alarm_history[:100]

    def get_alarm_history(self, limit: int = 50) -> List[Dict[str, Any]]:
        with self._alarm_lock:
            return self._alarm_history[:limit]

    def set_meter_history(self, records: List[Dict[str, Any]]) -> None:
        with self._meter_lock:
            self._meter_history = list(records[:200])

    def add_meter_record(self, record: Dict[str, Any]) -> None:
        with self._meter_lock:
            self._meter_history.insert(0, record)
            if len(self._meter_history) > 200:
                self._meter_history = self._meter_history[:200]

    def get_meter_history(self, limit: int = 50) -> List[Dict[str, Any]]:
        with self._meter_lock:
            return self._meter_history[:limit]

    def delete_meter_record(self, record_id: str) -> bool:
        with self._meter_lock:
            before = len(self._meter_history)
            self._meter_history = [
                item for item in self._meter_history
                if str(item.get("id", "")) != record_id
            ]
            return len(self._meter_history) != before

    def update_robot_pose(self, pose: Dict[str, Any]) -> None:
        with self._lock:
            self._robot_pose = pose
            self._last_ros_update = time.time()

    def update_global_path(self, path: List[Dict[str, float]]) -> None:
        with self._lock:
            self._global_path = path
            self._last_ros_update = time.time()

    def update_local_path(self, path: List[Dict[str, float]]) -> None:
        with self._lock:
            self._local_path = path
            self._last_ros_update = time.time()

    def update_inspection(self, status_payload: Dict[str, Any],
                          planned_path_payload: Dict[str, Any],
                          artifacts_payload: Dict[str, Any]) -> None:
        with self._lock:
            self._inspection_status = status_payload
            self._planned_path = planned_path_payload.get("planned_path", [])
            self._map_artifacts = artifacts_payload
            self._last_inspection_update = time.time()

    def update_costmap(self, costmap_type: str, data: Dict[str, Any]) -> None:
        with self._lock:
            if costmap_type == "global":
                self._global_costmap = data
            elif costmap_type == "local":
                self._local_costmap = data

    def update_live_map(self, data: Dict[str, Any]) -> None:
        with self._lock:
            self._live_map = data
            self._live_map_version += 1

    def get_live_map_version(self) -> int:
        with self._lock:
            return self._live_map_version

    def update_robot_status(self, status: Dict[str, Any]) -> None:
        with self._lock:
            self._robot_status = status

    def update_service_status(self, status: Dict[str, Any]) -> None:
        with self._lock:
            self._service_status = status

    def update_task(self, task_id: str, data: Dict[str, Any]) -> None:
        with self._lock:
            self._task_status[task_id] = data

    def get_task(self, task_id: str) -> Dict[str, Any]:
        with self._lock:
            return dict(self._task_status.get(task_id, {}))

    def snapshot(self) -> Dict[str, Any]:
        with self._lock:
            return {
                "robot_pose": self._robot_pose,
                "global_path": list(self._global_path),
                "local_path": list(self._local_path),
                "inspection_status": dict(self._inspection_status),
                "planned_path": list(self._planned_path),
                "map_artifacts": dict(self._map_artifacts),
                "last_ros_update": self._last_ros_update,
                "last_inspection_update": self._last_inspection_update,
                "live_map_version": self._live_map_version,
                "robot_status": dict(self._robot_status),
                "service_status": dict(self._service_status),
                "tasks": dict(self._task_status),
                "sensor_data": dict(self._sensor_data) if self._sensor_data else None,
                "vision_data": dict(self._vision_data),
            }


# ── Main Node ───────────────────────────────────────────────────────────────

class CyberdogWebBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("cyberdog_web_bridge")

        workspace_default = os.environ.get("CYBERDOG_ROS2_WS", "/opt/cyberdog/ros2_ws")
        data_default = os.environ.get("CYBERDOG_DATA_DIR", "/opt/cyberdog/runtime_data")
        vision_ws_default = os.environ.get(
            "CYBERDOG_VISION_WS_URL", "ws://127.0.0.1:9091/vision"
        )

        self.declare_parameter("http_host", "0.0.0.0")
        self.declare_parameter("http_port", 8090)
        self.declare_parameter("inspection_base_url", "http://127.0.0.1:8083")
        self.declare_parameter("inspection_poll_sec", 0.5)
        self.declare_parameter("ws_interval_sec", 1.0)
        self.declare_parameter("cyberdog_ns", "mi1035085")
        self.declare_parameter("workspace_dir", workspace_default)
        self.declare_parameter("vision_ws_url", vision_ws_default)
        self.declare_parameter("vision_annotated", False)
        self.declare_parameter("alarm_history_dir", os.path.join(data_default, "alarms"))
        self.declare_parameter(
            "meter_history_path", os.path.join(data_default, "meter_history.json")
        )
        self.declare_parameter(
            "meter_points_path",
            os.path.join(data_default, "meter_points.json"),
        )
        self.declare_parameter(
            "nav2_cyclonedds_xml",
            os.path.join(workspace_default, "src/cyberdog_nav2_lidar/cyclonedds.xml"),
        )
        self.declare_parameter("map_output_dir", os.path.join(workspace_default, "maps"))
        self.declare_parameter(
            "cors_origins",
            ["http://127.0.0.1:8090", "http://localhost:8090"],
        )
        self.declare_parameter("alarm_tts_enabled", False)
        self.declare_parameter("alarm_tts_url", "http://127.0.0.1:8091")
        self.declare_parameter("alarm_tts_format", "wav")
        self.declare_parameter(
            "voice_brain_dir", os.environ.get("CYBERDOG_VOICE_BRAIN_DIR", "")
        )
        self.declare_parameter(
            "tts_model_path", os.environ.get("CYBERDOG_TTS_MODEL", "")
        )

        self.http_host = self.get_parameter("http_host").value
        self.http_port = int(self.get_parameter("http_port").value)
        self.inspection_poll_sec = float(self.get_parameter("inspection_poll_sec").value)
        self.ws_interval_sec = float(self.get_parameter("ws_interval_sec").value)
        self.cyberdog_ns = str(self.get_parameter("cyberdog_ns").value)
        self.workspace_dir = str(self.get_parameter("workspace_dir").value)
        self.vision_ws_url = str(self.get_parameter("vision_ws_url").value)
        self.vision_annotated = bool(self.get_parameter("vision_annotated").value)
        self.alarm_history_dir = str(self.get_parameter("alarm_history_dir").value)
        self.meter_history_path = str(self.get_parameter("meter_history_path").value)
        self.meter_points_path = str(self.get_parameter("meter_points_path").value)
        self.nav2_cyclonedds_xml = str(self.get_parameter("nav2_cyclonedds_xml").value)
        self.map_output_dir = str(self.get_parameter("map_output_dir").value)
        self.cors_origins = [
            str(origin) for origin in self.get_parameter("cors_origins").value if origin
        ]

        os.makedirs(self.alarm_history_dir, exist_ok=True)
        os.makedirs(os.path.dirname(self.meter_history_path), exist_ok=True)
        os.makedirs(os.path.dirname(self.meter_points_path), exist_ok=True)

        self.inspection_client = InspectionHttpClient(
            str(self.get_parameter("inspection_base_url").value)
        )
        self.state = BridgeState()
        self._load_alarm_history()
        self._load_meter_history()
        self.service_manager = ServiceManager(self.get_logger())
        self.static_dir = (
            Path(get_package_share_directory("cyberdog_web_bridge")) / "static"
        )
        self._last_poll_error = 0.0
        self._map_frame_transforms: Dict[str, Dict[str, float]] = {}
        self._odom_to_base: Optional[Dict[str, float]] = None

        ns = self.cyberdog_ns

        # ── Standard publishers ──────────────────────────────────────
        self.initial_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10
        )
        self.navigation_goal_publisher = self.create_publisher(
            PoseStamped, "/goal_pose", 10
        )
        self.navigation_goal_ns_publisher = self.create_publisher(
            PoseStamped, f"/{ns}/goal_pose", 10
        )

        # ── CyberDog-specific publishers/subscribers ─────────────────
        if HAS_MOTION_MSGS:
            self.action_pub = self.create_publisher(
                ActionRequest, f"/{ns}/cyberdog_action", 10
            )
            self.body_cmd_pub = self.create_publisher(
                SE3VelocityCMD, f"/{ns}/body_cmd", 20
            )
            self.create_subscription(
                ControlState, f"/{ns}/status_out", self._on_robot_status, 10
            )
            self.get_logger().info("motion_msgs loaded — robot control enabled")
        else:
            self.action_pub = None
            self.body_cmd_pub = None
            self.get_logger().warn(
                "motion_msgs unavailable — robot control disabled"
            )

        # ── Standard subscribers ─────────────────────────────────────
        self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self._on_amcl_pose, 10
        )
        self.create_subscription(NavPath, "/plan", self._on_global_path, 10)
        self.create_subscription(NavPath, "/local_plan", self._on_local_path, 10)
        self.create_subscription(
            NavPath, f"/{ns}/plan", self._on_global_path, 10
        )
        self.create_subscription(
            NavPath, f"/{ns}/local_plan", self._on_local_path, 10
        )
        self.create_subscription(TFMessage, "/tf", self._on_tf, 10)

        map_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        volatile_map_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(OccupancyGrid, "/map", self._on_map, map_qos)
        self.create_subscription(
            OccupancyGrid, "/map", self._on_map, volatile_map_qos
        )
        self.create_subscription(
            OccupancyGrid, f"/{ns}/map", self._on_map, map_qos
        )
        self.create_subscription(
            OccupancyGrid, "/global_costmap/costmap",
            lambda msg: self._on_costmap(msg, "global"), 1,
        )
        self.create_subscription(
            OccupancyGrid, "/local_costmap/costmap",
            lambda msg: self._on_costmap(msg, "local"), 1,
        )

        # ── Timers ───────────────────────────────────────────────────
        self.create_timer(self.inspection_poll_sec, self._poll_inspection_state)
        self.create_timer(2.0, self._sync_service_status)

        # ── Vision WS client ──────────────────────────────────────────
        self._vision_connected = False
        if HAS_WS_CLIENT:
            threading.Thread(
                target=self._vision_ws_loop, daemon=True,
            ).start()
        else:
            self.get_logger().warn(
                "websocket-client unavailable — vision WS disabled"
            )

        # ── Web server ───────────────────────────────────────────────
        self.app = self._create_app()
        threading.Thread(target=self._run_web_server, daemon=True).start()
        self.get_logger().info(
            f"CyberDog Web Console → http://{self.http_host}:{self.http_port}"
        )

    # ── Web server ───────────────────────────────────────────────────────────

    def _run_web_server(self) -> None:
        uvicorn.Server(
            uvicorn.Config(
                self.app, host=self.http_host,
                port=self.http_port, log_level="info",
            )
        ).run()

    # ── Vision WS client ─────────────────────────────────────────────────────

    def _on_vision_ws_message(self, raw: str) -> None:
        """Handle incoming vision WS message from the dog."""
        try:
            msg = json.loads(raw)
        except json.JSONDecodeError:
            self.get_logger().warn(f"Vision WS: bad JSON ({raw[:80]})")
            return

        msg_type = msg.get("type")
        data = msg.get("data", {})

        if msg_type == "snapshot":
            self.state.update_vision_snapshot(data)
        elif msg_type == "detections":
            self.state.update_vision_data(msg_type, data)
        elif msg_type in ("status", "runtime"):
            self.state.update_vision_data(msg_type, data)
        elif msg_type == "annotated":
            if "image_base64" in data:
                try:
                    jpeg_bytes = base64.b64decode(data["image_base64"])
                    self.state.update_frame(jpeg_bytes)
                except Exception:
                    pass
            if self.vision_annotated:
                self.state.update_vision_data(msg_type, data)
        elif msg_type == "alarm":
            self._handle_alarm_message(data)

    def _handle_alarm_message(self, data: Dict[str, Any]) -> None:
        """Save alarm image and metadata to local storage."""
        try:
            timestamp = data.get("timestamp", "")
            detections = data.get("detections", [])
            image_base64 = data.get("image_base64", "")

            if not timestamp or not image_base64:
                return

            # Decode and save image
            jpeg_bytes = base64.b64decode(image_base64)
            img_filename = f"alarm_{timestamp}.jpg"
            json_filename = f"alarm_{timestamp}.json"
            img_path = os.path.join(self.alarm_history_dir, img_filename)
            json_path = os.path.join(self.alarm_history_dir, json_filename)

            with open(img_path, "wb") as f:
                f.write(jpeg_bytes)

            with open(json_path, "w") as f:
                json.dump(detections, f, ensure_ascii=True, indent=2)

            # Add to in-memory history
            for det in detections:
                alarm_entry = {
                    "id": f"alarm_{timestamp}",
                    "timestamp": timestamp,
                    "detector": det.get("detector", "unknown"),
                    "class_id": det.get("class_id", "unknown"),
                    "description": det.get("description", ""),
                    "score": det.get("score", 0.0),
                }
                self.state.add_alarm(alarm_entry)

            # Cleanup old alarms
            self._cleanup_old_alarms()

            self.get_logger().info(f"Alarm saved: {img_filename} ({len(detections)} detection(s))")
            self._alarm_tts(detections)

        except Exception as e:
            self.get_logger().error(f"Failed to handle alarm message: {e}")

    def _alarm_tts(self, detections: List[Dict]) -> None:
        """用 Piper TTS 合成报警语音，通过 audio_server 播放。"""
        if not self.get_parameter("alarm_tts_enabled").value:
            return

        # 生成播报文本：取 detections 的 description
        descs = [d.get("description", "") for d in detections if d.get("description")]
        seen = set()
        unique = []
        for d in descs:
            d = d.strip()
            if d and d not in seen:
                seen.add(d)
                unique.append(d)

        if not unique:
            return

        text = "注意，检测到：" + "，".join(unique)

        # 子线程执行，不阻塞主循环
        threading.Thread(target=self._do_alarm_tts, args=(text,), daemon=True).start()

    def _do_alarm_tts(self, text: str) -> None:
        """实际执行 TTS 合成和播放（子线程），播报期间暂停巡检。"""
        try:
            import sys
            voice_brain_dir = str(self.get_parameter("voice_brain_dir").value).strip()
            tts_model_path = str(self.get_parameter("tts_model_path").value).strip()
            if not voice_brain_dir or not tts_model_path:
                self.get_logger().warn(
                    "Alarm TTS skipped: voice_brain_dir/tts_model_path is not configured"
                )
                return
            if voice_brain_dir not in sys.path:
                sys.path.insert(0, voice_brain_dir)
            from tts_piper import PiperTTS
            tts_cfg = {
                "tts": {
                    "piper": {
                        "model": tts_model_path
                    }
                }
            }
            tts = PiperTTS(tts_cfg)
            audio_data = tts.synthesize(text)
            if not audio_data:
                return

            # 暂停巡检
            inspection_was_running = False
            try:
                status = self.inspection_client.get_status()
                status_data = status.get("data", status)
                state_str = str(
                    status_data.get("inspection_status_name")
                    or status_data.get("status_name")
                    or status_data.get("status")
                    or ""
                ).upper()
                if state_str in ("INSPECTION_IN_PROGRESS", "IN_PROGRESS", "RUNNING"):
                    self.inspection_client.pause()
                    inspection_was_running = True
                    self.get_logger().info("Inspection paused for alarm TTS")
            except Exception as e:
                self.get_logger().warn(f"Failed to pause inspection: {e}")

            tts_format = str(self.get_parameter("alarm_tts_format").value)
            base_url = str(self.get_parameter("alarm_tts_url").value)
            content_type = "audio/wav" if tts_format == "wav" else "audio/mpeg"

            import requests
            requests.post(
                f"{base_url}/play",
                data=audio_data,
                headers={"Content-Type": content_type},
                timeout=30,
            )
            self.get_logger().info(f"Alarm TTS played: {text}")

            # 播报完成，恢复巡检
            if inspection_was_running:
                try:
                    self.inspection_client.resume()
                    self.get_logger().info("Inspection resumed after alarm TTS")
                except Exception as e:
                    self.get_logger().warn(f"Failed to resume inspection: {e}")
        except Exception as e:
            self.get_logger().error(f"Alarm TTS failed: {e}")

    def _cleanup_old_alarms(self) -> None:
        """Remove oldest alarm files when exceeding limit."""
        try:
            alarm_files = sorted(
                [f for f in os.listdir(self.alarm_history_dir) if f.startswith("alarm_") and f.endswith(".jpg")]
            )
            if len(alarm_files) > 100:
                for old_file in alarm_files[:-100]:
                    img_path = os.path.join(self.alarm_history_dir, old_file)
                    json_path = img_path.replace(".jpg", ".json")
                    if os.path.exists(img_path):
                        os.remove(img_path)
                    if os.path.exists(json_path):
                        os.remove(json_path)
        except Exception as e:
            self.get_logger().warn(f"Alarm cleanup failed: {e}")

    def _load_meter_points(self) -> Dict[str, Any]:
        try:
            if not os.path.isfile(self.meter_points_path):
                return {"points": []}
            with open(self.meter_points_path, "r", encoding="utf-8") as f:
                data = json.load(f)
            points = data.get("points", []) if isinstance(data, dict) else []
            return {"points": points if isinstance(points, list) else []}
        except Exception as e:
            self.get_logger().warn(f"Failed to load meter points: {e}")
            return {"points": []}

    def _save_meter_points(self, data: Dict[str, Any]) -> None:
        with open(self.meter_points_path, "w", encoding="utf-8") as f:
            json.dump(data, f, ensure_ascii=False, indent=2)
            f.write("\n")

    def _next_meter_point_id(self, points: List[Dict[str, Any]]) -> str:
        used = {str(point.get("point_id", "")) for point in points}
        index = 1
        while True:
            candidate = f"meter_{index:02d}"
            if candidate not in used:
                return candidate
            index += 1

    def _normalize_meter_point(self, point: Dict[str, Any]) -> Dict[str, Any]:
        pose = point.get("pose", {}) if isinstance(point.get("pose"), dict) else {}
        alarm_low = point.get("alarm_low")
        alarm_high = point.get("alarm_high")
        return {
            "point_id": str(point.get("point_id", "")).strip(),
            "name": str(point.get("name", "")).strip(),
            "pose": {
                "x": float(pose.get("x", 0.0)),
                "y": float(pose.get("y", 0.0)),
                "yaw": float(pose.get("yaw", 0.0)),
            },
            "meter_type": str(point.get("meter_type", "pressure") or "pressure"),
            "detect_confirm_frames": max(1, int(point.get("detect_confirm_frames", 5))),
            "detect_pass_ratio": min(1.0, max(0.1, float(point.get("detect_pass_ratio", 0.7)))),
            "read_frames": max(1, int(point.get("read_frames", 7))),
            "max_retry": max(0, int(point.get("max_retry", 2))),
            "alarm_low": None if alarm_low in (None, "") else float(alarm_low),
            "alarm_high": None if alarm_high in (None, "") else float(alarm_high),
        }

    def _record_meter_point(self, req: RecordMeterPointRequest) -> Dict[str, Any]:
        snap = self.state.snapshot()
        robot_pose = snap.get("robot_pose")
        if not robot_pose:
            raise HTTPException(503, "No robot pose available")

        data = self._load_meter_points()
        points = data.get("points", [])
        point_id = (req.point_id or "").strip() or self._next_meter_point_id(points)
        name = (req.name or "").strip() or point_id
        point = self._normalize_meter_point({
            "point_id": point_id,
            "name": name,
            "pose": {
                "x": robot_pose.get("x", 0.0),
                "y": robot_pose.get("y", 0.0),
                "yaw": robot_pose.get("yaw", 0.0),
            },
            "meter_type": req.meter_type,
            "detect_confirm_frames": req.detect_confirm_frames,
            "detect_pass_ratio": req.detect_pass_ratio,
            "read_frames": req.read_frames,
            "max_retry": req.max_retry,
            "alarm_low": req.alarm_low,
            "alarm_high": req.alarm_high,
        })

        replaced = False
        for index, existing in enumerate(points):
            if str(existing.get("point_id", "")) == point_id:
                points[index] = point
                replaced = True
                break
        if not replaced:
            points.append(point)
        self._save_meter_points({"points": points})
        return point

    def _normalize_meter_record(self, record: Dict[str, Any]) -> Dict[str, Any]:
        pose = record.get("pose", {}) if isinstance(record.get("pose"), dict) else {}
        reading_value = record.get("reading_value")
        normalized_value = None
        if reading_value not in (None, ""):
            try:
                normalized_value = float(reading_value)
            except Exception:
                normalized_value = None
        route_index = record.get("route_index", -1)
        try:
            route_index = int(route_index)
        except Exception:
            route_index = -1
        return {
            "id": str(record.get("id", "")).strip(),
            "timestamp": str(record.get("timestamp", "")).strip(),
            "point_id": str(record.get("point_id", "")).strip(),
            "point_name": str(record.get("point_name", "")).strip(),
            "meter_type": str(record.get("meter_type", "pressure") or "pressure"),
            "reading_value": normalized_value,
            "reading_unit": str(record.get("reading_unit", "") or "").strip(),
            "reading_text": str(record.get("reading_text", "") or "").strip(),
            "route_index": route_index,
            "source": str(record.get("source", "manual_confirm") or "manual_confirm").strip(),
            "pose": {
                "x": float(pose.get("x", 0.0)),
                "y": float(pose.get("y", 0.0)),
                "yaw": float(pose.get("yaw", 0.0)),
            },
        }

    def _persist_meter_history(self) -> None:
        records = self.state.get_meter_history(200)
        with open(self.meter_history_path, "w", encoding="utf-8") as f:
            json.dump({"records": records}, f, ensure_ascii=False, indent=2)
            f.write("\n")

    def _record_meter_reading(self, req: RecordMeterReadingRequest) -> Dict[str, Any]:
        reading_text = str(req.reading_text or "").strip()
        reading_value = None if req.reading_value is None else float(req.reading_value)
        reading_unit = str(req.reading_unit or "").strip()
        if reading_value is None and not reading_text:
            raise HTTPException(400, "No confirmed meter reading")

        if not reading_text and reading_value is not None:
            reading_text = f"{reading_value}{(' ' + reading_unit) if reading_unit else ''}".strip()

        snap = self.state.snapshot()
        robot_pose = snap.get("robot_pose") or {}
        pose = req.pose or {
            "x": robot_pose.get("x", 0.0),
            "y": robot_pose.get("y", 0.0),
            "yaw": robot_pose.get("yaw", 0.0),
        }

        now = time.time()
        timestamp = time.strftime("%Y%m%d_%H%M%S", time.localtime(now))
        record = self._normalize_meter_record({
            "id": f"meter_{timestamp}_{int((now % 1.0) * 1000):03d}",
            "timestamp": timestamp,
            "point_id": req.point_id,
            "point_name": req.point_name or req.point_id,
            "meter_type": req.meter_type,
            "reading_value": reading_value,
            "reading_unit": reading_unit,
            "reading_text": reading_text,
            "route_index": req.route_index,
            "source": req.source,
            "pose": pose,
        })
        self.state.add_meter_record(record)
        self._persist_meter_history()
        return record

    def _load_alarm_history(self) -> None:
        """Load existing alarm files from disk into in-memory history on startup."""
        try:
            if not os.path.isdir(self.alarm_history_dir):
                return

            json_files = sorted(
                [f for f in os.listdir(self.alarm_history_dir) if f.startswith("alarm_") and f.endswith(".json")],
                reverse=True,
            )

            loaded = 0
            for json_file in json_files:
                if loaded >= 100:
                    break
                alarm_id = json_file[:-5]
                img_file = alarm_id + ".jpg"
                img_path = os.path.join(self.alarm_history_dir, img_file)
                json_path = os.path.join(self.alarm_history_dir, json_file)

                if not os.path.exists(img_path):
                    continue

                try:
                    with open(json_path, "r") as f:
                        detections = json.load(f)
                    if not isinstance(detections, list):
                        continue
                except Exception:
                    continue

                timestamp = alarm_id.replace("alarm_", "", 1)
                for det in detections:
                    if loaded >= 100:
                        break
                    alarm_entry = {
                        "id": alarm_id,
                        "timestamp": timestamp,
                        "detector": det.get("detector", "unknown"),
                        "class_id": det.get("class_id", "unknown"),
                        "description": det.get("description", ""),
                        "score": det.get("score", 0.0),
                    }
                    self.state.add_alarm(alarm_entry)
                    loaded += 1

            if loaded:
                self.get_logger().info(f"Loaded {loaded} alarm(s) from {self.alarm_history_dir}")

        except Exception as e:
            self.get_logger().warn(f"Failed to load alarm history from disk: {e}")

    def _load_meter_history(self) -> None:
        try:
            if not os.path.isfile(self.meter_history_path):
                return
            with open(self.meter_history_path, "r", encoding="utf-8") as f:
                data = json.load(f)
            records = data.get("records", []) if isinstance(data, dict) else []
            if not isinstance(records, list):
                records = []
            normalized = []
            for record in records[:200]:
                if not isinstance(record, dict):
                    continue
                normalized.append(self._normalize_meter_record(record))
            self.state.set_meter_history(normalized)
            if normalized:
                self.get_logger().info(
                    f"Loaded {len(normalized)} meter record(s) from {self.meter_history_path}"
                )
        except Exception as e:
            self.get_logger().warn(f"Failed to load meter history from disk: {e}")

    def _vision_ws_loop(self) -> None:
        """Background loop connecting to the dog's vision WS gateway."""
        url = self.vision_ws_url
        while True:
            self.get_logger().info(f"Vision WS → connecting {url}")
            try:
                ws = websocket.WebSocketApp(
                    url,
                    on_message=lambda _, msg: self._on_vision_ws_message(msg),
                    on_error=lambda _, err: self.get_logger().warn(
                        f"Vision WS error: {err}"
                    ),
                    on_open=lambda _: (
                        setattr(self, "_vision_connected", True),
                        self.get_logger().info("Vision WS connected"),
                        self.get_logger().info(f"Vision WS: subscribed to {url}, waiting for messages..."),
                    ),
                    on_close=lambda _, code, reason: (
                        setattr(self, "_vision_connected", False),
                        self.get_logger().info(
                            f"Vision WS closed (code={code})"
                        ),
                    ),
                )
                ws.run_forever(ping_interval=15, ping_timeout=10)
            except Exception as exc:
                self.get_logger().error(f"Vision WS loop error: {exc}")
            self._vision_connected = False
            self.get_logger().info("Vision WS → reconnect in 5s")
            time.sleep(5)

    # ── Timer callbacks ──────────────────────────────────────────────────────

    def _poll_inspection_state(self) -> None:
        try:
            s = self.inspection_client.get_status()
            p = self.inspection_client.get_planned_path()
            a = self.inspection_client.get_map_artifacts()
            self.state.update_inspection(s, p, a)
            self._inspection_fail_count = 0
        except Exception as exc:
            self._inspection_fail_count = getattr(
                self, "_inspection_fail_count", 0
            ) + 1
            now = time.time()
            backoff = min(
                60.0, 5.0 * (2 ** min(self._inspection_fail_count - 1, 4))
            )
            if now - self._last_poll_error > backoff:
                self.get_logger().debug(f"Inspection poll: {exc}")
                self._last_poll_error = now

    def _sync_service_status(self) -> None:
        self.state.update_service_status(self.service_manager.get_status())

    # ── ROS2 callbacks ───────────────────────────────────────────────────────

    def _on_amcl_pose(self, msg: PoseWithCovarianceStamped) -> None:
        pose = msg.pose.pose
        self.state.update_robot_pose({
            "x": pose.position.x, "y": pose.position.y,
            "yaw": quaternion_to_yaw(pose.orientation.z, pose.orientation.w),
            "frame_id": msg.header.frame_id or "map",
            "stamp_sec": float(msg.header.stamp.sec),
        })

    def _on_tf(self, msg: TFMessage) -> None:
        updated = False
        for t in msg.transforms:
            fid = t.header.frame_id
            cid = t.child_frame_id
            if fid == "map" and cid in {"odom_fixed", "odom"}:
                self._map_frame_transforms[cid] = {
                    "x": t.transform.translation.x,
                    "y": t.transform.translation.y,
                    "yaw": quaternion_to_yaw(
                        t.transform.rotation.z, t.transform.rotation.w
                    ),
                }
                updated = True
            if fid in {"odom_fixed", "odom"} and cid == "base_footprint_fixed":
                self._odom_to_base = {
                    "x": t.transform.translation.x,
                    "y": t.transform.translation.y,
                    "yaw": quaternion_to_yaw(
                        t.transform.rotation.z, t.transform.rotation.w
                    ),
                    "parent": fid,
                    "stamp_sec": float(t.header.stamp.sec),
                }
                updated = True
        if updated:
            self._compose_robot_pose()

    def _compose_robot_pose(self) -> None:
        if self._odom_to_base is None:
            return
        pf = self._odom_to_base.get("parent", "odom_fixed")
        m2o = self._map_frame_transforms.get(pf)
        if m2o is None:
            return
        c, s = math.cos(m2o["yaw"]), math.sin(m2o["yaw"])
        bx = m2o["x"] + c * self._odom_to_base["x"] - s * self._odom_to_base["y"]
        by = m2o["y"] + s * self._odom_to_base["x"] + c * self._odom_to_base["y"]
        self.state.update_robot_pose({
            "x": bx, "y": by,
            "yaw": m2o["yaw"] + self._odom_to_base["yaw"],
            "frame_id": "map",
            "stamp_sec": self._odom_to_base.get("stamp_sec", 0.0),
        })

    def _on_map(self, msg: OccupancyGrid) -> None:
        w, h = msg.info.width, msg.info.height
        if w == 0 or h == 0:
            return
        data = np.array(msg.data, dtype=np.int8).reshape((h, w))
        data = np.flipud(data)
        img = np.zeros((h, w, 4), dtype=np.uint8)
        free = (data >= 0) & (data <= 30)
        img[free] = [255, 255, 255, 255]
        occ = data >= 65
        img[occ] = [29, 29, 31, 255]
        partial = (data > 30) & (data < 65)
        if np.any(partial):
            gray = np.clip(220 - data[partial].astype(np.uint8) * 2, 80, 200)
            img[partial, 0] = gray
            img[partial, 1] = gray
            img[partial, 2] = gray
            img[partial, 3] = 255
        ok, enc = cv2.imencode(".png", img)
        if ok:
            self.state.update_live_map({
                "image_bytes": enc.tobytes(),
                "width": w, "height": h,
                "resolution": msg.info.resolution,
                "origin": [
                    msg.info.origin.position.x,
                    msg.info.origin.position.y, 0.0,
                ],
            })

    def _on_costmap(self, msg: OccupancyGrid, costmap_type: str) -> None:
        w, h = msg.info.width, msg.info.height
        res = msg.info.resolution
        fid = msg.header.frame_id or "map"
        ox = msg.info.origin.position.x
        oy = msg.info.origin.position.y
        oyaw = quaternion_to_yaw(
            msg.info.origin.orientation.z, msg.info.origin.orientation.w,
        )
        data = np.array(msg.data, dtype=np.int8).reshape((h, w))
        data = np.flipud(data)
        img = np.zeros((h, w, 4), dtype=np.uint8)
        lr, ir = ((255, 59, 48), (255, 159, 10)) if costmap_type == "local" \
            else ((88, 86, 214), (0, 122, 255))
        lethal = data == 100
        img[lethal] = [lr[0], lr[1], lr[2], 230]
        inflated = (data > 0) & (data < 100)
        if np.any(inflated):
            img[inflated, 0] = ir[0]
            img[inflated, 1] = ir[1]
            img[inflated, 2] = ir[2]
            img[inflated, 3] = np.clip(
                data[inflated] * 2.2, 20, 190
            ).astype(np.uint8)
        ok, enc = cv2.imencode(".png", img)
        if ok:
            mx, my, myaw, mf = ox, oy, oyaw, fid
            if fid != "map":
                tf = self._map_frame_transforms.get(fid)
                if tf is not None:
                    c, s = math.cos(tf["yaw"]), math.sin(tf["yaw"])
                    mx = tf["x"] + ox * c - oy * s
                    my = tf["y"] + ox * s + oy * c
                    myaw = tf["yaw"] + oyaw
                    mf = "map"
            self.state.update_costmap(costmap_type, {
                "image_bytes": enc.tobytes(),
                "width": w, "height": h, "resolution": res,
                "origin": [mx, my, 0.0],
                "origin_yaw": myaw, "frame_id": mf,
            })

    def _on_global_path(self, msg: NavPath) -> None:
        self.state.update_global_path(self._path_to_points(msg))

    def _on_local_path(self, msg: NavPath) -> None:
        self.state.update_local_path(self._path_to_points(msg))

    @staticmethod
    def _path_to_points(msg: NavPath) -> List[Dict[str, float]]:
        return [
            {
                "x": ps.pose.position.x, "y": ps.pose.position.y,
                "yaw": quaternion_to_yaw(
                    ps.pose.orientation.z, ps.pose.orientation.w
                ),
            }
            for ps in msg.poses
        ]

    def _on_robot_status(self, msg) -> None:
        self.state.update_robot_status({
            "control_mode": msg.modestamped.control_mode,
            "mode_type": msg.modestamped.mode_type,
            "mode_name": MODE_NAMES.get(
                (msg.modestamped.control_mode, msg.modestamped.mode_type), "?"
            ),
            "gait": msg.gaitstamped.gait,
            "gait_name": GAIT_NAMES.get(msg.gaitstamped.gait, "?"),
            "order_id": msg.orderstamped.id,
            "velocity": {
                "vx": round(msg.velocitystamped.linear_x, 4),
                "vy": round(msg.velocitystamped.linear_y, 4),
                "wz": round(msg.velocitystamped.angular_z, 4),
            },
            "pose": {
                "x": round(msg.posestamped.position_x, 4),
                "y": round(msg.posestamped.position_y, 4),
                "z": round(msg.posestamped.position_z, 4),
            },
            "foot_contact": msg.foot_contact
            if hasattr(msg, "foot_contact") else 0,
            "timestamp": time.time(),
        })

    # ── Robot control helpers ────────────────────────────────────────────────

    def _require_motion(self) -> None:
        if not HAS_MOTION_MSGS:
            raise HTTPException(
                status_code=501,
                detail="motion_msgs not available on this host",
            )

    def _send_mode(self, control_mode: int, mode_type: int) -> None:
        msg = ActionRequest()
        msg.type = ActionRequest.CHECKOUT_MODE
        msg.request_id = int(time.time() * 1000) % 2147483647
        msg.mode.control_mode = int(control_mode)
        msg.mode.mode_type = int(mode_type)
        msg.mode.timestamp = self.get_clock().now().to_msg()
        msg.timeout = 30
        self.action_pub.publish(msg)

    def _send_order(self, order_id: int) -> None:
        msg = ActionRequest()
        msg.type = ActionRequest.EXTMONORDER
        msg.request_id = int(time.time() * 1000) % 2147483647
        msg.order.id = int(order_id)
        msg.order.para = 0.0
        msg.order.timestamp = self.get_clock().now().to_msg()
        msg.timeout = 30
        self.action_pub.publish(msg)

    def _send_velocity(self, vx: float, vy: float, wz: float) -> None:
        msg = SE3VelocityCMD()
        msg.sourceid = SE3VelocityCMD.REMOTEC
        msg.velocity.frameid.id = Frameid.BODY_FRAME
        msg.velocity.timestamp = self.get_clock().now().to_msg()
        msg.velocity.linear_x = float(vx)
        msg.velocity.linear_y = float(vy)
        msg.velocity.linear_z = 0.0
        msg.velocity.angular_x = 0.0
        msg.velocity.angular_y = 0.0
        msg.velocity.angular_z = float(wz)
        self.body_cmd_pub.publish(msg)

    def _emergency_stop(self) -> None:
        for _ in range(5):
            self._send_velocity(0.0, 0.0, 0.0)
            time.sleep(0.05)

    def _set_gait_subprocess(self, gait: int) -> bool:
        ns = self.cyberdog_ns
        ts = int(time.time())
        cmd = (
            f"timeout 10 ros2 action send_goal /{ns}/checkout_gait "
            f"motion_msgs/action/ChangeGait "
            f"'{{motivation: 253, gaitstamped: "
            f"{{timestamp: {{sec: {ts}, nanosec: 0}}, gait: {gait}}}}}'"
        )
        r = subprocess.run(
            ["bash", "-c", cmd], capture_output=True, timeout=15
        )
        return r.returncode == 0

    def _camera_subprocess(self, enable: bool) -> bool:
        ns = self.cyberdog_ns
        val = "true" if enable else "false"
        cmd = (
            f"timeout 5 ros2 service call /{ns}/camera/enable "
            f"std_srvs/srv/SetBool '{{data: {val}}}'"
        )
        # 显式传入 DDS 环境，确保子进程能发现狗上的相机服务（跨机调用）
        env = os.environ.copy()
        try:
            pkg_dir = get_package_share_directory("cyberdog_web_bridge")
            env["CYCLONEDDS_URI"] = f"file://{os.path.join(pkg_dir, 'config', 'cyclonedds_bridge.xml')}"
        except Exception:
            pass  # 保留已有 CYCLONEDDS_URI 或默认
        r = subprocess.run(
            ["bash", "-c", cmd], capture_output=True, timeout=10, env=env
        )
        return r.returncode == 0

    def _empty_runtime_detector_payload(self) -> Dict[str, Any]:
        payload: Dict[str, Any] = {
            "success": False,
            "message": "",
            "desired": [],
            "running": [],
            "available": [],
            "unavailable": [],
            "failed": [],
        }
        return payload

    def _set_runtime_detectors_ws(self, detectors: list) -> Dict[str, Any]:
        payload = self._empty_runtime_detector_payload()
        payload["transport_ok"] = False

        if not HAS_WS_CLIENT:
            payload["message"] = "websocket-client unavailable"
            return payload

        request_id = "set_detectors_%d" % int(time.time() * 1000)
        ws = None
        try:
            ws = websocket.create_connection(self.vision_ws_url, timeout=8)
            ws.send(
                json.dumps(
                    {
                        "action": "set_detectors",
                        "detectors": [str(d) for d in detectors],
                        "request_id": request_id,
                    },
                    ensure_ascii=True,
                )
            )

            deadline = time.time() + 10.0
            while time.time() < deadline:
                raw = ws.recv()
                if not raw:
                    continue
                try:
                    msg = json.loads(raw)
                except json.JSONDecodeError:
                    continue

                msg_type = msg.get("type")
                if msg_type == "set_detectors_result" and msg.get("request_id") == request_id:
                    data = msg.get("data", {})
                    for field in ("success", "message", "desired", "running", "available", "unavailable", "failed"):
                        if field in data:
                            payload[field] = data[field]
                    payload["transport_ok"] = True
                    break

                # Reuse the existing WS message handler so runtime/status state stays fresh.
                self._on_vision_ws_message(raw)

            if not payload["transport_ok"]:
                payload["message"] = "Timed out waiting for runtime toggle result"
        except Exception as exc:
            payload["message"] = "Vision WS request failed: %s" % exc
        finally:
            if ws is not None:
                try:
                    ws.close()
                except Exception:
                    pass

        self.get_logger().info(
            "RuntimeSetDetectors via WS transport_ok=%s success=%s running=%s failed=%s"
            % (
                payload.get("transport_ok"),
                payload.get("success"),
                payload.get("running"),
                payload.get("failed"),
            )
        )
        return payload

    # ── Background tasks ─────────────────────────────────────────────────────

    def _run_prepare(self, gait: int = 8, enable_cam: bool = True) -> None:
        tid = "prepare"
        try:
            self.state.update_task(tid, {
                "running": True, "step": "切换手动模式", "progress": 10,
            })
            self._send_mode(3, 0)
            time.sleep(1.5)

            self.state.update_task(tid, {
                "running": True, "step": "站立", "progress": 30,
            })
            self._send_order(9)
            time.sleep(2.5)

            self.state.update_task(tid, {
                "running": True, "step": "切换TROT步态", "progress": 55,
            })
            self._set_gait_subprocess(gait)
            time.sleep(2.0)

            if enable_cam:
                self.state.update_task(tid, {
                    "running": True, "step": "启用相机", "progress": 80,
                })
                self._camera_subprocess(True)
                time.sleep(1.5)

            self.state.update_task(tid, {
                "running": False, "step": "完成", "progress": 100,
            })
        except Exception as e:
            self.state.update_task(tid, {
                "running": False, "step": "失败", "progress": 0,
                "error": str(e),
            })

    def _resolve_map_filename(self, requested_filename: str) -> str:
        """Resolve a map stem and keep it inside the configured output directory."""
        base_dir = os.path.abspath(os.path.expanduser(self.map_output_dir))
        requested = requested_filename.strip() or "cyberdog_map"
        candidate = os.path.expanduser(requested)
        if not os.path.isabs(candidate):
            candidate = os.path.join(base_dir, candidate)
        candidate = os.path.abspath(candidate)

        try:
            inside_output_dir = os.path.commonpath([base_dir, candidate]) == base_dir
        except ValueError as exc:
            raise ValueError("invalid map output path") from exc
        if not inside_output_dir:
            raise ValueError("map output must stay inside map_output_dir")

        for suffix in (".pbstream", ".yaml", ".pgm"):
            if candidate.endswith(suffix):
                candidate = candidate[: -len(suffix)]
                break
        return candidate

    def _run_save_map(self, filename: str, export_pgm: bool) -> None:
        tid = "save_map"
        try:
            filename = self._resolve_map_filename(filename)
            os.makedirs(os.path.dirname(filename) or ".", exist_ok=True)
            self.state.update_task(tid, {
                "running": True, "step": "结束轨迹", "progress": 20,
            })
            subprocess.run(
                [
                    "timeout", "10", "ros2", "service", "call",
                    "/finish_trajectory",
                    "cartographer_ros_msgs/srv/FinishTrajectory",
                    "{trajectory_id: 0}",
                ],
                capture_output=True, timeout=15,
            )
            time.sleep(1.0)

            pbstream = f"{filename}.pbstream"
            self.state.update_task(tid, {
                "running": True, "step": "保存pbstream", "progress": 50,
            })
            write_state_request = (
                f'{{filename: "{pbstream}", include_unfinished_submaps: true}}'
            )
            subprocess.run(
                [
                    "timeout", "15", "ros2", "service", "call",
                    "/write_state", "cartographer_ros_msgs/srv/WriteState",
                    write_state_request,
                ],
                capture_output=True, timeout=20,
            )
            time.sleep(1.0)

            if export_pgm:
                self.state.update_task(tid, {
                    "running": True, "step": "导出pgm/yaml", "progress": 80,
                })
                subprocess.run(
                    [
                        "timeout", "20", "ros2", "run", "cartographer_ros",
                        "cartographer_pbstream_to_ros_map",
                        f"-pbstream_filename={pbstream}",
                        f"-map_filestem={filename}",
                        "-resolution=0.05",
                    ],
                    capture_output=True, timeout=25,
                )

            self.state.update_task(tid, {
                "running": False, "step": "完成",
                "progress": 100, "filename": filename,
            })
        except Exception as e:
            self.state.update_task(tid, {
                "running": False, "step": "失败", "progress": 0,
                "error": str(e),
            })

    # ── Service launching ────────────────────────────────────────────────────

    def _build_nav2_env(self) -> Dict[str, str]:
        env = os.environ.copy()
        xml = self.nav2_cyclonedds_xml
        if xml and os.path.isfile(xml):
            env["CYCLONEDDS_URI"] = f"file://{xml}"
        return env

    def _start_nav2(self, mode: str, pbstream_file: str = "",
                    map_yaml: str = "") -> Dict[str, Any]:
        existing = self.service_manager.get_service_status("nav2")
        if existing:
            existing_mode = existing.get("mode", "")
            if existing_mode == mode:
                return {
                    "pid": existing["pid"],
                    "name": "nav2",
                    "mode": mode,
                    "already_running": True,
                }
            mode_hint = f" in {existing_mode} mode" if existing_mode else ""
            raise RuntimeError(
                f"nav2 already running{mode_hint} (pid={existing['pid']}), "
                f"stop it before starting {mode}"
            )

        cmd = [
            "ros2", "launch", "cyberdog_nav2_lidar", "bringup.launch.py",
            f"mode:={mode}",
        ]
        if pbstream_file:
            cmd.append(f"pbstream_file:={pbstream_file}")
        if map_yaml:
            cmd.append(f"map_yaml:={map_yaml}")
        started = self.service_manager.start(
            "nav2", cmd, env=self._build_nav2_env(),
            extra_info={"mode": mode},
        )
        started["mode"] = mode
        started["already_running"] = False
        return started

    def _start_inspection_service(self) -> Dict[str, Any]:
        config = os.path.join(
            self.workspace_dir,
            "src/cyberdog_inspection/config/inspection_config.yaml",
        )
        cmd = [
            "ros2", "launch", "cyberdog_inspection",
            "cyberdog_inspection.launch.py", f"config:={config}",
        ]
        return self.service_manager.start("inspection", cmd)

    def _start_voice_brain(self) -> Dict[str, Any]:
        script = os.path.join(self.workspace_dir, "src/voice_brain/start.sh")
        cmd = ["bash", script]
        return self.service_manager.start("voice_brain", cmd)

    # ── Existing helpers ─────────────────────────────────────────────────────

    def _publish_initial_pose(self, req: SetInitialPoseRequest) -> None:
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.pose.pose.position.x = req.x
        msg.pose.pose.position.y = req.y
        msg.pose.pose.orientation.z = math.sin(req.yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(req.yaw / 2.0)
        msg.pose.covariance[0] = req.covariance_xy
        msg.pose.covariance[7] = req.covariance_xy
        msg.pose.covariance[35] = req.covariance_yaw
        self.initial_pose_publisher.publish(msg)

    def _publish_navigation_goal(self, req: SetNavigationGoalRequest) -> None:
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = req.x
        msg.pose.position.y = req.y
        msg.pose.orientation.z = math.sin(req.yaw / 2.0)
        msg.pose.orientation.w = math.cos(req.yaw / 2.0)
        self.navigation_goal_publisher.publish(msg)
        self.navigation_goal_ns_publisher.publish(msg)

    def _call_inspection(self, fn):
        try:
            return fn()
        except RuntimeError as exc:
            raise HTTPException(status_code=503, detail=str(exc)) from exc

    def _wrap(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        return {
            "success": bool(payload.get("success", False)),
            "message": payload.get("message", ""),
            "data": payload,
        }

    def _get_map_info(self, scene_name: str) -> Dict[str, Any]:
        payload = self._call_inspection(self.inspection_client.get_maps)
        for item in payload.get("maps", []):
            if item.get("scene_name") == scene_name:
                if not item.get("available", False):
                    raise HTTPException(
                        status_code=400,
                        detail=item.get("error_message", "Map unavailable"),
                    )
                return item
        raise HTTPException(status_code=404, detail=f"Unknown scene: {scene_name}")

    def _decorate_map_info(self, info: Dict[str, Any]) -> Dict[str, Any]:
        o = info.get("origin", [0.0, 0.0, 0.0])
        r = float(info.get("resolution", 0.0))
        w, h = int(info.get("width", 0)), int(info.get("height", 0))
        return {
            **info,
            "image_url": f"/api/v1/maps/{info['scene_name']}/image",
            "metadata_url": f"/api/v1/maps/{info['scene_name']}/metadata",
            "world_bounds": {
                "min_x": o[0], "min_y": o[1],
                "max_x": o[0] + w * r, "max_y": o[1] + h * r,
            },
        }

    def _build_live_payload(self) -> Dict[str, Any]:
        snap = self.state.snapshot()
        return {
            "robot_pose": snap["robot_pose"],
            "global_path": snap["global_path"],
            "local_path": snap["local_path"],
            "inspection_status": snap["inspection_status"],
            "planned_path": snap["planned_path"],
            "map_artifacts": snap["map_artifacts"],
            "live_map_version": snap["live_map_version"],
            "robot_status": snap["robot_status"],
            "service_status": snap["service_status"],
            "tasks": snap["tasks"],
            "sensor_data": snap["sensor_data"],
            "vision_data": snap["vision_data"],
            "timestamp": time.time(),
        }

    # ── FastAPI app ──────────────────────────────────────────────────────────

    def _create_app(self) -> FastAPI:
        app = FastAPI(title="CyberDog Web Console", version="1.0.0")
        app.add_middleware(
            CORSMiddleware, allow_origins=self.cors_origins, allow_credentials=True,
            allow_methods=["*"], allow_headers=["*"],
        )
        app.mount(
            "/static",
            StaticFiles(directory=str(self.static_dir), follow_symlink=True),
            name="static",
        )

        @app.get("/")
        async def root():
            return FileResponse(self.static_dir / "index.html")

        # ── Health ───────────────────────────────────────────────────

        @app.get("/api/v1/health")
        async def health():
            snap = self.state.snapshot()
            return {
                "success": True, "data": {
                    "http_port": self.http_port,
                    "has_motion_msgs": HAS_MOTION_MSGS,
                    "last_ros_update": snap["last_ros_update"],
                    "live_map_version": snap["live_map_version"],
                },
            }

        # ── Sensor callback ──────────────────────────────────────────

        @app.post("/api/v1/sensor/callback")
        async def sensor_callback(request_data: dict):
            self.state.update_sensor_data(request_data)
            return {"success": True}

        # ── Maps (inspection) ────────────────────────────────────────

        @app.get("/api/v1/maps")
        async def maps():
            payload = self._call_inspection(self.inspection_client.get_maps)
            d = [self._decorate_map_info(i) for i in payload.get("maps", [])]
            return {"success": True, "data": {"maps": d}}

        @app.get("/api/v1/maps/{scene_name}/metadata")
        async def map_metadata(scene_name: str):
            return {
                "success": True,
                "data": self._decorate_map_info(self._get_map_info(scene_name)),
            }

        @app.get("/api/v1/maps/{scene_name}/image")
        async def map_image(scene_name: str):
            info = self._get_map_info(scene_name)
            image = cv2.imread(info["pgm_path"], cv2.IMREAD_GRAYSCALE)
            if image is None:
                raise HTTPException(404, "Map image not found")
            h, w = image.shape
            rgba = np.zeros((h, w, 4), dtype=np.uint8)
            rgba[image >= 250] = [255, 255, 255, 0]
            rgba[image <= 10] = [31, 29, 29, 255]
            rgba[(image > 10) & (image < 250)] = [245, 245, 247, 0]
            ok, enc = cv2.imencode(".png", rgba)
            if not ok:
                raise HTTPException(500, "Encode failed")
            return Response(content=enc.tobytes(), media_type="image/png")

        # ── Live map ─────────────────────────────────────────────────

        @app.get("/api/v1/live-map/metadata")
        async def live_map_metadata():
            lm = self.state._live_map
            if not lm:
                raise HTTPException(404, "Live map not available")
            return {"success": True, "data": {
                "width": lm["width"], "height": lm["height"],
                "resolution": lm["resolution"], "origin": lm["origin"],
                "version": self.state._live_map_version,
            }}

        @app.get("/api/v1/live-map/image")
        async def live_map_image():
            lm = self.state._live_map
            if not lm:
                raise HTTPException(404, "Live map not available")
            return Response(content=lm["image_bytes"], media_type="image/png")

        # ── Costmaps ─────────────────────────────────────────────────

        @app.get("/api/v1/costmap/{ct}/metadata")
        async def costmap_metadata(ct: str):
            cm = self.state._global_costmap if ct == "global" \
                else self.state._local_costmap
            if not cm:
                raise HTTPException(404, f"{ct} costmap not ready")
            return {"success": True, "data": {
                "width": cm["width"], "height": cm["height"],
                "resolution": cm["resolution"], "origin": cm["origin"],
                "origin_yaw": cm.get("origin_yaw", 0.0),
                "frame_id": cm.get("frame_id", "map"),
            }}

        @app.get("/api/v1/costmap/{ct}/image")
        async def costmap_image(ct: str):
            cm = self.state._global_costmap if ct == "global" \
                else self.state._local_costmap
            if not cm:
                raise HTTPException(404, f"{ct} costmap not ready")
            return Response(content=cm["image_bytes"], media_type="image/png")

        # ── Inspection ───────────────────────────────────────────────

        @app.get("/api/v1/inspection/status")
        async def inspection_status():
            return {
                "success": True,
                "data": self.state.snapshot()["inspection_status"],
            }

        @app.get("/api/v1/inspection/planned-path")
        async def inspection_planned_path():
            s = self.state.snapshot()
            return {"success": True, "data": {
                "active_scene_name": s["inspection_status"].get(
                    "active_scene_name", ""
                ),
                "planned_path": s["planned_path"],
                "map_artifacts": s["map_artifacts"],
            }}

        @app.post("/api/v1/inspection/initialize")
        async def inspection_initialize(req: InitializeInspectionRequest):
            return self._wrap(self._call_inspection(
                lambda: self.inspection_client.start_initialization(
                    req.scene_name
                )
            ))

        @app.post("/api/v1/inspection/start")
        async def inspection_start():
            return self._wrap(
                self._call_inspection(self.inspection_client.start)
            )

        @app.post("/api/v1/inspection/pause")
        async def inspection_pause():
            return self._wrap(
                self._call_inspection(self.inspection_client.pause)
            )

        @app.post("/api/v1/inspection/resume")
        async def inspection_resume():
            return self._wrap(
                self._call_inspection(self.inspection_client.resume)
            )

        @app.post("/api/v1/inspection/stop")
        async def inspection_stop():
            return self._wrap(
                self._call_inspection(self.inspection_client.stop)
            )

        # ── Localization / Navigation ────────────────────────────────

        @app.post("/api/v1/localization/initial-pose")
        async def set_initial_pose(req: SetInitialPoseRequest):
            self._publish_initial_pose(req)
            return {
                "success": True, "message": "Initial pose published",
                "data": req.model_dump(),
            }

        @app.post("/api/v1/navigation/goal")
        async def set_nav_goal(req: SetNavigationGoalRequest):
            self._publish_navigation_goal(req)
            return {
                "success": True, "message": "Nav goal published",
                "data": req.model_dump(),
            }

        @app.post("/api/v1/navigation/cancel")
        def cancel_navigation():
            """取消当前导航目标（通过 ros2 action cancel）"""
            try:
                result = subprocess.run(
                    ["ros2", "action", "cancel", "/navigate_to_pose"],
                    capture_output=True, text=True, timeout=5,
                )
                if result.returncode == 0:
                    return {"success": True, "message": "Navigation cancelled"}
                # 无目标时 cancel 可能返回非0，仍视为成功
                return {"success": True, "message": "Cancel request sent"}
            except subprocess.TimeoutExpired:
                raise HTTPException(504, "Cancel timeout")
            except FileNotFoundError:
                raise HTTPException(503, "ros2 CLI not found")

        # ── Service management ───────────────────────────────────────

        @app.get("/api/v1/services/status")
        async def services_status():
            return {
                "success": True,
                "data": self.service_manager.get_status(),
            }

        @app.post("/api/v1/services/nav2/start")
        def nav2_start(req: StartNav2Request):
            try:
                result = self._start_nav2(
                    req.mode, req.pbstream_file, req.map_yaml,
                )
                return {"success": True, "data": result}
            except RuntimeError as e:
                raise HTTPException(409, str(e))

        @app.post("/api/v1/services/nav2/stop")
        def nav2_stop():
            self.service_manager.stop("nav2")
            return {"success": True, "message": "Nav2 stopped"}

        @app.post("/api/v1/services/inspection/start")
        def inspection_svc_start():
            try:
                result = self._start_inspection_service()
                return {"success": True, "data": result}
            except RuntimeError as e:
                raise HTTPException(409, str(e))

        @app.post("/api/v1/services/inspection/stop")
        def inspection_svc_stop():
            self.service_manager.stop("inspection")
            return {"success": True, "message": "Inspection service stopped"}

        @app.post("/api/v1/services/voice-brain/start")
        def voice_brain_start():
            try:
                result = self._start_voice_brain()
                return {"success": True, "data": result}
            except RuntimeError as e:
                raise HTTPException(409, str(e))

        @app.post("/api/v1/services/voice-brain/stop")
        def voice_brain_stop():
            self.service_manager.stop("voice_brain")
            return {"success": True, "message": "Voice brain stopped"}

        @app.post("/api/v1/services/stop-all")
        def stop_all_services():
            self.service_manager.stop_all()
            return {"success": True, "message": "All services stopped"}

        # ── Map saving ───────────────────────────────────────────────

        @app.post("/api/v1/mapping/save")
        def save_map(req: SaveMapRequest):
            task = self.state.get_task("save_map")
            if task.get("running"):
                raise HTTPException(409, "Save already in progress")
            threading.Thread(
                target=self._run_save_map,
                args=(req.filename, req.export_pgm),
                daemon=True,
            ).start()
            return {"success": True, "message": "Map save started"}

        @app.get("/api/v1/mapping/save/status")
        async def save_map_status():
            return {"success": True, "data": self.state.get_task("save_map")}

        # ── Robot control ────────────────────────────────────────────

        @app.get("/api/v1/robot/status")
        async def robot_status():
            return {
                "success": True,
                "data": self.state.snapshot()["robot_status"],
            }

        @app.post("/api/v1/robot/mode")
        def robot_mode(req: RobotModeRequest):
            self._require_motion()
            if req.preset and req.preset in MODE_PRESETS:
                cm, mt = MODE_PRESETS[req.preset]
            elif req.control_mode >= 0:
                cm, mt = req.control_mode, max(0, req.mode_type)
            else:
                raise HTTPException(400, "Provide preset name or control_mode")
            self._send_mode(cm, mt)
            return {
                "success": True,
                "message": f"Mode → {MODE_NAMES.get((cm, mt), f'{cm},{mt}')}",
            }

        @app.post("/api/v1/robot/order")
        def robot_order(req: RobotOrderRequest):
            self._require_motion()
            if req.preset and req.preset in ORDER_PRESETS:
                oid = ORDER_PRESETS[req.preset]
            elif req.order_id >= 0:
                oid = req.order_id
            else:
                raise HTTPException(400, "Provide preset name or order_id")
            self._send_order(oid)
            return {"success": True, "message": f"Order → {oid}"}

        @app.post("/api/v1/robot/gait")
        def robot_gait(req: RobotGaitRequest):
            self._require_motion()
            ok = self._set_gait_subprocess(req.gait)
            name = GAIT_NAMES.get(req.gait, str(req.gait))
            return {
                "success": ok,
                "message": f"Gait → {name}" if ok else "Gait change failed",
            }

        @app.post("/api/v1/robot/velocity")
        def robot_velocity(req: RobotVelocityRequest):
            self._require_motion()
            self._send_velocity(req.vx, req.vy, req.wz)
            return {"success": True}

        @app.post("/api/v1/robot/stop")
        def robot_stop():
            self._require_motion()
            self._emergency_stop()
            return {"success": True, "message": "Emergency stop sent"}

        @app.post("/api/v1/robot/prepare")
        def robot_prepare():
            self._require_motion()
            task = self.state.get_task("prepare")
            if task.get("running"):
                raise HTTPException(409, "Prepare already running")
            threading.Thread(
                target=self._run_prepare, daemon=True,
            ).start()
            return {"success": True, "message": "Prepare sequence started"}

        @app.get("/api/v1/robot/prepare/status")
        async def prepare_status():
            return {"success": True, "data": self.state.get_task("prepare")}

        @app.post("/api/v1/robot/camera")
        def robot_camera(req: CameraRequest):
            ok = self._camera_subprocess(req.enable)
            action = "启用" if req.enable else "禁用"
            return {
                "success": ok,
                "message": f"相机{action}{'成功' if ok else '失败'}",
            }

        @app.post("/api/v1/vision/detectors")
        def set_vision_detectors(req: SetDetectorsRequest):
            payload = self._set_runtime_detectors_ws(req.detectors)
            if not payload.get("transport_ok"):
                raise HTTPException(
                    503,
                    detail=str(payload.get("message") or "Runtime WS request failed"),
                )
            if not payload.get("success"):
                raise HTTPException(
                    409,
                    detail=str(payload.get("message") or "Runtime detector update failed"),
                )
            self.state.clear_vision_detections()
            return {
                "success": True,
                "message": payload.get("message", "检测器已更新"),
                "data": payload,
            }

        # ── Vision Frame ──────────────────────────────────────────────

        @app.get("/api/v1/vision/frame")
        def get_vision_frame():
            frame = self.state.get_frame()
            if frame is None:
                raise HTTPException(404, "No frame available")
            return Response(content=frame, media_type="image/jpeg")

        # ── Meter Points ───────────────────────────────────────────────

        @app.get("/api/v1/meter-points")
        def list_meter_points():
            data = self._load_meter_points()
            return {"success": True, "data": data}

        @app.post("/api/v1/meter-points/record")
        def record_meter_point(req: RecordMeterPointRequest):
            point = self._record_meter_point(req)
            return {"success": True, "message": "点位已记录", "data": point}

        @app.post("/api/v1/meter-points/rename")
        def rename_meter_point(req: RenameMeterPointRequest):
            data = self._load_meter_points()
            points = data.get("points", [])
            updated = None
            for index, point in enumerate(points):
                if str(point.get("point_id", "")) == req.point_id:
                    point["name"] = req.name.strip()
                    updated = self._normalize_meter_point(point)
                    points[index] = updated
                    break
            if updated is None:
                raise HTTPException(404, "Point not found")
            self._save_meter_points({"points": points})
            return {"success": True, "message": "点位已重命名", "data": updated}

        @app.delete("/api/v1/meter-points/{point_id}")
        def delete_meter_point(point_id: str):
            data = self._load_meter_points()
            points = data.get("points", [])
            kept = [point for point in points if str(point.get("point_id", "")) != point_id]
            if len(kept) == len(points):
                raise HTTPException(404, "Point not found")
            self._save_meter_points({"points": kept})
            return {"success": True, "message": "点位已删除", "data": {"point_id": point_id}}

        @app.get("/api/v1/meter-history")
        def list_meter_history(limit: int = 50):
            records = self.state.get_meter_history(limit)
            return {"success": True, "data": {"records": records}}

        @app.post("/api/v1/meter-history/record")
        def record_meter_history(req: RecordMeterReadingRequest):
            record = self._record_meter_reading(req)
            return {"success": True, "message": "表盘读数已记录", "data": record}

        @app.delete("/api/v1/meter-history/{record_id}")
        def delete_meter_history(record_id: str):
            if not re.match(r"^meter_[\w\-]+$", record_id):
                raise HTTPException(400, "Invalid meter record ID")
            removed = self.state.delete_meter_record(record_id)
            if not removed:
                raise HTTPException(404, "Meter record not found")
            self._persist_meter_history()
            return {"success": True, "message": "表盘记录已删除", "data": {"id": record_id}}

        # ── Alarm History ─────────────────────────────────────────────

        @app.get("/api/v1/vision/alarms")
        def list_alarms(limit: int = 50):
            """Return list of alarm events, newest first."""
            alarms = self.state.get_alarm_history(limit)
            return {"success": True, "data": {"alarms": alarms}}

        @app.get("/api/v1/vision/alarms/{alarm_id}")
        def get_alarm_image(alarm_id: str):
            """Serve an alarm image by ID."""
            import re
            if not re.match(r'^alarm_[\w\-]+$', alarm_id):
                raise HTTPException(400, "Invalid alarm ID")
            img_path = os.path.join(self.alarm_history_dir, f"{alarm_id}.jpg")
            if not os.path.isfile(img_path):
                raise HTTPException(404, "Alarm image not found")
            return FileResponse(img_path, media_type="image/jpeg")

        # ── Live WebSocket ───────────────────────────────────────────

        @app.get("/api/v1/live")
        async def live_snapshot():
            return {"success": True, "data": self._build_live_payload()}

        @app.websocket("/ws/live")
        async def live_ws(ws: WebSocket):
            await ws.accept()
            try:
                while True:
                    await ws.send_json(self._build_live_payload())
                    await asyncio.sleep(self.ws_interval_sec)
            except WebSocketDisconnect:
                return

        return app


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = CyberdogWebBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.service_manager.stop_all()
        node.destroy_node()
        rclpy.shutdown()
