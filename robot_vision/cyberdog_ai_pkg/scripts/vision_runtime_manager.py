#!/usr/bin/env python3

import json
import os
import signal
import subprocess
import threading
import time
from collections import deque
from typing import Any, Deque, Dict, List, Optional, Sequence, Set

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from cyberdog_ai_pkg.srv import RuntimeSetDetectors
from cyberdog_ai_pkg.srv import SetDetectors


class DetectorSpec:
    def __init__(
        self,
        detector_id: str,
        topic: str,
        command: List[str],
        startup_wait_sec: float,
        engine_rel: str = "",
    ) -> None:
        self.detector_id = detector_id
        self.topic = topic
        self.command = command
        self.startup_wait_sec = startup_wait_sec
        self.engine_rel = engine_rel


class DetectorRuntime:
    def __init__(self) -> None:
        self.process: Optional[subprocess.Popen] = None
        self.started_at = 0.0
        self.last_error = ""
        self.last_exit_code: Optional[int] = None
        self.recent_output: Deque[str] = deque(maxlen=40)


class VisionRuntimeManager(Node):
    def __init__(self) -> None:
        super().__init__("vision_runtime_manager")

        self.declare_parameter("pkg_install_share_dir", "")
        self.declare_parameter("workspace_dir", "")
        self.declare_parameter("pkg_install_lib_dir", "")
        self.declare_parameter("pkg_install_libexec_dir", "")
        self.declare_parameter("image_topic", "/mi1035085/camera/color/image_raw")
        self.declare_parameter("body_topic", "/mi1035085/body")
        self.declare_parameter("fire_topic", "/mi1035085/fire_alarm")
        self.declare_parameter("fall_topic", "/mi1035085/fall_alarm")
        self.declare_parameter("hat_topic", "/mi1035085/hat_alarm")
        self.declare_parameter("gather_topic", "/mi1035085/gather_alarm")
        self.declare_parameter("meter_topic", "/mi1035085/meter")
        self.declare_parameter("depth_image_topic", "/mi1035085/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter("camera_info_topic", "/mi1035085/camera/color/camera_info")
        self.declare_parameter("meter_process_every_n_frames", 3)
        self.declare_parameter("body_process_every_n_frames", 3)
        self.declare_parameter("runtime_status_topic", "/vision/runtime_status")
        self.declare_parameter("static_enabled_detectors", [])
        self.declare_parameter("sync_period_sec", 1.0)

        self.pkg_install_share_dir = str(self.get_parameter("pkg_install_share_dir").value).strip()
        self.workspace_dir = str(self.get_parameter("workspace_dir").value).strip()
        self.pkg_install_lib_dir = str(self.get_parameter("pkg_install_lib_dir").value).strip()
        self.pkg_install_libexec_dir = str(self.get_parameter("pkg_install_libexec_dir").value).strip()
        self.image_topic = str(self.get_parameter("image_topic").value)
        self.body_topic = str(self.get_parameter("body_topic").value)
        self.fire_topic = str(self.get_parameter("fire_topic").value)
        self.fall_topic = str(self.get_parameter("fall_topic").value)
        self.hat_topic = str(self.get_parameter("hat_topic").value)
        self.gather_topic = str(self.get_parameter("gather_topic").value)
        self.meter_topic = str(self.get_parameter("meter_topic").value)
        self.depth_image_topic = str(self.get_parameter("depth_image_topic").value)
        self.camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        self.meter_process_every_n_frames = max(
            1, int(self.get_parameter("meter_process_every_n_frames").value)
        )
        self.body_process_every_n_frames = max(
            1, int(self.get_parameter("body_process_every_n_frames").value)
        )
        self.runtime_status_topic = str(self.get_parameter("runtime_status_topic").value)
        self.static_enabled_detectors = self._normalize_detectors(
            self.get_parameter("static_enabled_detectors").value
        )
        sync_period_sec = max(0.2, float(self.get_parameter("sync_period_sec").value))

        self._lock = threading.RLock()
        self._desired_detectors: Set[str] = set()
        self._aggregator_sync_requested = True
        self._aggregator_sync_inflight = False
        self._last_synced_aggregator: Optional[List[str]] = None
        self._base_env = self._build_base_env()

        self.detector_specs = self._build_detector_specs()
        self.detector_order = list(self.detector_specs.keys())
        self.detector_runtime = {
            detector_id: DetectorRuntime() for detector_id in self.detector_order
        }

        self.runtime_pub = self.create_publisher(String, self.runtime_status_topic, 10)
        self.aggregator_client = self.create_client(SetDetectors, "/vision/set_detectors")
        self.create_service(
            RuntimeSetDetectors,
            "/vision/runtime/set_detectors",
            self.handle_set_detectors,
        )
        self.status_timer = self.create_timer(sync_period_sec, self._on_runtime_timer)

        self.get_logger().info(
            "vision_runtime_manager ready: detectors=%s static_enabled=%s"
            % (",".join(self.detector_order), ",".join(self.static_enabled_detectors))
        )
        self._publish_runtime_status()

    def _normalize_detector_name(self, value: Any) -> str:
        return str(value).strip().lower()

    def _normalize_detectors(self, values: Any) -> List[str]:
        if values is None:
            return []
        if isinstance(values, str):
            values = values.strip().strip("[]")
            if not values:
                return []
            values = [item.strip().strip("'\"") for item in values.split(",")]

        normalized: List[str] = []
        for value in values:
            text = self._normalize_detector_name(value)
            if text and text not in normalized:
                normalized.append(text)
        return normalized

    def _engine_path(self, engine_rel: str) -> str:
        if not engine_rel:
            return ""
        if os.path.isabs(engine_rel):
            return engine_rel
        if not self.pkg_install_share_dir:
            return engine_rel
        return os.path.join(self.pkg_install_share_dir, engine_rel)

    def _prepend_env_path(self, env: Dict[str, str], var_name: str, value: str) -> None:
        if not value or not os.path.isdir(value):
            return
        current = env.get(var_name, "")
        parts = [item for item in current.split(":") if item]
        if value in parts:
            return
        env[var_name] = ":".join([value] + parts) if parts else value

    def _build_base_env(self) -> Dict[str, str]:
        env = os.environ.copy()
        install_prefix = ""
        if self.workspace_dir:
            install_prefix = os.path.join(self.workspace_dir, "install")
        if install_prefix:
            self._prepend_env_path(env, "COLCON_PREFIX_PATH", install_prefix)
            self._prepend_env_path(env, "AMENT_PREFIX_PATH", install_prefix)
            self._prepend_env_path(env, "CMAKE_PREFIX_PATH", install_prefix)
        self._prepend_env_path(env, "LD_LIBRARY_PATH", self.pkg_install_lib_dir)
        self._prepend_env_path(env, "PATH", self.pkg_install_libexec_dir)

        python_candidates = []
        if self.workspace_dir:
            python_candidates.extend(
                [
                    os.path.join(
                        self.workspace_dir,
                        "install/cyberdog_ai_pkg/local/lib/python3.10/dist-packages",
                    ),
                    os.path.join(
                        self.workspace_dir,
                        "install/cyberdog_ai_pkg/local/lib/python3.8/dist-packages",
                    ),
                ]
            )
        for candidate in python_candidates:
            self._prepend_env_path(env, "PYTHONPATH", candidate)
        return env

    def _pkg_exec_cmd(self, exec_name: str) -> List[str]:
        if self.pkg_install_libexec_dir:
            candidate = os.path.join(self.pkg_install_libexec_dir, exec_name)
            if os.path.exists(candidate):
                return [candidate]
        return ["ros2", "run", "cyberdog_ai_pkg", exec_name]

    def _engine_ready(self, detector_id: str) -> bool:
        spec = self.detector_specs[detector_id]
        if not spec.engine_rel:
            return True
        return os.path.isfile(self._engine_path(spec.engine_rel))

    def _available_detectors(self) -> List[str]:
        return [detector_id for detector_id in self.detector_order if self._engine_ready(detector_id)]

    def _unavailable_detectors(self) -> List[str]:
        return [detector_id for detector_id in self.detector_order if not self._engine_ready(detector_id)]

    def _build_detector_specs(self) -> Dict[str, DetectorSpec]:
        return {
            "person": DetectorSpec(
                detector_id="person",
                topic=self.body_topic,
                command=self._pkg_exec_cmd("body_detector_node") + [
                    "--ros-args",
                    "-p",
                    f"input_image_topic:={self.image_topic}",
                    "-p",
                    f"output_body_topic:={self.body_topic}",
                    "-p",
                    "publish_features:=false",
                    "-p",
                    f"process_every_n_frames:={self.body_process_every_n_frames}",
                ],
                startup_wait_sec=1.0,
            ),
            "fire_alarm": DetectorSpec(
                detector_id="fire_alarm",
                topic=self.fire_topic,
                command=self._pkg_exec_cmd("fire_detector_node") + [
                    "--ros-args",
                    "-p",
                    f"input_image_topic:={self.image_topic}",
                    "-p",
                    f"output_detection_topic:={self.fire_topic}",
                    "-p",
                    "detector_name:=fire_alarm",
                    "-p",
                    "process_every_n_frames:=2",
                ],
                startup_wait_sec=3.0,
                engine_rel="assets/onnx/fire_250527.engine",
            ),
            "fall_alarm": DetectorSpec(
                detector_id="fall_alarm",
                topic=self.fall_topic,
                command=self._pkg_exec_cmd("fire_detector_node") + [
                    "--ros-args",
                    "-r",
                    "__node:=fall_detector",
                    "-p",
                    f"input_image_topic:={self.image_topic}",
                    "-p",
                    f"output_detection_topic:={self.fall_topic}",
                    "-p",
                    "detector_name:=fall_alarm",
                    "-p",
                    "config_path:=assets/configs/fall.json",
                    "-p",
                    "process_every_n_frames:=2",
                ],
                startup_wait_sec=3.0,
                engine_rel="assets/onnx/fall_250325.engine",
            ),
            "hat_alarm": DetectorSpec(
                detector_id="hat_alarm",
                topic=self.hat_topic,
                command=self._pkg_exec_cmd("fire_detector_node") + [
                    "--ros-args",
                    "-r",
                    "__node:=hat_detector",
                    "-p",
                    f"input_image_topic:={self.image_topic}",
                    "-p",
                    f"output_detection_topic:={self.hat_topic}",
                    "-p",
                    "detector_name:=hat_alarm",
                    "-p",
                    "config_path:=assets/configs/hat.json",
                    "-p",
                    "process_every_n_frames:=2",
                ],
                startup_wait_sec=3.0,
                engine_rel="assets/onnx/hat_250613.engine",
            ),
            "gather_alarm": DetectorSpec(
                detector_id="gather_alarm",
                topic=self.gather_topic,
                command=self._pkg_exec_cmd("gather_detector_node") + [
                    "--ros-args",
                    "-p",
                    f"body_topic:={self.body_topic}",
                    "-p",
                    f"output_detection_topic:={self.gather_topic}",
                    "-p",
                    "detector_name:=gather_alarm",
                    "-p",
                    "process_every_n_frames:=3",
                    "-p",
                    "log_detection_summary:=true",
                ],
                startup_wait_sec=1.0,
                engine_rel=None,
            ),
            "meter": DetectorSpec(
                detector_id="meter",
                topic=self.meter_topic,
                command=self._pkg_exec_cmd("meter_reading_node") + [
                    "--ros-args",
                    "-p",
                    f"input_image_topic:={self.image_topic}",
                    "-p",
                    f"depth_image_topic:={self.depth_image_topic}",
                    "-p",
                    f"camera_info_topic:={self.camera_info_topic}",
                    "-p",
                    f"output_meter_topic:={self.meter_topic}",
                    "-p",
                    f"process_every_n_frames:={self.meter_process_every_n_frames}",
                    "-p",
                    "log_detection_summary:=true",
                ],
                startup_wait_sec=3.0,
                engine_rel="assets/onnx/meter_detect.engine",
            ),
        }

    def _ordered(self, detectors: Sequence[str]) -> List[str]:
        selected = {self._normalize_detector_name(detector) for detector in detectors}
        return [detector_id for detector_id in self.detector_order if detector_id in selected]

    def _ordered_runtime_plus_static(self, detectors: Sequence[str]) -> List[str]:
        selected = {self._normalize_detector_name(detector) for detector in detectors}
        ordered: List[str] = []
        for detector_id in self.static_enabled_detectors:
            if detector_id in selected and detector_id not in ordered:
                ordered.append(detector_id)
        for detector_id in self.detector_order:
            if detector_id in selected and detector_id not in ordered:
                ordered.append(detector_id)
        return ordered

    def _refresh_runtime_locked(self, detector_id: str) -> None:
        runtime = self.detector_runtime[detector_id]
        process = runtime.process
        if process is None:
            return

        exit_code = process.poll()
        if exit_code is None:
            return

        runtime.last_exit_code = int(exit_code)
        if not runtime.last_error:
            runtime.last_error = f"process exited with code {exit_code}"
        runtime.process = None
        runtime.started_at = 0.0
        self._aggregator_sync_requested = True

    def _running_detectors_locked(self) -> List[str]:
        running = []
        for detector_id in self.detector_order:
            self._refresh_runtime_locked(detector_id)
            if self.detector_runtime[detector_id].process is not None:
                running.append(detector_id)
        return running

    def _start_output_reader(self, detector_id: str, process: subprocess.Popen) -> None:
        def _reader() -> None:
            try:
                assert process.stdout is not None
                for raw in iter(process.stdout.readline, b""):
                    line = raw.decode("utf-8", errors="replace").rstrip()
                    with self._lock:
                        runtime = self.detector_runtime[detector_id]
                        runtime.recent_output.append(line)
            except Exception:
                return

        threading.Thread(target=_reader, daemon=True).start()

    def _start_detector_locked(self, detector_id: str) -> Optional[str]:
        self._refresh_runtime_locked(detector_id)
        runtime = self.detector_runtime[detector_id]
        if runtime.process is not None:
            return None

        if not self._engine_ready(detector_id):
            spec = self.detector_specs[detector_id]
            runtime.last_error = "missing engine: %s" % self._engine_path(spec.engine_rel)
            return runtime.last_error

        spec = self.detector_specs[detector_id]
        runtime.last_error = ""
        runtime.last_exit_code = None
        runtime.recent_output.clear()

        process = subprocess.Popen(
            spec.command,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            env=dict(self._base_env),
            preexec_fn=os.setsid,
        )
        runtime.process = process
        runtime.started_at = time.time()
        self._start_output_reader(detector_id, process)

        time.sleep(spec.startup_wait_sec)
        self._refresh_runtime_locked(detector_id)
        if runtime.process is None:
            output = " | ".join(list(runtime.recent_output)[-3:])
            runtime.last_error = runtime.last_error or output or "startup failed"
            return runtime.last_error

        self.get_logger().info(
            "Started detector %s (pid=%d)" % (detector_id, runtime.process.pid)
        )
        self._aggregator_sync_requested = True
        return None

    def _stop_detector_locked(self, detector_id: str, timeout_sec: float = 10.0) -> Optional[str]:
        runtime = self.detector_runtime[detector_id]
        self._refresh_runtime_locked(detector_id)
        process = runtime.process
        if process is None:
            runtime.last_error = ""
            return None

        pid = process.pid
        try:
            os.killpg(os.getpgid(pid), signal.SIGINT)
        except ProcessLookupError:
            runtime.process = None
            runtime.started_at = 0.0
            self._aggregator_sync_requested = True
            return None
        except Exception as exc:
            runtime.last_error = f"failed to stop process {pid}: {exc}"
            return runtime.last_error

        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            self._refresh_runtime_locked(detector_id)
            if runtime.process is None:
                runtime.last_error = ""
                self.get_logger().info("Stopped detector %s" % detector_id)
                return None
            time.sleep(0.1)

        try:
            os.killpg(os.getpgid(pid), signal.SIGKILL)
        except Exception:
            pass

        deadline = time.time() + 5.0
        while time.time() < deadline:
            self._refresh_runtime_locked(detector_id)
            if runtime.process is None:
                runtime.last_error = ""
                self.get_logger().info("Stopped detector %s after SIGKILL" % detector_id)
                return None
            time.sleep(0.1)

        runtime.last_error = "failed to stop process %d" % pid
        return runtime.last_error

    def _build_status_locked(self) -> Dict[str, Any]:
        running = self._running_detectors_locked()
        desired = self._ordered(self._desired_detectors)
        available = self._available_detectors()
        unavailable = self._unavailable_detectors()

        detectors = {}
        for detector_id in self.detector_order:
            runtime = self.detector_runtime[detector_id]
            process = runtime.process
            detectors[detector_id] = {
                "desired": detector_id in self._desired_detectors,
                "running": process is not None,
                "pid": process.pid if process is not None else 0,
                "topic": self.detector_specs[detector_id].topic,
                "last_error": runtime.last_error,
                "engine_ready": self._engine_ready(detector_id),
                "recent_output": list(runtime.recent_output)[-8:],
            }

        return {
            "timestamp": round(time.time(), 3),
            "desired": desired,
            "running": running,
            "available": available,
            "unavailable": unavailable,
            "detectors": detectors,
        }

    def _publish_runtime_status(self) -> None:
        with self._lock:
            payload = self._build_status_locked()

        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=True)
        self.runtime_pub.publish(msg)

    def _request_aggregator_sync(self) -> None:
        with self._lock:
            self._aggregator_sync_requested = True

    def _target_aggregator_detectors_locked(self) -> List[str]:
        running = self._running_detectors_locked()
        selected = set(self.static_enabled_detectors)
        selected.update(running)
        return self._ordered_runtime_plus_static(selected)

    def _sync_aggregator_callback(self, future) -> None:
        success = False
        try:
            result = future.result()
            success = bool(result and result.success)
        except Exception as exc:
            self.get_logger().warn("vision_manager sync failed: %s" % exc)

        with self._lock:
            self._aggregator_sync_inflight = False
            if success:
                self._last_synced_aggregator = self._target_aggregator_detectors_locked()
                self._aggregator_sync_requested = False
            else:
                self._aggregator_sync_requested = True

    def _maybe_sync_aggregator(self) -> None:
        with self._lock:
            target = self._target_aggregator_detectors_locked()
            if self._aggregator_sync_inflight:
                return
            if not self._aggregator_sync_requested and target == (self._last_synced_aggregator or []):
                return

        if not self.aggregator_client.wait_for_service(timeout_sec=0.1):
            return

        req = SetDetectors.Request()
        req.detectors = list(target)

        with self._lock:
            self._aggregator_sync_inflight = True

        future = self.aggregator_client.call_async(req)
        future.add_done_callback(self._sync_aggregator_callback)

    def _on_runtime_timer(self) -> None:
        self._publish_runtime_status()
        self._maybe_sync_aggregator()

    def handle_set_detectors(self, request, response):
        requested = self._normalize_detectors(request.detectors)
        if "all" in requested:
            requested = self._available_detectors()

        unsupported = [detector for detector in requested if detector not in self.detector_specs]
        unavailable_requested = [detector for detector in requested if detector in self._unavailable_detectors()]

        if unsupported or unavailable_requested:
            with self._lock:
                status = self._build_status_locked()
            self._publish_runtime_status()
            failed = self._ordered(unsupported + unavailable_requested)
            response.success = False
            response.message = "unsupported or unavailable detectors: %s" % ",".join(failed)
            response.desired = list(status["desired"])
            response.running = list(status["running"])
            response.available = list(status["available"])
            response.unavailable = list(status["unavailable"])
            response.failed = failed
            return response

        requested_set = set(requested)
        with self._lock:
            current_running = set(self._running_detectors_locked())
            to_add = [detector for detector in self._ordered(requested) if detector not in current_running]
            to_remove = [detector for detector in self._ordered(current_running) if detector not in requested_set]

            started_this_call: List[str] = []
            stopped_this_call: List[str] = []
            for detector_id in to_add:
                error = self._start_detector_locked(detector_id)
                if error:
                    for rollback_detector in reversed(started_this_call):
                        self._stop_detector_locked(rollback_detector)
                    status = self._build_status_locked()
                    self._publish_runtime_status()
                    response.success = False
                    response.message = "failed to start %s: %s" % (detector_id, error)
                    response.desired = list(status["desired"])
                    response.running = list(status["running"])
                    response.available = list(status["available"])
                    response.unavailable = list(status["unavailable"])
                    response.failed = [detector_id]
                    return response
                started_this_call.append(detector_id)

            for detector_id in to_remove:
                error = self._stop_detector_locked(detector_id)
                if error:
                    for rollback_detector in reversed(stopped_this_call):
                        self._start_detector_locked(rollback_detector)
                    for rollback_detector in reversed(started_this_call):
                        self._stop_detector_locked(rollback_detector)
                    status = self._build_status_locked()
                    self._publish_runtime_status()
                    response.success = False
                    response.message = "failed to stop %s: %s" % (detector_id, error)
                    response.desired = list(status["desired"])
                    response.running = list(status["running"])
                    response.available = list(status["available"])
                    response.unavailable = list(status["unavailable"])
                    response.failed = [detector_id]
                    return response
                stopped_this_call.append(detector_id)

            self._desired_detectors = set(requested_set)
            self._aggregator_sync_requested = True
            status = self._build_status_locked()
            self._publish_runtime_status()

        self._maybe_sync_aggregator()

        response.success = True
        response.message = "runtime detectors updated"
        response.desired = list(status["desired"])
        response.running = list(status["running"])
        response.available = list(status["available"])
        response.unavailable = list(status["unavailable"])
        response.failed = []
        return response

    def stop_all(self) -> None:
        with self._lock:
            for detector_id in reversed(self.detector_order):
                self._stop_detector_locked(detector_id, timeout_sec=3.0)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VisionRuntimeManager()
    try:
        rclpy.spin(node)
    finally:
        node.stop_all()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
