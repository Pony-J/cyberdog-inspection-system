#!/usr/bin/env python3

import base64
import json
import os
import time
from datetime import datetime
from typing import Any, Dict, List, Optional, Tuple

import cv2
from cv_bridge import CvBridge
from interaction_msgs.msg import BodyInfo
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from std_msgs.msg import String

from cyberdog_ai_pkg.msg import DetectionInfo
from cyberdog_ai_pkg.msg import MeterInfo
from cyberdog_ai_pkg.srv import SetDetectors

try:
    from PIL import Image as PilImage
    from PIL import ImageDraw
    from PIL import ImageFont
except Exception:  # pragma: no cover - optional runtime dependency
    PilImage = None
    ImageDraw = None
    ImageFont = None


class VisionManagerNode(Node):
    def __init__(self) -> None:
        super().__init__("vision_manager_node")

        self.declare_parameter("image_topic", "/mi1035085/camera/color/image_raw")
        self.declare_parameter("body_topic", "/mi1035085/body")
        self.declare_parameter("meter_topic", "/mi1035085/meter")
        self.declare_parameter("detections_topic", "/vision/detections")
        self.declare_parameter("annotated_topic", "/vision/annotated/compressed")
        self.declare_parameter("annotated_ws_topic", "/vision/annotated_ws")
        self.declare_parameter("status_topic", "/vision/status")
        self.declare_parameter("extra_detection_topics", [])
        self.declare_parameter("status_period_sec", 1.0)
        self.declare_parameter("annotated_jpeg_quality", 85)
        self.declare_parameter("annotated_ws_fps", 5.0)
        self.declare_parameter("available_detectors", ["person", "meter"])
        self.declare_parameter("enabled_detectors", ["person", "meter"])
        self.declare_parameter("stale_detection_sec", 1.0)
        self.declare_parameter("meter_hold_sec", 3.5)
        self.declare_parameter("alarm_log_dir", os.path.expanduser("~/cyberdog_ws/alarm_logs"))
        self.declare_parameter("alarm_cooldown_sec", 5.0)
        self.declare_parameter("alarm_ws_topic", "/vision/alarm_ws")

        self.image_topic = str(self.get_parameter("image_topic").value)
        self.body_topic = str(self.get_parameter("body_topic").value)
        self.meter_topic = str(self.get_parameter("meter_topic").value)
        self.detections_topic = str(self.get_parameter("detections_topic").value)
        annotated_topic = str(self.get_parameter("annotated_topic").value)
        annotated_ws_topic = str(self.get_parameter("annotated_ws_topic").value)
        status_topic = str(self.get_parameter("status_topic").value)
        status_period_sec = float(self.get_parameter("status_period_sec").value)
        self.annotated_jpeg_quality = int(self.get_parameter("annotated_jpeg_quality").value)
        annotated_ws_fps = max(0.1, float(self.get_parameter("annotated_ws_fps").value))
        self.stale_detection_sec = float(self.get_parameter("stale_detection_sec").value)
        self.meter_hold_sec = max(
            self.stale_detection_sec,
            float(self.get_parameter("meter_hold_sec").value),
        )
        self.alarm_log_dir = str(self.get_parameter("alarm_log_dir").value)
        self.alarm_cooldown_sec = float(self.get_parameter("alarm_cooldown_sec").value)
        self.annotated_ws_interval = 1.0 / annotated_ws_fps

        os.makedirs(self.alarm_log_dir, exist_ok=True)
        self._last_alarm_save_at = 0.0
        self._last_annotated_ws_publish_at = 0.0
        self._overlay_font_path = self._find_overlay_font_path()
        self._overlay_font_cache: Dict[int, Any] = {}

        self.extra_detection_topics = self._parse_extra_detection_topics(
            self.get_parameter("extra_detection_topics").value
        )
        self.available_detectors = self._normalize_detectors(
            self.get_parameter("available_detectors").value
        )
        for detector_name in self.extra_detection_topics:
            if detector_name not in self.available_detectors:
                self.available_detectors.append(detector_name)

        requested_enabled = self._normalize_detectors(
            self.get_parameter("enabled_detectors").value
        )
        if not requested_enabled:
            self.enabled_detectors = list(self.available_detectors)
        else:
            self.enabled_detectors = [
                detector for detector in requested_enabled if detector in self.available_detectors
            ]
        if requested_enabled and not self.enabled_detectors:
            self.get_logger().warn(
                "No requested detectors matched available set: requested=%s available=%s"
                % (requested_enabled, self.available_detectors)
            )

        self.bridge = CvBridge()
        self.annotated_pub = self.create_publisher(CompressedImage, annotated_topic, 10)
        self.annotated_ws_pub = self.create_publisher(String, annotated_ws_topic, 10)
        self.status_pub = self.create_publisher(String, status_topic, 10)
        self.detections_pub = self.create_publisher(String, self.detections_topic, 10)
        alarm_ws_topic = str(self.get_parameter("alarm_ws_topic").value)
        self.alarm_ws_pub = self.create_publisher(String, alarm_ws_topic, 10)
        self.set_detectors_srv = self.create_service(
            SetDetectors, "/vision/set_detectors", self.handle_set_detectors
        )

        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.on_image,
            qos_profile_sensor_data,
        )
        self.body_sub = self.create_subscription(
            BodyInfo, self.body_topic, self.on_body_info, 10
        )
        self.meter_sub = self.create_subscription(
            MeterInfo, self.meter_topic, self.on_meter_info, 10
        )
        self.extra_detection_subs = {}
        for detector_name, topic_name in self.extra_detection_topics.items():
            self.extra_detection_subs[detector_name] = self.create_subscription(
                DetectionInfo,
                topic_name,
                lambda msg, name=detector_name: self.on_extra_detection(name, msg),
                10,
            )
        self.status_timer = self.create_timer(status_period_sec, self.publish_status)

        self.detector_sources = {
            "person": self.body_topic,
            "meter": self.meter_topic,
            **self.extra_detection_topics,
        }

        self.latest_image = None
        self.latest_image_header = None
        self.latest_body_msg = None
        self.latest_meter_msg = None
        self.latest_meter_positive_msg = None
        self.latest_meter_positive_received_at = 0.0
        self.latest_extra_msgs: Dict[str, DetectionInfo] = {}

        self.last_image_received_at = 0.0
        self.last_detection_publish_at = 0.0
        self.last_status_publish_at = 0.0
        self.image_fps = 0.0
        self._last_image_frame_at = None

        self.get_logger().info(
            "vision_manager ready: image=%s body=%s meter=%s extra=%s enabled=%s"
            % (
                self.image_topic,
                self.body_topic,
                self.meter_topic,
                self.extra_detection_topics,
                ",".join(self.enabled_detectors),
            )
        )

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
        normalized = []
        for value in values:
            text = self._normalize_detector_name(value)
            if text and text not in normalized:
                normalized.append(text)
        return normalized

    def _parse_extra_detection_topics(self, values: Any) -> Dict[str, str]:
        if values is None:
            return {}
        if isinstance(values, str):
            values = values.strip().strip("[]")
            values = [] if not values else [item.strip().strip("'\"") for item in values.split(",")]

        mapping = {}
        for value in values:
            if "=" not in str(value):
                self.get_logger().warn(
                    "Ignoring malformed extra_detection_topics entry: %s" % value
                )
                continue
            detector_name, topic_name = str(value).split("=", 1)
            detector_name = self._normalize_detector_name(detector_name)
            topic_name = topic_name.strip()
            if not detector_name or not topic_name:
                continue
            mapping[detector_name] = topic_name
        return mapping

    def handle_set_detectors(self, request, response):
        requested = self._normalize_detectors(request.detectors)
        if "all" in requested:
            requested = list(self.available_detectors)

        unsupported = [item for item in requested if item not in self.available_detectors]
        active = [item for item in requested if item in self.available_detectors]

        self.enabled_detectors = active
        response.success = len(unsupported) == 0
        response.active = list(self.enabled_detectors)
        if unsupported:
            response.message = "unsupported detectors: %s" % ",".join(unsupported)
        else:
            response.message = "active detectors updated"

        self.publish_current_detections()
        self.publish_status()
        self.publish_annotated_image()
        return response

    def on_image(self, msg: Image) -> None:
        now = time.monotonic()
        self.last_image_received_at = now
        if self._last_image_frame_at is not None:
            delta = now - self._last_image_frame_at
            if delta > 0.0:
                fps = 1.0 / delta
                self.image_fps = fps if self.image_fps == 0.0 else (0.8 * self.image_fps + 0.2 * fps)
        self._last_image_frame_at = now

        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.latest_image_header = msg.header
        except Exception as exc:
            self.get_logger().warn("image conversion failed: %s" % exc)
            return

        self.publish_current_detections()
        self.publish_annotated_image()

    def on_body_info(self, msg: BodyInfo) -> None:
        self.latest_body_msg = msg
        self.publish_current_detections()
        self.publish_annotated_image()

    def on_meter_info(self, msg: MeterInfo) -> None:
        self.latest_meter_msg = msg
        if int(msg.count) > 0 and len(msg.infos) > 0:
            self.latest_meter_positive_msg = msg
            self.latest_meter_positive_received_at = time.time()
        self.publish_current_detections()
        self.publish_annotated_image()

    def on_extra_detection(self, detector_name: str, msg: DetectionInfo) -> None:
        actual_name = self._normalize_detector_name(msg.detector_name or detector_name)
        if actual_name not in self.detector_sources:
            self.detector_sources[actual_name] = self.extra_detection_topics.get(detector_name, "")
        if actual_name not in self.available_detectors:
            self.available_detectors.append(actual_name)
        self.latest_extra_msgs[actual_name] = msg
        self.publish_current_detections()
        self.publish_annotated_image()

    def _stamp_to_sec(self, stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) / 1_000_000_000.0

    def _is_recent(self, stamp) -> bool:
        if stamp.sec == 0 and stamp.nanosec == 0:
            return True
        return (time.time() - self._stamp_to_sec(stamp)) <= self.stale_detection_sec

    def _build_detection(
        self,
        detector_name: str,
        class_id: str,
        score: float,
        roi,
        code: str = "",
        description: str = "",
        is_alarm: bool = False,
    ) -> Dict[str, Any]:
        width = max(1, int(roi.width))
        height = max(1, int(roi.height))
        roi_dict = {
            "x": int(roi.x_offset),
            "y": int(roi.y_offset),
            "width": width,
            "height": height,
        }
        return {
            "detector": detector_name,
            "class_id": class_id,
            "code": code,
            "description": description,
            "score": float(score),
            "is_alarm": bool(is_alarm),
            "bbox": {
                "center_x": float(roi.x_offset) + float(width) / 2.0,
                "center_y": float(roi.y_offset) + float(height) / 2.0,
                "size_x": float(width),
                "size_y": float(height),
                "theta": 0.0,
            },
            "roi": roi_dict,
        }

    def _format_meter_reading_text(self, meter) -> str:
        value = float(meter.value)
        unit = str(meter.unit or "").strip()
        if abs(value - round(value)) < 0.05:
            value_text = str(int(round(value)))
        else:
            value_text = ("%.1f" % value).rstrip("0").rstrip(".")
        return ("%s %s" % (value_text, unit)).strip()

    def _select_meter_msg(self) -> Optional[MeterInfo]:
        if (
            self.latest_meter_msg is not None
            and self._is_recent(self.latest_meter_msg.header.stamp)
            and int(self.latest_meter_msg.count) > 0
            and len(self.latest_meter_msg.infos) > 0
        ):
            return self.latest_meter_msg

        if (
            self.latest_meter_positive_msg is not None
            and (time.time() - self.latest_meter_positive_received_at) <= self.meter_hold_sec
        ):
            return self.latest_meter_positive_msg

        return None

    def _init_detector_stats(self) -> Dict[str, Dict[str, Any]]:
        return {
            name: {
                "count": 0,
                "source": self.detector_sources.get(name, ""),
                "classes": {},
            }
            for name in self.available_detectors
        }

    def _register_detection(
        self,
        detections: List[Dict[str, Any]],
        counts: Dict[str, int],
        detector_stats: Dict[str, Dict[str, Any]],
        detector_name: str,
        detection: Dict[str, Any],
    ) -> None:
        if detector_name not in counts:
            counts[detector_name] = 0
        if detector_name not in detector_stats:
            detector_stats[detector_name] = {
                "count": 0,
                "source": self.detector_sources.get(detector_name, ""),
                "classes": {},
            }

        detections.append(detection)
        counts[detector_name] += 1
        detector_stats[detector_name]["count"] += 1
        class_counts = detector_stats[detector_name]["classes"]
        class_id = detection.get("class_id", "unknown")
        class_counts[class_id] = int(class_counts.get(class_id, 0)) + 1

    def _collect_detections(
        self,
    ) -> Tuple[List[Dict[str, Any]], Dict[str, int], Dict[str, Dict[str, Any]], float]:
        detections: List[Dict[str, Any]] = []
        counts = {name: 0 for name in self.available_detectors}
        detector_stats = self._init_detector_stats()
        latest_stamp_sec = 0.0

        if (
            "person" in self.enabled_detectors
            and self.latest_body_msg is not None
            and self._is_recent(self.latest_body_msg.header.stamp)
        ):
            latest_stamp_sec = max(latest_stamp_sec, self._stamp_to_sec(self.latest_body_msg.header.stamp))
            for body in self.latest_body_msg.infos:
                self._register_detection(
                    detections,
                    counts,
                    detector_stats,
                    "person",
                    self._build_detection(
                        detector_name="person",
                        class_id="person",
                        score=1.0,
                        roi=body.roi,
                        description="person",
                        is_alarm=False,
                    ),
                )

        meter_msg = self._select_meter_msg()
        if "meter" in self.enabled_detectors and meter_msg is not None:
            latest_stamp_sec = max(latest_stamp_sec, self._stamp_to_sec(meter_msg.header.stamp))
            for meter in meter_msg.infos:
                class_id = meter.meter_label or "meter"
                reading_text = self._format_meter_reading_text(meter)
                detection = self._build_detection(
                    detector_name="meter",
                    class_id=class_id,
                    score=1.0 if meter.valid_depth else 0.8,
                    roi=meter.roi,
                    description=reading_text or meter.note or class_id,
                    is_alarm=False,
                )
                detection["reading_value"] = float(meter.value)
                detection["reading_unit"] = str(meter.unit or "")
                detection["reading_text"] = reading_text
                self._register_detection(
                    detections,
                    counts,
                    detector_stats,
                    "meter",
                    detection,
                )

        for detector_name, msg in self.latest_extra_msgs.items():
            if detector_name not in self.enabled_detectors:
                continue
            if not self._is_recent(msg.header.stamp):
                continue

            latest_stamp_sec = max(latest_stamp_sec, self._stamp_to_sec(msg.header.stamp))
            detector_key = self._normalize_detector_name(msg.detector_name or detector_name)
            for info in msg.infos:
                self._register_detection(
                    detections,
                    counts,
                    detector_stats,
                    detector_key,
                    self._build_detection(
                        detector_name=detector_key,
                        class_id=info.class_id,
                        score=info.score,
                        roi=info.roi,
                        code=info.code,
                        description=info.description,
                        is_alarm=info.is_alarm,
                    ),
                )

        return detections, counts, detector_stats, latest_stamp_sec

    def _build_sources_payload(self) -> Dict[str, str]:
        payload = {"image": self.image_topic}
        for detector_name in self.available_detectors:
            payload[detector_name] = self.detector_sources.get(detector_name, "")
        return payload

    def publish_current_detections(self) -> None:
        detections, counts, detector_stats, _ = self._collect_detections()
        payload = {
            "timestamp": round(time.time(), 3),
            "count": len(detections),
            "counts": counts,
            "detectors": detector_stats,
            "sources": self._build_sources_payload(),
            "detections": detections,
        }

        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=True)
        self.detections_pub.publish(msg)
        self.last_detection_publish_at = time.monotonic()

    def _save_alarm(self, detections, frame) -> None:
        now = time.monotonic()
        if now - self._last_alarm_save_at < self.alarm_cooldown_sec:
            return

        alarm_dets = [d for d in detections if d.get("is_alarm", False)]
        if not alarm_dets:
            return

        ts = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        img_path = os.path.join(self.alarm_log_dir, "alarm_%s.jpg" % ts)
        json_path = os.path.join(self.alarm_log_dir, "alarm_%s.json" % ts)

        cv2.imwrite(img_path, frame)

        entries = []
        for detection in alarm_dets:
            bbox = detection.get("bbox", {})
            entries.append({
                "detector": detection.get("detector", "unknown"),
                "class_id": detection.get("class_id", "unknown"),
                "code": detection.get("code", ""),
                "description": detection.get("description", ""),
                "score": round(float(detection.get("score", 0.0)), 4),
                "bbox": {
                    "center_x": round(float(bbox.get("center_x", 0.0)), 1),
                    "center_y": round(float(bbox.get("center_y", 0.0)), 1),
                    "size_x": round(float(bbox.get("size_x", 0.0)), 1),
                    "size_y": round(float(bbox.get("size_y", 0.0)), 1),
                },
                "timestamp": ts,
            })
        with open(json_path, "w") as f:
            f.write(json.dumps(entries, ensure_ascii=True, indent=2))

        self._last_alarm_save_at = now
        self.get_logger().info(
            "Alarm saved: %d detection(s) -> %s" % (len(alarm_dets), img_path)
        )

        # Publish alarm to WS topic for bridge
        _, jpeg_buffer = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, self.annotated_jpeg_quality])
        image_base64 = base64.b64encode(jpeg_buffer).decode("ascii")
        alarm_payload = {
            "timestamp": ts,
            "detections": entries,
            "image_base64": image_base64,
        }
        alarm_msg = String()
        alarm_msg.data = json.dumps(alarm_payload, ensure_ascii=True)
        self.alarm_ws_pub.publish(alarm_msg)

    def _color_for_detection(self, detection: Dict[str, Any]) -> Tuple[int, int, int]:
        class_id = detection.get("class_id", "unknown")
        detector_name = detection.get("detector", "")

        if class_id == "person":
            return (0, 255, 0)
        if class_id == "white_smoke":
            return (0, 255, 255)
        if class_id == "fire":
            return (0, 0, 255)
        if detector_name == "meter":
            return (255, 128, 0)
        if detection.get("is_alarm", False):
            return (0, 0, 255)
        return (255, 255, 0)

    def _find_overlay_font_path(self) -> str:
        candidates = [
            "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
            "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc",
            "/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf",
        ]
        for path in candidates:
            if os.path.exists(path):
                return path
        return ""

    def _annotation_font_size(self, frame, width: int, height: int) -> int:
        frame_h, frame_w = frame.shape[:2]
        by_frame = int(min(frame_h, frame_w) * 0.03)
        by_box = int(max(width, height) * 0.22)
        return max(15, min(28, max(by_frame, by_box)))

    def _ascii_label_for_detection(
        self,
        detector_name: str,
        class_id: str,
    ) -> str:
        if detector_name == "meter":
            return "meter"
        if detector_name.endswith("_alarm"):
            return class_id
        if detector_name == class_id:
            return class_id
        return "%s:%s" % (detector_name, class_id)

    def _display_label_for_detection(
        self,
        detector_name: str,
        class_id: str,
        description: str,
    ) -> str:
        if detector_name == "meter":
            return description or class_id or detector_name
        if detector_name.endswith("_alarm"):
            if description and description != class_id:
                return "%s (%s)" % (description, class_id)
            return class_id
        if detector_name == class_id:
            return description if description and description != class_id else class_id
        if description and description != class_id:
            return "%s (%s:%s)" % (description, detector_name, class_id)
        return "%s:%s" % (detector_name, class_id)

    def _get_overlay_font(self, size: int):
        if not self._overlay_font_path or ImageFont is None:
            return None
        if size not in self._overlay_font_cache:
            try:
                self._overlay_font_cache[size] = ImageFont.truetype(self._overlay_font_path, size)
            except Exception:
                self._overlay_font_cache[size] = None
        return self._overlay_font_cache[size]

    def _label_origin(
        self,
        frame_w: int,
        frame_h: int,
        left: int,
        top: int,
        width: int,
        height: int,
        label_w: int,
        label_h: int,
    ) -> Tuple[int, int]:
        x = max(0, min(left, max(0, frame_w - label_w)))
        y = top - label_h - 6
        if y < 0:
            y = min(frame_h - label_h, top + height + 6)
        return x, max(0, y)

    def _draw_label_cv2(
        self,
        frame,
        left: int,
        top: int,
        width: int,
        height: int,
        text: str,
        score: float,
        color: Tuple[int, int, int],
        show_score: bool = True,
    ) -> None:
        label = "%s %.2f" % (text, score) if show_score else text
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.6 if max(width, height) >= 48 else 0.52
        thickness = 2
        (text_w, text_h), baseline = cv2.getTextSize(label, font, font_scale, thickness)
        pad_x = 6
        pad_y = 4
        box_w = text_w + pad_x * 2
        box_h = text_h + baseline + pad_y * 2
        box_x, box_y = self._label_origin(
            frame.shape[1], frame.shape[0], left, top, width, height, box_w, box_h
        )
        cv2.rectangle(
            frame,
            (box_x, box_y),
            (box_x + box_w, box_y + box_h),
            color,
            thickness=-1,
        )
        cv2.putText(
            frame,
            label,
            (box_x + pad_x, box_y + box_h - baseline - pad_y + 1),
            font,
            font_scale,
            (255, 255, 255),
            thickness,
            cv2.LINE_AA,
        )

    def _draw_label_pil(
        self,
        draw,
        frame_shape,
        left: int,
        top: int,
        width: int,
        height: int,
        text: str,
        score: float,
        color: Tuple[int, int, int],
        font,
        show_score: bool = True,
    ) -> None:
        label = "%s %.2f" % (text, score) if show_score else text
        if hasattr(draw, "textbbox"):
            bbox = draw.textbbox((0, 0), label, font=font)
            text_w = int(bbox[2] - bbox[0])
            text_h = int(bbox[3] - bbox[1])
        else:
            text_w, text_h = font.getsize(label)
        pad_x = 7
        pad_y = 5
        box_w = text_w + pad_x * 2
        box_h = text_h + pad_y * 2
        frame_h, frame_w = frame_shape[:2]
        box_x, box_y = self._label_origin(frame_w, frame_h, left, top, width, height, box_w, box_h)
        fill = (int(color[2]), int(color[1]), int(color[0]))
        draw.rectangle(
            [(box_x, box_y), (box_x + box_w, box_y + box_h)],
            fill=fill,
        )
        draw.text((box_x + pad_x, box_y + pad_y - 1), label, font=font, fill=(255, 255, 255))

    def publish_annotated_image(self) -> None:
        if self.latest_image is None or self.latest_image_header is None:
            return

        frame = self.latest_image.copy()
        detections, counts, _, _ = self._collect_detections()
        text_tasks: List[Dict[str, Any]] = []

        for detection in detections:
            detector_name = detection.get("detector", "unknown")
            class_id = detection.get("class_id", "unknown")
            description = str(detection.get("description", "") or "").strip()
            score = float(detection.get("score", 0.0))
            bbox = detection.get("bbox", {})
            left = int(float(bbox.get("center_x", 0.0)) - float(bbox.get("size_x", 0.0)) / 2.0)
            top = int(float(bbox.get("center_y", 0.0)) - float(bbox.get("size_y", 0.0)) / 2.0)
            width = max(1, int(float(bbox.get("size_x", 0.0))))
            height = max(1, int(float(bbox.get("size_y", 0.0))))
            left = max(0, min(left, max(0, frame.shape[1] - 1)))
            top = max(0, min(top, max(0, frame.shape[0] - 1)))
            width = max(1, min(width, frame.shape[1] - left))
            height = max(1, min(height, frame.shape[0] - top))
            color = self._color_for_detection(detection)
            thickness = 3 if detection.get("is_alarm", False) else 2
            cv2.rectangle(
                frame,
                (left, top),
                (left + width, top + height),
                color,
                thickness,
                cv2.LINE_AA,
            )
            display_label = self._display_label_for_detection(detector_name, class_id, description)
            ascii_label = self._ascii_label_for_detection(detector_name, class_id)
            text_tasks.append(
                {
                    "detector_name": detector_name,
                    "left": left,
                    "top": top,
                    "width": width,
                    "height": height,
                    "color": color,
                    "score": score,
                    "display_label": display_label,
                    "ascii_label": ascii_label,
                }
            )

        pil_draw = None
        pil_frame = None
        can_draw_unicode = PilImage is not None and ImageDraw is not None and self._overlay_font_path

        for task in text_tasks:
            font_size = self._annotation_font_size(frame, task["width"], task["height"])
            font = self._get_overlay_font(font_size) if can_draw_unicode else None
            if font is not None:
                if pil_draw is None:
                    pil_frame = PilImage.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
                    pil_draw = ImageDraw.Draw(pil_frame)
                self._draw_label_pil(
                    pil_draw,
                    frame.shape,
                    task["left"],
                    task["top"],
                    task["width"],
                    task["height"],
                    task["display_label"],
                    task["score"],
                    task["color"],
                    font,
                    task["detector_name"] != "meter",
                )
            else:
                self._draw_label_cv2(
                    frame,
                    task["left"],
                    task["top"],
                    task["width"],
                    task["height"],
                    task["ascii_label"],
                    task["score"],
                    task["color"],
                    task["detector_name"] != "meter",
                )

        if pil_frame is not None:
            frame = cv2.cvtColor(np.asarray(pil_frame), cv2.COLOR_RGB2BGR)

        count_text = " ".join(
            "%s=%d" % (name, counts.get(name, 0)) for name in self.available_detectors
        )
        overlay = "enabled=%s %s fps=%.1f" % (
            ",".join(self.enabled_detectors),
            count_text,
            self.image_fps,
        )
        cv2.putText(
            frame,
            overlay,
            (16, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
        )

        self._save_alarm(detections, frame)

        encoded, buffer = cv2.imencode(
            ".jpg",
            frame,
            [int(cv2.IMWRITE_JPEG_QUALITY), self.annotated_jpeg_quality],
        )
        if not encoded:
            self.get_logger().warn("failed to encode annotated frame")
            return

        compressed = CompressedImage()
        compressed.header = self.latest_image_header
        compressed.format = "jpeg"
        compressed.data = buffer.tobytes()
        self.annotated_pub.publish(compressed)

        now = time.monotonic()
        if now - self._last_annotated_ws_publish_at >= self.annotated_ws_interval:
            payload = String()
            payload.data = json.dumps(
                {
                    "format": compressed.format,
                    "image_base64": base64.b64encode(
                        compressed.data
                    ).decode("ascii"),
                },
                ensure_ascii=True,
            )
            self.annotated_ws_pub.publish(payload)
            self._last_annotated_ws_publish_at = now

    def publish_status(self) -> None:
        _, counts, detector_stats, latest_stamp_sec = self._collect_detections()
        latency_ms = 0.0
        if latest_stamp_sec > 0.0:
            latency_ms = max(0.0, (time.time() - latest_stamp_sec) * 1000.0)

        payload = {
            "enabled": list(self.enabled_detectors),
            "available": list(self.available_detectors),
            "fps": round(self.image_fps, 2),
            "latency_ms": round(latency_ms, 1),
            "counts": counts,
            "detectors": detector_stats,
            "sources": self._build_sources_payload(),
            "errors": {},
        }

        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=True)
        self.status_pub.publish(msg)
        self.last_status_publish_at = time.monotonic()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VisionManagerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
