#!/usr/bin/env python3
import json
import os
from typing import Any, Dict, List, Optional


DEFAULT_POINT = {
    "point_id": "",
    "name": "",
    "pose": {"x": 0.0, "y": 0.0, "yaw": 0.0},
    "meter_type": "pressure",
    "detect_confirm_frames": 5,
    "detect_pass_ratio": 0.7,
    "read_frames": 7,
    "max_retry": 2,
    "alarm_low": None,
    "alarm_high": None,
}


class MeterPointsManager:
    def __init__(self, config_path: str):
        self.config_path = config_path

    def _ensure_parent_dir(self) -> None:
        parent = os.path.dirname(self.config_path)
        if parent:
            os.makedirs(parent, exist_ok=True)

    def load(self) -> Dict[str, Any]:
        if not os.path.isfile(self.config_path):
            return {"points": []}
        with open(self.config_path, "r", encoding="utf-8") as f:
            data = json.load(f)
        if not isinstance(data, dict):
            return {"points": []}
        points = data.get("points", [])
        if not isinstance(points, list):
            points = []
        normalized = []
        for point in points:
            if not isinstance(point, dict):
                continue
            normalized.append(self._normalize_point(point))
        return {"points": normalized}

    def save(self, data: Dict[str, Any]) -> None:
        self._ensure_parent_dir()
        points = data.get("points", []) if isinstance(data, dict) else []
        payload = {
            "points": [self._normalize_point(point) for point in points if isinstance(point, dict)]
        }
        with open(self.config_path, "w", encoding="utf-8") as f:
            json.dump(payload, f, ensure_ascii=False, indent=2)
            f.write("\n")

    def list_points(self) -> List[Dict[str, Any]]:
        return self.load()["points"]

    def get_point(self, point_id: str) -> Optional[Dict[str, Any]]:
        for point in self.list_points():
            if point.get("point_id") == point_id:
                return point
        return None

    def upsert_point(self, point: Dict[str, Any]) -> Dict[str, Any]:
        normalized = self._normalize_point(point)
        data = self.load()
        points = data["points"]
        for index, existing in enumerate(points):
            if existing.get("point_id") == normalized["point_id"]:
                points[index] = normalized
                self.save(data)
                return normalized
        points.append(normalized)
        self.save(data)
        return normalized

    def delete_point(self, point_id: str) -> bool:
        data = self.load()
        points = data["points"]
        kept = [point for point in points if point.get("point_id") != point_id]
        if len(kept) == len(points):
            return False
        data["points"] = kept
        self.save(data)
        return True

    def rename_point(self, point_id: str, name: str) -> Optional[Dict[str, Any]]:
        data = self.load()
        for point in data["points"]:
            if point.get("point_id") == point_id:
                point["name"] = str(name or "")
                normalized = self._normalize_point(point)
                point.update(normalized)
                self.save(data)
                return normalized
        return None

    def next_point_id(self, prefix: str = "meter") -> str:
        used = {point.get("point_id", "") for point in self.list_points()}
        index = 1
        while True:
            candidate = f"{prefix}_{index:02d}"
            if candidate not in used:
                return candidate
            index += 1

    def record_current_pose_point(
        self,
        pose: Dict[str, Any],
        point_id: Optional[str] = None,
        name: Optional[str] = None,
        meter_type: str = "pressure",
        detect_confirm_frames: int = 5,
        detect_pass_ratio: float = 0.7,
        read_frames: int = 7,
        max_retry: int = 2,
        alarm_low: Optional[float] = None,
        alarm_high: Optional[float] = None,
    ) -> Dict[str, Any]:
        point_id = str(point_id or self.next_point_id())
        name = str(name or point_id)
        point = {
            "point_id": point_id,
            "name": name,
            "pose": {
                "x": float(pose.get("x", 0.0)),
                "y": float(pose.get("y", 0.0)),
                "yaw": float(pose.get("yaw", 0.0)),
            },
            "meter_type": str(meter_type or "pressure"),
            "detect_confirm_frames": int(detect_confirm_frames),
            "detect_pass_ratio": float(detect_pass_ratio),
            "read_frames": int(read_frames),
            "max_retry": int(max_retry),
            "alarm_low": alarm_low,
            "alarm_high": alarm_high,
        }
        return self.upsert_point(point)

    def _normalize_point(self, point: Dict[str, Any]) -> Dict[str, Any]:
        merged = dict(DEFAULT_POINT)
        merged.update(point)
        pose = dict(DEFAULT_POINT["pose"])
        pose.update(point.get("pose", {}))
        merged["pose"] = {
            "x": float(pose.get("x", 0.0)),
            "y": float(pose.get("y", 0.0)),
            "yaw": float(pose.get("yaw", 0.0)),
        }
        merged["point_id"] = str(merged.get("point_id", "")).strip()
        merged["name"] = str(merged.get("name", "")).strip()
        merged["meter_type"] = str(merged.get("meter_type", "pressure") or "pressure")
        merged["detect_confirm_frames"] = max(1, int(merged.get("detect_confirm_frames", 5)))
        merged["detect_pass_ratio"] = min(1.0, max(0.1, float(merged.get("detect_pass_ratio", 0.7))))
        merged["read_frames"] = max(1, int(merged.get("read_frames", 7)))
        merged["max_retry"] = max(0, int(merged.get("max_retry", 2)))
        merged["alarm_low"] = self._maybe_float(merged.get("alarm_low"))
        merged["alarm_high"] = self._maybe_float(merged.get("alarm_high"))
        return merged

    @staticmethod
    def _maybe_float(value: Any) -> Optional[float]:
        if value is None or value == "":
            return None
        try:
            return float(value)
        except (TypeError, ValueError):
            return None
