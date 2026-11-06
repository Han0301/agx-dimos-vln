# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Optional HTTP perception client, geometry and conservative instance association.

No detector/ROS dependencies are imported into the DimOS interpreter.
"""

from __future__ import annotations

import json
import logging
import math
import os
import subprocess
import threading
import time
import uuid
from collections import deque
from copy import deepcopy
from dataclasses import dataclass
from logging.handlers import RotatingFileHandler
from pathlib import Path
from typing import Any
from urllib.parse import quote, urlsplit

import cv2
import numpy as np
import requests


def rotation_rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """ROS base axes: x forward, y left, z up; radians, intrinsic XYZ."""
    cr, sr, cp, sp, cy, sy = (
        math.cos(roll),
        math.sin(roll),
        math.cos(pitch),
        math.sin(pitch),
        math.cos(yaw),
        math.sin(yaw),
    )
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )


def quaternion_rotation(q: np.ndarray) -> np.ndarray:
    x, y, z, w = q / max(float(np.linalg.norm(q)), 1e-12)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def angular_distance(a: float, b: float) -> float:
    return abs((a - b + 180) % 360 - 180)


@dataclass(frozen=True)
class DetectionObservation:
    camera: str
    frame_id: str
    stamp: float
    width: int
    height: int
    label: str
    score: float
    bbox: tuple[float, float, float, float]
    track_id: str
    appearance: tuple[float, ...]
    generation: int


@dataclass(frozen=True)
class TargetFrameSnapshot:
    detection: DetectionObservation
    rgb: Any
    depth: Any
    capture_pose: Any


def decode_observations(
    payload: dict[str, Any], max_age_ms: float
) -> list[DetectionObservation]:
    """Reject malformed coordinates and wrong units before they reach navigation."""
    if payload.get("schema_version") != 1:
        raise ValueError("Unsupported perception schema")
    result = []
    now = time.time()
    for view in payload.get("views", []):
        if view.get("status") != "fresh" and not (
            max_age_ms > 350 and view.get("status") == "stale"
        ):
            continue
        stamp = int(view["source_stamp_ns"]) / 1e9
        if not 0 <= (now - stamp) * 1000 <= max_age_ms:
            continue
        width, height = int(view["width"]), int(view["height"])
        if width <= 0 or height <= 0:
            raise ValueError("Invalid image dimensions")
        for item in view["detections"]:
            box = tuple(float(v) for v in item["bbox_xyxy"])
            score = float(item["score"])
            if len(box) != 4 or not all(math.isfinite(v) for v in (*box, score)):
                raise ValueError("Nonfinite or malformed detection")
            x1, y1, x2, y2 = box
            if not (
                0 <= x1 < x2 <= width and 0 <= y1 < y2 <= height and 0 <= score <= 1
            ):
                raise ValueError("Detection outside original image coordinates")
            result.append(
                DetectionObservation(
                    str(view["camera_id"]),
                    str(view["frame_id"]),
                    stamp,
                    width,
                    height,
                    str(item["class_name"]),
                    score,
                    box,
                    str(item["camera_track_id"]),
                    tuple(float(v) for v in item.get("appearance", [])),
                    int(payload["generation"]),
                )
            )
    return result


class PoseHistory:
    def __init__(self, max_gap_s: float = 0.3, history_size: int = 256) -> None:
        self.max_gap_s = max_gap_s
        self.items: deque[tuple[float, np.ndarray, np.ndarray]] = deque(
            maxlen=history_size
        )
        self.lock = threading.Lock()

    def add(self, pose: Any) -> None:
        p, q = pose.position, pose.orientation
        item = (
            float(pose.ts),
            np.array([p.x, p.y, p.z]),
            np.array([q.x, q.y, q.z, q.w]),
        )
        with self.lock:
            if not self.items or item[0] > self.items[-1][0]:
                self.items.append(item)

    def at(self, stamp: float) -> tuple[np.ndarray, np.ndarray] | None:
        with self.lock:
            items = list(self.items)
        if not items:
            return None
        nearest = min(items, key=lambda p: abs(p[0] - stamp))
        if abs(nearest[0] - stamp) > self.max_gap_s:
            return None
        for a, b in zip(items, items[1:]):
            if a[0] <= stamp <= b[0]:
                t = (stamp - a[0]) / max(b[0] - a[0], 1e-9)
                qa, qb = a[2], b[2]
                qa = qa / max(np.linalg.norm(qa), 1e-12)
                qb = qb / max(np.linalg.norm(qb), 1e-12)
                dot = float(np.dot(qa, qb))
                if dot < 0:
                    qb, dot = -qb, -dot
                if dot > 0.9995:
                    q = qa + t * (qb - qa)
                else:
                    angle = math.acos(min(1, dot))
                    q = (
                        math.sin((1 - t) * angle) * qa + math.sin(t * angle) * qb
                    ) / math.sin(angle)
                return a[1] + t * (b[1] - a[1]), quaternion_rotation(q)
        return nearest[1], quaternion_rotation(nearest[2])


def observation_ray(obs: DetectionObservation, config: dict[str, Any]) -> np.ndarray:
    u, v = (obs.bbox[0] + obs.bbox[2]) / 2, (obs.bbox[1] + obs.bbox[3]) / 2
    if obs.camera == "d455":
        k = config.get("d455_intrinsic")
        if not k:
            raise ValueError("D455 intrinsics unavailable")
        fx, fy, cx, cy = k[0][0], k[1][1], k[0][2], k[1][2]
        rotation = rotation_rpy(0, float(config.get("d455_pitch", 0)), 0)
    else:
        yaw = {"x3_front": 0, "x3_right": 90, "x3_back": 180, "x3_left": 270}[
            obs.camera
        ]
        cx, cy = (obs.width - 1) / 2, (obs.height - 1) / 2
        fx = (obs.width - 1) / (
            2 * math.tan(math.radians(config.get("x3_horizontal_fov_deg", 90)) / 2)
        )
        fy = (obs.height - 1) / (
            2 * math.tan(math.radians(config.get("x3_vertical_fov_deg", 60)) / 2)
        )
        rpy = [math.radians(a) for a in config.get("x3_mount_rpy_deg", [0, 0, 0])]
        rotation = rotation_rpy(*rpy) @ rotation_rpy(0, 0, -math.radians(yaw))
    ray = rotation @ np.array([1, -(u - cx) / fx, -(v - cy) / fy])
    return ray / np.linalg.norm(ray)


class PerceptionClient:
    """Background-only IO. Control loops consume immutable local snapshots."""

    def __init__(self, config: dict[str, Any]):
        self.config = config
        self.endpoint = str(config.get("endpoint", "")).rstrip("/")
        if urlsplit(self.endpoint).hostname not in ("localhost", "127.0.0.1", "::1"):
            raise ValueError("Perception endpoint must be loopback")
        self.lock = threading.RLock()
        self.control_lock = threading.Lock()
        self.stop_event = threading.Event()
        self.thread: threading.Thread | None = None
        self.process: subprocess.Popen[Any] | None = None
        self._log_file: Any = None
        self._control = requests.Session()
        self._control.trust_env = False
        self.task_id: str | None = None
        self.description = ""
        self.labels: list[str] = []
        self.generation = -1
        self.instance_id: str | None = None
        self.payload: dict[str, Any] = {}
        self.last_receive = 0.0
        self.error = "starting"
        self.poses = PoseHistory(
            max_gap_s=float(config.get("pose_max_age_s", 0.3)), history_size=1024
        )
        self.geometry = config
        self.target: DetectionObservation | None = None
        self.target_seen = 0.0
        self.hits: dict[str, deque[float]] = {}
        self._seen_seq = -1
        self._state = "preparing"
        self._task_lost = False
        self.log = logging.getLogger("dimos.target_perception." + uuid.uuid4().hex[:8])
        self.log.setLevel(logging.INFO)
        self.log.propagate = False
        log_dir = config.get("log_dir")
        if log_dir:
            Path(log_dir).mkdir(parents=True, exist_ok=True)
            handler = RotatingFileHandler(
                Path(log_dir) / "observations.jsonl", maxBytes=10_000_000, backupCount=2
            )
            self.log.addHandler(handler)

    def start(self) -> None:
        self.thread = threading.Thread(
            target=self._run, daemon=True, name="target_perception_client"
        )
        self.thread.start()

    def _run(self) -> None:
        session = requests.Session()
        session.trust_env = False
        last_log = 0.0
        try:
            try:
                health = session.get(self.endpoint + "/v1/health", timeout=(0.1, 0.2))
                health.raise_for_status()
                if health.json().get("schema_version") != 1:
                    raise ValueError("Unsupported service protocol")
            except requests.ConnectionError:
                if self.config.get("lifecycle") == "managed":
                    launcher = Path(self.config["service_launcher"])
                    cfg = Path(self.config["service_config"])
                    python = Path(self.config["service_python"])
                    if not all(p.is_file() for p in (launcher, cfg, python)):
                        raise FileNotFoundError(
                            "Provision perception service before enabling it"
                        )
                    env = os.environ.copy()
                    env["TARGET_PERCEPTION_PYTHON"] = str(python)
                    log_dir = Path(self.config["log_dir"])
                    self._log_file = (log_dir / "managed-service.log").open("a")
                    self.process = subprocess.Popen(
                        [str(launcher), "--config", str(cfg)],
                        env=env,
                        stdout=self._log_file,
                        stderr=subprocess.STDOUT,
                    )
            while not self.stop_event.is_set():
                started = time.monotonic()
                with self.lock:
                    task_id = self.task_id
                path = (
                    f"/v1/sessions/{task_id}/observations"
                    if task_id
                    else "/v1/observations"
                )
                try:
                    response = session.get(self.endpoint + path, timeout=(0.1, 0.2))
                    if response.status_code == 404 and task_id:
                        with self.lock:
                            self._task_lost = True
                        raise ValueError(
                            "Target session lost; service may have restarted"
                        )
                    response.raise_for_status()
                    payload = response.json()
                    decode_observations(
                        payload, float(self.config.get("max_observation_age_ms", 350))
                    )
                    with self.lock:
                        if task_id != self.task_id:
                            continue
                        if (
                            self.instance_id
                            and payload["instance_id"] != self.instance_id
                        ):
                            self.target = None
                            self.hits.clear()
                            if task_id:
                                self._task_lost = True
                                raise ValueError(
                                    "Service restarted; target identity invalidated"
                                )
                        self.instance_id = payload["instance_id"]
                        if task_id and (
                            payload.get("task_id") != task_id
                            or payload.get("generation") != self.generation
                        ):
                            raise ValueError("Task/generation mismatch")
                        self.payload = payload
                        self.last_receive = time.monotonic()
                        self.error = ""
                    if started - last_log >= 1:
                        self.log.info(
                            json.dumps(
                                {
                                    "time": time.time(),
                                    "mode": self.config.get("mode"),
                                    "task_id": task_id,
                                    "seq": payload.get("seq"),
                                    "status": payload.get("status"),
                                    "process_ms": payload.get("process_ms"),
                                    "views": [
                                        {
                                            "camera": v["camera_id"],
                                            "age_ms": v.get("age_ms"),
                                            "count": len(v.get("detections", [])),
                                        }
                                        for v in payload.get("views", [])
                                    ],
                                }
                            )
                        )
                        last_log = started
                except (
                    requests.RequestException,
                    ValueError,
                    KeyError,
                    TypeError,
                ) as exc:
                    with self.lock:
                        self.error = str(exc)
                        self.payload = {}
                self.stop_event.wait(
                    max(
                        0.01,
                        1 / float(self.config.get("consume_hz", 10))
                        - (time.monotonic() - started),
                    )
                )
        except Exception as exc:
            with self.lock:
                self.error = str(exc)
            self.log.error("perception startup failed: %s", exc)
        finally:
            session.close()

    def close(self) -> None:
        self.stop_event.set()
        if self.thread:
            self.thread.join(timeout=2)
        with self.control_lock:
            if self.task_id and not self._task_lost:
                try:
                    self._control.delete(
                        self.endpoint + f"/v1/sessions/{self.task_id}",
                        timeout=(0.1, 0.2),
                    )
                except requests.RequestException:
                    pass
            self._control.close()
        if self.process:
            self.process.terminate()
            try:
                self.process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=3)
        if self._log_file:
            self._log_file.close()
        for handler in list(self.log.handlers):
            handler.close()
            self.log.removeHandler(handler)

    def ensure_task(
        self, description: str, labels: list[str], task_id: str = ""
    ) -> str:
        labels = list(dict.fromkeys(s.strip().lower() for s in labels))
        with self.control_lock:
            with self.lock:
                if task_id:
                    if task_id != self.task_id or self._task_lost:
                        raise ValueError("Unknown or expired local target task")
                    return task_id
                if (
                    self.task_id
                    and self.description == description
                    and self.labels == labels
                    and not self._task_lost
                ):
                    return self.task_id
                old_task = self.task_id
            if old_task and not self._task_lost:
                self._control.delete(
                    self.endpoint + f"/v1/sessions/{old_task}", timeout=(0.1, 0.2)
                ).raise_for_status()
            new_id = uuid.uuid4().hex
            response = self._control.post(
                self.endpoint + "/v1/sessions",
                json={
                    "task_id": new_id,
                    "description": description,
                    "labels_en": labels,
                },
                timeout=(0.1, 0.3),
            )
            response.raise_for_status()
            data = response.json()
            if data.get("schema_version") != 1 or data.get("task_id") != new_id:
                raise ValueError("Invalid session response")
            with self.lock:
                self.task_id, self.description, self.labels = (
                    new_id,
                    description,
                    list(labels),
                )
                self.generation, self.instance_id = (
                    int(data["generation"]),
                    str(data["instance_id"]),
                )
                self.target = None
                self.target_seen = 0
                self.hits.clear()
                self.payload = {}
                self._seen_seq = -1
                self._task_lost = False
            return new_id

    def image(self, observation: DetectionObservation) -> np.ndarray:
        response = self._control.get(
            self.endpoint + "/v1/frames/" + quote(observation.frame_id, safe=""),
            timeout=(0.1, 0.3),
        )
        response.raise_for_status()
        bgr = cv2.imdecode(np.frombuffer(response.content, np.uint8), cv2.IMREAD_COLOR)
        if bgr is None or bgr.shape[:2] != (observation.height, observation.width):
            raise ValueError("Frame dimensions do not match detection")
        return bgr

    def current_bearing(self, obs: DetectionObservation) -> float | None:
        ray = observation_ray(obs, self.geometry)
        captured, current = self.poses.at(obs.stamp), self.poses.at(time.time())
        if captured is None or current is None:
            return None
        if np.linalg.norm(captured[0] - current[0]) > float(
            self.config.get("bearing_max_translation_m", 0.05)
        ):
            return None
        local = current[1].T @ captured[1] @ ray
        return math.degrees(-math.atan2(local[1], local[0])) % 360

    def _same_instance(self, a: DetectionObservation, b: DetectionObservation) -> bool:
        if a.camera == b.camera:
            return a.track_id == b.track_id
        if abs(a.stamp - b.stamp) > 0.35 or not a.appearance or not b.appearance:
            return False
        ra, rb = observation_ray(a, self.geometry), observation_ray(b, self.geometry)
        pa, pb = self.poses.at(a.stamp), self.poses.at(b.stamp)
        if pa is None or pb is None or np.linalg.norm(pa[0] - pb[0]) > 0.05:
            return False
        angle = math.degrees(
            math.acos(float(np.clip(np.dot(pa[1] @ ra, pb[1] @ rb), -1, 1)))
        )
        aa, bb = np.array(a.appearance), np.array(b.appearance)
        if (
            aa.shape != bb.shape
            or not np.all(np.isfinite(aa))
            or not np.all(np.isfinite(bb))
        ):
            return False
        cosine = float(
            np.dot(aa, bb) / max(np.linalg.norm(aa) * np.linalg.norm(bb), 1e-9)
        )
        return angle < 15 and cosine > 0.94

    def state(self, candidate_id: str = "") -> dict[str, Any]:
        with self.lock:
            payload = deepcopy(self.payload)
            base = {
                "task_id": self.task_id,
                "motion_enabled": bool(self.config.get("motion_enabled", False)),
                "mode": self.config.get("mode"),
                "generation": self.generation,
            }
            if (
                self.error
                or self._task_lost
                or time.monotonic() - self.last_receive
                > float(self.config.get("client_max_age_s", 0.5))
            ):
                return {
                    **base,
                    "status": "unavailable",
                    "reason": self.error or "client_stale",
                }
            if not payload:
                return {**base, "status": "preparing"}
            if payload.get("status") in ("preparing", "unavailable"):
                return {**base, "status": payload["status"]}
            observations = decode_observations(
                payload, float(self.config.get("max_observation_age_ms", 350))
            )
            labels = set(self.labels)
            observations = [
                o for o in observations if o.label in labels and o.score >= 0.25
            ]
            lock_identity = bool(self.config.get("lock_target_identity", True))

            def confirmation_key(observation: DetectionObservation) -> str:
                return (
                    observation.track_id
                    if lock_identity
                    else f"category:{observation.camera}:{observation.label}"
                )

            if payload.get("seq") != self._seen_seq:
                self._seen_seq = int(payload["seq"])
                self.hits = {
                    key: values
                    for key, values in self.hits.items()
                    if values and time.time() - values[-1] < 2
                }
                for obs in observations:
                    hits = self.hits.setdefault(confirmation_key(obs), deque(maxlen=3))
                    if not hits or obs.stamp > hits[-1]:
                        hits.append(obs.stamp)
            confirmed = [
                o
                for o in observations
                if len(self.hits.get(confirmation_key(o), ())) >= 2
                and o.stamp - self.hits[confirmation_key(o)][-2]
                < (0.4 if lock_identity else 0.9)
            ]
            selected: DetectionObservation | None = None
            candidates = [
                {
                    "candidate_id": o.track_id,
                    "camera": o.camera,
                    "label": o.label,
                    "score": o.score,
                    "bbox": list(o.bbox),
                    "frame_id": o.frame_id,
                }
                for o in observations
            ]
            if not lock_identity:
                # The task asks for a category, not a persistent physical identity.
                # Prefer D455 for depth; otherwise prefer continuity of camera view.
                front_candidates = [o for o in confirmed if o.camera == "d455"]
                pool = front_candidates or confirmed
                if pool:
                    if not front_candidates and self.target is not None:
                        same_view = [o for o in pool if o.camera == self.target.camera]
                        pool = same_view or pool
                    selected = max(pool, key=lambda o: o.score)
            elif candidate_id:
                selected = next(
                    (o for o in confirmed if o.track_id == candidate_id), None
                )
                if selected is None:
                    return {
                        **base,
                        "status": "candidate_expired",
                        "candidates": candidates,
                    }
            elif self.target:
                matches = [o for o in confirmed if self._same_instance(self.target, o)]
                if matches:
                    # Existing exact camera track first; do not switch view merely for a higher score.
                    exact = [o for o in matches if o.camera == self.target.camera]
                    selected = max(exact or matches, key=lambda o: o.score)
            elif confirmed:
                anchor = max(confirmed, key=lambda o: o.score)
                if all(self._same_instance(anchor, o) for o in confirmed):
                    selected = next(
                        (o for o in confirmed if o.camera == "d455"), anchor
                    )
                else:
                    return {**base, "status": "ambiguous", "candidates": candidates}
            if selected:
                self.target = selected
                self.target_seen = selected.stamp
                matches = (
                    [selected]
                    if not lock_identity
                    else [o for o in confirmed if self._same_instance(selected, o)]
                )
                front = next((o for o in matches if o.camera == "d455"), None)
                if front:
                    selected = front
                    self.target = front
                bearing = self.current_bearing(selected)
                capture_ray = observation_ray(selected, self.geometry)
                capture_bearing = (
                    math.degrees(-math.atan2(capture_ray[1], capture_ray[0])) % 360
                )
                return {
                    **base,
                    "status": "front_visible" if front else "x3_covered",
                    "target": selected,
                    "front": front,
                    "selection_mode": "identity" if lock_identity else "category",
                    "candidates": candidates,
                    "clockwise_bearing_deg": bearing,
                    "capture_clockwise_bearing_deg": capture_bearing,
                    "relative_move_degrees": None
                    if bearing is None
                    else ((-bearing + 180) % 360 - 180),
                    "observation_age_ms": (time.time() - selected.stamp) * 1000,
                }
            if self.target:
                hold_s = float(self.config.get("bearing_hold_s", 0))
                if 0 <= time.time() - self.target.stamp <= hold_s:
                    bearing = self.current_bearing(self.target)
                    if bearing is not None:
                        return {
                            **base,
                            "status": "bearing_hold",
                            "reason": "recent_target_direction",
                            "clockwise_bearing_deg": bearing,
                            "observation_age_ms": (time.time() - self.target.stamp)
                            * 1000,
                            "front": None,
                            "candidates": candidates,
                        }
                healthy = [
                    v
                    for v in payload.get("views", [])
                    if (
                        v.get("status") == "fresh"
                        or (
                            float(self.config.get("max_observation_age_ms", 350)) > 350
                            and v.get("status") == "stale"
                        )
                    )
                    and 0
                    <= (time.time() - int(v.get("source_stamp_ns", "0")) / 1e9) * 1000
                    <= float(self.config.get("max_observation_age_ms", 350))
                ]
                if len(healthy) < 5:
                    return {
                        **base,
                        "status": "unavailable",
                        "reason": "camera_coverage_incomplete",
                    }
                if confirmed:
                    return {**base, "status": "ambiguous", "candidates": candidates}
                return {
                    **base,
                    "status": "lost"
                    if time.time() - self.target_seen > 0.5
                    else "temporarily_lost",
                }
            return {
                **base,
                "status": "candidate" if observations else "searching",
                "candidates": candidates,
            }

    def public_state(self, candidate_id: str = "") -> dict[str, Any]:
        state = self.state(candidate_id)
        return {
            **{k: v for k, v in state.items() if k not in ("target", "front")},
            "d455_visible": state.get("status") == "front_visible",
        }


def english_labels(description: str) -> list[str]:
    """Resolve simple categories; richer descriptions need explicit English labels."""
    description = description.strip()
    aliases = {
        "显示器": "monitor",
        "电脑屏幕": "monitor",
        "椅子": "chair",
        "人": "person",
        "绿植": "potted plant",
        "盆栽": "potted plant",
        "背包": "backpack",
        "瓶子": "bottle",
        "杯子": "cup",
        "桌子": "table",
        "沙发": "sofa",
        "sofa": "sofa",
        "couch": "sofa",
    }
    if description.lower() in aliases:
        return [aliases[description.lower()]]
    if description.isascii() and 0 < len(description.split()) <= 4:
        lower = description.lower().strip()
        for label in (
            "computer monitor",
            "monitor",
            "chair",
            "person",
            "potted plant",
            "backpack",
            "bottle",
            "cup",
            "table",
        ):
            if lower == label:
                return [label]
    return []
