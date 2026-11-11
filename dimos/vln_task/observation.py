"""Read native compressed D455 inputs and freeze the observation before inference."""

import io
import logging
import threading
import time
from collections import deque
from dataclasses import dataclass
from uuid import uuid4

import numpy as np
from PIL import Image

from dimos.plugins.runtime_log import event, recording_enabled
from dimos.plugins.xnavdp.client import ODOM_TOPIC, RGB_TOPIC, stamp_ns

LOG = logging.getLogger('dimos.plugins.vln_task.observation')
DEPTH_TOPIC = "/camera/camera/aligned_depth_to_color/image_raw/compressed"
INFO_TOPIC = "/realsense_camera_info"
PNG = b"\x89PNG\r\n\x1a\n"
TRACKING_MAX_AGE_S = 0.5


@dataclass(frozen=True)
class Observation:
    jpeg: bytes
    stamp_ns: int
    header: object
    k: tuple
    distortion_model: str
    distortion: tuple
    depth: np.ndarray
    odom: object
    depth_gap_ms: float
    odom_gap_ms: float
    wait_ms: float
    observation_id: str | None = None


class Observations:
    def __init__(self, node, settings, depth_scale, timeout):
        from geometry_msgs.msg import PoseStamped
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import CameraInfo, CompressedImage

        self.settings = settings
        self.depth_scale = depth_scale
        self.timeout = timeout
        self.cv = threading.Condition()
        self.frames = {
            topic: deque(maxlen=120) for topic in (RGB_TOPIC, DEPTH_TOPIC, ODOM_TOPIC)
        }
        self.info = None
        self.info_received = None
        self.motion_arrivals = {}
        self.last_stale_inputs = None
        self.subscriptions = [
            node.create_subscription(
                kind, topic, self._receiver(topic), qos_profile_sensor_data
            )
            for topic, kind in (
                (RGB_TOPIC, CompressedImage),
                (DEPTH_TOPIC, CompressedImage),
                (ODOM_TOPIC, PoseStamped),
                (INFO_TOPIC, CameraInfo),
            )
        ]

    def _receiver(self, topic):
        def receive(msg):
            stamp = stamp_ns(msg.header)
            if stamp <= 0:
                return
            with self.cv:
                received = time.monotonic()
                if topic == INFO_TOPIC:
                    self.info = msg
                    self.info_received = received
                else:
                    self.frames[topic].append((received, msg))
                    if topic not in self.motion_arrivals or stamp > self.motion_arrivals[topic][0]:
                        self.motion_arrivals[topic] = (stamp, received)
                self.cv.notify_all()

        return receive

    def camera_frame(self, depth):
        """Return the latest camera JPEG without waiting for task synchronization."""
        topic = DEPTH_TOPIC if depth else RGB_TOPIC
        with self.cv:
            if not self.frames[topic]:
                return None
            received, msg = self.frames[topic][-1]
            if time.monotonic() - received > self.settings["cache_s"]:
                return None

        data = bytes(msg.data)
        if not depth:
            return data if data.startswith(b"\xff\xd8") else None
        offset = data.find(PNG, 0, 20)
        if offset < 0:
            return None
        import cv2

        raw = cv2.imdecode(np.frombuffer(data[offset:], np.uint8), cv2.IMREAD_UNCHANGED)
        if raw is None or raw.dtype != np.uint16 or raw.ndim != 2:
            return None
        level = np.clip(raw * (255.0 / (6.0 * self.depth_scale)), 0, 255)
        colored = cv2.applyColorMap(level.astype(np.uint8), cv2.COLORMAP_TURBO)
        ok, jpeg = cv2.imencode(".jpg", colored, [cv2.IMWRITE_JPEG_QUALITY, 70])
        return jpeg.tobytes() if ok else None

    def latest_rgb_stamp(self):
        with self.cv:
            return self.motion_arrivals.get(RGB_TOPIC, (0, 0))[0]

    def motion_inputs_ready(self):
        with self.cv:
            now = time.monotonic()
            if not recording_enabled():
                return self.info is not None and all(
                    topic in self.motion_arrivals
                    and now - self.motion_arrivals[topic][1] <= TRACKING_MAX_AGE_S
                    for topic in self.frames
                )
            stale = tuple(
                topic for topic in self.frames
                if topic not in self.motion_arrivals
                or now - self.motion_arrivals[topic][1] > TRACKING_MAX_AGE_S
            )
            if self.info is None:
                stale += (INFO_TOPIC,)
            report = stale != self.last_stale_inputs
            if report:
                self.last_stale_inputs = stale
                ages = self._latest_sample_fields(now)
        if report:
            event(LOG, "motion_observation_state", ready=not stale, stale_topics=stale,
                  freshness_limit_ms=TRACKING_MAX_AGE_S * 1000, samples=ages)
        return not stale

    def _latest_sample_fields(self, now):
        latest = {
            topic: {"stamp_ns": stamp, "age_ms": (now - received) * 1000}
            for topic, (stamp, received) in self.motion_arrivals.items()
        }
        if self.info is not None:
            received = self.info_received
            latest[INFO_TOPIC] = {
                "stamp_ns": stamp_ns(self.info.header),
                "age_ms": (now - received) * 1000 if received is not None else None,
            }
        return latest

    def _snapshot_timeout(self, started, after_stamp_ns, max_age_s, blocked_by):
        now = time.monotonic()
        with self.cv:
            samples = self._latest_sample_fields(now)
        event(LOG, "observation_timeout", wait_ms=(now - started) * 1000,
              after_rgb_stamp_ns=after_stamp_ns, max_age_s=max_age_s,
              reason=blocked_by, samples=samples,
              missing_topics=[topic for topic in (*self.frames, INFO_TOPIC) if topic not in samples])

    def snapshot(self, remaining, *, after_stamp_ns=0, accept_odom=None, max_age_s=None):
        started = time.monotonic()
        end = started + min(self.timeout, remaining())
        recording = recording_enabled()
        blocked_by = "missing_or_stale_inputs"
        while True:
            try:
                wait = min(end - time.monotonic(), remaining())
            except TimeoutError:
                if recording:
                    self._snapshot_timeout(started, after_stamp_ns, max_age_s, blocked_by)
                raise
            if wait <= 0:
                if recording:
                    self._snapshot_timeout(started, after_stamp_ns, max_age_s, blocked_by)
                raise TimeoutError(
                    "No recent RGB, aligned depth, odom and camera calibration"
                )
            with self.cv:
                now = time.monotonic()
                oldest = now - self.settings["cache_s"]
                for queue in self.frames.values():
                    while queue and queue[0][0] < oldest:
                        queue.popleft()
                frames = self.frames
                if max_age_s is not None:
                    fresh_since = now - max_age_s
                    frames = {
                        topic: [item for item in queue if item[0] >= fresh_since]
                        for topic, queue in self.frames.items()
                    }
                blocked_by = "missing_or_stale_inputs"
                if self.info is not None and all(frames.values()):
                    rgb_at, rgb = frames[RGB_TOPIC][-1]
                    stamp = stamp_ns(rgb.header)
                    (depth_at, depth), (odom_at, odom) = (
                        min(
                            frames[key],
                            key=lambda item: abs(stamp_ns(item[1].header) - stamp),
                        )
                        for key in (DEPTH_TOPIC, ODOM_TOPIC)
                    )
                    blocked_by = "new_rgb_not_received" if stamp <= after_stamp_ns else "viewpoint_unchanged"
                    if stamp > after_stamp_ns and (
                        accept_odom is None or accept_odom(odom)
                    ):
                        info = self.info
                        info_at = self.info_received if recording else None
                        wait_ms = (time.monotonic() - started) * 1000
                        break
                self.cv.wait(timeout=min(wait, 0.1))
        depth_stamp, odom_stamp = stamp_ns(depth.header), stamp_ns(odom.header)
        depth_offset, odom_offset = (depth_stamp - stamp) / 1e6, (odom_stamp - stamp) / 1e6
        depth_gap, odom_gap = abs(depth_offset), abs(odom_offset)
        observation_id = uuid4().hex if recording else None
        if recording:
            event(
                LOG, "observation_selected", observation_id=observation_id,
                rgb_stamp_ns=stamp, depth_stamp_ns=depth_stamp,
                odom_stamp_ns=odom_stamp, camera_info_stamp_ns=stamp_ns(info.header),
                rgb_age_ms=(now - rgb_at) * 1000, depth_age_ms=(now - depth_at) * 1000,
                odom_age_ms=(now - odom_at) * 1000,
                camera_info_age_ms=(now - info_at) * 1000 if info_at is not None else None,
                depth_offset_ms=depth_offset, odom_offset_ms=odom_offset,
                depth_gap_ms=depth_gap, odom_gap_ms=odom_gap, wait_ms=wait_ms,
                rgb_frame=rgb.header.frame_id, depth_frame=depth.header.frame_id,
                odom_frame=odom.header.frame_id, camera_info_frame=info.header.frame_id,
                after_rgb_stamp_ns=after_stamp_ns, max_age_s=max_age_s,
            )
        if max(depth_gap, odom_gap) > self.settings["gap_warning_ms"]:
            LOG.warning(
                "Observation gap: depth=%.1f ms odom=%.1f ms; using nearest samples",
                depth_gap,
                odom_gap,
            )
        try:
            if (
                not rgb.header.frame_id
                or rgb.header.frame_id != info.header.frame_id
                or not odom.header.frame_id
            ):
                raise ValueError("RGB, CameraInfo or odom frame is missing or inconsistent")
            depth_m = self._decode_depth(depth, info)
            jpeg = bytes(rgb.data)
            with Image.open(io.BytesIO(jpeg)) as image:
                if image.format != "JPEG" or image.size != (info.width, info.height):
                    raise ValueError("RGB encoding or dimensions differ from CameraInfo")
        except (OSError, ValueError) as exc:
            event(LOG, "observation_rejected", rgb_stamp_ns=stamp,
                  error_type=type(exc).__name__, reason=str(exc))
            raise
        return Observation(
            jpeg=jpeg,
            stamp_ns=stamp,
            header=rgb.header,
            k=tuple(info.k),
            distortion_model=info.distortion_model,
            distortion=tuple(info.d),
            depth=depth_m,
            odom=odom,
            depth_gap_ms=depth_gap,
            odom_gap_ms=odom_gap,
            wait_ms=wait_ms,
            observation_id=observation_id,
        )

    def tracking_inputs(self):
        """Take fresh local inputs for the speed preview, independent of the VLM frame."""
        with self.cv:
            depths = self.frames[DEPTH_TOPIC]
            poses = self.frames[ODOM_TOPIC]
            if not depths or not poses or self.info is None:
                return None
            depth_at, depth = depths[-1]
            odom_at, odom = poses[-1]
            if time.monotonic() - min(depth_at, odom_at) > TRACKING_MAX_AGE_S:
                return None
            info = self.info
        return odom, self._decode_depth(depth, info), stamp_ns(depth.header)

    def _decode_depth(self, depth, info):
        compressed = bytes(depth.data)
        offset = compressed.find(PNG, 0, 20)
        if offset < 0:
            raise ValueError("Expected aligned uint16 PNG depth")
        with Image.open(io.BytesIO(compressed[offset:])) as image:
            if image.mode not in {"I;16", "I;16B", "I;16L"} or image.size != (
                info.width,
                info.height,
            ):
                raise ValueError("Depth encoding or dimensions differ from CameraInfo")
            return np.asarray(image, dtype=np.float32) / self.depth_scale
