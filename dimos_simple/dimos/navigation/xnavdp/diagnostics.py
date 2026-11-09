"""ROS2 health and serialized inference checks for the XNavDP service."""

from __future__ import annotations

import argparse
import json
import os
import statistics
import threading
import time

import numpy as np

from dimos.navigation.xnavdp.client import XNavDPClient


def _wait_for_camera_intrinsic(client: XNavDPClient, timeout: float = 10.0) -> np.ndarray:
    """Read live camera calibration only when initializing a cold backend."""
    from rclpy.qos import qos_profile_sensor_data

    topic = os.environ.get("XNAVDP_CAMERA_INFO_TOPIC", "/camera/camera/color/camera_info")
    ready = threading.Event()
    calibration: list[np.ndarray] = []

    def receive(message):
        intrinsic = np.asarray(message.k, dtype=float).reshape(3, 3)
        if (np.isfinite(intrinsic).all() and intrinsic[0, 0] > 0
                and intrinsic[1, 1] > 0
                and np.allclose(intrinsic[2], [0.0, 0.0, 1.0])):
            calibration[:] = [intrinsic]
            ready.set()

    subscription = client.node.create_subscription(
        client.info_type, topic, receive, qos_profile_sensor_data
    )
    try:
        if not ready.wait(timeout):
            raise TimeoutError(
                f"No valid camera calibration on {topic} within {timeout:g}s; "
                "start the camera or set XNAVDP_CAMERA_INFO_TOPIC"
            )
        return calibration[0].copy()
    finally:
        client.node.destroy_subscription(subscription)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--requests", type=int, default=5)
    parser.add_argument("--health-only", action="store_true")
    args = parser.parse_args()
    if args.requests < 0:
        parser.error("--requests must be >= 0")

    client = XNavDPClient(debug_mode=False)
    try:
        client.verify_backend()
        print("XNavDP ROS2 health: OK")
        print(json.dumps(client.health, indent=2, ensure_ascii=False))
        if args.health_only or args.requests == 0:
            return

        if client.health.get("initialized"):
            client.attach_initialized_backend()
            print("Using initialized XNavDP model; retaining backend calibration and history.")
        else:
            intrinsic = _wait_for_camera_intrinsic(client)
            client.reset(intrinsic)
            print("Initialized XNavDP model with live camera calibration:")
            print(intrinsic)
        # Use the real independent ROS inputs; never inject synthetic /odom or
        # camera messages into the live robot's topics.
        deadline = time.monotonic() + 10
        while client.latest_rgb is None or client.latest_odom is None:
            if time.monotonic() > deadline:
                raise TimeoutError("Waiting for original compressed RGB and ROS /odom")
            time.sleep(.05)
        goal = np.array([3.0, 0.0])
        latencies: list[float] = []
        match_failures = 0
        started = time.monotonic()
        for index in range(args.requests):
            request_started = time.monotonic()
            rgb = client.latest_rgb
            odom = client.latest_odom
            p, q = odom.pose.position, odom.pose.orientation
            position = np.array([p.x, p.y, 0.0])
            quaternion = np.array([q.x, q.y, q.z, q.w])
            stamp = rgb.header.stamp.sec + rgb.header.stamp.nanosec / 1e9
            try:
                trajectory, candidates, values = client.pointgoal_step(
                    None, None, goal, position, quaternion,
                    stamp=stamp, frame_id=odom.header.frame_id,
                )
            except RuntimeError as exc:
                if not str(exc).startswith("No RGB/depth/odom match in 2 s cache within 150 ms"):
                    raise
                match_failures += 1
                print(
                    f"WARNING request {index + 1}/{args.requests}: {exc}; "
                    "skipping this request and continuing with the latest sensors",
                    flush=True,
                )
                continue
            latency = time.monotonic() - request_started
            latencies.append(latency)
            print(
                f"request {index + 1}/{args.requests}: "
                f"{latency * 1000:.1f} ms "
                f"trajectory={trajectory.shape} "
                f"candidates={candidates.shape} values={values.shape}"
            )
        elapsed = time.monotonic() - started
        if not latencies:
            print(
                "XNavDP ROS2 communication: "
                f"requests={args.requests} successful=0 match_failures={match_failures} "
                f"elapsed={elapsed:.3f}s rate=0.000Hz"
            )
            return
        ordered = sorted(latencies)
        p95 = ordered[max(0, int(0.95 * len(ordered)) - 1)]
        print(
            "XNavDP ROS2 communication: "
            f"requests={args.requests} successful={len(latencies)} "
            f"match_failures={match_failures} elapsed={elapsed:.3f}s "
            f"rate={len(latencies) / elapsed:.3f}Hz "
            f"median={statistics.median(latencies) * 1000:.1f}ms "
            f"p95={p95 * 1000:.1f}ms"
        )
    finally:
        client.close()


if __name__ == "__main__":
    main()
