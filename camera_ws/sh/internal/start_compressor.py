"""Select the existing C++ compressor's clock from the active ROS input source."""

import argparse
import os
import shlex
import time
from pathlib import Path

COMPRESSED = (
    "/camera/camera/color/image_raw/compressed",
    "/camera/camera/aligned_depth_to_color/image_raw/compressed",
)


def inspect_source(time_source="auto"):
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from rclpy.parameter_client import AsyncParameterClient
    from rclpy.qos import ReliabilityPolicy, qos_profile_sensor_data
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import CompressedImage

    context = Context()
    rclpy.init(context=context)
    node = None
    executor = None
    seen = set()
    stamps = {}

    def receive(topic, stamp):
        seen.add(topic)
        stamps[topic] = stamp.sec + stamp.nanosec / 1e9

    try:
        node = rclpy.create_node(
            "compressor_input_check",
            context=context,
            enable_rosout=False,
            start_parameter_services=False,
        )
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        subscriptions = [
            node.create_subscription(
                Clock,
                "/clock",
                lambda msg: receive("/clock", msg.clock),
                qos_profile_sensor_data,
            )
        ]
        for topic in COMPRESSED:
            subscriptions.append(
                node.create_subscription(
                    CompressedImage,
                    topic,
                    lambda msg, topic=topic: receive(topic, msg.header.stamp),
                    qos_profile_sensor_data,
                )
            )
        raw_topics = (
            "/camera/camera/color/image_raw",
            "/camera/camera/aligned_depth_to_color/image_raw",
        )
        started = time.monotonic()
        announced = False
        while True:
            executor.spin_once(timeout_sec=0.1)
            raw_endpoints = [
                node.get_publishers_info_by_topic(topic) for topic in raw_topics
            ]
            compressed_endpoints = [
                node.get_publishers_info_by_topic(topic) for topic in COMPRESSED
            ]
            bag_input = any(
                endpoint.node_name.startswith("rosbag2_player")
                for endpoints in raw_endpoints
                for endpoint in endpoints
            )
            has_clock = bool(
                "/clock" in seen or node.count_publishers("/clock") or bag_input
            )
            # Allow discovery to settle. Auto never assumes wall time with no source.
            native_compressed_input = all(compressed_endpoints) and any(
                endpoint.node_name != "compressed_rgbd_tx"
                for endpoints in compressed_endpoints
                for endpoint in endpoints
            )
            ready = has_clock or all(raw_endpoints) or native_compressed_input
            if time.monotonic() - started >= 2 and (ready or time_source != "auto"):
                break
            if not ready and not announced and time.monotonic() - started >= 2:
                print(
                    "Waiting for RGB-D input or bag /clock; no wall-clock fallback.",
                    flush=True,
                )
                announced = True

        if any(len(endpoints) > 1 for endpoints in compressed_endpoints):
            raise RuntimeError(
                "Multiple compressed publishers exist; stop duplicate sources first"
            )
        existing = [
            topic
            for topic, endpoints in zip(COMPRESSED, compressed_endpoints)
            if endpoints
        ]
        if existing and len(existing) != len(COMPRESSED):
            raise RuntimeError(
                "Only one compressed input topic exists; refusing duplicate output"
            )
        desired_sim = time_source == "sim" or (time_source == "auto" and has_clock)
        known_cpp = all(compressed_endpoints) and all(
            endpoint.node_name == "compressed_rgbd_tx"
            for endpoints in compressed_endpoints
            for endpoint in endpoints
        )
        if known_cpp:
            # Reuse the native compressor only after correcting its clock. This
            # covers a still-running live instance when the input changes to bag.
            namespaces = {
                endpoint.node_namespace
                for endpoints in compressed_endpoints
                for endpoint in endpoints
            }
            if len(namespaces) != 1:
                raise RuntimeError(
                    "Multiple compressor namespaces exist; refusing ambiguous clock update"
                )
            remote = next(iter(namespaces)).rstrip("/") + "/compressed_rgbd_node"
            client = AsyncParameterClient(node, remote)
            if not client.wait_for_services(timeout_sec=3):
                raise RuntimeError(
                    f"Cannot inspect existing compressor clock at {remote}; stop the old compressor first"
                )
            future = client.get_parameters(["use_sim_time"])
            executor.spin_until_future_complete(future, timeout_sec=3)
            if not future.done() or future.exception():
                raise RuntimeError("Existing compressor clock query timed out")
            current = future.result().values[0].bool_value
            if current != desired_sim:
                future = client.set_parameters(
                    [Parameter("use_sim_time", value=desired_sim)]
                )
                executor.spin_until_future_complete(future, timeout_sec=3)
                if (
                    not future.done()
                    or future.exception()
                    or not all(result.successful for result in future.result().results)
                ):
                    raise RuntimeError(
                        "Existing compressor refused the required use_sim_time setting"
                    )
                print(
                    f"Existing compressor corrected: use_sim_time={str(desired_sim).lower()}",
                    flush=True,
                )
                stamps.clear()
                seen.clear()
            else:
                print(
                    f"Existing compressor verified: use_sim_time={str(desired_sim).lower()}",
                    flush=True,
                )

        if existing:
            deadline = time.monotonic() + 5
            while time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.1)
                if all(topic in stamps for topic in COMPRESSED):
                    if not desired_sim:
                        break
                    if (
                        "/clock" in stamps
                        and stamps["/clock"] > 0
                        and all(
                            stamps[topic] > 0
                            and abs(stamps[topic] - stamps["/clock"]) < 1.0
                            for topic in COMPRESSED
                        )
                    ):
                        break
            else:
                raise RuntimeError(
                    "Existing compressed RGB-D does not match the required clock; no duplicate started. Check playback/producer timestamps"
                )
        input_reliable = (
            all(
                endpoint.qos_profile.reliability == ReliabilityPolicy.RELIABLE
                for endpoints in raw_endpoints
                for endpoint in endpoints
            )
            if all(raw_endpoints)
            else None
        )
        return (
            has_clock,
            bool(existing),
            input_reliable,
        )

    finally:
        if executor is not None:
            executor.shutdown(timeout_sec=2)
        if node is not None:
            node.destroy_node()
        if context.ok():
            context.try_shutdown()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--time-source", choices=["auto", "sim", "wall"], default="auto"
    )
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args(argv)
    workspace = Path(__file__).resolve().parents[2]
    try:
        if args.dry_run:
            selected = {
                "sim": "true",
                "wall": "false",
                "auto": "<detect bag publisher or /clock; wait for input>",
            }[args.time_source]
            print(
                "Compressor only; verify/correct an existing compressor clock before reuse."
            )
            print("use_sim_time:", selected)
            print("Launch:", workspace / "launch/compressed_d455.launch.py")
            return 0
        has_clock, already_compressed, input_reliable = inspect_source(args.time_source)
        if already_compressed:
            print(
                "Existing compressed RGB-D input is active; no compressor started.",
                flush=True,
            )
            return 0
        use_sim = args.time_source == "sim" or (
            args.time_source == "auto" and has_clock
        )
        if input_reliable is None:
            input_reliable = not use_sim
        command = [
            "ros2",
            "launch",
            str(workspace / "launch/compressed_d455.launch.py"),
            f"use_sim_time:={str(use_sim).lower()}",
            f"input_reliable:={str(input_reliable).lower()}",
        ]
        print("Compressor only:", shlex.join(command), flush=True)
        os.execvp(command[0], command)
    except KeyboardInterrupt:
        return 130
    except (OSError, RuntimeError, ImportError) as exc:
        parser.exit(1, f"ERROR: {exc}\n")


if __name__ == "__main__":
    raise SystemExit(main())
