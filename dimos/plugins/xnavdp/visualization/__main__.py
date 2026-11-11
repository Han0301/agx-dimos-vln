"""Independent RViz view using the original XNavDP display style and native results."""

import argparse
import logging
import os
import shlex
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

from dimos.core.config import ROOT
from dimos.plugins.runtime_log import capture, event

logger = logging.getLogger("dimos.plugins.xnavdp.visualization")


def bag_clock_present():
    import rclpy

    rclpy.init()
    node = rclpy.create_node(
        "xnavdp_rviz_clock_check", enable_rosout=False, start_parameter_services=False
    )
    try:
        deadline = time.monotonic() + 1.5
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        return node.count_publishers("/clock") > 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


def stop(process):
    try:
        os.killpg(process.pid, signal.SIGINT)
    except ProcessLookupError:
        pass
    try:
        process.wait(timeout=5)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        process.wait(timeout=5)


def _commands(config, use_sim):
    helpers = Path(__file__).resolve().parent
    ros_args = ["--ros-args", "-p", f"use_sim_time:={use_sim}"]
    commands = [
        [sys.executable, str(helpers / name), *ros_args]
        for name in ("depth_to_cloud.py", "visualization_tf.py", "result_visualizer.py")
    ]
    commands.append(["rviz2", "-d", str(config), *ros_args])
    return commands


def _launch(commands):
    processes = []
    previous = {}
    stopped = threading.Event()
    status = 0

    def interrupt(sig, _):
        nonlocal status
        status = 128 + sig
        stopped.set()

    try:
        for sig in (signal.SIGINT, signal.SIGTERM):
            previous[sig] = signal.signal(sig, interrupt)
        for command in commands:
            processes.append(subprocess.Popen(command, start_new_session=True))
        while not stopped.wait(0.1):
            for process in processes:
                code = process.poll()
                if code is not None:
                    status = code if process is processes[-1] or code else 1
                    stopped.set()
                    break
        return status
    finally:
        for process in reversed(processes):
            stop(process)
        for sig, handler in previous.items():
            signal.signal(sig, handler)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--time-source", choices=["auto", "sim", "wall"], default="auto"
    )
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args(argv)
    config = ROOT / "config/plugins/xnavdp/xnavdp.rviz"
    if args.dry_run:
        use_sim = (
            "<detect /clock>"
            if args.time_source == "auto"
            else str(args.time_source == "sim").lower()
        )
        for command in _commands(config, use_sim):
            print(shlex.join(command))
        return 0

    metadata = {"time_source": args.time_source, "rviz_config": str(config)}
    with capture("rviz", ros=True, metadata=metadata):
        try:
            use_sim = args.time_source == "sim" or (
                args.time_source == "auto" and bag_clock_present()
            )
            event(
                logger, "rviz_clock_selected",
                time_source=args.time_source, use_sim_time=use_sim,
            )
            return _launch(_commands(config, str(use_sim).lower()))
        except Exception as exc:
            event(logger, "rviz_launch_failed", error_type=type(exc).__name__, reason=str(exc))
            raise


if __name__ == "__main__":
    raise SystemExit(main())
