"""Independent simple MPC tracker using minimum's native inputs."""

import argparse
import importlib
import json
import logging
import signal
import sys
import threading
from contextlib import ExitStack
from pathlib import Path

import yaml

from dimos.config import ROOT

from .config import DEFAULT, load_config


def check_dependencies():
    expected = (ROOT / ".venv").resolve()
    if Path(sys.prefix).resolve() != expected:
        raise RuntimeError("Use sh/start_tracker.sh with this workspace .venv")
    for name in ("numpy", "scipy", "yaml"):
        module = importlib.import_module(name)
        if not Path(module.__file__).resolve().is_relative_to(expected):
            raise RuntimeError(
                f"{name} is outside workspace .venv; install with sh/setup/install.sh"
            )
    from scipy.optimize import minimize

    return minimize


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=str(DEFAULT))
    parser.add_argument(
        "--output-only",
        action="store_true",
        default=None,
        help="print final velocity commands only; never create /cmd_vel publisher",
    )
    parser.add_argument(
        "--check",
        action="store_true",
        help="validate config without ROS, SciPy or process replacement",
    )
    args = parser.parse_args(argv)
    stack = ExitStack()
    try:
        cfg = load_config(args.config)
        if args.output_only:
            cfg["output_only"] = True
        if args.check:
            print(json.dumps(cfg, ensure_ascii=False, indent=2))
            return 0
        if argv is None:
            from dimos.plugins.runtime_log import capture

            stack.enter_context(capture("path_tracker", ros=True))
        logging.basicConfig(
            level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s"
        )
        check_dependencies()
        from .session import TrackerSession

        # Set handlers before acquiring the startup gate: replacement can arrive early.
        stopped = threading.Event()
        old_handlers = {s: signal.getsignal(s) for s in (signal.SIGINT, signal.SIGTERM)}

        def stop(*_):
            stopped.set()

        for sig in old_handlers:
            signal.signal(sig, stop)
        try:
            with TrackerSession():
                if stopped.is_set():
                    return 0
                import rclpy
                from rclpy.context import Context
                from rclpy.executors import SingleThreadedExecutor
                from rclpy.node import Node
                from rclpy.signals import SignalHandlerOptions

                from .node import TrackerNode

                context = Context()
                rclpy.init(
                    context=context, signal_handler_options=SignalHandlerOptions.NO
                )
                node = None
                adapter = None
                executor = None
                try:
                    node = Node("path_tracker", context=context)
                    executor = SingleThreadedExecutor(context=context)
                    executor.add_node(node)
                    adapter = TrackerNode(node, cfg)
                    adapter.tracker.start()
                    logging.getLogger("dimos.path_tracker").info(
                        "Started output_only=%s; inputs=%s",
                        cfg["output_only"],
                        cfg["topics"],
                    )
                    while not stopped.is_set() and context.ok():
                        executor.spin_once(timeout_sec=0.2)
                finally:
                    try:
                        if adapter is not None:
                            adapter.tracker.stop()
                    finally:
                        if executor is not None:
                            executor.shutdown(timeout_sec=2)
                        if node is not None:
                            node.destroy_node()
                        context.try_shutdown()
        finally:
            for sig, handler in old_handlers.items():
                signal.signal(sig, handler)
        return 0
    except KeyboardInterrupt:
        return 130
    except (OSError, ValueError, RuntimeError, ImportError, yaml.YAMLError) as exc:
        logging.getLogger("dimos.path_tracker").error("%s", exc)
        return 1
    finally:
        stack.close()


if __name__ == "__main__":
    raise SystemExit(main())
