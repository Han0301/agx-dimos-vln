"""Body communication lifecycle; cameras, playback and model plugins run independently."""

import importlib
import logging
import signal
import threading

log = logging.getLogger(__name__)
REGISTRY = {
    "body": ("dimos.robot.unitree.go2.body_bridge", "Go2BodyBridge"),
}


def run(config, stop_event=None, max_seconds=None):
    # No robot, ROS or vision SDK import during configuration inspection.
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.signals import SignalHandlerOptions

    stop_event = stop_event or threading.Event()
    context = Context()
    rclpy.init(context=context, signal_handler_options=SignalHandlerOptions.NO)
    try:
        node = Node(
            config["ros"]["node_name"],
            context=context,
            parameter_overrides=[
                Parameter("use_sim_time", value=config["ros"]["use_sim_time"])
            ],
        )
    except Exception:
        if context.ok():
            context.shutdown()
        raise
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    modules = []
    previous = {}
    import time

    started = time.monotonic()
    try:
        if threading.current_thread() is threading.main_thread():
            for sig in (signal.SIGINT, signal.SIGTERM):
                previous[sig] = signal.signal(sig, lambda *_: stop_event.set())
        for name, (module, cls) in REGISTRY.items():
            if config["modules"][name]:
                instance = getattr(importlib.import_module(module), cls)(node, config)
                modules.append(instance)  # cleanup also covers partially failed startup
                instance.start()
                log.info("started module: %s", name)
        while not stop_event.is_set() and context.ok():
            executor.spin_once(timeout_sec=0.1)
            for instance in modules:
                if hasattr(instance, "check"):
                    instance.check()
            if max_seconds is not None and time.monotonic() - started >= max_seconds:
                break
    finally:
        for instance in reversed(modules):
            try:
                instance.stop()
            except Exception:
                log.exception("module cleanup failed")
        executor.shutdown(timeout_sec=2)
        node.destroy_node()
        if context.ok():
            context.shutdown()
        for sig, handler in previous.items():
            signal.signal(sig, handler)
