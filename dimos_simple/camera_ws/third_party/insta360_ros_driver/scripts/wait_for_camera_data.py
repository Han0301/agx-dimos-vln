#!/usr/bin/env python3
"""Gate camera startup on USB access or real image messages; optionally monitor gaps."""
import argparse
import math
import os
from pathlib import Path
import signal
import time
from insta360_ros_driver.camera_state import record_error, record_heartbeat


def wait_usb(vendor, timeout):
    deadline = time.monotonic() + timeout
    print(f'Waiting up to {timeout:g}s for USB vendor {vendor} and read/write access...', flush=True)
    while time.monotonic() < deadline:
        for device in Path('/sys/bus/usb/devices').glob('*'):
            try:
                if (device / 'idVendor').read_text().strip().lower() != vendor.lower():
                    continue
                bus = int((device / 'busnum').read_text())
                number = int((device / 'devnum').read_text())
                node = Path(f'/dev/bus/usb/{bus:03d}/{number:03d}')
                if os.access(node, os.R_OK | os.W_OK):
                    print(f'USB ready: {device.name} -> {node}', flush=True)
                    return 0
            except (OSError, ValueError):
                continue
        time.sleep(0.2)
    print('ERROR: camera USB not accessible. Check power, Android mode, cable and permissions.', flush=True)
    return 2


def wait_images(topics, compressed, timeout, watchdog, state_group=None):
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image, CompressedImage
    rclpy.init(args=[])
    node = Node('camera_stream_watchdog' if watchdog else 'wait_camera_images')
    last = {topic: None for topic in topics}
    counts = {topic: 0 for topic in topics}
    previous_counts = dict(counts)
    subscriptions = []
    started = time.monotonic()
    ready = False
    heartbeat_at = started
    run_id = os.environ.get('CAMERA_STATE_RUN_ID')
    msg_type = CompressedImage if compressed else Image
    for topic in topics:
        def callback(message, topic=topic):
            valid = bool(message.data) if compressed else (
                message.width > 0 and message.height > 0 and message.step > 0 and
                len(message.data) == message.step * message.height)
            if valid:
                counts[topic] += 1
                if last[topic] is None:
                    node.get_logger().info(f'First valid frame: {topic}')
                last[topic] = time.monotonic()
        subscriptions.append(node.create_subscription(msg_type, topic, callback, qos_profile_sensor_data))
    node.get_logger().info(f"{'Monitoring' if watchdog else 'Waiting for'} actual frames: {topics}; timeout={timeout:g}s")
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            now = time.monotonic()
            if all(stamp is not None and now - stamp <= min(2.0, timeout) for stamp in last.values()):
                if not ready:
                    ready = True
                    node.get_logger().info('All required image streams are ready')
                if not watchdog:
                    return 0
            if watchdog and state_group and run_id and now - heartbeat_at >= 1.0:
                healthy = all(stamp is not None and now - stamp <= min(2.0, timeout) for stamp in last.values())
                missing = [topic for topic, stamp in last.items() if stamp is None or now - stamp > 2.0]
                detail = '真实图像码流持续接收中。' if healthy else ('等待真实图像：' if not ready else '图像已中断：') + ', '.join(missing)
                record_heartbeat(state_group, run_id, ready=healthy, detail=detail,
                    state='OK' if healthy else ('ERROR' if ready else 'WAIT'),
                    rates={topic: round((counts[topic] - previous_counts[topic]) / (now - heartbeat_at), 1) for topic in topics},
                    ages={topic: None if stamp is None else round(now - stamp, 2) for topic, stamp in last.items()})
                heartbeat_at = now
                previous_counts = dict(counts)
            failed = [topic for topic, stamp in last.items()
                      if now - (stamp if stamp is not None else started) > timeout]
            if failed:
                message = f'No valid frames within {timeout:g}s: {failed}'
                node.get_logger().error(message)
                if state_group and run_id:
                    record_error(state_group, run_id, message, fatal=True)
                return 2
        return 130
    except (KeyboardInterrupt, ExternalShutdownException):
        return 130
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--usb-vendor')
    group.add_argument('--topics', nargs='+')
    group.add_argument('--compressed-topics', nargs='+')
    parser.add_argument('--timeout', type=float, default=30.0)
    parser.add_argument('--watchdog', action='store_true')
    parser.add_argument('--state-group', choices=['D455', 'X3'])
    args = parser.parse_args()
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error('timeout must be positive and finite')
    if args.usb_vendor:
        return wait_usb(args.usb_vendor, args.timeout)
    return wait_images(args.compressed_topics or args.topics,
                       bool(args.compressed_topics), args.timeout, args.watchdog, args.state_group)


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit(130)
