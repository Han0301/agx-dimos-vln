#!/usr/bin/env python3

import json
import os
import signal
import subprocess
import sys
import threading
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image

COLOR_INPUT = '/camera/camera/color/image_raw'
COLOR_OUTPUT = '/camera/camera/color/image_raw/compressed'
DEPTH_INPUT = '/camera/camera/aligned_depth_to_color/image_raw'
DEPTH_OUTPUT = '/camera/camera/aligned_depth_to_color/image_raw/compressed'


class ImageCompressor(Node):
    def __init__(self, stream: str):
        super().__init__(f'image_compressor_{stream}')
        self.stream = stream
        self.bridge = CvBridge()

        self.declare_parameter('color_quality', 80)
        self.declare_parameter('depth_compression_level', 1)
        self.color_quality = int(self.get_parameter('color_quality').value)
        self.depth_compression_level = int(
            self.get_parameter('depth_compression_level').value
        )

        # Match sensor-data QoS. BEST_EFFORT avoids reliable retransmission of
        # multi-megabyte raw frames. The callback only stores the newest message,
        # so this receive window cannot create processing backlog.
        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )
        self._latest = None
        self._latest_lock = threading.Lock()
        self._latest_ready = threading.Event()
        self._stopping = threading.Event()
        self._latest_overwrites = 0
        self._last_rx_mono_ns = None
        self._last_rx_stamp_ns = None
        self._trace_slow_ms = float(os.environ.get('COMPRESSOR_TRACE_SLOW_MS', '0'))
        if stream == 'color':
            self.color_sub = self.create_subscription(
                Image, COLOR_INPUT, self.color_callback, qos
            )
            self.color_pub = self.create_publisher(CompressedImage, COLOR_OUTPUT, qos)
        else:
            self.depth_sub = self.create_subscription(
                Image, DEPTH_INPUT, self.depth_callback, qos
            )
            self.depth_pub = self.create_publisher(CompressedImage, DEPTH_OUTPUT, qos)
        self._processor = threading.Thread(
            target=self._run_processor,
            daemon=True,
            name=f'image_compressor_{stream}_processor',
        )
        self._processor.start()

        self.get_logger().info(
            f'Image compressor worker started: stream={stream}, '
            f'color_quality={self.color_quality}, '
            f'depth_compression_level={self.depth_compression_level}'
        )

    def _image_to_bgr(self, msg: Image) -> np.ndarray:
        encoding = msg.encoding.lower()
        if encoding not in ('rgb8', 'bgr8'):
            return self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        image = np.ndarray(
            shape=(msg.height, msg.width, 3),
            dtype=np.uint8,
            buffer=msg.data,
            strides=(msg.step, 3, 1),
        )
        if encoding == 'bgr8':
            return np.ascontiguousarray(image)
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

    def _image_to_depth(self, msg: Image) -> np.ndarray:
        if msg.encoding.lower() != '16uc1':
            return self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        depth = np.ndarray(
            shape=(msg.height, msg.width),
            dtype=np.uint16,
            buffer=msg.data,
            strides=(msg.step, 2),
        )
        return np.ascontiguousarray(depth)

    def _queue_latest(self, msg: Image) -> None:
        now_ns = time.monotonic_ns()
        stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
        arrival_gap_ns = (
            now_ns - self._last_rx_mono_ns if self._last_rx_mono_ns is not None else 0
        )
        stamp_gap_ns = (
            stamp_ns - self._last_rx_stamp_ns if self._last_rx_stamp_ns is not None else 0
        )
        self._last_rx_mono_ns = now_ns
        self._last_rx_stamp_ns = stamp_ns
        with self._latest_lock:
            if self._latest is not None:
                self._latest_overwrites += 1
            self._latest = (
                now_ns,
                msg,
                self._latest_overwrites,
                arrival_gap_ns,
                stamp_gap_ns,
            )
            self._latest_overwrites = 0
        self._latest_ready.set()

    def _run_processor(self) -> None:
        while not self._stopping.is_set():
            with self._latest_lock:
                item = self._latest
                self._latest = None
                if item is None:
                    self._latest_ready.clear()
            if item is None:
                self._latest_ready.wait(timeout=0.1)
                continue
            callback_ns, msg, overwritten, arrival_gap_ns, stamp_gap_ns = item
            started_ns = time.monotonic_ns()
            convert_started = time.monotonic_ns()
            if self.stream == 'color':
                converted = self._image_to_bgr(msg)
            else:
                converted = self._image_to_depth(msg)
            convert_ns = time.monotonic_ns() - convert_started
            if self.stream == 'color':
                published, encode_ns, publish_ns = self._compress_color(msg, converted)
            else:
                published, encode_ns, publish_ns = self._compress_depth(msg, converted)
            total_ns = time.monotonic_ns() - started_ns
            slow_ns = int(self._trace_slow_ms * 1_000_000)
            if slow_ns and (
                total_ns > slow_ns
                or overwritten
                or arrival_gap_ns > slow_ns
                or stamp_gap_ns > slow_ns
            ):
                self.get_logger().warning('COMPRESSOR_SLOW ' + json.dumps({
                    'stream': self.stream,
                    'stamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                    'wait_ms': round((started_ns - callback_ns) / 1e6, 3),
                    'convert_ms': round(convert_ns / 1e6, 3),
                    'encode_ms': round(encode_ns / 1e6, 3),
                    'publish_ms': round(publish_ns / 1e6, 3),
                    'total_ms': round(total_ns / 1e6, 3),
                    'overwritten': overwritten,
                    'rx_arrival_gap_ms': round(arrival_gap_ns / 1e6, 3),
                    'rx_stamp_gap_ms': round(stamp_gap_ns / 1e6, 3),
                    'published': published,
                }, separators=(',', ':')))

    def color_callback(self, msg: Image):
        self._queue_latest(msg)

    def depth_callback(self, msg: Image):
        self._queue_latest(msg)

    def _compress_color(self, msg: Image, cv_image: np.ndarray) -> tuple[bool, int, int]:
        try:
            encode_started = time.monotonic_ns()
            ok, encoded = cv2.imencode(
                '.jpg',
                cv_image,
                [int(cv2.IMWRITE_JPEG_QUALITY), self.color_quality],
            )
            if not ok:
                raise ValueError('JPEG encoding failed')
            compressed = CompressedImage()
            compressed.header = msg.header
            compressed.format = 'jpeg'
            compressed.data = encoded.tobytes()
            encode_ns = time.monotonic_ns() - encode_started
            publish_started = time.monotonic_ns()
            self.color_pub.publish(compressed)
            return True, encode_ns, time.monotonic_ns() - publish_started
        except Exception as exc:
            self.get_logger().error(f'Color image compression failed: {exc}')
            return False, 0, 0

    def _compress_depth(self, msg: Image, cv_image: np.ndarray) -> tuple[bool, int, int]:
        try:
            encode_started = time.monotonic_ns()
            ok, encoded = cv2.imencode(
                '.png',
                cv_image,
                [
                    int(cv2.IMWRITE_PNG_COMPRESSION), self.depth_compression_level,
                    int(cv2.IMWRITE_PNG_STRATEGY), int(cv2.IMWRITE_PNG_STRATEGY_RLE),
                ],
            )
            if not ok:
                raise ValueError('PNG encoding failed')
            compressed = CompressedImage()
            compressed.header = msg.header
            compressed.format = '16UC1; compressedDepth png'
            compressed.data = encoded.tobytes()
            encode_ns = time.monotonic_ns() - encode_started
            publish_started = time.monotonic_ns()
            self.depth_pub.publish(compressed)
            return True, encode_ns, time.monotonic_ns() - publish_started
        except Exception as exc:
            self.get_logger().error(f'Depth image compression failed: {exc}')
            return False, 0, 0

    def destroy_node(self):
        self._stopping.set()
        self._latest_ready.set()
        self._processor.join(timeout=2.0)
        return super().destroy_node()


def run_worker(stream: str) -> None:
    rclpy.init()
    node = ImageCompressor(stream)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def run_supervisor() -> int:
    workers = [
        subprocess.Popen([sys.executable, '-u', os.path.abspath(__file__), '--worker', stream])
        for stream in ('color', 'depth')
    ]
    stopping = False

    def stop_workers(_signum=None, _frame=None):
        nonlocal stopping
        if stopping:
            return
        stopping = True
        for worker in workers:
            if worker.poll() is None:
                worker.terminate()

    signal.signal(signal.SIGINT, stop_workers)
    signal.signal(signal.SIGTERM, stop_workers)
    while True:
        for stream, worker in zip(('color', 'depth'), workers, strict=True):
            code = worker.poll()
            if code is not None:
                if not stopping:
                    print(f'{stream} compressor exited unexpectedly with code {code}', flush=True)
                stop_workers()
                for pending in workers:
                    try:
                        pending.wait(timeout=2)
                    except subprocess.TimeoutExpired:
                        pending.kill()
                return 0 if stopping and code == 0 else (code or 1)
        time.sleep(0.2)


def main(args=None):
    if len(sys.argv) >= 3 and sys.argv[1] == '--worker':
        stream = sys.argv[2]
        if stream not in ('color', 'depth'):
            raise SystemExit(f'Invalid compressor worker: {stream}')
        run_worker(stream)
        return
    raise SystemExit(run_supervisor())


if __name__ == '__main__':
    main()
