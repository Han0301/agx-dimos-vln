#!/usr/bin/env python3
"""Tune this workspace's C++ projection, inspired by ai4ce's calibration client.

https://github.com/ai4ce/insta360_ros_driver/blob/humble/scripts/calibrate_cpp.py
Uses the local crop-size convention: square side length, not pixels removed.
"""
from copy import deepcopy
from pathlib import Path
import time

import cv2
import numpy as np
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter, parameter_value_to_python
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
import yaml


NAMES = ['cx_offset', 'cy_offset', 'back_cx_offset', 'back_cy_offset',
         'back_radius_scale', 'crop_size', 'mount_roll_deg', 'translation',
         'rotation_deg', 'gpu', 'out_width', 'out_height']


class CalibrationClient(Node):
    def __init__(self):
        super().__init__('x3_cpp_calibration_client')
        self.declare_parameter('target_node', '/equirectangular_node')
        self.declare_parameter('image_topic', '/equirectangular/image')
        self.declare_parameter('output_path', '')
        self.target = self.get_parameter('target_node').value
        self.output = self.get_parameter('output_path').value
        self.client = AsyncParameterClient(self, self.target)
        if not self.client.wait_for_services(timeout_sec=6.0):
            raise RuntimeError(f'Calibration services unavailable: {self.target}')
        future = self.client.get_parameters(NAMES)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if not future.done() or future.result() is None:
            raise RuntimeError('Could not read current calibration; no defaults applied')
        self.applied = self.decode(future.result())
        self.original = deepcopy(self.applied)
        self.pending = False
        self.dirty = False
        self.ready = False
        self.bridge = CvBridge()
        self.latest = None
        self.grid = True
        self.status = 'Ready: adjust sliders, then A to apply'
        self.window = 'X3 Panorama Calibration'
        self.controls = 'X3 Calibration Controls'
        cv2.namedWindow(self.window, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window, 1200, 600)
        cv2.namedWindow(self.controls, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.controls, 680, 650)
        self.sliders = {}
        self.specs = []
        for name, label in [('cx_offset', 'Front CX'), ('cy_offset', 'Front CY'),
                            ('back_cx_offset', 'Back CX'), ('back_cy_offset', 'Back CY')]:
            self.specs.append((label + ' 0=-100 1000=0', name, None, 10.0, 1000, 2000))
        self.specs.append(('Back radius 200=1.0', 'back_radius_scale', None, 1000.0, -800, 400))
        self.specs.append(('Crop SIDE (oversize = full image)', 'crop_size', None, 1.0, 0, 4096))
        for index, label in enumerate(['TX', 'TY', 'TZ']):
            self.specs.append((label + ' 500=0 step=.001', 'translation', index, 1000.0, 500, 1000))
        for index, label in enumerate(['Roll', 'Pitch', 'Yaw']):
            self.specs.append((label + ' 1800=0 step=.1deg', 'rotation_deg', index, 10.0, 1800, 3600))
        for label, name, index, scale, offset, maximum in self.specs:
            value = self.applied[name] if index is None else self.applied[name][index]
            position = max(0, min(maximum, int(round(value * scale + offset))))
            self.sliders[label] = position
            cv2.createTrackbar(label, self.controls, position, maximum,
                              lambda value, key=label: self.changed(key, value))
        self.ready = True
        self.subscription = self.create_subscription(
            Image, self.get_parameter('image_topic').value,
            self.image_callback, qos_profile_sensor_data)
        self.timer = self.create_timer(0.04, self.gui)
        self.get_logger().info('A apply | S apply and save | R restore opening values | G grid | Q quit')
        self.get_logger().info(f'Save destination: {self.output or "not set"}')

    @staticmethod
    def decode(response):
        values = {name: parameter_value_to_python(value)
                  for name, value in zip(NAMES, response.values)}
        if len(values) != len(NAMES) or any(value is None for value in values.values()):
            raise RuntimeError('Target is missing required local calibration parameters')
        return values

    def changed(self, label, value):
        self.sliders[label] = value
        if self.ready:
            self.dirty = True
            self.status = 'Unapplied changes: A apply | S apply and save'

    def requested(self):
        values = deepcopy(self.applied)
        for label, name, index, scale, offset, _ in self.specs:
            value = (self.sliders[label] - offset) / scale
            if name == 'crop_size':
                value = max(2, int(value))
            if index is None:
                values[name] = value
            else:
                values[name][index] = value
        return values

    def sync_sliders(self, values):
        self.ready = False
        for label, name, index, scale, offset, maximum in self.specs:
            value = values[name] if index is None else values[name][index]
            cv2.setTrackbarPos(label, self.controls,
                              max(0, min(maximum, int(round(value * scale + offset)))))
        self.ready = True
        self.dirty = False

    def apply(self, save=False, restore=False):
        if self.pending:
            return
        values = deepcopy(self.original) if restore else self.requested()
        parameters = [Parameter(name, value=values[name]) for name in NAMES]
        self.pending = True
        self.status = 'Applying calibration...'
        future = self.client.set_parameters_atomically(parameters)
        started = time.monotonic()

        def applied(done):
            try:
                result = done.result().result
                if not result.successful:
                    raise RuntimeError(result.reason)
                readback = self.client.get_parameters(NAMES)
                readback.add_done_callback(verified)
            except Exception as exc:
                self.pending = False
                self.status = f'Apply failed: {exc}'
                self.get_logger().error(self.status)

        def verified(done):
            self.pending = False
            try:
                actual = self.decode(done.result())
                if actual != values:
                    raise RuntimeError('Parameter readback differs; nothing saved')
                self.applied = actual
                self.sync_sliders(actual)
                self.status = 'Applied: inspect BOTH seams at 1/4 and 3/4 width'
                if restore:
                    self.status = 'Restored values from opening this window'
                if save:
                    self.save(actual)
                self.get_logger().info(f'{self.status} ({time.monotonic() - started:.2f}s)')
            except Exception as exc:
                self.status = f'Verification/save failed: {exc}'
                self.get_logger().error(self.status)

        future.add_done_callback(applied)

    def save(self, values):
        if not self.output:
            raise RuntimeError('Set output_path to save a separate YAML profile')
        path = Path(self.output).expanduser()
        path.parent.mkdir(parents=True, exist_ok=True)
        data = {self.target.strip('/'): {'ros__parameters': values}}
        temporary = path.with_suffix(path.suffix + '.tmp')
        temporary.write_text(yaml.safe_dump(data, sort_keys=False), encoding='utf-8')
        temporary.replace(path)
        self.status = f'Saved: {path.name}'

    def image_callback(self, message):
        self.latest = self.bridge.imgmsg_to_cv2(message, 'bgr8')

    def gui(self):
        if self.latest is None:
            preview = np.zeros((600, 1200, 3), dtype='uint8')
            cv2.putText(preview, 'Waiting for /equirectangular/image...', (20, 200),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 1)
        else:
            preview = self.latest.copy()
            height, width = preview.shape[:2]
            if self.grid:
                for y in [height // 4, height // 2, height * 3 // 4]:
                    cv2.line(preview, (0, y), (width - 1, y), (80, 80, 80), 1)
                for x in [width // 4, width * 3 // 4]:
                    cv2.line(preview, (x, 0), (x, height - 1), (0, 255, 255), 1)
        cv2.putText(preview, self.status[:110], (15, 28), cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, (0, 255, 0), 1)
        cv2.putText(preview, 'A apply | S save | R restore | G grid | Q quit',
                    (15, preview.shape[0] - 18), cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, (0, 255, 255), 1)
        cv2.imshow(self.window, preview)
        key = cv2.waitKey(1) & 0xff
        if key == ord('a'):
            self.apply()
        elif key == ord('s'):
            self.apply(save=True)
        elif key == ord('r'):
            self.apply(restore=True)
        elif key == ord('g'):
            self.grid = not self.grid
        elif key == ord('q') or cv2.getWindowProperty(self.window, cv2.WND_PROP_VISIBLE) < 1:
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CalibrationClient()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
