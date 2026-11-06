#!/usr/bin/env python3
"""Check robot LAN signaling and live configured camera frames."""
import base64
import http.client
import ipaddress
import json
import os
import socket
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
from dimos.agents.skills.config import load_vln_config

ip = os.environ.get('DIMOS_ROBOT_IP', os.environ.get('ROBOT_IP', '192.168.123.161'))
interfaces = json.loads(subprocess.check_output(['ip', '-j', 'address', 'show'], text=True))
candidates = []
for iface in interfaces:
    name = iface['ifname']
    if name == 'lo' or 'UP' not in iface.get('flags', []):
        continue
    wireless = Path('/sys/class/net', name, 'wireless').exists() or name.startswith(('wl', 'wlan'))
    for addr in iface.get('addr_info', []):
        if addr.get('family') != 'inet':
            continue
        network = ipaddress.ip_network(f"{addr['local']}/{addr['prefixlen']}", strict=False)
        if ipaddress.ip_address(ip) in network:
            candidates.append((wireless, name, addr['local']))
candidates.sort()
robot_ok = False
for wireless, name, source in candidates:
    print(f"机器人: 尝试{'无线' if wireless else '有线'} {name} ({source}) → {ip}", flush=True)
    for port in (9991, 8081):
        try:
            with socket.create_connection((ip, port), timeout=2, source_address=(source, 0)):
                pass
        except OSError as exc:
            print(f'  端口 {port} 不可用: {exc}', flush=True)
            continue
        if port == 8081:
            print('  旧版 8081 端口可连接；未验证 SDP/WebRTC 握手', flush=True)
            robot_ok = True
            break
        try:
            conn = http.client.HTTPConnection(ip, port, timeout=4, source_address=(source, 0))
            conn.request('POST', '/con_notify', body=b'')
            response = conn.getresponse()
            body = response.read()
            conn.close()
            data = json.loads(base64.b64decode(body, validate=True).decode())
            if response.status != 200 or not isinstance(data, dict) or not data.get('data1'):
                raise ValueError(f'HTTP {response.status} 或响应格式无效')
            print('  9991 con_notify 响应有效；尚未验证完整 WebRTC 握手', flush=True)
            robot_ok = True
            break
        except Exception as exc:
            print(f'  9991 con_notify 失败: {type(exc).__name__}: {exc}', flush=True)
    if robot_ok:
        break
if not robot_ok:
    print(f'✗ 机器人信令检查失败；同网段可用有线/无线地址数={len(candidates)}', flush=True)

cfg = load_vln_config(os.environ['VLN_CONFIG'])
profile = cfg.navdp.trajectory_camera
print(f'相机配置: {profile}', flush=True)
if profile not in ('d455i', 'realsense_d435'):
    print('✗ 此检查仅支持 ROS 压缩图像相机配置，不能确认此配置的相机连接', flush=True)
    sys.exit(1)
cam = cfg.navdp.cameras[profile]
from dimos.hardware.sensors.camera.ros2_compressed_bridge import normalize_compressed_payload, decode_jpeg_to_rgb_image, decode_png_depth_to_image
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage

rclpy.init()
node = Node(f'dimos_hardware_check_{os.getpid()}')
received = {}
errors = {}
def callback(kind, msg):
    try:
        payload = normalize_compressed_payload(msg.data)
        img = (decode_jpeg_to_rgb_image(payload, log_prefix='[check]') if kind == 'rgb'
               else decode_png_depth_to_image(payload, cam.ros2_depth_scale, log_prefix='[check]'))
        if img is None or img.data.size == 0:
            raise ValueError('图像无法解码或为空')
        if kind == 'depth' and not np.any(np.isfinite(img.data) & (img.data > 0)):
            raise ValueError('深度图没有有效深度')
        received[kind] = (img.data.shape, msg.header.frame_id)
    except Exception as exc:
        errors[kind] = str(exc)
subs = [node.create_subscription(CompressedImage, topic, lambda msg, k=kind: callback(k, msg), qos_profile_sensor_data)
        for kind, topic in [('rgb', cam.ros2_rgb_topic), ('depth', cam.ros2_depth_topic)]]
end = time.monotonic() + 12
while time.monotonic() < end and len(received) < 2:
    rclpy.spin_once(node, timeout_sec=0.2)
for kind, topic in [('rgb', cam.ros2_rgb_topic), ('depth', cam.ros2_depth_topic)]:
    if kind in received:
        print(f'✓ {kind}: {topic} shape={received[kind][0]} frame={received[kind][1]}', flush=True)
    else:
        print(f'✗ {kind}: {topic} 未收到有效帧 {errors.get(kind, "")} ', flush=True)
node.destroy_node()
rclpy.shutdown()
if len(received) == 2 and received['rgb'][0][:2] != received['depth'][0][:2]:
    print('✗ RGB 和深度分辨率不一致', flush=True)
    sys.exit(1)
if not robot_ok or len(received) != 2:
    sys.exit(1)
print('✓ 机器人信令及相机数据检查通过（不包含完整 WebRTC、里程计和运动验证）')
