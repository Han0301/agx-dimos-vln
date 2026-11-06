#!/usr/bin/env bash
# Run from any directory. Optional first argument: robot IP.
set -e
DIMOS_BODY_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$DIMOS_BODY_ROOT/sh/activate_vln.sh" >/dev/null
GO2_BODY_IP="${1:-192.168.123.161}"
python -m dimos.robot.unitree.go2.wired_link "$GO2_BODY_IP"
# Apply only missing LCM prerequisites before DimOS reaches its y/N prompt.
# Uses normal sudo authentication when needed; no password or sudo rule is stored.
python - <<'PY'
import subprocess
from dimos.protocol.service.system_configurator.lcm import (
    MulticastConfiguratorLinux, BufferConfiguratorLinux,
)

multicast = MulticastConfiguratorLinux(loopback_interface='lo')
if not multicast.check():
    if not multicast.loopback_ok:
        subprocess.run(['sudo', '/usr/sbin/ip', 'link', 'set', 'lo', 'multicast', 'on'], check=True)
    if not multicast.route_ok:
        subprocess.run(['sudo', '/usr/sbin/ip', 'route', 'add', '224.0.0.0/4', 'dev', 'lo'], check=True)
buffers = BufferConfiguratorLinux()
if not buffers.check():
    for key, value in buffers.needs:
        subprocess.run(['sudo', '/usr/sbin/sysctl', '-w', f'{key}={value}'], check=True)
if not multicast.check() or not buffers.check():
    raise SystemExit('Go2 body LCM prerequisites failed; refusing to start.')
print('Go2 body: multicast and LCM receive buffers ready.', flush=True)
PY
exec dimos --robot-ip "$GO2_BODY_IP" --viewer none run unitree-go2-body-ros
