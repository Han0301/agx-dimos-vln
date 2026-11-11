#!/usr/bin/env bash
# Shared rosbag2 environment only; this file starts no ROS participant.
_BAG_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd -P)"
bag_load_settings() {
  local config="${1:-$_BAG_ROOT/config/bag.yaml}" settings
  [[ "$config" == /* ]] || config="$_BAG_ROOT/$config"
  settings="$(/usr/bin/python3 -I - "$_BAG_ROOT" "$config" <<'PY'
from pathlib import Path
import math
import os
import sys
import yaml
root, config = map(Path, sys.argv[1:])
try:
    cfg = yaml.safe_load(config.read_text())
    expected = {'rosbag_setup', 'bag_path', 'rate', 'loop', 'clock_hz'}
    if not isinstance(cfg, dict) or set(cfg) != expected:
        raise ValueError(f'expected config keys: {sorted(expected)}')
    for name in ('rosbag_setup', 'bag_path'):
        if not isinstance(cfg[name], str) or '\n' in cfg[name]:
            raise ValueError(f'{name} must be a single-line path')
    for name in ('rate', 'clock_hz'):
        v = cfg[name]
        if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) or v <= 0:
            raise ValueError(f'{name} must be positive and finite')
    if type(cfg['loop']) is not bool:
        raise ValueError('loop must be boolean')
    setup = Path(os.path.expandvars(cfg['rosbag_setup'])).expanduser()
    setup = (root / setup).resolve() if not setup.is_absolute() else setup.resolve()
    if not cfg['rosbag_setup'] or not setup.is_file():
        raise ValueError(f'ROS Jazzy bag setup missing: {setup}')
    print('\n'.join(map(str, [setup, cfg['bag_path'], cfg['rate'], str(cfg['loop']).lower(), cfg['clock_hz']])))
except (OSError, TypeError, KeyError, ValueError, yaml.YAMLError) as exc:
    sys.exit(f'ERROR: {config}: {exc}')
PY
)" || return $?
  mapfile -t _BAG_SETTINGS <<< "$settings"
}
bag_prepare_environment() {
  unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH ROS_PACKAGE_PATH
  unset LD_LIBRARY_PATH PYTHONPATH PYTHONHOME PKG_CONFIG_PATH VIRTUAL_ENV
  unset AMENT_CURRENT_PREFIX COLCON_CURRENT_PREFIX ROS_LOG_DIR
  unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION
  export PATH=/usr/bin:/bin
  source "${_BAG_SETTINGS[0]}"
  source "$_BAG_ROOT/sh/xnavdp_ros2/dds_env.sh"
  source "$_BAG_ROOT/sh/internal/runtime_log.bash"
}
