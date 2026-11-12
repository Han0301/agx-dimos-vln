#!/usr/bin/env bash
# Optional XNavDP visualization only; no inference, sensor or playback startup.
set -eo pipefail
_RVIZ_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
_RVIZ_MODULE=dimos.plugins.xnavdp.visualization
if [[ ${1:-} == --memory-view ]]; then
  _RVIZ_MODULE=dimos.vln_task.rviz
  shift
fi
for _RVIZ_ARG in "$@"; do
  case "$_RVIZ_ARG" in
    --dry-run|-h|--help)
      export PYTHONPATH="$_RVIZ_ROOT${PYTHONPATH:+:$PYTHONPATH}"
      exec /usr/bin/python3 -m "$_RVIZ_MODULE" "$@";;
  esac
done
_RVIZ_SETUP="$(/usr/bin/python3 -I - "$_RVIZ_ROOT" <<'PY'
from pathlib import Path
import os, sys, yaml
root = Path(sys.argv[1])
cfg = yaml.safe_load((root/'config/plugins/xnavdp/rviz.yaml').read_text())
path = Path(os.path.expandvars(cfg['ros_setup'])).expanduser()
path = (root/path).resolve() if not path.is_absolute() else path.resolve()
if not path.is_file(): sys.exit(f'RViz ROS setup missing: {path}')
print(path)
PY
)"
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH ROS_PACKAGE_PATH
unset PYTHONPATH PYTHONHOME LD_LIBRARY_PATH VIRTUAL_ENV
unset ROS_VERSION ROS_PYTHON_VERSION PKG_CONFIG_PATH COLCON_CURRENT_PREFIX
unset AMENT_CURRENT_PREFIX ROS_LOG_DIR ROS_DISTRO
export PATH=/usr/bin:/bin
source "$_RVIZ_SETUP"
source "$_RVIZ_ROOT/sh/xnavdp_ros2/dds_env.sh"
export PYTHONPATH="$_RVIZ_ROOT${PYTHONPATH:+:$PYTHONPATH}"
exec /usr/bin/python3 -m "$_RVIZ_MODULE" "$@"
