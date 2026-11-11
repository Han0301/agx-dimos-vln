#!/usr/bin/env bash
# Independent existing C++ compressor; does not start cameras or bag playback.
set -eo pipefail
_COMPRESSOR_WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
for _COMPRESSOR_ARG in "$@"; do
  case "$_COMPRESSOR_ARG" in
    --dry-run|-h|--help)
      exec /usr/bin/python3 "$_COMPRESSOR_WS/sh/internal/start_compressor.py" "$@";;
  esac
done
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH ROS_PACKAGE_PATH
unset PYTHONPATH PYTHONHOME LD_LIBRARY_PATH
unset ROS_VERSION ROS_PYTHON_VERSION PKG_CONFIG_PATH COLCON_CURRENT_PREFIX
unset AMENT_CURRENT_PREFIX ROS_LOG_DIR
export PATH=/usr/bin:/bin
export ROS_DISTRO=jazzy
source "$_COMPRESSOR_WS/sh/setup_local.bash"
source "$_COMPRESSOR_WS/install/local_setup.bash"
source "$_COMPRESSOR_WS/config/xnavdp_dds.bash"
if [[ -r "$_COMPRESSOR_WS/../sh/internal/runtime_log.bash" ]]; then
  source "$_COMPRESSOR_WS/../sh/internal/runtime_log.bash"
  dimos_log_start compressor
fi
exec /usr/bin/python3 "$_COMPRESSOR_WS/sh/internal/start_compressor.py" "$@"
