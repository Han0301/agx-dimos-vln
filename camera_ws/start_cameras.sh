#!/usr/bin/env bash
set -e
WORKSPACE_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
# Avoid mixing packages and libraries from a previously sourced ROS overlay.
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH ROS_PACKAGE_PATH
unset PYTHONPATH PYTHONHOME LD_LIBRARY_PATH
unset ROS_VERSION ROS_PYTHON_VERSION PKG_CONFIG_PATH COLCON_CURRENT_PREFIX
unset AMENT_CURRENT_PREFIX ROS_LOG_DIR
export PATH=/usr/bin:/bin
export ROS_DISTRO=jazzy
exec bash "${WORKSPACE_DIR}/sh/start_cameras.sh" "$@"
