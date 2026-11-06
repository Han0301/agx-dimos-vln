#!/usr/bin/env bash
# Source this file before building or using ros2 in a terminal.
# The ROS runtime and compatibility packages are bundled under vendor/ so this
# workspace does not depend on another ROS installation or another workspace.
INSTA360_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
export INSTA360_ROOT

# Rebuild a self-contained environment instead of inheriting another ROS
# workspace that happens to be sourced in the caller's shell.
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH ROS_PACKAGE_PATH
unset LD_LIBRARY_PATH PYTHONPATH PYTHONHOME PKG_CONFIG_PATH
unset COLCON_CURRENT_PREFIX AMENT_CURRENT_PREFIX

export ROS_DISTRO="${ROS_DISTRO:-jazzy}"
export ROS_VERSION=2
export ROS_PYTHON_VERSION=3
export PYTHONNOUSERSITE=1

INSTA360_ROS_BASE="${INSTA360_ROOT}/vendor/ros/${ROS_DISTRO}"
INSTA360_COMPAT="${INSTA360_ROOT}/vendor/compat"
INSTA360_ROS_OVERLAY="${INSTA360_COMPAT}/opt/ros/${ROS_DISTRO}"

if [[ ! -d "${INSTA360_ROS_BASE}/lib/python3.12/site-packages" ]]; then
  echo "Bundled ROS runtime not found: ${INSTA360_ROS_BASE}" >&2
  return 1 2>/dev/null || exit 1
fi
if [[ ! -d "${INSTA360_ROS_OVERLAY}" ]]; then
  echo "Bundled ROS compatibility overlay not found: ${INSTA360_ROS_OVERLAY}" >&2
  return 1 2>/dev/null || exit 1
fi

_insta360_prepend_unique() {
  local variable="$1" value="$2" current
  current="${!variable-}"
  if [[ ":${current}:" != *":${value}:"* ]]; then
    printf -v "$variable" '%s%s' "$value" "${current:+:${current}}"
    export "$variable"
  fi
}

_insta360_prepend_unique AMENT_PREFIX_PATH "${INSTA360_ROS_OVERLAY}"
_insta360_prepend_unique AMENT_PREFIX_PATH "${INSTA360_ROS_BASE}"
_insta360_prepend_unique CMAKE_PREFIX_PATH "${INSTA360_ROS_OVERLAY}"
_insta360_prepend_unique CMAKE_PREFIX_PATH "${INSTA360_ROS_BASE}"
_insta360_prepend_unique COLCON_PREFIX_PATH "${INSTA360_ROS_OVERLAY}"
_insta360_prepend_unique COLCON_PREFIX_PATH "${INSTA360_ROS_BASE}"
_insta360_prepend_unique PATH "${INSTA360_ROS_BASE}/bin"
_insta360_prepend_unique PATH "${INSTA360_ROS_OVERLAY}/bin"
_insta360_prepend_unique LD_LIBRARY_PATH "${INSTA360_ROS_OVERLAY}/lib/aarch64-linux-gnu"
_insta360_prepend_unique LD_LIBRARY_PATH "${INSTA360_ROS_OVERLAY}/lib"
_insta360_prepend_unique LD_LIBRARY_PATH "${INSTA360_ROS_BASE}/lib/aarch64-linux-gnu"
_insta360_prepend_unique LD_LIBRARY_PATH "${INSTA360_ROS_BASE}/lib"
_insta360_prepend_unique PYTHONPATH "${INSTA360_ROS_OVERLAY}/lib/python3.12/site-packages"
_insta360_prepend_unique PYTHONPATH "${INSTA360_COMPAT}/usr/lib/python3/dist-packages"
_insta360_prepend_unique PYTHONPATH "${INSTA360_ROS_BASE}/lib/python3.12/site-packages"
_insta360_prepend_unique PKG_CONFIG_PATH "${INSTA360_COMPAT}/usr/lib/aarch64-linux-gnu/pkgconfig"
_insta360_prepend_unique PKG_CONFIG_PATH "${INSTA360_ROS_BASE}/lib/aarch64-linux-gnu/pkgconfig"
export OpenCV_DIR="${INSTA360_COMPAT}/usr/lib/aarch64-linux-gnu/cmake/opencv4"

unset -f _insta360_prepend_unique
export LD_LIBRARY_PATH="${INSTA360_ROOT}/third_party/insta360_ros_driver/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-${INSTA360_ROOT}/log}"
