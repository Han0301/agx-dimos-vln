#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd -P)"
set +u
source "${WORKSPACE_DIR}/sh/setup_local.bash"
source "${WORKSPACE_DIR}/install/local_setup.bash"
set -u

failures=0
fail() {
  printf 'FAIL: %s\n' "$*" >&2
  failures=$((failures + 1))
}

for path in \
  "${WORKSPACE_DIR}/vendor/ros/jazzy" \
  "${WORKSPACE_DIR}/vendor/compat/opt/ros/jazzy" \
  "${WORKSPACE_DIR}/install/local_setup.bash" \
  "${WORKSPACE_DIR}/third_party/insta360_ros_driver/lib/libCameraSDK.so"; do
  [[ -e "$path" ]] || fail "required path is missing: $path"
done

for variable in AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH; do
  IFS=: read -r -a entries <<< "${!variable-}"
  for entry in "${entries[@]}"; do
    [[ -z "$entry" || "$entry" == "${WORKSPACE_DIR}/"* ]] ||
      fail "$variable contains external path: $entry"
  done
done

ros2_path="$(command -v ros2 || true)"
[[ "$ros2_path" == "${WORKSPACE_DIR}/"* ]] ||
  fail "ros2 resolves outside the workspace: ${ros2_path:-not found}"

python3 - <<'PY' || fail "Python ROS modules do not resolve inside the workspace"
import os
from pathlib import Path
import launch
import launch_ros
import rclpy
workspace = Path(os.environ["INSTA360_ROOT"]).resolve()
for module in (launch, launch_ros, rclpy):
    path = Path(module.__file__).resolve()
    if workspace not in path.parents:
        raise SystemExit(f"{module.__name__} resolves outside the workspace: {path}")
PY

for package in insta360_ros_driver realsense2_camera launch launch_ros; do
  prefix="$(ros2 pkg prefix "$package" 2>/dev/null || true)"
  [[ "$prefix" == "${WORKSPACE_DIR}/"* ]] ||
    fail "package '$package' resolves outside the workspace: ${prefix:-not found}"
done

binary_paths=(
  "${WORKSPACE_DIR}/install/insta360_ros_driver/lib/insta360_ros_driver/insta360_ros_driver"
  "${WORKSPACE_DIR}/install/insta360_ros_driver/lib/insta360_ros_driver/decoder"
  "${WORKSPACE_DIR}/install/insta360_ros_driver/lib/insta360_ros_driver/equirectangular_cpp"
  "${WORKSPACE_DIR}/install/insta360_ros_driver/lib/insta360_ros_driver/surround_views"
  "${WORKSPACE_DIR}/install/realsense2_camera/lib/librealsense2_camera.so"
)
while IFS= read -r line; do
  if [[ "$line" == *"=> /opt/ros/"* ||
        ( "$line" == *"=> ${HOME}/"* && "$line" != *"=> ${WORKSPACE_DIR}/"* ) ||
        "$line" == *"not found"* ]]; then
    fail "external or unresolved library: $line"
  fi
done < <(ldd "${binary_paths[@]}" 2>&1)

ros_log_dir="$(mktemp -d /tmp/camera-standalone-check.XXXXXX)"
trap 'rm -rf -- "$ros_log_dir"' EXIT
ROS_LOG_DIR="$ros_log_dir" ros2 launch "${WORKSPACE_DIR}/launch/start_d455.launch.py" --show-args >/dev/null ||
  fail "D455 launch description cannot be loaded"
ROS_LOG_DIR="$ros_log_dir" ros2 launch "${WORKSPACE_DIR}/launch/compressed_d455.launch.py" --show-args >/dev/null ||
  fail "C++ compressor launch description cannot be loaded"
ROS_LOG_DIR="$ros_log_dir" ros2 launch insta360_ros_driver \
  bringup.launch.xml --show-args >/dev/null ||
  fail "X3 launch description cannot be loaded"

if ((failures)); then
  printf 'Standalone check failed with %d issue(s).\n' "$failures" >&2
  exit 1
fi
printf 'Standalone check passed: runtime, Python modules, packages and libraries are workspace-local.\n'
