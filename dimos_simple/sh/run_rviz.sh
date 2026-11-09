#!/usr/bin/env bash
set -e
DIMOS_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
XNAVDP_DIR="$DIMOS_DIR/sh/xnavdp_rviz"
if [[ -f "${ROS2_JAZZY:-$HOME/ros2_jazzy}/install/setup.bash" ]]; then
    source "${ROS2_JAZZY:-$HOME/ros2_jazzy}/install/setup.bash"
elif [[ -f /opt/ros/jazzy/setup.bash ]]; then
    source /opt/ros/jazzy/setup.bash
else
    echo 'ROS2 setup.bash not found.' >&2
    exit 1
fi
source "$DIMOS_DIR/sh/xnavdp_ros2/dds_env.sh"
# --bag explicitly selects replay mode; otherwise detect the standard player node.
ROS_TIME_ARGS=()
BAG_MODE=false
if [[ "${1:-}" == --bag ]]; then
    BAG_MODE=true
    shift
elif timeout 4 ros2 node list 2>/dev/null | grep -q rosbag2_player; then
    BAG_MODE=true
fi
if $BAG_MODE; then
    ROS_TIME_ARGS=(--ros-args -p use_sim_time:=true)
    echo 'Bag mode: using /clock (play bag with --clock).'
fi
ODOM_TF_PID=""
if ! $BAG_MODE; then
    "$DIMOS_DIR/.venv/bin/python" "$XNAVDP_DIR/src/odom_visualization_tf.py" &
    ODOM_TF_PID=$!
fi
"$DIMOS_DIR/.venv/bin/python" "$XNAVDP_DIR/src/depth_to_cloud.py" "${ROS_TIME_ARGS[@]}" &
CLOUD_PID=$!
"$DIMOS_DIR/.venv/bin/python" "$XNAVDP_DIR/src/camera_visualization_tf.py" "${ROS_TIME_ARGS[@]}" &
TF_PID=$!
"$DIMOS_DIR/.venv/bin/python" "$XNAVDP_DIR/src/result_visualizer.py" "${ROS_TIME_ARGS[@]}" &
VIS_PID=$!
cleanup() {
    kill "$VIS_PID" "$TF_PID" "$CLOUD_PID" ${ODOM_TF_PID:+"$ODOM_TF_PID"} 2>/dev/null || true
    wait "$VIS_PID" 2>/dev/null || true
    wait "$TF_PID" 2>/dev/null || true
    wait "$CLOUD_PID" 2>/dev/null || true
    if [[ -n "$ODOM_TF_PID" ]]; then wait "$ODOM_TF_PID" 2>/dev/null || true; fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
rviz2 -d "$XNAVDP_DIR/config/xnavdp.rviz" "${ROS_TIME_ARGS[@]}" "$@"
