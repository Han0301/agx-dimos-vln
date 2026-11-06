#!/usr/bin/env bash
set -e
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /home/guest/native_ros_jazzy/setup.bash
source /home/guest/bag5090_bench_20261103/strict_chain/host_config.sh
export XNAVDP_MODEL_ROOT=/home/guest/bag5090_bench_20261103/strict_chain
export ROS_DOMAIN_ID="${XNAVDP_ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY=0 ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export TRACE_OUT="$SCRIPT_DIR/logs"
mkdir -p "$TRACE_OUT"
export FASTRTPS_DEFAULT_PROFILES_FILE="$SCRIPT_DIR/server_dds.xml"
unset ROS_LOCALHOST_ONLY
cd "$XNAVDP_MODEL_ROOT"
exec /usr/bin/python3 -u "$SCRIPT_DIR/server.py" "$@"
