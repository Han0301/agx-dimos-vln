#!/usr/bin/env bash
# Activate feat/vln-local DimOS on AGX
export PATH="$HOME/.local/bin:$PATH"
# clear broken office proxy if present
unset http_proxy https_proxy HTTP_PROXY HTTPS_PROXY ALL_PROXY all_proxy
# Do not inherit an RMW/DDS configuration from an earlier ROS shell. The bridge
# environment below will then restore its own defaults.
unset RMW_IMPLEMENTATION CYCLONEDDS_URI

# Resolve the workspace from this script so the directory can be renamed or
# moved without changing this file.
DIMOS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export DIMOS_ROOT
cd "$DIMOS_ROOT"

if [[ ! -f "$DIMOS_ROOT/.venv/bin/activate" ]]; then
  echo "DimOS virtual environment not found: $DIMOS_ROOT/.venv" >&2
  return 1 2>/dev/null || exit 1
fi
source "$DIMOS_ROOT/.venv/bin/activate"
source "$DIMOS_ROOT/sh/xnavdp_ros2/dds_env.sh"
# ROS/LCM bridge env (X3 color_image etc.)
export ROS2_BRIDGE_ROOT="${ROS2_BRIDGE_ROOT:-$HOME/dimos-ros2-bridge}"
export ROS2_JAZZY="${ROS2_JAZZY:-$HOME/ros2_jazzy}"
if [[ -f "$ROS2_BRIDGE_ROOT/env.sh" ]]; then
  # shellcheck disable=SC1091
  source "$ROS2_BRIDGE_ROOT/env.sh"
fi
# Unitree device credentials are kept outside the repository.
if [[ -f "$HOME/.config/dimos/unitree.env" ]]; then
  source "$HOME/.config/dimos/unitree.env"
fi
# Live VLN services are on the LAN; do not route them through the stale office proxy.
unset http_proxy https_proxy HTTP_PROXY HTTPS_PROXY ALL_PROXY all_proxy
export VLN_CONFIG="${VLN_CONFIG:-$DIMOS_ROOT/dimos/agents/skills/config/vln_config.yaml}"
# Use the XNavDP adapter and 5090 inference host exclusively.
export NAVIGATION_POLICY="xnavdp"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export NAVIGATION_SERVER_URL="ros2:///xnavdp"
export XNAVDP_ROS_DOMAIN_ID="${XNAVDP_ROS_DOMAIN_ID:-0}"
source "$DIMOS_ROOT/sh/xnavdp_ros2/dds_env.sh"
# Direct Ethernet connection to the Go2.
export ROBOT_IP="192.168.123.161"
export DIMOS_ROBOT_IP="$ROBOT_IP"
if git -C "$DIMOS_ROOT" rev-parse --is-inside-work-tree >/dev/null 2>&1; then
  echo "DimOS: $(git -C "$DIMOS_ROOT" rev-parse --abbrev-ref HEAD) @ $(git -C "$DIMOS_ROOT" rev-parse --short HEAD)"
else
  echo "DimOS workspace: $DIMOS_ROOT"
fi
echo "Run VLN: dimos run unitree-go2-vln-local"
