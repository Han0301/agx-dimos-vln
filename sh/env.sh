#!/usr/bin/env bash
# Source before creating ROS nodes. Camera/DDS environment is the existing one.
_DIMOS_MIN_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
if [[ -d "$_DIMOS_MIN_ROOT/camera_ws/vendor/ros/jazzy" ]]; then
  source "$_DIMOS_MIN_ROOT/camera_ws/sh/setup_local.bash"
elif [[ -f /opt/ros/jazzy/setup.bash ]]; then
  source /opt/ros/jazzy/setup.bash
else
  echo 'ROS Jazzy runtime missing; restore camera_ws/vendor or source ROS Jazzy.' >&2
  return 1 2>/dev/null || exit 1
fi
source "$_DIMOS_MIN_ROOT/sh/xnavdp_ros2/dds_env.sh"
if [[ -f "$_DIMOS_MIN_ROOT/.venv/bin/activate" ]]; then
  source "$_DIMOS_MIN_ROOT/.venv/bin/activate"
fi
if [[ -f "$HOME/.config/dimos/unitree.env" ]]; then
  source "$HOME/.config/dimos/unitree.env"
fi
# Keep server HTTP clients direct even when the parent shell has a proxy.
_DIMOS_NO_PROXY="${no_proxy:-}"
if [[ -n "${NO_PROXY:-}" && "${NO_PROXY:-}" != "$_DIMOS_NO_PROXY" ]]; then
  _DIMOS_NO_PROXY="${_DIMOS_NO_PROXY:+$_DIMOS_NO_PROXY,}$NO_PROXY"
fi
case ",$_DIMOS_NO_PROXY," in
  *,10.10.10.56,*) ;;
  *) _DIMOS_NO_PROXY="${_DIMOS_NO_PROXY:+$_DIMOS_NO_PROXY,}10.10.10.56" ;;
esac
export no_proxy="$_DIMOS_NO_PROXY" NO_PROXY="$_DIMOS_NO_PROXY"
unset _DIMOS_NO_PROXY
export PYTHONPATH="$_DIMOS_MIN_ROOT${PYTHONPATH:+:$PYTHONPATH}"
unset _DIMOS_MIN_ROOT
