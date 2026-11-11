#!/usr/bin/env bash
# One entry point and dashboard; camera groups run in the background with logs.
set -eo pipefail
STACK_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
STACK_SELF="${STACK_DIR}/start_cameras.sh"
WORKSPACE_DIR="$(cd -- "${STACK_DIR}/.." && pwd -P)"
CAMERA_RUNTIME_LOG="${WORKSPACE_DIR}/../sh/internal/runtime_log.bash"
source "${WORKSPACE_DIR}/config/camera_stack.conf"
# The original compressed publishers must be visible to the XNavDP server.
if [[ -f "${WORKSPACE_DIR}/config/xnavdp_dds.bash" ]]; then
  source "${WORKSPACE_DIR}/config/xnavdp_dds.bash"
fi

state_line() {
  local state="$1" message="$2" code="" badge=" STATE: $1 "
  case "$state" in
    OK) code='1;30;42';;
    ERROR) code='1;97;41';;
    WAIT) code='1;30;43';;
    STOP) code='1;30;47';;
  esac
  if [[ -z "${NO_COLOR+x}" && "${FORCE_COLOR:-}" != 0 ]] && { [[ -t 1 && "${TERM:-}" != dumb ]] || [[ -n "${FORCE_COLOR:-}" ]]; }; then
    printf '\033[%sm%s\033[0m %s\n' "$code" "$badge" "$message"
  else
    printf '%s %s\n' "$badge" "$message"
  fi
}

commands() {
  D455_CMD=(ros2 launch "${WORKSPACE_DIR}/launch/start_d455.launch.py"
    "color_profile:=${COLOR_PROFILE}" "depth_profile:=${DEPTH_PROFILE}"
    "python:=${PYTHON_BIN}" "startup_timeout_sec:=${D455_STARTUP_TIMEOUT}"
    "stream_timeout_sec:=${STREAM_TIMEOUT}")
  D455_COMPRESSED_CMD=(ros2 launch "${WORKSPACE_DIR}/launch/compressed_d455.launch.py"
    "color_quality:=80" "depth_compression_level:=1"
    "stats_period_s:=10.0" "trace_slow_ms:=0.0")
  X3_CMD=(ros2 launch insta360_ros_driver bringup.launch.xml "equirectangular:=${X3_EQUIRECTANGULAR:-true}" "surround:=${X3_SURROUND:-true}" "imu_filter:=${X3_IMU_FILTER:-false}"
    "equirectangular_config:=${X3_CONFIG}" "usb_timeout_sec:=${X3_USB_TIMEOUT}"
    "startup_timeout_sec:=${X3_STARTUP_TIMEOUT}" "stream_timeout_sec:=${STREAM_TIMEOUT}")
}

print_dry_run() {
  local group
  for group in "$@"; do
    case "$group" in
      d455)
        printf 'D455: '; printf '%q ' "${D455_CMD[@]}"; printf '\n'
        printf 'D455 compress: '; printf '%q ' "${D455_COMPRESSED_CMD[@]}"; printf '\n'
        ;;
      x3) printf 'X3: '; printf '%q ' "${X3_CMD[@]}"; printf '\n';;
    esac
  done
}

usage() {
  cat <<EOF
Usage:
  bash start_cameras.sh                         # start D455 and X3
  bash start_cameras.sh --camera x3             # start X3 only
  bash start_cameras.sh --camera d455           # restart D455 in a new session
  bash start_cameras.sh --camera x3 d455        # start both
  bash start_cameras.sh --status                # read-only status dashboard
  bash start_cameras.sh --status --groups D455  # status for a selected camera
  bash start_cameras.sh --camera x3 --dry-run   # print the selected launch command
EOF
}

guard_existing() {
  "$PYTHON_BIN" - "$1" <<'PY'
import sys,time
import rclpy
from rclpy.node import Node
from insta360_ros_driver.console_status import status_line
rclpy.init(args=[])
node=Node('camera_stack_duplicate_check')
deadline=time.monotonic()+1.5
while time.monotonic()<deadline:
    rclpy.spin_once(node,timeout_sec=0.1)
full=[(namespace.rstrip('/')+'/'+name) for name,namespace in node.get_node_names_and_namespaces()]
if sys.argv[1]=='d455':
    existing=[name for name in full if name in ['/camera/camera','/compressed_rgbd_node','/image_compressor','/image_compressor_node']]
else:
    existing=[name for name in full if name in ['/insta360_ros_driver','/image_decoder','/equirectangular_node','/surround_views']]
node.destroy_node()
rclpy.shutdown()
if existing:
    print(status_line(sys.argv[1].upper(),'ERROR','Already running: '+', '.join(existing)),flush=True)
    print('Stop the old camera group first. No duplicate instance was started.',flush=True)
    raise SystemExit(2)
PY
}

run_group() {
  source "${INSTA_WORKSPACE}/sh/setup_local.bash" || return $?
  source "${INSTA_WORKSPACE}/install/local_setup.bash" || return $?
  source "${WORKSPACE_DIR}/config/xnavdp_dds.bash"
  # ROS child stdout is captured by launch; preserve color inside the terminal window.
  if [[ -t 1 && -z "${NO_COLOR+x}" ]]; then export FORCE_COLOR="${FORCE_COLOR:-1}"; fi
  cd "$INSTA_WORKSPACE" || return $?
  if [[ -n "${CAMERA_STACK_SESSION:-}" ]]; then
    "$PYTHON_BIN" - "$1" "$CAMERA_STACK_SESSION" "$BASHPID" <<'PY'
import sys
from insta360_ros_driver.camera_state import register_wrapper
raise SystemExit(0 if register_wrapper(sys.argv[1].upper(),sys.argv[2],int(sys.argv[3])) else 130)
PY
    [[ "$?" == 0 ]] || return 130
  fi
  state_line WAIT "Checking ${1^^} startup and existing nodes..."
  guard_existing "$1" || return $?
  if [[ "$1" == d455 ]]; then
    run_d455_group
  else
    "${X3_CMD[@]}"
  fi
}

run_d455_group() {
  local d455_pid compressor_pid status
  "${D455_CMD[@]}" &
  d455_pid=$!

  set +e
  "$PYTHON_BIN" -u \
    "${INSTA_WORKSPACE}/install/insta360_ros_driver/lib/insta360_ros_driver/wait_for_camera_data.py" \
    --topics \
      /camera/camera/color/image_raw \
      /camera/camera/aligned_depth_to_color/image_raw \
    --timeout "${D455_STARTUP_TIMEOUT}"
  status=$?
  set -e
  if ((status != 0)); then
    kill -INT "$d455_pid" 2>/dev/null || true
    wait "$d455_pid" 2>/dev/null || true
    return "$status"
  fi

  "${D455_COMPRESSED_CMD[@]}" &
  compressor_pid=$!
  set +e
  wait -n "$d455_pid" "$compressor_pid"
  status=$?
  kill -INT "$d455_pid" "$compressor_pid" 2>/dev/null || true
  wait "$d455_pid" 2>/dev/null || true
  wait "$compressor_pid" 2>/dev/null || true
  set -e
  return "$status"
}

run_dashboard() {
  source "${INSTA_WORKSPACE}/sh/setup_local.bash"
  source "${INSTA_WORKSPACE}/install/local_setup.bash"
  exec "$PYTHON_BIN" -u "${INSTA_WORKSPACE}/install/insta360_ros_driver/lib/insta360_ros_driver/camera_dashboard.py" "$@"
}

start_background() {
  local group="$1" wrapper_pid
  setsid --wait env "CAMERA_STACK_SESSION=$controller_session" \
    bash "$STACK_SELF" "--$group" < /dev/null > "$CAMERA_STACK_LOG_DIR/${group}.log" 2>&1 &
  wrapper_pid=$!
  "$PYTHON_BIN" - "$group" "$controller_session" "$wrapper_pid" <<'PY_REGISTER'
import sys
from insta360_ros_driver.camera_state import register_wrapper
raise SystemExit(0 if register_wrapper(sys.argv[1].upper(),sys.argv[2],int(sys.argv[3])) else 130)
PY_REGISTER
}

cleanup_controller() {
  trap '' INT TERM HUP
  "$PYTHON_BIN" - "$controller_session" <<'PY'
import sys
from insta360_ros_driver.camera_state import stop_session
stop_session(sys.argv[1])
PY
}

start_selected_groups() {
  local group dashboard_groups="" log_owner=$BASHPID
  command -v setsid >/dev/null || { echo 'Missing setsid (util-linux).' >&2; exit 1; }
  source "${INSTA_WORKSPACE}/sh/setup_local.bash"
  source "${INSTA_WORKSPACE}/install/local_setup.bash"
  source "${WORKSPACE_DIR}/config/xnavdp_dds.bash"

  for group in "$@"; do
    if [[ "$group" == d455 ]]; then
      state_line WAIT 'Stopping any previous local D455 group before starting this session...' 
      "$PYTHON_BIN" "${STACK_DIR}/internal/restart_d455.py" "$WORKSPACE_DIR"
    fi
    if ! guard_existing "$group"; then
      state_line ERROR "Old ${group^^} camera group is still running. Stop it first, or use --status for read-only monitoring."
      exit 2
    fi
    dashboard_groups+="${dashboard_groups:+,}${group^^}"
  done

  controller_session="camera-${BASHPID}-${RANDOM}-${RANDOM}"
  umask 077
  CAMERA_STACK_LOG_DIR=""
  if [[ -z "${CAMERA_STACK_LOG_ROOT:-}" && -r "$CAMERA_RUNTIME_LOG" ]]; then
    source "$CAMERA_RUNTIME_LOG"
    CAMERA_STACK_LOG_DIR="$(dimos_log_allocate cameras '' "$log_owner")" || CAMERA_STACK_LOG_DIR=""
  fi
  if [[ -z "$CAMERA_STACK_LOG_DIR" ]]; then
    CAMERA_STACK_LOG_DIR="${CAMERA_STACK_LOG_ROOT:-${WORKSPACE_DIR}/log}/$(date +%Y%m%d_%H%M%S_%N)"
  fi
  mkdir -p -- "$CAMERA_STACK_LOG_DIR/ros"
  CAMERA_STACK_LOG_DIR="$(cd -- "$CAMERA_STACK_LOG_DIR" && pwd -P)"
  export CAMERA_STACK_LOG_DIR
  export ROS_LOG_DIR="$CAMERA_STACK_LOG_DIR/ros"
  state_line WAIT "Camera logs: $CAMERA_STACK_LOG_DIR"
  # Cover Ctrl+C/failure during background startup, before the dashboard takes over.
  trap cleanup_controller EXIT
  trap 'exit 130' INT
  trap 'exit 143' TERM
  trap 'exit 129' HUP
  for group in "$@"; do
    start_background "$group"
  done
  run_dashboard --session-id "$controller_session" --groups "$dashboard_groups"
}

commands
case "${1:-}" in
  -h|--help)
    usage
    ;;
  --camera)
    shift
    camera_groups=()
    while [[ $# -gt 0 && "$1" != --* ]]; do
      case "${1,,}" in
        x3) camera_group=x3;;
        d455) camera_group=d455;;
        *)
          echo "Unknown camera '${1}'. Use x3, d455, or both." >&2
          exit 2
          ;;
      esac
      duplicate=0
      for existing_group in "${camera_groups[@]}"; do
        [[ "$existing_group" == "$camera_group" ]] && duplicate=1
      done
      [[ "$duplicate" == 1 ]] || camera_groups+=("$camera_group")
      shift
    done
    if [[ ${#camera_groups[@]} -eq 0 ]]; then
      echo 'Missing camera name. Use --camera x3, --camera d455, or --camera x3 d455.' >&2
      exit 2
    fi
    dry_run=0
    if [[ $# -eq 1 && "$1" == "--dry-run" ]]; then
      dry_run=1
      shift
    fi
    if [[ $# -gt 0 ]]; then
      echo "Unexpected argument after --camera selection: $1" >&2
      exit 2
    fi
    if [[ "$dry_run" == 1 ]]; then
      print_dry_run "${camera_groups[@]}"
    else
      start_selected_groups "${camera_groups[@]}"
    fi
    ;;
  --status|--status-demo)
    shift
    run_dashboard "$@"
    ;;
  --color-demo)
    state_line WAIT 'Demo: waiting for camera frames'
    state_line OK 'Demo: camera frames are ready'
    state_line ERROR 'Demo: startup failed or camera stream lost'
    state_line STOP 'Demo: stopped by user'
    ;;
  --dry-run)
    print_dry_run d455 x3
    ;;
  --d455|--x3)
    group="${1#--}"
    if [[ -z "${CAMERA_STACK_SESSION:-}" && -r "$CAMERA_RUNTIME_LOG" ]]; then
      source "$CAMERA_RUNTIME_LOG"
      dimos_log_start "camera_$group"
    fi
    set +e
    run_group "$group"
    status=$?
    if [[ "$status" == 0 || "$status" == 130 ]]; then
      state_line STOP "${group^^} stopped, exit code: $status"
    else
      state_line ERROR "${group^^} failed, exit code: $status; see the error above."
    fi
    # Controller-owned groups do not read the terminal; standalone debug runs can pause.
    if [[ -t 0 && -z "${CAMERA_STACK_SESSION:-}" ]]; then read -r -p 'Press Enter to close this window...' _; fi
    exit "$status"
    ;;
  '')
    start_selected_groups d455 x3
    ;;
  *)
    usage >&2
    exit 2
    ;;
esac
