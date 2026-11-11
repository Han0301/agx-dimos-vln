#!/usr/bin/env bash
# Record exactly the requested topics into the project's data/bags directory.
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/internal/rosbag_env.bash"
_RECORD_ROOT="$_BAG_ROOT"
usage() {
  cat <<'HELP'
Usage: bash sh/record_bag.sh [--dry-run] /topic [/topic ...]
Record only the supplied topics; Ctrl+C finishes the bag.
Output: data/bags/record_<Asia-Shanghai timestamp>_<pid>/ (MCAP)
ROS bag environment: config/bag.yaml -> rosbag_setup
HELP
}
_RECORD_DRY_RUN=0
_RECORD_TOPICS=()
declare -A _RECORD_SEEN=()
for _RECORD_ARG in "$@"; do
  case "$_RECORD_ARG" in
    -h|--help) usage; exit 0;;
    --dry-run) _RECORD_DRY_RUN=1;;
    *)
      if [[ ! "$_RECORD_ARG" =~ ^/([a-zA-Z_][a-zA-Z0-9_]*)(/[a-zA-Z_][a-zA-Z0-9_]*)*$ ]]; then
        printf 'ERROR: expected an absolute ROS topic name, got: %s\n' "$_RECORD_ARG" >&2
        exit 2
      fi
      if [[ -z "${_RECORD_SEEN[$_RECORD_ARG]+set}" ]]; then
        _RECORD_TOPICS+=("$_RECORD_ARG")
        _RECORD_SEEN[$_RECORD_ARG]=1
      fi
      ;;
  esac
done
if ((${#_RECORD_TOPICS[@]} == 0)); then
  usage >&2
  exit 2
fi
bag_load_settings
_RECORD_SETUP="${_BAG_SETTINGS[0]}"
_RECORD_OUTPUT="$_RECORD_ROOT/data/bags/record_$(TZ=Asia/Shanghai date +%Y%m%d_%H%M%S_%N)_$$"
_RECORD_COMMAND=(ros2 bag record --storage mcap --output "$_RECORD_OUTPUT" --topics "${_RECORD_TOPICS[@]}")
if ((_RECORD_DRY_RUN)); then
  printf 'Bag output: %s\n' "$_RECORD_OUTPUT"
  printf 'ROS bag environment: %s\n' "$_RECORD_SETUP"
  printf 'Command: '; printf '%q ' "${_RECORD_COMMAND[@]}"; printf '\n'
  exit 0
fi
bag_prepare_environment
mkdir -p -- "$_RECORD_ROOT/data/bags"
[[ ! -e "$_RECORD_OUTPUT" ]] || { echo 'ERROR: output already exists; refusing overwrite' >&2; exit 1; }
dimos_log_start bag_record
printf 'Bag output: %s\n' "$_RECORD_OUTPUT"
printf 'Recording requested topics only; Ctrl+C to finish.\n'
exec "${_RECORD_COMMAND[@]}"
