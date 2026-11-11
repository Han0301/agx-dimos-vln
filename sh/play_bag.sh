#!/usr/bin/env bash
# Replay recorded topics and /clock only. No compressor, camera or model startup.
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/internal/rosbag_env.bash"
usage() {
  echo 'Usage: bash sh/play_bag.sh --bag PATH [--rate N] [--loop|--no-loop] [--dry-run] [--bag-config PATH]'
  echo 'Replays all recorded topics plus /clock; other modules run independently.'
}
_PLAY_BAG='' _PLAY_RATE='' _PLAY_LOOP='' _PLAY_CONFIG='' _PLAY_DRY=0
while (($#)); do
  case "$1" in
    --bag|--rate|--bag-config)
      [[ $# -ge 2 && "$2" != --* ]] || { echo "ERROR: missing value for $1" >&2; exit 2; }
      case "$1" in --bag) _PLAY_BAG="$2";; --rate) _PLAY_RATE="$2";; --bag-config) _PLAY_CONFIG="$2";; esac
      shift 2;;
    --loop) _PLAY_LOOP=true; shift;;
    --no-loop) _PLAY_LOOP=false; shift;;
    --dry-run) _PLAY_DRY=1; shift;;
    -h|--help) usage; exit 0;;
    *) echo "ERROR: unknown argument: $1" >&2; usage >&2; exit 2;;
  esac
done
bag_load_settings "$_PLAY_CONFIG"
_PLAY_BAG="${_PLAY_BAG:-${_BAG_SETTINGS[1]}}"
_PLAY_RATE="${_PLAY_RATE:-${_BAG_SETTINGS[2]}}"
_PLAY_LOOP="${_PLAY_LOOP:-${_BAG_SETTINGS[3]}}"
[[ -n "$_PLAY_BAG" ]] || { echo 'ERROR: provide --bag PATH or config/bag.yaml bag_path' >&2; exit 2; }
_PLAY_INFO="$(/usr/bin/python3 -I - "$_BAG_ROOT" "$_PLAY_BAG" "$_PLAY_RATE" <<'PY'
from pathlib import Path
import json
import math
import os
import sys
import yaml

def optional_ns(data, section, key):
    value = data.get(section)
    value = value.get(key) if isinstance(value, dict) else None
    return value if type(value) is int else "-"

def topic_coverage(data):
    rows = data.get("topics_with_message_count")
    if not isinstance(rows, list):
        return "-"
    counts = {}
    remaining = 4000  # Leave room for the envelope below the log CLI's 4096 limit.
    omitted = 0
    for row in rows:
        topic = row.get("topic_metadata") if isinstance(row, dict) else None
        name = topic.get("name") if isinstance(topic, dict) else None
        count = row.get("message_count") if isinstance(row, dict) else None
        if not isinstance(name, str) or not name or type(count) is not int or count < 0:
            omitted += 1
            continue
        size = len(json.dumps(name)) + len(str(count)) + 2
        if size > remaining:
            omitted += 1
            continue
        counts[name] = count
        remaining -= size
    return json.dumps({"counts": counts, "omitted": omitted}, separators=(",", ":"))

try:
    root = Path(sys.argv[1]); path = Path(os.path.expandvars(sys.argv[2])).expanduser()
    path = (root / path).resolve() if not path.is_absolute() else path.resolve()
    rate = float(sys.argv[3])
    if not math.isfinite(rate) or rate <= 0:
        raise ValueError('rate must be positive and finite')
    data = yaml.safe_load((path / 'metadata.yaml').read_text())['rosbag2_bagfile_information']
    if data.get('message_count', 0) <= 0:
        raise ValueError('bag contains 0 messages; finish recording before replay')
    files = data.get('relative_file_paths', [])
    if not files or any(not (path / f).resolve().is_relative_to(path) or not (path / f).is_file() for f in files):
        raise ValueError('bag has missing or invalid storage files')
    print(
        data["message_count"],
        optional_ns(data, "starting_time", "nanoseconds_since_epoch"),
        optional_ns(data, "duration", "nanoseconds"),
    )
    print(topic_coverage(data))
    print(path)
except (OSError, TypeError, KeyError, ValueError, yaml.YAMLError) as exc:
    sys.exit(f'ERROR: {exc}')
PY
)"
read -r _PLAY_MESSAGES _PLAY_START_NS _PLAY_DURATION_NS <<< "${_PLAY_INFO%%$'\n'*}"
_PLAY_INFO="${_PLAY_INFO#*$'\n'}"
_PLAY_TOPIC_COVERAGE="${_PLAY_INFO%%$'\n'*}"
_PLAY_BAG="${_PLAY_INFO#*$'\n'}"
_PLAY_COMMAND=(ros2 bag play "$_PLAY_BAG" --clock "${_BAG_SETTINGS[4]}" --rate "$_PLAY_RATE")
[[ "$_PLAY_LOOP" != true ]] || _PLAY_COMMAND+=(--loop)
if ((_PLAY_DRY)); then
  printf 'ROS bag environment: %s\n' "${_BAG_SETTINGS[0]}"
  printf 'Player only: '; printf '%q ' "${_PLAY_COMMAND[@]}"; printf '\n'
  exit 0
fi
bag_prepare_environment
_PLAY_LOG=(
  "bag_path=$_PLAY_BAG" "rate=$_PLAY_RATE" "loop=$_PLAY_LOOP"
  "clock_hz=${_BAG_SETTINGS[4]}" "bag_message_count=$_PLAY_MESSAGES"
)
if [[ "$_PLAY_START_NS" != "-" ]]; then _PLAY_LOG+=("bag_start_stamp_ns=$_PLAY_START_NS"); fi
if [[ "$_PLAY_DURATION_NS" != "-" ]]; then _PLAY_LOG+=("bag_duration_ns=$_PLAY_DURATION_NS"); fi
if [[ "$_PLAY_TOPIC_COVERAGE" != "-" ]]; then
  _PLAY_LOG+=("bag_topic_coverage=$_PLAY_TOPIC_COVERAGE")
fi
dimos_log_start bag_play "${_PLAY_LOG[@]}"
exec "${_PLAY_COMMAND[@]}"
