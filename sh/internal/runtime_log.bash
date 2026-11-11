#!/usr/bin/env bash
# Preserve launcher PID, argv, stdin and independent stdout/stderr.
_DIMOS_LOG_PROJECT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd -P)"

dimos_log_allocate() {
  local module="$1"
  local -a root=() metadata=()
  local item
  if [[ -n "${2:-}" ]]; then root=(--root "$2"); fi
  for item in "${@:4}"; do metadata+=("--metadata=$item"); done
  PYTHONPATH="$_DIMOS_LOG_PROJECT" PYTHONDONTWRITEBYTECODE=1 \
    /usr/bin/python3 -m dimos.plugins.runtime_log "$module" --create \
      --pid "${3:-$$}" "${root[@]}" "${metadata[@]}"
}

dimos_log_start() {
  local directory owner="$BASHPID" module="$1"
  shift
  if ! directory="$(dimos_log_allocate "$module" "" "$owner" "$@")" || [[ -z "$directory" ]]; then
    return 0
  fi
  export ROS_LOG_DIR="$directory/ros" PYTHONUNBUFFERED=1
  exec > >(
    trap '' INT TERM HUP
    PYTHONPATH="$_DIMOS_LOG_PROJECT" PYTHONDONTWRITEBYTECODE=1 \
      exec /usr/bin/python3 -m dimos.plugins.runtime_log.relay "$directory/stdout.log"
  ) 2> >(
    trap '' INT TERM HUP
    PYTHONPATH="$_DIMOS_LOG_PROJECT" PYTHONDONTWRITEBYTECODE=1 \
      exec /usr/bin/python3 -m dimos.plugins.runtime_log.relay "$directory/stderr.log" >&2
  )
}
