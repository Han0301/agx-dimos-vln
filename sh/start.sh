#!/usr/bin/env bash
set -eo pipefail
_MIN_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
# Configuration checks deliberately do not need ROS, SDKs, cameras or a robot.
if [[ " $* " != *" --check "* ]]; then
  source "$_MIN_ROOT/sh/env.sh"
fi
_MIN_PYTHON="${DIMOS_PYTHON:-$_MIN_ROOT/.venv/bin/python}"
if [[ ! -x "$_MIN_PYTHON" ]]; then _MIN_PYTHON=python3; fi
cd "$_MIN_ROOT"
export PYTHONPATH="$_MIN_ROOT${PYTHONPATH:+:$PYTHONPATH}"
exec "$_MIN_PYTHON" -m dimos "$@"
