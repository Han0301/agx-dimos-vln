#!/usr/bin/env bash
set -e
_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
cd "$_ROOT"
export PYTHONPATH="$_ROOT${PYTHONPATH:+:$PYTHONPATH}"
exec /usr/bin/python3 -m dimos.plugins.runtime_log record "$@"
