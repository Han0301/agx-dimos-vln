#!/usr/bin/env bash
set -eo pipefail
_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
if [[ " $* " != *" --check "* && " $* " != *" --help "* && " $* " != *" -h "* ]]; then
  source "$_ROOT/sh/env.sh"
fi
_PYTHON="$_ROOT/.venv/bin/python"
[[ -x "$_PYTHON" ]] || { echo 'Missing workspace .venv; run sh/setup/install.sh' >&2; exit 1; }
export PYTHONPATH="$_ROOT${PYTHONPATH:+:$PYTHONPATH}"
cd "$_ROOT"
exec "$_PYTHON" -m dimos.plugins.path_tracker "$@"
