#!/usr/bin/env bash
# Independent XNavDP client; forwards the existing plugin arguments unchanged.
set -eo pipefail
_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
if [[ " $* " != *" --check "* && " $* " != *" --help "* && " $* " != *" -h "* ]]; then
  source "$_ROOT/sh/env.sh"
fi
_PYTHON="${DIMOS_PYTHON:-$_ROOT/.venv/bin/python}"
if [[ ! -x "$_PYTHON" ]]; then _PYTHON=python3; fi
cd "$_ROOT"
export PYTHONPATH="$_ROOT${PYTHONPATH:+:$PYTHONPATH}"
exec "$_PYTHON" -m dimos.plugins.xnavdp "$@"
