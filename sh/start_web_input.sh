#!/usr/bin/env bash
set -eo pipefail
_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
_PYTHON="$_ROOT/.venv/bin/python"
[[ -x "$_PYTHON" ]] || { echo 'Missing .venv; run sh/setup/install.sh' >&2; exit 1; }
_PROFILE=0
_CHECK=0
for arg in "$@"; do
  case "$arg" in
    --profile|--profile=*) _PROFILE=1 ;;
    --check|--help|-h) _CHECK=1 ;;
  esac
done
if ((_PROFILE && !_CHECK)); then
  source "$_ROOT/sh/env.sh"
fi
export PYTHONPATH="$_ROOT${PYTHONPATH:+:$PYTHONPATH}"
cd "$_ROOT"
exec "$_PYTHON" -m dimos.plugins.web_input "$@"
