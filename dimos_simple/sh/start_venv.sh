#!/usr/bin/env bash
# One-click entry point for creating and validating the DimOS virtualenv.
set -euo pipefail

if [[ "${BASH_SOURCE[0]}" != "$0" ]]; then
  echo "请直接执行此脚本，不要 source：bash sh/start_venv.sh" >&2
  return 1 2>/dev/null || exit 1
fi

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$SCRIPT_DIR/start_venv/install.sh" "$@"
