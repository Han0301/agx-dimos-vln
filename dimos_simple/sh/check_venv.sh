#!/usr/bin/env bash
# Check the project-local virtualenv and its DimOS/Go2 VLN dependencies.
set -euo pipefail

# Match activate_vln.sh: LAN inference services must be reached directly.
unset http_proxy https_proxy HTTP_PROXY HTTPS_PROXY ALL_PROXY all_proxy

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$SCRIPT_DIR/start_venv/verify.sh" "$@"
