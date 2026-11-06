#!/usr/bin/env bash
set -e
WORKSPACE_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
exec bash "${WORKSPACE_DIR}/sh/start_cameras.sh" "$@"
