#!/usr/bin/env bash
set -eo pipefail
WORKSPACE_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd -P)"
cd "$WORKSPACE_DIR"
source ./sh/setup_local.bash
exec colcon build --base-paths third_party src --parallel-workers 2 \
  --cmake-args -DBUILD_TESTING=OFF -DCMAKE_INSTALL_RPATH_USE_LINK_PATH=OFF -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3 "$@"
