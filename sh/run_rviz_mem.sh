#!/usr/bin/env bash
# Use the same ROS runtime and display helpers with the exploration profile.
set -euo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
exec bash "$script_dir/run_rviz.sh" --memory-view "$@"
