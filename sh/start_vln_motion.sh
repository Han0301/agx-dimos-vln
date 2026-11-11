#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
cd "$root"

config=config/profiles/vln_motion.yaml
check=false
while (($#)); do
  case "$1" in
    --config)
      [[ $# -ge 2 && $2 != --* ]] || { echo '--config requires a file' >&2; exit 2; }
      config=$2
      shift
      ;;
    --check) check=true ;;
    -h|--help)
      echo 'Usage: bash sh/start_vln_motion.sh [--config FILE] [--check]'
      exit 0
      ;;
    *) echo "Unknown argument: $1" >&2; exit 2 ;;
  esac
  shift
done

refs="$("$root/.venv/bin/python" - "$config" <<'PY'
import sys
from pathlib import Path

import yaml

keys = ("web_profile", "tracker_config")
try:
    config = yaml.safe_load(Path(sys.argv[1]).read_text())
    if not isinstance(config, dict) or set(config) != set(keys):
        raise ValueError("expected web_profile and tracker_config")
    for key in keys:
        path = config[key]
        if not isinstance(path, str) or not path.strip() or "\n" in path:
            raise ValueError(f"{key} must reference a configuration file")
        print(path)
except (OSError, ValueError, yaml.YAMLError) as exc:
    sys.exit(f"{sys.argv[1]}: {exc}")
PY
)"
mapfile -t paths <<< "$refs"
web=(bash sh/start_web_input.sh --profile "${paths[0]}" --motion-tracker-config "${paths[1]}")

if "$check"; then
  exec "${web[@]}" --check
fi
source "$root/sh/internal/runtime_log.bash"
dimos_log_start vln_motion
exec "${web[@]}"
