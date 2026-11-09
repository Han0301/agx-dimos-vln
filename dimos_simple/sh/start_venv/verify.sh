#!/usr/bin/env bash
# Validate imports, editable installation, ROS bridge, and package parity.
set -Eeuo pipefail
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Proxy variables can point at unsupported schemes such as socks://127.0.0.1.
# Direct LAN access is also required for the inference services.
unset http_proxy https_proxy HTTP_PROXY HTTPS_PROXY ALL_PROXY all_proxy

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd -- "$SCRIPT_DIR/../.." && pwd)"
VENV_DIR="$WORKSPACE_ROOT/.venv"
export VLN_CONFIG="$WORKSPACE_ROOT/dimos/agents/skills/config/vln_config.yaml"
source "$WORKSPACE_ROOT/sh/xnavdp_ros2/dds_env.sh"
# shellcheck source=/dev/null
source "$SCRIPT_DIR/config.env"

CHECK_REMOTE=1
while (($#)); do
  case "$1" in
    --skip-remote) CHECK_REMOTE=0 ;;
    -h|--help)
      cat <<'EOF'
用法: bash sh/check_venv.sh [--skip-remote]

  --skip-remote  只检查本地 venv，不检查 Ollama/Qwen/XNavDP 服务
EOF
      exit 0
      ;;
    *) echo "未知选项: $1" >&2; exit 2 ;;
  esac
  shift
done

info() { printf '\033[36m▸\033[0m %s\n' "$*"; }
ok() { printf '\033[32m✓\033[0m %s\n' "$*"; }
warn() { printf '\033[33m⚠\033[0m %s\n' "$*" >&2; }
die() { printf '\033[31m✗\033[0m %s\n' "$*" >&2; exit 1; }

PYTHON="$VENV_DIR/bin/python"
DIMOS_BIN="$VENV_DIR/bin/dimos"
[[ -x "$PYTHON" ]] || die "找不到虚拟环境 Python: $PYTHON"
[[ -x "$DIMOS_BIN" ]] || die "找不到 dimos 命令: $DIMOS_BIN"

CHECK_TEMP="${TMPDIR:-/tmp}/dimos-check-$UID"
state_root="${XDG_STATE_HOME:-$HOME/.local/state}"
if [[ ! -w "$state_root/dimos/logs" ]]; then
  export XDG_STATE_HOME="$CHECK_TEMP/state"
  mkdir -p "$XDG_STATE_HOME/dimos/logs"
fi
if [[ ! -w "${MPLCONFIGDIR:-$HOME/.config/matplotlib}" ]]; then
  export MPLCONFIGDIR="$CHECK_TEMP/matplotlib"
  mkdir -p "$MPLCONFIGDIR"
fi
if [[ ! -w "$WORKSPACE_ROOT/dimos/mapping/pointclouds/__pycache__" ]]; then
  export NUMBA_CACHE_DIR="$CHECK_TEMP/numba"
  mkdir -p "$NUMBA_CACHE_DIR"
fi

ROOT="$WORKSPACE_ROOT" env -u PYTHONPATH -u PYTHONHOME "$PYTHON" - <<'PY'
import importlib.metadata
import json
import os
from pathlib import Path
import sysconfig
from urllib.parse import unquote, urlparse

root = Path(os.environ["ROOT"]).resolve()
site_packages = Path(sysconfig.get_paths()["purelib"])
dist_infos = sorted(site_packages.glob("dimos-*.dist-info"))
if not dist_infos:
    raise SystemExit(f"虚拟环境中没有 dimos dist-info: {site_packages}")
dist = importlib.metadata.PathDistribution(dist_infos[-1])
direct = dist.read_text("direct_url.json")
if not direct:
    raise SystemExit("dimos 不是 editable 安装")
url = json.loads(direct).get("url", "")
installed_root = Path(unquote(urlparse(url).path)).resolve()
if installed_root != root:
    raise SystemExit(f"dimos editable 路径错误: {installed_root} != {root}")

import cv2  # noqa: F401
import fastapi  # noqa: F401
import langchain  # noqa: F401
import torch  # noqa: F401
import transformers  # noqa: F401
import turbojpeg  # noqa: F401
import ultralytics  # noqa: F401
import unitree_webrtc_connect  # noqa: F401
from dimos.core.global_config import global_config
global_config.navigation_policy = "xnavdp"
from dimos.robot.unitree.go2.blueprints.agentic.unitree_go2_vln_local import (  # noqa: E402
    unitree_go2_vln_local,
)

print(f"editable dimos: {installed_root}")
print(f"VLN blueprint: {type(unitree_go2_vln_local).__name__}")
PY
ok "Python 包、editable 路径及 Go2 VLN 蓝图加载正确"

env -u PYTHONPATH -u PYTHONHOME "$DIMOS_BIN" --help >/dev/null
blueprints="$(env -u PYTHONPATH -u PYTHONHOME "$DIMOS_BIN" list)"
grep -Fxq "unitree-go2-vln-local" <<<"$blueprints" ||
  die "蓝图列表中没有 unitree-go2-vln-local"
ok "dimos CLI 和 Go2 VLN 蓝图可用"

if [[ -f "$ROS2_BRIDGE_ROOT/env.sh" ]]; then
  (
    export DIMOS_ROOT="$WORKSPACE_ROOT"
    # ROS setup scripts intentionally tolerate unset variables.
    set +u
    # shellcheck disable=SC1091
    source "$ROS2_BRIDGE_ROOT/env.sh" >/dev/null
    env -u PYTHONHOME "$PYTHON" -c \
      'import rclpy, sensor_msgs; print("rclpy:", rclpy.__file__)'
  )
  ok "ROS 2 Jazzy 和 D455i bridge 依赖可用"
else
  echo "⚠ 未找到 $ROS2_BRIDGE_ROOT/env.sh，跳过 ROS 2 导入检查"
fi

if [[ -x "$DIMOS_REFERENCE_VENV/bin/python" ]]; then
  python3 "$SCRIPT_DIR/compare_env.py" \
    --current "$PYTHON" \
    --reference "$DIMOS_REFERENCE_VENV/bin/python" \
    --ignore navdp-bridge
else
  echo "⚠ 未找到参考环境 $DIMOS_REFERENCE_VENV，跳过包集合对比"
fi

if ((CHECK_REMOTE)); then
  info "检查远端推理服务"
  set +u
  if [[ -f "$ROS2_BRIDGE_ROOT/env.sh" ]]; then
    source "$ROS2_BRIDGE_ROOT/env.sh" >/dev/null
  elif [[ -f "$ROS2_JAZZY/install/setup.bash" ]]; then
    source "$ROS2_JAZZY/install/setup.bash" >/dev/null
  fi
  set -u
  # The ROS bridge can restore proxy settings; LAN services must use direct connections.
  unset http_proxy https_proxy HTTP_PROXY HTTPS_PROXY ALL_PROXY all_proxy
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  ROOT="$WORKSPACE_ROOT" env -u PYTHONHOME "$PYTHON" - <<'PY'
import os
import json
from pathlib import Path
import sys
from urllib.error import HTTPError, URLError
from urllib.request import urlopen

from dimos.agents.skills.config import load_vln_config
from dimos.navigation.xnavdp.client import XNavDPClient

cfg = load_vln_config(
    str(Path(os.environ["ROOT"]) / "dimos/agents/skills/config/vln_config.yaml")
)
client = XNavDPClient(debug_mode=False)
try:
    client.verify_backend()
    print(f"  OK   XNavDP ROS2: {json.dumps(client.health, ensure_ascii=False)}")
finally:
    client.close()
services = [
    ("Ollama", f"{cfg.agent.ollama_base_url.rstrip('/')}/api/tags"),
    ("Qwen3-VL", f"{cfg.vlm.base_url.rstrip('/')}/health"),
]

failed = []
for name, url in services:
    try:
        with urlopen(url, timeout=8) as response:
            print(f"  OK   {name}: HTTP {response.status} {url}")
    except HTTPError as exc:
        print(f"  OK   {name}: HTTP {exc.code} {url}")
    except (URLError, TimeoutError, ConnectionError) as exc:
        print(f"  FAIL {name}: {exc} {url}", file=sys.stderr)
        failed.append(name)

if failed:
    raise SystemExit("远端服务不可用: " + ", ".join(failed))
PY
  ok "远端推理服务可用"
fi

ok "环境校验通过"
