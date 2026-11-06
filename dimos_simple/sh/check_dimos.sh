#!/usr/bin/env bash
# Independent XNavDP integration, dependency isolation, and live communication check.
set -Eeuo pipefail
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

unset http_proxy https_proxy HTTP_PROXY HTTPS_PROXY ALL_PROXY all_proxy
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
PYTHON="$ROOT/.venv/bin/python"
export VLN_CONFIG="$ROOT/dimos/agents/skills/config/vln_config.yaml"

cd "$ROOT"
set +u
source "$SCRIPT_DIR/activate_vln.sh" >/dev/null
set -u

echo "== Robot and camera connection =="
(
  set +u
  source "$SCRIPT_DIR/activate_vln.sh" >/dev/null
  set -u
  "$PYTHON" "$SCRIPT_DIR/check_dimos/check_hardware.py"
)

echo
echo "== venv and service validation =="
bash "$SCRIPT_DIR/check_venv.sh"

echo
echo "== XNavDP / NavDP isolation =="
"$PYTHON" - <<'PY'
import importlib.abc
import sys


class BlockNavDP(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname == "navdp_bridge" or fullname.startswith("navdp_bridge."):
            raise ModuleNotFoundError(fullname)
        if fullname == "dimos.navigation.navdp" or fullname.startswith(
            "dimos.navigation.navdp."
        ):
            raise ModuleNotFoundError(fullname)
        return None


sys.meta_path.insert(0, BlockNavDP())

from dimos.core.global_config import global_config

global_config.navigation_policy = "xnavdp"
assert global_config.debug_mode is True
from dimos.navigation.xnavdp.navigator import XNavDPNavigator
from dimos.navigation.xnavdp.client import XNavDPClient
from dimos.navigation.xnavdp.client import VERSION
from dimos.robot.unitree.go2.blueprints.agentic.unitree_go2_vln_local import (
    unitree_go2_vln_local,
)

module_paths = {bp.module.__module__ for bp in unitree_go2_vln_local.blueprints}
assert "dimos.navigation.xnavdp.navigator" in module_paths
assert not any(name.startswith("dimos.navigation.navdp") for name in module_paths)
assert XNavDPNavigator.__module__ == "dimos.navigation.xnavdp.navigator"
assert VERSION == "xnavdp-ros2-topics-v2"
client_source = __import__("inspect").getsource(XNavDPClient)
assert "requests.post" not in client_source
assert "http://" not in client_source
assert "create_publisher" in client_source
tick_source = __import__("inspect").getsource(XNavDPNavigator._tick)
assert tick_source.count("plan.command(") == 1
from dimos.core.transport import ROSTransport, pSHMTransport
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
assert isinstance(unitree_go2_vln_local.transport_map[("odom", PoseStamped)], ROSTransport)
assert not any(isinstance(tx, pSHMTransport) for tx in unitree_go2_vln_local.transport_map.values())
print("NavDP/navdp_bridge blocked: XNavDP blueprint imported successfully")
PY

echo
echo "== debug_mode CLI =="
bash -c 'source ./sh/activate_vln.sh >/dev/null && dimos --help' |
  grep -- '--debug_mode' >/dev/null
echo "debug_mode CLI: OK"

echo
echo "== debug session files =="
"$PYTHON" - <<'PY'
from pathlib import Path
import tempfile
import time

from dimos.utils.debug_session import DebugSession, write_debug_mark

with tempfile.TemporaryDirectory(prefix="dimos-debug-check-") as root:
    log_dir = Path(root) / "run" / "check" / "logs"
    session = DebugSession("check", log_dir=log_dir)
    session.start()
    print("terminal capture check")
    time.sleep(0.4)
    write_debug_mark("marker check", log_root=Path(root) / "run", session_id="check")
    session.close()
    required = [
        session.terminal_log,
        session.timing_log,
        session.trace_log,
        session.marker_log,
        session.session_dir / "session.json",
        session.status_log,
    ]
    for path in required:
        assert path.is_file() and path.stat().st_size > 0, path
    assert "terminal capture check" in session.terminal_log.read_text()
    assert '"text": "marker check"' in session.marker_log.read_text()
print("debug session files: OK")
PY

echo
echo "== XNavDP live communication =="
"$PYTHON" -m dimos.navigation.xnavdp.diagnostics \
  --health-only

echo
echo "check_dimos: PASS (连接与配置检查；未重置正在运行的模型)"
