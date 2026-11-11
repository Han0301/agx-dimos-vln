import subprocess
import sys
import time
from types import SimpleNamespace

import pytest
import yaml
from dimos.core.config import ROOT, load_config
from dimos.robot.unitree.go2.body_bridge import Go2BodyBridge
from dimos.core.runtime import REGISTRY


def config():
    return load_config(ROOT / "config/base.yaml")


def write_config(tmp_path, cfg):
    p = tmp_path / "config.yaml"
    p.write_text(yaml.safe_dump(cfg))
    return p


def test_config_and_existing_contracts():
    cfg = config()
    assert cfg["modules"] == {"body": True}
    assert not cfg["body"]["allow_motion"]
    assert "algorithms" not in cfg
    assert cfg["camera"]["mount_frame"] == "camera_link"
    assert cfg["topics"]["odom"] == "/odom"
    spec = yaml.safe_load((ROOT / "config/interfaces.yaml").read_text())
    assert spec["base"]["publish"]["/odom"]["type"] == "geometry_msgs/msg/PoseStamped"
    assert set(REGISTRY) == {"body"}


@pytest.mark.parametrize(
    "mutation",
    [
        lambda c: c["modules"].update(web=True),
        lambda c: c["modules"].update(body="false"),
        lambda c: c["topics"].update(odom="/cmd_vel"),
        lambda c: c["body"].update(command_timeout_s=0),
        lambda c: c["body"].update(max_linear_mps=float("nan")),
        lambda c: c["camera"].update(depth_scale=1000),
        lambda c: c["camera"].update(translation=[1, 2]),
        lambda c: c.update(algorithms={"vln": {"selected": "xnavdp"}}),
        lambda c: c["modules"].update(rgbd_bridge=True),
        lambda c: c.update(version=1),
    ],
)
def test_reject_bad_configuration(tmp_path, mutation):
    cfg = config()
    mutation(cfg)
    with pytest.raises(ValueError):
        load_config(write_config(tmp_path, cfg))


def test_check_imports_no_hardware_or_visualization():
    script = """
import sys
from dimos.core.cli import main
assert main(["--check"])==0
for prefix in ("rclpy","unitree_webrtc_connect","cv2","rerun","langchain","open3d","dimos.robot","dimos.hardware"):
    assert not any(n==prefix or n.startswith(prefix+".") for n in sys.modules),prefix
"""
    subprocess.run(
        [sys.executable, "-c", script],
        cwd=ROOT,
        check=True,
        capture_output=True,
        text=True,
    )


def test_disabled_motion_and_watchdog():
    cfg = config()
    body = Go2BodyBridge(None, cfg)
    calls = []
    body.connection = SimpleNamespace(move=lambda *v: calls.append(v))
    msg = SimpleNamespace(
        linear=SimpleNamespace(x=1.0, y=-1.0), angular=SimpleNamespace(z=1.0)
    )
    body._on_command(msg)
    assert calls == []
    cfg["body"]["allow_motion"] = True
    body._on_command(msg)
    assert calls[-1] == (0.4, -0.4, 0.2)
    body.last_command = time.monotonic() - 1
    body._watchdog()
    assert calls[-1] == (0.0, 0.0, 0.0) and not body.commands_active
    count = len(calls)
    body._watchdog()
    assert len(calls) == count
    msg.linear.x = float("nan")
    body._on_command(msg)
    assert calls[-1] == (0.0, 0.0, 0.0)


def test_only_cpp_compressor_remains():
    ws = ROOT / "camera_ws"
    assert not list(ws.rglob("compresser.py"))
    assert not list(ws.rglob("d455_with_compresser.launch.py"))
    cmake = (ws / "third_party/insta360_ros_driver/CMakeLists.txt").read_text()
    assert "scripts/compresser.py" not in cmake
    launch = (ws / "launch/start_d455.launch.py").read_text()
    assert (
        "SetRemap(src='/camera/camera/color/camera_info', dst='/realsense_camera_info')"
        in launch
    )
    assert (
        ws / "install/compressed_RGBD_pkg/lib/compressed_RGBD_pkg/compressed_rgbd_node"
    ).is_file()


def test_no_duplicate_camera_info_publisher():
    spec = yaml.safe_load((ROOT / "config/interfaces.yaml").read_text())
    assert "/realsense_camera_info" not in spec["base"]["publish"]
    assert "/realsense_camera_info" in spec["camera_ws"]
    assert "calibration" not in REGISTRY
    assert not (ROOT / "dimos/hardware/sensors/camera/calibration.py").exists()
    source = (ROOT / "dimos/robot/unitree/go2/body_bridge.py").read_text()
    assert "create_publisher(CameraInfo" not in source


def test_aes_credentials_match_original_environment(monkeypatch):
    from dimos.robot.unitree.go2.body_bridge import read_aes_key

    cfg = config()
    assert cfg["body"]["aes_key_env"] == "UNITREE_AES_128_KEY"
    monkeypatch.setenv("UNITREE_AES_128_KEY", "a" * 32)
    assert read_aes_key(cfg["body"]["aes_key_env"]) == "a" * 32
    monkeypatch.delenv("UNITREE_AES_128_KEY")
    with pytest.raises(RuntimeError, match="Missing UNITREE_AES_128_KEY"):
        read_aes_key(cfg["body"]["aes_key_env"])
    invalid = "invalid-secret-value"
    monkeypatch.setenv("UNITREE_AES_128_KEY", invalid)
    with pytest.raises(RuntimeError) as error:
        read_aes_key(cfg["body"]["aes_key_env"])
    assert invalid not in str(error.value)


def test_startup_logging_does_not_dump_sdk_packets():
    script = """
import logging
import sys
from types import ModuleType
runtime = ModuleType("dimos.core.runtime")
def run(*args, **kwargs):
    logging.info("SDK_PACKET_MUST_BE_HIDDEN")
    logging.warning("SDK_WARNING_MUST_REMAIN")
    logging.getLogger("dimos.runtime").info("APPLICATION_START_MUST_REMAIN")
runtime.run = run
sys.modules["dimos.core.runtime"] = runtime
from dimos.core.cli import main
assert main([]) == 0
"""
    result = subprocess.run(
        [sys.executable, "-c", script],
        cwd=ROOT,
        capture_output=True,
        text=True,
        check=True,
    )
    assert "SDK_PACKET_MUST_BE_HIDDEN" not in result.stderr
    assert "SDK_WARNING_MUST_REMAIN" in result.stderr
    assert "APPLICATION_START_MUST_REMAIN" in result.stderr
