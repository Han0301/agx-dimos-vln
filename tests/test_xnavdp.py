import ast
import copy
import json
import math
import subprocess
import sys
from types import SimpleNamespace

import pytest
import yaml
from dimos.core.config import ROOT
from dimos.plugins.xnavdp.__main__ import load_test_config
from dimos.plugins.xnavdp.client import (
    XNavDPClient,
    relative_goal_xy,
    validate_result,
)


def test_relative_goal_matches_simple():
    assert relative_goal_xy((10.0, 20.0), (0.0, 0.0, 0.0, 1.0), (3.0, 0.0)) == (
        13.0,
        20.0,
    )
    actual = relative_goal_xy(
        (10.0, 20.0), (0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5)), (3.0, 0.0)
    )
    assert actual == pytest.approx((10.0, 23.0))
    with pytest.raises(ValueError):
        relative_goal_xy((0.0, 0.0), (0.0, 0.0, 0.0, 0.0), (3.0, 0.0))


def test_config_and_check_have_no_ros_imports():
    cfg = load_test_config(ROOT / "config/plugins/xnavdp/xnavdp_test.yaml")
    assert cfg["mode"] == "health_only" and not cfg["initialization"]["reset_if_needed"]
    code = """
import sys
from dimos.plugins.xnavdp.__main__ import main
assert main(["--check"])==0
for prefix in ("rclpy","cv2","unitree_webrtc_connect","dimos.robot","dimos.core.runtime"):
    assert not any(n==prefix or n.startswith(prefix+".") for n in sys.modules),prefix
"""
    result = subprocess.run(
        [sys.executable, "-c", code],
        cwd=ROOT,
        capture_output=True,
        text=True,
        check=True,
    )
    assert "7411" in result.stdout and "8889" not in result.stdout


@pytest.mark.parametrize(
    "change",
    [
        lambda c: c.update(mode="navigate"),
        lambda c: c["test"].update(requests=0),
        lambda c: c["test"].update(response_timeout_s=float("nan")),
        lambda c: c["goal"].update(xy=[1]),
        lambda c: c["initialization"].update(reset_if_needed="false"),
    ],
)
def test_config_rejects_bad_values(tmp_path, change):
    cfg = yaml.safe_load((ROOT / "config/plugins/xnavdp/xnavdp_test.yaml").read_text())
    change(cfg)
    path = tmp_path / "bad.yaml"
    path.write_text(yaml.safe_dump(cfg))
    with pytest.raises(ValueError):
        load_test_config(path)


def test_only_original_goal_publisher_in_plugin_source():
    source = (ROOT / "dimos/plugins/xnavdp/client.py").read_text()
    tree = ast.parse(source)
    publishers = [
        n
        for n in ast.walk(tree)
        if isinstance(n, ast.Call)
        and isinstance(n.func, ast.Attribute)
        and n.func.attr == "create_publisher"
    ]
    assert len(publishers) == 1
    assert (
        isinstance(publishers[0].args[1], ast.Name)
        and publishers[0].args[1].id == "GOAL_TOPIC"
    )
    assert "cmd_vel" not in source
    assert "info_pub" not in source


def header(ns=123, frame="world"):
    return SimpleNamespace(
        stamp=SimpleNamespace(sec=ns // 10**9, nanosec=ns % 10**9), frame_id=frame
    )


def result_message(ns=123, status="success", **extra):
    return SimpleNamespace(
        data=json.dumps(dict(request_stamp_ns=ns, status=status, **extra))
    )


def test_exact_nanosecond_and_late_result_correlation():
    client = XNavDPClient({})
    client.pending = 123
    client._receive("path", SimpleNamespace(header=header(122)), header(122))
    client._result(result_message(122))
    assert not client.parts
    path = SimpleNamespace(header=header(123, "base_link"))
    client._receive("path", path, path.header)
    assert not client.event.is_set()
    msg = result_message(123)
    client._result(msg)
    assert client.parts == {"path": path, "result": msg} and client.event.is_set()
    client.pending = None
    client.parts.clear()
    client._result(result_message())
    assert not client.parts


def test_error_status_wakes_waiter():
    client = XNavDPClient({})
    client.pending = 123
    client._result(result_message(error="test model failure", status="error"))
    assert client.error == "test model failure" and client.event.is_set()


def test_native_result_validation():
    point = SimpleNamespace(x=1.0, y=2.0, z=0.0)
    frame = header(123, "base_link")
    pose = SimpleNamespace(position=point)
    parts = {
        "path": SimpleNamespace(
            header=frame, poses=[SimpleNamespace(header=frame, pose=pose)]
        ),
        "result": result_message(
            trajectory=[[[1.0, 2.0, 0.0]]],
            all_trajectory=[[[[1.0, 2.0, 0.0]]]],
            all_values=[[0.2]],
            trajectory_frame="base_link",
            goal_frame="base_link",
            world_frame="world",
            goal=[3.0, 0.0],
            robot_pos=[0.0, 0.0, 0.0],
            robot_quat=[0.0, 0.0, 0.0, 1.0],
            odom_rgb_gap_ms=10.0,
        ),
    }
    goal = SimpleNamespace(
        header=header(), pose=SimpleNamespace(position=SimpleNamespace(x=3.0, y=0.0))
    )
    assert validate_result(parts, goal)["odom_rgb_gap_ms"] == 10.0
    bad = copy.deepcopy(parts)
    bad["path"].header.frame_id = "world"
    with pytest.raises(ValueError):
        validate_result(bad, goal)
    bad = copy.deepcopy(parts)
    payload = json.loads(bad["result"].data)
    payload["all_values"] = [[float("nan")]]
    bad["result"].data = json.dumps(payload)
    with pytest.raises(ValueError):
        validate_result(bad, goal)


def test_historical_bag_timestamp_is_fresh_when_just_received(monkeypatch):
    from dimos.plugins.xnavdp import client as module

    client = XNavDPClient({"sensor_timeout_s": 0.05})
    rgb, odom = (
        SimpleNamespace(header=header(100_000_000_000)),
        SimpleNamespace(header=header(100_000_000_000)),
    )
    client._rgb(rgb)
    client._odom(odom)
    monkeypatch.setattr(module, "build_goal", lambda image, pose, xy: (image, pose, xy))
    assert client.next_goal([3.0, 0.0]) == (rgb, odom, [3.0, 0.0])
    client.last_rgb_at -= 2
    client.last_odom_at -= 2
    with pytest.raises(TimeoutError, match="fresh"):
        client.next_goal([3.0, 0.0])


def test_prepare_waits_for_input_and_calibration_without_reset(monkeypatch):
    client = XNavDPClient({"sensor_timeout_s": 0.5})
    replies = iter(
        [
            {"initialized": True, "calibrated": False, "buffers": {}},
            {
                "initialized": True,
                "calibrated": True,
                "buffers": {"rgb": 1, "depth": 1, "odom": 1},
            },
        ]
    )
    monkeypatch.setattr(client, "health", lambda: next(replies))
    assert client.prepare()["calibrated"] is True


def test_explicit_initialization_flag_preserves_default_and_check_has_no_ros():
    path = ROOT / "config/plugins/xnavdp/xnavdp_test.yaml"
    assert not load_test_config(path)["initialization"]["reset_if_needed"]
    assert load_test_config(path, reset_if_needed=True)["initialization"][
        "reset_if_needed"
    ]
    script = """
import contextlib,io,json,sys
from dimos.plugins.xnavdp.__main__ import main
output=io.StringIO()
with contextlib.redirect_stdout(output):
    assert main(["--mode","inference","--reset-if-needed","--check"])==0
assert json.loads(output.getvalue())["reset_if_needed"] is True
assert "rclpy" not in sys.modules
"""
    subprocess.run(
        [sys.executable, "-c", script],
        cwd=ROOT,
        check=True,
        capture_output=True,
        text=True,
    )
