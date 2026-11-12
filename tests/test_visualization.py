"""RViz wiring and optional result rendering without launching a window."""

import json
import shlex
import subprocess
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml
from dimos.core.config import ROOT


def test_rviz_layout_uses_trajectory_and_result_display():
    profile = yaml.safe_load((ROOT / "config/plugins/xnavdp/xnavdp.rviz").read_text())
    manager = profile["Visualization Manager"]
    assert manager["Global Options"]["Fixed Frame"] == "base_link"
    topics = {
        d["Name"]: d["Topic"]["Value"] for d in manager["Displays"] if "Topic" in d
    }
    assert topics == {
        "Selected Trajectory": "/xnavdp/trajectory",
        "Result Candidates Scores Goal": "/xnavdp/result_markers",
        "D435 Point Cloud": "/xnavdp/depth/points",
        "Camera Color": "/camera/camera/color/image_raw",
    }
    assert profile["Window Geometry"] == {"Width": 1400, "Height": 900}


def test_rviz_dry_run_is_independent_of_device_and_model_startup():
    result = subprocess.run(
        ["bash", str(ROOT / "sh/run_rviz.sh"), "--time-source", "sim", "--dry-run"],
        cwd="/",
        text=True,
        capture_output=True,
        check=True,
        timeout=10,
    )
    assert "rviz2 -d" in result.stdout and "use_sim_time:=true" in result.stdout
    for helper in ("depth_to_cloud.py", "visualization_tf.py", "result_visualizer.py"):
        assert helper in result.stdout
    assert (
        "start_compressor" not in result.stdout
        and "pointgoal_step" not in result.stdout
    )


def test_memory_rviz_dry_run_loads_exploration_displays_from_another_directory():
    result = subprocess.run(
        ["bash", str(ROOT / "sh/run_rviz_mem.sh"), "--time-source", "sim", "--dry-run"],
        cwd="/",
        text=True,
        capture_output=True,
        check=True,
        timeout=10,
    )
    commands = [shlex.split(line) for line in result.stdout.splitlines()]
    launch = next(command for command in commands if command[0] == "rviz2")
    assert all("use_sim_time:=true" in command for command in commands)
    assert {Path(command[1]).name for command in commands[:-1]} == {
        "depth_to_cloud.py", "visualization_tf.py", "result_visualizer.py",
    }
    profile = yaml.safe_load(Path(launch[launch.index("-d") + 1]).read_text())
    manager = profile["Visualization Manager"]
    assert manager["Global Options"]["Fixed Frame"] == "world"
    displays = {display["Topic"]["Value"]: display for display in manager["Displays"] if "Topic" in display}
    assert displays["/vln/exploration/map"]["Class"] == "rviz_default_plugins/Map"
    assert displays["/vln/exploration/markers"]["Class"] == "rviz_default_plugins/MarkerArray"
    assert "/xnavdp/trajectory" in displays


def test_rviz_records_requested_and_resolved_time_source(tmp_path, monkeypatch):
    from dimos.plugins.xnavdp.visualization import __main__ as visualization

    monkeypatch.setenv("DIMOS_LOG_ROOT", str(tmp_path))
    monkeypatch.setattr(visualization, "bag_clock_present", lambda: True)
    monkeypatch.setattr(visualization, "stop", lambda process: None)
    launched = []

    def spawn(command, *, start_new_session):
        assert start_new_session
        launched.append(command)
        return SimpleNamespace(poll=lambda: 0 if command[0] == "rviz2" else None)

    monkeypatch.setattr(visualization, "subprocess", SimpleNamespace(Popen=spawn))

    assert visualization.main(["--time-source", "auto"]) == 0
    assert all("use_sim_time:=true" in command for command in launched)
    run = next((tmp_path / "rviz").iterdir())
    assert json.loads((run / "run.json").read_text())["metadata"]["time_source"] == "auto"
    events = [json.loads(line) for line in (run / "modules/xnavdp.jsonl").read_text().splitlines()]
    resolved = next(event for event in events if event.get("event") == "rviz_clock_selected")
    assert resolved["use_sim_time"] is True


def test_rviz_clock_probe_failure_is_archived(tmp_path, monkeypatch):
    from dimos.plugins.xnavdp.visualization import __main__ as visualization

    monkeypatch.setenv("DIMOS_LOG_ROOT", str(tmp_path))

    def probe():
        raise RuntimeError("clock probe failed")

    monkeypatch.setattr(visualization, "bag_clock_present", probe)
    with pytest.raises(RuntimeError, match="clock probe failed"):
        visualization.main([])

    run = next((tmp_path / "rviz").iterdir())
    events = [json.loads(line) for line in (run / "modules/xnavdp.jsonl").read_text().splitlines()]
    failed = next(event for event in events if event.get("event") == "rviz_launch_failed")
    assert failed["error_type"] == "RuntimeError"
    assert failed["reason"] == "clock probe failed"


def test_result_visualizer_preserves_reference_colors_and_scores():
    pytest.importorskip("visualization_msgs.msg")
    from builtin_interfaces.msg import Time
    from dimos.plugins.xnavdp.visualization.result_visualizer import ResultVisualizer

    renderer = SimpleNamespace(
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(to_msg=lambda: Time(sec=100))
        )
    )
    data = {
        "trajectory": [[[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]]],
        "all_trajectory": [[[[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]]]],
        "all_values": [[0.3]],
        "goal": [3.0, 0.0],
        "trajectory_frame": "base_link",
        "goal_frame": "base_link",
        "roundtrip_seconds": 0.123,
        "pose_mode": "dimos_odom",
    }
    markers = ResultVisualizer.convert(renderer, data).markers
    by_ns = {m.ns: m for m in markers if m.ns}
    assert by_ns["selected"].color.g == 1.0 and by_ns[
        "selected"
    ].scale.x == pytest.approx(0.045)
    assert by_ns["candidates"].color.b == 1.0 and by_ns[
        "candidates"
    ].scale.x == pytest.approx(0.018)
    assert by_ns["scores"].text == "0: 0.300"
    assert by_ns["goal"].header.frame_id == "base_link"
    assert "Inference 0.123s" in by_ns["info"].text
