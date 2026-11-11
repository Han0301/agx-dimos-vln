"""Independent recording/playback and compressor command boundaries."""

import importlib.util
import json
import os
import shutil
import signal
import subprocess
import time
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def workspace(tmp_path):
    files = [
        "sh/record_bag.sh",
        "sh/play_bag.sh",
        "sh/internal/rosbag_env.bash",
        "sh/internal/runtime_log.bash",
        "sh/xnavdp_ros2/dds_env.sh",
        "dimos/__init__.py",
        "dimos/plugins/__init__.py",
        "camera_ws/start_cameras.sh",
        "camera_ws/sh/start_cameras.sh",
        "camera_ws/config/camera_stack.conf",
        "camera_ws/start_compressor.sh",
        "camera_ws/sh/internal/start_compressor.py",
    ]
    for name in files:
        target = tmp_path / name
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(ROOT / name, target)
    log_package = "dimos/plugins/runtime_log"
    shutil.copytree(
        ROOT / log_package, tmp_path / log_package,
        ignore=shutil.ignore_patterns("__pycache__"),
    )
    binary = tmp_path / "fake-bin"
    binary.mkdir()
    capture = tmp_path / "capture.py"
    capture.write_text(
        "import json,os,signal,sys\nfrom pathlib import Path\n"
        "def stop(*_):\n"
        '    print("native interrupted", file=sys.stderr)\n'
        "    sys.exit(130)\n"
        'wait_for_signal = os.environ.get("BAG_TEST_WAIT_FOR_SIGINT") == "1"\n'
        "if wait_for_signal:\n"
        "    signal.signal(signal.SIGINT, stop)\n"
        'Path(os.environ["BAG_TEST_CAPTURE"]).write_text(json.dumps({'
        '"args": sys.argv[1:], "pythonpath": os.environ.get("PYTHONPATH"), '
        '"ros_log_dir": os.environ.get("ROS_LOG_DIR")}))\n'
        'print("native stdout")\n'
        'print("native stderr", file=sys.stderr)\n'
        "if wait_for_signal:\n"
        "    sys.stdout.flush()\n"
        '    Path(os.environ["BAG_TEST_CAPTURE"]).with_suffix(".ready").touch()\n'
        "    signal.pause()\n"
        'sys.exit(int(os.environ.get("BAG_TEST_EXIT", "0")))\n'
    )
    (binary / "ros2").write_text(
        f'#!/usr/bin/env bash\nexec /usr/bin/python3 "{capture}" "$@"\n'
    )
    (binary / "ros2").chmod(0o755)
    setup = tmp_path / "fake-ros-setup.bash"
    setup.write_text(f'export PATH="{binary}:$PATH"\n')
    (tmp_path / "config").mkdir()
    (tmp_path / "config/bag.yaml").write_text(
        yaml.safe_dump(
            {
                "rosbag_setup": str(setup),
                "bag_path": "",
                "rate": 1.0,
                "loop": False,
                "clock_hz": 100.0,
            }
        )
    )
    return tmp_path


def bag_environment(root, exit_code=0):
    return dict(
        os.environ,
        BAG_TEST_CAPTURE=str(root / "captured.json"),
        BAG_TEST_EXIT=str(exit_code),
        DIMOS_LOG_ROOT=str(root / "data/log"),
        PYTHONPATH="/caller/overlay",
    )


def invoke(root, entry, *args, exit_code=0):
    return subprocess.run(
        ["bash", str(root / entry), *map(str, args)],
        cwd="/",
        env=bag_environment(root, exit_code),
        check=False,
        capture_output=True,
        text=True,
        timeout=10,
    )


def make_bag(root, empty=False):
    bag = root / "bag with spaces"
    bag.mkdir()
    (bag / "storage.mcap").write_bytes(b"metadata-only test; native ros2 is stubbed")
    (bag / "metadata.yaml").write_text(
        yaml.safe_dump(
            {
                "rosbag2_bagfile_information": {
                    "message_count": 0 if empty else 2,
                    "relative_file_paths": ["storage.mcap"],
                    "starting_time": {"nanoseconds_since_epoch": 1712345678000000123},
                    "duration": {"nanoseconds": 2100000000},
                    "topics_with_message_count": [
                        {
                            "topic_metadata": {
                                "name": "/custom_topic",
                                "type": "std_msgs/msg/String",
                            },
                            "message_count": 2,
                        }
                    ],
                }
            }
        )
    )
    return bag


def test_record_only_requested_topics_and_fixed_output(workspace):
    result = invoke(workspace, "sh/record_bag.sh", "/odom", "/custom/topic", "/odom")
    assert result.returncode == 0, result.stderr
    data = json.loads((workspace / "captured.json").read_text())
    args = data["args"]
    assert args[:4] == ["bag", "record", "--storage", "mcap"]
    assert args[6:] == ["--topics", "/odom", "/custom/topic"]
    path = Path(args[5])
    assert path.parent == workspace / "data/bags" and path.name.startswith("record_")
    assert data["pythonpath"] is None


def test_record_dry_run_has_no_side_effects_and_unique_destination(workspace):
    before = sorted(str(p.relative_to(workspace)) for p in workspace.rglob("*"))
    one = invoke(workspace, "sh/record_bag.sh", "/odom", "--dry-run")
    two = invoke(workspace, "sh/record_bag.sh", "/odom", "--dry-run")
    assert one.returncode == two.returncode == 0
    assert "bag record" in one.stdout and one.stdout != two.stdout
    assert sorted(str(p.relative_to(workspace)) for p in workspace.rglob("*")) == before


@pytest.mark.parametrize(
    "args", [[], ["--all"], ["relative/topic"], ["/bad;name"], ["/1name"]]
)
def test_invalid_record_arguments_do_not_create_output(workspace, args):
    result = invoke(workspace, "sh/record_bag.sh", *args)
    assert result.returncode != 0
    assert (
        not (workspace / "data").exists() and not (workspace / "captured.json").exists()
    )


def test_native_record_failure_is_propagated(workspace):
    result = invoke(workspace, "sh/record_bag.sh", "/odom", exit_code=17)
    assert result.returncode == 17
    assert "native stdout" in result.stdout
    assert "native stderr" in result.stderr
    data = json.loads((workspace / "captured.json").read_text())
    ros_log_dir = Path(data["ros_log_dir"])
    assert ros_log_dir.is_relative_to(workspace / "data/log/bag_record")
    assert "native stdout" in (ros_log_dir.parent / "stdout.log").read_text()
    assert "native stderr" in (ros_log_dir.parent / "stderr.log").read_text()


def test_record_sigint_preserves_last_native_output(workspace):
    env = bag_environment(workspace)
    env["BAG_TEST_WAIT_FOR_SIGINT"] = "1"
    child = subprocess.Popen(
        ["bash", str(workspace / "sh/record_bag.sh"), "/odom"],
        cwd="/",
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        start_new_session=True,
    )
    try:
        capture = workspace / "captured.json"
        deadline = time.monotonic() + 5
        while not capture.with_suffix(".ready").is_file():
            assert child.poll() is None
            assert time.monotonic() < deadline
            time.sleep(0.02)
        os.killpg(child.pid, signal.SIGINT)
        stdout, stderr = child.communicate(timeout=5)
        assert child.returncode == 130
        assert "native stdout" in stdout and "native interrupted" in stderr
        data = json.loads(capture.read_text())
        run = Path(data["ros_log_dir"]).parent
        assert json.loads((run / "run.json").read_text())["pid"] == child.pid
        assert "native interrupted" in (run / "stderr.log").read_text()
    finally:
        if child.poll() is None:
            os.killpg(child.pid, signal.SIGKILL)
            child.communicate(timeout=5)


def test_play_is_player_only_with_clock_and_accepts_arbitrary_topics(workspace):
    path = make_bag(workspace)
    result = invoke(
        workspace, "sh/play_bag.sh", "--bag", path.name, "--rate", ".5", "--loop"
    )
    assert result.returncode == 0, result.stderr
    args = json.loads((workspace / "captured.json").read_text())["args"]
    assert args == [
        "bag",
        "play",
        str(path),
        "--clock",
        "100.0",
        "--rate",
        ".5",
        "--loop",
    ]
    assert "compressor" not in result.stdout.lower()

    run = next((workspace / "data/log/bag_play").glob("*/run.json"))
    metadata = json.loads(run.read_text())["metadata"]
    assert metadata["bag_path"] == str(path)
    assert float(metadata["rate"]) == 0.5
    assert str(metadata["loop"]).lower() == "true"
    assert float(metadata["clock_hz"]) == 100.0
    assert int(metadata["bag_start_stamp_ns"]) == 1712345678000000123
    assert int(metadata["bag_duration_ns"]) == 2100000000
    assert int(metadata["bag_message_count"]) == 2
    assert json.loads(metadata["bag_topic_coverage"]) == {
        "counts": {"/custom_topic": 2}, "omitted": 0,
    }


def test_play_dry_run_does_not_launch_or_add_other_modules(workspace):
    path = make_bag(workspace)
    result = invoke(workspace, "sh/play_bag.sh", "--bag", path, "--dry-run")
    assert result.returncode == 0, result.stderr
    assert "--clock 100.0" in result.stdout and "launch" not in result.stdout
    assert not (workspace / "captured.json").exists()
    assert not (workspace / "data/log").exists()


def test_empty_bag_is_rejected(workspace):
    path = make_bag(workspace, empty=True)
    result = invoke(workspace, "sh/play_bag.sh", "--bag", path, "--dry-run")
    assert result.returncode != 0 and "0 messages" in result.stderr


@pytest.mark.parametrize("rate", ["0", "-1", "nan"])
def test_bad_play_rate_is_rejected(workspace, rate):
    path = make_bag(workspace)
    result = invoke(
        workspace, "sh/play_bag.sh", "--bag", path, "--rate", rate, "--dry-run"
    )
    assert result.returncode != 0 and "positive and finite" in result.stderr


def test_camera_entry_rejects_bag(workspace):
    result = invoke(
        workspace, "camera_ws/start_cameras.sh", "--camera", "d455bag", "--dry-run"
    )
    assert result.returncode != 0 and "Unknown camera" in result.stderr


def test_camera_start_logs_only_the_selected_group(workspace):
    camera = workspace / "camera_ws"
    setup = camera / "sh/setup_local.bash"
    setup.write_text(f'export PATH="{workspace / "fake-bin"}:$PATH"\n')
    (camera / "install").mkdir()
    (camera / "install/local_setup.bash").touch()
    (camera / "config/xnavdp_dds.bash").touch()
    python = workspace / "fake-camera-python"
    python.write_text(
        "#!/usr/bin/python3\n"
        "import os, sys, time\n"
        "from pathlib import Path\n"
        'if sys.argv[1] == "-":\n'
        "    sys.stdin.read()\n"
        'elif sys.argv[1] == "-u":\n'
        '    output = Path(os.environ["CAMERA_STACK_LOG_DIR"]) / "x3.log"\n'
        "    for _ in range(100):\n"
        '        if output.is_file() and "native stderr" in output.read_text():\n'
        "            sys.exit(0)\n"
        "        time.sleep(0.02)\n"
        "    sys.exit(2)\n"
    )
    python.chmod(0o755)
    with (camera / "config/camera_stack.conf").open("a") as config:
        config.write(f'\nPYTHON_BIN="{python}"\n')

    result = invoke(workspace, "camera_ws/start_cameras.sh", "--camera", "x3")
    assert result.returncode == 0, result.stderr
    (session,) = (workspace / "data/log/cameras").iterdir()
    assert "native stdout" in (session / "x3.log").read_text()
    assert not (session / "d455.log").exists()
    data = json.loads((workspace / "captured.json").read_text())
    assert Path(data["ros_log_dir"]) == session / "ros"


@pytest.mark.parametrize(
    "mode,clock,expected",
    [
        ("auto", True, True),
        ("auto", False, False),
        ("sim", False, True),
        ("wall", True, False),
    ],
)
def test_compressor_time_selection_uses_existing_launch(
    monkeypatch, mode, clock, expected
):
    path = ROOT / "camera_ws/sh/internal/start_compressor.py"
    spec = importlib.util.spec_from_file_location("compressor_start_test", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, "inspect_source", lambda *_: (clock, False, None))
    commands = []
    monkeypatch.setattr(module.os, "execvp", lambda _, args: commands.append(args))
    module.main(["--time-source", mode])
    assert f"use_sim_time:={str(expected).lower()}" in commands[0]
    assert commands[0][0:2] == ["ros2", "launch"]


def test_compressor_reuses_existing_compressed_inputs(monkeypatch):
    path = ROOT / "camera_ws/sh/internal/start_compressor.py"
    spec = importlib.util.spec_from_file_location("compressor_reuse_test", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, "inspect_source", lambda *_: (True, True, None))
    monkeypatch.setattr(
        module.os, "execvp", lambda *_: pytest.fail("duplicate compressor")
    )
    assert module.main([]) == 0


def test_high_frequency_entries():
    assert {p.name for p in (ROOT / "sh").iterdir() if p.is_file()} == {
        "start.sh",
        "start_xnavdp.sh",
        "start_tracker.sh",
        "start_web_input.sh",
        "start_log.sh",
        "start_vln_motion.sh",
        "env.sh",
        "play_bag.sh",
        "record_bag.sh",
        "run_rviz.sh",
    }
    assert not (ROOT / "sh/start_camera.sh").exists()
