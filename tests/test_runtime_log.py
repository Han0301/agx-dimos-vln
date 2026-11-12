import json
import logging
import os
import shutil
import subprocess
import sys
from pathlib import Path


def test_runtime_log_captures_streams_and_only_used_components(
    tmp_path, capsys, monkeypatch
):
    from dimos.plugins.runtime_log import capture, task_context

    monkeypatch.setenv("ROS_LOG_DIR", "/previous/ros")
    with capture("web_input", ros=True, base=tmp_path) as run:
        assert Path(os.environ["ROS_LOG_DIR"]) == run / "ros"
        print("result")
        logging.getLogger("dimos.vln_task.agent").warning("waiting")
        with task_context("task-1"):
            logging.getLogger("dimos.plugins.llm.client").info("done elapsed_ms=3")
        print("error", file=sys.stderr)

    output = capsys.readouterr()
    assert output.out == "result\n"
    assert output.err == "error\n"
    assert os.environ["ROS_LOG_DIR"] == "/previous/ros"
    assert (run / "stdout.log").read_text() == "result\n"
    assert (run / "stderr.log").read_text() == "error\n"
    assert {path.name for path in (run / "modules").iterdir()} == {
        "llm.jsonl",
        "vln_task.jsonl",
    }
    record = json.loads((run / "modules" / "llm.jsonl").read_text())
    assert record["task_id"] == "task-1"
    assert record["message"] == "done elapsed_ms=3"


def test_shell_log_preserves_output_channels_and_exit_status(tmp_path):
    root = Path(__file__).resolve().parents[1]
    script = (
        'source "$1"; dimos_log_start bag_play; '
        'exec bash -c \'printf "data\\n"; printf "error\\n" >&2; exit 17\''
    )
    env = {**os.environ, "DIMOS_LOG_ROOT": str(tmp_path)}
    result = subprocess.run(
        ["bash", "-c", script, "test", str(root / "sh/internal/runtime_log.bash")],
        cwd="/",
        env=env,
        check=False,
        capture_output=True,
        text=True,
        timeout=5,
    )
    assert result.returncode == 17
    assert result.stdout == "data\n"
    assert result.stderr == "error\n"
    (run,) = (tmp_path / "bag_play").iterdir()
    assert (run / "stdout.log").read_text() == "data\n"
    assert (run / "stderr.log").read_text() == "error\n"
    assert (run / "ros").is_dir()


def test_native_writes_are_captured_by_python_entrypoint(tmp_path):
    root = Path(__file__).resolve().parents[1]
    code = """
import os
from dimos.plugins.runtime_log import capture
with capture("body", ros=True):
    os.write(1, b"native data\\n")
    os.write(2, b"native error\\n")
"""
    env = {**os.environ, "DIMOS_LOG_ROOT": str(tmp_path)}
    result = subprocess.run(
        [sys.executable, "-c", code],
        cwd=root,
        env=env,
        check=True,
        capture_output=True,
        text=True,
        timeout=5,
    )
    assert result.stdout == "native data\n"
    assert result.stderr == "native error\n"
    (run,) = (tmp_path / "body").iterdir()
    assert (run / "stdout.log").read_text() == result.stdout
    assert (run / "stderr.log").read_text() == result.stderr


def test_motion_launcher_collects_integrated_console_output(tmp_path):
    root = Path(__file__).resolve().parents[1]
    for relative in (
        "sh/start_vln_motion.sh",
        "sh/internal/runtime_log.bash",
        "dimos/__init__.py",
        "dimos/plugins/__init__.py",
    ):
        target = tmp_path / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(root / relative, target)
    shutil.copytree(
        root / "dimos/plugins/runtime_log", tmp_path / "dimos/plugins/runtime_log"
    )
    (tmp_path / ".venv").symlink_to(Path(sys.executable).parent.parent)
    config = tmp_path / "motion.yaml"
    config.write_text("web_profile: web.yaml\ntracker_config: tracker.yaml\n")
    (tmp_path / "sh/start_web_input.sh").write_text(
        '[[ "$*" == "--profile web.yaml --motion-tracker-config tracker.yaml" ]] || exit 13\n'
        'printf "web ready\\n"\n'
        'printf "tracker ready\\n" >&2\n'
    )
    (tmp_path / "sh/start_tracker.sh").write_text(
        'touch tracker_started_separately\nexit 14\n'
    )

    env = {**os.environ, "DIMOS_LOG_ROOT": str(tmp_path / "logs")}
    result = subprocess.run(
        ["bash", "sh/start_vln_motion.sh", "--config", str(config)],
        cwd=tmp_path,
        env=env,
        check=True,
        capture_output=True,
        text=True,
        timeout=6,
    )
    assert not (tmp_path / "tracker_started_separately").exists()
    assert result.stdout == "web ready\n"
    assert result.stderr == "tracker ready\n"
    (run,) = (tmp_path / "logs/vln_motion").iterdir()
    assert (run / "stdout.log").read_text() == result.stdout
    assert (run / "stderr.log").read_text() == result.stderr


def test_web_capture_records_tracker_info(tmp_path):
    import json
    import logging

    from dimos.plugins.runtime_log import capture

    logger = logging.getLogger("dimos.path_tracker")
    prior = logger.level
    logger.setLevel(logging.NOTSET)
    try:
        with capture("web_input", base=tmp_path) as run:
            logger.info("velocity state changed reason=goal_reached linear_x=0.000")
        event = json.loads((run / "modules/path_tracker.jsonl").read_text())
        assert event["level"] == "INFO"
        assert "goal_reached" in event["message"]
    finally:
        logger.setLevel(prior)
