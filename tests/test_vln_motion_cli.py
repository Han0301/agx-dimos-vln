import json
import subprocess
import sys
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
PROFILE = "config/profiles/vln_preview.yaml"
TRACKER = "config/plugins/path_tracker/tracker_motion.yaml"


def check(*args):
    return subprocess.run(
        [sys.executable, "-m", "dimos.plugins.web_input", "--check", *args],
        cwd=ROOT,
        capture_output=True,
        text=True,
        timeout=5,
    )


def test_web_motion_check_exposes_effective_profile_and_tracker():
    result = check("--profile", PROFILE, "--motion-tracker-config", TRACKER)
    assert result.returncode == 0, result.stderr
    decoder = json.JSONDecoder()
    profile, index = decoder.raw_decode(result.stdout)
    motion_tracker = decoder.decode(result.stdout[index:].lstrip())
    assert profile["tracker"]["output_only"] is True
    assert motion_tracker["output_only"] is False
    assert motion_tracker["topics"]["cmd_vel"] == "/cmd_vel"


@pytest.mark.parametrize(
    ("args", "message"),
    [
        (("--motion-tracker-config", TRACKER), "requires --profile"),
        (
            ("--profile", PROFILE, "--motion-tracker-config", TRACKER, "--locate-only"),
            "incompatible with --locate-only",
        ),
        (
            (
                "--profile",
                PROFILE,
                "--motion-tracker-config",
                "config/plugins/path_tracker/tracker.yaml",
            ),
            "requires output_only: false",
        ),
    ],
)
def test_web_rejects_inconsistent_motion_config(args, message):
    result = check(*args)
    assert result.returncode != 0
    assert message in result.stderr
    assert not result.stdout


def test_task_console_shows_events_and_hides_successful_status_polls(capsys):
    import logging

    from dimos.plugins.runtime_log import task_console, task_context

    agent = logging.getLogger("dimos.plugins.vln_task.agent")
    tracker = logging.getLogger("dimos.path_tracker")
    access = logging.getLogger("uvicorn.access")
    previous_level = access.level
    access.setLevel(logging.INFO)
    access_handler = logging.StreamHandler()
    access.addHandler(access_handler)

    def request(method, path, status):
        access.info('%s - "%s %s HTTP/%s" %d', "local", method, path, "1.1", status)

    try:
        with task_console():
            with task_context("abc123"):
                agent.info("tool started name=find_object")
            tracker.info("path accepted request_stamp_ns=1")
            request("GET", "/status?task_id=abc123", 200)
            request("GET", "/status", 404)
            request("POST", "/navigate", 202)
        request("GET", "/status?after", 200)
    finally:
        access.removeHandler(access_handler)
        access.setLevel(previous_level)

    stderr = capsys.readouterr().err
    assert "task_id=abc123 tool started name=find_object" in stderr
    assert "path accepted request_stamp_ns=1" in stderr
    assert "/status?task_id=abc123" not in stderr
    assert "/status HTTP/1.1" in stderr
    assert "/navigate HTTP/1.1" in stderr
    assert "/status?after" in stderr
