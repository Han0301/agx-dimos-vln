"""A Web launch replaces only its own recorded project/domain process."""

import subprocess
import sys


def test_web_session_gracefully_replaces_recorded_instance(tmp_path):
    try:
        from dimos.plugins.web_input.session import WebSession
    except ModuleNotFoundError:
        __import__("pytest").fail("Web lifecycle session is missing")
    code = """
import signal, sys, threading
from pathlib import Path
from dimos.plugins.web_input.session import WebSession
done = threading.Event()
signal.signal(signal.SIGINT, lambda *_: done.set())
with WebSession(Path(sys.argv[1]), '91'):
    print('ready', flush=True)
    done.wait(10)
"""
    child = subprocess.Popen(
        [sys.executable, "-c", code, str(tmp_path)], stdout=subprocess.PIPE, text=True
    )
    try:
        assert child.stdout.readline().strip() == "ready"
        with WebSession(tmp_path, "92"):
            assert child.poll() is None
        with WebSession(tmp_path, "91"):
            child.wait(timeout=3)
            assert child.returncode == 0
    finally:
        if child.poll() is None:
            child.terminate()
            child.wait(timeout=3)
