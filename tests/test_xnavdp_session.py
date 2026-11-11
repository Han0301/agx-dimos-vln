"""Process replacement without ROS, cameras, or a remote model."""
import subprocess
import sys
import time

from dimos.plugins.xnavdp import session


def test_replacement_stops_only_previous_client(tmp_path, monkeypatch):
    old = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(60)'])
    unrelated = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(60)'])
    try:
        time.sleep(0.2)
        identity = session._identity(old.pid)
        monkeypatch.setattr(session, 'ROOT', tmp_path)
        monkeypatch.setattr(session, '_previous_clients', lambda: iter([(old.pid, identity)]))
        with session.InferenceSession():
            assert old.wait(timeout=2) != 0
            assert unrelated.poll() is None
        # The previous lease was released, so another startup succeeds.
        with session.InferenceSession():
            assert unrelated.poll() is None
    finally:
        for proc in (old, unrelated):
            if proc.poll() is None:
                proc.terminate()
            proc.wait(timeout=3)


def test_changed_identity_is_never_signalled(tmp_path, monkeypatch):
    monkeypatch.setattr(session, 'ROOT', tmp_path)
    monkeypatch.setattr(session, '_previous_clients', lambda: iter([(123, 'old')]))
    monkeypatch.setattr(session, '_identity', lambda pid: 'new')
    def forbidden(*args):
        raise AssertionError('PID has been reused')
    monkeypatch.setattr(session.os, 'kill', forbidden)
    with session.InferenceSession():
        pass
