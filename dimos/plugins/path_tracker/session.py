"""Single local tracker per project/domain; graceful replacement only."""

import fcntl
import os
import signal
import time
from pathlib import Path

from dimos.config import ROOT


def identity(pid):
    try:
        proc = Path(f"/proc/{pid}")
        fields = (proc / "stat").read_text().rsplit(")", 1)[1].split()
        if proc.stat().st_uid == os.getuid() and fields[0] != "Z":
            return fields[19]
    except (OSError, IndexError):
        pass
    return None


def previous():
    domain = os.environ.get("ROS_DOMAIN_ID", "0").encode()
    for proc in Path("/proc").iterdir():
        if not proc.name.isdigit() or int(proc.name) == os.getpid():
            continue
        try:
            args = (proc / "cmdline").read_bytes().split(b"\0")
            index = args.index(b"-m")
            if (
                index > 3
                or b"-c" in args[:index]
                or args[index + 1] != b"dimos.plugins.path_tracker"
            ):
                continue
            if any(arg in args for arg in (b"--check", b"--help", b"-h")):
                continue
            env = dict(
                item.split(b"=", 1)
                for item in (proc / "environ").read_bytes().split(b"\0")
                if b"=" in item
            )
            if env.get(b"ROS_DOMAIN_ID", b"0") != domain:
                continue
            if (proc / "cwd").resolve() != ROOT and str(ROOT).encode() not in env.get(
                b"PYTHONPATH", b""
            ).split(b":"):
                continue
            pid = int(proc.name)
            ident = identity(pid)
            if ident is not None:
                yield pid, ident
        except (OSError, ValueError, IndexError):
            continue


class TrackerSession:
    def __enter__(self):
        self.lease = None
        directory = ROOT / "run/modules"
        directory.mkdir(parents=True, exist_ok=True)
        prefix = (
            directory
            / f"path_tracker-{os.getuid()}-{int(os.environ.get('ROS_DOMAIN_ID', '0'))}"
        )
        with Path(f"{prefix}.gate").open("a+") as gate:
            fcntl.flock(gate, fcntl.LOCK_EX)
            old = list(previous())
            for pid, ident in old:
                if identity(pid) == ident:
                    print(f"Stopping previous tracker PID {pid}", flush=True)
                    try:
                        os.kill(pid, signal.SIGINT)
                    except ProcessLookupError:
                        pass
            deadline = time.monotonic() + 30
            while any(identity(pid) == ident for pid, ident in old):
                if time.monotonic() >= deadline:
                    raise RuntimeError(
                        "Previous tracker did not stop; no new tracker started"
                    )
                time.sleep(0.1)
            lease = Path(f"{prefix}.lease").open("a+")
            try:
                fcntl.flock(lease, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BaseException:
                lease.close()
                raise
            self.lease = lease
        return self

    def __exit__(self, *_):
        if self.lease is not None:
            self.lease.close()
