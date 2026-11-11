"""Gracefully replace the recorded Web instance for this project and domain."""

import fcntl
import json
import os
import signal
import time
from pathlib import Path

from dimos.plugins._agent_common import ROOT


def identity(pid):
    try:
        proc = Path(f"/proc/{pid}")
        fields = (proc / "stat").read_text().rsplit(")", 1)[1].split()
        if proc.stat().st_uid == os.getuid() and fields[0] != "Z":
            return fields[19]
    except (OSError, IndexError):
        pass
    return None


class WebSession:
    def __init__(self, root=ROOT, domain=None):
        domain = int(
            domain if domain is not None else os.environ.get("ROS_DOMAIN_ID", "0")
        )
        self.directory = Path(root) / "run/modules"
        self.prefix = self.directory / f"web_input-{os.getuid()}-{domain}"
        self.record = Path(f"{self.prefix}.json")
        self.owner = {"pid": os.getpid(), "start": identity(os.getpid())}

    def __enter__(self):
        self.directory.mkdir(parents=True, exist_ok=True)
        with Path(f"{self.prefix}.gate").open("a+") as gate:
            fcntl.flock(gate, fcntl.LOCK_EX)
            try:
                old = json.loads(self.record.read_text())
            except FileNotFoundError:
                old = None
            if (
                old
                and old["pid"] != os.getpid()
                and identity(old["pid"]) == old["start"]
            ):
                try:
                    os.kill(old["pid"], signal.SIGINT)
                except ProcessLookupError:
                    pass
                deadline = time.monotonic() + 65
                while identity(old["pid"]) == old["start"]:
                    if time.monotonic() >= deadline:
                        raise RuntimeError("Previous Web instance has not stopped")
                    time.sleep(0.05)
            self.record.write_text(json.dumps(self.owner))
        return self

    def __exit__(self, *_):
        try:
            if json.loads(self.record.read_text()) == self.owner:
                self.record.unlink()
        except FileNotFoundError:
            pass
