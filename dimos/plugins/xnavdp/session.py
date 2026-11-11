"""Replace only this project's local inference client, never the model service."""

import fcntl
import os
import signal
import time
from pathlib import Path

from dimos.config import ROOT


def _identity(pid):
    """Include kernel start ticks to protect against PID reuse."""
    try:
        proc = Path(f"/proc/{pid}")
        stat = (proc / "stat").read_text().rsplit(")", 1)[1].split()
        if proc.stat().st_uid != os.getuid() or stat[0] == "Z":
            return None
        return stat[19]
    except (OSError, IndexError):
        return None


def _previous_clients():
    domain = os.environ.get("ROS_DOMAIN_ID", "0")
    for proc in Path("/proc").iterdir():
        if not proc.name.isdigit() or int(proc.name) == os.getpid():
            continue
        try:
            argv = (proc / "cmdline").read_bytes().split(b"\0")
            # Exact module invocation excludes visualization and remote workers.
            index = argv.index(b"-m")
            if b"-c" in argv[:index] or index > 3:
                continue
            if argv[index + 1] != b"dimos.plugins.xnavdp":
                continue
            if b"--check" in argv or b"--help" in argv:
                continue
            env = dict(
                item.split(b"=", 1)
                for item in (proc / "environ").read_bytes().split(b"\0")
                if b"=" in item
            )
            if env.get(b"ROS_DOMAIN_ID", b"0").decode() != domain:
                continue
            paths = env.get(b"PYTHONPATH", b"").decode().split(":")
            if (proc / "cwd").resolve() != ROOT and str(ROOT) not in paths:
                continue
            from .__main__ import load_test_config

            args = [item.decode() for item in argv[index + 2:] if item]
            config = ROOT / "config/plugins/xnavdp/xnavdp_test.yaml"
            mode = None
            for offset, argument in enumerate(args):
                if argument == "--mode":
                    mode = args[offset + 1]
                elif argument.startswith("--mode="):
                    mode = argument.split("=", 1)[1]
                elif argument == "--config":
                    config = Path(args[offset + 1])
                elif argument.startswith("--config="):
                    config = Path(argument.split("=", 1)[1])
            if not Path(config).is_absolute():
                config = (proc / "cwd").resolve() / config
            if load_test_config(config, mode)["mode"] != "inference":
                continue
            pid = int(proc.name)
            identity = _identity(pid)
            if identity is not None:
                yield pid, identity
        except (OSError, ValueError, IndexError):
            continue


class InferenceSession:
    """Serialize replacement; release the lease only after client cleanup."""

    def __enter__(self):
        self.lease = None
        directory = ROOT / "run/modules"
        directory.mkdir(parents=True, exist_ok=True)
        domain = int(os.environ.get("ROS_DOMAIN_ID", "0"))
        prefix = directory / f"xnavdp-{os.getuid()}-{domain}"
        with Path(f"{prefix}.gate").open("a+") as gate:
            fcntl.flock(gate, fcntl.LOCK_EX)
            previous = list(_previous_clients())
            for pid, identity in previous:
                if _identity(pid) == identity:
                    print(f"Stopping previous XNavDP client PID {pid}", flush=True)
                    try:
                        os.kill(pid, signal.SIGINT)
                    except ProcessLookupError:
                        pass
            deadline = time.monotonic() + 30.0
            while any(_identity(pid) == ident for pid, ident in previous):
                if time.monotonic() >= deadline:
                    raise RuntimeError("Previous XNavDP client did not stop; no new client started")
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
