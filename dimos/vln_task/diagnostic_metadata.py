"""Code, model and effective-config provenance for a recorded VLN run."""

import hashlib
import os
import subprocess
from pathlib import Path

from dimos.plugins.runtime_log._redaction import redact_config


def _checkout(root):
    def git(*args):
        try:
            return subprocess.run(
                ["git", "-C", str(root), *args], capture_output=True,
                text=True, check=True, timeout=1.0,
            ).stdout.strip()
        except (OSError, subprocess.SubprocessError):
            return None

    commit = git("rev-parse", "HEAD")
    status = git("status", "--porcelain", "--untracked-files=normal") if commit else None
    return {"root": str(root), "commit": commit, "dirty": bool(status) if status is not None else None}


def _model_file(path):
    try:
        stat = path.stat()
        # Bound startup IO for large model weights and label the digest as sampled.
        with path.open("rb") as stream:
            first = stream.read(1024 * 1024)
            stream.seek(max(len(first), stat.st_size - 1024 * 1024))
            last = stream.read(1024 * 1024)
        digest = hashlib.sha256(first + last).hexdigest()
    except OSError:
        return {"path": str(path), "available": False}
    return {
        "path": str(path), "available": True, "size_bytes": stat.st_size,
        "mtime_ns": stat.st_mtime_ns, "first_last_mib_sha256": digest,
    }


def diagnostics_metadata(root, effective_config):
    metadata = {
        "effective_config": redact_config(effective_config),
        "checkouts": {"mininum": _checkout(root)},
    }
    if effective_config["exploration_memory"]:
        simple = Path(os.environ.get("DIMOS_SIMPLE_ROOT") or root.parent / "dimos_simple").expanduser().resolve()
        metadata["checkouts"]["simple"] = _checkout(simple)
        metadata["spatial_memory_model"] = _model_file(simple / "data/models_clip/model.onnx")
    return metadata
