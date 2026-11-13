"""Register bounded spatial-memory image evidence with the runtime recorder."""

import json
import logging
import re

from dimos.plugins.runtime_log import BackgroundWriter, create_run, event

logger = logging.getLogger("dimos.plugins.spatial_memory")
_MAX_IMAGE_BYTES = 1024 * 1024
_MAX_TOTAL_BYTES = 16 * 1024 * 1024
_MAX_IMAGES = 32
_IMAGE_ID = re.compile(r"[a-zA-Z0-9][a-zA-Z0-9_.-]{0,95}")


class MemoryArchive:
    def __init__(self):
        self.run = None
        self.writer = None
        self.count = 0
        self.size = 0
        self.unavailable = False
        try:
            run = create_run("spatial_memory")
            if run is not None:
                self.run = run
                self.writer = BackgroundWriter(
                    self._write, self._report, capacity=8, name="spatial-memory-images",
                    report=self._report,
                )
        except (OSError, RuntimeError, ValueError) as exc:
            self.unavailable = True
            event(logger, "simple_memory_archive_unavailable", reason="archive_init_failed",
                  error_type=type(exc).__name__)

    def relative_path(self, relative):
        if self.run is None:
            return None
        return (self.run / relative).relative_to(self.run.parent.parent).as_posix()

    def offer(self, memory_id, jpeg):
        if self.writer is None:
            return None, "archive_unavailable" if self.unavailable else "recording_off"
        if not isinstance(memory_id, str) or not _IMAGE_ID.fullmatch(memory_id):
            return None, "invalid_memory_id"
        size = len(jpeg)
        if not 0 < size <= _MAX_IMAGE_BYTES:
            return None, "image_size_limit"
        if self.count >= _MAX_IMAGES or self.size + size > _MAX_TOTAL_BYTES:
            return None, "archive_limit"
        relative = f"images/{memory_id}.jpg"
        if not self.writer.offer((relative, jpeg)):
            return None, "archive_queue_full"
        self.count += 1
        self.size += size
        return self.relative_path(relative), "queued"

    def _write(self, item):
        relative, data = item
        target = self.run / relative
        target.parent.mkdir(exist_ok=True)
        temporary = target.with_suffix(".tmp")
        try:
            temporary.write_bytes(data)
            temporary.replace(target)
        finally:
            temporary.unlink(missing_ok=True)

    def _report(self, dropped, errors):
        if dropped or errors:
            (self.run / "writer_health.json").write_text(
                json.dumps({"dropped_images": dropped, "write_errors": errors}) + "\n",
                encoding="utf-8",
            )

    def close(self):
        if self.writer is not None:
            self.writer.close()
