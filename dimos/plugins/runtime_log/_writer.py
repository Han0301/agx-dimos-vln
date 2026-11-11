"""Bounded best-effort writers for optional diagnostic output."""

import time
from queue import Empty, Full, Queue
from threading import Event, Thread


class BackgroundWriter:
    def __init__(self, write, finish, *, capacity, name, report=None):
        self._write = write
        self._finish = finish
        self._report = report
        self._queue = Queue(maxsize=capacity)
        self._stopping = Event()
        self.dropped = 0
        self.errors = 0
        self._thread = Thread(target=self._run, name=name, daemon=True)
        self._thread.start()

    def offer(self, item):
        if self._stopping.is_set():
            return False
        try:
            self._queue.put_nowait(item)
        except Full:
            self.dropped += 1
            return False
        return True

    def close(self, timeout=0.5):
        self._stopping.set()
        self._thread.join(timeout)

    def _run(self):
        reported = (0, 0)
        next_report = 0
        while not self._stopping.is_set() or not self._queue.empty():
            try:
                item = self._queue.get(timeout=0.05)
            except Empty:
                pass
            else:
                try:
                    self._write(item)
                except Exception:
                    self.errors += 1
            counts = self.dropped, self.errors
            if self._report is not None and counts != reported and time.monotonic() >= next_report:
                next_report = time.monotonic() + 1.0
                try:
                    self._report(*counts)
                except OSError:
                    pass
                else:
                    reported = counts
        try:
            self._finish(self.dropped, self.errors)
        except OSError:
            pass


class LogFile:
    """Write and rotate on the worker, independently of logging.shutdown()."""

    def __init__(self, path, *, max_bytes=10 * 1024 * 1024):
        self._path = path
        self._max_bytes = max_bytes
        self._stream = None
        self._size = 0

    def write(self, data):
        if self._stream is None:
            self._stream = self._path.open("ab", buffering=0)
            self._size = self._stream.tell()
        if self._size and self._size + len(data) > self._max_bytes:
            self.close()
            for index in (2, 1):
                source = self._path.with_name(f"{self._path.name}.{index}")
                if source.exists():
                    source.replace(self._path.with_name(f"{self._path.name}.{index + 1}"))
            self._path.replace(self._path.with_name(f"{self._path.name}.1"))
            self._stream = self._path.open("ab", buffering=0)
            self._size = 0
        written = self._stream.write(data)
        if written != len(data):
            self._stream.truncate(self._size)
            raise OSError("Incomplete diagnostic file write")
        self._size += written

    def close(self):
        if self._stream is not None:
            self._stream.close()
            self._stream = None


class ConsoleFile(LogFile):
    def close(self, dropped=None, errors=None):
        super().close()
        if dropped or errors:
            self._path.with_suffix(".capture.json").write_text(
                f'{{"dropped_chunks": {dropped}, "write_errors": {errors}}}\n',
                encoding="utf-8",
            )
