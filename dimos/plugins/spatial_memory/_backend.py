"""Choose the CLIP inference backend inside the Simple worker."""

import sys


class MemorySession:
    def __init__(self, gpu, cpu):
        self._cpu = cpu
        if gpu is None:
            self._session = cpu()
            self.backend = "cpu"
            return
        self.backend = "tensorrt"
        try:
            self._session = gpu()
        except Exception as exc:
            self._fallback(exc)

    def _fallback(self, exc):
        print(f"spatial_memory backend=cpu reason={type(exc).__name__}: {exc}",
              file=sys.stderr, flush=True)
        self._session = self._cpu()
        self.backend = "cpu"

    def get_inputs(self):
        return self._session.get_inputs()

    def get_outputs(self):
        return self._session.get_outputs()

    def run(self, outputs, inputs):
        try:
            return self._session.run(outputs, inputs)
        except Exception as exc:
            if self.backend == "cpu":
                raise
            close = getattr(self._session, "close", None)
            if close is not None:
                close()
            self._fallback(exc)
            return self._session.run(outputs, inputs)
