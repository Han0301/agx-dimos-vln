"""Compatibility entry for the upper-layer benchmark and its public helpers."""
import importlib
import sys

_implementation = importlib.import_module("dimos.vln_task.benchmark")
if __name__ == "__main__":
    raise SystemExit(_implementation.main())
else:
    sys.modules[__name__] = _implementation
