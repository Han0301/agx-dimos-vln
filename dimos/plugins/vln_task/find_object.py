"""Compatibility entry; implementation is owned by the VLN Task upper layer."""
import importlib
import sys

_implementation = importlib.import_module("dimos.vln_task.find_object")
sys.modules[__name__] = _implementation
