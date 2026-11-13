"""TensorRT execution for the batch-one, fused CLIP memory model."""

import ctypes
import hashlib
import importlib
import os
from pathlib import Path
import shutil
import subprocess
import sys
import sysconfig
import tempfile
from types import SimpleNamespace

import numpy as np


def _import_tensorrt():
    try:
        return importlib.import_module("tensorrt")
    except ModuleNotFoundError as exc:
        if exc.name != "tensorrt":
            raise
    # Jetson's apt-installed bindings are outside the isolated Simple venv.
    system_packages = f"/usr/lib/python{sysconfig.get_python_version()}/dist-packages"
    sys.path.append(system_packages)
    try:
        return importlib.import_module("tensorrt")
    finally:
        sys.path.remove(system_packages)


def _engine_path(model, version):
    stat = model.stat()
    fingerprint = (model.resolve(), stat.st_size, stat.st_mtime_ns, version)
    key = hashlib.sha256(repr(fingerprint).encode()).hexdigest()[:24]
    cache = Path(os.environ.get("XDG_CACHE_HOME") or Path.home() / ".cache")
    return cache / "dimos-mininum" / "spatial-memory" / f"clip-fp32-{key}.plan"


def _build_engine(model, engine):
    trtexec = shutil.which("trtexec")
    if trtexec is None:
        raise FileNotFoundError("trtexec is required to build the CLIP TensorRT engine")
    engine.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(dir=engine.parent, prefix="build-") as directory:
        temporary = Path(directory) / engine.name
        log = Path(directory) / "build.log"
        with log.open("wb") as output:
            build = subprocess.run([
                trtexec, f"--onnx={model}", f"--saveEngine={temporary}",
                "--minShapes=input_ids:1x1,attention_mask:1x1,pixel_values:1x3x224x224",
                "--optShapes=input_ids:1x4,attention_mask:1x4,pixel_values:1x3x224x224",
                "--maxShapes=input_ids:1x77,attention_mask:1x77,pixel_values:1x3x224x224",
                "--noTF32", "--skipInference",
            ], stdout=output, stderr=subprocess.STDOUT, timeout=40, check=False)
        if build.returncode:
            with log.open("rb") as output:
                output.seek(max(0, log.stat().st_size - 2048))
                detail = output.read().decode(errors="replace")
            raise RuntimeError(f"TensorRT CLIP build failed: {detail}")
        temporary.replace(engine)


def _check_cuda(status):
    if status:
        raise RuntimeError(f"CUDA runtime error {status}")


class TensorRTSession:
    def __init__(self, model):
        self._buffers = {}
        self._cuda = ctypes.CDLL("libcudart.so")
        device_count = ctypes.c_int()
        _check_cuda(self._cuda.cudaGetDeviceCount(ctypes.byref(device_count)))
        if device_count.value < 1:
            raise RuntimeError("No CUDA device available")
        trt = _import_tensorrt()
        self._logger = trt.Logger(trt.Logger.ERROR)
        self._runtime = trt.Runtime(self._logger)
        path = _engine_path(model, trt.__version__)
        if not path.is_file():
            _build_engine(model, path)
        self._engine = self._runtime.deserialize_cuda_engine(path.read_bytes())
        if self._engine is None:
            raise RuntimeError(f"Could not load TensorRT engine {path}")
        self._context = self._engine.create_execution_context()
        if self._context is None:
            raise RuntimeError("Could not create TensorRT execution context")
        self._inputs, self._outputs = [], []
        for index in range(self._engine.num_io_tensors):
            name = self._engine.get_tensor_name(index)
            is_input = self._engine.get_tensor_mode(name) == trt.TensorIOMode.INPUT
            tensor = SimpleNamespace(name=name, dtype=trt.nptype(self._engine.get_tensor_dtype(name)))
            (self._inputs if is_input else self._outputs).append(tensor)

    def get_inputs(self):
        return self._inputs

    def get_outputs(self):
        return self._outputs

    def _bind(self, name, size):
        pointer, capacity = self._buffers.get(name, (None, 0))
        if capacity < size:
            if pointer is not None:
                _check_cuda(self._cuda.cudaFree(pointer))
                del self._buffers[name]
            pointer = ctypes.c_void_p()
            _check_cuda(self._cuda.cudaMalloc(ctypes.byref(pointer), ctypes.c_size_t(size)))
            self._buffers[name] = pointer, size
            if not self._context.set_tensor_address(name, pointer.value):
                raise RuntimeError(f"Could not bind TensorRT tensor {name}")
        return pointer

    def run(self, outputs, inputs):
        for tensor in self._inputs:
            name = tensor.name
            value = np.ascontiguousarray(inputs[name], dtype=tensor.dtype)
            if not self._context.set_input_shape(name, value.shape):
                raise RuntimeError(f"Unsupported TensorRT input shape for {name}: {value.shape}")
            device = self._bind(name, value.nbytes)
            _check_cuda(self._cuda.cudaMemcpy(device, ctypes.c_void_p(value.ctypes.data),
                                             ctypes.c_size_t(value.nbytes), 1))
        result = {}
        for tensor in self._outputs:
            value = np.empty(tuple(self._context.get_tensor_shape(tensor.name)), dtype=tensor.dtype)
            self._bind(tensor.name, value.nbytes)
            result[tensor.name] = value
        if not self._context.execute_async_v3(0):
            raise RuntimeError("TensorRT CLIP inference failed")
        for name, value in result.items():
            device, _ = self._buffers[name]
            _check_cuda(self._cuda.cudaMemcpy(ctypes.c_void_p(value.ctypes.data), device,
                                             ctypes.c_size_t(value.nbytes), 2))
        return list(result.values()) if outputs is None else [result[name] for name in outputs]

    def close(self):
        for pointer, _ in self._buffers.values():
            self._cuda.cudaFree(pointer)
        self._buffers.clear()
        self._context = self._engine = self._runtime = None
