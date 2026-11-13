from types import SimpleNamespace

import pytest

from dimos.plugins.spatial_memory._backend import MemorySession


@pytest.mark.parametrize("failure", [None, "startup", "inference"])
def test_prefers_gpu_and_retries_on_cpu(failure):
    calls = []
    io = [SimpleNamespace(name="tensor")]

    def run_gpu(outputs, inputs):
        calls.append("gpu run")
        if failure == "inference":
            raise RuntimeError("device lost")
        return [42]

    def gpu():
        calls.append("gpu init")
        if failure == "startup":
            raise OSError("GPU unavailable")
        return SimpleNamespace(run=run_gpu, get_inputs=lambda: io, get_outputs=lambda: io)

    def cpu():
        calls.append("cpu init")
        return SimpleNamespace(run=lambda *_: [42], get_inputs=lambda: io, get_outputs=lambda: io)

    session = MemorySession(gpu, cpu)
    assert session.get_inputs() == io
    assert session.run(None, {"tensor": 1}) == [42]
    assert session.run(None, {"tensor": 2}) == [42]
    assert session.get_outputs() == io
    assert calls.count("cpu init") == int(failure is not None)
    assert calls.count("gpu run") == {None: 2, "startup": 0, "inference": 1}[failure]
