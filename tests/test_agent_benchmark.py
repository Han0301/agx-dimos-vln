def test_summary_uses_successful_latencies_and_reports_failures():
    try:
        from dimos.plugins.vln_task.benchmark import summarize
    except ModuleNotFoundError:
        __import__("pytest").fail("Agent benchmark is missing")
    result = summarize(
        [
            {"ok": True, "elapsed_ms": 100, "semantic_ok": True},
            {"ok": True, "elapsed_ms": 300, "semantic_ok": False},
            {"ok": False, "elapsed_ms": 2000, "semantic_ok": False},
        ]
    )
    assert result["success"] == 2 and result["attempts"] == 3
    assert result["mean_ms"] == 200 and result["p50_ms"] == 100
    assert result["p95_ms"] == 300 and result["max_ms"] == 300
    assert result["semantic_pass"] == 1


def test_legacy_benchmark_propagates_failure_exit_code():
    import subprocess
    import sys
    from pathlib import Path

    script = """
import runpy
import sys
from types import ModuleType
benchmark = ModuleType("dimos.vln_task.benchmark")
benchmark.main = lambda: 7
sys.modules[benchmark.__name__] = benchmark
runpy.run_module("dimos.plugins.vln_task.benchmark", run_name="__main__")
"""
    result = subprocess.run(
        [sys.executable, "-c", script],
        cwd=Path(__file__).resolve().parents[1],
        capture_output=True, text=True, timeout=10,
    )
    assert result.returncode == 7, result.stderr
