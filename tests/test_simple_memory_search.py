import threading
import time
from types import SimpleNamespace

import pytest

from dimos.vln_task.search.memory_exploration import MemoryExploration
from dimos.vln_task.search.search import ActiveSearch

from test_frontier_execution import Mapper, frame


@pytest.mark.parametrize("redetection", ["localized", "not_visible"])
def test_recalled_simple_view_is_navigated_and_redetected_before_confirmation(redetection):
    first, stopped = frame(100, 0.0), frame(102, 0.8)
    first.received_at = stopped.received_at = time.monotonic()
    requests, detections, queries, frontier_selections = [], [], [], []

    def query(target, remaining):
        queries.append(target)
        historical = SimpleNamespace(memory_id="historical", x=0.8, y=0.0, yaw=0.0, similarity=0.9)
        if len(queries) == 1:
            current = SimpleNamespace(memory_id="current", x=0.1, y=0.0, yaw=0.0, similarity=0.95)
            return [current, historical]
        return [historical]

    class Planner:
        def path_is_clear(self, grid, *, origin_xy, resolution, robot_xy, path_xy):
            return True

        def select(self, *args, **kwargs):
            assert detections == [102]
            frontier_selections.append(True)
            return None

    def approach(outcome, remaining, cancel, progress, *, accept_path, **kwargs):
        assert callable(accept_path)
        assert outcome["exploration_leg"]["leg_kind"] == "memory_recall"
        assert outcome["exploration_leg"]["memory_id"] == "historical"
        requests.append(outcome["goal_xy"])
        progress({"status": "tracking"})
        return {"status": "arrived"}

    def detect(view, target, remaining, **kwargs):
        detections.append(view.stamp_ns)
        return {"status": redetection}

    step = MemoryExploration(
        lambda *_args, **_kwargs: stopped, lambda: 101, approach, Mapper(),
        Planner(), threading.Event(), query_memory=query,
        turn=None, settle_s=0,
    )
    search = ActiveSearch(None, detect, lambda _view, result, _remaining: result)
    result = step.run_search(
        search, "bin", lambda: 60, None, first, {"status": "not_visible"}, camera={},
    )

    assert queries == (["bin"] if redetection == "localized" else ["bin", "bin"])
    assert requests == [(0.8, 0.0)]
    assert detections == [102]
    if redetection == "localized":
        assert result["status"] == "localized"
        assert not frontier_selections
    else:
        assert result["status"] == "search_exhausted"
        assert result["search"]["reason"] == "no_reachable_frontier"
        assert frontier_selections == [True]


def test_recall_logs_why_each_candidate_was_skipped(monkeypatch):
    from dimos.vln_task.search import memory_exploration

    seed = frame(100, 0.0)
    seed.received_at = time.monotonic()
    candidates = [
        SimpleNamespace(memory_id="low-score", x=1.0, y=0.0, yaw=0.0, similarity=0.1),
        SimpleNamespace(memory_id="here", x=0.1, y=0.0, yaw=0.0, similarity=0.9),
        SimpleNamespace(memory_id="blocked", x=1.0, y=0.0, yaw=0.0, similarity=0.9),
    ]
    logs = []
    monkeypatch.setattr(memory_exploration, "recording_enabled", lambda: True)
    monkeypatch.setattr(memory_exploration, "event", lambda logger, name, **fields: logs.append((name, fields)))

    class Planner:
        def path_is_clear(self, *args, **kwargs):
            return False

    step = MemoryExploration(
        None, None, None, Mapper(), Planner(), threading.Event(),
        query_memory=lambda *_: candidates, turn=None,
    )
    assert step._recall_view("bin", seed, lambda: 60, None) is None
    decisions = [fields for name, fields in logs if name == "memory_candidate_checked"]
    assert [(item["memory_id"], item["reason"]) for item in decisions] == [
        ("low-score", "below_similarity"),
        ("here", "near_current_pose"),
        ("blocked", "insufficient_clearance"),
    ]
    assert len({item["memory_lookup_id"] for item in decisions}) == 1


def test_recall_without_recorder_skips_log_only_lookup_identity(monkeypatch):
    from dimos.vln_task.search import memory_exploration

    def unexpected(*args, **kwargs):
        pytest.fail("disabled logging prepared recall diagnostics")

    monkeypatch.setattr(memory_exploration, "recording_enabled", lambda: False)
    monkeypatch.setattr(memory_exploration, "uuid4", unexpected)
    monkeypatch.setattr(memory_exploration, "event_context", unexpected)
    lookups = []

    def query(target, remaining):
        lookups.append(target)
        return []

    step = MemoryExploration(
        None, None, None, Mapper(), None, threading.Event(),
        query_memory=query, turn=None,
    )
    assert step._recall_view("bin", frame(100, 0.0), lambda: 60, None) is None
    assert lookups == ["bin"]
