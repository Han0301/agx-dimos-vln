import json
import threading
import time
from types import SimpleNamespace

import pytest

from dimos.vln_task.search.memory_exploration import MemoryExploration
from dimos.vln_task.search.search import ActiveSearch
from dimos.plugins.runtime_log import capture, task_context

from test_frontier_execution import Mapper, frame


@pytest.mark.parametrize("distance", [0.1, 0.8])
@pytest.mark.parametrize("redetection", ["localized", "not_visible"])
def test_recalled_simple_view_is_navigated_and_redetected_before_confirmation(redetection, distance, tmp_path):
    first, stopped = frame(100, 0.0), frame(102, distance)
    first.received_at = stopped.received_at = time.monotonic()
    first.observation_id, stopped.observation_id = "seed-observation", "recalled-observation"
    requests, detections, queries, frontier_selections = [], [], [], []

    def query(target, remaining):
        queries.append(target)
        return [SimpleNamespace(
            memory_id="historical", x=distance, y=0.0, yaw=0.7, similarity=0.2376,
        )]

    turns = []

    class Planner:
        def path_is_clear(self, grid, *, origin_xy, resolution, robot_xy, path_xy):
            pytest.fail("memory recall checked straight-line clearance")

        def select(self, *args, **kwargs):
            assert detections == [102]
            frontier_selections.append(True)
            return None

    def approach(outcome, remaining, cancel, progress, *, accept_path=None, **kwargs):
        assert accept_path is None
        assert outcome["exploration_leg"]["leg_kind"] == "memory_recall"
        assert outcome["exploration_leg"]["memory_id"] == "historical"
        requests.append(outcome["goal_xy"])
        progress({"status": "tracking"})
        return {"status": "arrived"}

    def detect(view, target, remaining, **kwargs):
        detections.append(view.stamp_ns)
        assert target == "bin with appearance from uploaded reference"
        return {"status": redetection}

    step = MemoryExploration(
        lambda *_args, **_kwargs: stopped, lambda: 101, approach, Mapper(),
        Planner(), threading.Event(), query_memory=query,
        turn=lambda yaw, *_: turns.append(yaw), settle_s=0,
    )
    search = ActiveSearch(None, detect, lambda _view, result, _remaining: result)
    with capture("web_input", base=tmp_path) as run, task_context("recall-task"):
        result = step.run_search(
            search, "bin with appearance from uploaded reference", lambda: 60, None, first,
            {"status": "not_visible"}, camera={}, memory_target="bin",
        )

    assert queries == (["bin"] if redetection == "localized" else ["bin", "bin"])
    assert requests == [(distance, 0.0)]
    assert turns == [0.7]
    assert detections == [102]
    if redetection == "localized":
        assert result["status"] == "localized"
        assert not frontier_selections
    else:
        assert result["status"] == "search_exhausted"
        assert result["search"]["reason"] == "no_reachable_frontier"
        assert frontier_selections == [True]

    events = [json.loads(line) for line in (run / "modules/active_search.jsonl").read_text().splitlines()]
    selected = next(item for item in events if item.get("event") == "memory_goal_selected")
    revisited = next(item for item in events if item.get("event") == "memory_recall_result")
    assert revisited["task_id"] == "recall-task"
    assert revisited["memory_lookup_id"] == selected["memory_lookup_id"]
    assert revisited["leg_id"] == selected["leg_id"]
    assert revisited["memory_id"] == "historical"
    assert revisited["observation_id"] == "recalled-observation"
    assert revisited["detection_status"] == redetection
    assert revisited["outcome"] == ("localized" if redetection == "localized" else "continue_search")


def test_recall_logs_why_each_candidate_was_skipped(monkeypatch):
    from dimos.vln_task.search import memory_exploration

    seed = frame(100, 0.0)
    seed.received_at = time.monotonic()
    candidates = [
        SimpleNamespace(memory_id="low-score", x=1.0, y=0.0, yaw=0.0, similarity=0.199),
        SimpleNamespace(memory_id="near", x=0.3, y=0.0, yaw=0.0, similarity=0.9),
        SimpleNamespace(memory_id="blocked", x=1.0, y=0.0, yaw=0.0, similarity=0.20),
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
    step.recalled_ids.update({"near", "blocked"})
    assert step._recall_view("bin", seed, lambda: 60, None) is None
    decisions = [fields for name, fields in logs if name == "memory_candidate_checked"]
    assert [(item["memory_id"], item["reason"]) for item in decisions] == [
        ("low-score", "below_similarity"),
        ("near", "already_recalled"),
        ("blocked", "already_recalled"),
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
