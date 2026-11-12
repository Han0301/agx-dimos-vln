import threading
import time

import pytest

from dimos.vln_task.search.frontier import FrontierGoal
from dimos.vln_task.search.memory_exploration import MemoryExploration

from test_frontier_execution import Mapper, frame


class Planner:
    explored_radius_m = 0.1

    def __init__(self, *goals):
        self.goals = goals
        self.explored = []

    def mark_explored(self, goal):
        if goal.final:
            self.explored.append(goal.frontier_xy)

    def select(self, grid, *, origin_xy, resolution, robot_xy, allow_frontier):
        return next((goal for goal in self.goals if allow_frontier(goal.frontier_xy)), None)


def observation(stamp, x):
    view = frame(stamp, x)
    view.received_at = time.monotonic()
    return view


@pytest.mark.parametrize("status", ["not_visible", "invalid_detection"])
def test_first_frontier_reuses_seed_and_completes_after_stopped_detection(status):
    seed, stopped = observation(100, 0.0), observation(102, 0.8)
    goal = FrontierGoal((0.8, 0.0), (1.0, 0.0), (), True)
    progress = []
    calls = []

    def approach(outcome, remaining, cancel, report, *, accept_path, motion_guard, initial_observation):
        assert initial_observation is seed
        calls.append(outcome["goal_xy"])
        assert outcome["observation"]["stamp_ns"] == 100
        assert callable(accept_path)
        report({"status": "tracking"})
        return {"status": "arrived"}

    def observe(remaining, *, after_stamp_ns, max_age_s, received_after):
        assert calls == [goal.xy]
        assert after_stamp_ns == 101
        return stopped

    step = MemoryExploration(
        observe, lambda: 101, approach, Mapper(), Planner(goal),
        threading.Event(), turn=None, settle_s=0,
    )
    step.record_detection(seed, {"status": "not_visible"})
    assert step(seed, lambda: 10, progress.append) is stopped
    assert step.planner.explored == []

    step.record_detection(stopped, {"status": status})
    assert step.planner.explored == [goal.frontier_xy]
    assert len(step.observed_poses) == 2
    assert [update["status"] for update in progress] == [
        "selecting_frontier", "planning_frontier", "tracking_frontier",
        "waiting_exploration_view", "checking_exploration_view",
    ]


def test_timed_out_frontier_is_cooled_and_alternate_is_selected(monkeypatch):
    import pytest

    from dimos.vln_task.search import memory_exploration

    def unexpected(*args, **kwargs):
        pytest.fail("disabled logging prepared frontier diagnostics")

    monkeypatch.setattr(memory_exploration, "recording_enabled", lambda: False)
    monkeypatch.setattr(memory_exploration, "event_context", unexpected)
    monkeypatch.setattr(memory_exploration, "diagnostic_artifact", unexpected)

    first = FrontierGoal((0.6, 0.0), (1.0, 0.0), (), False)
    second = FrontierGoal((0.8, 0.0), (2.0, 0.0), (), False)
    calls = []
    frames = iter([observation(101, 0.0), observation(103, 0.8)])

    def approach(outcome, remaining, cancel, report, **kwargs):
        calls.append(outcome["goal_xy"])
        if len(calls) == 1:
            raise TimeoutError()
        return {"status": "arrived"}

    step = MemoryExploration(
        lambda *args, **kwargs: next(frames), lambda: 102, approach, Mapper(),
        Planner(first, second), threading.Event(), turn=None, settle_s=0,
    )
    stopped = step(observation(100, 0.0), lambda: 10, None)
    step.record_detection(stopped, {"status": "not_visible"})

    assert calls == [first.xy, second.xy]
    assert step.cooling[first.frontier_xy] > time.monotonic()
    assert step.planner.explored == []


def test_search_records_the_existing_seed_and_uses_frontier_terminal_reason():
    from dimos.vln_task.search.search import ActiveSearch

    seed, stopped = observation(100, 0.0), observation(102, 0.8)
    views = iter([stopped, None])
    detections = []
    recorded = []

    def detect(view, target, remaining, **kwargs):
        detections.append(view.stamp_ns)
        return {"status": "not_visible"}

    search = ActiveSearch(None, detect, None)
    result = search(
        "bin", lambda: 10, initial_frame=seed,
        initial_detection={"status": "not_visible"},
        next_view=lambda *args: next(views),
        on_detection=lambda view, detection: recorded.append(view.stamp_ns),
        exhausted_reason="no_reachable_frontier",
    )

    assert detections == [102]
    assert recorded == [100, 102]
    assert result["search"]["reason"] == "no_reachable_frontier"


def test_frontier_logs_progress_and_stopped_observation_timeout(monkeypatch):
    import pytest

    from dimos.vln_task.search import memory_exploration

    logged, updates = [], []
    monkeypatch.setattr(memory_exploration, "event", lambda logger, name, **fields: logged.append((name, fields)))
    monkeypatch.setattr(memory_exploration, "recording_enabled", lambda: True)
    monkeypatch.setattr(memory_exploration, "diagnostic_artifact", lambda *args, **kwargs: None)

    def approach(outcome, remaining, cancel, report, **kwargs):
        for status in ("tracking", "replanning", "tracking", "arrived"):
            report({"status": status})
        return {"status": "arrived"}

    def observe(*args, **kwargs):
        raise TimeoutError("no RGB frame")

    step = MemoryExploration(
        observe, lambda: 101, approach, Mapper(),
        Planner(FrontierGoal((0.8, 0.0), (1.0, 0.0), (), True)),
        threading.Event(), turn=None, settle_s=0,
    )
    with pytest.raises(TimeoutError, match="no RGB frame"):
        step(observation(100, 0.0), lambda: 10, updates.append)

    stages = [update["status"] for update in updates]
    assert stages == [
        "selecting_frontier", "planning_frontier", "tracking_frontier",
        "replanning_frontier", "tracking_frontier", "waiting_exploration_view",
    ]
    assert [fields["stage"] for name, fields in logged if name == "exploration_stage"] == stages
    finish, = (fields for name, fields in logged if name == "exploration_leg_finished")
    assert (finish["leg_id"], finish["reason"]) == (1, "no_new_view")


def test_cancelled_memory_search_records_terminal_reason(monkeypatch):
    import pytest

    from dimos.plugins._agent_common import TaskCancelled
    from dimos.vln_task.search import memory_exploration

    logged = []
    monkeypatch.setattr(memory_exploration, "event", lambda logger, name, **fields: logged.append((name, fields)))
    monkeypatch.setattr(memory_exploration, "recording_enabled", lambda: True)

    def cancelled_search(*args, **kwargs):
        raise TaskCancelled("operator stopped")

    step = MemoryExploration(
        None, None, None, Mapper(), Planner(), threading.Event(),
        turn=None,
    )
    with pytest.raises(TaskCancelled):
        step.run_search(
            cancelled_search, "bin", lambda: 10, None,
            observation(100, 0.0), {"status": "not_visible"}, camera={},
        )

    name, fields = logged[-1]
    assert name == "search_finished"
    assert fields["outcome"] == "cancelled"
    assert fields["reason"] == "task_cancelled"


def test_no_frontier_retries_memory_during_query_interval_without_turning(monkeypatch):
    seed, recalled = observation(100, 0.0), observation(102, 0.8)
    fallback = observation(104, 1.2)
    queries = []

    def unexpected_turn(*args):
        pytest.fail("Frontier exhaustion should not turn the robot in place")

    step = MemoryExploration(
        None, None, None, Mapper(), Planner(), threading.Event(),
        turn=unexpected_turn, query_memory=lambda *_: None,
    )

    def recall(target, previous, budget, progress):
        queries.append(previous.stamp_ns)
        return recalled if len(queries) == 1 else fallback

    monkeypatch.setattr(step, "_recall_view", recall)

    def search(target, budget, progress, *, next_view, **kwargs):
        assert next_view(seed, budget, progress) is recalled
        assert next_view(recalled, budget, progress) is fallback

    step.run_search(search, "bin", lambda: 10, None, seed, {"status": "not_visible"}, camera={})
    assert queries == [100, 102]


def test_frontier_motion_budget_is_fifteen_seconds_and_respects_task_budget(monkeypatch):
    from dimos.vln_task.search import memory_exploration

    monkeypatch.setattr(memory_exploration.time, "monotonic", lambda: 100.0)
    budgets = []
    task_times = iter([120.0, 6.0])

    def approach(outcome, remaining, cancel, report, **kwargs):
        budgets.extend((remaining(), remaining()))
        return {"status": "arrived"}

    step = MemoryExploration(
        None, None, approach, Mapper(), Planner(), threading.Event(),
        turn=None,
    )
    seed = observation(100, 0.0)
    grid, _ = step._map(seed)
    goal = FrontierGoal((0.8, 0.0), (1.0, 0.0), (), True)
    step._move_to_view(seed, grid, goal.xy, {"leg_id": 1}, lambda: next(task_times), lambda *_: None, {})

    assert budgets == [15.0, 6.0]
