from types import SimpleNamespace

import pytest

from dimos.plugins.active_search.search import ActiveSearch


def search_views(
    detections, *, exploration_error=None, initial_frame=None, initial_detection=None, directed=True, directed_available=True,
):
    visited = []
    stages = []

    def observe(previous, remaining):
        assert previous is None
        visited.append(("observe", 1))
        return SimpleNamespace(stamp_ns=1)

    def next_view(previous, remaining, on_progress):
        assert remaining() > 0
        stamp = previous.stamp_ns + 1
        visited.append(("explore", stamp))
        if exploration_error is not None:
            raise exploration_error
        # The exploration boundary stops the tracker before returning the view.
        visited.append(("stop", stamp))
        return SimpleNamespace(stamp_ns=stamp)

    def toward_target(previous, detection, remaining):
        assert remaining() > 0
        stamp = previous.stamp_ns + 1
        visited.append(("toward_target", stamp))
        if directed_available:
            return SimpleNamespace(stamp_ns=stamp)
        return None

    def detect(frame, target, remaining):
        visited.append(("detect", frame.stamp_ns))
        status = detections[frame.stamp_ns - 1]
        result = {"status": status}
        if status == "localized":
            result.update(bbox_2d=[100, 200, 300, 400], target_xyz=[2, 3, 0], world_frame="world")
        return result

    def plan(frame, result, remaining):
        visited.append(("plan", frame.stamp_ns))
        return {"status": "preview_ready", "frame": frame.stamp_ns}

    search = ActiveSearch(
        observe,
        detect,
        plan,
    )
    result = search(
        "trash can",
        lambda: 20,
        lambda progress: stages.append(progress["status"]),
        next_view=next_view,
        toward_target=toward_target if directed else None,
        initial_frame=initial_frame,
        initial_detection=initial_detection,
    )
    return result, visited, stages


def test_motion_explores_after_miss_and_resumes_after_rejected_candidate():
    result, visited, stages = search_views(
        ["not_visible", "target_mismatch", "localized"]
    )

    assert result["status"] == "preview_ready"
    assert result["frame"] == 3
    assert visited == [
        ("observe", 1), ("detect", 1),
        ("explore", 2), ("stop", 2), ("detect", 2),
        ("explore", 3), ("stop", 3), ("detect", 3), ("plan", 3),
    ]
    assert stages == [
        "searching", "exploring", "searching",
        "exploring", "searching", "confirmed",
    ]


def test_first_view_hit_plans_without_exploration():
    result, visited, stages = search_views(["localized", "not_visible"])

    assert result["status"] == "preview_ready"
    assert visited == [("observe", 1), ("detect", 1), ("plan", 1)]
    assert "exploring" not in stages


def test_motion_search_continues_after_invalid_detection():
    result, visited, stages = search_views(
        ["not_visible", "invalid_detection", "localized"]
    )

    assert result["status"] == "preview_ready"
    assert result["search"]["attempts"][1]["status"] == "invalid_detection"
    assert visited == [
        ("observe", 1), ("detect", 1),
        ("explore", 2), ("stop", 2), ("detect", 2),
        ("explore", 3), ("stop", 3), ("detect", 3), ("plan", 3),
    ]
    assert stages[-1] == "confirmed"


def test_unconfirmed_candidate_continues_frontier_search():
    result, visited, stages = search_views(
        ["not_visible", "verification_unavailable", "localized"]
    )

    assert result["status"] == "preview_ready"
    assert ("explore", 3) in visited
    assert stages[-1] == "confirmed"


def test_unavailable_exploration_reports_no_safe_frontier():
    from dimos.plugins.active_search.search import ExplorationUnavailable

    result, visited, stages = search_views(
        ["not_visible", "localized"],
        exploration_error=ExplorationUnavailable("no_reachable_frontier"),
    )

    assert result["status"] == "exploration_unavailable"
    assert result["search"]["reason"] == "no_reachable_frontier"
    assert visited[-1] == ("explore", 2)
    assert stages[-1] == "exploration_unavailable"



def test_preconfirmed_initial_miss_does_not_run_d455_detection_twice():
    result, visited, stages = search_views(
        ["not_visible", "localized"],
        initial_frame=SimpleNamespace(stamp_ns=1),
        initial_detection={"status": "not_visible", "timing_ms": {"vlm": 200}},
    )

    assert result["status"] == "preview_ready"
    assert result["search"]["attempts"][0] == {
        "stamp_ns": 1, "status": "not_visible", "timing_ms": {"vlm": 200}
    }
    assert visited == [("explore", 2), ("stop", 2), ("detect", 2), ("plan", 2)]
    assert stages[0] == "exploring"



def test_non_miss_cannot_be_used_as_exploration_seed():
    with pytest.raises(ValueError, match="initial detection"):
        search_views(
            ["localized", "not_visible"],
            initial_frame=SimpleNamespace(stamp_ns=1),
            initial_detection={"status": "localized"},
        )


def test_bad_depth_seed_repeatedly_updates_direction_until_localized():
    result, visited, stages = search_views(
        ["invalid_depth", "invalid_depth", "localized"],
        initial_frame=SimpleNamespace(stamp_ns=1),
        initial_detection={"status": "invalid_depth", "bbox_2d": [100, 200, 300, 400]},
    )
    assert result["status"] == "preview_ready"
    assert visited == [
        ("toward_target", 2), ("detect", 2),
        ("toward_target", 3), ("detect", 3), ("plan", 3),
    ]
    assert stages.count("exploring_toward_target") == 2


@pytest.mark.parametrize("available, followup", [(False, "localized"), (True, "not_visible")])
def test_unavailable_direction_or_lost_target_falls_back_to_frontier(available, followup):
    result, visited, _ = search_views(
        ["not_visible", "invalid_depth", followup, "localized"],
        directed_available=available,
    )
    assert result["status"] == "preview_ready"
    assert ("toward_target", 3) in visited
    assert ("explore", 3 if not available else 4) in visited


@pytest.mark.parametrize("seed_status", ["target_mismatch", "verification_unavailable"])
def test_unmatched_box_uses_frontier_and_never_follows_the_box(seed_status):
    result, visited, stages = search_views(
        [seed_status, "localized"],
        initial_frame=SimpleNamespace(stamp_ns=1),
        initial_detection={"status": seed_status, "bbox_2d": [100, 200, 300, 400]},
    )
    assert result["status"] == "preview_ready"
    assert result["search"]["attempts"][0]["status"] == seed_status
    assert visited == [("explore", 2), ("stop", 2), ("detect", 2), ("plan", 2)]
    assert stages[0] == "exploring"


def test_scan_finishes_when_out_of_views_without_an_exploration_deadline(monkeypatch):
    from dimos.plugins.active_search import search as search_module

    now = [0.]
    monkeypatch.setattr(search_module.time, "monotonic", lambda: now[0])
    first = SimpleNamespace(stamp_ns=100)
    progress = []
    checked = []

    def next_view(previous, budget, report):
        if previous.stamp_ns == 105:
            return None
        now[0] += 8
        assert budget() == 120 - now[0]
        return SimpleNamespace(stamp_ns=previous.stamp_ns + 1)

    def detect(frame, target, budget):
        checked.append(frame.stamp_ns)
        return {"status": "not_visible"}

    def must_not_plan(*args):
        pytest.fail("A miss must not reach the planner")

    search = ActiveSearch(
        None, detect, must_not_plan,
    )
    result = search(
        "bin", lambda: 120 - now[0], progress.append, next_view=next_view,
        initial_frame=first, initial_detection={"status": "not_visible"},
    )
    assert checked == [101, 102, 103, 104, 105]
    assert [attempt["stamp_ns"] for attempt in result["search"]["attempts"]] == [100, *checked]
    assert result["search"]["reason"] == "sweep_complete"
    assert progress[0]["attempts"] == 0
    assert progress[-1]["attempts"] == 5
    assert all("max_attempts" not in item for item in progress)


def test_depth_recovery_can_localize_after_more_than_five_checks():
    first = SimpleNamespace(stamp_ns=1)
    directed_views = []

    def toward_target(previous, detection, remaining):
        view = SimpleNamespace(stamp_ns=previous.stamp_ns + 1)
        directed_views.append(view.stamp_ns)
        return view

    def detect(frame, target, remaining):
        return {"status": "localized" if frame.stamp_ns == 8 else "invalid_depth"}

    def must_not_turn(*args):
        pytest.fail("Matching candidate must remain in directed approach")

    search = ActiveSearch(
        None, detect, lambda frame, detection, remaining: detection,
    )
    result = search(
        "bin", lambda: 120, initial_frame=first,
        initial_detection={"status": "invalid_depth"},
        next_view=must_not_turn, toward_target=toward_target,
    )
    assert directed_views == list(range(2, 9))
    assert result["status"] == "localized"
