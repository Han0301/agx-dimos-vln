from types import SimpleNamespace

import pytest

from dimos.plugins.active_search.config import load_config
from dimos.plugins.active_search.search import ActiveSearch


@pytest.mark.parametrize("expected_frame", [1, 2, 3])
def test_search_plans_at_first_localized_view(expected_frame):
    frames = iter(range(1, expected_frame + 1))

    def observe(previous, remaining):
        return SimpleNamespace(stamp_ns=next(frames))

    def detect(frame, target, remaining):
        if frame.stamp_ns < expected_frame:
            return {"status": "not_visible"}
        return {"status": "localized"}

    def plan(frame, detection, remaining):
        return {"status": "preview_ready", "frame": frame.stamp_ns}

    result = ActiveSearch(observe, detect, plan)("bin", lambda: 10)
    assert result["status"] == "preview_ready"
    assert result["frame"] == expected_frame
    assert len(result["search"]["attempts"]) == expected_frame


@pytest.mark.parametrize("mode", ["single", "multi_view"])
def test_old_verification_mode_is_ignored(tmp_path, mode):
    import yaml

    path = tmp_path / "active_search.yaml"
    settings = load_config()
    path.write_text(yaml.safe_dump({**settings, "verification_mode": mode, "max_attempts": 5}))
    assert load_config(path) == settings


def test_diagnostics_accept_plan_callbacks_without_a_status_field():
    search = ActiveSearch(
        lambda *_: SimpleNamespace(stamp_ns=1),
        lambda *_: {"status": "localized"},
        lambda *_: {"path_points": 4},
    )
    assert search("bin", lambda: 10)["path_points"] == 4
