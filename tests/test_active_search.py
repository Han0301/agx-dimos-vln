from types import SimpleNamespace

import pytest

from dimos.plugins.active_search.search import ActiveSearch


@pytest.mark.parametrize("status", ["not_visible", "target_mismatch", "verification_unavailable"])
def test_passive_search_does_not_plan_without_a_new_localized_view(status):
    first = SimpleNamespace(stamp_ns=1000)

    def observe(previous, remaining):
        if previous is None:
            return first
        assert previous is first
        raise TimeoutError("waiting for a new view")

    def must_not_plan(*args):
        pytest.fail("An unconfirmed view must not reach the planner")

    search = ActiveSearch(
        observe, lambda *_: {"status": status}, must_not_plan,
    )
    result = search("bin", lambda: 20)
    assert result["status"] == "search_exhausted"
    assert result["search"]["reason"] == "no_new_view"
    assert [item["stamp_ns"] for item in result["search"]["attempts"]] == [1000]


def test_view_change_uses_translation_or_wrapped_rotation():
    from math import radians

    from dimos.plugins.active_search.search import viewpoint_changed

    settings = {"min_translation_m": 0.12, "min_rotation_deg": 8}
    first = (0, 0, radians(179))
    assert not viewpoint_changed(first, (0.01, 0.02, radians(-179)), settings)
    assert viewpoint_changed(first, (0.2, 0.0, radians(179)), settings)
    assert viewpoint_changed(first, (0.0, 0.0, radians(-170)), settings)
