"""Dispatch a standalone current-frame VLM result to approach or exploration."""

import logging

from dimos.vln_task.search.search import SCAN_STATUSES, SEARCH_SEED_STATUSES
from dimos.plugins.runtime_log import event, event_context

logger = logging.getLogger('dimos.plugins.vln_task.find_object')


class FindObject:
    def __init__(self, observe, detect, explore, localized):
        self.observe = observe
        self.detect = detect
        self.explore = explore
        self.localized = localized

    def __call__(self, target, remaining, on_progress, cancel, *, stop_m=None):
        if not isinstance(target, str) or not target.strip() or len(target) > 1000:
            raise ValueError("Expected a nonempty object description")
        if on_progress is not None:
            on_progress({"status": "checking_current_view"})
        stop_options = {"stop_m": stop_m} if stop_m is not None else {}
        with event_context(purpose="first_det"):
            frame = self.observe(remaining)
            result = self.detect(frame, target, remaining, allow_bad_depth=True, **stop_options)
        remaining()
        branch = {
            "localized": "localized_goal",
            **dict.fromkeys(SCAN_STATUSES, "rotation_scan"),
            "invalid_depth": "recover_depth",
        }.get(result["status"], "end")
        event(logger, "first_detection_branch", rgb_stamp_ns=frame.stamp_ns,
              observation_id=getattr(frame, "observation_id", None),
              status=result["status"], branch=branch)
        if result["status"] == "localized":
            return self.localized(frame, result, remaining)
        if result["status"] in SEARCH_SEED_STATUSES:
            return self.explore(target, remaining, on_progress, cancel, frame, result)
        return result
