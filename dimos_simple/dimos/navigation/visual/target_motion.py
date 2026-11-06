# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Optional exclusive turn coordinator, reusing the existing navigation RPCs."""

from __future__ import annotations

import threading
import uuid
from typing import Any, Protocol

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.navigation.replanning_a_star.module_spec import ReplanningAStarPlannerSpec
from dimos.spec.utils import Spec


class TargetMotionSpec(Spec, Protocol):
    def begin_target_turn(self) -> str: ...
    def set_target_turn(self, lease: str, goal: PoseStamped) -> bool: ...
    def end_target_turn(self, lease: str) -> bool: ...


class NavDPTargetControlSpec(Spec, Protocol):
    def cancel_goal(self) -> bool: ...
    def get_navdp_state(self) -> str: ...
    def resume_motion(self) -> bool: ...


class TargetMotionConfig(ModuleConfig):
    enabled: bool = False


class TargetMotionCoordinator(Module[TargetMotionConfig]):
    """Does nothing at startup; each turn requires a new explicit lease."""

    default_config = TargetMotionConfig
    planner: ReplanningAStarPlannerSpec

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._lease_lock = threading.Lock()
        self._lease = ""

    def _release_auxiliary_controller(self) -> bool:
        return True

    @rpc
    def begin_target_turn(self) -> str:
        if not self.config.enabled or not self._lease_lock.acquire(blocking=False):
            return ""
        try:
            if (
                not self._release_auxiliary_controller()
                or not self.planner.cancel_goal()
            ):
                self._lease_lock.release()
                return ""
            self._lease = uuid.uuid4().hex
            return self._lease
        except Exception:
            self._lease_lock.release()
            raise

    @rpc
    def set_target_turn(self, lease: str, goal: PoseStamped) -> bool:
        if not lease or lease != self._lease:
            return False
        return self.planner.set_goal(goal)

    @rpc
    def end_target_turn(self, lease: str) -> bool:
        if not lease or lease != self._lease:
            return False
        try:
            return self.planner.cancel_goal()
        finally:
            self._lease = ""
            self._lease_lock.release()

    @rpc
    def stop(self) -> None:
        if self._lease:
            self.end_target_turn(self._lease)
        super().stop()


class NavDPTargetMotionCoordinator(TargetMotionCoordinator):
    """Only composed when NavDP exists, preserving non-NavDP blueprints."""

    navigator: NavDPTargetControlSpec

    def _release_auxiliary_controller(self) -> bool:
        if (
            not self.navigator.cancel_goal()
            or self.navigator.get_navdp_state() != "IDLE"
        ):
            return False
        return self.navigator.resume_motion()
