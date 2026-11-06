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

"""Opt-in coordinate approach. Legacy VLN skills and imagegoal remain unchanged."""

from __future__ import annotations

import json
import math
import time

from dimos.agents.skills.vln_skill import TargetPerceptionHandoff
from dimos.agents.skills.vln_skill import VLNSkillContainer as LegacyVLN
from dimos.models.qwen.bbox import BBox
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.navigation.visual.object_localizer import ObjectLocalizer
from dimos.navigation.xnavdp.pointgoal import PointGoalNavigatorSpec
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class XNavDPVLNSkillContainer(LegacyVLN):
    """Keep inherited skills and RPC aliases with a distinct concrete class name."""

    point_navigator: PointGoalNavigatorSpec
    _pair_vlm_observations = True

    def _has_navdp(self) -> bool:
        """Use the injected XNavDP navigator instead of the legacy RPC alias."""
        try:
            return self.point_navigator.get_navdp_state() != "UNINITIALIZED"
        except Exception:
            logger.exception("XNavDP navigator is unavailable")
            # Keep the request on the XNavDP path: do not silently drive via A*.
            return True

    def _recover_target_with_x3(self, description: str) -> None:
        """Keep the target session and recover fresh D455 RGB-D before proceeding."""
        client = self._perception
        if client is None:
            self._target_handoff("perception_unavailable", "target_client_missing")
        logger.info(
            "[X3-Recovery] Pausing approach/search; retaining task_id=%s",
            client.task_id,
        )
        result = json.loads(self._face_target(client.task_id, internal=True))
        if result.get("status") != "front_reacquired":
            self._target_handoff(
                "needs_selection"
                if result.get("status") == "ambiguous"
                else "needs_reacquire",
                "x3_recovery_"
                + str(result.get("reason", result.get("status", "failed"))),
            )
        deadline = time.monotonic() + 1.5
        while time.monotonic() < deadline and not self._search_stop.is_set():
            box, _ = super()._fast_target_check(description)
            snapshot = self._target_snapshot
            if box is not None and snapshot is not None and snapshot.depth is not None:
                if self._estimate_observed_target(box) is not None:
                    self._resume_motion_after_vlm_check()
                    logger.info(
                        "[X3-Recovery] Fresh paired D455 target reacquired; resuming same task"
                    )
                    return
            time.sleep(0.05)
        self._target_handoff(
            "perception_unavailable", "reacquired_d455_depth_or_pose_missing"
        )

    def _fast_target_check(
        self, description: str
    ) -> tuple[BBox | None, PoseStamped | None]:
        if not self._perception_assist():
            return super()._fast_target_check(description)
        self._target_snapshot = None
        try:
            box, pose = super()._fast_target_check(description)
        except TargetPerceptionHandoff as event:
            if event.result.get("status") not in (
                "needs_reacquire",
                "perception_unavailable",
            ):
                raise
            self._recover_target_with_x3(description)
            return super()._fast_target_check(description)
        known_target = (
            self._perception is not None and self._perception.target is not None
        )
        off_center = (
            self._perception_approaching
            and self.config.perception.get("center_target", False)
            and self._target_snapshot is not None
            and not self._front_is_centered(
                self._target_snapshot.detection, tolerance=0.22
            )
        )
        if off_center:
            logger.info(
                "[Target-Center] Target leaving central region; pausing to correct heading"
            )
        # Searching must not initiate a turn merely because depth pairing is pending.
        # During approach, pairing remains mandatory before any forward movement.
        needs_recovery = self._perception_approaching or (known_target and box is None)
        if needs_recovery and (
            off_center
            or box is None
            or self._target_snapshot is None
            or self._target_snapshot.depth is None
        ):
            self._recover_target_with_x3(description)
            return super()._fast_target_check(description)
        return box, pose

    def _check_object_in_view(
        self, object_description: str, *, resume_search: bool = False
    ) -> tuple[BBox | None, PoseStamped | None]:
        # An assisted task must use the same confirmed five-camera identity.
        # A slow VLM box alone cannot refresh that identity or its RGB-D snapshot.
        if self._perception_assist():
            if not self._ensure_perception_target(object_description):
                self._target_handoff(
                    "perception_unavailable", "target_session_unavailable"
                )
            return self._fast_target_check(object_description)
        return super()._check_object_in_view(
            object_description, resume_search=resume_search
        )

    def _hybrid_frontier_loop(self) -> None:
        """Advance point-goal frontiers on arrival, travel, or lack of progress."""
        stop = self._hybrid_stop_event
        goal = None
        last_position = None
        selected_at = 0.0
        last_wait_log = 0.0
        while not stop.wait(0.5):
            odom = self._latest_odom
            if odom is None:
                if time.monotonic() - last_wait_log >= 5:
                    logger.warning("[XNavDP-Wait] Frontier blocked by missing odometry")
                    last_wait_log = time.monotonic()
                continue
            position = (odom.position.x, odom.position.y)
            arrived = goal is not None and math.dist(position, (goal.x, goal.y)) < 0.4
            stalled = goal is not None and time.monotonic() - selected_at > 20
            if (
                goal is not None
                and not arrived
                and not stalled
                and last_position is not None
            ):
                if (
                    math.dist(position, last_position)
                    < self.config.frontier_replan_distance_m
                ):
                    continue
            if arrived or stalled:
                try:
                    self.get_rpc_calls("WavefrontFrontierExplorer.mark_explored_goal")(
                        goal
                    )
                except Exception as exc:
                    logger.warning("XNavDP frontier completion update failed: %s", exc)
            try:
                goal = self.get_rpc_calls(
                    "WavefrontFrontierExplorer.get_best_frontier_goal"
                )()
                if goal is None:
                    if time.monotonic() - last_wait_log >= 5:
                        logger.warning(
                            "[XNavDP-Wait] No frontier returned; waiting for costmap/frontier availability"
                        )
                        last_wait_log = time.monotonic()
                    continue
                destination = PoseStamped(
                    position=Vector3(goal.x, goal.y, 0),
                    orientation=Quaternion.from_euler(Vector3(0, 0, 0)),
                    frame_id=odom.frame_id,
                )
                path = self.get_rpc_calls(
                    "ReplanningAStarPlanner.compute_path_to_goal"
                )(destination)
                if path is None or not path.poses:
                    self.get_rpc_calls("WavefrontFrontierExplorer.mark_explored_goal")(
                        goal
                    )
                    goal = None
                    continue
                if stop.is_set():
                    return
                waypoints = [[p.position.x, p.position.y] for p in path.poses]
                if not self.point_navigator.set_path_guidance(waypoints):
                    goal = None
                    continue
                selected_at = time.monotonic()
                last_position = position
                logger.info(
                    "XNavDP frontier path: %d points, goal=(%.2f,%.2f)",
                    len(waypoints),
                    goal.x,
                    goal.y,
                )
            except Exception as exc:
                goal = None
                logger.warning("XNavDP frontier planning failed: %s", exc)

    def _approach_via_navdp(
        self, object_description: str, bbox: BBox | None = None
    ) -> str:
        # Stop the A* local controller before XNavDP takes velocity ownership.
        try:
            self.get_rpc_calls("ReplanningAStarPlanner.cancel_goal")()
        except Exception:
            logger.exception("Unable to stop A* before XNavDP approach")
            return "Unable to stop A*; XNavDP approach was not started."
        if not self._perception_assist():
            return self._approach_via_d455_pointgoal(object_description, bbox)
        self._stop_exploration()
        try:
            self.point_navigator.resume_motion()
            last_update = 0.0
            last_stamp = -1.0
            while not self._search_stop.is_set():
                box, _ = self._fast_target_check(object_description)
                snapshot = self._target_snapshot
                if box is None or snapshot is None or snapshot.depth is None:
                    self._target_handoff(
                        "perception_unavailable", "pointgoal_requires_paired_d455_depth"
                    )
                estimate = self._estimate_observed_target(box)
                odom = self._latest_odom
                if estimate is None or odom is None:
                    self._target_handoff(
                        "perception_unavailable", "pointgoal_depth_or_pose_invalid"
                    )
                distance = math.hypot(
                    estimate.x - odom.position.x, estimate.y - odom.position.y
                )
                if distance <= self.config.approach_stop_distance + 0.2:
                    return "Successfully approached the confirmed target via XNavDP point-goal."
                if (
                    time.monotonic() - last_update >= 0.3
                    and snapshot.detection.stamp != last_stamp
                ):
                    # Full standoff goal; navigator supplies the local coordinate each frame.
                    x, y, yaw = ObjectLocalizer.compute_approach_goal(
                        odom,
                        estimate,
                        self.config.approach_stop_distance,
                        max_step_distance=100.0,
                    )
                    goal = PoseStamped(
                        position=Vector3(x, y, odom.position.z),
                        orientation=Quaternion.from_euler(Vector3(0, 0, yaw)),
                        frame_id=odom.frame_id,
                    )
                    if not self.point_navigator.set_point_goal(goal, valid_for_s=1.0):
                        return "XNavDP rejected the coordinate goal."
                    last_update = time.monotonic()
                    last_stamp = snapshot.detection.stamp
                time.sleep(0.05)
            return "Search was cancelled."
        finally:
            self.point_navigator.cancel_goal()

    def _approach_via_d455_pointgoal(
        self, object_description: str, bbox: BBox | None
    ) -> str:
        """Approach the confirmed target using one current D455 RGB-D observation."""
        observation = self._last_vlm_observation
        if bbox is None or observation is None or tuple(bbox) != tuple(observation[0]):
            return "XNavDP approach requires the RGB-D observation associated with the detected target."
        _, image, depth, capture_odom = observation
        odom = self._latest_odom
        if image is None or depth is None or capture_odom is None or odom is None or self._localizer is None:
            return "XNavDP D455 approach unavailable without a paired RGB-D observation and odometry."
        image_h, image_w = image.data.shape[:2]
        target_bbox = bbox
        if self.config.vlm_backend == "qwen3_local":
            target_bbox = (
                bbox[0] / 1000.0 * image_w,
                bbox[1] / 1000.0 * image_h,
                bbox[2] / 1000.0 * image_w,
                bbox[3] / 1000.0 * image_h,
            )

        estimate = self._localizer.estimate_from_bbox_depth(
            target_bbox, depth, capture_odom,
            min_confidence=self.config.approach_min_depth_confidence,
        )
        if estimate is None:
            return "XNavDP D455 approach could not localize the target in depth."
        logger.info("[XNavDP-Target] bbox=%s target=(%.2f, %.2f) depth=%.2f", target_bbox, estimate.x, estimate.y, estimate.depth_m)

        self._stop_exploration()
        self.point_navigator.resume_motion()
        try:
            x, y, yaw = ObjectLocalizer.compute_approach_goal(
                self._latest_odom,
                estimate,
                self.config.approach_stop_distance,
                max_step_distance=100.0,
            )
            goal = PoseStamped(
                position=Vector3(x, y, self._latest_odom.position.z),
                orientation=Quaternion.from_euler(Vector3(0, 0, yaw)),
                frame_id=self._latest_odom.frame_id,
            )
            logger.info("[XNavDP-Target] sending goal=(%.2f, %.2f) frame=%s", x, y, goal.frame_id)
            if not self.point_navigator.set_point_goal(goal, valid_for_s=1.0):
                return "XNavDP rejected the coordinate goal."

            while not self._search_stop.is_set():
                odom = self._latest_odom
                if odom is None:
                    time.sleep(0.05)
                    continue
                distance = math.hypot(
                    estimate.x - odom.position.x,
                    estimate.y - odom.position.y,
                )
                if distance <= self.config.approach_stop_distance + 0.2:
                    return "Successfully approached the confirmed target via XNavDP point-goal."
                time.sleep(0.05)
            return "Search was cancelled."
        finally:
            self.point_navigator.cancel_goal()


# Keep the blueprint import API compatible.
VLNSkillContainer = XNavDPVLNSkillContainer
