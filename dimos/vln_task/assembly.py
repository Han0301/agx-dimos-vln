"""Own the preview stack and connect the standalone clients."""

import copy
import logging
import time
from contextlib import ExitStack
from functools import partial

from dimos.vln_task.search.verification import verify_candidate
from dimos.plugins.llm.client import LLMClient
from dimos.plugins.path_tracker.messages import decode_result, odom_pose
from dimos.plugins.path_tracker.tracker import Tracker
from dimos.plugins.vlm.client import VLMClient
from dimos.plugins.xnavdp.client import XNavDPClient, stamp_ns

from .agent import TaskAgent
from .observation import Observations
from .preview import NavigationPreview

tracker_logger = logging.getLogger("dimos.plugins.path_tracker")


def create_runner(profile, *, locate_only=False, motion_tracker=None):
    with ExitStack() as resources:
        llm = resources.enter_context(LLMClient(profile["llm"]))
        vlm = resources.enter_context(VLMClient(profile["vlm"]))
        if motion_tracker is not None:
            if locate_only or profile.get("active_search") is None:
                raise ValueError("Motion requires a profile with active search")
            from dimos.plugins.path_tracker.session import TrackerSession

            resources.enter_context(TrackerSession())
        node_name = "vln_motion" if motion_tracker is not None else "vln_preview"
        client = XNavDPClient(profile["xnavdp"], node_name=node_name).start()
        resources.callback(client.stop)
        observations = Observations(
            client.node,
            profile["observation"],
            profile["tracker"]["depth_scale"],
            profile["xnavdp"]["sensor_timeout_s"],
        )
        if not locate_only:
            client.warmup_requests()

        def plan(frame, xy, check, timing_ms):
            goal = client.pose_type()
            goal.header.stamp = copy.deepcopy(frame.header.stamp)
            goal.header.frame_id = frame.odom.header.frame_id
            goal.pose.position.x, goal.pose.position.y = xy
            goal.pose.orientation.w = 1.0
            parts, seconds = client.infer(goal, check=check, timing_ms=timing_ms)
            return parts["result"].data, seconds, len(parts["path"].poses)

        approach = None
        if motion_tracker is not None:
            from dimos.plugins.path_tracker.node import TrackerNode

            from .approach import ContinuousApproach

            adapter = TrackerNode(
                client.node, motion_tracker, subscribe_results=False,
                inputs_ready=observations.motion_inputs_ready,
            )
            tracker = adapter.tracker.start()
            resources.callback(tracker.stop)

        def preview(data):
            started = time.perf_counter()
            inputs = observations.tracking_inputs()
            if inputs is None:
                tracker_logger.info("preview skipped reason=tracking_inputs_stale")
                return {
                    "linear_x": 0.0,
                    "angular_z": 0.0,
                    "reason": "tracking_inputs_stale",
                }
            odom, depth, depth_stamp = inputs
            # One isolated MPC evaluation, with no ROS velocity publisher.
            tracker = Tracker(profile["tracker"], lambda *_: None, clock=lambda: 0.0)
            tracker.update_odom(*odom_pose(odom))
            tracker.update_depth(depth)
            result = tracker.preview(decode_result(data))
            tracker_logger.info(
                "preview completed reason=%s linear_x=%.3f angular_z=%.3f elapsed_ms=%.1f",
                result["reason"],
                result["linear_x"],
                result["angular_z"],
                (time.perf_counter() - started) * 1000,
            )
            return {
                **result,
                "odom_stamp_ns": stamp_ns(odom.header),
                "depth_stamp_ns": depth_stamp,
            }

        verify = partial(verify_candidate, vlm)
        skill = NavigationPreview(
            vlm,
            observations.snapshot,
            None if locate_only else plan,
            preview,
            profile["camera"],
            profile["observation"]["approach_stop_m"],
            verify=verify,
        )
        if motion_tracker is not None:
            approach = ContinuousApproach(
                observations.snapshot, plan, tracker,
                detect=partial(
                    skill.detect, allow_bad_depth=True,
                    allow_verification=False, purpose="target_refresh",
                ),
                max_target_shift_m=profile["active_search"]["max_target_shift_m"],
            )
            resources.callback(approach.close)
        find_object = None
        if profile.get("active_search") is not None and not locate_only:
            from dimos.vln_task.search.search import (
                ActiveSearch,
                viewpoint_changed,
            )

            from .find_object import FindObject

            settings = profile["active_search"]
            if approach is not None:
                from dimos.vln_task.search.exploration import (
                    DirectedExploration, ExplorationCloud, RotationSweep,
                )
                from dimos.vln_task.search.frontier import FrontierPlanner
                from dimos.vln_task.search.occupancy import DepthOccupancyMap

                cloud = ExplorationCloud(client.node, motion_tracker["topics"]["lidar"])
                clearance = motion_tracker["collision"]["robot_radius"]

            def observe(previous, remaining):
                if previous is None:
                    return observations.snapshot(remaining)
                pose, frame_id = odom_pose(previous.odom)

                def accept_odom(msg):
                    current, current_frame = odom_pose(msg)
                    return current_frame == frame_id and viewpoint_changed(
                        pose, current, settings
                    )

                return observations.snapshot(
                    remaining, after_stamp_ns=previous.stamp_ns, accept_odom=accept_odom
                )

            def confirmed(frame, outcome, remaining):
                if approach is not None:
                    return outcome
                return skill.plan_detection(frame, outcome, remaining)

            search = ActiveSearch(
                observe,
                partial(skill.detect, allow_bad_depth=True, purpose="search_det"),
                confirmed,
            )

            def explore(target, remaining, progress, cancel, frame, detection):
                next_view = toward_target = None
                if approach is not None:
                    next_view = RotationSweep(
                        observations.snapshot, observations.latest_rgb_stamp,
                        tracker.turn_to, frame, cancel,
                    )
                    directed = DirectedExploration(
                        observations.snapshot, observations.latest_rgb_stamp, approach,
                        DepthOccupancyMap(profile["camera"], footprint_radius=clearance),
                        FrontierPlanner(clearance_m=clearance),
                        cancel, cloud=cloud,
                    )
                    toward_target = partial(directed.toward_target, camera=profile["camera"])
                return search(
                    target, remaining, progress, next_view=next_view,
                    initial_frame=frame, initial_detection=detection,
                    toward_target=toward_target,
                )

            find_object = FindObject(observations.snapshot, skill.detect, explore, confirmed)
        agent = TaskAgent(
            llm, vlm, profile["task"], navigation_preview=skill,
            find_object=find_object, approach=approach,
        )
        return PreviewRunner(agent, observations, resources.pop_all(), approach)


class PreviewRunner:
    def __init__(self, agent, observations, resources, approach=None):
        self.cancel_motion = None if approach is None else approach.cancel
        self.agent = agent
        self.camera_frame = observations.camera_frame
        self.resources = resources

    def preload_llm(self, timeout):
        return self.agent.llm.preload(timeout=timeout)

    def run(self, goal, image=None, cancel=None):
        return self.run_with_progress(goal, image, cancel)

    def run_with_progress(self, goal, image=None, cancel=None, on_progress=None):
        return self.agent.run(goal, image, cancel, on_progress=on_progress)

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.resources.close()
