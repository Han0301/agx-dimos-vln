"""Own the preview stack and connect the standalone clients."""

import copy
import logging
import math
import threading
import time
from concurrent.futures import ThreadPoolExecutor, TimeoutError as FutureTimeout
from contextlib import ExitStack
from contextvars import copy_context
from functools import partial
from uuid import uuid4

from httpx import HTTPError

from dimos.plugins.llm.client import LLMClient
from dimos.plugins.path_tracker.messages import decode_result, odom_pose
from dimos.plugins.path_tracker.tracker import Tracker
from dimos.plugins.runtime_log import event, event_context, recording_enabled
from dimos.plugins.vlm.client import VLMClient
from dimos.plugins.xnavdp.client import XNavDPClient, stamp_ns

from .agent import TaskAgent
from .observation import Observations
from .preview import NavigationPreview

tracker_logger = logging.getLogger("dimos.plugins.path_tracker")
logger = logging.getLogger(__name__)


def create_runner(profile, *, locate_only=False, motion_tracker=None, exploration_memory=False):
    if exploration_memory and (motion_tracker is None or locate_only or profile.get("active_search") is None):
        raise ValueError("Exploration memory requires a motion profile with active search")
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
            lidar_source = motion_tracker.get("costmap_lidar")
            if lidar_source is not None:
                from .costmap import LidarCostmap

                LidarCostmap(
                    client.node, adapter.tracker, lidar_source, motion_tracker["topics"]["costmap"],
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

        skill = NavigationPreview(
            vlm,
            observations.snapshot,
            None if locate_only else plan,
            preview,
            profile["camera"],
            profile["observation"]["approach_stop_m"],
        )
        if motion_tracker is not None:
            approach = ContinuousApproach(
                observations.snapshot, plan, tracker,
                detect=partial(
                    skill.detect, allow_bad_depth=True,
                    purpose="target_refresh",
                ),
                last_request_stamp=lambda: client.last_request_stamp,
                arrival_condition=tracker.cv if exploration_memory else None,
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

                lidar_topic = motion_tracker["topics"]["lidar"]
                cloud = ExplorationCloud(client.node, lidar_topic) if lidar_topic else None
                clearance = motion_tracker["collision"]["robot_radius"]

            detect_for_search = skill.detect
            if exploration_memory:
                from .search.memory_exploration import MemoryExploration
                from dimos.plugins.spatial_memory import SimpleMemory, SimpleMemoryUnavailable

                from .search.visualization import ExplorationVisualization

                visualization = ExplorationVisualization(client.node)
                resources.callback(visualization.close)
                memory = SimpleMemory()
                resources.callback(memory.close)
                memory_available = True
                last_memory_frame = None
                has_memory = False

                def memory_call(method, *args):
                    nonlocal memory_available, has_memory
                    if not memory_available:
                        return None
                    try:
                        return method(*args)
                    except TimeoutError:
                        has_memory = False
                        raise
                    except SimpleMemoryUnavailable as exc:
                        memory_available = False
                        event(logger, "simple_memory_unavailable", operation=method.__name__,
                              reason=str(exc))
                        logger.warning("Simple spatial memory unavailable: %s", exc)
                        return None

                def record_memory(frame, remaining, *, background=False):
                    nonlocal last_memory_frame, has_memory
                    frame_id = frame.odom.header.frame_id
                    odom_stamp = stamp_ns(frame.odom.header)
                    frame_fields = None
                    if recording_enabled():
                        frame_fields = {
                            "observation_id": getattr(frame, "observation_id", None),
                            "source_stamp_ns": frame.stamp_ns, "odom_stamp_ns": odom_stamp,
                            "world_frame": frame_id, "sampling_source": "background" if background else "detection",
                            "depth_gap_ms": frame.depth_gap_ms if math.isfinite(getattr(frame, "depth_gap_ms", math.inf)) else None,
                            "odom_gap_ms": frame.odom_gap_ms if math.isfinite(getattr(frame, "odom_gap_ms", math.inf)) else None,
                        }
                    if not memory_available:
                        if frame_fields is not None:
                            event(logger, "simple_memory_sample", **frame_fields,
                                  outcome="skipped", reason="memory_unavailable")
                        return
                    if last_memory_frame is not None:
                        old_id, old_rgb_stamp, old_odom_stamp = last_memory_frame
                        reset_reason = None
                        if frame_id != old_id:
                            reset_reason = "world_frame_changed"
                        elif frame.stamp_ns < old_rgb_stamp:
                            reset_reason = "rgb_stamp_regressed"
                        elif odom_stamp < old_odom_stamp:
                            reset_reason = "odom_stamp_regressed"
                        if reset_reason is not None:
                            memory.reset()
                            last_memory_frame = None
                            has_memory = False
                            if frame_fields is not None:
                                event(logger, "simple_memory_reset", **frame_fields, reason=reset_reason,
                                      previous_world_frame=old_id, previous_rgb_stamp_ns=old_rgb_stamp,
                                      previous_odom_stamp_ns=old_odom_stamp)
                        elif frame.stamp_ns == old_rgb_stamp:
                            if frame_fields is not None:
                                event(logger, "simple_memory_sample", **frame_fields,
                                      outcome="skipped", reason="duplicate_rgb_stamp")
                            return
                    last_memory_frame = frame_id, frame.stamp_ns, odom_stamp
                    position = frame.odom.pose.position
                    orientation = frame.odom.pose.orientation
                    pose = (
                        position.x, position.y, position.z,
                        orientation.x, orientation.y, orientation.z, orientation.w,
                    )
                    record = partial(
                        memory.record,
                        min_distance_m=0.2,
                        min_yaw_delta_deg=15.0,
                        min_interval_s=1.0,
                        sample_group="background" if background else None,
                        stamp_ns=frame.stamp_ns,
                        observation_id=getattr(frame, "observation_id", None),
                    )
                    with event_context(**(frame_fields or {})):
                        stored = memory_call(record, frame.jpeg, pose, remaining)
                    if stored:
                        has_memory = True
                    return

                def query_memory(target, remaining):
                    if not memory_available or not has_memory:
                        event(logger, "simple_memory_lookup", target=target, outcome="skipped",
                              reason="empty_memory" if memory_available else "memory_unavailable")
                        return None
                    return memory_call(memory.query_hits, target, remaining)

                memory_lock = threading.Lock()
                stopped = threading.Event()
                executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="memory-query")
                motion_detection_executor = resources.enter_context(
                    ThreadPoolExecutor(max_workers=1, thread_name_prefix="memory-motion-detection")
                )
                prefetched = None

                def lookup(target, remaining, lookup_id):
                    with memory_lock, event_context(memory_lookup_id=lookup_id, memory_trigger="parallel_detection"):
                        return query_memory(target, remaining)

                def detect_for_search(frame, target, remaining, *, memory_target=None, **kwargs):
                    nonlocal prefetched
                    prefetched = None
                    recording = recording_enabled()
                    lookup_id = uuid4().hex if recording else None
                    fields = {
                        "memory_lookup_id": lookup_id, "source_stamp_ns": frame.stamp_ns,
                        "observation_id": getattr(frame, "observation_id", None),
                    } if recording else {}
                    if recording:
                        event(logger, "parallel_detection_started", **fields)
                    decision = "detection_error"
                    future = executor.submit(
                        copy_context().run, lookup, memory_target or target, remaining, lookup_id,
                    )
                    try:
                        result = skill.detect(frame, target, remaining, **kwargs)
                        if not result.get("bbox_2d") and result["status"] != "localized":
                            decision = "memory_wait"
                            try:
                                hits = future.result(timeout=remaining())
                                prefetched = frame.stamp_ns, (lookup_id, hits)
                                decision = "memory_hits" if hits else "memory_no_hits"
                            except FutureTimeout:
                                remaining()
                                prefetched = frame.stamp_ns, (lookup_id, None)
                                decision = "memory_timeout"
                        else:
                            decision = "vision_priority"
                        if memory_lock.acquire(blocking=False):
                            try:
                                if last_memory_frame is None or frame.stamp_ns > last_memory_frame[1]:
                                    record_memory(frame, remaining)
                            finally:
                                memory_lock.release()
                        return result
                    finally:
                        cancelled = future.cancel()
                        if recording:
                            event(logger, "parallel_detection_finished", **fields,
                                  decision=decision, query_cancelled=cancelled)

                def take_prefetched(frame):
                    return prefetched[1] if prefetched and prefetched[0] == frame.stamp_ns else (None, None)

                def sample_background():
                    def remaining():
                        if stopped.is_set():
                            raise TimeoutError("Memory sampling stopped")
                        return 65.0

                    try:
                        with memory_lock:
                            metrics = memory_call(memory.preload, remaining)
                        if metrics is not None:
                            logger.info("Spatial memory preload ready backend=%s", metrics["backend"])
                    except TimeoutError:
                        if stopped.is_set():
                            return
                        logger.warning("Spatial memory preload timed out")

                    last_stamp = None
                    while not stopped.wait(0.2):
                        frame = observations.memory_view()
                        if frame is None or frame.stamp_ns == last_stamp:
                            continue
                        if not memory_lock.acquire(blocking=False):
                            continue
                        try:
                            record_memory(frame, lambda: min(30.0, remaining()), background=True)
                            last_stamp = frame.stamp_ns
                        except TimeoutError:
                            if not stopped.is_set():
                                logger.warning("Background memory sampling timed out")
                                if recording_enabled():
                                    event(logger, "background_memory_timeout", source_stamp_ns=frame.stamp_ns)
                        finally:
                            memory_lock.release()

                sampler = threading.Thread(target=sample_background, name="memory-sampling")
                sampler.start()

                def stop_memory():
                    stopped.set()
                    sampler.join()
                    executor.shutdown(cancel_futures=True)

                resources.callback(stop_memory)

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

            def retry_view(previous, remaining):
                return observations.snapshot(remaining, after_stamp_ns=previous.stamp_ns)

            def confirmed(frame, outcome, remaining):
                if approach is not None:
                    return outcome
                return skill.plan_detection(frame, outcome, remaining)

            search = ActiveSearch(
                observe,
                partial(detect_for_search, allow_bad_depth=True, purpose="search_det"),
                confirmed,
            )

            def explore(target, remaining, progress, cancel, frame, detection, *, memory_target=None):
                next_view = toward_target = None
                if approach is not None:
                    mapper = DepthOccupancyMap(profile["camera"], footprint_radius=clearance)
                    planner = FrontierPlanner(clearance_m=clearance)
                    motion_args = (
                        observations.snapshot, observations.latest_rgb_stamp, approach,
                        mapper, planner, cancel,
                    )
                    if exploration_memory:
                        directed = MemoryExploration(
                            *motion_args, cloud=cloud, query_memory=query_memory,
                            turn=tracker.turn_to, visualization=visualization,
                            prefetched_memory=take_prefetched,
                        )
                        def detect_during_motion(seed, stopped, deadline):
                            def motion_budget():
                                if stopped.is_set():
                                    raise TimeoutError("Historical viewpoint motion ended")
                                left = deadline - time.monotonic()
                                if left <= 0:
                                    raise TimeoutError("Historical viewpoint motion ended")
                                return min(remaining(), left)

                            stamp = seed.stamp_ns
                            while not stopped.is_set():
                                try:
                                    candidate = observations.snapshot(motion_budget, after_stamp_ns=stamp)
                                    stamp = candidate.stamp_ns
                                    result = skill.detect(
                                        candidate, target, motion_budget, allow_bad_depth=True,
                                        stop_m=detection.get("standoff_m"), purpose="memory_motion_det",
                                    )
                                    motion_budget()
                                except (HTTPError, RuntimeError, ValueError, TimeoutError) as exc:
                                    logger.info("Memory motion detection ended error_type=%s", type(exc).__name__)
                                    return None
                                if result["status"] in {"localized", "invalid_depth"} and result.get("bbox_2d"):
                                    return candidate, result
                                if result["status"] == "provider_failure":
                                    return None
                            return None

                        def start_motion_detection(seed, stopped, deadline):
                            return motion_detection_executor.submit(
                                copy_context().run, detect_during_motion, seed, stopped, deadline,
                            )

                        directed.start_motion_detection = start_motion_detection
                        return directed.run_search(
                            ActiveSearch(observe, partial(detect_for_search, allow_bad_depth=True,
                                                        purpose="search_det", memory_target=memory_target),
                                         confirmed),
                            target, remaining, progress, frame, detection,
                            camera=profile["camera"], memory_target=memory_target, rotation_fallback=True,
                            retry_view=retry_view,
                        )
                    else:
                        next_view = RotationSweep(
                            observations.snapshot, observations.latest_rgb_stamp,
                            tracker.turn_to, frame, cancel,
                        )
                        directed = DirectedExploration(*motion_args, cloud=cloud)
                    toward_target = partial(directed.toward_target, camera=profile["camera"])
                return search(
                    target, remaining, progress, next_view=next_view,
                    initial_frame=frame, initial_detection=detection,
                    toward_target=toward_target, retry_view=retry_view,
                )

            find_object = FindObject(
                observations.snapshot, detect_for_search, explore, confirmed,
                search_branch="frontier_memory" if exploration_memory else "rotation_scan",
            )
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

    def run_with_reference(self, goal, image, *, cancel=None, on_progress=None):
        return self.agent.run_with_reference(goal, image, cancel=cancel, on_progress=on_progress)

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.resources.close()
