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

"""Fast tests without detector, ROS, live robot or LLM calls."""

from __future__ import annotations

import json
import math
import time
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np
from dimos.agents.skills import config
from dimos.agents.skills import vln_skill as vln
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.navigation.visual import target_motion as motion
from dimos.navigation.visual import target_perception as adapter
from dimos.protocol.rpc.pubsubrpc import LCMRPC

CAMERAS = ["d455", "x3_front", "x3_right", "x3_back", "x3_left"]
GEOMETRY = {"d455_intrinsic": [[50, 0, 50], [0, 50, 30], [0, 0, 1]], "d455_pitch": 0}


def obs(camera="d455", stamp=None, track=None, bbox=(40, 10, 60, 50)):
    return adapter.DetectionObservation(
        camera,
        "frame",
        stamp or time.time() - 0.05,
        100,
        60,
        "person",
        0.9,
        bbox,
        track or camera + ":1",
        tuple([1] + [0] * 63),
        1,
    )


def payload(objects, seq=1, generation=1):
    views = []
    for camera in CAMERAS:
        items = [o for o in objects if o.camera == camera]
        ts = items[0].stamp if items else time.time() - 0.02
        views.append(
            {
                "camera_id": camera,
                "frame_id": items[0].frame_id if items else camera,
                "source_stamp_ns": str(round(ts * 1e9)),
                "width": 100,
                "height": 60,
                "status": "fresh",
                "detections": [
                    {
                        "class_name": o.label,
                        "score": o.score,
                        "bbox_xyxy": list(o.bbox),
                        "camera_track_id": o.track_id,
                        "appearance": list(o.appearance),
                    }
                    for o in items
                ],
            }
        )
    return {
        "schema_version": 1,
        "instance_id": "instance",
        "generation": generation,
        "task_id": "task",
        "seq": seq,
        "status": "ok",
        "views": views,
    }


def client():
    c = adapter.PerceptionClient(
        {
            "endpoint": "http://127.0.0.1:1",
            "lifecycle": "external",
            "mode": "assist",
            **GEOMETRY,
        }
    )
    c.task_id = "task"
    c.generation = 1
    c.labels = ["person"]
    c.error = ""
    c.last_receive = time.monotonic()
    return c


def make_skill(mode="assist"):
    class QuietRPC(LCMRPC):
        def __init__(self):
            pass

        def serve_module_rpc(self, *args, **kwargs):
            pass

        def start(self):
            pass

        def stop(self):
            pass

    skill = vln.VLNSkillContainer(
        vlm_backend="qwen3_local",
        rpc_transport=QuietRPC,
        perception={"mode": mode, "motion_enabled": False},
    )
    skill._started = True
    skill._stop_exploration = Mock(return_value=True)
    skill._stop_motion_for_vlm_check = Mock()
    skill._resume_motion_after_vlm_check = Mock()
    return skill


class StateTests(unittest.TestCase):
    def test_repeated_poll_does_not_confirm_same_frame(self):
        c = client()
        o = obs()
        c.payload = payload([o])
        for _ in range(5):
            self.assertEqual(c.state()["status"], "candidate")
        n = obs(stamp=o.stamp + 0.01)
        c.payload = payload([n], seq=2)
        self.assertEqual(c.state()["status"], "front_visible")

    def test_multiple_instances_require_selection(self):
        c = client()
        a = obs(track="a")
        b = obs(track="b", bbox=(5, 10, 25, 50))
        c.payload = payload([a, b])
        c.state()
        c.payload = payload(
            [
                obs(stamp=a.stamp + 0.01, track="a"),
                obs(stamp=a.stamp + 0.01, track="b", bbox=b.bbox),
            ],
            seq=2,
        )
        self.assertEqual(c.state()["status"], "ambiguous")
        self.assertEqual(c.state("a")["status"], "front_visible")

    def test_stale_views_are_unavailable_not_lost(self):
        c = client()
        c.target = obs()
        c.target_seen = time.time() - 1
        p = payload([])
        p["views"][2]["status"] = "stale"
        c.payload = p
        self.assertEqual(c.state()["status"], "unavailable")
        c.payload = payload([], seq=2)
        self.assertEqual(c.state()["status"], "lost")

    def test_different_instance_does_not_replace_locked_target(self):
        c = client()
        a = obs(track="a")
        c.payload = payload([a])
        c.state()
        c.payload = payload([obs(stamp=a.stamp + 0.01, track="a")], seq=2)
        c.state()
        b = obs(track="b")
        c.payload = payload([b], seq=3)
        c.state()
        c.payload = payload([obs(stamp=b.stamp + 0.01, track="b")], seq=4)
        self.assertEqual(c.state()["status"], "ambiguous")
        self.assertEqual(c.target.track_id, "a")

    def test_invalid_bbox_units_and_nonfinite_rejected(self):
        for bad in [[0, 0, 1000, 1000], [60, 10, 40, 50], [0, 0, float("nan"), 10]]:
            p = payload([obs()])
            p["views"][0]["detections"][0]["bbox_xyxy"] = bad
            with self.assertRaises(ValueError):
                adapter.decode_observations(p, 350)
        p = payload([obs(stamp=time.time() - 2)])
        self.assertEqual(adapter.decode_observations(p, 350), [])

    def test_true_projection_and_clockwise_sign(self):
        o = adapter.DetectionObservation(
            "x3_front",
            "f",
            time.time(),
            101,
            61,
            "person",
            0.9,
            (65, 20, 85, 40),
            "a",
            (),
            1,
        )
        ray = adapter.observation_ray(o, {})
        clockwise = math.degrees(-math.atan2(ray[1], ray[0]))
        self.assertAlmostEqual(clockwise, math.degrees(math.atan(0.5)))
        right = adapter.observation_ray(obs("x3_right", bbox=(40, 20, 59, 39)), {})
        self.assertAlmostEqual(math.degrees(-math.atan2(right[1], right[0])) % 360, 90)

    def test_pose_interpolation_and_rotation_compensation(self):
        c = client()
        ts = time.time() - 0.15

        def pose(t, yaw):
            return SimpleNamespace(
                ts=t,
                position=SimpleNamespace(x=0, y=0, z=0),
                orientation=SimpleNamespace(
                    x=0, y=0, z=math.sin(yaw / 2), w=math.cos(yaw / 2)
                ),
            )

        c.poses.add(pose(ts, 0))
        c.poses.add(pose(ts + 0.15, math.pi / 2))
        position, rotation = c.poses.at(ts + 0.075)
        np.testing.assert_allclose(
            rotation @ np.array([1, 0, 0]),
            [math.sqrt(0.5), math.sqrt(0.5), 0],
            atol=1e-6,
        )
        bearing = c.current_bearing(obs("x3_right", stamp=ts, bbox=(40, 20, 59, 39)))
        self.assertAlmostEqual(bearing, 180, delta=0.05)


class IntegrationTests(unittest.TestCase):
    def test_direct_sofa_search_binds_before_search_thread(self):
        skill = make_skill()
        skill._perception = Mock()
        skill._perception.task_id = None
        calls = []

        def bind(description, labels, task_id):
            calls.append(("bind", description, labels))
            skill._perception.task_id = "sofa-task"
            return "sofa-task"

        skill._perception.ensure_task.side_effect = bind
        skill._vln_search = Mock(
            side_effect=lambda *args: calls.append(("search",)) or "done"
        )
        self.assertEqual(skill.active_search("sofa"), "done")
        self.assertEqual(calls, [("bind", "sofa", ["sofa"]), ("search",)])
        self.assertFalse(skill._search_active)

    def test_sofa_aliases_and_task_category_match(self):
        for description in ("sofa", " SOFA ", "couch", "沙发"):
            with self.subTest(description=description):
                self.assertEqual(adapter.english_labels(description), ["sofa"])
        skill = make_skill()
        skill._perception = Mock()
        skill._perception.labels = ["couch"]
        self.assertTrue(skill._ensure_perception_target("sofa", "task"))
        skill._perception.ensure_task.assert_called_once_with(
            "sofa", ["couch"], "task"
        )

    def test_assist_missing_client_blocks_before_exploration(self):
        skill = make_skill()
        skill._vln_search = Mock()
        result = json.loads(skill.active_search("sofa"))
        self.assertEqual(result["status"], "needs_target_binding")
        self.assertEqual(result["next_action"], "locate_target_with_x3")
        skill._vln_search.assert_not_called()
        skill._stop_exploration.assert_not_called()
        self.assertFalse(skill._search_active)

    def test_assist_backend_failure_blocks_before_exploration(self):
        skill = make_skill()
        skill._perception = Mock()
        skill._perception.task_id = None
        skill._perception.ensure_task.side_effect = vln.requests.ConnectionError(
            "offline"
        )
        skill._vln_search = Mock()
        self.assertEqual(
            json.loads(skill.active_search("sofa"))["status"], "needs_target_binding"
        )
        skill._vln_search.assert_not_called()
        self.assertFalse(skill._search_active)

    def test_unsupported_description_requires_explicit_labels(self):
        skill = make_skill()
        skill._perception = Mock()
        skill._vln_search = Mock()
        result = json.loads(skill.active_search("穿黑色衣服的人"))
        self.assertEqual(result["status"], "needs_target_binding")
        skill._perception.ensure_task.assert_not_called()
        skill._vln_search.assert_not_called()

    def test_room_object_search_requires_binding_too(self):
        skill = make_skill()
        skill._decompose_goal = Mock(
            return_value={"room": "office", "object": "sofa"}
        )
        skill._vln_search = Mock()
        result = json.loads(skill.find_object_in_room("sofa in office"))
        self.assertEqual(result["status"], "needs_target_binding")
        skill._vln_search.assert_not_called()
        self.assertFalse(skill._search_active)

    def test_explicit_person_session_cannot_bind_sofa_search(self):
        skill = make_skill()
        skill._perception = Mock()
        skill._perception.labels = ["person"]
        skill._vln_search = Mock()
        result = json.loads(skill.active_search("sofa", "person-task"))
        self.assertEqual(result["status"], "invalid_target_task")
        skill._perception.ensure_task.assert_not_called()
        skill._vln_search.assert_not_called()

    def test_legacy_modes_keep_direct_search(self):
        for mode in ("disabled", "shadow"):
            with self.subTest(mode=mode):
                skill = make_skill(mode)
                skill._vln_search = Mock(return_value="legacy done")
                self.assertEqual(skill.active_search("sofa"), "legacy done")
                skill._vln_search.assert_called_once_with(None, "sofa")

    def test_backend_failure_does_not_spam_vlm_fallback(self):
        skill = make_skill()
        skill._ensure_perception_target = Mock(return_value=True)
        skill._perception = Mock()
        skill._perception.state.return_value = {"status": "unavailable"}
        skill._latest_image = Image(
            data=np.zeros((60, 100, 3), np.uint8), format=ImageFormat.BGR
        )
        skill._run_vlm_check_while_paused = Mock(side_effect=lambda fn: fn())
        with patch.object(
            vln, "get_object_bbox_from_image", return_value=None
        ) as query:
            skill._check_object_in_view("person")
            skill._check_object_in_view("person")
            self.assertEqual(query.call_count, 1)

    def test_shadow_never_uses_detector_to_control(self):
        skill = make_skill("shadow")
        skill._perception = Mock()
        self.assertFalse(skill._perception_assist())
        result = json.loads(skill.face_target("task"))
        self.assertEqual(result["status"], "motion_disabled")
        skill._stop_exploration.assert_not_called()
        result = json.loads(skill.locate_target_with_x3("person", ["person"]))
        self.assertEqual(result["status"], "shadow")
        self.assertNotIn("clockwise_bearing_deg", result)
        skill._perception.ensure_task.assert_not_called()

    def test_turn_uses_latest_bearing_and_requires_new_front_frame(self):
        skill = make_skill()
        skill.config.perception.update(motion_enabled=True, calibration_verified=True)
        skill._latest_odom = PoseStamped(
            position=[0, 0, 0], orientation=[0, 0, 0, 1], frame_id="world"
        )
        c = Mock()
        c.task_id = "task"
        skill._perception = c
        c.state.side_effect = [
            {"status": "x3_covered", "clockwise_bearing_deg": 90},
            {
                "status": "front_visible",
                "front": SimpleNamespace(stamp=time.time() + 0.1),
            },
        ]
        skill.target_motion = SimpleNamespace(
            begin_target_turn=Mock(return_value="lease"),
            set_target_turn=Mock(return_value=True),
            end_target_turn=Mock(return_value=True),
        )
        result = json.loads(skill.face_target("task"))
        self.assertEqual(result["status"], "front_reacquired")
        goal = skill.target_motion.set_target_turn.call_args.args[1]
        self.assertAlmostEqual(goal.orientation.to_euler().z, -math.pi / 2)
        skill.target_motion.end_target_turn.assert_called_once_with("lease")
        self.assertFalse(skill._target_turn_lock.locked())

    def test_turn_failure_releases_ownership(self):
        skill = make_skill()
        skill.config.perception.update(motion_enabled=True, calibration_verified=True)
        skill._latest_odom = PoseStamped(position=[0, 0, 0], orientation=[0, 0, 0, 1])
        skill._perception = Mock()
        skill._perception.task_id = "task"
        skill._perception.state.return_value = {
            "status": "x3_covered",
            "clockwise_bearing_deg": 90,
        }
        skill.target_motion = SimpleNamespace(
            begin_target_turn=Mock(return_value="lease"),
            set_target_turn=Mock(side_effect=RuntimeError("planner failed")),
            end_target_turn=Mock(),
        )
        with self.assertRaises(RuntimeError):
            skill.face_target("task")
        skill.target_motion.end_target_turn.assert_called_once_with("lease")
        self.assertFalse(skill._target_turn_lock.locked())

    def test_invalid_task_does_not_start_navigation(self):
        skill = make_skill()
        skill._ensure_perception_target = Mock(return_value=False)
        skill._vln_search = Mock()
        self.assertEqual(
            json.loads(skill.active_search("person", "wrong"))["status"],
            "invalid_target_task",
        )
        skill._vln_search.assert_not_called()
        self.assertFalse(skill._search_active)

    def test_fast_check_pairs_source_rgb_depth_and_pose(self):
        skill = make_skill()
        c = client()
        skill._perception = c
        o = obs()
        pose = PoseStamped(ts=o.stamp, position=[0, 0, 0], orientation=[0, 0, 0, 1])
        c.poses.add(pose)
        skill._latest_odom = pose
        c.state = Mock(return_value={"status": "front_visible", "front": o})
        skill._target_images.append(
            Image(
                data=np.zeros((60, 100, 3), np.uint8),
                ts=o.stamp,
                format=ImageFormat.BGR,
            )
        )
        skill._target_depths.append(
            Image(
                data=np.ones((60, 100), np.float32),
                ts=o.stamp + 0.04,
                format=ImageFormat.DEPTH,
            )
        )
        bbox, cap = skill._fast_target_check("person")
        self.assertEqual(bbox, o.bbox)
        self.assertAlmostEqual(cap.ts, o.stamp)
        self.assertIsNotNone(skill._target_snapshot.depth)
        skill._stop_motion_for_vlm_check.assert_not_called()
        skill._target_depths.clear()
        skill._target_depths.append(
            Image(
                data=np.ones((60, 100), np.float32),
                ts=o.stamp + 0.2,
                format=ImageFormat.DEPTH,
            )
        )
        skill._fast_target_check("person")
        self.assertIsNone(skill._target_snapshot.depth)

    def test_fast_confirmation_has_no_legacy_wait(self):
        skill = make_skill()
        skill._perception = Mock()
        skill._perception_bound_object = "person"
        skill._fast_target_check = Mock(return_value=((1, 2, 3, 4), None))
        start = time.monotonic()
        self.assertEqual(skill._confirm_detection("person"), (1, 2, 3, 4))
        self.assertLess(time.monotonic() - start, 0.1)

    def test_reacquisition_propagates_out_of_blocking_skill(self):
        skill = make_skill()
        skill._ensure_perception_target = Mock(return_value=True)
        skill._vln_search = Mock(
            side_effect=vln.TargetPerceptionHandoff(
                {"status": "needs_reacquire", "task_id": "task"}
            )
        )
        result = json.loads(skill.active_search("person"))
        self.assertEqual(result["status"], "needs_reacquire")
        self.assertFalse(skill._search_active)

    def test_exceptions_do_not_become_lost(self):
        skill = make_skill()
        skill._perception = Mock()
        skill._perception_approaching = True
        skill._perception.state.return_value = {
            "status": "unavailable",
            "reason": "server_down",
        }
        skill._perception.public_state.return_value = {"status": "unavailable"}
        with self.assertRaises(vln.TargetPerceptionHandoff) as caught:
            skill._fast_target_check("person")
        self.assertEqual(caught.exception.result["status"], "perception_unavailable")

    def test_motion_lease_checks_navdp_then_releases_pause(self):
        class QuietRPC(LCMRPC):
            def __init__(self):
                pass

            def serve_module_rpc(self, *a, **k):
                pass

            def start(self):
                pass

            def stop(self):
                pass

        coordinator = motion.NavDPTargetMotionCoordinator(
            enabled=True, rpc_transport=QuietRPC
        )
        calls = []
        coordinator.navigator = SimpleNamespace(
            cancel_goal=lambda: calls.append("navdp_cancel") or True,
            get_navdp_state=lambda: "IDLE",
            resume_motion=lambda: calls.append("navdp_resume") or True,
        )
        coordinator.planner = SimpleNamespace(
            cancel_goal=lambda: calls.append("astar_cancel") or True,
            set_goal=Mock(return_value=True),
        )
        lease = coordinator.begin_target_turn()
        self.assertTrue(lease)
        self.assertEqual(calls, ["navdp_cancel", "navdp_resume", "astar_cancel"])
        self.assertEqual(coordinator.begin_target_turn(), "")
        self.assertFalse(coordinator.set_target_turn("wrong", PoseStamped()))
        self.assertTrue(coordinator.end_target_turn(lease))
        self.assertFalse(coordinator._lease_lock.locked())

    def test_disabled_config_default_and_invalid_modes(self):
        self.assertEqual(config.PerceptionConfig().mode, "disabled")
        with self.assertRaises(ValueError):
            config.PerceptionConfig(mode="typo")
