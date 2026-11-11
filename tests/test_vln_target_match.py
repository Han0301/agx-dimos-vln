import json
import threading
from types import SimpleNamespace as NS

import numpy as np
import pytest


BOX = [400, 400, 600, 600]
TARGET = "wall-mounted fire extinguisher box"


def frame(stamp=100, depth=2.0):
    return NS(
        k=[100., 0., 50., 0., 100., 50., 0., 0., 1.],
        depth=np.full((100, 100), depth),
        distortion_model="",
        distortion=(),
        odom=NS(
            header=NS(frame_id="world"),
            pose=NS(
                position=NS(x=0., y=0., z=0.),
                orientation=NS(x=0., y=0., z=0., w=1.),
            ),
        ),
        jpeg=b"frame", stamp_ns=stamp, depth_gap_ms=0, odom_gap_ms=0, wait_ms=0,
    )


def candidate(label, appearance="match", evidence="cabinet with an extinguisher inside"):
    return dict(label=label, bbox_2d=BOX, appearance_match=appearance, evidence=evidence)


def make_skill(observation, reply):
    from dimos.plugins.vln_task.preview import NavigationPreview

    requests = []

    def query(*args, source, **kwargs):
        requests.append(source)
        return {"text": json.dumps(reply), "elapsed_ms": 10}

    skill = NavigationPreview(
        NS(config={"timeout_s": 2}, query=query), lambda _: observation,
        None, None, {"translation": [0., 0., 0.], "pitch": 0.}, 0.8,
    )
    return skill, requests


@pytest.mark.parametrize("appearance", ["mismatch", "uncertain"])
def test_first_view_mismatch_seeds_frontier_with_its_detection(appearance):
    from dimos.plugins.vln_task.find_object import FindObject

    first = frame()
    skill, requests = make_skill(first, [
        candidate("fire extinguisher box", appearance, "water taps and drip tray"),
    ])
    reviews = []

    def verify(*args, include_context):
        reviews.append(include_context)
        return "mismatch", 5

    def explore(target, remaining, progress, cancel, observation, seed):
        assert observation is first
        assert seed["status"] == "target_mismatch"
        assert seed["bbox_2d"] == BOX
        return {"status": "search_exhausted"}

    def localized(*args):
        pytest.fail("A rejected candidate started an approach")

    skill.verify = verify
    find = FindObject(skill.snapshot, skill.detect, explore, localized)
    result = find(TARGET, lambda: 2, None, threading.Event())
    assert result["status"] == "search_exhausted"
    assert requests == ["d455"]
    assert reviews == ([True] if appearance == "uncertain" else [])


def test_agent_uses_one_vlm_verdict_even_when_the_label_uses_another_language():
    from dimos.plugins.vln_task.agent import TaskAgent
    from dimos.plugins.vln_task.find_object import FindObject

    skill, requests = make_skill(frame(), [candidate("灭火器箱")])
    find = FindObject(skill.snapshot, skill.detect, None, lambda _, result, __: result)
    llm_requests = []

    def chat(*args, **kwargs):
        llm_requests.append(kwargs)
        return {"message": {"role": "assistant", "content": "", "tool_calls": [
            {"function": {"name": "find_object", "arguments": {
                "target": TARGET,
            }}},
        ]}}

    def approach(result, *_args, **_kwargs):
        return {**result, "status": "arrived"}

    llm = NS(config={"timeout_s": 2}, chat=chat)
    agent = TaskAgent(
        llm, None, {"timeout_s": 5, "max_steps": 2}, skill,
        find_object=find, approach=approach,
    )
    result = agent.run("find a wall-mounted fire extinguisher box")
    assert result["navigation"]["status"] == "arrived"
    assert result["navigation"]["label"] == "灭火器箱"
    assert result["navigation"]["target_match"] == {"verdict": "match", "method": "detector"}
    assert len(llm_requests) == 1
    tool = next(
        tool["function"] for tool in llm_requests[0]["tools"]
        if tool["function"]["name"] == "find_object"
    )
    assert set(tool["parameters"]["properties"]) == {"target"}
    assert requests == ["d455"]


@pytest.mark.parametrize("flag", [None, "maybe"])
def test_box_without_a_valid_vlm_verdict_cannot_start_any_motion(flag):
    from dimos.plugins.vln_task.find_object import FindObject

    detection = candidate(TARGET, flag)
    if flag is None:
        del detection["appearance_match"]
    skill, requests = make_skill(frame(depth=0.), [detection])
    entered = []

    def explore(*args):
        entered.append("explore")
        return {"status": "search_exhausted"}

    find = FindObject(skill.snapshot, skill.detect, explore, None)
    result = find(TARGET, lambda: 2, None, threading.Event())
    assert result["status"] == "invalid_detection"
    assert entered == []
    assert requests == ["d455"]


def test_weak_visual_evidence_is_rechecked_once_and_reused_during_search():
    from dimos.plugins.active_search.search import ActiveSearch

    skill, requests = make_skill(frame(101), [candidate("fire extinguisher box", "uncertain", "")])
    verification_calls = []

    def verify(*args, include_context):
        verification_calls.append(include_context)
        return "match", 5

    skill.verify = verify
    search = ActiveSearch(
        None, skill.detect, lambda _, result, __: result,
    )
    result = search(
        TARGET, lambda: 3,
        next_view=lambda *args: skill.snapshot(None),
        initial_frame=frame(), initial_detection={"status": "not_visible"},
    )
    assert result["status"] == "localized"
    assert result["target_match"]["method"] == "visual"
    assert requests == ["d455"]
    assert verification_calls == [True]


def test_fast_refresh_discards_ambiguous_boxes_without_an_extra_model_call():
    skill, requests = make_skill(frame(), [candidate("dispenser", "uncertain", "spout")])

    def unexpected(*args, **kwargs):
        pytest.fail("Background refresh called a second model")

    skill.verify = unexpected
    result = skill.detect(skill.snapshot(None), TARGET, lambda: 2, allow_verification=False)
    assert result["status"] == "verification_unavailable"
    assert requests == ["d455"]


def test_matching_candidate_with_unusable_depth_seeds_directional_exploration():
    from dimos.plugins.vln_task.find_object import FindObject

    skill, _ = make_skill(frame(depth=0.), [candidate(TARGET)])

    def explore(target, remaining, progress, cancel, observation, seed):
        assert target == TARGET
        assert observation is skill.snapshot(None)
        assert seed["status"] == "invalid_depth"
        assert seed["label"] == TARGET
        assert seed["bbox_2d"] == candidate(TARGET)["bbox_2d"]
        assert seed["detection_error"] == "Target box has no usable depth"
        return {"status": "search_exhausted"}

    def localized(*args):
        pytest.fail("Invalid depth started localized approach")

    find = FindObject(skill.snapshot, skill.detect, explore, localized)
    result = find(TARGET, lambda: 2, None, threading.Event())
    assert result["status"] == "search_exhausted"


@pytest.mark.parametrize(("reply", "seed_status"), [
    ([], "not_visible"),
    ([{"status": "uncertain"}], "verification_unavailable"),
    ([candidate(TARGET, "uncertain")], "verification_unavailable"),
])
def test_empty_detection_and_unconfirmed_candidate_seed_frontier(reply, seed_status):
    from dimos.plugins.vln_task.find_object import FindObject

    skill, requests = make_skill(frame(), reply)
    explored = []

    def explore(*args):
        explored.append(args[-1]["status"])
        return {"status": "search_exhausted"}

    find = FindObject(skill.snapshot, skill.detect, explore, None)
    result = find(TARGET, lambda: 2, None, threading.Event())
    assert result["status"] == "search_exhausted"
    assert explored == [seed_status]
    assert requests == ["d455"]


@pytest.mark.parametrize("first_reply", [
    [candidate(TARGET)],
    [{"status": "uncertain"}],
    [candidate(TARGET, "mismatch")],
    [],
])
def test_search_plans_from_explicit_match_or_keeps_exploring(first_reply):
    from dimos.plugins.active_search.search import ActiveSearch

    requests = []
    moves = []

    def detect(observation, target, remaining):
        reply = first_reply if observation.stamp_ns == 101 else [candidate(TARGET)]
        skill, sources = make_skill(observation, reply)
        result = skill.detect(observation, target, remaining, allow_bad_depth=True)
        requests.extend(sources)
        return result

    def next_view(previous, remaining, progress):
        stamp = previous.stamp_ns + 1
        moves.append(stamp)
        return frame(stamp)

    search = ActiveSearch(
        None, detect, lambda observation, result, remaining: {
            "status": "preview_ready", "stamp_ns": observation.stamp_ns,
        },
    )
    result = search(
        TARGET, lambda: 3, next_view=next_view,
        initial_frame=frame(), initial_detection={"status": "not_visible"},
    )
    expected_moves = [101] if first_reply == [candidate(TARGET)] else [101, 102]
    assert moves == expected_moves
    assert requests == ["d455"] * len(expected_moves)
    assert result["status"] == "preview_ready"
    assert result["stamp_ns"] == expected_moves[-1]


def test_visual_prompts_only_request_target_and_response_format():
    from dimos.plugins.active_search.verification import verification_prompt
    from dimos.plugins.vln_task.preview import detection_prompt

    target = "黄色脚踏垃圾桶，位于绿植左侧"
    assert detection_prompt(target).splitlines() == [
        f'Target: "{target}"',
        "Find the target in the image. Return only JSON:",
        "One candidate: an array with one object containing label, bbox_2d, and appearance_match.",
        "bbox_2d: [xmin,ymin,xmax,ymax], normalized to 0–1000, origin at top left.",
        "appearance_match: match, mismatch, or uncertain.",
        "No candidate: []. Uncertain without a box: [{\"status\":\"uncertain\"}].",
    ]
    assert verification_prompt(target, include_context=True) == (
        f'Does the object inside the magenta box match "{target}"? '
        'Return only {"match":"yes"} or {"match":"no"}.'
    )
