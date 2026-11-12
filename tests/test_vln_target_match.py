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


def candidate(name=TARGET):
    return json.dumps({"name": name, "bbox": BOX}, ensure_ascii=False)


def make_skill(observation, reply):
    from dimos.vln_task.preview import NavigationPreview

    requests = []

    def query(*args, source, **kwargs):
        requests.append(source)
        return {"text": reply, "elapsed_ms": 10}

    skill = NavigationPreview(
        NS(config={"timeout_s": 2}, query=query), lambda _: observation,
        None, None, {"translation": [0., 0., 0.], "pitch": 0.}, 0.8,
    )
    return skill, requests


@pytest.mark.parametrize("reply", [
    "None",
    "[]",
    '{"bbox_2d":[400,400,600,600]}',
    '{"name":"cabinet","bbox":[400,400,600]}',
    '{"name":"cabinet","bbox":[400,400,600,}',
    '[{"bbox":[400,400,600,600]},{"bbox":[100,100,200,200]}]',
    '{"name":"cabinet","bbox":[600,400,400,600]}',
    '{"name":"cabinet","bbox":[400,400,1001,600]}',
])
def test_unusable_first_detection_seeds_search_without_a_box(reply):
    from dimos.vln_task.find_object import FindObject

    first = frame()
    skill, requests = make_skill(first, reply)

    def explore(target, remaining, progress, cancel, observation, seed):
        assert observation is first
        assert seed["status"] == "not_visible"
        assert "bbox_2d" not in seed
        return {"status": "search_exhausted"}

    def localized(*args):
        pytest.fail("An unusable detection started an approach")

    find = FindObject(skill.snapshot, skill.detect, explore, localized)
    assert find(TARGET, lambda: 2, None, threading.Event())["status"] == "search_exhausted"
    assert requests == ["d455"]


def test_agent_uses_one_bbox_without_a_label_or_semantic_check():
    from dimos.vln_task.agent import TaskAgent
    from dimos.vln_task.find_object import FindObject

    skill, requests = make_skill(frame(), candidate("灭火器箱"))
    find = FindObject(skill.snapshot, skill.detect, None, lambda _, result, __: result)
    llm_requests = []

    def chat(*args, **kwargs):
        llm_requests.append(kwargs)
        return {"message": {"role": "assistant", "content": "", "tool_calls": [
            {"function": {"name": "find_object", "arguments": {"target": TARGET}}},
        ]}}

    def approach(result, *_args, **_kwargs):
        return {**result, "status": "arrived"}

    agent = TaskAgent(
        NS(config={"timeout_s": 2}, chat=chat), None, {"timeout_s": 5, "max_steps": 2},
        skill, find_object=find, approach=approach,
    )
    result = agent.run("find a wall-mounted fire extinguisher box")
    assert result["navigation"]["status"] == "arrived"
    assert result["navigation"]["bbox_2d"] == BOX
    assert "target_match" not in result["navigation"]
    assert len(llm_requests) == 1
    tool = next(
        tool["function"] for tool in llm_requests[0]["tools"]
        if tool["function"]["name"] == "find_object"
    )
    assert set(tool["parameters"]["properties"]) == {"target"}
    assert requests == ["d455"]


def test_bbox_with_unusable_depth_seeds_directional_exploration():
    from dimos.vln_task.find_object import FindObject

    skill, requests = make_skill(frame(depth=0.), candidate())

    def explore(target, remaining, progress, cancel, observation, seed):
        assert target == TARGET
        assert observation is skill.snapshot(None)
        assert seed["status"] == "invalid_depth"
        assert seed["bbox_2d"] == BOX
        assert seed["detection_error"] == "Target box has no usable depth"
        return {"status": "search_exhausted"}

    def localized(*args):
        pytest.fail("Invalid depth started localized approach")

    find = FindObject(skill.snapshot, skill.detect, explore, localized)
    assert find(TARGET, lambda: 2, None, threading.Event())["status"] == "search_exhausted"
    assert requests == ["d455"]


@pytest.mark.parametrize("first_reply,expected_moves", [
    (candidate(), [101]),
    ("None", [101, 102]),
    ('{"bbox_2d":[400,400,600,600]}', [101, 102]),
])
def test_search_plans_from_a_valid_bbox_or_keeps_exploring(first_reply, expected_moves):
    from dimos.plugins.active_search.search import ActiveSearch

    requests = []
    moves = []

    def detect(observation, target, remaining):
        reply = first_reply if observation.stamp_ns == 101 else candidate()
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
    assert moves == expected_moves
    assert requests == ["d455"] * len(expected_moves)
    assert result["status"] == "preview_ready"
    assert result["stamp_ns"] == expected_moves[-1]


def test_find_object_prompt_requests_valid_json_with_normalized_coordinates():
    from dimos.vln_task.preview import detection_prompt

    target = "黄色脚踏垃圾桶，位于绿植左侧"
    prompt = detection_prompt(target)
    assert target in prompt and "0–1000" in prompt
    assert '{"name":"object_name","bbox":' in prompt


def test_independent_verification_prompt_defines_match_criteria():
    from dimos.plugins.active_search.verification import verification_prompt

    target = "黄色脚踏垃圾桶，位于绿植左侧"
    review = verification_prompt(target, include_context=True)
    assert f'Does the object inside the magenta box match "{target}"?' in review
    assert "category and every requested attribute" in review
    assert 'Return only {"match":"yes"} or {"match":"no"}.' in review


@pytest.mark.parametrize("reply,status,message", [
    (candidate(), "localized", "FOUND bbox="),
    ("None", "not_visible", "not found"),
])
def test_detection_logs_bbox_or_miss(reply, status, message, caplog):
    from dimos.vln_task import perception

    caplog.set_level("INFO", logger=perception.logger.name)
    observation = frame()
    skill, _ = make_skill(observation, reply)

    assert skill.detect(observation, TARGET, lambda: 2)["status"] == status
    assert f"VLM check {TARGET!r}: {message}" in caplog.text
