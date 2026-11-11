"""Bounded LLM tool loop. Each run owns its messages and optional image."""

import json
import logging
import threading
import time

from dimos.plugins.runtime_log import event, event_context

from dimos.plugins._agent_common import TaskCancelled

from .standoff import requested_standoff

logger = logging.getLogger('dimos.plugins.vln_task.agent')

TOOLS = [
    {
        "type": "function",
        "function": {
            "name": "inspect_image",
            "description": "Ask the VLM about the image attached to this task.",
            "parameters": {
                "type": "object",
                "properties": {"prompt": {"type": "string"}},
                "required": ["prompt"],
                "additionalProperties": False,
            },
        },
    }
]
SYSTEM = """You are a robot task assistant running standalone tests. Answer in the user's language.
For questions about the attached image, call inspect_image first and answer using its observation.
For text-only questions, answer directly. The only available tool is inspect_image.
For navigation requests, explain the intended target and information needed to locate it.
Report only actions you actually performed. Return a concise final answer after the tool result."""

ACTIVE_SEARCH_TOOL = {
    "type": "function",
    "function": {
        "name": "active_search",
        "description": "Search changing camera views for a described object, verify it, and preview a path.",
        "parameters": {
            "type": "object",
            "properties": {"target": {"type": "string"}},
            "required": ["target"],
            "additionalProperties": False,
        },
    },
}


FIND_OBJECT_TOOL = {
    **ACTIVE_SEARCH_TOOL,
    "function": {
        **ACTIVE_SEARCH_TOOL["function"],
        "name": "find_object",
        "description": "Check the current D455 view, then explore if the object is not visible.",
    },
}


class TaskAgent:
    def __init__(
        self, llm, vlm, config, navigation_preview=None, active_search=None,
        *, approach=None, find_object=None,
    ):
        self.llm, self.vlm, self.config = llm, vlm, config
        self.navigation_preview = navigation_preview
        self.active_search = active_search
        self.approach = approach
        self.find_object = find_object

    @classmethod
    def from_config(cls, config):
        from dimos.plugins._agent_common import load_config
        from dimos.plugins.llm.client import LLMClient
        from dimos.plugins.vlm.client import VLMClient

        llm_config = load_config("llm", config["llm_config"])
        vlm_config = load_config("vlm", config["vlm_config"])
        return cls(LLMClient(llm_config), VLMClient(vlm_config), config)

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.llm.__exit__()
        self.vlm.__exit__()

    def run(self, goal, image=None, cancel=None, *, on_progress=None):
        if not isinstance(goal, str) or not goal.strip() or len(goal) > 4000:
            raise ValueError("Goal must contain 1..4000 characters")
        if image is not None:
            from dimos.plugins._agent_common import validate_image

            validate_image(image)
        cancel = cancel or threading.Event()
        started = time.perf_counter()
        deadline = time.monotonic() + self.config["timeout_s"]
        tools = list(TOOLS) if image is not None else []
        system = SYSTEM
        user_content = goal + (
            "\nAn image is attached." if image else "\nNo image is attached."
        )
        if self.navigation_preview is not None and image is None:
            from .preview import LOCATE_TOOL, PREVIEW_TOOL

            if self.navigation_preview.plan is not None:
                tools.append(PREVIEW_TOOL)
            tools.append(LOCATE_TOOL)
            system = (
                "You are a robot task assistant with a connected D455 camera. "
                "The tools retrieve the current camera frame themselves. "
                "Do not ask for an image. "
                "Copy the user's target appearance, spatial relations, landmarks, and exclusions into "
                "the target argument. Keep those details even when translating. "
                "After the tool returns, report its actual result in the user's language. "
                "The tools cannot search rooms. "
                "For unrelated text questions, answer directly."
            )
            if self.find_object is not None:
                tools.append(FIND_OBJECT_TOOL)
                system += (
                    " For an object-search task call find_object once. "
                    "It checks the current D455 view first, then searches if necessary. "
                    "For explicit preview only use preview_approach, and for only reporting "
                    "where a visible object is use locate_visible_object."
                )
            elif self.active_search is not None:
                tools.append(ACTIVE_SEARCH_TOOL)
                system += (
                    " For an object search, call active_search once, even if the user also asks "
                    "for an approach path. "
                    "It starts with the current view and waits for new viewpoints if needed. "
                    "For an approach preview limited to the current view, call preview_approach "
                    "once. For only locating an object in the current view, call locate_visible_object once."
                )
            else:
                system += (
                    " For a request to plan a path or preview an approach, "
                    "call preview_approach once if available. For a request to find an object, "
                    "call locate_visible_object once."
                )
            if self.approach is not None:
                system += (
                    " In this profile a find_object task may move the robot to search or approach. "
                    "If the user requests only a path preview or prohibits motion, use "
                    "preview_approach or locate_visible_object."
                )
            system += " Treat a requested stopping distance as a motion constraint, not part of the target description."
            user_content = goal
        messages = [
            {"role": "system", "content": system},
            {"role": "user", "content": user_content},
        ]
        steps, tool_count = [], 0
        navigation = None
        logger.info(
            "task started has_image=%s navigation_enabled=%s search_enabled=%s",
            image is not None,
            self.navigation_preview is not None and image is None,
            (self.find_object is not None or self.active_search is not None) and image is None,
        )

        def remaining():
            if cancel.is_set():
                raise TaskCancelled("Task cancelled; late results discarded")
            seconds = deadline - time.monotonic()
            if seconds <= 0:
                raise TimeoutError("Task deadline exceeded")
            return seconds

        def finish(answer):
            elapsed_ms = (time.perf_counter() - started) * 1000
            event(logger, "agent_finished", elapsed_ms=elapsed_ms, tool_calls=tool_count,
                  navigation_status=navigation.get("status") if navigation is not None else None)
            logger.info(
                "task completed elapsed_ms=%.1f tool_calls=%d navigation_status=%s",
                elapsed_ms, tool_count,
                navigation.get("status") if navigation is not None else None,
            )
            return {
                "answer": answer,
                "elapsed_ms": elapsed_ms,
                "tool_calls": tool_count,
                "steps": steps,
                **({"navigation": navigation} if navigation is not None else {}),
            }

        for step in range(self.config["max_steps"]):
            with event_context(agent_step=step + 1, purpose="agent_route" if step == 0 else "agent_followup"):
                response = self.llm.chat(
                    messages,
                    tools=tools,
                    timeout=min(remaining(), self.llm.config["timeout_s"]),
                )
            remaining()
            steps.append({"kind": "llm", **response})
            message = response["message"]
            messages.append(message)
            calls = message.get("tool_calls", [])
            if not calls:
                return finish(message["content"])
            if (
                not isinstance(calls, list)
                or len(calls) != 1
                or not isinstance(calls[0], dict)
            ):
                raise ValueError("Agent expects exactly one tool per step")
            function = calls[0].get("function", {})
            if not isinstance(function, dict):
                raise ValueError("Invalid tool function")
            args = function.get("arguments")
            tool_name = function.get("name")
            available_names = {tool["function"]["name"] for tool in tools}
            if (
                tool_name
                in ("locate_visible_object", "preview_approach", "active_search", "find_object")
                and tool_name in available_names
            ):
                if (
                    navigation is not None
                    or not isinstance(args, dict)
                    or set(args) != {"target"}
                ):
                    raise ValueError("Expected one navigation tool call with a target")
                distance = (
                    requested_standoff(goal) if tool_name != "locate_visible_object" else None
                )
                stop_options = {"stop_m": distance} if distance is not None else {}
                logger.info("tool started name=%s target=%r", tool_name, args["target"])
                event(
                    logger, "navigation_tool_started", tool=tool_name, original_goal=goal,
                    target=args["target"], requested_standoff_m=distance,
                    motion_enabled=self.approach is not None,
                )
                if distance is not None:
                    logger.info("task requested standoff_m=%.3f", distance)
                if tool_name in {"active_search", "find_object"}:
                    if tool_name == "find_object":
                        navigation = self.find_object(
                            args["target"], remaining, on_progress, cancel, **stop_options,
                        )
                    else:
                        navigation = self.active_search(
                            args["target"], remaining, on_progress, **stop_options
                        )
                    if self.approach is not None and navigation["status"] == "localized":
                        navigation = self.approach(
                            navigation, remaining, cancel, on_progress,
                            target=args["target"],
                        )
                else:
                    navigation = self.navigation_preview(
                        args["target"],
                        remaining,
                        plan_path=tool_name == "preview_approach",
                        **stop_options,
                    )
                remaining()
                logger.info(
                    "tool completed name=%s status=%s",
                    tool_name,
                    navigation.get("status"),
                )
                search = navigation.get("search")
                event(
                    logger, "navigation_tool_finished", tool=tool_name,
                    navigation_status=navigation.get("status"),
                    search_reason=search.get("reason") if isinstance(search, dict) else None,
                    timing_ms=navigation.get("timing_ms"),
                )
                steps.append({"kind": "navigation_preview", **navigation})
                tool_count += 1
                if navigation["status"] == "arrived":
                    return finish("Arrived at the approach goal.")
                if navigation["status"] == "search_exhausted" and (search or {}).get("reason") == "sweep_complete":
                    chinese = any("\u4e00" <= char <= "\u9fff" for char in goal)
                    bad_depth = any(
                        attempt["status"] == "invalid_depth" for attempt in search["attempts"]
                    )
                    if bad_depth:
                        return finish(
                            "发现匹配目标，但未能取得有效深度。" if chinese
                            else "Found a matching target, but could not obtain valid depth."
                        )
                    return finish("没有找到目标。" if chinese else "Target not found.")
                messages.append(
                    {
                        "role": "tool",
                        "tool_name": tool_name,
                        "content": json.dumps(navigation, ensure_ascii=False),
                    }
                )
                continue
            if (
                function.get("name") != "inspect_image"
                or not isinstance(args, dict)
                or set(args) != {"prompt"}
            ):
                raise ValueError("Unsupported tool or tool arguments")
            if image is None:
                raise ValueError("Visual tool needs an explicit image")
            logger.info("tool started name=inspect_image")
            observation = self.vlm.query(
                image,
                args["prompt"],
                source="task_upload",
                timeout=min(remaining(), self.vlm.config["timeout_s"]),
            )
            remaining()
            steps.append({"kind": "vlm", **observation})
            tool_count += 1
            messages.append(
                {
                    "role": "tool",
                    "tool_name": "inspect_image",
                    "content": observation["text"],
                }
            )
        logger.warning("task failed reason=max_steps tool_calls=%d", tool_count)
        raise ValueError("Agent exceeded max_steps without a final answer")
