"""Independent entry points. --check parses configuration without service calls."""

import argparse
import json
import sys
from contextlib import ExitStack
from pathlib import Path

from dimos.plugins._agent_common import load_config


def main(name, argv=None):
    parser = argparse.ArgumentParser(description=f"Standalone {name} plugin")
    config = parser.add_mutually_exclusive_group()
    config.add_argument("--config")
    if name in {"vln_task", "web_input"}:
        config.add_argument("--profile", type=Path)
        parser.add_argument(
            "--locate-only", action="store_true", help="Stop after RGB-D localization"
        )
    if name == "web_input":
        parser.add_argument(
            "--motion-tracker-config",
            type=Path,
            help="Run the real tracker with a VLN profile and validated task paths",
        )
    parser.add_argument("--check", action="store_true")
    if name in {"vlm", "llm"}:
        parser.add_argument("--health", action="store_true")
        parser.add_argument("--prompt")
    if name in {"vlm", "vln_task"}:
        parser.add_argument("--image", type=Path)
    if name == "vln_task":
        parser.add_argument("--goal")
    args = parser.parse_args(argv)
    if getattr(args, "motion_tracker_config", None):
        if not args.profile:
            parser.error("--motion-tracker-config requires --profile")
        if args.locate_only:
            parser.error("--motion-tracker-config is incompatible with --locate-only")
    stack = ExitStack()
    try:
        if getattr(args, "profile", None):
            return run_profile(name, args, parser, stack if argv is None else None)
        if getattr(args, "locate_only", False):
            parser.error("--locate-only requires --profile")
        cfg = load_config(name, args.config)
        if name == "vln_task":
            load_config("llm", cfg["llm_config"])
            load_config("vlm", cfg["vlm_config"])
        if name == "web_input":
            task_cfg = load_config("vln_task", cfg["task_config"])
            load_config("llm", task_cfg["llm_config"])
            load_config("vlm", task_cfg["vlm_config"])
        if args.check:
            print(json.dumps(cfg, ensure_ascii=False, indent=2))
            return 0
        health = getattr(args, "health", False)
        if name == "vlm" and not health and (not args.image or not args.prompt):
            parser.error("vlm requires --image and --prompt, or --health")
        if name == "llm" and not health and not args.prompt:
            parser.error("llm requires --prompt, or --health")
        if name == "vln_task" and not args.goal:
            parser.error("vln_task requires --goal")
        if argv is None and not health:
            from dimos.plugins.runtime_log import capture

            stack.enter_context(capture(name, ros=False))
        if name == "web_input":
            import uvicorn

            from dimos.vln_task.web import create_app
            from dimos.plugins.web_input.session import WebSession

            with WebSession():
                uvicorn.run(
                    create_app(cfg), host=cfg["host"], port=cfg["port"], workers=1
                )
            return 0
        if name == "vlm":
            from dimos.plugins.vlm.client import VLMClient

            with VLMClient(cfg) as client:
                if args.health:
                    result = client.health()
                else:
                    result = client.query(
                        args.image.read_bytes(), args.prompt, source=str(args.image)
                    )
        elif name == "llm":
            from dimos.plugins.llm.client import LLMClient

            with LLMClient(cfg) as client:
                if args.health:
                    result = client.health()
                else:
                    result = client.chat([{"role": "user", "content": args.prompt}])
        else:
            from dimos.vln_task.agent import TaskAgent

            with TaskAgent.from_config(cfg) as agent:
                result = agent.run(
                    args.goal, args.image.read_bytes() if args.image else None
                )
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 0
    except KeyboardInterrupt:
        return 130
    except Exception as exc:
        print(f"{name}: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 1
    finally:
        stack.close()


def run_profile(name, args, parser, stack=None):
    from dimos.vln_task.cli import run_profile as run_task_profile

    return run_task_profile(name, args, parser, stack)
