"""Upper-layer profile composition for existing CLI and Web entry points."""

import hashlib
import json

def run_profile(name, args, parser, stack=None):
    from dimos.vln_task.profile import load_profile

    profile = load_profile(args.profile)
    motion_tracker = None
    if getattr(args, "motion_tracker_config", None):
        from dimos.plugins.path_tracker.config import load_config as load_tracker

        motion_tracker = load_tracker(args.motion_tracker_config)
        if motion_tracker["output_only"]:
            raise ValueError("--motion-tracker-config requires output_only: false")
    if args.check:
        print(json.dumps(profile, ensure_ascii=False, indent=2))
        if motion_tracker is not None:
            print(json.dumps(motion_tracker, ensure_ascii=False, indent=2))
        return 0
    if name != "web_input" and not args.goal:
        parser.error("vln_task requires --goal")
    if stack is not None:
        from dimos.plugins.runtime_log import capture

        effective_config = json.dumps(
            {"profile": profile, "motion_tracker": motion_tracker},
            sort_keys=True,
        ).encode()
        stack.enter_context(capture(
            name, ros=True,
            metadata={
                "profile": str(args.profile),
                "profile_version": profile["version"],
                "effective_config_sha256": hashlib.sha256(effective_config).hexdigest(),
                "motion_tracker_config": str(args.motion_tracker_config) if motion_tracker else None,
                "output_only": motion_tracker is None,
                "locate_only": args.locate_only,
            },
        ))

    from functools import partial

    from dimos.vln_task.assembly import create_runner

    runner = partial(create_runner, profile, locate_only=args.locate_only)
    if motion_tracker is not None:
        runner = partial(runner, motion_tracker=motion_tracker)
    if name == "web_input":
        import uvicorn

        from dimos.plugins.runtime_log import task_console
        from dimos.vln_task.web import create_app
        from dimos.plugins.web_input.session import WebSession

        web = profile["web"]
        with WebSession(), task_console():
            uvicorn.run(
                create_app(
                    web, runner, llm_preload_timeout_s=profile["llm_preload_timeout_s"]
                ),
                host=web["host"],
                port=web["port"],
                workers=1,
                timeout_graceful_shutdown=2,
            )
        return 0
    with runner() as task:
        result = task.run(args.goal, args.image.read_bytes() if args.image else None)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0
