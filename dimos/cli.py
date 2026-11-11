import argparse
import json
import logging
import math
from contextlib import ExitStack

from dimos.config import ROOT, load_config


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="DimOS minimal: native ROS communication only by default"
    )
    parser.add_argument("--config", default=str(ROOT / "config/base.yaml"))
    parser.add_argument(
        "--check",
        action="store_true",
        help="validate config and print selection; no ROS or hardware activity",
    )
    parser.add_argument(
        "--duration", type=float, help="stop after this many seconds (for smoke checks)"
    )
    args = parser.parse_args(argv)
    if args.duration is not None and (
        not math.isfinite(args.duration) or args.duration <= 0
    ):
        parser.error("--duration must be positive")
    stack = ExitStack()
    try:
        config = load_config(args.config)
        if args.check:
            print(
                json.dumps(
                    {
                        "modules": config["modules"],
                        "topics": config["topics"],
                        "motion_enabled": config["body"]["allow_motion"],
                    },
                    ensure_ascii=False,
                    indent=2,
                )
            )
            return 0
        if argv is None:
            from dimos.plugins.runtime_log import capture

            module = "body" if config["modules"]["body"] else "dimos"
            stack.enter_context(capture(module, ros=True))
        logging.basicConfig(
            level=logging.WARNING,
            format="%(asctime)s %(levelname)s %(name)s: %(message)s",
        )
        # The SDK logs every raw packet through the root logger at INFO.
        # Keep application lifecycle messages without enabling packet dumps.
        logging.getLogger("dimos").setLevel(logging.INFO)
        from dimos.runtime import run

        run(config, max_seconds=args.duration)
        return 0
    except (
        ValueError,
        KeyError,
        TypeError,
        FileNotFoundError,
        ImportError,
        RuntimeError,
        OSError,
    ) as exc:
        logging.getLogger("dimos").error("%s", exc)
        return 1
    finally:
        stack.close()
