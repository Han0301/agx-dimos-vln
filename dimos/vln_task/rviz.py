"""Launch the exploration-memory RViz profile and existing XNavDP display helpers."""

import argparse
import logging
import shlex

from dimos.core.config import ROOT
from dimos.plugins.runtime_log import capture, event
from dimos.plugins.xnavdp.visualization import __main__ as xnavdp_rviz

logger = logging.getLogger(__name__)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--time-source", choices=("auto", "sim", "wall"), default="auto")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args(argv)
    config = ROOT / "config/plugins/spatial_memory/exploration_memory.rviz"

    if args.dry_run:
        use_sim = (
            "<detect /clock>" if args.time_source == "auto"
            else str(args.time_source == "sim").lower()
        )
        for command in xnavdp_rviz._commands(config, use_sim):
            print(shlex.join(command))
        return 0

    metadata = {"time_source": args.time_source, "rviz_config": str(config)}
    with capture("rviz", ros=True, metadata=metadata):
        try:
            use_sim = args.time_source == "sim" or (
                args.time_source == "auto" and xnavdp_rviz.bag_clock_present()
            )
            event(logger, "rviz_clock_selected", time_source=args.time_source, use_sim_time=use_sim)
            return xnavdp_rviz._launch(xnavdp_rviz._commands(config, str(use_sim).lower()))
        except Exception as exc:
            event(logger, "rviz_launch_failed", error_type=type(exc).__name__, reason=str(exc))
            raise


if __name__ == "__main__":
    raise SystemExit(main())
