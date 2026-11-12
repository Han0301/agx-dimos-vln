import argparse

from . import DEFAULT_EVIDENCE, MODULE_NAME, create_run, log_root, module_directory


def main():
    parser = argparse.ArgumentParser(description="DimOS runtime log directories")
    parser.add_argument("module", help="record, or a module name with --latest")
    parser.add_argument("--root", help="override the log root")
    parser.add_argument("--mode", choices=("live", "bag"), help="label the recording")
    parser.add_argument(
        "--evidence", choices=("off", "failures"), default=DEFAULT_EVIDENCE,
        help="save bounded evidence from failed or uncertain detections (default: %(default)s)",
    )
    parser.add_argument("--metadata", action="append", default=[], help=argparse.SUPPRESS)
    parser.add_argument("--pid", type=int, help=argparse.SUPPRESS)
    action = parser.add_mutually_exclusive_group()
    action.add_argument("--latest", action="store_true", help="print the latest run")
    action.add_argument("--create", action="store_true", help=argparse.SUPPRESS)
    args = parser.parse_args()
    try:
        if args.module == "record" and not (args.create or args.latest):
            from .recorder import record

            return record(log_root(args.root), mode=args.mode, evidence=args.evidence)
        if args.create:
            metadata = {}
            for item in args.metadata:
                key, separator, value = item.partition("=")
                if not separator or not MODULE_NAME.fullmatch(key) or len(value) > 4096:
                    raise ValueError("Expected log metadata key=value")
                metadata[key] = value
            run = create_run(
                args.module, base=args.root, ros=True, pid=args.pid, metadata=metadata,
            )
        elif args.latest:
            base = log_root(args.root)
            directories = [module_directory(args.module, base)]
            directories.extend(
                module_directory(args.module, session)
                for session in base.glob("session_*")
                if session.is_dir()
            )
            runs = (
                path
                for directory in directories
                if directory.is_dir()
                for path in directory.iterdir()
                if path.is_dir()
            )
            run = max(runs, key=lambda path: path.name, default=None)
            if run is None:
                raise ValueError(f"No recorded runs for {args.module}")
        else:
            parser.error("Use 'record' to start recording, or MODULE --latest")
    except (OSError, ValueError, RuntimeError) as exc:
        parser.exit(1, f"Runtime logging: {exc}\n")
    if run is not None:
        print(run)


if __name__ == "__main__":
    raise SystemExit(main())
