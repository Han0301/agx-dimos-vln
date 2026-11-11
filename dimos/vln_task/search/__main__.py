import argparse
import json
from pathlib import Path

from .config import DEFAULT_CONFIG, load_config


def main():
    parser = argparse.ArgumentParser(description="Check active_search configuration")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--check", required=True, action="store_true")
    args = parser.parse_args()
    try:
        settings = load_config(args.config)
    except (OSError, ValueError) as exc:
        parser.error(str(exc))
    print(json.dumps(settings, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
