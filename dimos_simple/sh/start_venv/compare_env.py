#!/usr/bin/env python3
"""Compare installed package versions between the new and reference venvs."""

from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
import sys

SNAPSHOT_CODE = r"""
import importlib.metadata
import json

packages = {}
for dist in importlib.metadata.distributions():
    name = dist.metadata.get("Name")
    if name:
        packages[name] = dist.version
print(json.dumps(packages))
"""


def canonicalize(name: str) -> str:
    return re.sub(r"[-_.]+", "-", name).lower()


def snapshot(python: str) -> dict[str, str]:
    env = os.environ.copy()
    # ROS setup scripts export a large PYTHONPATH. Environment parity should
    # compare packages owned by each virtualenv, not ROS overlay packages.
    env.pop("PYTHONPATH", None)
    env.pop("PYTHONHOME", None)
    env["PYTHONSAFEPATH"] = "1"
    env["PYTHONNOUSERSITE"] = "1"
    result = subprocess.run(
        [python, "-c", SNAPSHOT_CODE],
        check=True,
        capture_output=True,
        text=True,
        env=env,
        cwd="/",
    )
    raw = json.loads(result.stdout)
    return {canonicalize(name): version for name, version in raw.items()}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--current", required=True)
    parser.add_argument("--reference", required=True)
    parser.add_argument("--ignore", action="append", default=[])
    args = parser.parse_args()

    current = snapshot(args.current)
    reference = snapshot(args.reference)
    ignored = {canonicalize(name) for name in args.ignore}
    current = {name: version for name, version in current.items() if name not in ignored}
    reference = {name: version for name, version in reference.items() if name not in ignored}

    missing = {
        name: version for name, version in reference.items() if name not in current
    }
    mismatched = {
        name: (version, current[name])
        for name, version in reference.items()
        if name in current and current[name] != version
    }
    extra = {
        name: version for name, version in current.items() if name not in reference
    }

    print(f"参考环境包数: {len(reference)}")
    print(f"当前环境包数: {len(current)}")
    print(f"缺失: {len(missing)}  版本不同: {len(mismatched)}  额外: {len(extra)}")

    for name, version in sorted(missing.items()):
        print(f"  MISSING {name}=={version}")
    for name, (expected, actual) in sorted(mismatched.items()):
        print(f"  DIFF    {name}: expected {expected}, got {actual}")
    for name, version in sorted(extra.items()):
        print(f"  EXTRA   {name}=={version}")

    if missing or mismatched:
        print("包集合与参考环境不一致", file=sys.stderr)
        return 1
    print("包集合与参考环境一致")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
