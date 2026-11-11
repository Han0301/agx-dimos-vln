"""Independent XNavDP health / point-goal communication test."""

import argparse
import json
import logging
import math
import os
import xml.etree.ElementTree as ET
from contextlib import ExitStack
from pathlib import Path

import yaml

from dimos.config import ROOT

from .client import XNavDPClient, stamp_ns, validate_result


def load_test_config(path, mode=None, requests=None, reset_if_needed=None):
    with Path(path).open() as stream:
        cfg = yaml.safe_load(stream)
    if not isinstance(cfg, dict) or set(cfg) != {
        "plugin",
        "mode",
        "test",
        "goal",
        "initialization",
    }:
        raise ValueError(
            "expected plugin, mode, test, goal and initialization sections"
        )
    if cfg["plugin"] != "xnavdp":
        raise ValueError("plugin must be xnavdp")
    if mode is not None:
        cfg["mode"] = mode
    if cfg["mode"] not in {"health_only", "inference"}:
        raise ValueError("unknown test mode")
    expected = {
        "requests",
        "response_timeout_s",
        "sensor_timeout_s",
        "service_timeout_s",
    }
    if not isinstance(cfg["test"], dict) or set(cfg["test"]) != expected:
        raise ValueError(f"test keys must be {sorted(expected)}")
    if requests is not None:
        cfg["test"]["requests"] = requests
    if type(cfg["test"]["requests"]) is not int or cfg["test"]["requests"] < 1:
        raise ValueError("requests must be a positive integer")
    for key in expected - {"requests"}:
        value = cfg["test"][key]
        if (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(value)
            or value <= 0
        ):
            raise ValueError(f"{key} must be positive and finite")
    goal = cfg["goal"]
    if (
        not isinstance(goal, dict)
        or set(goal) != {"mode", "xy"}
        or goal["mode"] != "relative"
    ):
        raise ValueError("first version supports goal.mode: relative only")
    if (
        not isinstance(goal["xy"], list)
        or len(goal["xy"]) != 2
        or not all(
            not isinstance(v, bool) and isinstance(v, (int, float)) and math.isfinite(v)
            for v in goal["xy"]
        )
    ):
        raise ValueError("goal.xy must contain two finite numbers")
    init = cfg["initialization"]
    if (
        not isinstance(init, dict)
        or set(init) != {"reset_if_needed"}
        or type(init["reset_if_needed"]) is not bool
    ):
        raise ValueError("initialization.reset_if_needed must be boolean")
    if reset_if_needed is not None:
        if type(reset_if_needed) is not bool:
            raise ValueError("reset_if_needed override must be boolean")
        init["reset_if_needed"] = reset_if_needed
    return cfg


def dds_peer():
    """Read the existing DDS XML rather than maintaining a second endpoint setting."""
    path = Path(
        os.environ.get("XNAVDP_DDS_PROFILE", ROOT / "sh/xnavdp_ros2/client_dds.xml")
    )
    ns = {"p": "http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles"}
    tree = ET.parse(path)
    peers = []
    for peer in tree.findall(".//p:initialPeersList/p:locator/p:tcpv4", ns):
        address = peer.findtext("p:address", namespaces=ns)
        port = peer.findtext("p:physical_port", namespaces=ns)
        if address and port:
            peers.append((address, int(port)))
    if len(peers) != 1 or peers[0][1] != 7411:
        raise ValueError("XNavDP must use the existing DDS TCP peer on port 7411")
    return f"{peers[0][0]}:{peers[0][1]}"


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config", default=str(ROOT / "config/plugins/xnavdp/xnavdp_test.yaml")
    )
    parser.add_argument("--mode", choices=["health_only", "inference"])
    parser.add_argument("--requests", type=int)
    parser.add_argument(
        "--reset-if-needed",
        action="store_true",
        default=None,
        help="explicitly allow initialization only if the model is not initialized",
    )
    parser.add_argument(
        "--check",
        action="store_true",
        help="validate config and DDS XML without creating ROS resources",
    )
    args = parser.parse_args(argv)
    stack = ExitStack()
    try:
        cfg = load_test_config(
            args.config, args.mode, args.requests, args.reset_if_needed
        )
        peer = dds_peer()
        if args.check:
            print(
                json.dumps(
                    {
                        "plugin": "xnavdp",
                        "mode": cfg["mode"],
                        "dds_tcp_peer": peer,
                        "requests": cfg["test"]["requests"],
                        "result_topics": ["/xnavdp/trajectory", "/xnavdp/result"],
                        "goal": cfg["goal"],
                        "reset_if_needed": cfg["initialization"]["reset_if_needed"],
                    },
                    ensure_ascii=False,
                    indent=2,
                )
            )
            return 0
        if argv is None and cfg["mode"] == "inference":
            from dimos.plugins.runtime_log import capture

            stack.enter_context(capture("xnavdp", ros=True))
        logging.basicConfig(
            level=logging.WARNING,
            format="%(asctime)s %(levelname)s %(name)s: %(message)s",
        )
        logging.getLogger("dimos").setLevel(logging.INFO)
        if os.environ.get("RMW_IMPLEMENTATION") != "rmw_fastrtps_cpp":
            raise RuntimeError("Source sh/env.sh before starting the XNavDP plugin")
        if cfg["mode"] == "inference":
            from .session import InferenceSession

            session = InferenceSession()
            stack.callback(session.__exit__)
            session.__enter__()
        print(
            f"XNavDP communication test: DDS TCP peer={peer}, mode={cfg['mode']}",
            flush=True,
        )
        client = XNavDPClient(cfg["test"]).start()
        stack.callback(client.stop)
        health = client.health()
        print(
            json.dumps(
                {
                    "health": "OK",
                    "release": health["release"],
                    "initialized": health.get("initialized"),
                    "calibrated": health.get("calibrated"),
                    "busy": health.get("busy"),
                    "buffers": health.get("buffers"),
                    "last_arrival_age_ms": health.get("last_arrival_age_ms"),
                },
                ensure_ascii=False,
            ),
            flush=True,
        )
        if cfg["mode"] == "health_only":
            return 0
        client.prepare(cfg["initialization"]["reset_if_needed"])
        client.open_requests()
        for index in range(cfg["test"]["requests"]):
            goal = client.next_goal(cfg["goal"]["xy"])
            parts, elapsed = client.infer(goal)
            details = validate_result(parts, goal)
            print(
                json.dumps(
                    {
                        "request": index + 1,
                        "stamp_ns": stamp_ns(goal.header),
                        "frame": goal.header.frame_id,
                        "world_goal_xy": [goal.pose.position.x, goal.pose.position.y],
                        "roundtrip_ms": round(elapsed * 1000, 1),
                        "path_points": len(parts["path"].poses),
                        "candidates": len(details["all_trajectory"][0]),
                        "trajectory_frame": details["trajectory_frame"],
                        "sensor_gaps_ms": {
                            k: v for k, v in details.items() if k.endswith("_gap_ms")
                        },
                        "status": "OK",
                    },
                    ensure_ascii=False,
                ),
                flush=True,
            )
        return 0
    except KeyboardInterrupt:
        print("XNavDP test stopped; base/camera/server were not stopped.", flush=True)
        return 130
    except (
        ValueError,
        KeyError,
        TypeError,
        OSError,
        RuntimeError,
        ImportError,
        ET.ParseError,
        yaml.YAMLError,
    ) as exc:
        logging.getLogger("dimos.xnavdp").error("%s", exc)
        return 1
    finally:
        stack.close()


if __name__ == "__main__":
    raise SystemExit(main())
