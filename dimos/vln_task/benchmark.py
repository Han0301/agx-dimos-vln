"""Serial, explicit-image benchmark for real model services and loopback Web API."""

import argparse
import base64
import hashlib
import json
import math
import platform
import statistics
import threading
import time
from datetime import datetime, timezone
from pathlib import Path

from dimos.plugins._agent_common import load_config


def summarize(samples):
    times = sorted(sample["elapsed_ms"] for sample in samples if sample["ok"])
    result = {
        "attempts": len(samples),
        "success": len(times),
        "semantic_pass": sum(sample["semantic_ok"] for sample in samples),
        "success_rate": len(times) / len(samples) if samples else 0,
    }
    for name in ("mean_ms", "p50_ms", "p95_ms", "max_ms", "stddev_ms"):
        result[name] = None
    if times:
        result.update(
            mean_ms=statistics.mean(times),
            p50_ms=times[math.ceil(len(times) * 0.5) - 1],
            p95_ms=times[math.ceil(len(times) * 0.95) - 1],
            max_ms=times[-1],
            stddev_ms=statistics.pstdev(times),
        )
    return result


def visual_correct(text):
    text = text.lower()
    return all(word in text for word in ("red", "blue", "left"))


def main(argv=None):
    import httpx
    import uvicorn

    from dimos.plugins.llm.client import LLMClient
    from dimos.plugins.vlm.client import VLMClient
    from dimos.vln_task.agent import TaskAgent
    from dimos.plugins.web_input.app import create_app

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--image", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--samples", type=int, default=10)
    parser.add_argument("--warmup", type=int, default=1)
    args = parser.parse_args(argv)
    if args.samples < 1 or args.warmup < 0:
        parser.error("samples must be positive and warmup nonnegative")
    image = args.image.read_bytes()
    configs = {
        name: load_config(name) for name in ("vlm", "llm", "vln_task", "web_input")
    }
    data = {
        "started_utc": datetime.now(timezone.utc).isoformat(),
        "python": platform.python_version(),
        "platform": platform.platform(),
        "concurrency": 1,
        "samples_per_suite": args.samples,
        "warmup_per_suite": args.warmup,
        "percentiles": "nearest rank, successful measured samples only",
        "fixture_sha256": hashlib.sha256(image).hexdigest(),
        "configs": configs,
        "suites": {},
        "service_metadata": {},
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)

    def save():
        temporary = args.output.with_suffix(".tmp")
        temporary.write_text(json.dumps(data, ensure_ascii=False, indent=2))
        temporary.replace(args.output)

    def measure(name, operation, verify):
        records = []
        data["suites"][name] = {"records": records}
        for index in range(args.warmup + args.samples):
            started = time.perf_counter()
            record = {
                "index": index,
                "warmup": index < args.warmup,
                "ok": False,
                "semantic_ok": False,
            }
            try:
                result = operation(index)
                record.update(ok=True, result=result, semantic_ok=bool(verify(result)))
            except Exception as exc:
                record["error"] = f"{type(exc).__name__}: {exc}"
            record["elapsed_ms"] = (time.perf_counter() - started) * 1000
            records.append(record)
            print(
                f"{name} {index}: ok={record['ok']} semantic={record['semantic_ok']} {record['elapsed_ms']:.1f} ms",
                flush=True,
            )
            measured = [sample for sample in records if not sample["warmup"]]
            data["suites"][name]["summary"] = summarize(measured)
            save()

    prompt = "Describe the two colored shapes and their left/right positions in one short English sentence."
    schema = {
        "type": "object",
        "properties": {"room": {"type": "string"}, "object": {"type": "string"}},
        "required": ["room", "object"],
        "additionalProperties": False,
    }
    text_cases = [("office", "red box"), ("kitchen", "cup"), ("bedroom", "backpack")]
    with VLMClient(configs["vlm"]) as vlm, LLMClient(configs["llm"]) as llm:
        data["service_metadata"]["vlm"] = vlm.health()
        data["service_metadata"]["llm"] = llm.health()
        save()
        measure(
            "vlm",
            lambda _: vlm.query(image, prompt, source="synthetic_shapes"),
            lambda result: visual_correct(result["text"]),
        )

        def text_request(index):
            room, obj = text_cases[index % len(text_cases)]
            result = llm.chat(
                [
                    {
                        "role": "system",
                        "content": "Extract room and object from the user's task as JSON. Use the exact English words.",
                    },
                    {"role": "user", "content": f"Find the {obj} in the {room}."},
                ],
                format=schema,
            )
            result["expected"] = {"room": room, "object": obj}
            return result

        measure(
            "llm",
            text_request,
            lambda result: (
                json.loads(result["message"]["content"]) == result["expected"]
            ),
        )
        agent = TaskAgent(llm, vlm, configs["vln_task"])
        measure(
            "vln_task",
            lambda _: agent.run(prompt, image),
            lambda result: (
                result["tool_calls"] == 1 and visual_correct(result["answer"])
            ),
        )

    server = uvicorn.Server(
        uvicorn.Config(
            create_app(configs["web_input"]),
            host="127.0.0.1",
            port=0,
            log_level="warning",
            access_log=False,
        )
    )
    thread = threading.Thread(target=server.run, name="benchmark-web-server")
    thread.start()
    try:
        deadline = time.monotonic() + 10
        while not server.started:
            if not thread.is_alive() or time.monotonic() > deadline:
                raise RuntimeError("Standalone benchmark Web server failed to start")
            time.sleep(0.01)
        port = server.servers[0].sockets[0].getsockname()[1]
        with httpx.Client(
            base_url=f"http://127.0.0.1:{port}", timeout=5, trust_env=False
        ) as web:
            data["loopback_health"] = web.get("/health").json()

            def web_request(_):
                started = time.perf_counter()
                response = web.post(
                    "/tasks",
                    json={
                        "goal": prompt,
                        "image_base64": base64.b64encode(image).decode(),
                    },
                )
                response.raise_for_status()
                task_id = response.json()["task_id"]
                accepted_ms = (time.perf_counter() - started) * 1000
                deadline = time.monotonic() + configs["vln_task"]["timeout_s"] + 10
                while time.monotonic() < deadline:
                    response = web.get(f"/tasks/{task_id}")
                    response.raise_for_status()
                    task = response.json()
                    if task["status"] == "succeeded":
                        task["accepted_ms"] = accepted_ms
                        task["queue_ms"] = (
                            task["started_at"] - task["created_at"]
                        ) * 1000
                        return task
                    if task["status"] in {"failed", "cancelled"}:
                        raise RuntimeError(task["error"] or task["status"])
                    time.sleep(0.02)
                web.post(f"/tasks/{task_id}/cancel")
                raise TimeoutError("Web task timed out")

            measure(
                "web_input",
                web_request,
                lambda task: (
                    task["result"]["tool_calls"] == 1
                    and visual_correct(task["result"]["answer"])
                ),
            )
    finally:
        server.should_exit = True
        thread.join()
    data["finished_utc"] = datetime.now(timezone.utc).isoformat()
    save()
    return (
        0
        if all(
            suite["summary"]["success"] == args.samples
            for suite in data["suites"].values()
        )
        else 1
    )


if __name__ == "__main__":
    raise SystemExit(main())
