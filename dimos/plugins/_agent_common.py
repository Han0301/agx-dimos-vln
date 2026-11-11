"""Configuration and HTTP lifecycle shared by the independent agent plugins."""

from __future__ import annotations

import io
import logging
import math
import time
from pathlib import Path
from urllib.parse import urlsplit

import yaml

ROOT = Path(__file__).resolve().parents[2]


def load_config(name, path=None):
    path = Path(path) if path else ROOT / "config/plugins" / name / f"{name}.yaml"
    with path.open() as handle:
        cfg = yaml.safe_load(handle)
    required = {
        "vlm": {"base_url", "timeout_s", "max_new_tokens", "temperature", "top_p"},
        "llm": {
            "base_url",
            "timeout_s",
            "model",
            "num_predict",
            "temperature",
            "think",
        },
        "vln_task": {"llm_config", "vlm_config", "max_steps", "timeout_s"},
        "web_input": {
            "task_config",
            "host",
            "port",
            "queue_size",
            "max_records",
            "max_body_bytes",
        },
    }[name]
    if name == "vlm" and isinstance(cfg, dict):
        backend = cfg.get("backend", "http")
        if backend == "dashscope":
            required = {"backend", "credentials_file", "timeout_s"}
        elif backend != "http":
            raise ValueError("vlm backend must be http or dashscope")
        elif "backend" in cfg:
            required.add("backend")
    if name == "llm" and isinstance(cfg, dict) and "keep_alive_s" in cfg:
        required.add("keep_alive_s")
    if (
        not isinstance(cfg, dict)
        or set(cfg) != required | {"plugin"}
        or cfg["plugin"] != name
    ):
        raise ValueError(f"Invalid {name} config fields: expected {sorted(required)}")
    for key in required:
        value = cfg[key]
        if key == "credentials_file":
            if not isinstance(value, str) or not value.strip():
                raise ValueError("credentials_file must be a nonempty path")
            cfg[key] = str(ROOT / Path(value).expanduser())
        elif key.endswith("_config"):
            target = Path(value)
            cfg[key] = str(
                (ROOT / target).resolve() if not target.is_absolute() else target
            )
        elif key in {"timeout_s", "temperature", "top_p"}:
            if type(value) not in (int, float) or not math.isfinite(value):
                raise ValueError(f"Invalid numeric {key}")
            if value < 0 or (key != "temperature" and value == 0):
                raise ValueError(f"Invalid range for {key}")
            if key == "top_p" and value > 1:
                raise ValueError("top_p must be in (0, 1]")
        elif key in {
            "max_new_tokens",
            "num_predict",
            "max_steps",
            "port",
            "queue_size",
            "max_records",
            "max_body_bytes",
            "keep_alive_s",
        }:
            if type(value) is not int or value <= 0:
                raise ValueError(f"{key} must be a positive integer")
            if key == "port" and value > 65535:
                raise ValueError("port exceeds 65535")
        elif key == "think":
            if type(value) is not bool:
                raise ValueError("think must be boolean")
        elif not isinstance(value, str) or not value.strip():
            raise ValueError(f"{key} must be a nonempty string")
    if "base_url" in cfg:
        url = urlsplit(cfg["base_url"])
        if (
            url.scheme not in {"http", "https"}
            or not url.hostname
            or url.query
            or url.fragment
            or url.username
        ):
            raise ValueError("base_url must be an HTTP(S) service URL")
        cfg["base_url"] = cfg["base_url"].rstrip("/")
    return cfg


def validate_image(data):
    from PIL import Image, UnidentifiedImageError

    if not isinstance(data, bytes) or not data or len(data) > 5 * 1024 * 1024:
        raise ValueError("Image must contain 1..5242880 bytes")
    try:
        with Image.open(io.BytesIO(data)) as image:
            if image.format not in {"JPEG", "PNG"} or max(image.size) > 4096:
                raise ValueError("Use JPEG/PNG with each side <= 4096")
            size = list(image.size)
            image.verify()
    except (OSError, UnidentifiedImageError, Image.DecompressionBombError) as exc:
        raise ValueError("Invalid image") from exc
    return size


class HTTPModel:
    def __init__(self, config, http=None):
        import httpx

        self.config = config
        self.http = http or httpx.Client(
            timeout=httpx.Timeout(
                config["timeout_s"], connect=min(5.0, config["timeout_s"])
            ),
            trust_env=False,
        )
        self.owns_http = http is None

    def __enter__(self):
        return self

    def __exit__(self, *_):
        if self.owns_http:
            self.http.close()

    def post(self, path, payload, timeout=None):
        from httpx import HTTPError, HTTPStatusError

        started = time.perf_counter()
        try:
            response = self.http.post(
                self.config["base_url"] + path,
                json=payload,
                timeout=timeout or self.config["timeout_s"],
            )
            response.raise_for_status()
            result = response.json()
            if not isinstance(result, dict):
                raise ValueError("Model response must be an object")
            return result
        except (HTTPError, ValueError) as exc:
            logging.getLogger(type(self).__module__).warning(
                "provider request failed endpoint=%s error_type=%s http_status=%s elapsed_ms=%.1f",
                path,
                type(exc).__name__,
                exc.response.status_code if isinstance(exc, HTTPStatusError) else None,
                (time.perf_counter() - started) * 1000,
            )
            raise

    def get(self, path):
        response = self.http.get(self.config["base_url"] + path)
        response.raise_for_status()
        return response.json()
