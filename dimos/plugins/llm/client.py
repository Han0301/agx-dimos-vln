"""Ollama chat protocol used by simple's primary agent."""

import copy
import logging
import time
from uuid import uuid4

from httpx import HTTPError, HTTPStatusError

from dimos.plugins._agent_common import HTTPModel
from dimos.plugins.runtime_log import event, recording_enabled

logger = logging.getLogger(__name__)


class LLMClient(HTTPModel):
    def preload(self, *, timeout=None):
        """Ask Ollama to load the configured model without generating tokens."""
        payload = {"model": self.config["model"], "messages": [], "stream": False}
        if "keep_alive_s" in self.config:
            payload["keep_alive"] = self.config["keep_alive_s"]
        started = time.perf_counter()
        try:
            result = self.post("/api/chat", payload, timeout)
        except (HTTPError, ValueError) as exc:
            event(logger, "llm_preload_finished", status="failed", model=payload["model"],
                  elapsed_ms=(time.perf_counter() - started) * 1000, error_type=type(exc).__name__)
            raise
        if result.get("error") or result.get("done") is not True:
            event(logger, "llm_preload_finished", status="failed", model=payload["model"],
                  elapsed_ms=(time.perf_counter() - started) * 1000, reason="incomplete_response")
            logger.warning(
                "preload failed model=%s reason=incomplete_response",
                self.config["model"],
            )
            raise ValueError("Ollama did not complete model preload")
        load_ns = result.get("load_duration")
        elapsed_ms = (time.perf_counter() - started) * 1000
        load_ms = load_ns / 1e6 if type(load_ns) in (int, float) else None
        logger.info(
            "preload completed model=%s elapsed_ms=%.1f load_ms=%s",
            self.config["model"],
            elapsed_ms,
            load_ms,
        )
        event(logger, "llm_preload_finished", status="succeeded",
              model=result.get("model", payload["model"]), elapsed_ms=elapsed_ms, load_ms=load_ms)
        return {
            "model": result.get("model", self.config["model"]),
            "elapsed_ms": elapsed_ms,
            "load_ms": load_ms,
        }

    def chat(self, messages, *, tools=None, format=None, timeout=None):
        if not isinstance(messages, list) or not messages:
            raise ValueError("messages must be a nonempty list")
        for message in messages:
            if not isinstance(message, dict) or message.get("role") not in {
                "system",
                "user",
                "assistant",
                "tool",
            }:
                raise ValueError("Invalid chat message role")
            if not isinstance(message.get("content"), str):
                raise ValueError("Chat message content must be text")
        payload = {
            "model": self.config["model"],
            "messages": copy.deepcopy(messages),
            "stream": False,
            "think": self.config["think"],
            "options": {
                "temperature": self.config["temperature"],
                "num_predict": self.config["num_predict"],
            },
        }
        if tools:
            payload["tools"] = tools
        if format is not None:
            payload["format"] = format
        if "keep_alive_s" in self.config:
            payload["keep_alive"] = self.config["keep_alive_s"]
        started = time.perf_counter()
        call_id = uuid4().hex if recording_enabled() else None

        def failed(reason, exc=None):
            if call_id is not None:
                event(logger, "llm_call_finished", call_id=call_id, status="failed",
                      model=payload["model"], reason=reason,
                      error_type=type(exc).__name__ if exc is not None else None,
                      http_status=exc.response.status_code if isinstance(exc, HTTPStatusError) else None,
                      elapsed_ms=(time.perf_counter() - started) * 1000)

        try:
            result = self.post("/api/chat", payload, timeout)
        except (HTTPError, ValueError) as exc:
            failed("provider_request", exc)
            raise
        message = result.get("message")
        if (
            result.get("error")
            or result.get("done") is not True
            or not isinstance(message, dict)
        ):
            logger.warning(
                "chat failed model=%s reason=incomplete_response", self.config["model"]
            )
            failed("incomplete_response")
            raise ValueError("LLM returned an incomplete/error response")
        if message.get("role") != "assistant" or not isinstance(
            message.get("content"), str
        ):
            logger.warning(
                "chat failed model=%s reason=invalid_message", self.config["model"]
            )
            failed("invalid_message")
            raise ValueError("LLM returned an invalid assistant message")
        if not message["content"].strip() and not message.get("tool_calls"):
            logger.warning(
                "chat failed model=%s reason=empty_response", self.config["model"]
            )
            failed("empty_response")
            raise ValueError("LLM returned an empty response")
        duration = result.get("eval_duration", 0)
        elapsed_ms = (time.perf_counter() - started) * 1000
        load_ms = result.get("load_duration", 0) / 1e6
        logger.info(
            "chat completed model=%s elapsed_ms=%.1f load_ms=%.1f tool_call=%s",
            self.config["model"],
            elapsed_ms,
            load_ms,
            bool(message.get("tool_calls")),
        )
        if call_id is not None:
            stages = {
                key.removesuffix("_duration"): result[key] / 1e6
                for key in ("total_duration", "load_duration", "prompt_eval_duration", "eval_duration")
                if type(result.get(key)) in (int, float)
            }
            event(logger, "llm_call_finished", call_id=call_id, status="succeeded",
                  model=result.get("model", payload["model"]), elapsed_ms=elapsed_ms,
                  stages_ms=stages, prompt_eval_count=result.get("prompt_eval_count"),
                  eval_count=result.get("eval_count"), tool_call=bool(message.get("tool_calls")))
        return {
            "message": message,
            "elapsed_ms": elapsed_ms,
            "model": result.get("model", self.config["model"]),
            "eval_count": result.get("eval_count"),
            "tokens_per_s": result.get("eval_count", 0) * 1e9 / duration
            if duration > 0
            else None,
            "load_ms": load_ms,
        }

    def health(self):
        return self.get("/api/tags")
