"""Exercise multipart reference-assisted search with offline model and camera inputs."""

import asyncio
import base64
import io
import json
from types import SimpleNamespace

import httpx
import numpy as np
import pytest
import websocket
from PIL import Image

from dimos.plugins._agent_common import load_config
from dimos.plugins.llm.client import LLMClient
from dimos.plugins.vlm.client import VLMClient
from dimos.vln_task.agent import TaskAgent
from dimos.vln_task.find_object import FindObject
from dimos.vln_task.preview import NavigationPreview
from dimos.vln_task.web import create_app


def jpeg(color):
    output = io.BytesIO()
    Image.new("RGB", (100, 100), color).save(output, "JPEG")
    return output.getvalue()


@pytest.mark.parametrize(
    ("backend", "found"),
    [
        pytest.param("dashscope", True, id="dashscope-two-step-found"),
        pytest.param("dashscope", False, id="dashscope-two-step-not-visible"),
        pytest.param("http", True, id="http-found"),
        pytest.param("http", False, id="http-reference-unidentified"),
    ],
)
def test_web_reference_image_guides_current_camera_search(backend, found, monkeypatch, tmp_path):
    target = "mug left of the kettle, exclude the plastic cup"
    goal = f"Find the {target}. Stop 1.2 m away."
    appearance = "cobalt glaze with a white crescent on the side"
    reference = jpeg("blue")
    if backend == "dashscope":
        large = io.BytesIO()
        Image.new("RGB", (1600, 1200), "blue").save(large, "JPEG")
        reference = large.getvalue()
    frame = SimpleNamespace(
        jpeg=jpeg("gray"), stamp_ns=1234, wait_ms=0, depth_gap_ms=0, odom_gap_ms=0,
        depth=np.pad(np.full((30, 30), 2.0), ((25, 45), (55, 15))),
        k=[100., 0., 50., 0., 100., 50., 0., 0., 1.],
        distortion_model="", distortion=(),
        odom=SimpleNamespace(
            header=SimpleNamespace(frame_id="world"),
            pose=SimpleNamespace(
                position=SimpleNamespace(x=0., y=0., z=0.),
                orientation=SimpleNamespace(x=0., y=0., z=0., w=1.),
            ),
        ),
    )
    bbox = [600, 300, 800, 500]
    visual_inputs = []
    observations = []
    explorations = []
    connections = []
    reference_tasks = []
    detection = json.dumps({"name": "mug", "bbox": bbox}) if found else "None"

    def model(request):
        body = json.loads(request.content)
        if request.url.path == "/api/chat":
            if body["messages"][-1]["role"] == "tool":
                expected = "localized" if found else "not_visible"
                assert json.loads(body["messages"][-1]["content"])["status"] == expected
                message = {"role": "assistant", "content": f"Target is {expected}."}
            else:
                assert goal in body["messages"][-1]["content"]
                assert "find_object" in {tool["function"]["name"] for tool in body["tools"]}, (
                    "Uploading a reference image must still expose the find_object tool"
                )
                message = {
                    "role": "assistant",
                    "content": "",
                    "tool_calls": [{
                        "function": {"name": "find_object", "arguments": {"target": target}},
                    }],
                }
            return httpx.Response(200, json={"done": True, "message": message})

        assert request.url.path == "/v1/image-inference-base64"
        image = base64.b64decode(body["image_base64"])
        visual_inputs.append(image)
        if image == reference:
            assert "mug" in body["prompt"]
            response = appearance if found else "UNIDENTIFIED"
        else:
            assert image == frame.jpeg
            assert "left of the kettle" in body["prompt"]
            assert "exclude the plastic cup" in body["prompt"]
            assert appearance in body["prompt"]
            response = detection
        return httpx.Response(200, json={"status": "success", "response": response})

    class RealtimeSocket:
        def __init__(self, reply):
            self.sent = []
            self.events = iter([
                {"type": "session.created"},
                {"type": "session.updated"},
                {"type": "input_audio_buffer.committed"},
                {"type": "response.text.delta", "delta": reply},
                {"type": "response.done", "response": {"status": "completed"}},
            ])

        def send(self, message):
            self.sent.append(json.loads(message))

        def recv(self):
            return json.dumps(next(self.events))

        def settimeout(self, timeout):
            pass

        def close(self, *, timeout):
            pass

    def connect(url, **kwargs):
        assert url == "wss://vlm.test/realtime"
        socket = RealtimeSocket(appearance if not connections else detection)
        connections.append(socket)
        return socket

    def explore(description, remaining, progress, cancel, seed_frame, seed, *, memory_target=None):
        assert seed_frame is frame
        assert memory_target == target
        assert appearance in description
        explorations.append(seed["status"])
        return seed

    def observe(remaining):
        observations.append(frame.stamp_ns)
        return frame

    async def submit():
        # Timer also wakes selectors in test sandboxes without thread wakeup sockets.
        loop = asyncio.get_running_loop()

        def wake_selector():
            loop.call_later(0.02, wake_selector)

        wake_selector()
        llm_config = {**load_config("llm"), "base_url": "http://llm.test"}
        with httpx.Client(transport=httpx.MockTransport(model)) as provider:
            if backend == "dashscope":
                monkeypatch.delenv("DASHSCOPE_API_KEY", raising=False)
                credentials = tmp_path / "vlm.yaml"
                credentials.write_text(
                    "api_key: fixture-key\nmodel: fixture-model\n"
                    "websocket_url: wss://vlm.test/realtime\n",
                    encoding="utf-8",
                )
                monkeypatch.setattr(websocket, "create_connection", connect)
                vlm = VLMClient({
                    "backend": "dashscope", "credentials_file": str(credentials), "timeout_s": 5,
                })
            else:
                vlm = VLMClient({
                    "backend": "http", "base_url": "http://vlm.test", "timeout_s": 2,
                    "max_new_tokens": 256, "temperature": 0.1, "top_p": 0.9,
                }, provider)
            preview = NavigationPreview(
                vlm, observe, None, None,
                {"translation": [0., 0., 0.], "pitch": 0.}, 0.8,
            )
            find_object = FindObject(
                preview.snapshot, preview.detect, explore,
                lambda observation, result, remaining: result,
            )
            agent = TaskAgent(
                LLMClient(llm_config, provider), vlm, load_config("vln_task"), preview,
                find_object=find_object,
            )
            run_with_reference = agent.run_with_reference

            def run_reference(goal, image, **kwargs):
                reference_tasks.append((goal, image))
                return run_with_reference(goal, image, **kwargs)

            monkeypatch.setattr(agent, "run_with_reference", run_reference)
            app = create_app(load_config("web_input"), lambda: agent)
            async with app.router.lifespan_context(app):
                async with httpx.AsyncClient(
                    transport=httpx.ASGITransport(app=app), base_url="http://web.test",
                ) as web:
                    submission = await web.post("/navigate", files={
                        "goal": (None, goal), "image": ("ref.jpg", reference, "image/jpeg"),
                    })
                    submission.raise_for_status()
                    task_id = submission.json()["task_id"]
                    response = await web.get(f"/tasks/{task_id}", params={"wait_s": 8})
                    response.raise_for_status()
                    return response.json()

    task = asyncio.run(submit())
    assert reference_tasks == [(goal, reference)]
    if backend == "http" and not found:
        assert task["status"] == "failed"
        assert "identify the target in the reference image" in task["error"]
        assert visual_inputs == [reference]
        assert observations == []
        return

    assert task["status"] == "succeeded", task["error"]
    assert observations == [frame.stamp_ns]
    if backend == "dashscope":
        assert len(connections) == 2
        prompts = []
        for socket, rgb in zip(connections, [(0, 0, 255), (128, 128, 128)], strict=True):
            kinds = [event["type"] for event in socket.sent]
            assert kinds.count("input_image_buffer.append") == 1
            assert kinds.count("response.create") == 1
            sent_image = socket.sent[kinds.index("input_image_buffer.append")]
            image = Image.open(io.BytesIO(base64.b64decode(sent_image["image"])))
            assert image.getpixel((50, 50)) == pytest.approx(rgb, abs=3)
            assert min(image.size) <= 1080
            prompts.append(socket.sent[0]["session"]["instructions"])
        assert "reference image" in prompts[0]
        assert "left of the kettle" in prompts[1] and "exclude the plastic cup" in prompts[1]
        assert appearance in prompts[1]
        assert '"bbox"' in prompts[1] and "1000" in prompts[1]
    else:
        assert visual_inputs == [reference, frame.jpeg]
    result = task["result"]
    assert result["navigation"]["observation"]["stamp_ns"] == frame.stamp_ns
    if found:
        assert result["answer"] == "Target is localized."
        assert result["navigation"]["status"] == "localized"
        assert result["navigation"]["bbox_2d"] == bbox
        assert result["navigation"]["target_xyz"] == pytest.approx([2.0, -0.4, 0.2])
        assert result["navigation"]["standoff_m"] == 1.2
        assert explorations == []
    else:
        assert result["navigation"]["status"] == "not_visible"
        assert "target_xyz" not in result["navigation"]
        assert "bbox_2d" not in result["navigation"]
        assert explorations == ["not_visible"]
