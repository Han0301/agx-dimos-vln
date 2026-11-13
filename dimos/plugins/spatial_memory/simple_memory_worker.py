"""Run Simple's SpatialMemory on paired image and pose observations over stdio."""

import base64
import json
import logging
import math
import os
import sys
import time
import traceback
from pathlib import Path


def _sampling_rejection(distance, yaw_delta, interval, min_distance, min_yaw, min_interval):
    if interval is None:
        return None
    if interval < min_interval:
        return "interval_below_minimum"
    if distance < min_distance and yaw_delta <= min_yaw:
        return "pose_change_below_minimum"
    return None


def _request_log(message):
    try:
        print(f"simple_memory_request {message}", file=sys.stderr, flush=True)
    except OSError:
        pass


def _configure_embedding(spatial_perception):
    # This worker owns its interpreter. Select the model implementation before
    # Simple constructs its memory and shares the provider with Chroma.
    if os.environ.get("DIMOS_MEMORY_BACKEND", "auto").lower() == "cpu":
        preferred = None
    else:
        if __package__:
            from ._tensorrt import TensorRTSession as preferred
        else:
            from _tensorrt import TensorRTSession as preferred
    if __package__:
        from ._backend import MemorySession
    else:
        from _backend import MemorySession

    class MemoryEmbedding(spatial_perception.ImageEmbeddingProvider):
        def _initialize_model(self):
            if self.model_name != "clip":
                return super()._initialize_model()
            import onnxruntime
            from transformers import CLIPProcessor
            from dimos.utils.data import get_data

            model = get_data("models_clip") / "model.onnx"
            self.model_path = str(model)
            self.processor = CLIPProcessor.from_pretrained(str(model.parent), local_files_only=True)
            self.model = MemorySession(
                (lambda: preferred(model)) if preferred is not None else None,
                lambda: onnxruntime.InferenceSession(str(model), providers=["CPUExecutionProvider"]),
            )

    spatial_perception.ImageEmbeddingProvider = MemoryEmbedding


def _serve(send, directory, *, recording):
    if not recording:
        import structlog
        from dimos.utils import logging_config

        # Simple's logger setup opens rotating file handlers on import. This
        # worker can run without those handlers while still using Simple's APIs.
        quiet = structlog.wrap_logger(
            None, wrapper_class=structlog.make_filtering_bound_logger(logging.CRITICAL),
        )
        logging_config.setup_logger = lambda **_: quiet

    import cv2
    import numpy as np
    from reactivex.subject import Subject
    from scipy.spatial.transform import Rotation

    from dimos.msgs.geometry_msgs.Vector3 import Vector3
    from dimos.perception import spatial_perception
    from dimos.protocol.rpc.spec import RPCSpec

    class LocalRPC(RPCSpec):
        # SpatialMemory's constructor normally exposes methods over multicast.
        # Here the parent uses the same methods through the stdio connection.
        def serve_module_rpc(self, module, name=None):
            pass

        def start(self):
            pass

        def stop(self):
            pass

    _configure_embedding(spatial_perception)
    memory = spatial_perception.SpatialMemory(
        rpc_transport=LocalRPC, db_path=str(directory / "chroma"),
        visual_memory_path=None, output_dir=None,
    )
    # The worker gates submissions by position or yaw before Simple embeds them.
    memory.min_distance_threshold = 0.0
    memory.min_time_threshold = 0.0
    last_pose = None
    last_recorded_at = None
    last_by_group = {}
    source = Subject()
    sampled = []
    subscription = memory.process_stream(source).subscribe(sampled.append)
    try:
        send(ok=True, result={"backend": memory.embedding_provider.model.backend})
        for line in sys.stdin.buffer:
            request_id = None
            try:
                request = json.loads(line)
                request_id = request.get("memory_request_id") if recording else None
                if request_id:
                    _request_log(f"started id={request_id} action={request['action']}")
                if request["action"] == "record":
                    jpeg = base64.b64decode(request["jpeg"], validate=True)
                    frame = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
                    if frame is None:
                        raise ValueError("Could not decode observation JPEG")
                    pose = request["pose"]
                    rotation = Rotation.from_quat(pose[3:]).as_euler("xyz")
                    diagnostics = request.get("diagnostics", False)
                    position = pose[:3]
                    yaw = float(rotation[2])
                    min_distance = request["min_distance_m"]
                    min_yaw = request["min_yaw_delta_deg"]
                    min_interval = request["min_interval_s"]
                    group = request.get("sample_group")
                    stamp = request.get("sample_stamp_ns")
                    distance = yaw_delta = interval = None
                    if group is None and last_pose is not None:
                        previous_position, previous_yaw = last_pose
                        distance = math.dist(position, previous_position)
                        yaw_delta = abs(math.degrees(math.remainder(yaw - previous_yaw, math.tau)))
                        interval = time.monotonic() - last_recorded_at
                    elif group is not None and group in last_by_group:
                        previous_position, previous_yaw, previous_stamp = last_by_group[group]
                        distance = math.dist(position[:2], previous_position[:2])
                        yaw_delta = abs(math.degrees(math.remainder(yaw - previous_yaw, math.tau)))
                        interval = (stamp - previous_stamp) / 1e9
                    reason = _sampling_rejection(
                        distance, yaw_delta, interval, min_distance, min_yaw, min_interval,
                    )
                    stored = None
                    if reason is None:
                        recorded_at = time.monotonic()
                        source.on_next({"frame": frame, "position": Vector3(*position),
                                        "rotation": Vector3(*rotation)})
                        stored = sampled.pop() if sampled else None
                        if stored is not None:
                            last_pose = position, yaw
                            last_recorded_at = recorded_at
                            if group is not None:
                                last_by_group[group] = position, yaw, stamp
                        else:
                            reason = "sampler_rejected"
                    result = stored is not None
                    if diagnostics:
                        result = {
                            "stored": result, "memory_id": stored["frame_id"] if stored else None,
                            "image_width": frame.shape[1], "image_height": frame.shape[0],
                            "reason": reason, "distance_m": distance, "interval_s": interval,
                            "yaw_delta_deg": yaw_delta, "min_yaw_delta_deg": min_yaw,
                            "min_distance_m": min_distance, "min_interval_s": min_interval,
                        }
                elif request["action"] == "preload":
                    result = {"backend": memory.embedding_provider.model.backend}
                elif request["action"] == "query":
                    result = memory.query_by_text(request["text"])
                else:
                    raise ValueError("Unknown memory operation")
                send(ok=True, result=result)
                if request_id:
                    _request_log(f"finished id={request_id} outcome=ok")
            except Exception as exc:
                if recording:
                    traceback.print_exc()
                if request_id:
                    _request_log(f"finished id={request_id} outcome=error")
                send(ok=False, error=f"{type(exc).__name__}: {exc}")
    finally:
        subscription.dispose()
        memory.stop()


def main(directory, *, recording=False):
    with os.fdopen(os.dup(sys.stdout.fileno()), "w", encoding="utf-8", buffering=1) as protocol:
        os.dup2(sys.stderr.fileno(), sys.stdout.fileno())

        def send(**message):
            protocol.write(json.dumps(message, allow_nan=False) + "\n")

        try:
            _serve(send, Path(directory), recording=recording)
        except Exception as exc:
            if recording:
                traceback.print_exc()
            send(ok=False, error=f"{type(exc).__name__}: {exc}")
            return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1], recording="--log" in sys.argv[2:]))
