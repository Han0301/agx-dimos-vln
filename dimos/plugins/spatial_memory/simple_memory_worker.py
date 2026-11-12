"""Run Simple's SpatialMemory on paired image and pose observations over stdio."""

import base64
import json
import logging
import os
import sys
import time
import traceback
from pathlib import Path


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
    from dimos.perception.spatial_perception import SpatialMemory
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

    memory = SpatialMemory(
        rpc_transport=LocalRPC, db_path=str(directory / "chroma"),
        visual_memory_path=None, output_dir=None,
    )
    source = Subject()
    sampled = []
    subscription = memory.process_stream(source).subscribe(sampled.append)
    try:
        send(ok=True)
        for line in sys.stdin.buffer:
            try:
                request = json.loads(line)
                if request["action"] == "record":
                    jpeg = base64.b64decode(request["jpeg"], validate=True)
                    frame = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
                    if frame is None:
                        raise ValueError("Could not decode observation JPEG")
                    pose = request["pose"]
                    rotation = Rotation.from_quat(pose[3:]).as_euler("xyz")
                    diagnostics = request.get("diagnostics", False)
                    if diagnostics:
                        previous = memory.last_position
                        distance = (
                            float(np.linalg.norm(np.asarray(pose[:3]) - [previous.x, previous.y, previous.z]))
                            if previous is not None else None
                        )
                        interval = (
                            time.time() - memory.last_record_time if memory.last_record_time is not None else None
                        )
                    source.on_next({"frame": frame, "position": Vector3(*pose[:3]),
                                    "rotation": Vector3(*rotation)})
                    stored = sampled.pop() if sampled else None
                    result = stored is not None
                    if diagnostics:
                        reason = None
                        if stored is None:
                            # Explain the sampler's decision without using these metrics to gate writes.
                            if distance is not None and distance < memory.min_distance_threshold:
                                reason = "distance_below_minimum"
                            elif interval is not None and interval < memory.min_time_threshold:
                                reason = "interval_below_minimum"
                            else:
                                reason = "sampler_rejected"
                        result = {
                            "stored": result, "memory_id": stored["frame_id"] if stored else None,
                            "reason": reason, "distance_m": distance, "interval_s": interval,
                            "min_distance_m": memory.min_distance_threshold,
                            "min_interval_s": memory.min_time_threshold,
                        }
                elif request["action"] == "query":
                    result = memory.query_by_text(request["text"])
                else:
                    raise ValueError("Unknown memory operation")
                send(ok=True, result=result)
            except Exception as exc:
                if recording:
                    traceback.print_exc()
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
