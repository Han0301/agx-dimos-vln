#!/usr/bin/env python3
"""Direct model worker for Python 3.10; binary stdio, no HTTP server/client."""
import contextlib
import os
import sys
import traceback
from local_model_protocol import pack, unpack, read_frame, write_frame

output = sys.stdout.buffer
sys.stdout = sys.stderr
import numpy as np
import cv2
import torch
sys.path.insert(0, os.environ['MODEL_SOURCE'])
sys.path.insert(0, os.environ['XNAVDP_MODEL_ROOT'])
from eval.src.policy_agent import NavDP_Agent
from xnavdp_cuda_graph import install
from async_plotter import AsyncPlotter
import traceutil as tr
tr.init("server_ros2")
from stuck_audit import install as install_audit
install_audit(NavDP_Agent)
torch.set_num_threads(1)
plotter = AsyncPlotter()
agent = None
owner = None
last_access = 0.0
import time
try:
    while True:
        try:
            metadata, arrays = unpack(read_frame(sys.stdin.buffer))
        except EOFError:
            break
        rid = metadata.get('id')
        try:
            tr.local.rid = rid
            operation = metadata['op']
            if operation == 'health':
                reply = pack({'id': rid, 'ok': True, 'health': {'service': 'xnavdp', 'release': 'unified-20261104', 'transport': 'xnavdp-ros2-v1', 'initialized': agent is not None, 'cpu_threads': torch.get_num_threads(), 'cuda_graph_configured': True}})
            elif operation == 'release':
                if owner == metadata['owner']:
                    owner = None
                reply = pack({'id': rid, 'ok': True})
            else:
                if owner not in (None, metadata['owner']) and time.monotonic() - last_access < 180:
                    raise RuntimeError('XNavDP is owned by another client; stop it before resetting')
                if operation == 'reset':
                    intrinsic = np.asarray(arrays['intrinsic'], dtype=float)
                    if intrinsic.shape != (3,3) or not np.isfinite(intrinsic).all():
                        raise ValueError('Invalid camera intrinsic')
                    if agent is None:
                        agent = NavDP_Agent(intrinsic, image_size=224, memory_size=8, predict_size=24, temporal_depth=16, heads=8, token_dim=384, navi_model=os.environ['CHECKPOINT'], device='cuda:0', embodiment=2, is_real=True)
                        install(agent.navi_former)
                        agent._plot_submit = plotter.submit
                    else:
                        # Same intrinsic is required for this model instance.
                        if not np.allclose(intrinsic, previous_intrinsic):
                            raise ValueError('Camera calibration changed; restart ROS2 model service')
                    agent.reset(np.array(1))
                    previous_intrinsic = intrinsic.copy()
                    agent.sample_idx_list = [0]
                    agent.current_scene_name = None
                    agent._occ_cache = {}
                    owner = metadata['owner']
                    last_access = time.monotonic()
                    reply = pack({'id': rid, 'ok': True})
                elif operation == 'step':
                    if agent is None or owner != metadata['owner']:
                        raise RuntimeError('Reset this client before inference')
                    image = cv2.imdecode(arrays['image'], cv2.IMREAD_COLOR)
                    depth = cv2.imdecode(arrays['depth'], cv2.IMREAD_UNCHANGED)
                    if image is None or depth is None or image.shape[:2] != depth.shape:
                        raise ValueError('Invalid RGB-D payload')
                    goal = np.asarray(arrays['goal'], dtype=float).reshape(2)
                    pos = np.asarray(arrays['position'], dtype=float).reshape(1,3)
                    quat = np.asarray(arrays['quaternion'], dtype=float).reshape(1,4)
                    if not all(np.isfinite(a).all() for a in (goal,pos,quat)):
                        raise ValueError('Invalid goal/pose')
                    trajectory, candidates, values, _ = agent.step_pointgoal_with_guidance(np.array([[goal[0],goal[1],0.0]]), image[None], depth.astype(np.float32)[None,...,None]/10000.0, pos, quat)
                    last_access = time.monotonic()
                    reply = pack({'id': rid, 'ok': True}, trajectory=np.asarray(trajectory), candidates=np.asarray(candidates), values=np.asarray(values))
                else:
                    raise ValueError('Unknown operation')
        except Exception as exc:
            traceback.print_exc(file=sys.stderr)
            reply = pack({'id': rid, 'ok': False, 'error': f'{type(exc).__name__}: {exc}'})
        write_frame(output, reply)
finally:
    plotter.close()
