#!/usr/bin/env python3
"""Stop local D455 processes before replacing them with a new camera session."""
import json
import os
from pathlib import Path
import signal
import sys
import time

from insta360_ros_driver.camera_state import boot_id, process_ticks, read_state, state_directory


def snapshot(workspace):
    processes = {}
    for entry in Path('/proc').glob('[0-9]*'):
        try:
            if entry.stat().st_uid != os.getuid():
                continue
            fields = (entry / 'stat').read_text().rsplit(')', 1)[1].split()
            if fields[0] == 'Z':
                continue
            args = [os.fsdecode(a) for a in (entry / 'cmdline').read_bytes().split(b'\0') if a]
            env = dict(v.split('=', 1) for v in (entry / 'environ').read_bytes().decode().split('\0') if '=' in v)
            if env.get('ROS_DOMAIN_ID', '0') != os.environ.get('ROS_DOMAIN_ID', '0'):
                continue
            processes[int(entry.name)] = {'parent': int(fields[1]), 'ticks': fields[19], 'args': args, 'env': env}
        except (OSError, ValueError, IndexError):
            continue
    return processes


def stop_existing(workspace, grace=10.0):
    processes = snapshot(workspace)
    roots = {}
    sessions = set()
    def add(pid, ticks):
        if pid in processes and ticks is not None and processes[pid]['ticks'] == ticks:
            roots[pid] = ticks
            return True
        return False
    directory = state_directory()
    for record in (directory / 'sessions').glob('*/d455.json'):
        try:
            data = json.loads(record.read_text())
            if data.get('boot_id') == boot_id() and add(data['pid'], data.get('ticks')):
                sessions.add(record.parent.name)
        except (OSError, ValueError, KeyError):
            pass
    state = read_state('D455')
    if state.get('boot_id') == boot_id() and add(state.get('owner_pid'), state.get('owner_ticks')):
        sessions.add(state.get('session_id', ''))

    # Recover children orphaned after their launcher/controller already exited.
    # Match exact workspace executables and the D455 camera name/namespace.
    compressor = str(workspace / 'install/compressed_RGBD_pkg/lib/compressed_RGBD_pkg/compressed_rgbd_node')
    camera = str(workspace / 'install/realsense2_camera/lib/realsense2_camera/realsense2_camera_node')
    controllers = {}
    for pid, info in processes.items():
        args = info['args']
        paths = {str(Path(a).resolve()) for a in args if a.startswith('/') and not a.startswith('/camera/')}
        if compressor in paths or (camera in paths and '__node:=camera' in args and '__ns:=/camera' in args):
            add(pid, info['ticks'])
        if any(a.endswith('/camera_dashboard.py') and str(workspace) in str(Path(a).resolve()) for a in args):
            try:
                groups = args[args.index('--groups') + 1]
                session = args[args.index('--session-id') + 1]
            except (ValueError, IndexError):
                continue
            # A controller that also owns X3 must remain alive.
            if groups == 'D455' and (session in sessions or info['env'].get('CAMERA_STACK_SESSION') in sessions):
                controllers[pid] = info['ticks']
    targets = dict(roots)
    targets.update(controllers)
    changed = True
    while changed:
        changed = False
        for pid, info in processes.items():
            if info['parent'] in targets and pid not in targets:
                targets[pid] = info['ticks']
                changed = True
    if not targets:
        print('[D455] No previous local camera processes; starting a new session.', flush=True)
        return
    print('[D455] Stopping previous camera group: ' + ', '.join(map(str, sorted(targets))), flush=True)
    def alive():
        return {pid: ticks for pid, ticks in targets.items() if process_ticks(pid) == ticks}
    def send(selected, sig):
        for pid, ticks in selected.items():
            if process_ticks(pid) == ticks:
                try:
                    os.kill(pid, sig)
                except ProcessLookupError:
                    pass
    # Let known controllers/launchers clean up first; orphan children get SIGINT too.
    send(targets, signal.SIGINT)
    for sig, seconds in ((signal.SIGTERM, grace), (signal.SIGKILL, 2.0)):
        deadline = time.monotonic() + seconds
        while alive() and time.monotonic() < deadline:
            time.sleep(0.1)
        remaining = alive()
        if not remaining:
            break
        send(remaining, sig)
    deadline = time.monotonic() + 2
    while alive() and time.monotonic() < deadline:
        time.sleep(0.1)
    if alive():
        raise RuntimeError('Old D455 processes did not exit; refusing duplicate startup: ' + str(sorted(alive())))
    print('[D455] Previous camera group stopped; starting this session.', flush=True)


if __name__ == '__main__':
    stop_existing(Path(sys.argv[1]).resolve())
