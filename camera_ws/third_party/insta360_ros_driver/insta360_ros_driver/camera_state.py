"""Tiny atomic state records shared by camera launchers, watchdogs and dashboard."""
import fcntl
import json
import os
from pathlib import Path
import re
import signal
import tempfile
import time
import uuid
from functools import lru_cache

GROUPS = ('D455', 'X3')
GROUP_METRICS = {'D455': 'compressed_d455_metrics.json'}


def process_ticks(pid):
    try:
        fields = Path(f'/proc/{int(pid)}/stat').read_text().rsplit(')', 1)[1].split()
        return None if fields[0] == 'Z' else fields[19]
    except (OSError, ValueError, IndexError):
        return None


def _session_directory(session_id):
    if not re.fullmatch(r'[A-Za-z0-9_.-]{1,100}', session_id or '') or session_id in ('.', '..'):
        raise ValueError('Invalid session ID')
    return state_directory() / 'sessions' / session_id


def session_cancelled(session_id):
    return bool(session_id) and (_session_directory(session_id) / 'cancelled').exists()


def register_wrapper(group, session_id, pid):
    return _register_process(group, session_id, pid, '')


def register_terminal(group, session_id, pid):
    """Register a dedicated terminal server's supervisor, never a shared server."""
    return _register_process(group, session_id, pid, '-terminal')


def _register_process(group, session_id, pid, suffix):
    if group not in GROUPS:
        raise ValueError('Unknown camera group')
    if not session_id:
        return True
    directory = _session_directory(session_id)
    directory.mkdir(parents=True, exist_ok=True)
    data = {'group': group, 'pid': int(pid), 'ticks': process_ticks(pid), 'boot_id': boot_id()}
    fd, temporary = tempfile.mkstemp(dir=directory, suffix='.tmp')
    with os.fdopen(fd, 'w') as stream:
        json.dump(data, stream)
    os.replace(temporary, directory / (group.lower() + suffix + '.json'))
    return not session_cancelled(session_id)


@lru_cache(maxsize=1)
def boot_id():
    try:
        return Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    except OSError:
        return ''


def state_directory():
    explicit = os.environ.get('CAMERA_STATE_DIR')
    if explicit:
        return Path(explicit).expanduser()
    base = os.environ.get('ROS_LOG_DIR')
    if not base:
        workspace = next((parent for parent in Path(__file__).resolve().parents
                          if (parent / 'config/camera_stack.conf').is_file()), Path.cwd())
        base = str(workspace / 'log')
    domain = os.environ.get('ROS_DOMAIN_ID', '0')
    return Path(base) / 'camera_state' / ('domain_' + domain)


def read_state(group):
    if group not in GROUPS:
        raise ValueError('Unknown camera group')
    try:
        value = json.loads((state_directory() / (group.lower() + '.json')).read_text())
        return value if isinstance(value, dict) else {}
    except (OSError, ValueError):
        return {}


def read_metrics(group):
    name = GROUP_METRICS.get(group)
    if not name:
        return {}
    try:
        value = json.loads((state_directory() / name).read_text())
        return value if isinstance(value, dict) and value.get('group') == group else {}
    except (OSError, ValueError):
        return {}


def _write(group, run_id, changes, *, begin=False, stop=False):
    if group not in GROUPS:
        raise ValueError('Unknown camera group')
    directory = state_directory()
    try:
        directory.mkdir(parents=True, exist_ok=True)
        path = directory / (group.lower() + '.json')
        with (directory / (group.lower() + '.lock')).open('a') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX)
            current = read_state(group)
            if begin:
                current = {'group': group, 'run_id': run_id, 'boot_id': boot_id(),
                           'owner_pid': os.getpid(), 'started_mono': time.monotonic(),
                           'fatal': False, 'last_error': '', 'heartbeat_mono': None}
            elif current.get('run_id') != run_id or current.get('boot_id') != boot_id():
                return False  # An older launcher/watchdog cannot overwrite a new run.
            elif current.get('fatal') and not changes.get('fatal'):
                return False  # Keep the root cause after a failed group shuts down.
            elif current.get('state') == 'STOP' and changes.get('state') != 'STOP':
                return False  # Ignore late shutdown logs/frames after a normal stop.
            if stop and current.get('fatal'):
                return False
            current.update(changes)
            current.update(updated_at=time.time(), updated_mono=time.monotonic())
            fd, temporary = tempfile.mkstemp(prefix=group.lower() + '.', suffix='.tmp', dir=directory)
            try:
                with os.fdopen(fd, 'w') as stream:
                    json.dump(current, stream, ensure_ascii=False)
                os.replace(temporary, path)
            finally:
                try:
                    os.unlink(temporary)
                except FileNotFoundError:
                    pass
        return True
    except OSError:
        # Display/telemetry failures must never interrupt image processing.
        return False


def begin_device(group, detail):
    session_id = os.environ.get('CAMERA_STACK_SESSION', '')
    if session_cancelled(session_id):
        raise RuntimeError('Camera controller session was cancelled before startup.')
    run_id = os.environ.get('CAMERA_STATE_RUN_ID') or uuid.uuid4().hex
    _write(group, run_id, {'state': 'WAIT', 'detail': detail, 'session_id': session_id,
                          'owner_ticks': process_ticks(os.getpid())}, begin=True)
    return run_id


def record_error(group, run_id, message, *, fatal=False):
    return _write(group, run_id, {'state': 'ERROR', 'detail': message[:2500],
                                 'last_error': message[:2500], 'error_at': time.time(), 'fatal': fatal})


def record_heartbeat(group, run_id, *, ready, detail, rates, ages, state=None):
    return _write(group, run_id, {'state': state or ('OK' if ready else 'ERROR'), 'detail': detail,
                                 'heartbeat_mono': time.monotonic(),
                                 'rates_hz': rates, 'frame_ages_sec': ages})


def stop_device(group, run_id):
    return _write(group, run_id, {'state': 'STOP', 'detail': '相机组已停止。'}, stop=True)


def effective_state(group, now=None):
    now = time.monotonic() if now is None else now
    value = read_state(group)
    if not value or value.get('boot_id') != boot_id():
        return {'group': group, 'state': 'WAIT', 'detail': '尚无本次启动状态，请启动相机窗口。', 'last_error': ''}
    result = dict(value)
    heartbeat = value.get('heartbeat_mono')
    if value.get('state') == 'STOP' or value.get('fatal'):
        return result
    try:
        pid = int(value.get('owner_pid', 0))
        if pid <= 0:
            raise ValueError('Invalid owner PID')
        os.kill(pid, 0)
        alive = True
    except (OSError, ValueError):
        alive = False
    if not alive:
        result.update(state='ERROR', detail='相机启动进程已退出，未收到正常停止状态。')
    elif heartbeat is not None and now - heartbeat > 3.5:
        result.update(state='ERROR', detail=f'监测心跳已中断 {now - heartbeat:.1f} 秒，请查看相机窗口。')
    return result


def stop_session(session_id, grace=8.0):
    """Stop only registered wrappers and launch trees belonging to this controller."""
    directory = _session_directory(session_id)
    directory.mkdir(parents=True, exist_ok=True)
    (directory / 'cancelled').touch()  # Delayed child windows must not start afterwards.
    wrappers, owners, terminals = {}, {}, {}
    for group in GROUPS:
        for suffix, processes in (('', wrappers), ('-terminal', terminals)):
            try:
                wrapper = json.loads((directory / (group.lower() + suffix + '.json')).read_text())
                if wrapper.get('ticks') is not None and wrapper.get('boot_id') == boot_id() and process_ticks(wrapper['pid']) == wrapper.get('ticks'):
                    processes[wrapper['pid']] = wrapper['ticks']
            except (OSError, ValueError, KeyError):
                pass
        state = read_state(group)
        if state.get('session_id') == session_id and state.get('boot_id') == boot_id():
            pid = state.get('owner_pid', 0)
            if pid > 0 and state.get('owner_ticks') is not None and process_ticks(pid) == state.get('owner_ticks'):
                owners[pid] = state['owner_ticks']
    descendants = dict(owners)
    descendants.update(wrappers)
    descendants.update(terminals)
    # Cache only these launchers' descendants for bounded cleanup if graceful exit stalls.
    relations = {}
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():
            continue
        try:
            fields = (entry / 'stat').read_text().rsplit(')', 1)[1].split()
            relations[int(entry.name)] = (int(fields[1]), fields[19])
        except (OSError, ValueError, IndexError):
            continue
    changed = True
    while changed:
        changed = False
        for pid, (parent, ticks) in relations.items():
            if parent in descendants and pid not in descendants:
                descendants[pid] = ticks
                changed = True

    def send(processes, sig):
        for pid, ticks in processes.items():
            if ticks is not None and process_ticks(pid) == ticks:
                try:
                    os.kill(pid, sig)
                except ProcessLookupError:
                    pass

    send(owners, signal.SIGINT)
    deadline = time.monotonic() + grace
    # Dedicated terminal servers stay alive until explicitly closed. Do not make
    # them hold up the camera launchers' graceful shutdown period.
    while time.monotonic() < deadline and any(process_ticks(pid) == ticks for pid, ticks in owners.items()):
        time.sleep(0.1)
    send(descendants, signal.SIGTERM)  # Includes this session's dedicated terminal servers.
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline and any(process_ticks(pid) == ticks for pid, ticks in descendants.items()):
        time.sleep(0.1)
    send(descendants, signal.SIGKILL)
    return {'launchers_stopped': len(owners), 'terminal_wrappers_closed': len(wrappers),
            'terminal_servers_closed': len(terminals)}
