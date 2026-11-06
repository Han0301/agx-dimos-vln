#!/usr/bin/env python3
"""Read-only DimOS cmd_vel/odom sidecar; standard library only; no unpickling."""
from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
import math
import mmap
import os
from pathlib import Path
import pickletools
import signal
import struct
import sys
import time

ALLOWED_CLASSES = {
    ('dimos.msgs.geometry_msgs.Twist', 'Twist'),
    ('dimos.msgs.geometry_msgs.Vector3', 'Vector3'),
    ('dimos.msgs.geometry_msgs.Quaternion', 'Quaternion'),
    ('dimos.msgs.geometry_msgs.Pose', 'Pose'),
    ('dimos.msgs.geometry_msgs.PoseStamped', 'PoseStamped'),
    ('dimos.robot.unitree.type.odometry', 'Odometry'),
}
MAX_PAYLOAD = 16384
MARK = object()


def system_snapshot(log_dir: Path, process_root: int | None) -> dict:
    """Local counters only; never inspect process arguments or environments."""
    result = {}
    try:
        result['loadavg'] = list(os.getloadavg())
        result['cpu_count'] = os.cpu_count()
        memory = {}
        for line in Path('/proc/meminfo').read_text().splitlines():
            key, rest = line.split(':', 1)
            if key in {'MemTotal', 'MemAvailable', 'SwapTotal', 'SwapFree', 'Dirty'}:
                memory[key + '_kib'] = int(rest.split()[0])
        result['memory'] = memory
        result['pressure'] = {name: p.read_text().strip()
                              for name in ('cpu', 'memory', 'io')
                              if (p := Path('/proc/pressure') / name).exists()}
        disk = os.statvfs(log_dir)
        result['log_disk_available_bytes'] = disk.f_bavail * disk.f_frsize
        interfaces = {}
        for line in Path('/proc/net/dev').read_text().splitlines()[2:]:
            name, counters = line.split(':', 1)
            values = [int(v) for v in counters.split()]
            interfaces[name.strip()] = dict(zip(
                ('rx_bytes', 'rx_errors', 'rx_drops', 'tx_bytes', 'tx_errors', 'tx_drops'),
                (values[0], values[2], values[3], values[8], values[10], values[11])))
        result['network_counters'] = interfaces
    except (OSError, ValueError, IndexError) as exc:
        result['system_read_error'] = type(exc).__name__
    if process_root is not None:
        inventory = {}
        for directory in Path('/proc').iterdir():
            if not directory.name.isdigit():
                continue
            try:
                raw = (directory / 'stat').read_text()
                close = raw.rfind(')')
                values = raw[close + 2:].split()
                inventory[int(directory.name)] = {
                    'pid': int(directory.name), 'comm': raw[raw.find('(') + 1:close],
                    'state': values[0], 'ppid': int(values[1]),
                    'cpu_ticks': int(values[11]) + int(values[12]),
                    'start_ticks': int(values[19]), 'threads': int(values[17]),
                    'rss_bytes': int(values[21]) * os.sysconf('SC_PAGE_SIZE')}
            except (OSError, ValueError, IndexError):
                continue
        selected = {process_root}
        while True:
            additions = {pid for pid, info in inventory.items()
                         if info['ppid'] in selected and pid not in selected}
            if not additions:
                break
            selected.update(additions)
        result['session_processes'] = [inventory[pid] for pid in sorted(selected)
                                      if pid in inventory and pid != os.getpid()]
        result['clock_ticks_per_second'] = os.sysconf('SC_CLK_TCK')
    return result


def inspect_fields(payload: bytes) -> dict:
    """Interpret a restricted data-only opcode subset into inert dictionaries.

    No imports, callable resolution, object construction, REDUCE, or __setstate__.
    Unknown schemas are rejected rather than deserialized.
    """
    stack, memo = [], {}
    for count, (op, arg, _) in enumerate(pickletools.genops(payload)):
        if count > 4096:
            raise ValueError('too many opcodes')
        name = op.name
        if name in {'PROTO', 'FRAME'}:
            continue
        if name in {'SHORT_BINUNICODE', 'BINUNICODE', 'UNICODE', 'BINFLOAT',
                    'BININT', 'BININT1', 'BININT2', 'LONG1', 'LONG4'}:
            stack.append(arg)
        elif name == 'NONE':
            stack.append(None)
        elif name in {'NEWTRUE', 'NEWFALSE'}:
            stack.append(name == 'NEWTRUE')
        elif name == 'EMPTY_DICT':
            stack.append({})
        elif name == 'EMPTY_TUPLE':
            stack.append(())
        elif name == 'MARK':
            stack.append(MARK)
        elif name == 'MEMOIZE':
            memo[len(memo)] = stack[-1]
        elif name in {'BINPUT', 'LONG_BINPUT'}:
            memo[arg] = stack[-1]
        elif name in {'BINGET', 'LONG_BINGET'}:
            stack.append(memo[arg])
        elif name == 'STACK_GLOBAL':
            cls = stack.pop()
            module = stack.pop()
            symbol = (module, cls)
            if symbol not in ALLOWED_CLASSES:
                raise ValueError('unsupported class')
            stack.append(symbol)  # inert identifier; never resolved or imported
        elif name == 'NEWOBJ':
            args = stack.pop()
            symbol = stack.pop()
            if args != () or symbol not in ALLOWED_CLASSES:
                raise ValueError('unsupported construction')
            stack.append({'_schema': '.'.join(symbol)})
        elif name in {'TUPLE1', 'TUPLE2', 'TUPLE3'}:
            size = int(name[-1])
            values = stack[-size:]
            del stack[-size:]
            stack.append(tuple(values))
        elif name == 'SETITEMS':
            pos = len(stack) - 1
            while pos >= 0 and stack[pos] is not MARK:
                pos -= 1
            if pos < 1 or not isinstance(stack[pos - 1], dict):
                raise ValueError('invalid dictionary')
            values = stack[pos + 1:]
            del stack[pos:]
            if len(values) % 2:
                raise ValueError('unpaired dictionary items')
            for key, value in zip(values[::2], values[1::2]):
                if not isinstance(key, str):
                    raise ValueError('non-string key')
                stack[-1][key] = value
        elif name == 'BUILD':
            state = stack.pop()
            target = stack[-1]
            if not isinstance(target, dict):
                raise ValueError('invalid state target')
            parts = state if isinstance(state, tuple) else (state,)
            for part in parts:
                if part is None:
                    continue
                if not isinstance(part, dict):
                    raise ValueError('unsupported state')
                target.update(part)  # dictionary data only; no object methods called
        elif name == 'STOP':
            if len(stack) != 1 or not isinstance(stack[0], dict):
                raise ValueError('invalid final value')
            return stack[0]
        else:
            raise ValueError('unsupported opcode: ' + name)
    raise ValueError('missing STOP')


def finite(value) -> float:
    if not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError('invalid numeric field')
    return float(value)


def fields(topic: str, data: dict) -> dict:
    if topic == 'cmd_vel':
        if data.get('_schema') != 'dimos.msgs.geometry_msgs.Twist.Twist':
            raise ValueError('unexpected cmd_vel schema')
        return {'v': finite(data['linear']['x']), 'vy': finite(data['linear']['y']),
                'w': finite(data['angular']['z'])}
    if data.get('_schema') not in {
        'dimos.robot.unitree.type.odometry.Odometry',
        'dimos.msgs.geometry_msgs.PoseStamped.PoseStamped',
    }:
        raise ValueError('unexpected odom schema')
    p, q = data['position'], data['orientation']
    x, y, z, w = (finite(q[k]) for k in ('x', 'y', 'z', 'w'))
    return {'x': finite(p['x']), 'y': finite(p['y']), 'z': finite(p['z']),
            'yaw': math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))}


class Channel:
    def __init__(self, root: Path, topic: str, capacity: int):
        digest = hashlib.blake2b(f'{topic}:{capacity}'.encode(), digest_size=8).hexdigest()
        self.paths = [root / f'psm_{digest}_{suffix}' for suffix in ('ctrl', 'data')]
        self.capacity = capacity
        self.maps, self.files, self.identities = [], [], []
        self.last_seq = None
        self.last_stamp_ns = None
        self.last_slot_id = None
        self.last_channel_seq = None

    def close(self):
        for m in self.maps:
            m.close()
        for f in self.files:
            f.close()
        self.maps, self.files, self.identities = [], [], []

    def connect(self):
        if self.maps:
            try:
                current = [(p.stat().st_dev, p.stat().st_ino, p.stat().st_size) for p in self.paths]
                if current == self.identities:
                    return
            except FileNotFoundError:
                pass
            self.close()
            self.last_seq = self.last_stamp_ns = self.last_slot_id = None
        try:
            for p in self.paths:
                f = p.open('rb')  # existing files only; never create shared memory
                self.files.append(f)
                st = os.fstat(f.fileno())
                self.identities.append((st.st_dev, st.st_ino, st.st_size))
                self.maps.append(mmap.mmap(f.fileno(), 0, access=mmap.ACCESS_READ))
            if len(self.maps[0]) < 24 or len(self.maps[1]) < 2 * (self.capacity + 20):
                raise ValueError('unexpected shared-memory size')
        except Exception:
            self.close()
            raise

    def read(self):
        self.connect()
        ctrl, data = self.maps
        before = bytes(ctrl[:24])
        seq, stamp, active = struct.unpack('<qqq', before)
        self.last_channel_seq = seq
        if seq <= 0 or seq == self.last_seq:
            return None
        if active not in (0, 1) or stamp <= 0:
            raise ValueError('invalid control block')
        off = active * (self.capacity + 20)
        size = struct.unpack_from('<I', data, off)[0]
        if not 16 <= size <= min(self.capacity + 16, MAX_PAYLOAD + 16):
            raise ValueError('invalid payload size')
        slot_id = bytes(data[off + 4:off + 20])
        payload = bytes(data[off + 20:off + 4 + size])
        if bytes(ctrl[:24]) != before:
            return None  # concurrent update; retry next poll
        if slot_id == self.last_slot_id:
            return None  # writer may have switched index before advancing sequence
        result = {'seq': seq, 'publish_ns': stamp,
                  'publish_age_s': round((time.time_ns() - stamp) / 1e9, 6),
                  'publish_gap_s': ((stamp - self.last_stamp_ns) / 1e9
                                    if self.last_stamp_ns is not None else None),
                  'skipped_updates': max(0, seq - self.last_seq - 1) if self.last_seq is not None else 0}
        self.last_seq, self.last_stamp_ns, self.last_slot_id = seq, stamp, slot_id
        return result, payload


def summarize(topic, rows):
    gaps = [r['publish_gap_s'] for r in rows if r['publish_gap_s'] is not None and r['publish_gap_s'] > 0]
    out = {'observed_updates': len(rows), 'skipped_updates': sum(r['skipped_updates'] for r in rows),
           'max_gap_s': round(max(gaps), 6) if gaps else None}
    if len(rows) >= 2:
        dt = (rows[-1]['publish_ns'] - rows[0]['publish_ns']) / 1e9
        out['observed_update_hz'] = round((len(rows) - 1) / dt, 3) if dt > 0 else None
    if topic == 'cmd_vel':
        out['forward_samples'] = sum(r['v'] > 0.05 for r in rows)
        out['reverse_samples'] = sum(r['v'] < -0.05 for r in rows)
        out['zero_samples'] = sum(abs(r['v']) < 1e-6 and abs(r['vy']) < 1e-6 and abs(r['w']) < 1e-6 for r in rows)
        out['turn_only_samples'] = sum(abs(r['v']) < 1e-6 and abs(r['w']) > 0.01 for r in rows)
    elif len(rows) >= 2:
        out['net_displacement_m'] = round(math.hypot(rows[-1]['x'] - rows[0]['x'], rows[-1]['y'] - rows[0]['y']), 6)
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=None)
    parser.add_argument('--shm-dir', type=Path, default=Path('/dev/shm'))
    parser.add_argument('--capacity', type=int, default=3686400)
    parser.add_argument('--poll-hz', type=float, default=50)
    parser.add_argument('--duration', type=float, default=0, help='seconds; 0 runs until Ctrl+C')
    parser.add_argument('--system', action='store_true', help='local resource counters every five seconds')
    parser.add_argument('--process-root', type=int, help='include descendants of this session PID; no command lines')
    args = parser.parse_args()
    if not 1 <= args.poll_hz <= 100 or args.capacity < MAX_PAYLOAD or args.duration < 0:
        parser.error('poll-hz must be 1..100, capacity >=16384, duration >=0')
    outpath = args.output or (Path.home() / 'Documents' / 'log'
                             / datetime.now().strftime('%Y%m%d_%H%M%S_%f') / 'drive_trace.jsonl')
    outpath.parent.mkdir(parents=True, exist_ok=True)
    try:
        os.nice(10)  # lower only this process's scheduling priority
    except (AttributeError, OSError):
        pass
    channels = {t: Channel(args.shm_dir, t, args.capacity) for t in ('cmd_vel', 'odom')}
    buckets = {t: [] for t in channels}
    errors = Counter()
    started = last_summary = time.monotonic()
    fd = os.open(outpath, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    print('Read-only recorder:', outpath, flush=True)
    with os.fdopen(fd, 'w', buffering=65536) as log:
        def emit(event):
            event['recorded_at_utc'] = datetime.now(timezone.utc).isoformat()
            log.write(json.dumps(event, ensure_ascii=False, allow_nan=False) + '\n')
        emit({'event': 'start', 'pid': os.getpid(), 'poll_hz': args.poll_hz,
              'scope': 'read-only cmd_vel/odom; no robot connection or publisher'})
        if args.system:
            emit({'event': 'system', **system_snapshot(outpath.parent, args.process_root)})
        def stop_requested(_signum, _frame):
            raise KeyboardInterrupt
        signal.signal(signal.SIGTERM, stop_requested)
        signal.signal(signal.SIGINT, stop_requested)
        try:
            while args.duration == 0 or time.monotonic() - started < args.duration:
                tick = time.monotonic()
                for topic, channel in channels.items():
                    try:
                        item = channel.read()
                        if item is None:
                            continue
                        meta, payload = item
                        numeric = fields(topic, inspect_fields(payload))
                        row = {'event': 'sample', 'topic': topic, **meta, **numeric}
                        emit(row)
                        # An old residual slot is recorded but is not a fresh-rate observation.
                        if -0.1 <= row['publish_age_s'] <= 1.0:
                            buckets[topic].append(row)
                    except (OSError, ValueError, KeyError, IndexError, TypeError, struct.error) as exc:
                        errors[(topic, type(exc).__name__)] += 1
                if tick - last_summary >= 5:
                    now = time.time_ns()
                    statuses = {}
                    for topic, channel in channels.items():
                        status = summarize(topic, buckets[topic])
                        status['last_age_s'] = round((now - channel.last_stamp_ns) / 1e9, 3) if channel.last_stamp_ns else None
                        status['channel_found'] = bool(channel.maps)
                        statuses[topic] = status
                    error_rows = [{'topic': t, 'kind': kind, 'count': n} for (t, kind), n in errors.items()]
                    emit({'event': 'summary', 'window_s': tick - last_summary, 'channels': statuses, 'read_errors': error_rows})
                    if args.system:
                        emit({'event': 'system', **system_snapshot(outpath.parent, args.process_root)})
                    log.flush()
                    print(json.dumps({'channels': statuses, 'read_errors': error_rows}, ensure_ascii=False), flush=True)
                    buckets = {t: [] for t in channels}
                    errors.clear()
                    last_summary = tick
                time.sleep(max(0, 1 / args.poll_hz - (time.monotonic() - tick)))
        except KeyboardInterrupt:
            pass
        finally:
            emit({'event': 'stop', 'elapsed_s': time.monotonic() - started,
                  'tail': {t: summarize(t, rows) for t, rows in buckets.items()},
                  'read_errors': [{'topic': t, 'kind': k, 'count': n} for (t, k), n in errors.items()]})
            for channel in channels.values():
                channel.close()
    print('Recorder stopped; original modules unchanged.', flush=True)


if __name__ == '__main__':
    try:
        main()
    except (OSError, ValueError) as exc:
        print('Recorder stopped:', exc, file=sys.stderr)
        raise SystemExit(1)
