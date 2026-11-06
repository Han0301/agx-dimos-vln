#!/usr/bin/env python3
"""Read selected camera status records and tiny telemetry files; never subscribe to images."""
import argparse
from datetime import datetime
from collections import deque
import json
import os
from pathlib import Path
import resource
import shutil
import signal
import sys
import time
import unicodedata
from zoneinfo import ZoneInfo

from insta360_ros_driver.camera_state import GROUPS, effective_state, read_metrics, read_state, stop_session
from insta360_ros_driver.console_status import status_line


class ExistingWatchdogs:
    """Read existing watchdogs' tiny logs until their next restart enables heartbeats.

    /proc is scanned once at startup. The refresh loop checks just the cached PIDs
    and incremental log bytes, without ROS subscriptions or command subprocesses.
    """
    def __init__(self):
        self.devices = {}
        workspace = next((parent for parent in Path(__file__).resolve().parents
                          if (parent / 'config/camera_stack.conf').is_file()), Path.cwd())
        log_dir = Path(os.environ.get('ROS_LOG_DIR', str(workspace / 'log')))
        for directory in Path('/proc').iterdir():
            if not directory.name.isdigit():
                continue
            try:
                args = (directory / 'cmdline').read_bytes().split(b'\0')
                if not any(arg.endswith(b'/wait_for_camera_data.py') for arg in args[:5]) or b'--watchdog' not in args:
                    continue
                if b'--state-group' in args:
                    continue  # New watchdogs already provide atomic heartbeat records.
                environment = (directory / 'environ').read_bytes().split(b'\0')
                domain = next((item.split(b'=', 1)[1].decode() for item in environment
                               if item.startswith(b'ROS_DOMAIN_ID=')), '0')
                if domain != os.environ.get('ROS_DOMAIN_ID', '0'):
                    continue
                group = 'X3' if b'--compressed-topics' in args else 'D455'
                ticks = (directory / 'stat').read_text().rsplit(')', 1)[1].split()[19]
                logs = list(log_dir.glob('*_' + directory.name + '_*.log'))
                if not logs:
                    continue
                candidate = {'pid': int(directory.name), 'ticks': ticks,
                             'log': max(logs, key=lambda path: path.stat().st_mtime),
                             'offset': 0, 'ready': False, 'errors': deque(maxlen=3)}
                if group not in self.devices or int(ticks) > int(self.devices[group]['ticks']):
                    self.devices[group] = candidate
            except (OSError, ValueError, IndexError):
                continue

    def read(self, group):
        device = self.devices.get(group)
        if device is None:
            return None
        from insta360_ros_driver.console_status import CameraDiagnostics
        try:
            with device['log'].open('rb') as stream:
                stream.seek(device['offset'])
                text = stream.read(8192).decode('utf-8', errors='replace')
                device['offset'] = stream.tell()
            for line in text.splitlines():
                if 'All required image streams are ready' in line:
                    device['ready'] = True
                if CameraDiagnostics.important.search(line):
                    device['errors'].append(clean(line))
        except OSError:
            pass
        try:
            ticks = Path(f"/proc/{device['pid']}/stat").read_text().rsplit(')', 1)[1].split()[19]
            alive = ticks == device['ticks']
        except (OSError, ValueError, IndexError):
            alive = False
        error = ' | '.join(device['errors'])
        if error or not alive:
            state, detail = 'ERROR', error or '现有取流监测进程已退出，请查看相机窗口。'
        elif device['ready']:
            state, detail = 'OK', '现有取流监测正常；持续断流时会自动报错。'
        else:
            state, detail = 'WAIT', '正在等待现有相机的取流监测就绪。'
        return {'group': group, 'state': state, 'detail': detail, 'last_error': error,
                'source': 'existing_watchdog', 'watchdog_pid': device['pid']}


def clean(text):
    # Status files may include child-process text: prevent terminal control injection.
    from insta360_ros_driver.console_status import CameraDiagnostics
    text = CameraDiagnostics.ansi.sub('', str(text))
    return ''.join(character if character >= ' ' else ' ' for character in text)


def cell_width(character):
    if unicodedata.combining(character):
        return 0
    return 2 if unicodedata.east_asian_width(character) in ('W', 'F') else 1


def clip_line(text, width):
    """Clip by terminal cells while preserving status badge color sequences."""
    from insta360_ros_driver.console_status import CameraDiagnostics
    if sum(cell_width(character) for character in CameraDiagnostics.ansi.sub('', text)) <= width:
        return text
    result, used, offset = [], 0, 0
    while offset < len(text):
        escape = CameraDiagnostics.ansi.match(text, offset)
        if escape:
            result.append(escape.group())
            offset = escape.end()
            continue
        character = text[offset]
        size = cell_width(character)
        if used + size > width - 1:
            break
        result.append(character)
        used += size
        offset += 1
    # Reset any badge/error color that was active at the clipping point.
    return ''.join(result) + '…' + ('\033[0m' if '\033[' in text else '')


def wrap_cells(text, width):
    line, used = [], 0
    for character in text:
        size = cell_width(character)
        if line and used + size > width:
            yield ''.join(line)
            line, used = [], 0
        line.append(character)
        used += size
    if line:
        yield ''.join(line)


def terminal_frame(text):
    # Clear each line before writing: clearing only after the final line leaves
    # stale rate/status suffixes when the new frame is shorter than the old one.
    return '\033[H\033[2K' + text.replace('\n', '\n\033[2K') + '\033[J'


def render(rows, cpu, memory, controlled=False):
    # Leave the final column unused to avoid terminal automatic line wrapping.
    width = max(1, min(shutil.get_terminal_size((110, 30)).columns - 1, 160))
    lines = ['多相机实时状态面板  |  ' + datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%H:%M:%S'),
             ('每秒刷新；Ctrl+C 停止本次启动的后台相机组。' if controlled else
              '只读面板；Ctrl+C 只退出面板，不停止其他终端的相机。'), '─' * width]
    if controlled and os.environ.get('CAMERA_STACK_LOG_DIR'):
        lines.append('相机日志：' + os.environ['CAMERA_STACK_LOG_DIR'])
    if not rows:
        lines.append('未检测到本次启动的相机组。')
    for row in rows:
        detail = clean(row.get('detail', ''))
        error = clean(row.get('last_error', ''))
        if row['state'] == 'ERROR' and error and detail == error:
            detail = '取流异常，原因见下方。'
        lines.append(status_line(row['group'], row['state'], detail))
        rates = row.get('rates_hz', {})
        metrics = row.get('stream_metrics', {})
        if metrics:
            raw = metrics.get('raw_rx_hz', {})
            compressed = metrics.get('compressed_tx_hz', {})
            lines.append(
                f"  原始输入：RGB {raw.get('color', 0.0):.1f} Hz | "
                f"depth {raw.get('depth', 0.0):.1f} Hz")
            lines.append(
                f"  压缩输出：RGB {compressed.get('color', 0.0):.1f} Hz | "
                f"depth {compressed.get('depth', 0.0):.1f} Hz")
            counters = metrics.get('counters', {})
            drops = []
            for key, label in (('color', 'RGB'), ('depth', 'depth')):
                value = counters.get(key, {})
                in_drop = int(value.get('input_overwritten', 0))
                out_drop = int(value.get('output_overwritten', 0))
                if in_drop or out_drop:
                    drops.append(f'{label} in={in_drop} out={out_drop}')
            if drops:
                lines.append('  累计丢弃：' + '；'.join(drops))
            ages = metrics.get('raw_age_sec', {})
            if any(isinstance(age, (int, float)) and age > 2.0 for age in ages.values()):
                lines.append('  原始输入已超时；压缩输出可能正在停止。')
        elif rates:
            short = []
            for topic, rate in rates.items():
                label = '彩色' if '/color/' in topic else '对齐深度' if 'aligned_depth' in topic else '相机码流'
                short.append(f'{label} {rate:.1f} Hz')
            lines.append('  原始话题接收：' + '；'.join(short))
        if row['group'] == 'D455' and not metrics and row.get('state') == 'OK':
            lines.append('  压缩指标：等待 compressed_rgbd_node 上报。')
        if row.get('last_error'):
            label = '核心错误' if row['state'] == 'ERROR' else '最近错误（当前已恢复）'
            lines.append(f'  {label}：')
            # Keep the important cause visible even when the terminal is narrow.
            for part in wrap_cells(error[:1800], max(1, width - 4)):
                lines.append('    ' + part)
        lines.append('')
    lines += ['─' * width, f'面板自身：CPU 约 {cpu:.2f}%（单核） | 内存 {memory:.1f} MiB']
    return '\n'.join(clip_line(line, width) for line in lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--once', action='store_true')
    parser.add_argument('--json', action='store_true')
    parser.add_argument('--duration', type=float, default=0.0)
    parser.add_argument('--session-id', default='')
    parser.add_argument('--groups', default='',
                        help='Comma-separated camera groups to display, e.g. D455 or D455,X3.')
    args = parser.parse_args()
    selected = []
    for value in args.groups.split(','):
        group = value.strip().upper()
        if not group:
            continue
        if group not in GROUPS:
            parser.error(f'unknown camera group: {value.strip()}')
        if group not in selected:
            selected.append(group)
    if args.session_id:
        def interrupt(signum, frame):
            raise KeyboardInterrupt
        signal.signal(signal.SIGTERM, interrupt)
        signal.signal(signal.SIGHUP, interrupt)
    terminal = sys.stdout.isatty() and not args.json
    last_visible = None
    existing = ExistingWatchdogs()
    started = previous_wall = time.monotonic()
    usage = resource.getrusage(resource.RUSAGE_SELF)
    previous_cpu = usage.ru_utime + usage.ru_stime
    if terminal:
        sys.stdout.write('\033[?1049h\033[?25l')
    try:
        while True:
            now = time.monotonic()
            rows = []
            groups = selected
            if not groups:
                groups = []
                for group in GROUPS:
                    state = read_state(group)
                    updated = state.get('updated_mono')
                    if (state.get('run_id') and state.get('state') != 'STOP' and
                            isinstance(updated, (int, float)) and now - updated <= 10.0):
                        groups.append(group)
            for group in groups:
                row = effective_state(group, now)
                if not row.get('run_id'):
                    row = existing.read(group) or row
                if group == 'D455':
                    metrics = read_metrics(group)
                    updated_at = metrics.get('updated_at')
                    if isinstance(updated_at, (int, float)) and time.time() - updated_at <= 3.5:
                        row = dict(row)
                        row['stream_metrics'] = metrics
                if args.session_id and row.get('session_id') != args.session_id:
                    row = dict(row)
                    row['detail'] = '其他会话（本面板不停止）：' + row.get('detail', '')
                rows.append(row)
            usage = resource.getrusage(resource.RUSAGE_SELF)
            cpu_now = usage.ru_utime + usage.ru_stime
            cpu = max(0.0, (cpu_now - previous_cpu) / max(0.001, now - previous_wall) * 100.0)
            if now - started < 0.5:
                cpu = 0.0
            memory = usage.ru_maxrss / 1024.0
            previous_wall, previous_cpu = now, cpu_now
            payload = {'devices': rows, 'mode': 'controller' if args.session_id else 'observer',
                       'dashboard_cpu_percent': round(cpu, 3), 'dashboard_rss_mib': round(memory, 2)}
            visible = [(row['state'], row.get('detail'), row.get('last_error')) for row in rows]
            if args.json:
                print(json.dumps(payload, ensure_ascii=False), flush=True)
            elif terminal:
                sys.stdout.write(terminal_frame(render(
                    rows, cpu, memory, bool(args.session_id))))
                sys.stdout.flush()
            elif visible != last_visible:
                print(render(rows, cpu, memory, bool(args.session_id)), flush=True)
            last_visible = visible
            if args.once or (args.duration > 0 and time.monotonic() - started >= args.duration):
                break
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        if terminal:
            sys.stdout.write('\033[?25h\033[?1049l')
            sys.stdout.flush()
        if args.session_id:
            for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
                signal.signal(sig, signal.SIG_IGN)
            print('正在停止本控制面板启动的相机组...', flush=True)
            stop_session(args.session_id)
            print('本次启动的后台相机组已停止。', flush=True)


if __name__ == '__main__':
    main()
