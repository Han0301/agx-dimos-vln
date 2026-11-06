"""Readable camera status badges, with ANSI backgrounds in terminals."""
import os
import re
import sys
from collections import deque


def status_line(group, state, message):
    colors = {'OK': '1;30;42', 'ERROR': '1;97;41', 'WAIT': '1;30;43', 'STOP': '1;30;47'}
    badge = f' STATE: {state:<5} '
    forced = os.environ.get('FORCE_COLOR', '')
    enabled = ('NO_COLOR' not in os.environ and forced != '0' and
               (bool(forced) or (sys.stdout.isatty() and os.environ.get('TERM') != 'dumb')))
    if enabled and state in colors:
        badge = f'\033[{colors[state]}m{badge}\033[0m'
        if state == 'ERROR':
            message = f'\033[1;31m{message}\033[0m'
    return f'[{group:<4}] {badge} {message}'


class CameraDiagnostics:
    """Highlight a bounded selection of actual child-process errors and retain causes."""
    ansi = re.compile(r'\x1b\[[0-?]*[ -/]*[@-~]')
    important = re.compile(
        r'\[\s*(?:ERROR|FATAL)\s*\]|\bERROR:|LIBUSB_ERROR_\w+|Permission denied|'
        r'No available camera|no Insta360.*usb device|No valid frames within|'
        r'No (?:RealSense )?devices? (?:were )?found|Failed to (?:open|start)|'
        r'device.*(?:busy|not connected|disconnected)|not accessible|Traceback|'
        r'(?:ImportError|ModuleNotFoundError|RuntimeError|ValueError):', re.IGNORECASE)

    def __init__(self, group, prefixes, run_id=None):
        self.group = group
        self.prefixes = tuple(prefixes)
        self.buffers = {}
        self.errors = deque(maxlen=3)
        self.seen = set()
        self.run_id = run_id

    def feed(self, event):
        if not event.process_name.startswith(self.prefixes):
            return
        key = (event.pid, event.from_stderr)
        text = self.buffers.get(key, '') + event.text.decode('utf-8', errors='replace')
        lines = text.splitlines(keepends=True)
        self.buffers[key] = ''
        if lines and not lines[-1].endswith(('\n', '\r')):
            self.buffers[key] = lines.pop()[-8192:]
        for line in lines:
            clean = self.ansi.sub('', line).strip()
            if not clean or not self.important.search(clean):
                continue
            clean = clean[:700]
            if clean in self.seen:
                continue
            self.errors.append(f'{event.process_name}: {clean}')
            if self.run_id:
                from .camera_state import record_error
                record_error(self.group, self.run_id, self.errors[-1])
            # Repeated device errors should not flood the terminal.
            if len(self.seen) < 12:
                self.seen.add(clean)
                from launch.logging import get_logger
                get_logger('camera.state').error(status_line(self.group, 'ERROR', 'KEY ERROR: ' + self.errors[-1]))

    def summary(self):
        return ' | '.join(self.errors)
