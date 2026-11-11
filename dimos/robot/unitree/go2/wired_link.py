"""Validate the direct Ethernet route used by the Go2 body bridge."""
from dataclasses import dataclass
from ipaddress import IPv4Address
import json
import os
from pathlib import Path
import subprocess
import sys


@dataclass(frozen=True)
class WiredLink:
    interface: str
    local_ip: str
    robot_ip: str

    @classmethod
    def check(cls, robot_ip: str) -> 'WiredLink':
        interface = os.environ.get('DIMOS_GO2_WIRED_INTERFACE', 'end0')
        error = f'Go2 wired connection unavailable ({interface} -> {robot_ip})'
        if not interface or Path(interface).name != interface:
            raise RuntimeError(f'{error}: invalid interface name')
        nic = Path('/sys/class/net') / interface
        try:
            IPv4Address(robot_ip)
            if not (nic / 'device').exists() or (nic / 'wireless').exists() or (nic / 'phy80211').exists():
                raise ValueError('interface is not a physical Ethernet device')
            if (nic / 'carrier').read_text().strip() != '1':
                raise ValueError('no Ethernet carrier; check the cable')
            result = subprocess.run(
                ['/usr/sbin/ip', '-j', '-4', 'route', 'get', robot_ip],
                check=True, capture_output=True, text=True, timeout=2,
            )
            route = json.loads(result.stdout)[0]
            if route.get('dev') != interface or route.get('gateway') or route.get('type') == 'local':
                raise ValueError(f'no direct route through {interface} (found {route.get("dev")})')
            local_ip = str(IPv4Address(route['prefsrc']))
        except (OSError, ValueError, KeyError, IndexError, subprocess.SubprocessError) as exc:
            raise RuntimeError(f'{error}: {exc}') from exc
        return cls(interface, local_ip, robot_ip)

    def check_current(self) -> None:
        if self.check(self.robot_ip) != self:
            raise RuntimeError(f'Go2 wired address or route changed on {self.interface}')


if __name__ == '__main__':
    try:
        link = WiredLink.check(sys.argv[1])
    except RuntimeError as exc:
        raise SystemExit(str(exc)) from None
    print(f'Go2 Ethernet ready: {link.interface} {link.local_ip} -> {link.robot_ip}', flush=True)
