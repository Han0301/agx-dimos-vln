"""A per-connection wired ICE policy for the pinned Go2 SDK and aiortc."""
import json
import logging
import socket

from aiortc import RTCConfiguration, RTCPeerConnection, RTCSessionDescription
from unitree_webrtc_connect.unitree_auth import NoSdpAnswerError, RobotBusyError
from unitree_webrtc_connect.webrtc_driver import UnitreeWebRTCConnection, WebRTCConnectionMethod
from unitree_webrtc_connect.webrtc_datachannel import WebRTCDataChannel

from dimos.robot.unitree.go2.wired_link import WiredLink

logger = logging.getLogger(__name__)


class WiredWebRTC(UnitreeWebRTCConnection):
    def __init__(self, link: WiredLink, aes_128_key: str | None):
        super().__init__(WebRTCConnectionMethod.LocalSTA, ip=link.robot_ip, aes_128_key=aes_128_key)
        self.link = link

    async def init_webrtc(self, turn_server_info=None, ip=None):
        self.link.check_current()
        self.pc = RTCPeerConnection(RTCConfiguration(iceServers=[]))
        self.datachannel = WebRTCDataChannel(self, self.pc)
        # Data channel creation has allocated the sole ICE transport, but has not gathered candidates.
        ice = self.pc.sctp.transport.transport.iceGatherer._connection
        gather = ice.get_component_candidates
        add_remote = ice.add_remote_candidate
        incoming = ice.request_received
        advertised = set()

        async def gather_wired(component, addresses, timeout=5):
            candidates = await gather(component, [self.link.local_ip], timeout=timeout)
            interface = self.link.interface.encode() + b'\0'
            for protocol in ice._protocols:
                sock = protocol.transport.get_extra_info('socket')
                try:
                    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BINDTODEVICE, interface)
                except OSError as exc:
                    raise RuntimeError(f'Cannot bind Go2 ICE socket to {self.link.interface}: {exc}') from exc
            return candidates

        async def add_wired(candidate):
            if candidate is not None:
                advertised.add(candidate.host)
                if candidate.host != self.link.robot_ip or candidate.transport.lower() != 'udp':
                    return
            await add_remote(candidate)

        def incoming_wired(message, addr, protocol, raw_data):
            # ICE can learn peer-reflexive candidates from inbound connectivity checks.
            if addr[0] == self.link.robot_ip:
                incoming(message, addr, protocol, raw_data)

        ice.get_component_candidates = gather_wired
        ice.add_remote_candidate = add_wired
        ice.request_received = incoming_wired

        @self.pc.on('connectionstatechange')
        def update_connection_state():
            self.isConnected = self.pc is not None and self.pc.connectionState == 'connected'

        await self.pc.setLocalDescription(await self.pc.createOffer())
        if not ice.local_candidates:
            raise RuntimeError(f'Cannot bind Go2 ICE to {self.link.local_ip} on {self.link.interface}')
        answer = await self.get_answer_from_local_peer(self.pc, self.link.robot_ip)
        if answer is None:
            raise NoSdpAnswerError()
        response = json.loads(answer)
        if response['sdp'] == 'reject':
            raise RobotBusyError()
        await self.pc.setRemoteDescription(RTCSessionDescription(sdp=response['sdp'], type=response['type']))
        if advertised and self.link.robot_ip not in advertised:
            raise RuntimeError(f'Go2 did not advertise wired ICE {self.link.robot_ip}; advertised {sorted(advertised)}')
        await self.datachannel.wait_datachannel_open()
        pair = ice._nominated.get(1)
        if pair is None or pair.local_addr[0] != self.link.local_ip or pair.remote_addr[0] != self.link.robot_ip:
            raise RuntimeError(f'Go2 failed wired ICE verification on {self.link.interface}: {pair}')
        self.link.check_current()
        logger.warning('Go2 wired ICE verified on %s: %s -> %s', self.link.interface, pair.local_addr, pair.remote_addr)
