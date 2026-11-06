"""Optional passive odom pipeline tracing. No ROS publications or robot requests."""
import json
import os
import queue
import threading
import time
from reactivex import operators as ops


class OdomTrace:
    def __init__(self, path):
        self.path = path
        self.events = queue.Queue(maxsize=100000)
        self.stopped = threading.Event()
        self.dropped = 0
        self.thread = threading.Thread(target=self._write, daemon=True)
        self.thread.start()

    def mark(self, stage, **values):
        if self.stopped.is_set():
            return
        row = dict(stage=stage, mono_ns=time.monotonic_ns(), **values)
        try:
            self.events.put_nowait(row)
        except queue.Full:
            self.dropped += 1

    def _write(self):
        with open(self.path, 'a', buffering=1) as f:
            while not self.stopped.is_set() or not self.events.empty():
                try:
                    row = self.events.get(timeout=.1)
                except queue.Empty:
                    continue
                f.write(json.dumps(row, separators=(',', ':')) + '\n')

    def stop(self):
        self.mark('trace_stop', dropped=self.dropped)
        self.stopped.set()
        self.thread.join(timeout=2)


def install_trace(bridge, path):
    trace = OdomTrace(path)
    connection = bridge.connection
    rtc = connection.conn
    channel = rtc.datachannel
    sctp = rtc.pc.sctp
    trace.mark('trace_start', wall_ns=time.time_ns(), ordered=channel.channel.ordered)

    # Timestamp the SDK's decoded message dispatch before Rx scheduling.
    resolve = channel.pub_sub.run_resolve
    def traced_resolve(message):
        topic = message.get('topic', '')
        if topic == 'rt/utlidar/robot_pose' and not trace.stopped.is_set():
            stamp = message.get('data', {}).get('header', {}).get('stamp', {})
            robot_stamp_ns = int(stamp['sec']) * 1_000_000_000 + int(stamp['nanosec']) if 'sec' in stamp and 'nanosec' in stamp else None
            trace.mark('sdk_resolve_enter', topic=topic, robot_stamp_ns=robot_stamp_ns)
        else:
            trace.mark('sdk_resolve_enter', topic=topic)
        try:
            return resolve(message)
        finally:
            trace.mark('sdk_resolve_exit', topic=topic)
    channel.pub_sub.run_resolve = traced_resolve

    decode = channel.deal_array_buffer
    def traced_decode(data):
        trace.mark('sdk_binary_decode_enter', size=len(data))
        try:
            return decode(data)
        finally:
            trace.mark('sdk_binary_decode_exit')
    channel.deal_array_buffer = traced_decode

    source = connection.unitree_sub_stream
    def traced_source(topic):
        return source(topic).pipe(ops.do_action(lambda _: trace.mark('raw_callback', topic=topic)))
    connection.unitree_sub_stream = traced_source

    raw = connection.raw_odom_stream
    connection.raw_odom_stream = lambda: raw().pipe(
        ops.do_action(lambda m: trace.mark('raw_rx_scheduled', stamp=m.get('_dimos_ros_received_at')))
    )
    odom = connection.odom_stream
    connection.odom_stream = lambda: odom().pipe(
        ops.do_action(lambda m: trace.mark('odom_rx_scheduled', stamp=m.ts))
    )

    publish_tf = bridge._publish_tf
    def traced_publish_tf(msg):
        trace.mark('bridge_enter', stamp=msg.ts)
        try:
            return publish_tf(msg)
        finally:
            trace.mark('bridge_exit', stamp=msg.ts)
    bridge._publish_tf = traced_publish_tf

    transport = bridge.odom.transport
    broadcast = transport.broadcast
    def traced_broadcast(stream, msg):
        trace.mark('ros_publish_enter', stamp=msg.ts)
        try:
            return broadcast(stream, msg)
        finally:
            trace.mark('ros_publish_exit', stamp=msg.ts)
    transport.broadcast = traced_broadcast

    # Distinguish received SCTP fragments from completed ordered messages.
    receive_chunk = sctp._receive_data_chunk
    async def traced_chunk(chunk):
        trace.mark('sctp_chunk', tsn=chunk.tsn, stream=chunk.stream_id,
                   seq=chunk.stream_seq, flags=chunk.flags, size=len(chunk.user_data),
                   cumulative_tsn=sctp._last_received_tsn)
        return await receive_chunk(chunk)
    sctp._receive_data_chunk = traced_chunk

    forward_tsn = sctp._receive_forward_tsn_chunk
    async def traced_forward_tsn(chunk):
        trace.mark('sctp_forward_tsn', cumulative_tsn=chunk.cumulative_tsn, streams=chunk.streams)
        return await forward_tsn(chunk)
    sctp._receive_forward_tsn_chunk = traced_forward_tsn

    receive_message = sctp._data_channel_receive
    async def traced_message(stream_id, pp_id, data):
        trace.mark('sctp_message', stream=stream_id, pp_id=pp_id, size=len(data))
        return await receive_message(stream_id, pp_id, data)
    sctp._data_channel_receive = traced_message

    def loop_heartbeat():
        if trace.stopped.is_set():
            return
        streams = {str(k): {'waiting_seq': v.sequence_number,
                            'queued': len(v.reassembly),
                            'first_seq': v.reassembly[0].stream_seq if v.reassembly else None,
                            'first_tsn': v.reassembly[0].tsn if v.reassembly else None}
                   for k, v in sctp._inbound_streams.items()}
        stats = list(sctp.transport._get_stats().values())
        trace.mark('loop_heartbeat', streams=streams,
                   cumulative_tsn=sctp._last_received_tsn,
                   misordered=len(sctp._sack_misordered),
                   packets_received=sum(s.packetsReceived for s in stats),
                   bytes_received=sum(s.bytesReceived for s in stats))
        connection.loop.call_later(.05, loop_heartbeat)
    connection.loop.call_soon_threadsafe(loop_heartbeat)

    def thread_heartbeat():
        while not trace.stopped.wait(.05):
            trace.mark('thread_heartbeat')
    threading.Thread(target=thread_heartbeat, daemon=True).start()
    timer = threading.Timer(float(os.environ.get("DIMOS_ODOM_TRACE_SECONDS", "180")), trace.stop)
    timer.daemon = True
    timer.start()
    return trace
