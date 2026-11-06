"""Versioned, data-only XNavDP packets carried by ROS2 ByteMultiArray."""
import io
import json
import struct
import numpy as np

VERSION = 'xnavdp-ros2-v1'
MAX_PACKET = 32 * 1024 * 1024

def pack(metadata, **arrays):
    stream = io.BytesIO()
    np.savez(stream, metadata=np.frombuffer(json.dumps({**metadata, 'protocol': VERSION}, allow_nan=False).encode(), dtype=np.uint8), **arrays)
    result = stream.getvalue()
    if len(result) > MAX_PACKET:
        raise ValueError('XNavDP packet exceeds size limit')
    return result

def unpack(payload):
    if len(payload) > MAX_PACKET:
        raise ValueError('XNavDP packet exceeds size limit')
    with np.load(io.BytesIO(payload), allow_pickle=False) as archive:
        metadata = json.loads(archive['metadata'].tobytes())
        if metadata.get('protocol') != VERSION:
            raise ValueError('XNavDP ROS2 protocol mismatch')
        return metadata, {key: archive[key].copy() for key in archive.files if key != 'metadata'}

def read_frame(stream):
    header = stream.read(8)
    if not header:
        raise EOFError
    if len(header) != 8:
        raise ValueError('Truncated frame header')
    size = struct.unpack('!Q', header)[0]
    if size > MAX_PACKET:
        raise ValueError('Frame too large')
    payload = bytearray()
    while len(payload) < size:
        chunk = stream.read(size - len(payload))
        if not chunk:
            raise EOFError
        payload.extend(chunk)
    return bytes(payload)

def write_frame(stream, payload):
    stream.write(struct.pack('!Q', len(payload)))
    stream.write(payload)
    stream.flush()
