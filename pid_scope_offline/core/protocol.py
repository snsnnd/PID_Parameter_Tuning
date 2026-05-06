import struct
from core.frame import PIDFrame

SOF = b"\xA5\x5A"
VERSION = 0x01

FRAME_TELEMETRY = 0x01
FRAME_PARAM_SET = 0x02
FRAME_PARAM_REPORT = 0x03
FRAME_EVENT = 0x05


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def encode_frame(frame_type: int, device_id: int, channel_id: int, payload: bytes) -> bytes:
    header = SOF + bytes([VERSION, frame_type, device_id, channel_id]) + struct.pack("<H", len(payload))
    crc = crc16_modbus(header + payload)
    return header + payload + struct.pack("<H", crc)


def decode_telemetry(payload: bytes) -> dict:
    fmt = "<I12fH"
    if len(payload) != struct.calcsize(fmt):
        raise ValueError("invalid telemetry payload length")
    vals = struct.unpack(fmt, payload)
    keys = [
        "timestamp_ms", "target", "feedback", "error", "output", "kp", "ki", "kd",
        "p_term", "i_term", "d_term", "extra1", "extra2", "status_flags"
    ]
    out = dict(zip(keys, vals))
    return out
