import struct
from core.frame import PIDFrame

SOF = b"\xA5\x5A"
VERSION = 0x01

FRAME_TELEMETRY = 0x01
FRAME_PARAM_SET = 0x02
FRAME_PARAM_REPORT = 0x03
FRAME_EVENT = 0x05
FRAME_HEARTBEAT = 0x06
FRAME_MAP_STATUS = 0x08

STATUS_MOTOR_ENABLE = 1 << 0
STATUS_MOTOR_OUTPUT = 1 << 1
STATUS_SPEED_PID = 1 << 2
STATUS_TUNE = 1 << 3
STATUS_OUTPUT_SAT = 1 << 4
STATUS_PROTECT = 1 << 5
STATUS_LOST = 1 << 6
STATUS_MAP_PREDICTION = 1 << 7
STATUS_LINE_L2 = 1 << 8
STATUS_LINE_L1 = 1 << 9
STATUS_LINE_R1 = 1 << 10
STATUS_LINE_R2 = 1 << 11
STATUS_LINE_VALID = 1 << 12
STATUS_LINE_MASK = STATUS_LINE_L2 | STATUS_LINE_L1 | STATUS_LINE_R1 | STATUS_LINE_R2


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


def decode_heartbeat(payload: bytes) -> dict:
    fmt = "<IH"
    if len(payload) != struct.calcsize(fmt):
        raise ValueError("invalid heartbeat payload length")
    uptime_ms, status_flags = struct.unpack(fmt, payload)
    return {"uptime_ms": uptime_ms, "status_flags": status_flags}


def decode_line_bin_from_flags(flags: int) -> list[bool] | None:
    if (flags & STATUS_LINE_VALID) == 0:
        return None
    return [
        bool(flags & STATUS_LINE_L2),
        bool(flags & STATUS_LINE_L1),
        bool(flags & STATUS_LINE_R1),
        bool(flags & STATUS_LINE_R2),
    ]


def decode_map_status(payload: bytes) -> dict:
    fmt = "<IffHBHBfBH"
    if len(payload) != struct.calcsize(fmt):
        raise ValueError("invalid map status payload length")
    vals = struct.unpack(fmt, payload)
    keys = [
        "uptime_ms", "distance_m", "yaw_deg", "current_id", "current_type",
        "next_id", "next_type", "dist_to_next_m", "car_mode", "status_flags",
    ]
    return dict(zip(keys, vals))


SEGMENT_NAMES = {
    0: "UNKNOWN",
    1: "STRAIGHT",
    2: "CURVE_LEFT",
    3: "CURVE_RIGHT",
    4: "SHARP_LEFT",
    5: "SHARP_RIGHT",
    6: "S_CURVE",
    7: "U_TURN",
    8: "RISK",
}

CAR_MODE_NAMES = {
    0: "IDLE",
    1: "CALIBRATION",
    2: "LEARNING",
    3: "TRACKING",
    4: "PREDICTION",
    5: "HAIRPIN",
    6: "LOST",
    7: "PROTECT",
}


def describe_status_flags(flags: int) -> list[str]:
    items = []
    if flags & STATUS_MOTOR_ENABLE:
        items.append("电机允许")
    if flags & STATUS_MOTOR_OUTPUT:
        items.append("电机输出")
    if flags & STATUS_SPEED_PID:
        items.append("速度闭环")
    if flags & STATUS_TUNE:
        items.append("调参")
    if flags & STATUS_OUTPUT_SAT:
        items.append("输出饱和")
    if flags & STATUS_PROTECT:
        items.append("保护")
    if flags & STATUS_LOST:
        items.append("丢线")
    if flags & STATUS_MAP_PREDICTION:
        items.append("地图预测")
    line_bin = decode_line_bin_from_flags(flags)
    if line_bin is not None:
        names = [name for name, active in zip(["L2", "L1", "R1", "R2"], line_bin) if active]
        items.append("循迹:" + ("/".join(names) if names else "全白"))
    return items or ["空闲"]


def encode_param_set_payload(kp: float, ki: float, kd: float,
                             output_min: float = -10000.0, output_max: float = 10000.0,
                             integral_min: float = -10000.0, integral_max: float = 10000.0,
                             deadband: float = 0.0, d_filter_alpha: float = 0.15,
                             feedforward_gain: float = 0.0, max_target_step: float = 0.0,
                             max_output_slew_rate: float = 0.0,
                             enable_integral: int = 1, enable_derivative: int = 1,
                             enable_anti_windup: int = 1, enable_feedforward: int = 0) -> bytes:
    """封装 PARAM_SET 协议对应的 12 个 float 和 4 个 uint8 载荷"""
    return struct.pack('<12f4B',
        kp, ki, kd, output_min, output_max, integral_min, integral_max,
        deadband, d_filter_alpha, feedforward_gain, max_target_step, max_output_slew_rate,
        enable_integral, enable_derivative, enable_anti_windup, enable_feedforward
    )
