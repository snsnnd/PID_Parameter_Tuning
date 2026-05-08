from dataclasses import dataclass


@dataclass(slots=True)
class PIDFrame:
    version: int
    frame_type: int
    device_id: int
    channel_id: int
    payload: bytes
