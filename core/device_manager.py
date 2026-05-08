from collections import defaultdict
from core.data_buffer import ChannelBuffer


class DeviceManager:
    def __init__(self, per_channel_buffer: int = 60000) -> None:
        self.buffers = defaultdict(lambda: ChannelBuffer(maxlen=per_channel_buffer))

    def push_telemetry(self, device_id: int, channel_id: int, tel: dict) -> None:
        self.buffers[(device_id, channel_id)].append(tel)

    def get_window(self, device_id: int, channel_id: int, seconds: float) -> list[dict]:
        return self.buffers[(device_id, channel_id)].latest_window(seconds)

    def channels(self):
        return list(self.buffers.keys())
