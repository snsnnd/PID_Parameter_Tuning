from collections import defaultdict
from core.data_buffer import ChannelBuffer


class DeviceManager:
    def __init__(self, per_channel_buffer: int = 60000) -> None:
        self.buffers = defaultdict(lambda: ChannelBuffer(maxlen=per_channel_buffer))

    def push_telemetry(self, device_id: int, channel_id: int, tel: dict) -> None:
        self.buffers[(device_id, channel_id)].append(tel)

    def get_window(self, device_id: int, channel_id: int, seconds: float) -> list[dict]:
        return self.buffers[(device_id, channel_id)].latest_window(seconds)

    def get_latest(self, device_id: int, channel_id: int, count: int) -> list[dict]:
        return self.buffers[(device_id, channel_id)].latest_count(count)

    def get_latest_arrays(self, device_id: int, channel_id: int, count: int):
        return self.buffers[(device_id, channel_id)].latest_arrays(count)

    def get_rows(self, device_id: int, channel_id: int) -> list[dict]:
        return self.buffers[(device_id, channel_id)].rows()

    def export_channel_csv(self, device_id: int, channel_id: int, path: str) -> int:
        return self.buffers[(device_id, channel_id)].export_csv(path)

    def clear_channel(self, device_id: int, channel_id: int) -> None:
        self.buffers[(device_id, channel_id)].clear()

    def clear_all(self) -> None:
        for buf in self.buffers.values():
            buf.clear()

    def channels(self):
        return list(self.buffers.keys())
