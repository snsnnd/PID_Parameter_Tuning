from collections import deque
from dataclasses import dataclass, field


@dataclass
class ChannelBuffer:
    maxlen: int = 10000
    rows: deque = field(default_factory=deque)

    def append(self, row: dict) -> None:
        if len(self.rows) >= self.maxlen:
            self.rows.popleft()
        self.rows.append(row)

    def latest_window(self, seconds: float) -> list[dict]:
        if not self.rows:
            return []
        end_ts = self.rows[-1]["timestamp_ms"]
        threshold = end_ts - int(seconds * 1000)
        return [r for r in self.rows if r["timestamp_ms"] >= threshold]
