import csv
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np


FIELDS = (
    "timestamp_ms", "target", "feedback", "error", "output",
    "kp", "ki", "kd", "p_term", "i_term", "d_term",
    "extra1", "extra2", "status_flags",
)


@dataclass
class ChannelBuffer:
    maxlen: int = 10000
    arrays: dict[str, np.ndarray] = field(init=False)
    start: int = 0
    size: int = 0

    def __post_init__(self) -> None:
        self.arrays = {name: np.zeros(self.maxlen, dtype=np.float64) for name in FIELDS}

    def append(self, row: dict) -> None:
        if self.maxlen <= 0:
            return
        idx = (self.start + self.size) % self.maxlen
        if self.size == self.maxlen:
            idx = self.start
            self.start = (self.start + 1) % self.maxlen
        else:
            self.size += 1
        for name in FIELDS:
            self.arrays[name][idx] = float(row.get(name, 0.0))

    def clear(self) -> None:
        self.start = 0
        self.size = 0

    def _indices(self) -> np.ndarray:
        if self.size <= 0:
            return np.array([], dtype=np.int64)
        return (self.start + np.arange(self.size, dtype=np.int64)) % self.maxlen

    def _ordered_arrays(self) -> dict[str, np.ndarray]:
        idx = self._indices()
        return {name: values[idx].copy() for name, values in self.arrays.items()}

    def latest_arrays(self, count: int) -> dict[str, np.ndarray]:
        if count <= 0 or self.size <= 0:
            return {name: np.array([], dtype=np.float64) for name in FIELDS}
        n = min(int(count), self.size)
        start = (self.start + self.size - n) % self.maxlen
        idx = (start + np.arange(n, dtype=np.int64)) % self.maxlen
        return {name: values[idx].copy() for name, values in self.arrays.items()}

    def latest_window_arrays(self, seconds: float) -> dict[str, np.ndarray]:
        data = self._ordered_arrays()
        if self.size <= 0:
            return data
        threshold = data["timestamp_ms"][-1] - float(seconds) * 1000.0
        mask = data["timestamp_ms"] >= threshold
        return {name: values[mask].copy() for name, values in data.items()}

    def latest_window(self, seconds: float) -> list[dict]:
        return self._arrays_to_rows(self.latest_window_arrays(seconds))

    def latest_count(self, count: int) -> list[dict]:
        return self._arrays_to_rows(self.latest_arrays(count))

    def rows(self) -> list[dict]:
        return self._arrays_to_rows(self._ordered_arrays())

    def _arrays_to_rows(self, data: dict[str, np.ndarray]) -> list[dict]:
        if not data or len(data["timestamp_ms"]) == 0:
            return []
        rows: list[dict] = []
        for i in range(len(data["timestamp_ms"])):
            row = {name: float(values[i]) for name, values in data.items()}
            row["timestamp_ms"] = int(row["timestamp_ms"])
            row["status_flags"] = int(row["status_flags"])
            rows.append(row)
        return rows

    def export_csv(self, path: str | Path) -> int:
        rows = self.rows()
        if not rows:
            return 0

        fieldnames = list(FIELDS)
        with Path(path).open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(rows)
        return len(rows)
