from PySide6.QtCore import QObject, Signal, Slot
from queue import Empty, Full, Queue
import struct
import time
import serial
from serial.tools import list_ports
from core.parser import FrameParser
from core.frame import PIDFrame
from core.protocol import FRAME_TELEMETRY


class SerialWorker(QObject):
    status = Signal(str)
    text_received = Signal(str)

    def __init__(self) -> None:
        super().__init__()
        self.ser = None
        self.running = False
        # 将解析器放在子线程中运行，处理 CPU 密集型任务
        self.parser = FrameParser()
        self.text_buffer = bytearray()
        self.write_queue: Queue[bytes] = Queue()
        self.frame_queue: Queue[PIDFrame] = Queue(maxsize=1000)
        self.dropped_frames = 0
        self.debug_print = False
        self._last_rx_host_s: dict[tuple[int, int], float] = {}
        self._last_rx_mcu_ms: dict[tuple[int, int], int] = {}

    def _safe_close_serial(self) -> None:
        ser = self.ser
        self.ser = None
        if not ser:
            return
        try:
            ser.close()
        except Exception as exc:
            self.status.emit(f"串口关闭异常，已忽略：{exc}")

    def _print_frame_debug(self, frame: PIDFrame) -> None:
        if not self.debug_print:
            return
        key = (frame.device_id, frame.channel_id)
        now_s = time.perf_counter()
        last_host_s = self._last_rx_host_s.get(key)
        host_dt_ms = None if last_host_s is None else (now_s - last_host_s) * 1000.0
        self._last_rx_host_s[key] = now_s

        mcu_ts_ms = None
        mcu_dt_ms = None
        if frame.frame_type == FRAME_TELEMETRY and len(frame.payload) >= 4:
            mcu_ts_ms = struct.unpack_from("<I", frame.payload, 0)[0]
            last_mcu_ms = self._last_rx_mcu_ms.get(key)
            mcu_dt_ms = None if last_mcu_ms is None else int(mcu_ts_ms - last_mcu_ms)
            self._last_rx_mcu_ms[key] = mcu_ts_ms

        host_text = "-" if host_dt_ms is None else f"{host_dt_ms:.1f}ms"
        mcu_text = "-" if mcu_dt_ms is None else f"{mcu_dt_ms}ms"
        ts_text = "-" if mcu_ts_ms is None else f"{mcu_ts_ms}ms"
        print(
            f"[RX] dev={frame.device_id} ch={frame.channel_id} type=0x{frame.frame_type:02X} "
            f"mcu_ts={ts_text} host_dt={host_text} mcu_dt={mcu_text} "
            f"queue={self.frame_queue.qsize()} dropped={self.dropped_frames}",
            flush=True,
        )

    def _queue_frame(self, frame: PIDFrame) -> None:
        try:
            self.frame_queue.put_nowait(frame)
            self._print_frame_debug(frame)
            return
        except Full:
            self.dropped_frames += 1
        try:
            self.frame_queue.get_nowait()
        except Empty:
            pass
        try:
            self.frame_queue.put_nowait(frame)
            self._print_frame_debug(frame)
        except Full:
            self.dropped_frames += 1

    def drain_frames(self, limit: int = 1000) -> list[PIDFrame]:
        frames: list[PIDFrame] = []
        for _ in range(max(0, limit)):
            try:
                frames.append(self.frame_queue.get_nowait())
            except Empty:
                break
        return frames

    def _feed_text_monitor(self, data: bytes) -> None:
        # PIDScope 二进制帧和文本命令共线；只显示明确的文本回复，避免二进制流刷爆 UI。
        for b in data:
            if b in (10, 13):
                if self.text_buffer:
                    line = self.text_buffer.decode("ascii", errors="ignore").strip()
                    self.text_buffer.clear()
                    if line.startswith(("$", ">")):
                        if self.debug_print:
                            print(f"[TEXT] {line}", flush=True)
                        self.text_received.emit(line)
                continue
            if 32 <= b <= 126:
                if len(self.text_buffer) < 256:
                    self.text_buffer.append(b)
            else:
                self.text_buffer.clear()

    def _flush_writes(self) -> None:
        if not self.ser or not self.ser.is_open:
            return
        while True:
            try:
                data = self.write_queue.get_nowait()
            except Empty:
                return
            try:
                self.ser.write(data)
                if self.debug_print:
                    try:
                        text = data.decode("ascii").strip()
                    except UnicodeDecodeError:
                        text = ""
                    if text and all(32 <= b <= 126 or b in (10, 13) for b in data):
                        print(f"[TX] {text}", flush=True)
                    else:
                        print(f"[TX] {len(data)} bytes", flush=True)
            except Exception as exc:
                self.status.emit(f"串口写入错误：{exc}")
                return

    def enqueue_bytes(self, data: bytes) -> None:
        if data:
            self.write_queue.put(bytes(data))

    def enqueue_text(self, text: str, monitor: bool = True) -> None:
        if not text:
            return
        self.write_queue.put(text.encode("ascii", errors="ignore"))
        if monitor:
            self.text_received.emit(f"> {text.strip()}")

    @staticmethod
    def list_serial_ports() -> list[tuple[str, str]]:
        ports: list[tuple[str, str]] = []
        for port in list_ports.comports():
            desc = port.description or "串口设备"
            hwid = port.hwid or ""
            text = f"{port.device}  |  {desc}  |  {hwid}"
            ports.append((port.device, text))
        return ports

    @Slot(str, int)
    def connect_port(self, port: str, baud: int) -> None:
        try:
            port = port.split("  |  ", 1)[0].strip()
            self.status.emit(f"正在打开：{port}@{baud}")
            self.ser = serial.Serial(port, baudrate=baud, timeout=0.05, write_timeout=0.2)
            self.running = True
            self.status.emit(f"已连接：{port}@{baud}")
            while self.running:
                ser = self.ser
                if not ser or not ser.is_open:
                    break
                self._flush_writes()
                data = ser.read(4096)
                if data:
                    self._feed_text_monitor(data)
                    frames = self.parser.feed(data)
                    for frame in frames:
                        self._queue_frame(frame)
        except Exception as exc:
            self.status.emit(f"串口错误：{exc}")
        finally:
            self._safe_close_serial()
            self.status.emit("已断开")

    @Slot()
    def stop(self) -> None:
        self.running = False
        try:
            if self.ser and self.ser.is_open:
                self.ser.cancel_read()
        except Exception:
            pass
        self._safe_close_serial()

    @Slot(bytes)
    def send_bytes(self, data: bytes) -> None:
        self.enqueue_bytes(data)

    @Slot(str)
    def send_text(self, text: str) -> None:
        self.enqueue_text(text)
