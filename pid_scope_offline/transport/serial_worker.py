from PySide6.QtCore import QObject, Signal, Slot
import serial


class SerialWorker(QObject):
    data_ready = Signal(bytes)
    status = Signal(str)

    def __init__(self) -> None:
        super().__init__()
        self.ser = None
        self.running = False

    @Slot(str, int)
    def connect_port(self, port: str, baud: int) -> None:
        try:
            self.ser = serial.Serial(port, baudrate=baud, timeout=0.02)
            self.running = True
            self.status.emit(f"Connected: {port}@{baud}")
            while self.running and self.ser and self.ser.is_open:
                data = self.ser.read(4096)
                if data:
                    self.data_ready.emit(data)
        except Exception as exc:
            self.status.emit(f"Serial error: {exc}")
        finally:
            if self.ser:
                self.ser.close()
            self.status.emit("Disconnected")

    @Slot()
    def stop(self) -> None:
        self.running = False


    @Slot(bytes)
    def send_bytes(self, data: bytes) -> None:
        try:
            if self.ser and self.ser.is_open:
                self.ser.write(data)
        except Exception as exc:
            self.status.emit(f"Serial write error: {exc}")
