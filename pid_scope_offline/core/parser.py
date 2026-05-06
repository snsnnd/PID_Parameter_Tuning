import struct
from core.frame import PIDFrame
from core.protocol import SOF, crc16_modbus


class FrameParser:
    def __init__(self) -> None:
        self.buf = bytearray()

    def feed(self, data: bytes) -> list[PIDFrame]:
        self.buf.extend(data)
        frames: list[PIDFrame] = []
        while True:
            sof = self.buf.find(SOF)
            if sof < 0:
                self.buf.clear()
                break
            if sof > 0:
                del self.buf[:sof]
            if len(self.buf) < 10:
                break
            version, ftype, dev, ch = self.buf[2], self.buf[3], self.buf[4], self.buf[5]
            plen = struct.unpack_from("<H", self.buf, 6)[0]
            total = 10 + plen
            if len(self.buf) < total:
                break
            frame = bytes(self.buf[:total])
            payload = frame[8:-2]
            rx_crc = struct.unpack("<H", frame[-2:])[0]
            calc_crc = crc16_modbus(frame[:-2])
            if rx_crc == calc_crc:
                frames.append(PIDFrame(version, ftype, dev, ch, payload))
            del self.buf[:total]
        return frames
