from __future__ import annotations

import glob
import time

from serial.tools import list_ports

FRAME_START = 0xAA
FRAME_MAX_PAYLOAD = 64

CMD_GET_STATUS = 0x1
CMD_SET_POSITION = 0x2
CMD_SET_DUTY = 0x3
CMD_CAL = 0x4
CMD_DEBUG = 0x5
CMD_ZERO_POSITION = 0x6
CMD_SET_VELOCITY = 0x7
CMD_SET_CONTROL = 0x8
CMD_BRAKE = 0x9
CMD_SEND_POSITION = 0xA
CMD_ENTER_BOOT = 0xB
CMD_SET_ADVANCE = 0xC
CMD_SET_QUIET = 0xD
CMD_GET_POS_VEL = 0xE
CMD_SEND_DUTY = 0xF

CAL_START = 0x1
CAL_STATUS = 0x2
CAL_READ_POINT = 0x3
CAL_WRITE_BLOB = 0x4
CAL_COMMIT = 0x5
CAL_CLEAR = 0x6
CAL_INFO = 0x7
CAL_READ_BLOB = 0x8

DEBUG_CAPTURE_START = 0x1
DEBUG_CAPTURE_STATUS = 0x2
DEBUG_CAPTURE_READ = 0x3
DEBUG_SET_OBSERVER = 0x4
DEBUG_STEP_SET = 0x5
DEBUG_STEP_TRANSITION = 0x6

OBSERVER_SAMPLE_LP2 = 0
OBSERVER_SECANT = 1
OBSERVER_SECANT_LP2 = 2
OBSERVER_SECANT_LP4 = 3
OBSERVER_SECANT_LP8 = 4
OBSERVER_SECANT_LP16 = 5
OBSERVER_AB_FAST = 6
OBSERVER_AB_MID = 7
OBSERVER_AB_SLOW = 8

OBSERVER_NAMES = {
    OBSERVER_SAMPLE_LP2: "sample_lp2",
    OBSERVER_SECANT: "secant",
    OBSERVER_SECANT_LP2: "secant_lp2",
    OBSERVER_SECANT_LP4: "secant_lp4",
    OBSERVER_SECANT_LP8: "secant_lp8",
    OBSERVER_SECANT_LP16: "secant_lp16",
    OBSERVER_AB_FAST: "ab_fast",
    OBSERVER_AB_MID: "ab_mid",
    OBSERVER_AB_SLOW: "ab_slow",
}

BOOT_MAGIC = 0x424F4F54

RESULT_OK = 0
RESULT_BAD_STATE = 1
RESULT_BAD_ARG = 2
RESULT_NOT_CALIBRATED = 3
RESULT_BUSY = 4
RESULT_RANGE = 5
RESULT_FLASH = 6
RESULT_CRC = 7

MODE_IDLE = 0
MODE_RUN = 1
MODE_CAL = 2

FLAG_CAL_VALID = 1 << 0
FLAG_BUSY = 1 << 1
FLAG_POSITION_REACHED = 1 << 2
FLAG_FAULT = 1 << 3
FLAG_SENSOR_OK = 1 << 4

FAULT_SENSOR = 1 << 0
FAULT_UNCALIBRATED = 1 << 1
FAULT_FLASH = 1 << 2


def crc8(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
    return crc


def encode_frame(address: int, cmd: int, payload: bytes = b"") -> bytes:
    if not 0 <= address <= 0xF:
        raise ValueError("address must fit in 4 bits")
    if len(payload) > FRAME_MAX_PAYLOAD:
        raise ValueError("payload too large")
    frame = bytes((FRAME_START, ((address & 0xF) << 4) | (cmd & 0xF), len(payload))) + payload
    return frame + bytes((crc8(frame),))


def decode_frame(frame: bytes) -> tuple[int, int, bytes]:
    if len(frame) < 4:
        raise ValueError("frame too short")
    if frame[0] != FRAME_START:
        raise ValueError("bad start byte")
    if frame[2] > FRAME_MAX_PAYLOAD:
        raise ValueError("payload too large")
    if len(frame) != frame[2] + 4:
        raise ValueError("bad frame length")
    if crc8(frame[:-1]) != frame[-1]:
        raise ValueError("bad frame crc")
    return frame[1] >> 4, frame[1] & 0xF, frame[3:-1]


class FrameReader:
    def __init__(self, port):
        self.port = port
        self.buffer = bytearray()
        self.frames = []
        self.crc_errors = 0

    def feed(self, chunk: bytes) -> None:
        self.buffer.extend(chunk)
        while self.buffer:
            start = self.buffer.find(bytes((FRAME_START,)))
            if start < 0:
                self.buffer.clear()
                return
            del self.buffer[:start]
            if len(self.buffer) < 3:
                return
            count = self.buffer[2] + 4
            if self.buffer[2] > FRAME_MAX_PAYLOAD:
                del self.buffer[0]
                continue
            if len(self.buffer) < count:
                return
            try:
                frame = decode_frame(bytes(self.buffer[:count]))
            except ValueError:
                self.crc_errors += 1
                del self.buffer[0]
            else:
                self.frames.append(frame)
                del self.buffer[:count]

    def poll(self):
        count = self.port.in_waiting
        if count:
            self.feed(self.port.read(count))
        frames, self.frames = self.frames, []
        return frames

    def read(self, address: int, expected_cmd: int, timeout: float) -> bytes:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            for index, (got_address, cmd, payload) in enumerate(self.frames):
                if got_address == address and cmd == expected_cmd:
                    del self.frames[index]
                    return payload
            self.feed(self.port.read(max(1, getattr(self.port, 'in_waiting', 0))))
        raise TimeoutError(f"timeout waiting for command 0x{expected_cmd:X} response")


def read_frame(port, address: int, expected_cmd: int, timeout: float) -> bytes:
    return FrameReader(port).read(address, expected_cmd, timeout)


def serial_port_allowed(value: str, text: str = "") -> bool:
    combined = f"{value} {text}".lower()
    blocked = ("bluetooth", "debug", "incoming-port", "debug-console")
    return bool(value) and not any(token in combined for token in blocked)


def find_serial_port(port: str | None) -> str:
    if port and port != "auto":
        return port

    ports = []
    for port_info in list_ports.comports():
        text = " ".join(
            [
                port_info.device or "",
                port_info.description or "",
                port_info.manufacturer or "",
                port_info.hwid or "",
            ]
        )
        if serial_port_allowed(port_info.device or "", text):
            ports.append(port_info)

    def score(port_info) -> tuple[int, str]:
        text = " ".join(
            [
                port_info.device or "",
                port_info.description or "",
                port_info.manufacturer or "",
                port_info.hwid or "",
            ]
        ).lower()
        value = 0
        if getattr(port_info, "vid", None) == 0x303A:
            value += 100
        if "esp32" in text or "espressif" in text:
            value += 80
        if "usbmodem" in text or "cdc" in text or "com" in text:
            value += 20
        return value, port_info.device or ""

    if ports:
        return sorted(ports, key=lambda item: (-score(item)[0], score(item)[1]))[0].device

    for pattern in ("/dev/cu.usbmodem*", "/dev/ttyACM*", "/dev/ttyUSB*"):
        matches = [value for value in sorted(glob.glob(pattern)) if serial_port_allowed(value)]
        if matches:
            return matches[0]

    raise TimeoutError("no serial port found")
