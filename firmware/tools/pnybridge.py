"""Host side of the stateless PennyESC bridge protocol (firmware/Lib/pennyesc_bridge.h).

Every message is a packet ``[type, payload..., crc8]``, COBS encoded between
0x00 delimiters. The bridge keeps no mode: every UART packet names its baud.

``Bridge`` looks like a pyserial port to the ESC tools: ``write`` sends ESC
bytes at ``bridge.baud`` and ``read`` returns what the ESC sent back.
"""

from __future__ import annotations

import time
from dataclasses import dataclass

import serial

from pnyproto import crc8

VERSION = 2

PKT_HELLO = 0x01
PKT_PINS = 0x02
PKT_AUX = 0x03
PKT_STREAM = 0x04
PKT_UART = 0x10
PKT_UART_RX = 0x11
PKT_TELEMETRY = 0x12
PKT_STATE = 0x81

BAUD_APP = 0
BAUD_BOOT = 1

ERRORS = {
    1: "invalid pin or value",
    2: "bridge could not save settings",
    3: "bridge could not start PWM",
    4: "bridge firmware does not support this command",
    5: "USB is using the bridge",
}


class BridgeError(RuntimeError):
    pass


def cobs_encode(data: bytes) -> bytes:
    out = bytearray([0])
    code_index, code = 0, 1
    for byte in data:
        if byte == 0:
            out[code_index] = code
            code_index, code = len(out), 1
            out.append(0)
            continue
        out.append(byte)
        code += 1
        if code == 0xFF:
            out[code_index] = code
            code_index, code = len(out), 1
            out.append(0)
    out[code_index] = code
    return bytes(out)


def cobs_decode(data: bytes) -> bytes | None:
    out = bytearray()
    index = 0
    while index < len(data):
        code = data[index]
        index += 1
        if code == 0 or index + code - 1 > len(data):
            return None
        out.extend(data[index : index + code - 1])
        index += code - 1
        if code != 0xFF and index < len(data):
            out.append(0)
    return bytes(out)


def encode_packet(kind: int, payload: bytes = b"") -> bytes:
    packet = bytes((kind,)) + bytes(payload)
    return b"\x00" + cobs_encode(packet + bytes((crc8(packet),))) + b"\x00"


@dataclass
class BridgeState:
    version: int
    rx: int
    tx: int
    gnd: int
    aux_pin: int
    aux_pulse: int
    aux_enabled: bool
    board: str


def _signed(value: int) -> int:
    return value - 256 if value > 127 else value


class Bridge:
    def __init__(self, port: str, timeout: float = 0.05) -> None:
        self.port_name = port
        self.timeout = timeout
        self.baud = BAUD_APP
        self._serial = serial.Serial(port=port, baudrate=115200, timeout=0, write_timeout=1.0)
        self._serial.dtr = False
        self._serial.rts = False
        self._encoded = bytearray()
        self._rx = bytearray()
        self._state: bytes | None = None
        self.telemetry: list[tuple[int, bytes]] = []
        try:
            self.state = self.hello()
        except Exception:
            self.close()
            raise

    # Connection

    @property
    def is_open(self) -> bool:
        return self._serial.is_open

    def close(self) -> None:
        if self._serial.is_open:
            self._serial.close()

    def __enter__(self) -> "Bridge":
        return self

    def __exit__(self, *_: object) -> None:
        self.close()

    def _pump(self, wait: float) -> None:
        deadline = time.monotonic() + wait
        while True:
            data = self._serial.read(self._serial.in_waiting or 1)
            if data:
                for byte in data:
                    if byte:
                        self._encoded.append(byte)
                        continue
                    packet = cobs_decode(bytes(self._encoded)) if self._encoded else None
                    self._encoded.clear()
                    if packet:
                        self._handle(packet)
                if not self._serial.in_waiting:
                    return
            elif time.monotonic() >= deadline:
                return
            else:
                time.sleep(0.0005)

    def _handle(self, packet: bytes) -> None:
        if len(packet) < 2 or crc8(packet[:-1]) != packet[-1]:
            return
        kind, data = packet[0], packet[1:-1]
        if kind == PKT_UART_RX:
            if data and data[0] == self.baud:
                self._rx.extend(data[1:])
        elif kind == PKT_STATE:
            self._state = data
        elif kind == PKT_TELEMETRY:
            offset = 0
            while offset + 8 <= len(data):
                size = data[offset + 6] + 4
                stamp = int.from_bytes(data[offset : offset + 4], "little")
                self.telemetry.append((stamp, data[offset + 4 : offset + 4 + size]))
                offset += 4 + size

    def _send(self, kind: int, payload: bytes = b"") -> None:
        self._serial.write(encode_packet(kind, payload))

    # Bridge commands

    def command(self, kind: int, payload: bytes = b"", timeout: float = 1.0) -> BridgeState:
        self._state = None
        self._send(kind, payload)
        deadline = time.monotonic() + timeout
        while self._state is None:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError("bridge did not reply")
            self._pump(remaining)
        data = self._state
        if len(data) < 9:
            raise BridgeError("invalid bridge reply")
        if data[0]:
            raise BridgeError(ERRORS.get(data[0], f"bridge error {data[0]}"))
        return BridgeState(data[1], _signed(data[2]), _signed(data[3]), _signed(data[4]), _signed(data[5]),
                           data[6] | data[7] << 8, data[8] == 1, data[9:].decode(errors="replace"))

    def hello(self, timeout: float = 3.0) -> BridgeState:
        """Greet the bridge. Opening native USB can reset it, so keep asking."""
        deadline = time.monotonic() + timeout
        while True:
            try:
                state = self.command(PKT_HELLO, timeout=0.4)
                break
            except TimeoutError:
                if time.monotonic() > deadline:
                    raise BridgeError(
                        "bridge did not answer; flash the bridge firmware "
                        "(pio run -d firmware/esp32s3demo -e bootbridge -t upload)"
                    ) from None
        if state.version != VERSION:
            raise BridgeError(f"bridge firmware version {state.version} is not supported; flash the bridge")
        return state

    def set_pins(self, rx: int, tx: int, gnd: int = -1) -> BridgeState:
        return self.command(PKT_PINS, bytes((rx & 0xFF, tx & 0xFF, gnd & 0xFF)))

    def set_aux(self, pin: int, pulse_us: int) -> BridgeState:
        return self.command(PKT_AUX, bytes((pin & 0xFF, pulse_us & 0xFF, pulse_us >> 8)))

    def stream(self, address: int, hz: int) -> BridgeState:
        self.telemetry.clear()
        return self.command(PKT_STREAM, bytes((address, hz)))

    # pyserial-like UART

    def set_baud(self, baud: int) -> None:
        if baud != self.baud:
            self.baud = baud
            self._rx.clear()

    @property
    def in_waiting(self) -> int:
        self._pump(0)
        return len(self._rx)

    def read(self, size: int = 1) -> bytes:
        deadline = time.monotonic() + (self.timeout or 0)
        self._pump(0)
        while len(self._rx) < size:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            self._pump(remaining)
        data = bytes(self._rx[:size])
        del self._rx[:size]
        return data

    def write(self, data: bytes) -> int:
        data = bytes(data)
        for start in range(0, len(data), 240):
            self._send(PKT_UART, bytes((self.baud,)) + data[start : start + 240])
        return len(data)

    def flush(self) -> None:
        self._serial.flush()

    def reset_input_buffer(self) -> None:
        self._pump(0)
        self._rx.clear()

    def reset_output_buffer(self) -> None:
        pass
