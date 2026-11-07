"""End-to-end tests of pnyboot through the real Bridge client and a byte-level bridge model."""

import struct
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import pnybridge  # noqa: E402
from pnyboot import (  # noqa: E402
    FLASH_APP_BYTES,
    FLASH_BASE,
    FLASH_PAGE_BYTES,
    BootError,
    Stm32Bootloader,
    check_connection,
    load_image,
    scan_addresses,
    upload_image,
    xor_bytes,
)
from pnybridge import BAUD_APP, BAUD_BOOT, Bridge, BridgeError, cobs_decode, cobs_encode, encode_packet  # noqa: E402
from pnyproto import CMD_ENTER_BOOT, CMD_GET_STATUS, crc8, decode_frame, encode_frame  # noqa: E402


class FakeEsc:
    """One ESC: an app answering frames at 921600 and the UART bootloader at 115200."""

    def __init__(self, address: int = 1) -> None:
        self.address = address
        self.running = True
        self.flash = bytearray(b"\x5a" * FLASH_APP_BYTES)
        self.state = "idle"
        self.erased: list[int] = []

    def uart(self, data: bytes, baud: int) -> bytes:
        if baud == BAUD_APP:
            if not self.running:
                return b""
            address, cmd, payload = decode_frame(data)
            if address != self.address:
                return b""
            if cmd == CMD_GET_STATUS:
                return encode_frame(address, cmd, bytes(60))
            if cmd == CMD_ENTER_BOOT:
                self.running = False
                return encode_frame(address, cmd, b"\x00")
            return b""
        return self.boot(data)

    def boot(self, data: bytes) -> bytes:
        if self.running:
            return b""
        if self.state == "idle":
            if data == b"\x7f":
                return b"\x79"
            if len(data) != 2 or data[0] ^ data[1] != 0xFF:
                raise AssertionError(f"bad command {data.hex()}")
            self.command = data[0]
            self.state = "erase" if self.command == 0x44 else "address"
            return b"\x79"
        if self.state == "address":
            self.target = struct.unpack(">I", data[:4])[0] - FLASH_BASE
            self.state = {0x31: "write", 0x11: "read", 0x21: "idle"}[self.command]
            if self.command == 0x21:
                self.running = True
            return b"\x79"
        if self.state == "erase":
            count = struct.unpack(">H", data[:2])[0] + 1
            for index in range(count):
                page = struct.unpack(">H", data[2 + index * 2 : 4 + index * 2])[0]
                self.erased.append(page)
                self.flash[page * FLASH_PAGE_BYTES : (page + 1) * FLASH_PAGE_BYTES] = b"\xff" * FLASH_PAGE_BYTES
            self.state = "idle"
            return b"\x79"
        if self.state == "write":
            self.flash[self.target : self.target + data[0] + 1] = data[1:-1]
            self.state = "idle"
            return b"\x79"
        if self.state == "read":
            self.state = "idle"
            return b"\x79" + bytes(self.flash[self.target : self.target + data[0] + 1])
        raise AssertionError(self.state)


class FakeBridgeSerial:
    """A pyserial port wired to a model of the bridge firmware."""

    def __init__(self, esc: FakeEsc, version: int = 2, answers: bool = True) -> None:
        self.esc = esc
        self.version = version
        self.answers = answers
        self.is_open = True
        self.output = bytearray()
        self.encoded = bytearray()
        self.uart_writes: list[tuple[int, bytes]] = []
        self.dtr = self.rts = True

    @property
    def in_waiting(self) -> int:
        return len(self.output)

    def read(self, size: int = 1) -> bytes:
        data = bytes(self.output[:size])
        del self.output[:size]
        return data

    def write(self, data: bytes) -> int:
        for byte in data:
            if byte:
                self.encoded.append(byte)
                continue
            packet = cobs_decode(bytes(self.encoded)) if self.encoded else None
            self.encoded.clear()
            if packet and crc8(packet[:-1]) == packet[-1]:
                self.handle(packet[0], packet[1:-1])
        return len(data)

    def send(self, kind: int, payload: bytes) -> None:
        self.output.extend(encode_packet(kind, payload))

    def handle(self, kind: int, data: bytes) -> None:
        if not self.answers:
            return
        if kind == pnybridge.PKT_HELLO:
            self.send(pnybridge.PKT_STATE, bytes((0, self.version, 12, 13, 255, 11, 0xDC, 0x05, 0)) + b"qtpy-esp32s3")
        elif kind == pnybridge.PKT_UART:
            self.uart_writes.append((data[0], bytes(data[1:])))
            reply = self.esc.uart(bytes(data[1:]), data[0])
            if reply:
                self.send(pnybridge.PKT_UART_RX, bytes((data[0],)) + reply)

    def flush(self) -> None:
        pass

    def close(self) -> None:
        self.is_open = False


def patched(fake: FakeBridgeSerial):
    return mock.patch("pnybridge.serial.Serial", return_value=fake)


class BridgeTests(unittest.TestCase):
    def test_cobs_round_trip(self) -> None:
        for data in (b"", b"\x00", b"\x01\x00\x02", bytes(range(256)) * 2, bytes(range(1, 256)) * 3):
            encoded = cobs_encode(data)
            self.assertNotIn(0, encoded)
            self.assertEqual(cobs_decode(encoded), data)

    def test_hello_reports_board_and_pins(self) -> None:
        with patched(FakeBridgeSerial(FakeEsc())):
            with Bridge("/dev/test") as bridge:
                self.assertEqual((bridge.state.board, bridge.state.rx, bridge.state.tx, bridge.state.gnd),
                                 ("qtpy-esp32s3", 12, 13, -1))

    def test_old_bridge_firmware_is_explained(self) -> None:
        with patched(FakeBridgeSerial(FakeEsc(), answers=False)):
            with self.assertRaisesRegex(BridgeError, "flash the bridge"):
                Bridge("/dev/test")
        with patched(FakeBridgeSerial(FakeEsc(), version=1)):
            with self.assertRaisesRegex(BridgeError, "version 1"):
                Bridge("/dev/test")

    def test_replies_at_another_baud_are_dropped(self) -> None:
        fake = FakeBridgeSerial(FakeEsc())
        with patched(fake), Bridge("/dev/test") as bridge:
            fake.send(pnybridge.PKT_UART_RX, bytes((BAUD_BOOT,)) + b"\x79")
            bridge.write(encode_frame(1, CMD_GET_STATUS))
            self.assertEqual(bridge.read(4)[:2], bytes((0xAA, 0x11)))


class BootToolTests(unittest.TestCase):
    def test_xor_bytes(self) -> None:
        self.assertEqual(xor_bytes(bytes((0x01, 0x02, 0x03))), 0x00)

    def test_go_writes_command_and_address(self) -> None:
        fake = FakeBridgeSerial(FakeEsc())
        with patched(fake), Bridge("/dev/test") as bridge:
            fake.esc.running = False
            bridge.set_baud(BAUD_BOOT)
            Stm32Bootloader(bridge).go(FLASH_BASE)
        self.assertEqual(b"".join(data for _, data in fake.uart_writes), bytes((0x21, 0xDE, 0x08, 0x00, 0x05, 0x80, 0x8D)))
        self.assertTrue(fake.esc.running)

    def test_load_image_pads_to_page(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "image.bin"
            path.write_bytes(bytes(range(200)))
            image, padded, pages = load_image(path)
        self.assertEqual(len(padded), 2 * FLASH_PAGE_BYTES)
        self.assertEqual(pages, [0, 1])
        self.assertTrue(padded.startswith(image))
        self.assertEqual(set(padded[len(image):]), {0xFF})

    def test_load_image_rejects_oversize(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "image.bin"
            path.write_bytes(b"\x00" * (FLASH_APP_BYTES + 1))
            with self.assertRaises(RuntimeError):
                load_image(path)

    def test_check_and_scan(self) -> None:
        with patched(FakeBridgeSerial(FakeEsc(3))):
            self.assertEqual(len(check_connection("/dev/test", 3)), 60)
            with self.assertRaisesRegex(BootError, "target address 4 not found"):
                check_connection("/dev/test", 4)
            self.assertEqual(scan_addresses("/dev/test"), [3])

    def test_upload_flashes_verifies_and_resumes(self) -> None:
        fake = FakeBridgeSerial(FakeEsc(2))
        image = bytes(range(256)) * 3
        with tempfile.TemporaryDirectory() as tmp, patched(fake):
            path = Path(tmp) / "fw.bin"
            path.write_bytes(image)
            upload_image("/dev/test", path, 2)
        self.assertEqual(bytes(fake.esc.flash[: len(image)]), image)
        self.assertEqual(fake.esc.erased, list(range(6)))
        self.assertTrue(fake.esc.running)
        self.assertEqual({baud for baud, _ in fake.uart_writes}, {BAUD_APP, BAUD_BOOT})

    def test_upload_to_a_silent_address_fails_before_erasing(self) -> None:
        fake = FakeBridgeSerial(FakeEsc(2))
        with tempfile.TemporaryDirectory() as tmp, patched(fake):
            path = Path(tmp) / "fw.bin"
            path.write_bytes(b"\x01" * 64)
            with self.assertRaisesRegex(BootError, "target address 5 not found"):
                upload_image("/dev/test", path, 5)
        self.assertEqual(fake.esc.erased, [])


if __name__ == "__main__":
    unittest.main()
