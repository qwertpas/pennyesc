from __future__ import annotations

import argparse
import struct
import time
from pathlib import Path

import serial

from pnyproto import BOOT_MAGIC, CMD_ENTER_BOOT, CMD_GET_STATUS, RESULT_OK, decode_frame, encode_frame

FLASH_BASE = 0x08000000
FLASH_PAGE_BYTES = 128
FLASH_APP_BYTES = 15744
ROM_ACK = 0x79
ROM_NACK = 0x1F
ROM_SYNC = 0x7F
ROM_GET = 0x00
ROM_READ_MEMORY = 0x11
ROM_GO = 0x21
ROM_WRITE_MEMORY = 0x31
ROM_EXTENDED_ERASE = 0x44
BRIDGE_IDLE_S = 3.0
BOOT_WINDOW_S = 2.0
UPLOAD_SWITCH_S = 0.4
APP_SYNC_RETRIES = 10


class BootError(RuntimeError):
    pass


class Progress:
    def __init__(self, label: str) -> None:
        self.label = label
        self.last_bucket = -1

    def update(self, index: int, total: int) -> None:
        if total <= 0:
            return
        percent = (index * 100) // total
        bucket = min(100, (percent // 5) * 5)
        if percent == 100:
            bucket = 100
        if bucket == self.last_bucket:
            return
        self.last_bucket = bucket
        print(f"{self.label} {bucket}%")


def xor_bytes(data: bytes) -> int:
    value = 0
    for byte in data:
        value ^= byte
    return value


def read_app_frame(port: serial.Serial, expected_cmd: int, address: int, timeout: float) -> bytes:
    deadline = time.monotonic() + timeout
    buf = bytearray()
    expected_len: int | None = None

    while time.monotonic() < deadline:
        chunk = port.read(1)
        if not chunk:
            continue
        byte = chunk[0]

        if not buf:
            if byte != 0xAA:
                continue
            buf.append(byte)
            continue

        if len(buf) < 3 and byte == 0xAA:
            buf.clear()
            expected_len = None
            buf.append(byte)
            continue

        buf.append(byte)
        if len(buf) == 3:
            expected_len = buf[2] + 4
        if expected_len is not None and len(buf) == expected_len:
            frame = bytes(buf)
            buf.clear()
            expected_len = None
            try:
                frame_address, frame_cmd, payload = decode_frame(frame)
            except ValueError:
                continue
            if frame_address == address and frame_cmd == expected_cmd:
                return payload

    raise TimeoutError(f"timeout waiting for app response 0x{expected_cmd:02X}")


def load_image(path: Path) -> tuple[bytes, bytes, list[int]]:
    image = path.read_bytes()
    if not image:
        raise BootError("image is empty")

    padded_len = ((len(image) + FLASH_PAGE_BYTES - 1) // FLASH_PAGE_BYTES) * FLASH_PAGE_BYTES
    if padded_len > FLASH_APP_BYTES:
        raise BootError(f"image is too large for app region: {padded_len} > {FLASH_APP_BYTES}")

    padded = image + (b"\xFF" * (padded_len - len(image)))
    pages = list(range(padded_len // FLASH_PAGE_BYTES))
    return image, padded, pages


class BootBridge:
    def __init__(self, port: str, baudrate: int = 115200) -> None:
        self.port_name = port
        self.serial = serial.Serial(port=port, baudrate=baudrate, timeout=0.05, write_timeout=1.0)

    def close(self) -> None:
        if self.serial.is_open:
            self.serial.close()

    def __enter__(self) -> "BootBridge":
        return self

    def __exit__(self, *_: object) -> None:
        self.close()

    def _read_line(self, timeout: float) -> str | None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            line = self.serial.readline()
            if not line:
                continue
            return line.decode("utf-8", errors="replace").strip()
        return None

    def sync_shell(self, timeout: float = 6.0) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.serial.reset_input_buffer()
            self.serial.write(b"\n")
            self.serial.write(b"ping\n")
            self.serial.flush()

            wait_deadline = time.monotonic() + 0.8
            while time.monotonic() < wait_deadline:
                line = self._read_line(0.2)
                if line is None:
                    continue
                if line == "pong":
                    return
            time.sleep(0.4)

        raise BootError("boot bridge shell did not respond")

    def enter_bridge(self, mode: str) -> None:
        expected = f"# bridge={mode}"
        self.sync_shell()
        self.serial.reset_input_buffer()
        self.serial.write(f"bridge {mode}\n".encode("utf-8"))
        self.serial.flush()

        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            line = self._read_line(0.25)
            if line is None:
                continue
            if line == expected:
                return
        raise BootError(f"bridge did not enter {mode} mode")


class Stm32Bootloader:
    def __init__(self, port: serial.Serial) -> None:
        self.port = port

    def _read_byte(self, timeout: float) -> int:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            chunk = self.port.read(1)
            if chunk:
                return chunk[0]
        raise TimeoutError("timeout waiting for bootloader byte")

    def _expect_ack(self, timeout: float = 1.0) -> None:
        value = self._read_byte(timeout)
        if value == ROM_ACK:
            return
        if value == ROM_NACK:
            raise BootError("bootloader returned NACK")
        raise BootError(f"unexpected bootloader byte 0x{value:02X}")

    def _send_command(self, command: int) -> None:
        self.port.write(bytes((command, command ^ 0xFF)))
        self.port.flush()
        self._expect_ack()

    def _send_address(self, address: int) -> None:
        payload = struct.pack(">I", address)
        self.port.write(payload + bytes((xor_bytes(payload),)))
        self.port.flush()
        self._expect_ack()

    def sync(self, retries: int = 25, delay: float = 0.1) -> None:
        self.port.reset_input_buffer()
        for _ in range(retries):
            self.port.write(bytes((ROM_SYNC,)))
            self.port.flush()
            try:
                self._expect_ack(timeout=0.2)
                return
            except (BootError, TimeoutError):
                time.sleep(delay)
        raise BootError("bootloader sync failed")

    def get(self) -> tuple[int, bytes]:
        self._send_command(ROM_GET)
        count = self._read_byte(1.0)
        version = self._read_byte(1.0)
        commands = bytes(self._read_byte(1.0) for _ in range(count))
        self._expect_ack()
        return version, commands

    def extended_erase(self, pages: list[int]) -> None:
        if not pages:
            raise BootError("no pages selected for erase")
        if len(pages) > 0x10000:
            raise BootError("too many pages for erase")

        self._send_command(ROM_EXTENDED_ERASE)
        count = len(pages) - 1
        payload = bytearray(struct.pack(">H", count))
        for page in pages:
            payload.extend(struct.pack(">H", page))
        payload.append(xor_bytes(payload))
        self.port.write(payload)
        self.port.flush()
        self._expect_ack(timeout=3.0)

    def write_memory(self, address: int, data: bytes) -> None:
        if not 1 <= len(data) <= 256:
            raise BootError("write chunk must be 1..256 bytes")

        self._send_command(ROM_WRITE_MEMORY)
        self._send_address(address)

        payload = bytearray()
        payload.append(len(data) - 1)
        payload.extend(data)
        payload.append(xor_bytes(payload))
        self.port.write(payload)
        self.port.flush()
        self._expect_ack(timeout=1.0)

    def read_memory(self, address: int, length: int) -> bytes:
        if not 1 <= length <= 256:
            raise BootError("read length must be 1..256 bytes")

        self._send_command(ROM_READ_MEMORY)
        self._send_address(address)
        n = length - 1
        self.port.write(bytes((n, n ^ 0xFF)))
        self.port.flush()
        self._expect_ack(timeout=1.0)

        data = bytearray()
        deadline = time.monotonic() + 1.0
        while len(data) < length and time.monotonic() < deadline:
            chunk = self.port.read(length - len(data))
            if chunk:
                data.extend(chunk)
                deadline = time.monotonic() + 1.0
        if len(data) != length:
            raise TimeoutError("timeout reading verify data")
        return bytes(data)

    def go(self, address: int) -> None:
        self._send_command(ROM_GO)
        self._send_address(address)


def verify_image(boot: Stm32Bootloader, base: int, expected: bytes) -> None:
    total = (len(expected) + FLASH_PAGE_BYTES - 1) // FLASH_PAGE_BYTES
    progress = Progress("verify")

    progress.update(0, total)
    for index, offset in enumerate(range(0, len(expected), FLASH_PAGE_BYTES), start=1):
        chunk = boot.read_memory(base + offset, min(FLASH_PAGE_BYTES, len(expected) - offset))
        if chunk != expected[offset : offset + len(chunk)]:
            raise BootError(f"verify failed at 0x{base + offset:08X}")
        progress.update(index, total)


def wait_for_app_status(port: str, address: int) -> bytes:
    with BootBridge(port) as bridge:
        bridge.enter_bridge("app")
        payload = encode_frame(address, CMD_GET_STATUS)
        bridge.serial.write(payload)
        bridge.serial.flush()
        return read_app_frame(bridge.serial, CMD_GET_STATUS, address, timeout=1.5)


def send_enter_boot_frames(port: serial.Serial, address: int, duration: float = BOOT_WINDOW_S) -> None:
    frame = encode_frame(address, CMD_ENTER_BOOT, struct.pack("<I", BOOT_MAGIC))
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        port.write(frame)
        port.flush()
        time.sleep(0.02)


def read_boot_ack(port: serial.Serial, address: int) -> int:
    payload = read_app_frame(port, CMD_ENTER_BOOT, address, timeout=0.8)
    if len(payload) < 1:
        raise BootError("boot ack payload invalid")
    return payload[0]


def open_bootloader_from_app(bridge: BootBridge, address: int) -> Stm32Bootloader:
    bridge.enter_bridge("upload")
    bridge.serial.write(encode_frame(address, CMD_ENTER_BOOT, struct.pack("<I", BOOT_MAGIC)))
    bridge.serial.flush()
    result = read_boot_ack(bridge.serial, address)
    if result != RESULT_OK:
        raise BootError(f"boot request failed: {result}")
    time.sleep(UPLOAD_SWITCH_S)
    boot = Stm32Bootloader(bridge.serial)
    boot.sync(retries=40, delay=0.05)
    return boot


def open_bootloader_after_reset(bridge: BootBridge, address: int) -> Stm32Bootloader:
    bridge.enter_bridge("upload")
    send_enter_boot_frames(bridge.serial, address)
    result = read_boot_ack(bridge.serial, address)
    if result != RESULT_OK:
        raise BootError(f"boot ack result={result}")
    time.sleep(UPLOAD_SWITCH_S)
    boot = Stm32Bootloader(bridge.serial)
    boot.sync(retries=40, delay=0.05)
    return boot


def enter_rom(port: str, address: int) -> None:
    with BootBridge(port) as bridge:
        input("hold NRST low, press Enter, then release NRST within 2 seconds...")
        boot = open_bootloader_after_reset(bridge, address)
        print("bootloader: synced")


def wait_for_app_ready(port: str, address: int) -> bytes:
    time.sleep(BRIDGE_IDLE_S)
    last_error: Exception | None = None
    for _ in range(APP_SYNC_RETRIES):
        try:
            return wait_for_app_status(port, address)
        except Exception as exc:
            last_error = exc
            time.sleep(0.3)
    if last_error is None:
        raise BootError("app status unavailable")
    raise BootError(f"app status unavailable: {last_error}")


def program_image(boot: Stm32Bootloader, padded: bytes, pages: list[int]) -> None:
    print(f"erase pages: {pages[0]}..{pages[-1]}")
    boot.extended_erase(pages)
    print("erase 100%")

    total = (len(padded) + FLASH_PAGE_BYTES - 1) // FLASH_PAGE_BYTES
    progress = Progress("write")

    progress.update(0, total)
    for index, offset in enumerate(range(0, len(padded), FLASH_PAGE_BYTES), start=1):
        boot.write_memory(FLASH_BASE + offset, padded[offset : offset + FLASH_PAGE_BYTES])
        progress.update(index, total)
    verify_image(boot, FLASH_BASE, padded)
    boot.go(FLASH_BASE)


def upload_image(port: str, image_path: Path, address: int) -> None:
    image, padded, pages = load_image(image_path)

    with BootBridge(port) as bridge:
        boot = open_bootloader_from_app(bridge, address)
        print("bootloader: synced")
        program_image(boot, padded, pages)

    wait_for_app_ready(port, address)
    print(f"verify ok image_len={len(image)}")


def recover_image(port: str, image_path: Path, address: int) -> None:
    image, padded, pages = load_image(image_path)

    with BootBridge(port) as bridge:
        input("hold NRST low, press Enter, then release NRST within 2 seconds...")
        boot = open_bootloader_after_reset(bridge, address)
        print("bootloader: synced")
        program_image(boot, padded, pages)

    wait_for_app_ready(port, address)
    print(f"verify ok image_len={len(image)}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("command", choices=("upload", "recover", "enter"))
    parser.add_argument("--port", default="/dev/cu.usbmodem101")
    parser.add_argument("--image", type=Path)
    parser.add_argument("--address", type=int, default=0)
    args = parser.parse_args()

    try:
        if args.command == "enter":
            enter_rom(args.port, args.address)
            return 0
        if args.image is None:
            raise BootError("--image is required")
        if args.command == "upload":
            upload_image(args.port, args.image, args.address)
        else:
            recover_image(args.port, args.image, args.address)
        return 0
    except (BootError, TimeoutError, serial.SerialException) as exc:
        print(f"error: {exc}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
