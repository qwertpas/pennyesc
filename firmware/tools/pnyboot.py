"""Update PennyESC application firmware over UART through the PennyESC bridge."""

from __future__ import annotations

import argparse
import struct
import time
from pathlib import Path

import serial

from pnybridge import BAUD_APP, BAUD_BOOT, Bridge, BridgeError
from pnyproto import BOOT_MAGIC, CMD_ENTER_BOOT, CMD_GET_STATUS, RESULT_OK, encode_frame, find_serial_port, read_frame

FLASH_BASE = 0x08000580
FLASH_PAGE_BYTES = 128
FLASH_APP_BYTES = 14336
ERASE_BATCH_PAGES = 8
ROM_ACK = 0x79
ROM_NACK = 0x1F
ROM_SYNC = 0x7F
ROM_GET = 0x00
ROM_READ_MEMORY = 0x11
ROM_GO = 0x21
ROM_WRITE_MEMORY = 0x31
ROM_EXTENDED_ERASE = 0x44
TARGET_STATUS_TIMEOUT_S = 0.25
APP_READY_UPLOAD_S = 8.0
APP_READY_RECOVER_S = 6.0
APP_READY_RETRY_S = 0.1
BOOT_HANDOFF_S = 2.0
BOOT_WINDOW_S = 15.0
APP_DISCOVERY_TIMEOUT_S = 0.03


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
        print(f"{self.label} {bucket}%", flush=True)


def xor_bytes(data: bytes) -> int:
    value = 0
    for byte in data:
        value ^= byte
    return value


def xor_bytes(data: bytes) -> int:
    value = 0
    for byte in data:
        value ^= byte
    return value


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


def send_enter_boot_frames(port, address: int, count: int = 1) -> None:
    frame = encode_frame(address, CMD_ENTER_BOOT, BOOT_MAGIC.to_bytes(4, "little"))
    for _ in range(count):
        port.write(frame)
        port.flush()


def read_boot_ack(port, address: int, timeout: float = 0.8) -> bool:
    payload = read_frame(port, address, CMD_ENTER_BOOT, timeout)
    return len(payload) >= 1 and payload[0] == RESULT_OK


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

    def _send_address(self, address: int, strict: bool = True) -> None:
        payload = struct.pack(">I", address)
        self.port.write(payload + bytes((xor_bytes(payload),)))
        self.port.flush()
        if strict:
            self._expect_ack()
            return
        try:
            self._expect_ack(timeout=0.2)
        except (BootError, TimeoutError):
            pass

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
        self._expect_ack(timeout=20.0)

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
        self._send_address(address, strict=False)


def verify_image(boot: Stm32Bootloader, base: int, expected: bytes) -> None:
    total = (len(expected) + FLASH_PAGE_BYTES - 1) // FLASH_PAGE_BYTES
    progress = Progress("verify")

    progress.update(0, total)
    for index, offset in enumerate(range(0, len(expected), FLASH_PAGE_BYTES), start=1):
        chunk = boot.read_memory(base + offset, min(FLASH_PAGE_BYTES, len(expected) - offset))
        if chunk != expected[offset : offset + len(chunk)]:
            raise BootError(f"verify failed at 0x{base + offset:08X}")
        progress.update(index, total)


def request_app_status(bridge: Bridge, address: int, timeout: float) -> bytes:
    bridge.set_baud(BAUD_APP)
    bridge.reset_input_buffer()
    bridge.write(encode_frame(address, CMD_GET_STATUS))
    try:
        return read_frame(bridge, address, CMD_GET_STATUS, timeout)
    except TimeoutError as exc:
        raise BootError(f"target address {address} not found") from exc


def wait_for_app_ready(bridge: Bridge, address: int, *, timeout_s: float, retry_delay: float = APP_READY_RETRY_S,
                       status_timeout: float = TARGET_STATUS_TIMEOUT_S) -> bytes:
    deadline = time.monotonic() + timeout_s
    while True:
        try:
            return request_app_status(bridge, address, status_timeout)
        except BootError as exc:
            if time.monotonic() + retry_delay >= deadline:
                raise BootError(f"app status unavailable: {exc}") from None
            time.sleep(retry_delay)


def find_devices(bridge: Bridge) -> list[int]:
    found = []
    for address in range(16):
        try:
            request_app_status(bridge, address, APP_DISCOVERY_TIMEOUT_S)
            found.append(address)
        except BootError:
            pass
    return found


def open_bootloader_from_reset(bridge: Bridge) -> Stm32Bootloader:
    bridge.set_baud(BAUD_BOOT)
    input("press Enter, then power-cycle the ESC while the uploader is waiting...")
    boot = Stm32Bootloader(bridge)
    boot.sync(retries=int(BOOT_WINDOW_S / 0.25), delay=0.05)
    return boot


def open_bootloader_from_app(bridge: Bridge, address: int) -> Stm32Bootloader:
    request_app_status(bridge, address, TARGET_STATUS_TIMEOUT_S)
    bridge.reset_input_buffer()
    send_enter_boot_frames(bridge, address)
    try:
        if not read_boot_ack(bridge, address, timeout=0.5):
            raise BootError(f"ESC {address} refused the bootloader request")
    except TimeoutError:
        raise BootError(f"ESC {address} did not answer the bootloader request") from None
    bridge.set_baud(BAUD_BOOT)
    boot = Stm32Bootloader(bridge)
    try:
        boot.sync(retries=int(BOOT_HANDOFF_S / 0.25), delay=0.05)
    except BootError:
        raise BootError("bootloader did not respond; power-cycle the ESC and use recover") from None
    return boot


def check_connection(port: str, address: int) -> bytes:
    with Bridge(port) as bridge:
        return request_app_status(bridge, address, TARGET_STATUS_TIMEOUT_S)


def scan_addresses(port: str) -> list[int]:
    with Bridge(port) as bridge:
        return find_devices(bridge)


def enter_rom(port: str) -> None:
    with Bridge(port) as bridge:
        open_bootloader_from_reset(bridge)
        print("bootloader: synced")


def probe_path(port: str, address: int, rounds: int) -> None:
    if rounds < 1:
        raise BootError("probe rounds must be >= 1")
    with Bridge(port) as bridge:
        for index in range(1, rounds + 1):
            wait_for_app_ready(bridge, address, timeout_s=APP_READY_RECOVER_S)
            print(f"probe round {index}: app ready", flush=True)
            boot = open_bootloader_from_app(bridge, address)
            print(f"probe round {index}: bootloader synced", flush=True)
            sp, pc = struct.unpack("<II", boot.read_memory(FLASH_BASE, 8))
            if (sp & 0x2FF80000) != 0x20000000:
                raise BootError(f"probe round {index}: invalid stack 0x{sp:08X}")
            if (pc & 1) == 0 or pc < FLASH_BASE or pc >= (FLASH_BASE + FLASH_APP_BYTES):
                raise BootError(f"probe round {index}: invalid reset vector 0x{pc:08X}")
            boot.go(FLASH_BASE)
            wait_for_app_ready(bridge, address, timeout_s=APP_READY_RECOVER_S)
            print(f"probe round {index}: app resumed", flush=True)


def program_image(boot: Stm32Bootloader, padded: bytes, pages: list[int]) -> None:
    print(f"erase pages: {pages[0]}..{pages[-1]}")
    for start in range(0, len(pages), ERASE_BATCH_PAGES):
        boot.extended_erase(pages[start : start + ERASE_BATCH_PAGES])
        done = min(len(pages), start + ERASE_BATCH_PAGES)
        print(f"erase {(done * 100) // len(pages)}%")

    total = (len(padded) + FLASH_PAGE_BYTES - 1) // FLASH_PAGE_BYTES
    progress = Progress("write")

    progress.update(0, total)
    for index, offset in enumerate(range(0, len(padded), FLASH_PAGE_BYTES), start=1):
        boot.write_memory(FLASH_BASE + offset, padded[offset : offset + FLASH_PAGE_BYTES])
        progress.update(index, total)
    verify_image(boot, FLASH_BASE, padded)


def _flash(boot: Stm32Bootloader, padded: bytes, pages: list[int]) -> None:
    try:
        program_image(boot, padded, pages)
        boot.go(FLASH_BASE)
    except (BootError, TimeoutError) as exc:
        raise BootError(f"{exc}; power-cycle the ESC and use recover") from None


def upload_image(port: str, image_path: Path, address: int, verify_address: int | None = None) -> None:
    image, padded, pages = load_image(image_path)
    with Bridge(port) as bridge:
        boot = open_bootloader_from_app(bridge, address)
        print("bootloader: synced")
        _flash(boot, padded, pages)
        wait_for_app_ready(bridge, address if verify_address is None else verify_address, timeout_s=APP_READY_UPLOAD_S)
    print(f"verify ok image_len={len(image)}")


def recover_image(port: str, image_path: Path, address: int) -> None:
    image, padded, pages = load_image(image_path)
    with Bridge(port) as bridge:
        boot = open_bootloader_from_reset(bridge)
        print("bootloader: synced")
        _flash(boot, padded, pages)
        wait_for_app_ready(bridge, address, timeout_s=APP_READY_RECOVER_S, retry_delay=0.3)
    print(f"verify ok image_len={len(image)}")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("upload", "recover", "check", "enter", "probe", "scan", "status"))
    parser.add_argument("--port", default="auto")
    parser.add_argument("--image", type=Path)
    parser.add_argument("--address", type=int, default=0)
    parser.add_argument("--verify-address", type=int)
    parser.add_argument("--rounds", type=int, default=3)
    args = parser.parse_args(argv)

    try:
        args.port = find_serial_port(args.port)
        if args.command == "enter":
            enter_rom(args.port)
        elif args.command in ("status", "check"):
            print(check_connection(args.port, args.address).hex(), flush=True)
        elif args.command == "probe":
            probe_path(args.port, args.address, args.rounds)
        elif args.command == "scan":
            for address in scan_addresses(args.port):
                print(address, flush=True)
        elif args.image is None:
            raise BootError("--image is required")
        elif args.command == "upload":
            upload_image(args.port, args.image, args.address, args.verify_address)
        else:
            recover_image(args.port, args.image, args.address)
        return 0
    except (BootError, BridgeError, TimeoutError, serial.SerialException) as exc:
        print(f"error: {exc}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
