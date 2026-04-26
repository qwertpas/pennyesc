from __future__ import annotations

import argparse
import struct
import time
from pathlib import Path

import serial

from pnyproto import BOOT_MAGIC, CMD_ENTER_BOOT, CMD_GET_STATUS, CMD_SET_QUIET, RESULT_OK, decode_frame, encode_frame

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
SHELL_SYNC_S = 1.0
TARGET_STATUS_TIMEOUT_S = 0.25
APP_READY_UPLOAD_S = 8.0
APP_READY_RECOVER_S = 6.0
APP_READY_RETRY_S = 0.1
BOOT_HANDOFF_S = 1.5
BOOT_WINDOW_S = 8.0
APP_DISCOVERY_TIMEOUT_S = 0.15
APP_QUIET_MS = 15000


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


def send_enter_boot_frames(port: serial.Serial, address: int, count: int = 3) -> None:
    frame = encode_frame(address, CMD_ENTER_BOOT, BOOT_MAGIC.to_bytes(4, "little"))
    for _ in range(count):
        port.write(frame)
        port.flush()
        time.sleep(0.02)


def read_boot_ack(port: serial.Serial, address: int, timeout: float = 0.8) -> bool:
    payload = read_app_frame(port, CMD_ENTER_BOOT, address, timeout)
    return len(payload) >= 1 and payload[0] == RESULT_OK


def read_app_result(port: serial.Serial, expected_cmd: int, address: int, timeout: float) -> int:
    payload = read_app_frame(port, expected_cmd, address, timeout)
    if len(payload) < 1:
        raise BootError(f"empty response for app command 0x{expected_cmd:02X}")
    return payload[0]


class BootBridge:
    def __init__(self, port: str, baudrate: int = 115200) -> None:
        self.port_name = port
        self.baudrate = baudrate
        self.command_prefix = ""
        self.serial = self._open_serial()

    def _open_serial(self) -> serial.Serial:
        port = serial.Serial(port=self.port_name, baudrate=self.baudrate, timeout=0.05, write_timeout=1.0)
        port.dtr = False
        port.rts = False
        return port

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

    def _try_sync_shell(self, timeout: float) -> str | None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.serial.reset_input_buffer()
            self.serial.write(b"!#ping\n")
            self.serial.flush()

            wait_deadline = time.monotonic() + 0.25
            while time.monotonic() < wait_deadline:
                line = self._read_line(0.05)
                if line is None:
                    continue
                if line == "pong":
                    self.command_prefix = "!#"
                    return "!#"

            self.serial.reset_input_buffer()
            self.serial.write(b"\n")
            self.serial.write(b"ping\n")
            self.serial.flush()

            wait_deadline = time.monotonic() + 0.25
            while time.monotonic() < wait_deadline:
                line = self._read_line(0.05)
                if line is None:
                    continue
                if line == "pong":
                    self.command_prefix = ""
                    return ""
            time.sleep(0.05)

        return None

    def reset_bridge(self) -> None:
        self.close()

        try:
            pulse = serial.Serial(port=self.port_name, baudrate=1200, timeout=0.2, write_timeout=0.2)
            pulse.close()
        except serial.SerialException:
            pass

        deadline = time.monotonic() + 8.0
        last_error: Exception | None = None
        while time.monotonic() < deadline:
            time.sleep(0.4)
            try:
                self.serial = self._open_serial()
                time.sleep(0.2)
                self.command_prefix = ""
                return
            except serial.SerialException as exc:
                last_error = exc

        raise BootError(f"boot bridge reset failed: {last_error}")

    def sync_shell(self, timeout: float = 6.0, allow_reset: bool = True) -> None:
        if self._try_sync_shell(timeout) is not None:
            return

        if allow_reset:
            self.reset_bridge()
            if self._try_sync_shell(timeout) is not None:
                return

        raise BootError("bridge shell unavailable")

    def _enter_bridge(self, mode: str, sync: bool, timeout: float, allow_reset: bool) -> None:
        expected = f"# bridge={mode}"
        if sync:
            self.sync_shell(timeout=timeout, allow_reset=allow_reset)
        self.serial.reset_input_buffer()
        self.serial.reset_output_buffer()
        prefix = self.command_prefix
        self.serial.write(f"{prefix}bridge {mode}\n".encode("utf-8"))
        self.serial.flush()

        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            line = self._read_line(0.25)
            if line is None:
                continue
            if line == expected:
                self.command_prefix = "" if mode == "off" else "!#"
                return
        raise BootError(f"bridge did not enter {mode} mode")

    def enter_bridge(self, mode: str, timeout: float = 6.0, allow_reset: bool = True) -> None:
        self._enter_bridge(mode, sync=True, timeout=timeout, allow_reset=allow_reset)

    def switch_bridge(self, mode: str) -> None:
        self._enter_bridge(mode, sync=False, timeout=0.0, allow_reset=False)


def app_command(port: serial.Serial, address: int, cmd: int, payload: bytes = b"", timeout: float = 0.8) -> int:
    port.reset_input_buffer()
    port.write(encode_frame(address, cmd, payload))
    port.flush()
    return read_app_result(port, cmd, address, timeout)


def find_devices(bridge: BootBridge, allow_reset: bool = False) -> list[int]:
    found: list[int] = []

    bridge.enter_bridge("handoff", timeout=SHELL_SYNC_S, allow_reset=allow_reset)
    for address in range(16):
        try:
            bridge.serial.reset_input_buffer()
            bridge.serial.write(encode_frame(address, CMD_GET_STATUS))
            bridge.serial.flush()
            read_app_frame(bridge.serial, CMD_GET_STATUS, address, timeout=APP_DISCOVERY_TIMEOUT_S)
            found.append(address)
        except TimeoutError:
            continue

    return found


def quiet_other_devices(bridge: BootBridge, target_address: int, allow_reset: bool = False) -> list[int]:
    quieted = [address for address in find_devices(bridge, allow_reset=allow_reset) if address != target_address]

    for address in quieted:
        result = app_command(
            bridge.serial,
            address,
            CMD_SET_QUIET,
            int(APP_QUIET_MS).to_bytes(2, "little"),
            timeout=0.8,
        )
        if result != RESULT_OK:
            raise BootError(f"device {address} refused quiet mode")

    if quieted:
        time.sleep(0.05)
    return quieted


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


def request_app_status(bridge: BootBridge, address: int, timeout: float) -> bytes:
    bridge.serial.reset_input_buffer()
    bridge.serial.write(encode_frame(address, CMD_GET_STATUS))
    bridge.serial.flush()
    try:
        return read_app_frame(bridge.serial, CMD_GET_STATUS, address, timeout=timeout)
    except TimeoutError as exc:
        raise BootError(f"target address {address} not found") from exc


def wait_for_app_status(
    port: str,
    address: int,
    *,
    shell_timeout: float = SHELL_SYNC_S,
    allow_reset: bool = False,
    status_timeout: float = TARGET_STATUS_TIMEOUT_S,
) -> bytes:
    with BootBridge(port) as bridge:
        bridge.enter_bridge("handoff", timeout=shell_timeout, allow_reset=allow_reset)
        return request_app_status(bridge, address, status_timeout)


def prompt_bootloader_reset() -> None:
    input("press Enter, then reset or power-cycle the STM32 while the uploader is waiting...")


def open_bootloader_from_reset(bridge: BootBridge) -> Stm32Bootloader:
    quiet_other_devices(bridge, -1, allow_reset=True)
    bridge.enter_bridge("upload", allow_reset=True)
    prompt_bootloader_reset()
    boot = Stm32Bootloader(bridge.serial)
    boot.sync(retries=int(BOOT_WINDOW_S / 0.05), delay=0.05)
    return boot


def open_bootloader_from_app(bridge: BootBridge, address: int) -> Stm32Bootloader:
    quieted = quiet_other_devices(bridge, address, allow_reset=False)
    if quieted:
        print(f"quiet devices: {quieted}", flush=True)

    bridge.enter_bridge("handoff", timeout=SHELL_SYNC_S, allow_reset=False)
    request_app_status(bridge, address, TARGET_STATUS_TIMEOUT_S)
    send_enter_boot_frames(bridge.serial, address)
    try:
        read_boot_ack(bridge.serial, address, timeout=0.1)
    except TimeoutError:
        pass

    bridge.switch_bridge("upload")
    boot = Stm32Bootloader(bridge.serial)
    deadline = time.monotonic() + BOOT_HANDOFF_S

    while time.monotonic() < deadline:
        try:
            boot.sync(retries=2, delay=0.05)
            return boot
        except (BootError, TimeoutError):
            time.sleep(0.05)

    raise BootError("normal upload path unavailable, use uart_recover")


def enter_rom(port: str, address: int) -> None:
    with BootBridge(port) as bridge:
        boot = open_bootloader_from_reset(bridge)
        print("bootloader: synced")


def wait_for_app_ready(
    port: str,
    address: int,
    *,
    timeout_s: float,
    retry_delay: float,
    shell_timeout: float,
    allow_reset: bool,
    status_timeout: float,
) -> bytes:
    deadline = time.monotonic() + timeout_s
    with BootBridge(port) as bridge:
        bridge.enter_bridge("handoff", timeout=shell_timeout, allow_reset=allow_reset)
        last_error: Exception | None = None
        while time.monotonic() < deadline:
            try:
                return request_app_status(bridge, address, status_timeout)
            except Exception as exc:
                last_error = exc
                if time.monotonic() + retry_delay >= deadline:
                    break
                time.sleep(retry_delay)
    if last_error is None:
        raise BootError("app status unavailable")
    raise BootError(f"app status unavailable: {last_error}")


def wait_for_app_ready_bridge(
    bridge: BootBridge,
    address: int,
    *,
    timeout_s: float,
    retry_delay: float,
    status_timeout: float,
) -> bytes:
    deadline = time.monotonic() + timeout_s
    last_error: Exception | None = None
    bridge.switch_bridge("handoff")
    while time.monotonic() < deadline:
        try:
            return request_app_status(bridge, address, status_timeout)
        except Exception as exc:
            last_error = exc
            if time.monotonic() + retry_delay >= deadline:
                break
            time.sleep(retry_delay)
    if last_error is None:
        raise BootError("app status unavailable")
    raise BootError(f"app status unavailable: {last_error}")


def check_connection(port: str, address: int) -> bytes:
    return wait_for_app_status(
        port,
        address,
        shell_timeout=SHELL_SYNC_S,
        allow_reset=False,
        status_timeout=TARGET_STATUS_TIMEOUT_S,
    )


def probe_path(port: str, address: int, rounds: int) -> None:
    if rounds < 1:
        raise BootError("probe rounds must be >= 1")

    for index in range(1, rounds + 1):
        print(f"probe round {index}: waiting for app", flush=True)
        wait_for_app_ready(
            port,
            address,
            timeout_s=APP_READY_RECOVER_S,
            retry_delay=0.3,
            shell_timeout=6.0,
            allow_reset=True,
            status_timeout=1.5,
        )
        print(f"probe round {index}: app ready", flush=True)

        with BootBridge(port) as bridge:
            print(f"probe round {index}: requesting bootloader", flush=True)
            boot = open_bootloader_from_app(bridge, address)
            print(f"probe round {index}: bootloader synced", flush=True)
            vector = boot.read_memory(FLASH_BASE, 8)
            sp, pc = struct.unpack("<II", vector)
            if (sp & 0x2FF80000) != 0x20000000:
                raise BootError(f"probe round {index}: invalid stack 0x{sp:08X}")
            if (pc & 1) == 0 or pc < FLASH_BASE or pc >= (FLASH_BASE + FLASH_APP_BYTES):
                raise BootError(f"probe round {index}: invalid reset vector 0x{pc:08X}")
            boot.go(FLASH_BASE)

        print(f"probe round {index}: waiting for app resume", flush=True)
        wait_for_app_ready(
            port,
            address,
            timeout_s=APP_READY_RECOVER_S,
            retry_delay=0.3,
            shell_timeout=6.0,
            allow_reset=True,
            status_timeout=1.5,
        )
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


def upload_image(port: str, image_path: Path, address: int) -> None:
    image, padded, pages = load_image(image_path)

    with BootBridge(port) as bridge:
        boot = open_bootloader_from_app(bridge, address)
        print("bootloader: synced")
        program_image(boot, padded, pages)
        boot.go(FLASH_BASE)

    wait_for_app_ready(
        port,
        address,
        timeout_s=APP_READY_UPLOAD_S,
        retry_delay=APP_READY_RETRY_S,
        shell_timeout=SHELL_SYNC_S,
        allow_reset=True,
        status_timeout=TARGET_STATUS_TIMEOUT_S,
    )

    print(f"verify ok image_len={len(image)}")


def recover_image(port: str, image_path: Path, address: int) -> None:
    image, padded, pages = load_image(image_path)

    with BootBridge(port) as bridge:
        boot = open_bootloader_from_reset(bridge)
        print("bootloader: synced")
        program_image(boot, padded, pages)
        boot.go(FLASH_BASE)

    wait_for_app_ready(
        port,
        address,
        timeout_s=APP_READY_RECOVER_S,
        retry_delay=0.3,
        shell_timeout=6.0,
        allow_reset=True,
        status_timeout=1.5,
    )

    print(f"verify ok image_len={len(image)}")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("command", choices=("upload", "recover", "check", "enter", "probe", "status"))
    parser.add_argument("--port", default="/dev/cu.usbmodem101")
    parser.add_argument("--image", type=Path)
    parser.add_argument("--address", type=int, default=0)
    parser.add_argument("--rounds", type=int, default=3)
    args = parser.parse_args(argv)

    try:
        if args.command == "enter":
            enter_rom(args.port, args.address)
            return 0
        if args.command == "status":
            payload = check_connection(args.port, args.address)
            print(payload.hex(), flush=True)
            return 0
        if args.command == "check":
            payload = check_connection(args.port, args.address)
            print(payload.hex(), flush=True)
            return 0
        if args.command == "probe":
            probe_path(args.port, args.address, args.rounds)
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
