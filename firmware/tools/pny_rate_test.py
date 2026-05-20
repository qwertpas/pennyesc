from __future__ import annotations

import argparse
import csv
import struct
import time
from dataclasses import dataclass
from pathlib import Path

import serial

from pnyboot import BootBridge, find_serial_port
from pnyproto import (
    CMD_GET_POS_VEL,
    CMD_GET_STATUS,
    CMD_SEND_DUTY,
    CMD_SET_DUTY,
    CMD_STOP,
    decode_frame,
    encode_frame,
)

STATUS_FMT = "<BBBBhhhHiihHHHHHHIIIIIIH"
STATUS_LEN = struct.calcsize(STATUS_FMT)


@dataclass
class Stats:
    cycles: int = 0
    timeouts: int = 0
    crc_errors: int = 0
    bad_frames: int = 0
    overruns_start: int = 0
    overruns_end: int = 0


class FrameReader:
    def __init__(self, port: serial.Serial) -> None:
        self.port = port
        self.buf = bytearray()
        self.expected_len: int | None = None
        self.frames: list[tuple[int, int, bytes]] = []

    def read(self, address: int, cmd: int, timeout_s: float, stats: Stats) -> bytes:
        deadline = time.monotonic() + timeout_s

        while time.monotonic() < deadline:
            for index, (got_address, got_cmd, payload) in enumerate(self.frames):
                if got_address != address or got_cmd != cmd:
                    continue
                del self.frames[index]
                return payload

            if not self._read_available(stats):
                continue

        stats.timeouts += 1
        raise TimeoutError(f"timeout address={address} cmd=0x{cmd:X}")

    def _read_available(self, stats: Stats) -> bool:
        chunk = self.port.read(max(1, self.port.in_waiting))
        if not chunk:
            return False

        for byte in chunk:
            if not self.buf:
                if byte == 0xAA:
                    self.buf.append(byte)
                continue

            if len(self.buf) < 3 and byte == 0xAA:
                self._reset()
                self.buf.append(byte)
                continue

            self.buf.append(byte)
            if len(self.buf) == 3:
                if self.buf[2] > 64:
                    stats.bad_frames += 1
                    self._reset()
                    continue
                self.expected_len = self.buf[2] + 4

            if self.expected_len is not None and len(self.buf) == self.expected_len:
                try:
                    self.frames.append(decode_frame(bytes(self.buf)))
                except ValueError:
                    stats.crc_errors += 1
                self._reset()

        return True

    def _reset(self) -> None:
        self.buf.clear()
        self.expected_len = None


def read_frame(port: serial.Serial, address: int, cmd: int, timeout_s: float, stats: Stats) -> bytes:
    deadline = time.monotonic() + timeout_s
    buf = bytearray()
    expected_len: int | None = None

    while time.monotonic() < deadline:
        chunk = port.read(1)
        if not chunk:
            continue
        byte = chunk[0]

        if not buf:
            if byte == 0xAA:
                buf.append(byte)
            continue

        if len(buf) < 3 and byte == 0xAA:
            buf.clear()
            expected_len = None
            buf.append(byte)
            continue

        buf.append(byte)
        if len(buf) == 3:
            if buf[2] > 64:
                stats.bad_frames += 1
                buf.clear()
                expected_len = None
                continue
            expected_len = buf[2] + 4

        if expected_len is None or len(buf) != expected_len:
            continue

        frame = bytes(buf)
        buf.clear()
        expected_len = None
        try:
            got_address, got_cmd, payload = decode_frame(frame)
        except ValueError:
            stats.crc_errors += 1
            continue
        if got_address == address and got_cmd == cmd:
            return payload
        stats.bad_frames += 1

    stats.timeouts += 1
    raise TimeoutError(f"timeout address={address} cmd=0x{cmd:X}")


def exchange(port: serial.Serial, address: int, cmd: int, payload: bytes, timeout_s: float, stats: Stats) -> bytes:
    port.write(encode_frame(address, cmd, payload))
    port.flush()
    return read_frame(port, address, cmd, timeout_s, stats)


def uart_overruns(payload: bytes) -> int:
    if len(payload) < STATUS_LEN:
        return 0
    fields = struct.unpack(STATUS_FMT, payload[:STATUS_LEN])
    return fields[21]


def stop_all(port: serial.Serial, addresses: list[int], stats_by_address: dict[int, Stats]) -> None:
    reader = FrameReader(port)
    for address in addresses:
        try:
            port.write(encode_frame(address, CMD_STOP))
            port.flush()
            reader.read(address, CMD_STOP, 0.2, stats_by_address[address])
        except Exception as exc:
            print(f"stop address={address} failed: {exc}", flush=True)


def run(args: argparse.Namespace) -> int:
    if args.host_stream:
        return run_host_stream(args)
    return run_bridge(args)


def run_bridge(args: argparse.Namespace) -> int:
    port_name = find_serial_port(args.port)
    duration_ms = int(args.duration * 1000.0)
    expected = int(args.hz * args.duration)
    lines: list[str] = []
    done_marker = "# pollfast_done" if args.fast_poll else "# rate_done"

    with BootBridge(port_name, baudrate=args.usb_baud) as bridge:
        bridge.sync_shell(allow_reset=args.allow_reset)
        bridge.serial.reset_input_buffer()
        command = (
            f"{bridge.command_prefix}pollfast {duration_ms} {int(args.hz)} {args.duty} {args.spinup_ms}\n"
            if args.fast_poll
            else f"{bridge.command_prefix}rate {duration_ms} {int(args.hz)} {args.duty} {args.spinup_ms} {args.timeout_ms}\n"
        )
        bridge.serial.write(command.encode("utf-8"))
        bridge.serial.flush()

        deadline = time.monotonic() + args.duration + (args.spinup_ms / 1000.0) + 5.0
        while time.monotonic() < deadline:
            line = bridge.serial.readline()
            if not line:
                continue
            text = line.decode("utf-8", errors="replace").strip()
            if text:
                is_row = text.startswith("# rate_row ") or text.startswith("# pollfast_row ")
                if not is_row or not args.quiet_rows:
                    print(text, flush=True)
                lines.append(text)
            if text == done_marker:
                break
        else:
            print("rate runner timeout (bridge firmware must use PennyEscDebugBridge)", flush=True)
            return 1

    ok = False
    failed = False
    rows: list[list[str]] = []
    for line in lines:
        if args.fast_poll and line.startswith("# pollfast_row "):
            rows.append(line.removeprefix("# pollfast_row ").split(","))
            continue
        if not args.fast_poll and line.startswith("# rate_row "):
            rows.append(line.removeprefix("# rate_row ").split(","))
            continue
        if args.fast_poll and line.startswith("# pollfast_summary "):
            parts = dict(part.split("=", 1) for part in line.split()[1:] if "=" in part)
            ticks = int(parts.get("ticks", "0"))
            poll_fail = int(parts.get("poll_fail", "1"))
            timeouts = int(parts.get("timeouts", "1"))
            if ticks < expected or poll_fail != 0 or timeouts != 0:
                failed = True
            ok = True
            continue
        if args.fast_poll and line.startswith("# pollfast_address "):
            parts = dict(part.split("=", 1) for part in line.split()[1:] if "=" in part)
            fail = int(parts.get("fail", "1"))
            overruns = int(parts.get("uart_overruns_delta", "1"))
            if fail != 0 or overruns != 0:
                failed = True
            ok = True
            continue
        if args.fast_poll:
            continue
        if not line.startswith("# rate_address "):
            continue
        parts = dict(part.split("=", 1) for part in line.split()[1:] if "=" in part)
        cycles = int(parts.get("cycles", "0"))
        fail = int(parts.get("fail", "1"))
        overruns = int(parts.get("uart_overruns_delta", "1"))
        if cycles < expected or fail != 0 or overruns != 0:
            failed = True
        ok = True
    if args.csv_out is not None:
        args.csv_out.parent.mkdir(parents=True, exist_ok=True)
        with args.csv_out.open("w", newline="") as f:
            writer = csv.writer(f)
            if args.fast_poll:
                writer.writerow([
                    "sample",
                    "t_us",
                    "sample_age_us",
                    "loop_us",
                    "fresh",
                    "valid",
                    "position_turn32",
                    "velocity_turn32_per_s",
                    "rpm",
                ])
            else:
                writer.writerow(["sample", "t_us", "loop_us", "ok", "position_turn32", "velocity_turn32_per_s", "rpm"])
            writer.writerows(rows)
    return 0 if ok and not failed else 1


def run_host_stream(args: argparse.Namespace) -> int:
    port_name = find_serial_port(args.port)
    addresses = [int(value, 0) for value in args.addresses.split(",")]
    stats_by_address = {address: Stats() for address in addresses}
    period_s = 1.0 / args.hz
    set_duty_payload = struct.pack("<h", args.duty)

    with BootBridge(port_name, baudrate=args.usb_baud) as bridge:
        bridge.enter_bridge("handoff", allow_reset=args.allow_reset)
        port = bridge.serial
        port.timeout = args.read_timeout
        reader = FrameReader(port)

        for address in addresses:
            port.write(encode_frame(address, CMD_GET_STATUS))
            port.flush()
            stats_by_address[address].overruns_start = uart_overruns(
                reader.read(address, CMD_GET_STATUS, 0.5, stats_by_address[address])
            )
            port.write(encode_frame(address, CMD_SET_DUTY, set_duty_payload))
            port.flush()
            reader.read(address, CMD_SET_DUTY, 0.5, stats_by_address[address])

        start = time.monotonic()
        next_tick = start
        loops = 0
        late_loops = 0
        worst_loop_s = 0.0

        try:
            while time.monotonic() - start < args.duration:
                loop_start = time.monotonic()
                for address in addresses:
                    stats = stats_by_address[address]
                    if args.set_duty_each_loop:
                        port.write(encode_frame(address, CMD_SET_DUTY, set_duty_payload))
                    elif args.send_duty_each_loop:
                        port.write(encode_frame(address, CMD_SEND_DUTY, set_duty_payload))
                    port.write(encode_frame(address, CMD_GET_POS_VEL))
                    port.flush()
                    if args.set_duty_each_loop:
                        reader.read(address, CMD_SET_DUTY, args.exchange_timeout, stats)
                    reader.read(address, CMD_GET_POS_VEL, args.exchange_timeout, stats)
                    stats.cycles += 1

                loops += 1
                elapsed = time.monotonic() - loop_start
                worst_loop_s = max(worst_loop_s, elapsed)
                next_tick += period_s
                sleep_s = next_tick - time.monotonic()
                if sleep_s > 0:
                    time.sleep(sleep_s)
                else:
                    late_loops += 1
                    next_tick = time.monotonic()
        finally:
            stop_all(port, addresses, stats_by_address)

        total_s = time.monotonic() - start
        for address in addresses:
            port.write(encode_frame(address, CMD_GET_STATUS))
            port.flush()
            stats_by_address[address].overruns_end = uart_overruns(
                reader.read(address, CMD_GET_STATUS, 0.5, stats_by_address[address])
            )

    ok = True
    print(
        f"summary duration_s={total_s:.3f} loops={loops} late_loops={late_loops} "
        f"worst_loop_ms={worst_loop_s * 1000.0:.3f}",
        flush=True,
    )
    for address in addresses:
        stats = stats_by_address[address]
        rate = stats.cycles / total_s if total_s > 0 else 0.0
        overrun_delta = stats.overruns_end - stats.overruns_start
        print(
            f"address={address} cycles={stats.cycles} rate_hz={rate:.1f} "
            f"timeouts={stats.timeouts} crc_errors={stats.crc_errors} bad_frames={stats.bad_frames} "
            f"uart_overruns_delta={overrun_delta}",
            flush=True,
        )
        if rate < args.hz or stats.timeouts != 0 or stats.crc_errors != 0 or overrun_delta != 0:
            ok = False

    return 0 if ok else 1


def main() -> int:
    parser = argparse.ArgumentParser(description="PennyESC two-address UART rate test")
    parser.add_argument("--port", default="auto")
    parser.add_argument("--addresses", default="1,3")
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--hz", type=float, default=500.0)
    parser.add_argument("--usb-baud", type=int, default=921600)
    parser.add_argument("--duty", type=int, default=0)
    parser.add_argument("--send-duty-each-loop", action="store_true")
    parser.add_argument("--set-duty-each-loop", action="store_true")
    parser.add_argument("--read-timeout", type=float, default=0.002)
    parser.add_argument("--exchange-timeout", type=float, default=0.02)
    parser.add_argument("--csv-out", type=Path)
    parser.add_argument("--fast-poll", action="store_true")
    parser.add_argument("--spinup-ms", type=int, default=0)
    parser.add_argument("--timeout-ms", type=int, default=5)
    parser.add_argument("--quiet-rows", action="store_true")
    parser.add_argument("--allow-reset", action="store_true")
    parser.add_argument("--host-stream", action="store_true")
    args = parser.parse_args()
    if args.send_duty_each_loop and args.set_duty_each_loop:
        parser.error("--send-duty-each-loop and --set-duty-each-loop are mutually exclusive")
    return run(args)


if __name__ == "__main__":
    raise SystemExit(main())
