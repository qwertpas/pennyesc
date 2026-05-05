#!/usr/bin/env python3.11
from __future__ import annotations

import argparse
import os
import shlex
import struct
import sys
import time
from dataclasses import dataclass
from pathlib import Path

TOOLS_DIR = Path(__file__).resolve().parent
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from pennycal import EspBridge  # noqa: E402
from pnyproto import (  # noqa: E402
    CMD_DEBUG,
    CMD_GET_STATUS,
    CMD_SET_ADVANCE,
    CMD_SET_DUTY,
    CMD_ZERO_POSITION,
    DEBUG_CAPTURE_READ,
    DEBUG_CAPTURE_START,
    DEBUG_CAPTURE_STATUS,
    DEBUG_SET_OBSERVER,
    DEBUG_STEP_SET,
    DEBUG_STEP_TRANSITION,
    OBSERVER_RAW,
    FRAME_MAX_PAYLOAD,
    FRAME_START,
    RESULT_OK,
    decode_frame,
    encode_frame,
)


HOST_FRAME_BYTE_GAP_S = 0.0005


@dataclass(frozen=True)
class Status:
    result: int
    mode: int
    flags: int
    faults: int
    x: int
    y: int
    z: int
    angle_turn16: int
    position_turn32: int
    velocity_turn32_per_s: int
    duty: int
    mct_fault_count: int
    isr_us: int
    isr_max_us: int
    i2c_us: int
    i2c_start_us: int
    i2c_end_us: int
    isr_overrun_count: int
    i2c_timeout_count: int
    i2c_nack_count: int
    i2c_recover_count: int
    uart_overrun_errors: int

    @property
    def step(self) -> int:
        return self.position_turn32

    @property
    def transitions(self) -> int:
        return self.velocity_turn32_per_s


@dataclass(frozen=True)
class CaptureStatus:
    result: int
    active: int
    done: int
    sample_hz: int
    duration_ms: int
    elapsed_ms: int
    sample_count: int
    missed_count: int
    mct_fault_count: int


@dataclass(frozen=True)
class CaptureSample:
    index: int
    t_ms: float
    angle_turn16: int
    rpm: int


class StepperClient:
    def __init__(self, port, address: int) -> None:
        self.port = port
        self.address = address

    def _read_frame(self, expected_cmd: int, timeout: float = 0.5) -> bytes:
        deadline = time.monotonic() + timeout
        buf = bytearray()
        expected_len: int | None = None

        while time.monotonic() < deadline:
            chunk = self.port.read(1)
            if not chunk:
                continue
            byte = chunk[0]

            if not buf:
                if byte != FRAME_START:
                    continue
                buf.append(byte)
                continue

            if len(buf) < 3 and byte == FRAME_START:
                buf.clear()
                expected_len = None
                buf.append(byte)
                continue

            buf.append(byte)
            if len(buf) == 3:
                if buf[2] > FRAME_MAX_PAYLOAD:
                    buf.clear()
                    expected_len = None
                    continue
                expected_len = buf[2] + 4

            if expected_len is not None and len(buf) == expected_len:
                address, cmd, payload = decode_frame(bytes(buf))
                if address != self.address or cmd != expected_cmd:
                    buf.clear()
                    expected_len = None
                    continue
                return payload

        raise TimeoutError(f"timeout waiting for command 0x{expected_cmd:X}")

    def exchange(self, cmd: int, payload: bytes = b"", timeout: float = 1.0) -> bytes:
        frame = encode_frame(self.address, cmd, payload)
        for index, byte in enumerate(frame):
            self.port.write(bytes((byte,)))
            self.port.flush()
            if index + 1 < len(frame):
                time.sleep(HOST_FRAME_BYTE_GAP_S)
        return self._read_frame(cmd, timeout=timeout)

    def exchange_retry(self, cmd: int, payload: bytes = b"", timeout: float = 1.0, attempts: int = 3) -> bytes:
        last_error: TimeoutError | None = None
        for _ in range(attempts):
            try:
                return self.exchange(cmd, payload, timeout)
            except TimeoutError as exc:
                last_error = exc
                time.sleep(0.02)
        raise last_error or TimeoutError(f"timeout waiting for command 0x{cmd:X}")

    def get_status(self) -> Status:
        return Status(*struct.unpack("<BBBBhhhHiihHHHHHHIIIII", self.exchange_retry(CMD_GET_STATUS)))

    def set_duty(self, duty: int) -> Status:
        return Status(*struct.unpack("<BBBBhhhHiihHHHHHHIIIII", self.exchange_retry(CMD_SET_DUTY, struct.pack("<h", duty))))

    def set_advance(self, advance_deg: int) -> Status:
        return Status(*struct.unpack("<BBBBhhhHiihHHHHHHIIIII", self.exchange_retry(CMD_SET_ADVANCE, struct.pack("<h", advance_deg))))

    def zero_position(self) -> Status:
        return Status(*struct.unpack("<BBBBhhhHiihHHHHHHIIIII", self.exchange_retry(CMD_ZERO_POSITION)))

    def set_step(self, step: int) -> Status:
        return Status(
            *struct.unpack(
                "<BBBBhhhHiihHHHHHHIIIII",
                self.exchange(CMD_DEBUG, struct.pack("<Bb", DEBUG_STEP_SET, step)),
            )
        )

    def transition_step(self, step: int, blank_ms: int) -> Status:
        payload = struct.pack("<BbH", DEBUG_STEP_TRANSITION, step, blank_ms)
        return Status(*struct.unpack("<BBBBhhhHiihHHHHHHIIIII", self.exchange(CMD_DEBUG, payload, timeout=1.0)))

    def set_observer(self, lead_us: int, mode: int = OBSERVER_RAW) -> CaptureStatus:
        return self._decode_capture_status(
            self.exchange_retry(CMD_DEBUG, struct.pack("<BhB", DEBUG_SET_OBSERVER, lead_us, mode))
        )

    def capture_start(self, duty: int, advance_deg: int, duration_ms: int, sample_hz: int) -> CaptureStatus:
        payload = struct.pack("<BhhHH", DEBUG_CAPTURE_START, duty, advance_deg, duration_ms, sample_hz)
        return self._decode_capture_status(self.exchange(CMD_DEBUG, payload))

    def capture_status(self) -> CaptureStatus:
        return self._decode_capture_status(
            self.exchange_retry(CMD_DEBUG, struct.pack("<B", DEBUG_CAPTURE_STATUS), attempts=2)
        )

    def capture_read(self, offset: int, count: int) -> list[tuple[int, int]]:
        payload = self.exchange_retry(CMD_DEBUG, struct.pack("<BHB", DEBUG_CAPTURE_READ, offset, count))
        if len(payload) < 5 or payload[0] != DEBUG_CAPTURE_READ:
            raise RuntimeError("bad capture read response")
        result, resp_offset, resp_count = struct.unpack("<BHB", payload[1:5])
        if result != RESULT_OK:
            raise RuntimeError(f"capture read failed: result={result}")
        if resp_offset != offset:
            raise RuntimeError("capture read offset mismatch")
        expected_len = 5 + resp_count * 4
        if len(payload) != expected_len:
            raise RuntimeError("bad capture read length")
        samples = []
        pos = 5
        for _ in range(resp_count):
            samples.append(struct.unpack("<Hh", payload[pos : pos + 4]))
            pos += 4
        return samples

    @staticmethod
    def _decode_capture_status(payload: bytes) -> CaptureStatus:
        if len(payload) != struct.calcsize("<BBBBHHHHHH") or payload[0] != DEBUG_CAPTURE_STATUS:
            raise RuntimeError("bad capture status response")
        fields = struct.unpack("<BBBBHHHHHH", payload)
        return CaptureStatus(*fields[1:])


def parse_step(value: str) -> int:
    text = value.strip().lower()
    if text in {"off", "-1"}:
        return -1
    step = int(text, 0)
    if step < -1 or step > 5:
        raise argparse.ArgumentTypeError("step must be 0..5 or off")
    return step


def print_status(status: Status) -> None:
    print(
        "mode=%d flags=0x%02X faults=0x%02X step=%d transitions=%d raw=(%d,%d,%d) duty=%d mct_faults=%d "
        "isr=%dus max=%dus overruns=%d i2c=%dus phase=%d->%d "
        "timeouts=%d nacks=%d recovers=%d uart_ore=%d"
        % (
            status.mode,
            status.flags,
            status.faults,
            status.step,
            status.transitions,
            status.x,
            status.y,
            status.z,
            status.duty,
            status.mct_fault_count,
            status.isr_us,
            status.isr_max_us,
            status.isr_overrun_count,
            status.i2c_us,
            status.i2c_start_us,
            status.i2c_end_us,
            status.i2c_timeout_count,
            status.i2c_nack_count,
            status.i2c_recover_count,
            status.uart_overrun_errors,
        )
    )


def print_capture_status(status: CaptureStatus) -> None:
    print(
        "capture result=%d active=%d done=%d hz=%d duration_ms=%d elapsed_ms=%d samples=%d missed=%d "
        "mct_faults=%d"
        % (
            status.result,
            status.active,
            status.done,
            status.sample_hz,
            status.duration_ms,
            status.elapsed_ms,
            status.sample_count,
            status.missed_count,
            status.mct_fault_count,
        )
    )


def print_interactive_help() -> None:
    print("commands:")
    print("  status")
    print("  duty <value>")
    print("  advance <deg>")
    print("  zero")
    print("  observer <lead_us> [mode]")
    print("  step <0..5|off>")
    print("  transition <0..5|off> [blank_ms]")
    print("  cycle [start] [count] [delay_ms] [blank_ms]")
    print("  capture <duty> [advance_deg] [duration_ms] [sample_hz]")
    print("  stop")
    print("  off")
    print("  help")
    print("  quit")


def run_cycle(client: StepperClient, start: int, count: int, delay_ms: int, blank_ms: int) -> None:
    step = start % 6
    for _ in range(count):
        if blank_ms >= 0:
            status = client.transition_step(step, blank_ms)
        else:
            status = client.set_step(step)
        print_status(status)
        time.sleep(delay_ms / 1000.0)
        step = (step + 1) % 6


def run_capture(
    client: StepperClient,
    duty: int,
    advance_deg: int,
    duration_ms: int,
    sample_hz: int,
    csv_path: Path | None = None,
) -> None:
    samples: list[CaptureSample] = []
    try:
        status = client.capture_start(duty, advance_deg, duration_ms, sample_hz)
        print_capture_status(status)
        if status.result != RESULT_OK:
            return

        deadline = time.monotonic() + (duration_ms / 1000.0) + 1.0
        while status.active and time.monotonic() < deadline:
            time.sleep(0.025)
            status = client.capture_status()
        print_capture_status(status)

        if status.active:
            raise TimeoutError("capture did not finish")

        offset = 0
        while offset < status.sample_count:
            chunk = client.capture_read(offset, min(14, status.sample_count - offset))
            for angle_turn16, rpm in chunk:
                samples.append(CaptureSample(offset, offset * 1000.0 / status.sample_hz, angle_turn16, rpm))
                offset += 1
    finally:
        try:
            client.set_duty(0)
        except Exception:
            pass

    if csv_path is not None:
        import csv

        with csv_path.open("w", newline="") as fp:
            writer = csv.writer(fp)
            writer.writerow(["index", "t_ms", "angle_turn16", "rpm", "duty", "advance_deg", "sample_hz"])
            for sample in samples:
                writer.writerow(
                    [sample.index, "%.3f" % sample.t_ms, sample.angle_turn16, sample.rpm, duty, advance_deg, sample_hz]
                )


def run_interactive(client: StepperClient) -> None:
    print_interactive_help()
    while True:
        try:
            line = input("pnystep> ")
        except EOFError:
            print()
            break
        except KeyboardInterrupt:
            print()
            continue

        text = line.strip()
        if not text:
            continue

        try:
            parts = shlex.split(text)
        except ValueError as exc:
            print(f"parse error: {exc}")
            continue

        cmd = parts[0].lower()
        args = parts[1:]

        try:
            if cmd in {"quit", "exit"}:
                break
            if cmd == "help":
                print_interactive_help()
                continue
            if cmd == "status":
                print_status(client.get_status())
                continue
            if cmd == "duty":
                if len(args) != 1:
                    print("usage: duty <value>")
                    continue
                print_status(client.set_duty(int(args[0], 0)))
                continue
            if cmd == "advance":
                if len(args) != 1:
                    print("usage: advance <deg>")
                    continue
                print_status(client.set_advance(int(args[0], 0)))
                continue
            if cmd == "zero":
                if args:
                    print("usage: zero")
                    continue
                print_status(client.zero_position())
                continue
            if cmd == "observer":
                if len(args) not in {1, 2}:
                    print("usage: observer <lead_us> [mode]")
                    continue
                mode = int(args[1], 0) if len(args) == 2 else OBSERVER_RAW
                print_capture_status(client.set_observer(int(args[0], 0), mode))
                continue
            if cmd in {"step", "hold"}:
                if len(args) != 1:
                    print("usage: step <0..5|off>")
                    continue
                print_status(client.set_step(parse_step(args[0])))
                continue
            if cmd == "transition":
                if len(args) not in {1, 2}:
                    print("usage: transition <0..5|off> [blank_ms]")
                    continue
                step = parse_step(args[0])
                blank_ms = int(args[1], 0) if len(args) == 2 else 1
                print_status(client.transition_step(step, blank_ms))
                continue
            if cmd == "cycle":
                start = int(args[0], 0) if len(args) >= 1 else 0
                count = int(args[1], 0) if len(args) >= 2 else 12
                delay_ms = int(args[2], 0) if len(args) >= 3 else 300
                blank_ms = int(args[3], 0) if len(args) >= 4 else -1
                if len(args) > 4:
                    print("usage: cycle [start] [count] [delay_ms] [blank_ms]")
                    continue
                run_cycle(client, start, count, delay_ms, blank_ms)
                continue
            if cmd == "capture":
                if len(args) not in {1, 2, 3, 4}:
                    print("usage: capture <duty> [advance_deg] [duration_ms] [sample_hz]")
                    continue
                duty = int(args[0], 0)
                advance_deg = int(args[1], 0) if len(args) >= 2 else 90
                duration_ms = int(args[2], 0) if len(args) >= 3 else 800
                sample_hz = int(args[3], 0) if len(args) >= 4 else 500
                run_capture(client, duty, advance_deg, duration_ms, sample_hz)
                continue
            if cmd == "stop":
                print_status(client.set_duty(0))
                continue
            if cmd == "off":
                print_status(client.set_step(-1))
                print_status(client.set_duty(0))
                continue

            print(f"unknown command: {cmd}")
        except Exception as exc:  # noqa: BLE001
            print(f"error: {exc}")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Minimal six-step test helper")
    parser.add_argument("--port", default=os.environ.get("BRIDGE_PORT"), help="bridge serial port")
    parser.add_argument("--address", type=int, default=int(os.environ.get("ESC_ADDRESS", "1")), help="ESC address")
    parser.add_argument("--bridge-mode", default=os.environ.get("BRIDGE_MODE", "handoff"))

    sub = parser.add_subparsers(dest="cmd", required=True)
    sub.add_parser("status")

    duty = sub.add_parser("duty")
    duty.add_argument("value", type=int)

    advance = sub.add_parser("advance")
    advance.add_argument("value", type=int)

    sub.add_parser("zero")

    observer = sub.add_parser("observer")
    observer.add_argument("lead_us", type=int)
    observer.add_argument("--mode", type=int, default=OBSERVER_RAW)

    step = sub.add_parser("step")
    step.add_argument("value", type=parse_step)

    transition = sub.add_parser("transition")
    transition.add_argument("value", type=parse_step)
    transition.add_argument("--blank-ms", type=int, default=1)

    sub.add_parser("interactive")

    cycle = sub.add_parser("cycle")
    cycle.add_argument("--start", type=int, default=0)
    cycle.add_argument("--count", type=int, default=12)
    cycle.add_argument("--delay-ms", type=int, default=300)
    cycle.add_argument("--blank-ms", type=int, default=-1, help="-1 uses direct step command")
    cycle.add_argument("--duty", type=int, default=None)

    capture = sub.add_parser("capture")
    capture.add_argument("duty", type=int)
    capture.add_argument("--advance", type=int, default=90)
    capture.add_argument("--duration-ms", type=int, default=800)
    capture.add_argument("--sample-hz", type=int, default=500)
    capture.add_argument("--csv", type=Path, default=None)

    return parser


def main() -> None:
    parser = build_parser()
    args = parser.parse_args()
    if not args.port:
        raise SystemExit("pass --port or set BRIDGE_PORT")

    with EspBridge(args.port) as bridge:
        bridge.enter_bridge(args.bridge_mode)
        try:
            client = StepperClient(bridge.serial, args.address)

            if args.cmd == "status":
                print_status(client.get_status())
                return

            if args.cmd == "duty":
                print_status(client.set_duty(args.value))
                return

            if args.cmd == "advance":
                print_status(client.set_advance(args.value))
                return

            if args.cmd == "zero":
                print_status(client.zero_position())
                return

            if args.cmd == "observer":
                print_capture_status(client.set_observer(args.lead_us, args.mode))
                return

            if args.cmd == "step":
                print_status(client.set_step(args.value))
                return

            if args.cmd == "transition":
                print_status(client.transition_step(args.value, args.blank_ms))
                return

            if args.cmd == "interactive":
                run_interactive(client)
                return

            if args.cmd == "cycle":
                if args.duty is not None:
                    print_status(client.set_duty(args.duty))
                run_cycle(client, args.start, args.count, args.delay_ms, args.blank_ms)
                return

            if args.cmd == "capture":
                run_capture(client, args.duty, args.advance, args.duration_ms, args.sample_hz, args.csv)
                return
        finally:
            try:
                bridge.switch_bridge("off")
            except Exception:
                pass


if __name__ == "__main__":
    main()
