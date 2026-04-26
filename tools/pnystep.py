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
    CMD_GET_STATUS,
    CMD_SET_DUTY,
    CMD_STEP_SET,
    CMD_STEP_TRANSITION,
    FRAME_MAX_PAYLOAD,
    FRAME_START,
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
    position_crad: int
    velocity_crads: int
    duty: int

    @property
    def step(self) -> int:
        return self.position_crad

    @property
    def transitions(self) -> int:
        return self.velocity_crads


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

    def exchange(self, cmd: int, payload: bytes = b"", timeout: float = 0.5) -> bytes:
        frame = encode_frame(self.address, cmd, payload)
        for index, byte in enumerate(frame):
            self.port.write(bytes((byte,)))
            self.port.flush()
            if index + 1 < len(frame):
                time.sleep(HOST_FRAME_BYTE_GAP_S)
        return self._read_frame(cmd, timeout=timeout)

    def get_status(self) -> Status:
        return Status(*struct.unpack("<BBBBhhhHiih", self.exchange(CMD_GET_STATUS)))

    def set_duty(self, duty: int) -> Status:
        return Status(*struct.unpack("<BBBBhhhHiih", self.exchange(CMD_SET_DUTY, struct.pack("<h", duty))))

    def set_step(self, step: int) -> Status:
        return Status(*struct.unpack("<BBBBhhhHiih", self.exchange(CMD_STEP_SET, struct.pack("<b", step))))

    def transition_step(self, step: int, blank_ms: int) -> Status:
        payload = struct.pack("<bH", step, blank_ms)
        return Status(*struct.unpack("<BBBBhhhHiih", self.exchange(CMD_STEP_TRANSITION, payload, timeout=1.0)))


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
        "mode=%d flags=0x%02X faults=0x%02X step=%d transitions=%d raw=(%d,%d,%d) duty=%d"
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
        )
    )


def print_interactive_help() -> None:
    print("commands:")
    print("  status")
    print("  duty <value>")
    print("  step <0..5|off>")
    print("  transition <0..5|off> [blank_ms]")
    print("  cycle [start] [count] [delay_ms] [blank_ms]")
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

    sub = parser.add_subparsers(dest="cmd", required=True)
    sub.add_parser("status")

    duty = sub.add_parser("duty")
    duty.add_argument("value", type=int)

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

    return parser


def main() -> None:
    parser = build_parser()
    args = parser.parse_args()
    if not args.port:
        raise SystemExit("pass --port or set BRIDGE_PORT")

    with EspBridge(args.port) as bridge:
        bridge.enter_bridge("app")
        try:
            client = StepperClient(bridge.serial, args.address)

            if args.cmd == "status":
                print_status(client.get_status())
                return

            if args.cmd == "duty":
                print_status(client.set_duty(args.value))
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
        finally:
            try:
                bridge.exit_bridge()
            except Exception:
                pass


if __name__ == "__main__":
    main()
