"""Compare commutation timing using short, internally clocked acceleration runs."""
from __future__ import annotations

import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import random
import struct
import time

import numpy as np

from pennycal import EspBridge, Stm32Client
from pnyproto import (CMD_DEBUG, DEBUG_CAPTURE_START, DEBUG_CAPTURE_STATUS,
                      DEBUG_CAPTURE_READ, DEBUG_SET_OBSERVER, OBSERVER_AB_FAST)


def capture_status(client, payload=bytes([DEBUG_CAPTURE_STATUS])):
    data = client.exchange(CMD_DEBUG, payload, timeout=0.15)
    values = struct.unpack("<BBBB6H", data)
    result = dict(zip(("subcmd", "result", "active", "done", "sample_hz",
                       "duration_ms", "elapsed_ms", "sample_count", "missed_count",
                       "mct_fault_count"), values))
    if result["subcmd"] != DEBUG_CAPTURE_STATUS or result["result"]:
        raise RuntimeError(f"Capture rejected: {result}")
    return result


def read_capture(client, count):
    samples = []
    while len(samples) < count:
        want = min(14, count - len(samples))
        data = client.exchange(CMD_DEBUG, struct.pack("<BHB", DEBUG_CAPTURE_READ,
                                                     len(samples), want))
        cmd, result, offset, received = struct.unpack("<BBHB", data[:5])
        if (cmd != DEBUG_CAPTURE_READ or result or offset != len(samples)
                or received != want or len(data) != 5 + 4 * received):
            raise RuntimeError("Invalid capture block")
        samples.extend(struct.iter_unpack("<Hh", data[5:]))
    return samples


def set_lead(client, lead):
    data = client.exchange(CMD_DEBUG, struct.pack("<BhB", DEBUG_SET_OBSERVER,
                                                 lead, OBSERVER_AB_FAST))
    if len(data) < 2 or data[1]:
        raise RuntimeError("Observer setting rejected")


def stop(client):
    client.brake()
    deadline = time.monotonic() + 4
    quiet = 0
    while time.monotonic() < deadline:
        time.sleep(0.05)
        status = client.get_status(timeout=0.15)
        if status.faults:
            raise RuntimeError(f"ESC fault: {status}")
        quiet = quiet + 1 if abs(status.velocity_rpm) < 100 else 0
        if quiet >= 3:
            return status
    raise RuntimeError("Motor has not stopped")


def acceleration(samples, duty, width=4000):
    """Mean acceleration between persistent crossings of matched RPM bands.

    Nine-sample centered smoothing suppresses observer noise. A crossing must
    persist for five samples. Unreached bands are omitted, never scored as zero.
    """
    rpm = np.array([s[1] for s in samples], dtype=float) * (1 if duty > 0 else -1)
    if len(rpm) < 13:
        return {}
    rpm = np.convolve(rpm, np.ones(9) / 9, mode="valid")
    ticks = np.arange(len(rpm), dtype=float) + 5
    crossings = {}
    for edge in range(2000, 30001, width):
        for i in range(1, len(rpm) - 4):
            if rpm[i - 1] < edge <= min(rpm[i:i + 5]):
                crossings[edge] = ticks[i - 1] + (edge - rpm[i - 1]) / (rpm[i] - rpm[i - 1])
                break
    return {f"{low}-{low + width}": width * 1000 / (crossings[low + width] - t)
            for low, t in crossings.items()
            if low + width in crossings and crossings[low + width] > t}


def speed_collapsed(samples, duty):
    rpm = np.array([s[1] for s in samples], dtype=float) * (1 if duty > 0 else -1)
    if not len(rpm):
        return False
    peak = np.maximum.accumulate(rpm)
    return bool(np.any((peak > 10000) & (rpm < peak - 3000)))


def run(args):
    if not 0 < abs(args.duty) <= 450 or not 20 <= args.duration <= 200:
        raise ValueError("Use nonzero duty up to ±450 and duration 20–200 ms")
    if args.repeats < 1 or args.rest < 1:
        raise ValueError("Use at least one repeat and one second rest")
    trials = []
    data = {"sample_hz": 1000, "settings": vars(args).copy(), "trials": trials}
    data["settings"]["output"] = str(args.output)
    if args.image:
        data["settings"]["image"] = str(args.image)
        data["image_sha256"] = hashlib.sha256(args.image.read_bytes()).hexdigest()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    rng = random.Random(args.seed)
    with EspBridge(args.port) as bridge:
        bridge.enter_bridge("app")
        bridge.serial.timeout = 0.002
        client = Stm32Client(bridge.serial, args.address)
        try:
            data["initial"] = asdict(stop(client))
            data["calibration"] = asdict(client.cal_info())
            for repeat in range(args.repeats):
                settings = [(lead, advance) for lead in args.leads for advance in args.advances]
                rng.shuffle(settings)
                for lead, advance in settings:
                    stop(client)
                    time.sleep(args.rest)
                    set_lead(client, lead)
                    before = client.get_status()
                    if before.faults:
                        raise RuntimeError(f"ESC fault: {before}")
                    capture = capture_status(client, struct.pack("<BhhHH", DEBUG_CAPTURE_START,
                                             args.duty, advance, args.duration, 1000))
                    deadline = time.monotonic() + args.duration / 1000 + 0.3
                    capped = False
                    peak_speed = 0
                    collapsed = False
                    while capture["active"]:
                        if time.monotonic() > deadline:
                            raise TimeoutError("Capture did not finish")
                        time.sleep(0.003)
                        speed = client.get_pos_vel().velocity_turn32_per_s * 60 / 65536
                        speed *= 1 if args.duty > 0 else -1
                        peak_speed = max(peak_speed, speed)
                        collapsed = peak_speed > 10000 and speed < peak_speed - 3000
                        if collapsed:
                            client.brake()
                        if abs(speed) >= 28000:
                            client.brake()
                            capped = True
                        capture = capture_status(client)
                    after = client.get_status()
                    samples = read_capture(client, capture["sample_count"])
                    collapsed = collapsed or speed_collapsed(samples, args.duty)
                    stop(client)
                    trial = dict(repeat=repeat, lead_us=lead, advance_deg=advance,
                                 duty=args.duty, capture=capture, speed_limit=capped, collapsed=collapsed,
                                 before=asdict(before), after=asdict(after), samples=samples,
                                 acceleration_rpm_s={} if collapsed else acceleration(samples, args.duty))
                    trials.append(trial)
                    args.output.write_text(json.dumps(data, indent=2) + "\n")
                    peak = max(abs(s[1]) for s in samples) if samples else 0
                    print(f"repeat={repeat + 1} lead={lead} advance={advance} peak={peak} "
                          f"accel={ {k: round(v) for k, v in trial['acceleration_rpm_s'].items()} }",
                          flush=True)
                    if (after.faults or capture["missed_count"] or
                            after.mct_fault_count != before.mct_fault_count):
                        raise RuntimeError("Fault or lost capture samples")
        finally:
            try:
                data["stopped"] = asdict(stop(client))
                set_lead(client, 145)
                client.set_advance_deg(90)
            finally:
                args.output.write_text(json.dumps(data, indent=2) + "\n")
                bridge.exit_bridge()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--address", type=int, default=1)
    parser.add_argument("--duty", type=int, default=400)
    parser.add_argument("--duration", type=int, default=200, help="milliseconds, at most 200")
    parser.add_argument("--leads", type=int, nargs="+", default=[140, 145, 150])
    parser.add_argument("--advances", type=int, nargs="+", default=[90])
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--rest", type=float, default=3, help="rest seconds between runs")
    parser.add_argument("--seed", type=int, default=17)
    parser.add_argument("--image", type=Path, help="record the verified flashed image hash")
    parser.add_argument("--output", type=Path, required=True)
    run(parser.parse_args())


if __name__ == "__main__":
    main()
