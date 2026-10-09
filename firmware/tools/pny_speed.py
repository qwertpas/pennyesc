"""Check sustained speed using full-width status telemetry and the GUI duty path."""
from __future__ import annotations

import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import statistics
import time

from pennycal import Control, EspBridge, Stm32Client
from pny_accel import stop


def check_speed(rows, duty, speed_limit):
    row = rows[-1]
    if row["faults"] or row["mode"] != 1 or row["duty"] != duty:
        return "output stopped"
    speeds = [r["velocity_turn32_per_s"] * 60 / 65536 * (1 if duty > 0 else -1)
              for r in rows]
    if abs(speeds[-1]) >= speed_limit:
        return "speed limit"
    peak = max(speeds)
    if len(speeds) >= 3 and peak > 10000 and max(speeds[-3:]) < peak - 5000:
        return "speed collapse"
    return None


def run(args):
    if not 0 < abs(args.duty) <= 799 or not 0.5 <= args.duration <= 3:
        raise ValueError("Use nonzero duty up to ±799 and duration 0.5–3 seconds")
    if args.repeats < 1 or args.rest < 3 or not 10000 <= args.speed_limit <= 60000:
        raise ValueError("Use repeats >= 1, rest >= 3 seconds and speed limit 10000–60000 rpm")
    data = {"settings": {k: str(v) if isinstance(v, Path) else v
                         for k, v in vars(args).items()}, "runs": []}
    if args.image:
        data["image_sha256"] = hashlib.sha256(args.image.read_bytes()).hexdigest()
    args.output.parent.mkdir(parents=True, exist_ok=True)

    def save():
        args.output.write_text(json.dumps(data, indent=2) + "\n")

    with EspBridge(args.port) as bridge:
        bridge.enter_bridge("app")
        bridge.serial.timeout = 0.002
        client = Stm32Client(bridge.serial, args.address)
        try:
            data["initial"] = asdict(stop(client))
            data["calibration"] = asdict(client.cal_info())
            for repeat in range(args.repeats):
                time.sleep(args.rest)
                rows = []
                trial = {"before": asdict(client.get_status()), "samples": rows}
                data["runs"].append(trial)
                response = client.set_control(Control(kf=args.duty, clip=799))
                if response.result:
                    raise RuntimeError(f"Duty command rejected: {response.result}")
                start = time.monotonic()
                reason = None
                while time.monotonic() - start < args.duration:
                    status = client.get_status(timeout=0.15)
                    rows.append({"t": time.monotonic() - start, **asdict(status)})
                    reason = check_speed(rows, args.duty, args.speed_limit)
                    if reason:
                        break
                    time.sleep(0.003)
                trial["reason"] = reason or "complete"
                trial["stopped"] = asdict(stop(client))
                tail = [r["velocity_turn32_per_s"] * 60 / 65536 for r in rows if r["t"] >= 0.3]
                trial["steady_mean_rpm"] = statistics.mean(tail) if tail else None
                save()
                print(f"run={repeat + 1} result={trial['reason']} "
                      f"steady_rpm={trial['steady_mean_rpm']}", flush=True)
                if reason:
                    raise RuntimeError(reason)
        finally:
            try:
                data["stopped"] = asdict(stop(client))
            finally:
                save()
                bridge.exit_bridge()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--address", type=int, default=1)
    parser.add_argument("--duty", type=int, default=500)
    parser.add_argument("--duration", type=float, default=3)
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--rest", type=float, default=3)
    parser.add_argument("--speed-limit", type=float, default=48000)
    parser.add_argument("--image", type=Path, help="record the flashed image hash")
    parser.add_argument("--output", type=Path, required=True)
    run(parser.parse_args())


if __name__ == "__main__":
    main()
