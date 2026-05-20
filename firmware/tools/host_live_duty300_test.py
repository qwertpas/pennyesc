from __future__ import annotations

import argparse
import csv
import statistics
import time
from pathlib import Path

from pennycal import Control, EspBridge, Stm32Client


DEFAULT_OUT = Path(
    "/Users/chris/Code/flywheeljumper/main/firmware/forcetorquemotor2/data/host_live_duty300_test.csv"
)


def rpm(velocity_turn32_per_s: float) -> float:
    return velocity_turn32_per_s * 60.0 / 65536.0


def run(args: argparse.Namespace) -> int:
    rows: list[tuple[int, int, int, int, int]] = []
    rtt_us: list[int] = []
    ok = 0
    fail = 0
    sum_v = 0
    min_v = 2**31 - 1
    max_v = -(2**31)
    first_pos: int | None = None
    last_pos: int | None = None
    end = None

    with EspBridge(args.port, baudrate=args.baud) as bridge:
        bridge.enter_bridge("app")
        port = bridge.serial
        port.timeout = args.read_timeout
        client = Stm32Client(port, address=args.address)

        start = client.get_status(timeout=0.5, attempts=3)
        print(
            f"start mode={start.mode} duty={start.duty} faults=0x{start.faults:02X} "
            f"mct={start.mct_fault_count} isr_max={start.isr_max_us} "
            f"over={start.isr_overrun_count} uart_ore={start.uart_overrun_errors}",
            flush=True,
        )

        t1 = 0.0
        t2 = 0.0
        try:
            client.set_control(Control(clip=args.duty))
            st = client.set_duty(args.duty)
            print(
                f"set_duty result={st.result} mode={st.mode} duty={st.duty} faults=0x{st.faults:02X}",
                flush=True,
            )

            time.sleep(args.spinup)
            t1 = time.monotonic()
            deadline = t1 + args.duration

            while time.monotonic() < deadline:
                request_t = time.monotonic()
                tus = int((request_t - t1) * 1_000_000)
                try:
                    pv = client.get_pos_vel(timeout=args.exchange_timeout)
                    reply_t = time.monotonic()
                    sample_rtt_us = int((reply_t - request_t) * 1_000_000)
                    rtt_us.append(sample_rtt_us)
                    ok += 1
                    rows.append((ok - 1, tus, sample_rtt_us, pv.position_turn32, pv.velocity_turn32_per_s))
                    sum_v += pv.velocity_turn32_per_s
                    min_v = min(min_v, pv.velocity_turn32_per_s)
                    max_v = max(max_v, pv.velocity_turn32_per_s)
                    if first_pos is None:
                        first_pos = pv.position_turn32
                    last_pos = pv.position_turn32
                except Exception:
                    fail += 1

            t2 = time.monotonic()
        finally:
            for i in range(8):
                try:
                    st = client.stop()
                    print(
                        f"stop{i} result={st.result} mode={st.mode} duty={st.duty} faults=0x{st.faults:02X}",
                        flush=True,
                    )
                    if st.duty == 0:
                        break
                except Exception as exc:
                    print(f"stop{i} failed {exc}", flush=True)
                time.sleep(0.05)

            time.sleep(0.2)
            end = client.get_status(timeout=0.5, attempts=3)
            bridge.exit_bridge()

    args.out.parent.mkdir(parents=True, exist_ok=True)
    with args.out.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["sample", "t_us", "rtt_us", "position_turn32", "velocity_turn32_per_s"])
        writer.writerows(rows)

    elapsed = t2 - t1 if t2 else args.duration
    avg = sum_v / ok if ok else 0.0
    moved = (last_pos - first_pos) if first_pos is not None and last_pos is not None else 0
    rtt_avg = statistics.fmean(rtt_us) if rtt_us else 0.0
    rtt_max = max(rtt_us) if rtt_us else 0
    rtt_p95 = sorted(rtt_us)[int(len(rtt_us) * 0.95)] if rtt_us else 0

    print(
        f"rate duty={args.duty} ok={ok} fail={fail} elapsed_s={elapsed:.6f} "
        f"rate_hz={ok / elapsed:.1f} rpm_avg={rpm(avg):.1f} "
        f"rpm_min={rpm(min_v) if ok else 0:.1f} rpm_max={rpm(max_v) if ok else 0:.1f} "
        f"rtt_avg_us={rtt_avg:.0f} rtt_p95_us={rtt_p95} rtt_max_us={rtt_max} moved={moved}"
    )
    if end is not None:
        print(
            f"end mode={end.mode} duty={end.duty} faults=0x{end.faults:02X} "
            f"mct_delta={end.mct_fault_count - start.mct_fault_count} "
            f"isr_max={end.isr_max_us} "
            f"over_delta={end.isr_overrun_count - start.isr_overrun_count} "
            f"uart_ore_delta={end.uart_overrun_errors - start.uart_overrun_errors}"
        )
    print(f"saved {args.out} rows={len(rows)}")
    return 0 if ok > 0 and end is not None and end.duty == 0 else 1


def main() -> int:
    parser = argparse.ArgumentParser(description="Host-observed PennyESC live pos/vel duty test")
    parser.add_argument("--port", default="/dev/cu.usbmodem101")
    parser.add_argument("--address", type=int, default=1)
    parser.add_argument("--baud", type=int, default=921600)
    parser.add_argument("--duty", type=int, default=300)
    parser.add_argument("--duration", type=float, default=0.5)
    parser.add_argument("--spinup", type=float, default=0.2)
    parser.add_argument("--read-timeout", type=float, default=0.002)
    parser.add_argument("--exchange-timeout", type=float, default=0.002)
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT)
    return run(parser.parse_args())


if __name__ == "__main__":
    raise SystemExit(main())
