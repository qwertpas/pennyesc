from __future__ import annotations

import argparse
import csv
import statistics
import time
from pathlib import Path

from pennycal import Control, EspBridge, Stm32Client


DEFAULT_OUT = Path(
    "/Users/chris/Code/flywheeljumper/main/firmware/forcetorquemotor2/data/duty300_rpm_samples.csv"
)


def run(args: argparse.Namespace) -> int:
    rows = []
    summaries = []

    with EspBridge(args.port, baudrate=args.baud) as bridge:
        bridge.enter_bridge("app")
        bridge.serial.timeout = 0.01
        client = Stm32Client(bridge.serial, args.address)

        for run_index in range(args.runs):
            samples = []
            start_t = 0.0
            try:
                client.set_control(Control(clip=args.duty))
                st = client.set_duty(args.duty)
                print(
                    f"run{run_index} set_duty result={st.result} mode={st.mode} duty={st.duty} "
                    f"faults=0x{st.faults:02X}",
                    flush=True,
                )
                time.sleep(args.spinup)
                start_t = time.monotonic()

                for sample_index in range(args.samples):
                    st = client.get_status(timeout=0.1, attempts=2)
                    samples.append(st)
                    t_us = int((time.monotonic() - start_t) * 1_000_000)
                    rows.append(
                        [
                            run_index,
                            sample_index,
                            t_us,
                            st.mode,
                            st.duty,
                            st.velocity_rpm,
                            st.velocity_turn32_per_s,
                            st.isr_us,
                            st.isr_max_us,
                            st.isr_overrun_count,
                            st.uart_overrun_errors,
                            st.tmag_sample_count,
                            st.tmag_sample_dt_us,
                            st.faults,
                        ]
                    )
                    print(
                        f"run{run_index}_status{sample_index} mode={st.mode} duty={st.duty} "
                        f"rpm={st.velocity_rpm:.0f} isr={st.isr_us} isr_max={st.isr_max_us} "
                        f"over={st.isr_overrun_count} uart_ore={st.uart_overrun_errors} "
                        f"tmag_dt={st.tmag_sample_dt_us} tmag_samples={st.tmag_sample_count} "
                        f"faults=0x{st.faults:02X}",
                        flush=True,
                    )
                    time.sleep(args.interval)
            finally:
                for i in range(5):
                    try:
                        st = client.stop()
                        print(
                            f"run{run_index} stop{i} result={st.result} mode={st.mode} "
                            f"duty={st.duty} faults=0x{st.faults:02X}",
                            flush=True,
                        )
                        if st.duty == 0:
                            break
                    except Exception as exc:
                        print(f"run{run_index} stop{i} failed {exc}", flush=True)
                    time.sleep(0.05)
                final = client.get_status(timeout=0.5, attempts=3)
                print(
                    f"run{run_index} final mode={final.mode} duty={final.duty} rpm={final.velocity_rpm:.0f} "
                    f"isr_max={final.isr_max_us} over={final.isr_overrun_count} "
                    f"uart_ore={final.uart_overrun_errors} tmag_dt={final.tmag_sample_dt_us} "
                    f"tmag_samples={final.tmag_sample_count} faults=0x{final.faults:02X}",
                    flush=True,
                )

            if samples:
                rpms = [abs(st.velocity_rpm) for st in samples]
                summary = {
                    "run": run_index,
                    "samples": len(samples),
                    "rpm_avg": statistics.fmean(rpms),
                    "rpm_min": min(rpms),
                    "rpm_max": max(rpms),
                    "isr_max": max(st.isr_max_us for st in samples),
                    "overruns_max": max(st.isr_overrun_count for st in samples),
                    "uart_ore_max": max(st.uart_overrun_errors for st in samples),
                    "tmag_dt_avg": statistics.fmean(st.tmag_sample_dt_us for st in samples),
                    "tmag_dt_min": min(st.tmag_sample_dt_us for st in samples),
                    "tmag_dt_max": max(st.tmag_sample_dt_us for st in samples),
                    "faults_any": max(st.faults for st in samples),
                }
                summaries.append(summary)
                print(
                    f"run{run_index} summary samples={summary['samples']} "
                    f"rpm_avg={summary['rpm_avg']:.0f} rpm_min={summary['rpm_min']:.0f} "
                    f"rpm_max={summary['rpm_max']:.0f} isr_max={summary['isr_max']} "
                    f"overruns_max={summary['overruns_max']} uart_ore_max={summary['uart_ore_max']} "
                    f"tmag_dt_avg={summary['tmag_dt_avg']:.0f} "
                    f"tmag_dt_min={summary['tmag_dt_min']} tmag_dt_max={summary['tmag_dt_max']} "
                    f"faults_any=0x{summary['faults_any']:02X}",
                    flush=True,
                )

            time.sleep(args.between_runs)

        bridge.exit_bridge()

    args.out.parent.mkdir(parents=True, exist_ok=True)
    with args.out.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "run",
                "sample",
                "t_us",
                "mode",
                "duty",
                "rpm",
                "velocity_turn32_per_s",
                "isr_us",
                "isr_max_us",
                "isr_overrun_count",
                "uart_overrun_errors",
                "tmag_sample_count",
                "tmag_sample_dt_us",
                "faults",
            ]
        )
        writer.writerows(rows)

    if summaries:
        all_rpms = [abs(row[5]) for row in rows]
        print(
            f"all_summary runs={len(summaries)} samples={len(rows)} "
            f"rpm_avg={statistics.fmean(all_rpms):.0f} rpm_min={min(all_rpms):.0f} "
            f"rpm_max={max(all_rpms):.0f} saved={args.out}",
            flush=True,
        )
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="Sample PennyESC duty-300 running status")
    parser.add_argument("--port", default="/dev/cu.usbmodem101")
    parser.add_argument("--address", type=int, default=1)
    parser.add_argument("--baud", type=int, default=921600)
    parser.add_argument("--duty", type=int, default=300)
    parser.add_argument("--spinup", type=float, default=0.5)
    parser.add_argument("--samples", type=int, default=20)
    parser.add_argument("--interval", type=float, default=0.025)
    parser.add_argument("--runs", type=int, default=1)
    parser.add_argument("--between-runs", type=float, default=0.3)
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT)
    return run(parser.parse_args())


if __name__ == "__main__":
    raise SystemExit(main())
