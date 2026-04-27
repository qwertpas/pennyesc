#!/usr/bin/env python3.11
from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
import sys
import time
from dataclasses import asdict, dataclass
from pathlib import Path

TOOLS_DIR = Path(__file__).resolve().parent
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from pennycal import EspBridge  # noqa: E402
from pnystep import CaptureSample, StepperClient  # noqa: E402
from pnyproto import OBSERVER_LP4, RESULT_OK  # noqa: E402

ADVANCES = [30, 50, 70, 90, 110, 130, 150]
DIRECTIONS = [1, -1]


@dataclass(frozen=True)
class PulseSummary:
    test: str
    direction: int
    advance_deg: int
    observer_lead_us: int
    observer_mode: int
    repeat: int
    baseline_duty: int
    pulse_duty: int
    baseline_advance_deg: int
    baseline_ms: int
    pulse_ms: int
    sample_hz: int
    sample_count: int
    missed_count: int
    mct_fault_delta: int
    faults: int
    baseline_rpm: float
    start_rpm: float
    rpm_50ms: float
    rpm_100ms: float
    rpm_150ms: float
    final_rpm: float
    signed_start_rpm: float
    signed_final_rpm: float
    zero_cross_ms: float
    gain_0_50_rpm: float
    gain_0_90_rpm: float
    accel_0_50_rpm_s: float
    accel_0_80_rpm_s: float
    accel_20_120_rpm_s: float
    accel_40_160_rpm_s: float
    ripple_120_220: float
    monotonic_fraction: float
    valid: bool
    note: str


def read_samples(client: StepperClient, sample_hz: int, sample_count: int) -> list[CaptureSample]:
    samples: list[CaptureSample] = []
    offset = 0
    while offset < sample_count:
        for angle_turn16, rpm in client.capture_read(offset, min(14, sample_count - offset)):
            samples.append(CaptureSample(offset, offset * 1000.0 / sample_hz, angle_turn16, rpm))
            offset += 1
    return samples


def unwrap(samples: list[CaptureSample]) -> list[int]:
    if not samples:
        return []
    out = [samples[0].angle_turn16]
    last = samples[0].angle_turn16
    acc = samples[0].angle_turn16
    for sample in samples[1:]:
        delta = sample.angle_turn16 - last
        if delta > 32767:
            delta -= 65536
        elif delta < -32768:
            delta += 65536
        acc += delta
        out.append(acc)
        last = sample.angle_turn16
    return out


def slope(samples: list[CaptureSample], start_ms: float, end_ms: float) -> float:
    points = [(sample.t_ms / 1000.0, abs(sample.rpm)) for sample in samples if start_ms <= sample.t_ms <= end_ms]
    if len(points) < 2:
        return float("nan")
    mean_t = statistics.fmean(t for t, _ in points)
    mean_rpm = statistics.fmean(rpm for _, rpm in points)
    denom = sum((t - mean_t) ** 2 for t, _ in points)
    if denom == 0:
        return float("nan")
    return sum((t - mean_t) * (rpm - mean_rpm) for t, rpm in points) / denom


def speed_at(samples: list[CaptureSample], t_ms: float) -> float:
    if not samples:
        return 0.0
    return float(abs(min(samples, key=lambda sample: abs(sample.t_ms - t_ms)).rpm))


def mean_abs(samples: list[CaptureSample], start_ms: float, end_ms: float) -> float:
    values = [abs(sample.rpm) for sample in samples if start_ms <= sample.t_ms <= end_ms]
    return statistics.fmean(values) if values else 0.0


def mean_signed(samples: list[CaptureSample], start_ms: float, end_ms: float) -> float:
    values = [sample.rpm for sample in samples if start_ms <= sample.t_ms <= end_ms]
    return statistics.fmean(values) if values else 0.0


def ripple(samples: list[CaptureSample], start_ms: float, end_ms: float) -> float:
    values = [abs(sample.rpm) for sample in samples if start_ms <= sample.t_ms <= end_ms]
    if len(values) < 2:
        return float("inf")
    mean = statistics.fmean(values)
    return statistics.pstdev(values) / mean if mean > 1.0 else float("inf")


def monotonic_fraction(samples: list[CaptureSample]) -> float:
    angles = unwrap(samples)
    deltas = [angles[i] - angles[i - 1] for i in range(1, len(angles))]
    if not deltas:
        return 0.0
    pos = sum(1 for delta in deltas if delta > 0)
    neg = sum(1 for delta in deltas if delta < 0)
    zero = len(deltas) - pos - neg
    return (max(pos, neg) + zero) / len(deltas)


def zero_cross_ms(samples: list[CaptureSample]) -> float:
    if len(samples) < 2:
        return float("nan")
    last = samples[0]
    for sample in samples[1:]:
        if last.rpm == 0:
            return last.t_ms
        if (last.rpm < 0 <= sample.rpm) or (last.rpm > 0 >= sample.rpm):
            span = sample.rpm - last.rpm
            if span == 0:
                return sample.t_ms
            frac = -last.rpm / span
            return last.t_ms + frac * (sample.t_ms - last.t_ms)
        last = sample
    return float("nan")


def run_pulse(
    client: StepperClient,
    direction: int,
    advance_deg: int,
    repeat: int,
    args: argparse.Namespace,
) -> tuple[PulseSummary, list[dict]]:
    baseline_duty = direction * args.baseline_duty
    pulse_duty = direction * args.pulse_duty
    rows: list[dict] = []
    try:
        start_faults = client.get_status().mct_fault_count
        client.set_observer(args.observer_lead_us, args.observer_mode)
        client.set_advance(args.baseline_advance)
        client.set_duty(baseline_duty)
        time.sleep(args.baseline_ms / 1000.0)
        baseline_status = client.get_status()
        status = client.capture_start(pulse_duty, advance_deg, args.pulse_ms, args.sample_hz)
        if status.result != RESULT_OK:
            raise RuntimeError(f"capture_start result={status.result}")
        deadline = time.monotonic() + args.pulse_ms / 1000.0 + 1.0
        while status.active and time.monotonic() < deadline:
            time.sleep(0.015)
            status = client.capture_status()
        if status.active:
            raise TimeoutError("capture did not finish")
        samples = read_samples(client, status.sample_hz, status.sample_count)
        end_status = client.get_status()
    finally:
        try:
            client.set_duty(0)
        except Exception:
            pass

    mct_fault_delta = max(0, status.mct_fault_count - start_faults)
    mono = monotonic_fraction(samples)
    note = []
    if status.missed_count:
        note.append(f"missed={status.missed_count}")
    if end_status.faults:
        note.append(f"faults=0x{end_status.faults:02X}")
    if mct_fault_delta:
        note.append(f"mct_fault_delta={mct_fault_delta}")
    if abs(baseline_status.velocity_crads) < 5000:
        note.append("baseline_not_spinning")
    if mono < 0.90:
        note.append("angle_not_monotonic")

    summary = PulseSummary(
        test="pulse",
        direction=direction,
        advance_deg=advance_deg,
        observer_lead_us=args.observer_lead_us,
        observer_mode=args.observer_mode,
        repeat=repeat,
        baseline_duty=args.baseline_duty,
        pulse_duty=args.pulse_duty,
        baseline_advance_deg=args.baseline_advance,
        baseline_ms=args.baseline_ms,
        pulse_ms=args.pulse_ms,
        sample_hz=status.sample_hz,
        sample_count=status.sample_count,
        missed_count=status.missed_count,
        mct_fault_delta=mct_fault_delta,
        faults=end_status.faults,
        baseline_rpm=abs(baseline_status.velocity_crads) * 60.0 / 628.0,
        start_rpm=mean_abs(samples, 0.0, 10.0),
        rpm_50ms=speed_at(samples, 50.0),
        rpm_100ms=speed_at(samples, 100.0),
        rpm_150ms=speed_at(samples, 150.0),
        final_rpm=mean_abs(samples, max(0.0, args.pulse_ms - 20.0), float(args.pulse_ms)),
        signed_start_rpm=mean_signed(samples, 0.0, 10.0),
        signed_final_rpm=mean_signed(samples, max(0.0, args.pulse_ms - 20.0), float(args.pulse_ms)),
        zero_cross_ms=float("nan"),
        gain_0_50_rpm=mean_abs(samples, 30.0, 50.0) - mean_abs(samples, 0.0, 10.0),
        gain_0_90_rpm=mean_abs(samples, 70.0, 90.0) - mean_abs(samples, 0.0, 10.0),
        accel_0_50_rpm_s=slope(samples, 0.0, 50.0),
        accel_0_80_rpm_s=slope(samples, 0.0, 80.0),
        accel_20_120_rpm_s=slope(samples, 20.0, 120.0),
        accel_40_160_rpm_s=slope(samples, 40.0, 160.0),
        ripple_120_220=ripple(samples, 120.0, min(220.0, args.pulse_ms)),
        monotonic_fraction=mono,
        valid=not note,
        note=";".join(note),
    )

    unwrapped = unwrap(samples)
    start_angle = unwrapped[0] if unwrapped else 0
    for sample, angle in zip(samples, unwrapped):
        rows.append(
            {
                "direction": direction,
                "test": "pulse",
                "advance_deg": advance_deg,
                "observer_lead_us": args.observer_lead_us,
                "observer_mode": args.observer_mode,
                "repeat": repeat,
                "t_ms": "%.3f" % sample.t_ms,
                "angle_turn16": sample.angle_turn16,
                "position_turns": "%.6f" % ((angle - start_angle) / 65536.0),
                "rpm": sample.rpm,
                "speed_rpm": abs(sample.rpm),
            }
        )
    return summary, rows


def run_reversal(
    client: StepperClient,
    repeat: int,
    args: argparse.Namespace,
) -> tuple[PulseSummary, list[dict]]:
    rows: list[dict] = []
    try:
        start_faults = client.get_status().mct_fault_count
        client.set_observer(args.observer_lead_us, args.observer_mode)
        client.set_advance(args.baseline_advance)
        client.set_duty(args.baseline_duty)
        time.sleep(args.baseline_ms / 1000.0)
        baseline_status = client.get_status()
        status = client.capture_start(-args.baseline_duty, args.baseline_advance, args.reversal_ms, args.sample_hz)
        if status.result != RESULT_OK:
            raise RuntimeError(f"capture_start result={status.result}")
        deadline = time.monotonic() + args.reversal_ms / 1000.0 + 1.0
        while status.active and time.monotonic() < deadline:
            time.sleep(0.015)
            status = client.capture_status()
        if status.active:
            raise TimeoutError("capture did not finish")
        samples = read_samples(client, status.sample_hz, status.sample_count)
        end_status = client.get_status()
    finally:
        try:
            client.set_duty(0)
        except Exception:
            pass

    mct_fault_delta = max(0, status.mct_fault_count - start_faults)
    note = []
    if status.missed_count:
        note.append(f"missed={status.missed_count}")
    if end_status.faults:
        note.append(f"faults=0x{end_status.faults:02X}")
    if mct_fault_delta:
        note.append(f"mct_fault_delta={mct_fault_delta}")
    if abs(baseline_status.velocity_crads) < 5000:
        note.append("baseline_not_spinning")
    zc = zero_cross_ms(samples)
    zero_note = "" if not math.isnan(zc) else "no_zero_cross"

    start_signed = mean_signed(samples, 0.0, 10.0)
    final_signed = mean_signed(samples, max(0.0, args.reversal_ms - 20.0), float(args.reversal_ms))
    summary = PulseSummary(
        test="reversal",
        direction=-1,
        advance_deg=args.baseline_advance,
        observer_lead_us=args.observer_lead_us,
        observer_mode=args.observer_mode,
        repeat=repeat,
        baseline_duty=args.baseline_duty,
        pulse_duty=-args.baseline_duty,
        baseline_advance_deg=args.baseline_advance,
        baseline_ms=args.baseline_ms,
        pulse_ms=args.reversal_ms,
        sample_hz=status.sample_hz,
        sample_count=status.sample_count,
        missed_count=status.missed_count,
        mct_fault_delta=mct_fault_delta,
        faults=end_status.faults,
        baseline_rpm=abs(baseline_status.velocity_crads) * 60.0 / 628.0,
        start_rpm=abs(start_signed),
        rpm_50ms=speed_at(samples, 50.0),
        rpm_100ms=speed_at(samples, 100.0),
        rpm_150ms=speed_at(samples, 150.0),
        final_rpm=abs(final_signed),
        signed_start_rpm=start_signed,
        signed_final_rpm=final_signed,
        zero_cross_ms=zc,
        gain_0_50_rpm=abs(final_signed) - abs(start_signed),
        gain_0_90_rpm=speed_at(samples, 90.0) - abs(start_signed),
        accel_0_50_rpm_s=slope(samples, 0.0, 50.0),
        accel_0_80_rpm_s=slope(samples, 0.0, 80.0),
        accel_20_120_rpm_s=slope(samples, 20.0, 120.0),
        accel_40_160_rpm_s=slope(samples, 40.0, 160.0),
        ripple_120_220=ripple(samples, 120.0, min(220.0, args.reversal_ms)),
        monotonic_fraction=monotonic_fraction(samples),
        valid=not note,
        note=";".join(note + ([zero_note] if zero_note else [])),
    )

    unwrapped = unwrap(samples)
    start_angle = unwrapped[0] if unwrapped else 0
    for sample, angle in zip(samples, unwrapped):
        rows.append(
            {
                "direction": -1,
                "test": "reversal",
                "advance_deg": args.baseline_advance,
                "observer_lead_us": args.observer_lead_us,
                "observer_mode": args.observer_mode,
                "repeat": repeat,
                "t_ms": "%.3f" % sample.t_ms,
                "angle_turn16": sample.angle_turn16,
                "position_turns": "%.6f" % ((angle - start_angle) / 65536.0),
                "rpm": sample.rpm,
                "speed_rpm": abs(sample.rpm),
            }
        )
    return summary, rows


def write_csv(path: Path, rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("")
        return
    with path.open("w", newline="") as fp:
        writer = csv.DictWriter(fp, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def write_report(path: Path, summaries: list[PulseSummary]) -> None:
    valid = [row for row in summaries if row.valid]
    lines = [
        "# Pulse Acceleration Metrics",
        "",
        f"runs: {len(summaries)}",
        f"valid: {len(valid)}",
        f"invalid: {len(summaries) - len(valid)}",
        "",
        "## Best Pulse Gain 0-50ms",
    ]
    pulse_rows = [row for row in valid if row.test == "pulse"]
    reversal_rows = [row for row in valid if row.test == "reversal"]
    groups = sorted({(row.observer_mode, row.observer_lead_us, row.advance_deg, row.pulse_duty) for row in pulse_rows})
    balanced = []
    for mode, lead, advance, pulse_duty in groups:
        dir_rows = []
        for direction in DIRECTIONS:
            rows = [
                row for row in pulse_rows
                if row.direction == direction
                and row.observer_mode == mode
                and row.observer_lead_us == lead
                and row.advance_deg == advance
                and row.pulse_duty == pulse_duty
            ]
            if rows:
                dir_rows.append(
                    (
                        direction,
                        statistics.fmean(row.gain_0_50_rpm for row in rows),
                        statistics.fmean(row.final_rpm for row in rows),
                        statistics.fmean(row.ripple_120_220 for row in rows),
                        len(rows),
                    )
                )
        if len(dir_rows) == 2:
            balanced.append((min(row[1] for row in dir_rows), mode, lead, advance, pulse_duty, dir_rows))
    if balanced:
        lines += ["", "## Balanced Score"]
        for score, mode, lead, advance, pulse_duty, dir_rows in sorted(balanced, reverse=True):
            parts = " ".join(
                f"dir={direction:+d} gain50={gain:.1f} final={final:.1f} ripple={rip:.4f} n={count}"
                for direction, gain, final, rip, count in dir_rows
            )
            lines.append(f"mode={mode} lead={lead} advance={advance} pulse={pulse_duty} score={score:.1f} {parts}")

    if pulse_rows:
        lines += ["", "## Balanced By Pulse Duty"]
        for pulse_duty in sorted({row.pulse_duty for row in pulse_rows}):
            rows = [row for row in balanced if row[4] == pulse_duty]
            if rows:
                score, mode, lead, advance, _, dir_rows = sorted(rows, reverse=True)[0]
                parts = " ".join(
                    f"dir={direction:+d} gain50={gain:.1f} final={final:.1f} ripple={rip:.4f} n={count}"
                    for direction, gain, final, rip, count in dir_rows
                )
                lines.append(f"pulse={pulse_duty} best_mode={mode} best_lead={lead} advance={advance} score={score:.1f} {parts}")

        lead_scores = []
        for mode, lead in sorted({(row.observer_mode, row.observer_lead_us) for row in pulse_rows}):
            duty_scores = []
            for pulse_duty in sorted({row.pulse_duty for row in pulse_rows}):
                candidates = [row for row in balanced if row[1] == mode and row[2] == lead and row[4] == pulse_duty]
                if candidates:
                    duty_scores.append(max(row[0] for row in candidates))
            if duty_scores:
                lead_scores.append((min(duty_scores), statistics.fmean(duty_scores), mode, lead, len(duty_scores)))
        if lead_scores:
            lines += ["", "## Overall Lead Score"]
            for worst, mean, mode, lead, count in sorted(lead_scores, reverse=True):
                lines.append(f"mode={mode} lead={lead} worst_duty_score={worst:.1f} mean_duty_score={mean:.1f} duties={count}")

    for direction in DIRECTIONS:
        rows = [row for row in pulse_rows if row.direction == direction]
        grouped: dict[tuple[int, int, int, int], list[PulseSummary]] = {}
        for row in rows:
            grouped.setdefault((row.observer_mode, row.observer_lead_us, row.advance_deg, row.pulse_duty), []).append(row)
        ranked = []
        for (mode, lead, advance, pulse_duty), items in grouped.items():
            ranked.append(
                (
                    statistics.fmean(row.gain_0_50_rpm for row in items),
                    statistics.pstdev(row.gain_0_50_rpm for row in items) if len(items) > 1 else 0.0,
                    statistics.fmean(row.accel_0_50_rpm_s for row in items),
                    statistics.fmean(row.start_rpm for row in items),
                    statistics.fmean(row.ripple_120_220 for row in items),
                    statistics.fmean(row.final_rpm for row in items),
                    mode,
                    lead,
                    advance,
                    pulse_duty,
                    len(items),
                )
            )
        for gain, gain_std, accel, start, rip, final, mode, lead, advance, pulse_duty, count in sorted(ranked, reverse=True):
            lines.append(
                f"dir={direction:+d} mode={mode} lead={lead} advance={advance} pulse={pulse_duty} gain50={gain:.1f} std={gain_std:.1f} accel50={accel:.1f} start={start:.1f} final={final:.1f} ripple={rip:.4f} n={count}"
            )

    lines += ["", "## Best Acceleration 0-50ms"]
    for direction in DIRECTIONS:
        rows = [row for row in pulse_rows if row.direction == direction]
        grouped: dict[tuple[int, int, int, int], list[PulseSummary]] = {}
        for row in rows:
            grouped.setdefault((row.observer_mode, row.observer_lead_us, row.advance_deg, row.pulse_duty), []).append(row)
        ranked = []
        for (mode, lead, advance, pulse_duty), items in grouped.items():
            ranked.append(
                (
                    statistics.fmean(row.accel_0_50_rpm_s for row in items),
                    statistics.pstdev(row.accel_0_50_rpm_s for row in items) if len(items) > 1 else 0.0,
                    statistics.fmean(row.gain_0_50_rpm for row in items),
                    statistics.fmean(row.start_rpm for row in items),
                    statistics.fmean(row.ripple_120_220 for row in items),
                    statistics.fmean(row.final_rpm for row in items),
                    mode,
                    lead,
                    advance,
                    pulse_duty,
                    len(items),
                )
            )
        for accel, accel_std, gain, start, rip, final, mode, lead, advance, pulse_duty, count in sorted(ranked, reverse=True):
            lines.append(
                f"dir={direction:+d} mode={mode} lead={lead} advance={advance} pulse={pulse_duty} accel50={accel:.1f} std={accel_std:.1f} gain50={gain:.1f} start={start:.1f} final={final:.1f} ripple={rip:.4f} n={count}"
            )

    lines += ["", "## Best Acceleration 20-120ms"]
    for direction in DIRECTIONS:
        rows = [row for row in pulse_rows if row.direction == direction]
        grouped: dict[tuple[int, int, int, int], list[PulseSummary]] = {}
        for row in rows:
            grouped.setdefault((row.observer_mode, row.observer_lead_us, row.advance_deg, row.pulse_duty), []).append(row)
        ranked = []
        for (mode, lead, advance, pulse_duty), items in grouped.items():
            ranked.append(
                (
                    statistics.fmean(row.accel_20_120_rpm_s for row in items),
                    statistics.pstdev(row.accel_20_120_rpm_s for row in items) if len(items) > 1 else 0.0,
                    statistics.fmean(row.baseline_rpm for row in items),
                    statistics.fmean(row.ripple_120_220 for row in items),
                    statistics.fmean(row.final_rpm for row in items),
                    mode,
                    lead,
                    advance,
                    pulse_duty,
                    len(items),
                )
            )
        for accel_mean, accel_std, baseline, rip, final, mode, lead, advance, pulse_duty, count in sorted(ranked, reverse=True):
            lines.append(
                f"dir={direction:+d} mode={mode} lead={lead} advance={advance} pulse={pulse_duty} accel={accel_mean:.1f} std={accel_std:.1f} baseline={baseline:.1f} final={final:.1f} ripple={rip:.4f} n={count}"
            )
    if reversal_rows:
        lines += ["", "## Reversal"]
        grouped: dict[tuple[int, int], list[PulseSummary]] = {}
        for row in reversal_rows:
            grouped.setdefault((row.observer_mode, row.observer_lead_us), []).append(row)
        ranked = []
        for (mode, lead), items in grouped.items():
            ranked.append(
                (
                    statistics.fmean(row.zero_cross_ms for row in items),
                    statistics.fmean(row.final_rpm for row in items),
                    statistics.fmean(row.ripple_120_220 for row in items),
                    mode,
                    lead,
                    len(items),
                )
            )
        for zc, final, rip, mode, lead, count in sorted(ranked):
            lines.append(f"mode={mode} lead={lead} zero_cross={zc:.1f}ms final_reverse={final:.1f} ripple={rip:.4f} n={count}")
    lines += ["", "## Invalid"]
    for row in summaries:
        if not row.valid:
            lines.append(f"test={row.test} dir={row.direction:+d} mode={row.observer_mode} lead={row.observer_lead_us} advance={row.advance_deg} pulse={row.pulse_duty} repeat={row.repeat} note={row.note}")
    path.write_text("\n".join(lines) + "\n")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Measure acceleration during a duty pulse from an already-spinning motor")
    parser.add_argument("--port", default="/dev/cu.usbmodem1301")
    parser.add_argument("--address", type=int, default=3)
    parser.add_argument("--bridge-mode", default="handoff")
    parser.add_argument("--baseline-duty", type=int, default=100)
    parser.add_argument("--pulse-duty", type=int, default=200)
    parser.add_argument("--pulse-duty-list", dest="pulse_duties", action="append", type=int, default=None)
    parser.add_argument("--baseline-advance", type=int, default=90)
    parser.add_argument("--advance", dest="advances", action="append", type=int, default=None)
    parser.add_argument("--observer-lead-us", type=int, default=450)
    parser.add_argument("--observer-lead", dest="observer_leads", action="append", type=int, default=None)
    parser.add_argument("--observer-mode", dest="observer_modes", action="append", type=int, default=None)
    parser.add_argument("--restore-observer-lead-us", type=int, default=450)
    parser.add_argument("--restore-observer-mode", type=int, default=OBSERVER_LP4)
    parser.add_argument("--baseline-ms", type=int, default=700)
    parser.add_argument("--pulse-ms", type=int, default=300)
    parser.add_argument("--reversal-ms", type=int, default=300)
    parser.add_argument("--sample-hz", type=int, default=500)
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--cooldown-s", type=float, default=0.5)
    parser.add_argument("--skip-pulses", action="store_true")
    parser.add_argument("--reversal", action="store_true")
    parser.add_argument("--output", type=Path, default=Path("firmware/dynamic_metrics/pulse_latest"))
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if max(args.pulse_ms, args.reversal_ms) * args.sample_hz // 1000 > 160:
        raise SystemExit("duration/sample rate exceeds 160-sample firmware buffer")
    summaries: list[PulseSummary] = []
    raw_rows: list[dict] = []
    args.output.mkdir(parents=True, exist_ok=True)
    advances = args.advances if args.advances is not None else ADVANCES
    pulse_duties = args.pulse_duties if args.pulse_duties is not None else [args.pulse_duty]
    observer_leads = args.observer_leads if args.observer_leads is not None else [args.observer_lead_us]
    observer_modes = args.observer_modes if args.observer_modes is not None else [OBSERVER_LP4]
    with EspBridge(args.port) as bridge:
        bridge.enter_bridge(args.bridge_mode)
        client = StepperClient(bridge.serial, args.address)
        try:
            for mode in observer_modes:
                args.observer_mode = mode
                for lead in observer_leads:
                    args.observer_lead_us = lead
                    if not args.skip_pulses:
                        for pulse_duty in pulse_duties:
                            args.pulse_duty = pulse_duty
                            for direction in DIRECTIONS:
                                for advance in advances:
                                    for repeat in range(args.repeats):
                                        print(f"pulse lead={lead} duty={pulse_duty} dir={direction:+d} advance={advance} repeat={repeat}")
                                        try:
                                            summary, rows = run_pulse(client, direction, advance, repeat, args)
                                        except Exception as exc:  # noqa: BLE001
                                            try:
                                                client.set_duty(0)
                                            except Exception:
                                                pass
                                            summary = PulseSummary(
                                                test="pulse",
                                                direction=direction,
                                                advance_deg=advance,
                                                observer_lead_us=lead,
                                                observer_mode=mode,
                                                repeat=repeat,
                                                baseline_duty=args.baseline_duty,
                                                pulse_duty=args.pulse_duty,
                                                baseline_advance_deg=args.baseline_advance,
                                                baseline_ms=args.baseline_ms,
                                                pulse_ms=args.pulse_ms,
                                                sample_hz=args.sample_hz,
                                                sample_count=0,
                                                missed_count=0,
                                                mct_fault_delta=0,
                                                faults=0,
                                                baseline_rpm=0.0,
                                                start_rpm=0.0,
                                                rpm_50ms=0.0,
                                                rpm_100ms=0.0,
                                                rpm_150ms=0.0,
                                                final_rpm=0.0,
                                                signed_start_rpm=0.0,
                                                signed_final_rpm=0.0,
                                                zero_cross_ms=float("nan"),
                                                gain_0_50_rpm=0.0,
                                                gain_0_90_rpm=0.0,
                                                accel_0_50_rpm_s=float("nan"),
                                                accel_0_80_rpm_s=float("nan"),
                                                accel_20_120_rpm_s=float("nan"),
                                                accel_40_160_rpm_s=float("nan"),
                                                ripple_120_220=float("inf"),
                                                monotonic_fraction=0.0,
                                                valid=False,
                                                note=type(exc).__name__,
                                            )
                                            rows = []
                                        print(summary)
                                        summaries.append(summary)
                                        raw_rows.extend(rows)
                                        time.sleep(args.cooldown_s)
                    if args.reversal:
                        for repeat in range(args.repeats):
                            print(f"reversal lead={lead} repeat={repeat}")
                            try:
                                summary, rows = run_reversal(client, repeat, args)
                            except Exception as exc:  # noqa: BLE001
                                try:
                                    client.set_duty(0)
                                except Exception:
                                    pass
                                summary = PulseSummary(
                                    test="reversal",
                                    direction=-1,
                                    advance_deg=args.baseline_advance,
                                    observer_lead_us=lead,
                                    observer_mode=mode,
                                    repeat=repeat,
                                    baseline_duty=args.baseline_duty,
                                    pulse_duty=-args.baseline_duty,
                                    baseline_advance_deg=args.baseline_advance,
                                    baseline_ms=args.baseline_ms,
                                    pulse_ms=args.reversal_ms,
                                    sample_hz=args.sample_hz,
                                    sample_count=0,
                                    missed_count=0,
                                    mct_fault_delta=0,
                                    faults=0,
                                    baseline_rpm=0.0,
                                    start_rpm=0.0,
                                    rpm_50ms=0.0,
                                    rpm_100ms=0.0,
                                    rpm_150ms=0.0,
                                    final_rpm=0.0,
                                    signed_start_rpm=0.0,
                                    signed_final_rpm=0.0,
                                    zero_cross_ms=float("nan"),
                                    gain_0_50_rpm=0.0,
                                    gain_0_90_rpm=0.0,
                                    accel_0_50_rpm_s=float("nan"),
                                    accel_0_80_rpm_s=float("nan"),
                                    accel_20_120_rpm_s=float("nan"),
                                    accel_40_160_rpm_s=float("nan"),
                                    ripple_120_220=float("inf"),
                                    monotonic_fraction=0.0,
                                    valid=False,
                                    note=type(exc).__name__,
                                )
                                rows = []
                            print(summary)
                            summaries.append(summary)
                            raw_rows.extend(rows)
                            time.sleep(args.cooldown_s)
        finally:
            try:
                client.set_duty(0)
                client.set_observer(args.restore_observer_lead_us, args.restore_observer_mode)
            except Exception:
                pass
            bridge.exit_bridge()
    write_csv(args.output / "summary.csv", [asdict(row) for row in summaries])
    write_csv(args.output / "raw_samples.csv", raw_rows)
    write_report(args.output / "report.md", summaries)
    (args.output / "report.json").write_text(json.dumps([asdict(row) for row in summaries], indent=2) + "\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
