#!/usr/bin/env python3.11
from __future__ import annotations

import argparse
import math
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


POLE_PAIRS = 6
TURN16 = 65536
SAMPLE_HZ = 10000
SAMPLE_US = 100
SENSOR_STALE_US = 10000
AB_FAST_ALPHA_DIV = 8
AB_FAST_BETA_DIV = 128


def c_div(num: int, den: int) -> int:
    if num >= 0:
        return num // den
    return -((-num) // den)


def wrap_delta_turn16(new: int, old: int) -> int:
    return ((new - old + 32768) & 0xFFFF) - 32768


def clamp_position_error(error: int) -> int:
    return max(-32768, min(32768, error))


def residual_turns(
    true_turns: float,
    error_deg: float,
    cycles_per_rev: float,
    error_model: str,
    rng: np.random.Generator | None,
) -> float:
    if error_model == "gaussian":
        if rng is None:
            raise ValueError("gaussian error model needs rng")
        return float(rng.normal(0.0, error_deg / 360.0))
    return (error_deg / 360.0) * math.sin(2.0 * math.pi * cycles_per_rev * true_turns)


def measured_angle_turn16(
    true_turns: float,
    error_deg: float,
    cycles_per_rev: float,
    error_model: str,
    rng: np.random.Generator | None,
) -> int:
    residual = residual_turns(true_turns, error_deg, cycles_per_rev, error_model, rng)
    return int(round(((true_turns + residual) % 1.0) * TURN16)) & 0xFFFF


def simulate_speed(
    rpm: float,
    error_deg: float,
    cycles_per_rev: float,
    error_model: str,
    duration_s: float,
    settle_s: float,
    rng: np.random.Generator | None,
) -> tuple[float, float, float]:
    true_velocity = rpm * TURN16 / 60.0
    true_turns_per_s = rpm / 60.0
    sample_count = int(round(duration_s * SAMPLE_HZ))
    settle_count = int(round(settle_s * SAMPLE_HZ))

    observer_position = 0
    observer_velocity = 0
    absolute_position = 0
    last_angle = 0
    position_tick = 0
    velocity_errors: list[float] = []

    for index in range(sample_count):
        tick = ((index + 1) * SAMPLE_US) & 0xFFFF
        true_turns = (index + 1) / SAMPLE_HZ * true_turns_per_s
        angle = measured_angle_turn16(true_turns, error_deg, cycles_per_rev, error_model, rng)

        if index == 0:
            last_angle = angle
            absolute_position = angle
            observer_position = absolute_position
            position_tick = tick
            continue

        absolute_position += wrap_delta_turn16(angle, last_angle)
        last_angle = angle

        dt_us = (tick - position_tick) & 0xFFFF
        if dt_us == 0 or dt_us >= SENSOR_STALE_US:
            observer_position = absolute_position
        else:
            predicted = observer_position + c_div(c_div(observer_velocity, 1000) * int(dt_us), 1000)
            error = clamp_position_error(absolute_position - predicted)
            observer_position = predicted + c_div(error, AB_FAST_ALPHA_DIV)
            correction = c_div(error * 1000, int(dt_us)) * 1000
            observer_velocity += c_div(correction, AB_FAST_BETA_DIV)

        position_tick = tick

        if index >= settle_count:
            velocity_errors.append(observer_velocity - true_velocity)

    if not velocity_errors:
        return 0.0, 0.0, 0.0

    errors = np.asarray(velocity_errors, dtype=np.float64) * 60.0 / TURN16
    return (
        float(np.sqrt(np.mean(errors**2))),
        float(np.percentile(np.abs(errors), 95.0)),
        float(np.max(np.abs(errors))),
    )


def run_sweep(args: argparse.Namespace, error_deg: float) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    rpms = np.arange(0.0, args.max_rpm + args.rpm_step * 0.5, args.rpm_step)
    rms = np.empty_like(rpms)
    p95 = np.empty_like(rpms)
    peak = np.empty_like(rpms)
    run_count = args.runs if args.error_model == "gaussian" else 1
    for index, rpm in enumerate(rpms):
        results = []
        for run in range(run_count):
            rng = None
            if args.error_model == "gaussian":
                rng = np.random.default_rng(args.seed + index * run_count + run)
            results.append(
                simulate_speed(
                    rpm,
                    error_deg,
                    args.cycles_per_rev,
                    args.error_model,
                    args.duration_s,
                    args.settle_s,
                    rng,
                )
            )
        values = np.asarray(results, dtype=np.float64)
        rms[index], p95[index], peak[index] = np.mean(values, axis=0)
    return rpms, rms, p95, peak


def plot_results(
    out_path: Path,
    rpms: np.ndarray,
    mech: tuple[np.ndarray, np.ndarray, np.ndarray],
    elec: tuple[np.ndarray, np.ndarray, np.ndarray],
    args: argparse.Namespace,
) -> None:
    mech_rms, mech_p95, mech_peak = mech
    elec_rms, elec_p95, elec_peak = elec

    fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    if args.error_model == "gaussian":
        model_text = f"Gaussian residual, std={args.error_deg:g} deg, runs={args.runs}"
    else:
        model_text = f"sinusoidal residual, amplitude={args.error_deg:g} deg, {args.cycles_per_rev:g} cycle/rev"
    fig.suptitle(
        "AB_FAST velocity error from angle residual\n"
        f"{model_text}, alpha=1/{AB_FAST_ALPHA_DIV}, beta=1/{AB_FAST_BETA_DIV}"
    )

    axes[0].plot(rpms, mech_rms, label=f"RMS, {args.error_deg:g} deg mechanical")
    axes[0].plot(rpms, mech_p95, label=f"P95, {args.error_deg:g} deg mechanical")
    axes[0].plot(rpms, mech_peak, label=f"Peak, {args.error_deg:g} deg mechanical")
    axes[0].set_ylabel("Velocity error (rpm)")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend()

    axes[1].plot(rpms, elec_rms, label=f"RMS, {args.error_deg:g} deg electrical")
    axes[1].plot(rpms, elec_p95, label=f"P95, {args.error_deg:g} deg electrical")
    axes[1].plot(rpms, elec_peak, label=f"Peak, {args.error_deg:g} deg electrical")
    axes[1].set_xlabel("True speed (rpm)")
    axes[1].set_ylabel("Velocity error (rpm)")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend()

    fig.tight_layout()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=160)
    if args.show:
        plt.show()
    plt.close(fig)


def print_summary(label: str, rpms: np.ndarray, rms: np.ndarray, p95: np.ndarray, peak: np.ndarray) -> None:
    print(label)
    for target in (1000, 3000, 6000, 10000, 20000):
        index = int(np.argmin(np.abs(rpms - target)))
        print(
            "  %5.0f rpm: rms=%7.1f rpm  p95=%7.1f rpm  peak=%7.1f rpm"
            % (rpms[index], rms[index], p95[index], peak[index])
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Simulate default PennyESC AB_FAST velocity error from angle residual."
    )
    parser.add_argument("--error-deg", type=float, default=5.0, help="Residual angle amplitude or std in degrees.")
    parser.add_argument("--error-model", choices=("sine", "gaussian"), default="sine")
    parser.add_argument("--cycles-per-rev", type=float, default=1.0, help="Residual sine cycles per mechanical rev.")
    parser.add_argument("--runs", type=int, default=20, help="Gaussian runs to average at each speed.")
    parser.add_argument("--seed", type=int, default=12345)
    parser.add_argument("--max-rpm", type=float, default=20000.0)
    parser.add_argument("--rpm-step", type=float, default=250.0)
    parser.add_argument("--duration-s", type=float, default=1.0)
    parser.add_argument("--settle-s", type=float, default=0.2)
    parser.add_argument("--out", type=Path, default=Path(__file__).with_suffix(".png"))
    parser.add_argument("--show", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rpms, mech_rms, mech_p95, mech_peak = run_sweep(args, args.error_deg)
    _, elec_rms, elec_p95, elec_peak = run_sweep(args, args.error_deg / POLE_PAIRS)

    plot_results(
        args.out,
        rpms,
        (mech_rms, mech_p95, mech_peak),
        (elec_rms, elec_p95, elec_peak),
        args,
    )
    print_summary(f"{args.error_deg:g} deg interpreted as mechanical", rpms, mech_rms, mech_p95, mech_peak)
    print_summary(f"{args.error_deg:g} deg interpreted as electrical", rpms, elec_rms, elec_p95, elec_peak)
    print(f"wrote {args.out}")


if __name__ == "__main__":
    main()
