#!/usr/bin/env python3.11
from __future__ import annotations

import argparse
import csv
import dataclasses
import json
import math
import sys
import time
from collections import deque
from datetime import datetime
from glob import glob
from pathlib import Path
from typing import Any, Callable, Sequence

import numpy as np
import serial

try:
    from serial.tools import list_ports
except ImportError:
    list_ports = None

try:
    import pyqtgraph as pg
    import pyqtgraph.exporters
    from pyqtgraph.Qt import QtCore, QtWidgets
except ImportError as exc:
    raise SystemExit("Install pyqtgraph and a Qt binding.") from exc

TOOLS_DIR = Path(__file__).resolve().parents[2] / "tools"
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from pennycal import (  # noqa: E402
    AFFINE_TARGET_SCALE,
    CAL_CAPTURE_TOTAL_POINTS,
    CAL_POINTS_PER_SWEEP,
    CAL_TOTAL_POINTS,
    DEFAULT_CHUNK_SIZE,
    CalibrationError,
    EspBridge,
    Stm32Client,
    apply_affine_q20,
    capture_points,
    pseudo_index,
    reduce_capture_points,
    solve_capture,
    upload_calibration,
    verify_calibration,
)

Signal = getattr(QtCore, "Signal", QtCore.pyqtSignal)

DYNAMIC_DEFAULT_ADVANCE = 40
DYNAMIC_SETTLE_S = 0.5
DYNAMIC_SAMPLE_S = 0.5
DYNAMIC_SAMPLE_HZ = 100
DUTY_LIMIT = 799
CRAD_PER_REV = 628.0
POLE_PAIRS = 6
ADVANCE_COARSE = [40, 60, 80, 100, 120, 140, 160]
ADVANCE_MIN = -180
ADVANCE_MAX = 180
RESULTS_DIR = Path(__file__).resolve().parent / "calibrationtest_sessions"
TARGET_TOLERANCE_MIN_RPM = 100.0
TARGET_TOLERANCE_FRAC = 0.05
TARGET_SUSPECT_MIN_RPM = 150.0
TARGET_SUSPECT_FRAC = 0.10
RAW_PLOT_WINDOW_S = 10.0
RAW_POLL_MS = 100
RAW_STATUS_TIMEOUT_S = 0.04
RAW_STATUS_ATTEMPTS = 2
DYNAMIC_STATUS_TIMEOUT_S = 0.25
DYNAMIC_STATUS_ATTEMPTS = 6


def format_status(status) -> str:
    return (
        "mode=%d flags=0x%02X faults=0x%02X raw=(%d,%d,%d) angle_turn16=%d pos_crad=%d vel_crads=%d duty=%d mct_faults=%d"
        % (
            status.mode,
            status.flags,
            status.faults,
            status.x,
            status.y,
            status.z,
            status.angle_turn16,
            status.position_crad,
            status.velocity_crads,
            status.duty,
            status.mct_fault_count,
        )
    )


def find_serial_port(pattern: str | None) -> str:
    if pattern:
        matches = sorted(glob(pattern))
        return matches[0] if matches else pattern

    if list_ports is not None:
        ports = list(list_ports.comports())

        def score(port_info) -> tuple[int, str]:
            text = " ".join(
                [
                    port_info.device or "",
                    port_info.description or "",
                    port_info.manufacturer or "",
                    port_info.hwid or "",
                ]
            ).lower()
            value = 0
            if getattr(port_info, "vid", None) == 0x303A:
                value += 100
            if "esp32" in text or "espressif" in text:
                value += 80
            if "usbmodem" in text or "cdc" in text:
                value += 20
            return value, port_info.device or ""

        if ports:
            return sorted(ports, key=lambda item: (-score(item)[0], score(item)[1]))[0].device

    for port_pattern in ("/dev/cu.usbmodem*", "/dev/ttyACM*", "/dev/ttyUSB*"):
        matches = sorted(glob(port_pattern))
        if matches:
            return matches[0]

    raise SystemExit("No serial port found")


def list_serial_ports(preferred: str | None = None) -> list[str]:
    ports: list[str] = []
    seen: set[str] = set()

    def add_port(value: str) -> None:
        if value and value not in seen:
            seen.add(value)
            ports.append(value)

    if preferred:
        add_port(preferred)

    if list_ports is not None:
        detected = list(list_ports.comports())

        def score(port_info) -> tuple[int, str]:
            text = " ".join(
                [
                    port_info.device or "",
                    port_info.description or "",
                    port_info.manufacturer or "",
                    port_info.hwid or "",
                ]
            ).lower()
            value = 0
            if getattr(port_info, "vid", None) == 0x303A:
                value += 100
            if "esp32" in text or "espressif" in text:
                value += 80
            if "usbmodem" in text or "cdc" in text:
                value += 20
            return value, port_info.device or ""

        for item in sorted(detected, key=lambda port_info: (-score(port_info)[0], score(port_info)[1])):
            add_port(item.device or "")

    for port_pattern in ("/dev/cu.usbmodem*", "/dev/ttyACM*", "/dev/ttyUSB*"):
        for value in sorted(glob(port_pattern)):
            add_port(value)

    return ports


def clamp(value: int, low: int, high: int) -> int:
    return max(low, min(high, value))


def clamp_float(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def rpm_from_crads(velocity_crads: int) -> float:
    return float(velocity_crads) * 60.0 / CRAD_PER_REV


def rpm_from_status(status) -> float:
    return rpm_from_crads(status.velocity_crads)


def direction_name(direction: int) -> str:
    return "forward" if direction > 0 else "reverse"


def direction_index(direction: int) -> int:
    return 0 if direction > 0 else 1


def signed_turn16_error_deg(measured: int, target: float) -> float:
    delta = ((measured - target + 32768.0) % 65536.0) - 32768.0
    return float(delta) * 360.0 / 65536.0


def angle_from_xy(x: int, y: int, affine_q20: Sequence[int], angle_lut: Sequence[int]) -> int:
    xs = np.array([x], dtype=np.int64)
    ys = np.array([y], dtype=np.int64)
    u, v = apply_affine_q20(xs, ys, affine_q20)
    return int(angle_lut[pseudo_index(int(u[0]), int(v[0]))])


def parse_speed_text(text: str) -> list[float]:
    speeds: list[float] = []
    seen: set[float] = set()
    for chunk in text.split(","):
        token = chunk.strip()
        if not token:
            continue
        value = float(token)
        if value <= 0:
            raise ValueError("speeds must be positive")
        rounded = round(value, 3)
        if rounded in seen:
            continue
        seen.add(rounded)
        speeds.append(float(rounded))
    speeds.sort()
    if len(speeds) < 2:
        raise ValueError("enter at least 2 speeds")
    return speeds


def choose_speed_key(speed: float) -> str:
    if abs(speed - round(speed)) < 1e-6:
        return str(int(round(speed)))
    return f"{speed:.3f}".rstrip("0").rstrip(".")


def line_color(index: int) -> Any:
    colors = [
        "#d1495b",
        "#2e86ab",
        "#3b8b3b",
        "#8e5ea2",
        "#e67e22",
        "#009688",
        "#7f8c8d",
        "#c0392b",
        "#2980b9",
        "#16a085",
    ]
    return pg.mkPen(colors[index % len(colors)], width=2)


def to_serializable(value: Any) -> Any:
    if dataclasses.is_dataclass(value):
        return to_serializable(dataclasses.asdict(value))
    if isinstance(value, dict):
        return {str(key): to_serializable(item) for key, item in value.items()}
    if isinstance(value, list):
        return [to_serializable(item) for item in value]
    if isinstance(value, tuple):
        return [to_serializable(item) for item in value]
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, np.generic):
        return value.item()
    return value


@dataclasses.dataclass
class SweepPoint:
    direction: int
    target_rpm: float
    advance_deg: int
    duty: int
    mean_rpm: float
    std_rpm: float
    ripple: float
    score: float
    valid: bool
    sign_flip: bool
    faults: int
    sample_count: int
    target_error_rpm: float
    samples: list[dict[str, Any]]


@dataclasses.dataclass
class BestPoint:
    direction: int
    target_rpm: float
    achieved_rpm: float
    advance_deg: int
    duty: int
    score: float
    ripple: float
    target_error_rpm: float


@dataclasses.dataclass
class DynamicFit:
    intercept_deg: float | None
    slope_deg_per_rpm: float | None
    delay_s: float | None
    averaged_points: list[dict[str, float]]


@dataclasses.dataclass
class StaticResult:
    raw_points: list[dict[str, Any]]
    reduced_points: list[dict[str, Any]]
    affine_q20: list[int] | None
    angle_lut: list[int] | None
    fit_max_error_deg: float | None
    sweep_delta_deg: float | None
    blob_crc32: int | None
    blob_size: int | None


@dataclasses.dataclass
class DynamicResult:
    input_speeds_rpm: list[float]
    base_advance_deg: int
    points: list[SweepPoint]
    best_points: list[BestPoint]
    fit: DynamicFit
    state: str
    notes: list[str]


@dataclasses.dataclass
class SessionData:
    mode: str
    port: str
    address: int
    export_dir: str
    started_at: str
    ended_at: str | None = None
    status: str = "not run"
    error: str | None = None
    static: StaticResult | None = None
    dynamic: DynamicResult | None = None


def capture_to_rows(points: Sequence[Any], total_per_dir: int | None = None) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for point in sorted(points, key=lambda item: item.index):
        rows.append(
            {
                "index": int(point.index),
                "step_index": int(point.step_index),
                "sweep_dir": int(point.sweep_dir),
                "direction": "forward" if int(point.sweep_dir) == 0 else "reverse",
                "rotation": int(point.index // total_per_dir) if total_per_dir else "",
                "x": int(point.x),
                "y": int(point.y),
                "z": int(point.z),
                "xy_radius": int(point.xy_radius),
                "sample_spread": int(point.sample_spread),
                "duty": int(point.duty),
            }
        )
    return rows


def build_static_result(raw_points: Sequence[Any], reduced_points: Sequence[Any], solved) -> StaticResult:
    return StaticResult(
        raw_points=capture_to_rows(raw_points, CAL_POINTS_PER_SWEEP),
        reduced_points=capture_to_rows(reduced_points),
        affine_q20=[int(value) for value in solved.affine_q20],
        angle_lut=[int(value) for value in solved.angle_lut],
        fit_max_error_deg=float(solved.fit_max_error_deg),
        sweep_delta_deg=float(solved.sweep_delta_deg),
        blob_crc32=int(solved.blob_crc32),
        blob_size=len(solved.blob),
    )


def build_partial_static_result(raw_points: Sequence[Any], reduced_points: Sequence[Any]) -> StaticResult:
    return StaticResult(
        raw_points=capture_to_rows(raw_points, CAL_POINTS_PER_SWEEP),
        reduced_points=capture_to_rows(reduced_points),
        affine_q20=None,
        angle_lut=None,
        fit_max_error_deg=None,
        sweep_delta_deg=None,
        blob_crc32=None,
        blob_size=None,
    )


def verify_dynamic_ready(client: Stm32Client) -> None:
    status = client.wait_until_ready()
    info = client.cal_info()
    if not status.calibrated or not info.valid:
        raise CalibrationError("device is not statically calibrated")
    if status.faults != 0:
        raise CalibrationError(f"device reported faults 0x{status.faults:02X}")


def read_dynamic_status(client: Stm32Client):
    return client.get_status(timeout=DYNAMIC_STATUS_TIMEOUT_S, attempts=DYNAMIC_STATUS_ATTEMPTS)


def collect_samples(client: Stm32Client, duration_s: float, sample_hz: int) -> list[dict[str, Any]]:
    count = max(1, int(round(duration_s * sample_hz)))
    start = time.monotonic()
    samples: list[dict[str, Any]] = []
    for index in range(count):
        target = start + (index / float(sample_hz))
        delay = target - time.monotonic()
        if delay > 0:
            time.sleep(delay)
        status = read_dynamic_status(client)
        samples.append(
            {
                "t_s": time.monotonic() - start,
                "rpm": rpm_from_status(status),
                "faults": int(status.faults),
                "duty": int(status.duty),
                "result": int(status.result),
                "mode": int(status.mode),
                "x": int(status.x),
                "y": int(status.y),
                "z": int(status.z),
                "angle_turn16": int(status.angle_turn16),
                "position_crad": int(status.position_crad),
                "velocity_crads": int(status.velocity_crads),
            }
        )
    return samples


def score_close_enough(score: float, best_score: float) -> bool:
    if best_score > 0:
        return score >= best_score * 0.99
    window = 0.01 * max(1.0, abs(best_score))
    return abs(score - best_score) <= window


def pick_best_point(points: Sequence[SweepPoint]) -> SweepPoint | None:
    valid = [point for point in points if point.valid]
    if not valid:
        return None
    best_score = max(point.score for point in valid)
    close = [point for point in valid if score_close_enough(point.score, best_score)]
    return min(close, key=lambda point: point.advance_deg)


def summarize_samples(
    direction: int,
    target_rpm: float,
    advance_deg: int,
    duty: int,
    samples: list[dict[str, Any]],
) -> SweepPoint:
    aligned = np.array([sample["rpm"] * direction for sample in samples], dtype=np.float64)
    mean_rpm = float(np.mean(aligned)) if len(aligned) else 0.0
    std_rpm = float(np.std(aligned)) if len(aligned) else 0.0
    ripple = float(std_rpm / mean_rpm) if mean_rpm > 0 else float("inf")
    sign_flip = any(value <= 0.0 for value in aligned)
    faults = 0
    for sample in samples:
        faults |= int(sample["faults"])
    valid = bool(samples) and faults == 0 and not sign_flip and ripple <= 0.08 and mean_rpm > 0
    score = mean_rpm - (0.2 * std_rpm)
    return SweepPoint(
        direction=direction,
        target_rpm=float(target_rpm),
        advance_deg=int(advance_deg),
        duty=int(duty),
        mean_rpm=float(mean_rpm),
        std_rpm=float(std_rpm),
        ripple=float(ripple),
        score=float(score),
        valid=bool(valid),
        sign_flip=bool(sign_flip),
        faults=int(faults),
        sample_count=len(samples),
        target_error_rpm=float(mean_rpm - target_rpm),
        samples=samples,
    )


def tune_duty_for_target(
    client: Stm32Client,
    direction: int,
    target_rpm: float,
    advance_deg: int,
    initial_duty: int | None,
) -> int:
    client.set_advance_deg(advance_deg)
    if initial_duty is None:
        base = clamp(int(round(target_rpm * 0.10)), 60, DUTY_LIMIT)
        duty = base * direction
    else:
        duty = clamp(int(initial_duty), -DUTY_LIMIT, DUTY_LIMIT)
        if duty == 0:
            duty = 60 * direction
    tolerance = max(TARGET_TOLERANCE_MIN_RPM, target_rpm * TARGET_TOLERANCE_FRAC)

    for _ in range(12):
        client.set_duty(duty)
        time.sleep(0.12)
        status = read_dynamic_status(client)
        aligned = rpm_from_status(status) * direction
        if status.faults != 0:
            break
        error = target_rpm - aligned
        if abs(error) <= tolerance:
            break
        step = clamp(int(round(abs(error) * 0.08)), 6, 80)
        if aligned <= 0:
            step = max(step, 24)
        if error > 0:
            duty += direction * step
        else:
            duty -= direction * step
        duty = clamp(duty, -DUTY_LIMIT, DUTY_LIMIT)
        if duty == 0:
            duty = 20 * direction

    client.set_duty(duty)
    return duty


def run_candidate(
    client: Stm32Client,
    direction: int,
    target_rpm: float,
    advance_deg: int,
    initial_duty: int | None,
) -> tuple[SweepPoint, int]:
    duty = tune_duty_for_target(client, direction, target_rpm, advance_deg, initial_duty)
    time.sleep(DYNAMIC_SETTLE_S)
    samples = collect_samples(client, DYNAMIC_SAMPLE_S, DYNAMIC_SAMPLE_HZ)
    return summarize_samples(direction, target_rpm, advance_deg, duty, samples), duty


def compute_dynamic_fit(best_points: Sequence[BestPoint]) -> DynamicFit:
    groups: dict[float, dict[int, BestPoint]] = {}
    for point in best_points:
        groups.setdefault(point.target_rpm, {})[point.direction] = point

    averaged_points: list[dict[str, float]] = []
    for target_rpm in sorted(groups):
        pair = groups[target_rpm]
        if 1 not in pair or -1 not in pair:
            continue
        forward = pair[1]
        reverse = pair[-1]
        achieved_rpm = (forward.achieved_rpm + reverse.achieved_rpm) * 0.5
        averaged_points.append(
            {
                "target_rpm": float(target_rpm),
                "achieved_rpm": float(achieved_rpm),
                "forward_advance_deg": float(forward.advance_deg),
                "reverse_advance_deg": float(reverse.advance_deg),
                "average_advance_deg": float((forward.advance_deg + reverse.advance_deg) * 0.5),
                "asymmetry_deg": float(forward.advance_deg - reverse.advance_deg),
            }
        )

    if len(averaged_points) < 2:
        return DynamicFit(intercept_deg=None, slope_deg_per_rpm=None, delay_s=None, averaged_points=averaged_points)

    xs = np.array([point["achieved_rpm"] for point in averaged_points], dtype=np.float64)
    ys = np.array([point["average_advance_deg"] for point in averaged_points], dtype=np.float64)
    slope, intercept = np.polyfit(xs, ys, deg=1)
    return DynamicFit(
        intercept_deg=float(intercept),
        slope_deg_per_rpm=float(slope),
        delay_s=float(slope / (6.0 * POLE_PAIRS)),
        averaged_points=averaged_points,
    )


def run_dynamic(
    client: Stm32Client,
    speeds: Sequence[float],
    base_advance_deg: int,
    log_cb: Callable[[str], None] | None = None,
    progress_cb: Callable[[int], None] | None = None,
) -> DynamicResult:
    verify_dynamic_ready(client)
    notes: list[str] = []
    all_points: list[SweepPoint] = []
    best_points: list[BestPoint] = []
    duty_guess: dict[tuple[int, float], int] = {}

    total_steps = len(speeds) * 2 * len(ADVANCE_COARSE)
    done_steps = 0

    def log(text: str) -> None:
        if log_cb is not None:
            log_cb(text)

    def tick() -> None:
        nonlocal done_steps
        done_steps += 1
        if progress_cb is not None and total_steps > 0:
            progress_cb(82 + (done_steps * 14) // total_steps)

    client.set_advance_deg(base_advance_deg)
    client.set_duty(0)

    try:
        for direction in (1, -1):
            for target_rpm in speeds:
                label = f"{direction_name(direction)} {target_rpm:.0f}rpm"
                coarse_points: list[SweepPoint] = []
                for advance_deg in ADVANCE_COARSE:
                    log(f"dynamic {label} advance={advance_deg}")
                    point, duty = run_candidate(
                        client,
                        direction=direction,
                        target_rpm=target_rpm,
                        advance_deg=advance_deg,
                        initial_duty=duty_guess.get((direction, target_rpm)),
                    )
                    duty_guess[(direction, target_rpm)] = duty
                    coarse_points.append(point)
                    all_points.append(point)
                    tick()

                coarse_best = pick_best_point(coarse_points)
                if coarse_best is None:
                    notes.append(f"no valid coarse point for {label}")
                    continue

                refine_values = sorted(
                    {
                        clamp(coarse_best.advance_deg + offset, ADVANCE_MIN, ADVANCE_MAX)
                        for offset in range(-10, 12, 2)
                    }
                )
                refine_points: list[SweepPoint] = []
                for advance_deg in refine_values:
                    if advance_deg in ADVANCE_COARSE:
                        continue
                    log(f"refine {label} advance={advance_deg}")
                    point, duty = run_candidate(
                        client,
                        direction=direction,
                        target_rpm=target_rpm,
                        advance_deg=advance_deg,
                        initial_duty=duty_guess.get((direction, target_rpm), coarse_best.duty),
                    )
                    duty_guess[(direction, target_rpm)] = duty
                    refine_points.append(point)
                    all_points.append(point)

                best = pick_best_point(coarse_points + refine_points)
                if best is None:
                    notes.append(f"no valid point for {label}")
                    continue

                best_points.append(
                    BestPoint(
                        direction=direction,
                        target_rpm=float(target_rpm),
                        achieved_rpm=float(best.mean_rpm),
                        advance_deg=int(best.advance_deg),
                        duty=int(best.duty),
                        score=float(best.score),
                        ripple=float(best.ripple),
                        target_error_rpm=float(best.target_error_rpm),
                    )
                )
                log(
                    "best %s target=%.0frpm achieved=%.0frpm advance=%d score=%.1f ripple=%.3f duty=%d"
                    % (
                        direction_name(direction),
                        target_rpm,
                        best.mean_rpm,
                        best.advance_deg,
                        best.score,
                        best.ripple,
                        best.duty,
                    )
                )
    finally:
        try:
            client.set_duty(0)
        except Exception:
            pass
        try:
            client.set_advance_deg(base_advance_deg)
        except Exception:
            pass

    fit = compute_dynamic_fit(best_points)
    expected_best = len(speeds) * 2
    state = "dynamic complete"
    if len(best_points) != expected_best:
        state = "dynamic incomplete"
    elif any(
        abs(point.target_error_rpm) > max(TARGET_SUSPECT_MIN_RPM, point.target_rpm * TARGET_SUSPECT_FRAC)
        for point in best_points
    ):
        state = "dynamic suspect"

    return DynamicResult(
        input_speeds_rpm=[float(speed) for speed in speeds],
        base_advance_deg=int(base_advance_deg),
        points=all_points,
        best_points=best_points,
        fit=fit,
        state=state,
        notes=notes,
    )


class TestWorker(QtCore.QThread):
    log = Signal(str)
    capture_status = Signal(int, int, int)
    capture_sample = Signal(int, int, int, int, int)
    upload = Signal(int, int)
    progress = Signal(int)
    session_ready = Signal(object)
    done = Signal(str)
    failed = Signal(str)

    def __init__(
        self,
        port: str,
        address: int,
        chunk_size: int,
        mode: str,
        speed_text: str,
        base_advance_deg: int,
    ) -> None:
        super().__init__()
        self.port = port
        self.address = address
        self.chunk_size = chunk_size
        self.mode = mode
        self.speed_text = speed_text
        self.base_advance_deg = base_advance_deg

    def run(self) -> None:
        stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        export_dir = RESULTS_DIR / stamp
        session = SessionData(
            mode=self.mode,
            port=self.port,
            address=self.address,
            export_dir=str(export_dir),
            started_at=datetime.now().isoformat(timespec="seconds"),
        )

        try:
            with EspBridge(self.port) as bridge:
                self.log.emit(f"open {self.port}")
                bridge.enter_bridge("app")
                self.log.emit("bridge app")
                client = Stm32Client(bridge.serial, address=self.address)

                if self.mode in {"full", "static"}:
                    self.progress.emit(1)
                    raw_points = capture_points(
                        client,
                        status_cb=self.on_capture_status,
                        sample_cb=self.on_capture_sample,
                    )
                    self.log.emit("average")
                    reduced_points = reduce_capture_points(raw_points)
                    session.static = build_partial_static_result(raw_points, reduced_points)
                    self.log.emit("solve")
                    solved = solve_capture(reduced_points)
                    session.static = build_static_result(raw_points, reduced_points, solved)
                    self.log.emit(
                        "fit %.2f deg sweep %.2f deg crc32 0x%08X"
                        % (solved.fit_max_error_deg, solved.sweep_delta_deg, solved.blob_crc32)
                    )
                    self.log.emit("upload")
                    upload_calibration(client, solved, self.chunk_size, upload_cb=self.on_upload)
                    self.log.emit("commit")
                    result, valid = client.cal_commit()
                    if result != 0 or valid != 1:
                        raise CalibrationError(f"CAL_COMMIT failed with result {result}")
                    info = verify_calibration(client, solved)
                    if session.static is not None:
                        session.static.blob_size = int(info.blob_size)
                    session.status = "static committed"
                    self.progress.emit(80)
                    self.log.emit(
                        "static committed crc32=0x%08X size=%d"
                        % (info.blob_crc32, info.blob_size)
                    )

                if self.mode in {"full", "dynamic"}:
                    speeds = parse_speed_text(self.speed_text)
                    self.log.emit("dynamic start")
                    dynamic = run_dynamic(
                        client,
                        speeds=speeds,
                        base_advance_deg=self.base_advance_deg,
                        log_cb=lambda text: self.log.emit(text),
                        progress_cb=lambda value: self.progress.emit(value),
                    )
                    session.dynamic = dynamic
                    session.status = dynamic.state

                if self.mode == "static" and session.status == "not run":
                    session.status = "static committed"

            session.ended_at = datetime.now().isoformat(timespec="seconds")
            self.session_ready.emit(session)
            self.done.emit(session.status)
        except Exception as exc:  # noqa: BLE001
            session.error = str(exc)
            if session.status == "not run":
                session.status = "dynamic incomplete" if self.mode != "static" else "failed"
            session.ended_at = datetime.now().isoformat(timespec="seconds")
            self.session_ready.emit(session)
            self.failed.emit(str(exc))

    def on_capture_status(self, status) -> None:
        self.capture_status.emit(status.active, status.next_index, status.total_points)
        if status.total_points > 0:
            self.progress.emit((status.next_index * 60) // status.total_points)

    def on_capture_sample(self, _point_index: int, _step_index: int, sweep_dir: int, status) -> None:
        self.capture_sample.emit(int(sweep_dir), int(status.x), int(status.y), int(status.z), int(status.faults))

    def on_upload(self, done: int, total: int) -> None:
        self.upload.emit(done, total)
        if total > 0:
            self.progress.emit(60 + (done * 20) // total)


class Window:
    def __init__(self, port: str, address: int, chunk_size: int, auto: bool, mode: str, speeds: str) -> None:
        if not 0 <= address <= 0xF:
            raise SystemExit("ESC address must be 0-15")
        self.port = port
        self.address = address
        self.chunk_size = chunk_size
        self.auto = auto
        self.auto_mode = mode
        self.worker: TestWorker | None = None
        self.bridge: EspBridge | None = None
        self.client: Stm32Client | None = None
        self.client_busy = False
        self.session: SessionData | None = None
        self.dynamic_advance_deg = DYNAMIC_DEFAULT_ADVANCE
        self.static_live_forward_x: list[int] = []
        self.static_live_forward_y: list[int] = []
        self.static_live_reverse_x: list[int] = []
        self.static_live_reverse_y: list[int] = []
        self.raw_times: deque[float] = deque()
        self.raw_x_vals: deque[int] = deque()
        self.raw_y_vals: deque[int] = deque()
        self.raw_z_vals: deque[int] = deque()
        self.raw_start_time: float | None = None
        self.raw_paused = False
        self.raw_last_status = "waiting for data"
        self.raw_last_values = (0, 0, 0)

        self.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv)
        self.app.aboutToQuit.connect(self.close_client)

        self.win = QtWidgets.QWidget()
        self.win.setWindowTitle("pennyesc calibration test")
        screen = self.app.primaryScreen()
        if screen is not None:
            available = screen.availableGeometry()
            width = clamp(available.width() - 40, 900, 1600)
            height = clamp(available.height() - 40, 760, 1200)
            self.win.resize(width, height)
        else:
            self.win.resize(1280, 960)

        layout = QtWidgets.QVBoxLayout(self.win)

        top_row = QtWidgets.QHBoxLayout()
        top_row.addWidget(QtWidgets.QLabel("Port"))
        self.port_box = QtWidgets.QComboBox()
        self.port_box.setMinimumWidth(180)
        self.port_box.setMaximumWidth(260)
        self.port_box.currentIndexChanged.connect(self.on_port_changed)
        top_row.addWidget(self.port_box)
        self.refresh_ports_button = QtWidgets.QPushButton("Refresh")
        self.refresh_ports_button.clicked.connect(self.refresh_ports)
        top_row.addWidget(self.refresh_ports_button)

        self.status = QtWidgets.QLabel()
        self.status.setSizePolicy(QtWidgets.QSizePolicy.Policy.Ignored, QtWidgets.QSizePolicy.Policy.Preferred)
        top_row.addWidget(self.status, stretch=1)
        layout.addLayout(top_row)

        controls = QtWidgets.QHBoxLayout()
        controls.addWidget(QtWidgets.QLabel("ESC"))
        self.esc_box = QtWidgets.QComboBox()
        for esc in range(16):
            self.esc_box.addItem(str(esc), esc)
        index = self.esc_box.findData(self.address)
        if index >= 0:
            self.esc_box.setCurrentIndex(index)
        self.esc_box.currentIndexChanged.connect(self.on_esc_changed)
        controls.addWidget(self.esc_box)
        controls.addWidget(QtWidgets.QLabel("Speeds"))
        self.speed_edit = QtWidgets.QLineEdit(speeds)
        self.speed_edit.setPlaceholderText("1500,2500")
        self.speed_edit.setMinimumWidth(180)
        self.speed_edit.setMaximumWidth(240)
        controls.addWidget(self.speed_edit)
        controls.addWidget(QtWidgets.QLabel("Dynamic Advance"))
        self.advance_label = QtWidgets.QLabel(str(self.dynamic_advance_deg))
        controls.addWidget(self.advance_label)

        self.full_button = QtWidgets.QPushButton("Start Full Test")
        self.full_button.clicked.connect(lambda: self.start_worker("full"))
        controls.addWidget(self.full_button)

        self.static_button = QtWidgets.QPushButton("Run Static Only")
        self.static_button.clicked.connect(lambda: self.start_worker("static"))
        controls.addWidget(self.static_button)

        self.dynamic_button = QtWidgets.QPushButton("Run Dynamic Only")
        self.dynamic_button.clicked.connect(lambda: self.start_worker("dynamic"))
        controls.addWidget(self.dynamic_button)

        self.clear_button = QtWidgets.QPushButton("Clear Plots")
        self.clear_button.clicked.connect(self.clear_session)
        controls.addWidget(self.clear_button)
        controls.addStretch(1)
        layout.addLayout(controls)

        self.progress = QtWidgets.QProgressBar()
        self.progress.setRange(0, 100)
        layout.addWidget(self.progress)

        self.tabs = QtWidgets.QTabWidget()
        layout.addWidget(self.tabs, stretch=1)

        self.plot_widgets: list[tuple[str, pg.PlotWidget]] = []
        self.build_raw_tab()
        self.build_static_tab()
        self.build_dynamic_tab()
        self.build_session_tab()
        self.tabs.setCurrentWidget(self.static_tab)

        terminal = QtWidgets.QHBoxLayout()
        self.command_line = QtWidgets.QLineEdit()
        self.command_line.setPlaceholderText("status | duty 120 | stop | advance 110")
        self.command_line.returnPressed.connect(self.send_terminal_command)
        terminal.addWidget(self.command_line, stretch=1)
        self.command_button = QtWidgets.QPushButton("Send")
        self.command_button.clicked.connect(self.send_terminal_command)
        terminal.addWidget(self.command_button)
        layout.addLayout(terminal)

        self.log_box = QtWidgets.QPlainTextEdit()
        self.log_box.setReadOnly(True)
        layout.addWidget(self.log_box, stretch=0)

        self.stage_text = "not run"
        self.capture_count = 0
        self.bin_count = 0
        self.refresh_ports()
        self.refresh_status()
        self.win.show()
        self.raw_timer = QtCore.QTimer()
        self.raw_timer.timeout.connect(self.poll_raw_status)
        self.raw_timer.start(RAW_POLL_MS)

        if self.auto:
            QtCore.QTimer.singleShot(0, lambda: self.start_worker(self.auto_mode))

    def make_scroll_tab(self) -> tuple[QtWidgets.QWidget, QtWidgets.QVBoxLayout]:
        outer = QtWidgets.QWidget()
        outer_layout = QtWidgets.QVBoxLayout(outer)
        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        outer_layout.addWidget(scroll)
        inner = QtWidgets.QWidget()
        inner_layout = QtWidgets.QVBoxLayout(inner)
        scroll.setWidget(inner)
        return outer, inner_layout

    def add_plot(self, layout: QtWidgets.QGridLayout, row: int, col: int, title: str, key: str) -> pg.PlotWidget:
        plot = pg.PlotWidget(title=title)
        plot.showGrid(x=True, y=True, alpha=0.25)
        layout.addWidget(plot, row, col)
        self.plot_widgets.append((key, plot))
        return plot

    @staticmethod
    def reset_plot(plot: pg.PlotWidget) -> None:
        plot.clear()
        legend = plot.plotItem.legend
        if legend is not None:
            legend.clear()

    def build_raw_tab(self) -> None:
        tab = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(tab)

        controls = QtWidgets.QHBoxLayout()
        self.raw_status_label = QtWidgets.QLabel("waiting for data")
        controls.addWidget(self.raw_status_label, stretch=1)

        self.raw_clear_button = QtWidgets.QPushButton("Clear")
        self.raw_clear_button.clicked.connect(self.clear_raw_data)
        controls.addWidget(self.raw_clear_button)

        self.raw_pause_button = QtWidgets.QPushButton("Pause")
        self.raw_pause_button.clicked.connect(self.toggle_raw_pause)
        controls.addWidget(self.raw_pause_button)
        layout.addLayout(controls)

        self.raw_mag_plot = pg.PlotWidget(title="Magnetic Field")
        self.raw_mag_plot.showGrid(x=True, y=True, alpha=0.25)
        self.raw_mag_plot.setLabel("bottom", "Time (s)")
        self.raw_mag_plot.setLabel("left", "Raw Field")
        self.raw_mag_plot.addLegend()
        layout.addWidget(self.raw_mag_plot, stretch=1)
        self.plot_widgets.append(("raw_magnetic", self.raw_mag_plot))

        self.raw_x_curve = self.raw_mag_plot.plot(name="X", pen=pg.mkPen("#e74c3c", width=2))
        self.raw_y_curve = self.raw_mag_plot.plot(name="Y", pen=pg.mkPen("#2ecc71", width=2))
        self.raw_z_curve = self.raw_mag_plot.plot(name="Z", pen=pg.mkPen("#3498db", width=2))

        self.raw_tab = tab
        self.tabs.addTab(tab, "Raw Magnetic")

    def build_static_tab(self) -> None:
        tab = QtWidgets.QWidget()
        self.static_tab = tab
        layout = QtWidgets.QVBoxLayout(tab)

        subtabs = QtWidgets.QTabWidget()
        layout.addWidget(subtabs)

        geometry_tab = QtWidgets.QWidget()
        geometry_grid = QtWidgets.QGridLayout(geometry_tab)

        self.raw_xy_plot = self.add_plot(geometry_grid, 0, 0, "Raw XY Loci", "static_raw_xy")
        self.raw_xy_plot.setLabel("bottom", "X")
        self.raw_xy_plot.setLabel("left", "Y")
        self.raw_xy_plot.setAspectLocked(True)
        self.raw_xy_plot.addLegend()
        self.static_live_forward_curve = self.raw_xy_plot.plot(
            [],
            [],
            pen=None,
            symbol="o",
            symbolSize=5,
            symbolBrush=pg.mkBrush(46, 204, 113, 140),
            name="forward live",
        )
        self.static_live_reverse_curve = self.raw_xy_plot.plot(
            [],
            [],
            pen=None,
            symbol="o",
            symbolSize=5,
            symbolBrush=pg.mkBrush(231, 76, 60, 140),
            name="reverse live",
        )

        self.corrected_plot = self.add_plot(geometry_grid, 0, 1, "Reduced + Corrected", "static_corrected")
        self.corrected_plot.setLabel("bottom", "U")
        self.corrected_plot.setLabel("left", "V")
        self.corrected_plot.setAspectLocked(True)
        self.corrected_plot.addLegend()
        subtabs.addTab(geometry_tab, "Geometry")

        residual_tab = QtWidgets.QWidget()
        residual_grid = QtWidgets.QGridLayout(residual_tab)
        self.residual_plot = self.add_plot(residual_grid, 0, 0, "Residual vs Step", "static_residual")
        self.residual_plot.setLabel("bottom", "Step")
        self.residual_plot.setLabel("left", "Error (deg)")
        self.residual_plot.addLegend()
        subtabs.addTab(residual_tab, "Residuals")

        self.tabs.addTab(tab, "Static")

    def build_dynamic_tab(self) -> None:
        tab = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(tab)

        subtabs = QtWidgets.QTabWidget()
        layout.addWidget(subtabs)

        score_tab = QtWidgets.QWidget()
        score_grid = QtWidgets.QGridLayout(score_tab)
        self.score_forward_plot = self.add_plot(score_grid, 0, 0, "Forward Score vs Advance", "dynamic_score_forward")
        self.score_reverse_plot = self.add_plot(score_grid, 0, 1, "Reverse Score vs Advance", "dynamic_score_reverse")
        subtabs.addTab(score_tab, "Score")

        mean_tab = QtWidgets.QWidget()
        mean_grid = QtWidgets.QGridLayout(mean_tab)
        self.mean_forward_plot = self.add_plot(mean_grid, 0, 0, "Forward Mean RPM vs Advance", "dynamic_mean_forward")
        self.mean_reverse_plot = self.add_plot(mean_grid, 0, 1, "Reverse Mean RPM vs Advance", "dynamic_mean_reverse")
        subtabs.addTab(mean_tab, "Mean RPM")

        ripple_tab = QtWidgets.QWidget()
        ripple_grid = QtWidgets.QGridLayout(ripple_tab)
        self.ripple_forward_plot = self.add_plot(ripple_grid, 0, 0, "Forward Ripple vs Advance", "dynamic_ripple_forward")
        self.ripple_reverse_plot = self.add_plot(ripple_grid, 0, 1, "Reverse Ripple vs Advance", "dynamic_ripple_reverse")
        subtabs.addTab(ripple_tab, "Ripple")

        best_tab = QtWidgets.QWidget()
        best_grid = QtWidgets.QGridLayout(best_tab)
        self.best_plot = self.add_plot(best_grid, 0, 0, "Best Advance vs Achieved RPM", "dynamic_best")
        self.asym_plot = self.add_plot(best_grid, 0, 1, "Asymmetry vs RPM", "dynamic_asym")
        subtabs.addTab(best_tab, "Best Fit")

        target_tab = QtWidgets.QWidget()
        target_grid = QtWidgets.QGridLayout(target_tab)
        self.target_plot = self.add_plot(target_grid, 0, 0, "Target RPM vs Achieved RPM", "dynamic_target")
        target_grid.setColumnStretch(0, 1)
        subtabs.addTab(target_tab, "Target")

        for plot in (
            self.score_forward_plot,
            self.score_reverse_plot,
            self.mean_forward_plot,
            self.mean_reverse_plot,
            self.ripple_forward_plot,
            self.ripple_reverse_plot,
            self.best_plot,
            self.asym_plot,
            self.target_plot,
        ):
            plot.addLegend()

        self.score_forward_plot.setLabel("bottom", "Advance (deg)")
        self.score_forward_plot.setLabel("left", "Score")
        self.score_reverse_plot.setLabel("bottom", "Advance (deg)")
        self.score_reverse_plot.setLabel("left", "Score")
        self.mean_forward_plot.setLabel("bottom", "Advance (deg)")
        self.mean_forward_plot.setLabel("left", "Mean RPM")
        self.mean_reverse_plot.setLabel("bottom", "Advance (deg)")
        self.mean_reverse_plot.setLabel("left", "Mean RPM")
        self.ripple_forward_plot.setLabel("bottom", "Advance (deg)")
        self.ripple_forward_plot.setLabel("left", "Ripple")
        self.ripple_reverse_plot.setLabel("bottom", "Advance (deg)")
        self.ripple_reverse_plot.setLabel("left", "Ripple")
        self.best_plot.setLabel("bottom", "Achieved RPM")
        self.best_plot.setLabel("left", "Advance (deg)")
        self.asym_plot.setLabel("bottom", "Achieved RPM")
        self.asym_plot.setLabel("left", "Forward - Reverse (deg)")
        self.target_plot.setLabel("bottom", "Target RPM")
        self.target_plot.setLabel("left", "Achieved RPM")

        self.tabs.addTab(tab, "Dynamic")

    def build_session_tab(self) -> None:
        tab = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(tab)

        header = QtWidgets.QHBoxLayout()
        header.addWidget(QtWidgets.QLabel("State"))
        self.badge_label = QtWidgets.QLabel("not run")
        font = self.badge_label.font()
        font.setPointSize(font.pointSize() + 6)
        font.setBold(True)
        self.badge_label.setFont(font)
        header.addWidget(self.badge_label)
        header.addStretch(1)
        layout.addLayout(header)

        self.summary_box = QtWidgets.QPlainTextEdit()
        self.summary_box.setReadOnly(True)
        layout.addWidget(self.summary_box, stretch=1)

        self.tabs.addTab(tab, "Summary")

    def clear_session(self) -> None:
        self.session = None
        self.capture_count = 0
        self.bin_count = 0
        self.progress.setValue(0)
        self.stage_text = "not run"
        self.badge_label.setText("not run")
        self.summary_box.setPlainText("")
        for _, plot in self.plot_widgets:
            self.reset_plot(plot)
        self.static_live_forward_x.clear()
        self.static_live_forward_y.clear()
        self.static_live_reverse_x.clear()
        self.static_live_reverse_y.clear()
        self.static_live_forward_curve = self.raw_xy_plot.plot(
            [],
            [],
            pen=None,
            symbol="o",
            symbolSize=5,
            symbolBrush=pg.mkBrush(46, 204, 113, 140),
            name="forward live",
        )
        self.static_live_reverse_curve = self.raw_xy_plot.plot(
            [],
            [],
            pen=None,
            symbol="o",
            symbolSize=5,
            symbolBrush=pg.mkBrush(231, 76, 60, 140),
            name="reverse live",
        )
        self.raw_x_curve = self.raw_mag_plot.plot(name="X", pen=pg.mkPen("#e74c3c", width=2))
        self.raw_y_curve = self.raw_mag_plot.plot(name="Y", pen=pg.mkPen("#2ecc71", width=2))
        self.raw_z_curve = self.raw_mag_plot.plot(name="Z", pen=pg.mkPen("#3498db", width=2))
        self.clear_raw_data()
        self.refresh_status()

    def append_log(self, text: str) -> None:
        self.log_box.appendPlainText(text)
        if self.auto:
            print(text, flush=True)

    def selected_port(self) -> str:
        value = self.port_box.currentData()
        if isinstance(value, str) and value:
            return value
        text = self.port_box.currentText().strip()
        if text:
            return text
        return self.port

    def refresh_ports(self) -> None:
        current = self.selected_port() if hasattr(self, "port_box") else self.port
        ports = list_serial_ports(current)
        if not ports and current:
            ports = [current]
        self.port_box.blockSignals(True)
        self.port_box.clear()
        for value in ports:
            self.port_box.addItem(value, value)
        index = self.port_box.findData(current)
        if index < 0 and ports:
            index = 0
        if index >= 0:
            self.port_box.setCurrentIndex(index)
            self.port = str(self.port_box.currentData())
        self.port_box.blockSignals(False)
        self.refresh_status()

    def on_port_changed(self) -> None:
        self.port = self.selected_port()
        self.close_client()
        self.clear_raw_data()
        self.refresh_status()

    def on_esc_changed(self) -> None:
        self.address = int(self.esc_box.currentData())
        self.clear_raw_data()
        if self.client is not None:
            self.client.address = self.address
        self.refresh_status()

    def refresh_status(self) -> None:
        self.status.setText(
            "port=%s | esc=%d | stage=%s | capture=%d/%d | bins=%d/%d | dynamic advance=%d"
            % (
                self.selected_port(),
                int(self.esc_box.currentData()),
                self.stage_text,
                self.capture_count,
                CAL_CAPTURE_TOTAL_POINTS,
                self.bin_count,
                CAL_TOTAL_POINTS,
                self.dynamic_advance_deg,
            )
        )

    def set_running(self, running: bool) -> None:
        self.full_button.setEnabled(not running)
        self.static_button.setEnabled(not running)
        self.dynamic_button.setEnabled(not running)
        self.clear_button.setEnabled(not running)
        self.port_box.setEnabled(not running)
        self.refresh_ports_button.setEnabled(not running)
        self.esc_box.setEnabled(not running)
        self.speed_edit.setEnabled(not running)
        self.command_line.setEnabled(not running)
        self.command_button.setEnabled(not running)

    def toggle_raw_pause(self) -> None:
        self.raw_paused = not self.raw_paused
        self.raw_pause_button.setText("Resume" if self.raw_paused else "Pause")
        self.update_raw_status_label()

    def clear_raw_data(self) -> None:
        self.raw_times.clear()
        self.raw_x_vals.clear()
        self.raw_y_vals.clear()
        self.raw_z_vals.clear()
        self.raw_start_time = None
        self.raw_last_values = (0, 0, 0)
        self.raw_last_status = "waiting for data"
        self.raw_x_curve.setData([], [])
        self.raw_y_curve.setData([], [])
        self.raw_z_curve.setData([], [])
        self.update_raw_status_label()

    def update_raw_status_label(self) -> None:
        x, y, z = self.raw_last_values
        paused = "paused | " if self.raw_paused else ""
        self.raw_status_label.setText(
            f"port={self.selected_port()} | esc={int(self.esc_box.currentData())} | "
            f"X={x} Y={y} Z={z} | {paused}{self.raw_last_status}"
        )

    def append_raw_sample(self, status) -> None:
        now = time.monotonic()
        if self.raw_start_time is None:
            self.raw_start_time = now
        sample_time = now - self.raw_start_time
        self.raw_times.append(sample_time)
        self.raw_x_vals.append(status.x)
        self.raw_y_vals.append(status.y)
        self.raw_z_vals.append(status.z)
        self.raw_last_values = (status.x, status.y, status.z)
        cutoff = sample_time - RAW_PLOT_WINDOW_S
        while self.raw_times and self.raw_times[0] < cutoff:
            self.raw_times.popleft()
            self.raw_x_vals.popleft()
            self.raw_y_vals.popleft()
            self.raw_z_vals.popleft()

    def update_raw_plot(self) -> None:
        if not self.raw_times:
            return
        times = list(self.raw_times)
        self.raw_x_curve.setData(times, list(self.raw_x_vals))
        self.raw_y_curve.setData(times, list(self.raw_y_vals))
        self.raw_z_curve.setData(times, list(self.raw_z_vals))
        latest = times[-1]
        start = max(0.0, latest - RAW_PLOT_WINDOW_S)
        self.raw_mag_plot.setXRange(start, start + RAW_PLOT_WINDOW_S, padding=0.0)

    def poll_raw_status(self) -> None:
        if self.tabs.currentWidget() is not self.raw_tab:
            return
        if self.client_busy:
            return
        if self.worker is not None and self.worker.isRunning():
            self.raw_last_status = "test running"
            self.update_raw_status_label()
            return

        self.client_busy = True
        try:
            status = self.ensure_client().get_status(timeout=RAW_STATUS_TIMEOUT_S, attempts=RAW_STATUS_ATTEMPTS)
        except TimeoutError:
            self.raw_last_status = "dropped sample"
            self.update_raw_status_label()
        except (CalibrationError, serial.SerialException) as exc:
            self.close_client()
            self.raw_last_status = f"error: {exc}"
            self.update_raw_status_label()
        else:
            self.raw_last_status = "mode=%d flags=0x%02X faults=0x%02X mct_faults=%d" % (
                status.mode,
                status.flags,
                status.faults,
                status.mct_fault_count,
            )
            self.raw_last_values = (status.x, status.y, status.z)
            if not self.raw_paused:
                self.append_raw_sample(status)
                self.update_raw_plot()
            self.update_raw_status_label()
        finally:
            self.client_busy = False

    def ensure_client(self) -> Stm32Client:
        address = int(self.esc_box.currentData())
        port = self.selected_port()
        if self.bridge is None or self.client is None:
            self.bridge = EspBridge(port)
            try:
                self.bridge.enter_bridge("app")
                self.client = Stm32Client(self.bridge.serial, address=address)
            except Exception:
                self.close_client()
                raise
        self.port = port
        self.client.address = address
        return self.client

    def close_client(self) -> None:
        if self.bridge is None:
            self.client = None
            return
        try:
            try:
                self.bridge.exit_bridge()
            except Exception:
                pass
            self.bridge.close()
        finally:
            self.bridge = None
            self.client = None

    def send_terminal_command(self) -> None:
        text = self.command_line.text().strip()
        if not text:
            return
        self.command_line.clear()
        self.append_log(f"> {text}")
        if self.worker is not None and self.worker.isRunning():
            self.append_log("error: stop current test before sending commands")
            return
        if self.client_busy:
            self.append_log("error: serial busy")
            return
        self.client_busy = True
        try:
            self.run_terminal_command(text)
        except (CalibrationError, TimeoutError, ValueError, serial.SerialException) as exc:
            if isinstance(exc, (TimeoutError, serial.SerialException)):
                self.close_client()
            self.append_log(f"error: {exc}")
        finally:
            self.client_busy = False

    def run_terminal_command(self, text: str) -> None:
        parts = text.split()
        cmd = parts[0].lower()
        if cmd == "help":
            self.append_log("commands: status | duty <value> | stop | advance <deg>")
            return

        client = self.ensure_client()

        if cmd == "status":
            status = client.get_status()
            self.append_log(format_status(status))
            return

        if cmd == "stop":
            status = client.set_duty(0)
            self.append_log(format_status(status))
            return

        if cmd == "duty":
            if len(parts) != 2:
                raise ValueError("usage: duty <value>")
            duty = int(parts[1], 0)
            status = client.set_duty(duty)
            self.append_log(format_status(status))
            return

        if cmd == "advance":
            if len(parts) != 2:
                raise ValueError("usage: advance <deg>")
            advance_deg = int(parts[1], 0)
            status = client.set_advance_deg(advance_deg)
            self.dynamic_advance_deg = advance_deg
            self.advance_label.setText(str(self.dynamic_advance_deg))
            self.refresh_status()
            self.append_log(f"dynamic advance default={advance_deg}")
            self.append_log(format_status(status))
            return

        raise ValueError("unknown command")

    def start_worker(self, mode: str) -> None:
        if self.worker is not None and self.worker.isRunning():
            return
        if mode in {"full", "dynamic"}:
            try:
                parse_speed_text(self.speed_edit.text())
            except ValueError as exc:
                self.append_log(f"error: {exc}")
                return

        self.close_client()
        self.capture_count = 0
        self.bin_count = 0
        self.static_live_forward_x.clear()
        self.static_live_forward_y.clear()
        self.static_live_reverse_x.clear()
        self.static_live_reverse_y.clear()
        self.static_live_forward_curve.setData([], [])
        self.static_live_reverse_curve.setData([], [])
        self.address = int(self.esc_box.currentData())
        self.port = self.selected_port()
        self.stage_text = f"{mode} start"
        self.refresh_status()
        self.append_log(f"start {mode}")
        self.set_running(True)

        self.worker = TestWorker(
            port=self.port,
            address=self.address,
            chunk_size=self.chunk_size,
            mode=mode,
            speed_text=self.speed_edit.text(),
            base_advance_deg=self.dynamic_advance_deg,
        )
        self.worker.log.connect(self.on_log)
        self.worker.capture_status.connect(self.on_capture_status)
        self.worker.capture_sample.connect(self.on_capture_sample)
        self.worker.upload.connect(self.on_upload)
        self.worker.progress.connect(self.progress.setValue)
        self.worker.session_ready.connect(self.on_session_ready)
        self.worker.done.connect(self.on_done)
        self.worker.failed.connect(self.on_failed)
        self.worker.start()

    def on_log(self, text: str) -> None:
        self.stage_text = text
        self.append_log(text)
        self.refresh_status()

    def on_capture_status(self, active: int, next_index: int, _total: int) -> None:
        self.capture_count = next_index
        self.stage_text = "capture active" if active else "capture done"
        self.refresh_status()

    def on_capture_sample(self, sweep_dir: int, x: int, y: int, _z: int, _faults: int) -> None:
        if sweep_dir == 0:
            self.static_live_forward_x.append(x)
            self.static_live_forward_y.append(y)
            self.static_live_forward_curve.setData(self.static_live_forward_x, self.static_live_forward_y)
        else:
            self.static_live_reverse_x.append(x)
            self.static_live_reverse_y.append(y)
            self.static_live_reverse_curve.setData(self.static_live_reverse_x, self.static_live_reverse_y)

    def on_upload(self, done: int, total: int) -> None:
        self.stage_text = "upload"
        if total > 0:
            self.bin_count = CAL_TOTAL_POINTS
        self.refresh_status()

    def on_session_ready(self, session: SessionData) -> None:
        self.session = session
        self.update_static_plots()
        self.update_dynamic_plots()
        self.update_summary()
        self.export_session()

    def on_done(self, text: str) -> None:
        self.stage_text = text
        self.progress.setValue(100)
        self.append_log(text)
        self.set_running(False)
        self.refresh_status()
        if self.auto:
            QtCore.QTimer.singleShot(200, self.app.quit)

    def on_failed(self, text: str) -> None:
        self.stage_text = "failed"
        self.append_log(f"error: {text}")
        self.set_running(False)
        self.refresh_status()
        if self.auto:
            self.app.exit(1)

    def update_static_plots(self) -> None:
        if self.session is None or self.session.static is None:
            return

        static = self.session.static
        self.reset_plot(self.raw_xy_plot)
        raw_forward_x = [row["x"] for row in static.raw_points if row["sweep_dir"] == 0]
        raw_forward_y = [row["y"] for row in static.raw_points if row["sweep_dir"] == 0]
        raw_reverse_x = [row["x"] for row in static.raw_points if row["sweep_dir"] == 1]
        raw_reverse_y = [row["y"] for row in static.raw_points if row["sweep_dir"] == 1]
        self.raw_xy_plot.plot(
            raw_forward_x,
            raw_forward_y,
            pen=None,
            symbol="o",
            symbolSize=4,
            symbolBrush=pg.mkBrush(46, 204, 113, 100),
            name="forward raw",
        )
        self.raw_xy_plot.plot(
            raw_reverse_x,
            raw_reverse_y,
            pen=None,
            symbol="o",
            symbolSize=4,
            symbolBrush=pg.mkBrush(231, 76, 60, 100),
            name="reverse raw",
        )

        reduced = sorted(static.reduced_points, key=lambda row: row["index"])
        fwd = sorted((row for row in reduced if row["sweep_dir"] == 0), key=lambda row: row["step_index"])
        rev = sorted((row for row in reduced if row["sweep_dir"] == 1), key=lambda row: row["step_index"])
        self.raw_xy_plot.plot(
            [row["x"] for row in fwd],
            [row["y"] for row in fwd],
            pen=pg.mkPen("#27ae60", width=2),
            symbol="x",
            symbolSize=8,
            name="forward reduced",
        )
        self.raw_xy_plot.plot(
            [row["x"] for row in rev],
            [row["y"] for row in rev],
            pen=pg.mkPen("#c0392b", width=2),
            symbol="x",
            symbolSize=8,
            name="reverse reduced",
        )

        if static.affine_q20 is None or static.angle_lut is None:
            self.bin_count = len(static.reduced_points)
            self.refresh_status()
            return

        self.reset_plot(self.corrected_plot)
        self.reset_plot(self.residual_plot)
        coeffs = static.affine_q20
        lut = static.angle_lut
        for rows, name, color in (
            (fwd, "forward corrected", "#27ae60"),
            (rev, "reverse corrected", "#c0392b"),
        ):
            xs = np.array([row["x"] for row in rows], dtype=np.int64)
            ys = np.array([row["y"] for row in rows], dtype=np.int64)
            u, v = apply_affine_q20(xs, ys, coeffs)
            self.corrected_plot.plot(
                (u / AFFINE_TARGET_SCALE).tolist(),
                (v / AFFINE_TARGET_SCALE).tolist(),
                pen=pg.mkPen(color, width=2),
                symbol="o",
                symbolSize=6,
                name=name,
            )

        circle = np.linspace(0.0, 2.0 * math.pi, 241)
        self.corrected_plot.plot(np.cos(circle), np.sin(circle), pen=pg.mkPen("#7f8c8d", width=1), name="unit circle")

        step_x = list(range(CAL_POINTS_PER_SWEEP))
        forward_err: list[float] = []
        reverse_err: list[float] = []
        target_turn16 = np.arange(CAL_POINTS_PER_SWEEP, dtype=np.float64) * (65536.0 / CAL_POINTS_PER_SWEEP)
        for step in range(CAL_POINTS_PER_SWEEP):
            fa = angle_from_xy(fwd[step]["x"], fwd[step]["y"], coeffs, lut)
            ra = angle_from_xy(rev[step]["x"], rev[step]["y"], coeffs, lut)
            forward_err.append(signed_turn16_error_deg(fa, target_turn16[step]))
            reverse_err.append(signed_turn16_error_deg(ra, target_turn16[step]))
        even = [(f + r) * 0.5 for f, r in zip(forward_err, reverse_err)]
        hyst = [(f - r) * 0.5 for f, r in zip(forward_err, reverse_err)]
        self.residual_plot.plot(step_x, forward_err, pen=pg.mkPen("#27ae60", width=2), name="forward")
        self.residual_plot.plot(step_x, reverse_err, pen=pg.mkPen("#c0392b", width=2), name="reverse")
        self.residual_plot.plot(step_x, even, pen=pg.mkPen("#2980b9", width=2), name="even")
        self.residual_plot.plot(step_x, hyst, pen=pg.mkPen("#8e44ad", width=2), name="hysteresis")
        self.bin_count = CAL_TOTAL_POINTS
        self.refresh_status()

    def update_dynamic_plots(self) -> None:
        for plot in (
            self.score_forward_plot,
            self.score_reverse_plot,
            self.mean_forward_plot,
            self.mean_reverse_plot,
            self.ripple_forward_plot,
            self.ripple_reverse_plot,
            self.best_plot,
            self.asym_plot,
            self.target_plot,
        ):
            self.reset_plot(plot)

        if self.session is None or self.session.dynamic is None:
            return

        dynamic = self.session.dynamic
        speed_index = {speed: idx for idx, speed in enumerate(dynamic.input_speeds_rpm)}

        for direction in (1, -1):
            direction_points = [point for point in dynamic.points if point.direction == direction]
            score_plot = self.score_forward_plot if direction > 0 else self.score_reverse_plot
            mean_plot = self.mean_forward_plot if direction > 0 else self.mean_reverse_plot
            ripple_plot = self.ripple_forward_plot if direction > 0 else self.ripple_reverse_plot

            grouped: dict[float, list[SweepPoint]] = {}
            for point in direction_points:
                grouped.setdefault(point.target_rpm, []).append(point)

            for speed in dynamic.input_speeds_rpm:
                items = sorted(grouped.get(speed, []), key=lambda point: point.advance_deg)
                if not items:
                    continue
                pen = line_color(speed_index[speed])
                name = f"{speed:.0f} rpm"
                advances = [point.advance_deg for point in items]
                scores = [point.score for point in items]
                means = [point.mean_rpm for point in items]
                ripples = [point.ripple for point in items]
                score_plot.plot(advances, scores, pen=pen, symbol="o", symbolSize=5, name=name)
                mean_plot.plot(advances, means, pen=pen, symbol="o", symbolSize=5, name=name)
                ripple_plot.plot(advances, ripples, pen=pen, symbol="o", symbolSize=5, name=name)

        forward_best = sorted([point for point in dynamic.best_points if point.direction > 0], key=lambda point: point.achieved_rpm)
        reverse_best = sorted([point for point in dynamic.best_points if point.direction < 0], key=lambda point: point.achieved_rpm)

        if forward_best:
            self.best_plot.plot(
                [point.achieved_rpm for point in forward_best],
                [point.advance_deg for point in forward_best],
                pen=None,
                symbol="o",
                symbolSize=7,
                symbolBrush=pg.mkBrush("#27ae60"),
                name="forward",
            )
            self.target_plot.plot(
                [point.target_rpm for point in forward_best],
                [point.achieved_rpm for point in forward_best],
                pen=None,
                symbol="o",
                symbolSize=7,
                symbolBrush=pg.mkBrush("#27ae60"),
                name="forward achieved",
            )

        if reverse_best:
            self.best_plot.plot(
                [point.achieved_rpm for point in reverse_best],
                [point.advance_deg for point in reverse_best],
                pen=None,
                symbol="t",
                symbolSize=8,
                symbolBrush=pg.mkBrush("#c0392b"),
                name="reverse",
            )
            self.target_plot.plot(
                [point.target_rpm for point in reverse_best],
                [point.achieved_rpm for point in reverse_best],
                pen=None,
                symbol="t",
                symbolSize=8,
                symbolBrush=pg.mkBrush("#c0392b"),
                name="reverse achieved",
            )

        averaged = dynamic.fit.averaged_points
        if averaged:
            self.best_plot.plot(
                [point["achieved_rpm"] for point in averaged],
                [point["average_advance_deg"] for point in averaged],
                pen=pg.mkPen("#2980b9", width=2),
                symbol="x",
                symbolSize=8,
                name="average",
            )
            self.asym_plot.plot(
                [point["achieved_rpm"] for point in averaged],
                [point["asymmetry_deg"] for point in averaged],
                pen=pg.mkPen("#8e44ad", width=2),
                symbol="o",
                symbolSize=7,
                name="asymmetry",
            )

        fit = dynamic.fit
        if fit.slope_deg_per_rpm is not None and fit.intercept_deg is not None and averaged:
            xs = np.array([point["achieved_rpm"] for point in averaged], dtype=np.float64)
            span = np.linspace(np.min(xs), np.max(xs), 128)
            ys = fit.intercept_deg + fit.slope_deg_per_rpm * span
            self.best_plot.plot(span.tolist(), ys.tolist(), pen=pg.mkPen("#34495e", width=2), name="fit")

        if dynamic.input_speeds_rpm:
            min_speed = min(dynamic.input_speeds_rpm)
            max_speed = max(dynamic.input_speeds_rpm)
            self.target_plot.plot(
                [min_speed, max_speed],
                [min_speed, max_speed],
                pen=pg.mkPen("#7f8c8d", width=1, style=QtCore.Qt.PenStyle.DashLine),
                name="target=achieved",
            )

    def update_summary(self) -> None:
        if self.session is None:
            self.badge_label.setText("not run")
            self.summary_box.setPlainText("")
            return

        self.badge_label.setText(self.session.status)
        lines = [
            f"mode: {self.session.mode}",
            f"port: {self.session.port}",
            f"esc: {self.session.address}",
            f"started: {self.session.started_at}",
            f"ended: {self.session.ended_at or '-'}",
            f"export: {self.session.export_dir}",
        ]
        if self.session.static is not None:
            lines.extend(["", "static:"])
            if self.session.static.fit_max_error_deg is not None:
                lines.append(f"  fit max error: {self.session.static.fit_max_error_deg:.2f} deg")
            if self.session.static.sweep_delta_deg is not None:
                lines.append(f"  sweep delta: {self.session.static.sweep_delta_deg:.2f} deg")
            if self.session.static.blob_crc32 is not None:
                lines.append(f"  blob crc32: 0x{self.session.static.blob_crc32:08X}")
            if self.session.static.fit_max_error_deg is None:
                lines.append("  solve: failed before calibration blob was built")
        if self.session.dynamic is not None:
            dynamic = self.session.dynamic
            speed_text = ", ".join(f"{speed:.0f}" for speed in dynamic.input_speeds_rpm)
            lines.extend(
                [
                    "",
                    "dynamic:",
                    f"  speeds: {speed_text}",
                    f"  base advance: {dynamic.base_advance_deg}",
                    f"  state: {dynamic.state}",
                    f"  notes: {len(dynamic.notes)}",
                ]
            )
            if dynamic.fit.slope_deg_per_rpm is not None and dynamic.fit.intercept_deg is not None:
                lines.extend(
                    [
                        f"  intercept: {dynamic.fit.intercept_deg:.4f} deg",
                        f"  slope: {dynamic.fit.slope_deg_per_rpm:.6f} deg/rpm",
                        f"  delay: {dynamic.fit.delay_s:.6f} s",
                    ]
                )
            if dynamic.notes:
                lines.append("  note list:")
                for note in dynamic.notes:
                    lines.append(f"    {note}")
        if self.session.error:
            lines.extend(["", f"error: {self.session.error}"])
        self.summary_box.setPlainText("\n".join(lines))

    def export_session(self) -> None:
        if self.session is None:
            return
        export_dir = Path(self.session.export_dir)
        export_dir.mkdir(parents=True, exist_ok=True)

        metadata = to_serializable(self.session)
        with (export_dir / "run_metadata.json").open("w", encoding="utf-8") as handle:
            json.dump(metadata, handle, indent=2, sort_keys=True)

        if self.session.static is not None:
            self.write_csv(export_dir / "static_raw.csv", self.session.static.raw_points)
            self.write_csv(export_dir / "static_reduced.csv", self.session.static.reduced_points)
            if self.session.static.affine_q20 is not None:
                summary = {
                    "fit_max_error_deg": self.session.static.fit_max_error_deg,
                    "sweep_delta_deg": self.session.static.sweep_delta_deg,
                    "blob_crc32": self.session.static.blob_crc32,
                    "blob_size": self.session.static.blob_size,
                    "affine_q20": self.session.static.affine_q20,
                }
                with (export_dir / "static_summary.json").open("w", encoding="utf-8") as handle:
                    json.dump(to_serializable(summary), handle, indent=2, sort_keys=True)

        if self.session.dynamic is not None:
            dynamic = self.session.dynamic
            with (export_dir / "dynamic_speeds.json").open("w", encoding="utf-8") as handle:
                json.dump(
                    {
                        "input_speeds_rpm": dynamic.input_speeds_rpm,
                        "base_advance_deg": dynamic.base_advance_deg,
                    },
                    handle,
                    indent=2,
                    sort_keys=True,
                )
            point_rows = [
                {
                    "direction": direction_name(point.direction),
                    "target_rpm": point.target_rpm,
                    "advance_deg": point.advance_deg,
                    "duty": point.duty,
                    "mean_rpm": point.mean_rpm,
                    "std_rpm": point.std_rpm,
                    "ripple": point.ripple,
                    "score": point.score,
                    "valid": int(point.valid),
                    "sign_flip": int(point.sign_flip),
                    "faults": point.faults,
                    "sample_count": point.sample_count,
                    "target_error_rpm": point.target_error_rpm,
                }
                for point in dynamic.points
            ]
            self.write_csv(export_dir / "dynamic_sweeps.csv", point_rows)
            sample_rows: list[dict[str, Any]] = []
            for point in dynamic.points:
                for sample in point.samples:
                    sample_rows.append(
                        {
                            "direction": direction_name(point.direction),
                            "target_rpm": point.target_rpm,
                            "advance_deg": point.advance_deg,
                            "duty": point.duty,
                            **sample,
                        }
                    )
            self.write_csv(export_dir / "dynamic_samples.csv", sample_rows)
            with (export_dir / "dynamic_best.json").open("w", encoding="utf-8") as handle:
                json.dump(to_serializable(dynamic.best_points), handle, indent=2, sort_keys=True)
            with (export_dir / "dynamic_fit.json").open("w", encoding="utf-8") as handle:
                json.dump(to_serializable(dynamic.fit), handle, indent=2, sort_keys=True)

        for key, plot in self.plot_widgets:
            path = export_dir / f"{key}.png"
            exporter = pyqtgraph.exporters.ImageExporter(plot.plotItem)
            exporter.export(str(path))

    @staticmethod
    def write_csv(path: Path, rows: Sequence[dict[str, Any]]) -> None:
        if not rows:
            path.write_text("", encoding="utf-8")
            return
        fieldnames: list[str] = []
        for row in rows:
            for key in row:
                if key not in fieldnames:
                    fieldnames.append(key)
        with path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=fieldnames)
            writer.writeheader()
            for row in rows:
                writer.writerow({key: to_serializable(value) for key, value in row.items()})


def main() -> int:
    parser = argparse.ArgumentParser(description="pennyesc calibration test GUI")
    parser.add_argument("--port")
    parser.add_argument("--address", type=int, default=1)
    parser.add_argument("--chunk-size", type=int, default=DEFAULT_CHUNK_SIZE)
    parser.add_argument("--auto", action="store_true", help="start immediately and exit when complete")
    parser.add_argument("--mode", choices=["full", "static", "dynamic"], default="full")
    parser.add_argument("--speeds", default="1500,2500,3500")
    args = parser.parse_args()

    port = find_serial_port(args.port)
    window = Window(port, args.address, args.chunk_size, args.auto, args.mode, args.speeds)
    return window.app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
