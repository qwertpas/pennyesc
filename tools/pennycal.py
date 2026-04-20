from __future__ import annotations

import argparse
import csv
import dataclasses
import math
import struct
import time
import zlib
from pathlib import Path
from typing import Callable, Iterable, Sequence

import numpy as np
import serial
from pnyproto import (
    FAULT_FLASH,
    FAULT_SENSOR,
    FAULT_UNCALIBRATED,
    FLAG_CAL_VALID,
    FLAG_BUSY,
    FLAG_FAULT,
    FLAG_POSITION_REACHED,
    FLAG_SENSOR_OK,
    FRAME_MAX_PAYLOAD,
    FRAME_START,
    MODE_CAL,
    MODE_IDLE,
    MODE_RUN,
    RESULT_OK,
    CMD_CAL_CLEAR,
    CMD_CAL_COMMIT,
    CMD_CAL_INFO,
    CMD_CAL_READ_POINT,
    CMD_CAL_START,
    CMD_CAL_STATUS,
    CMD_CAL_WRITE_BLOB,
    CMD_SET_ADVANCE,
    CMD_GET_STATUS,
    CMD_SET_DUTY,
    CMD_SET_POSITION,
    crc8,
    decode_frame,
    encode_frame,
)

LUT_BITS = 5
SEGMENT_SIZE = 1 << LUT_BITS
LUT_SIZE = 8 * SEGMENT_SIZE

CAL_POINTS_PER_SWEEP = 36
CAL_SWEEP_COUNT = 2
CAL_TOTAL_POINTS = CAL_POINTS_PER_SWEEP * CAL_SWEEP_COUNT
CAL_ROTATIONS_PER_DIR = 3
CAL_CAPTURE_POINTS_PER_DIR = CAL_POINTS_PER_SWEEP * CAL_ROTATIONS_PER_DIR
CAL_CAPTURE_TOTAL_POINTS = CAL_TOTAL_POINTS * CAL_ROTATIONS_PER_DIR

CAL_MAGIC = 0x314C4143
CAL_VERSION = 1
CAL_BLOB_SIZE = 640
AFFINE_SHIFT = 20
AFFINE_TARGET_SCALE = 16384.0
DEFAULT_CHUNK_SIZE = 48
BRIDGE_IDLE_S = 2.7
MAX_FIT_ERROR_DEG = 8.0
MAX_SWEEP_DELTA_DEG = 12.5


class CalibrationError(RuntimeError):
    pass


@dataclasses.dataclass(frozen=True)
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
    def calibrated(self) -> bool:
        return bool(self.flags & FLAG_CAL_VALID)


@dataclasses.dataclass(frozen=True)
class CalStatus:
    result: int
    active: int
    next_index: int
    total_points: int


@dataclasses.dataclass(frozen=True)
class CapturePoint:
    index: int
    step_index: int
    sweep_dir: int
    x: int
    y: int
    z: int
    xy_radius: int
    sample_spread: int


@dataclasses.dataclass(frozen=True)
class CalibrationInfo:
    result: int
    valid: int
    blob_size: int
    blob_crc32: int


@dataclasses.dataclass(frozen=True)
class SolvedCalibration:
    affine_q20: tuple[int, int, int, int, int, int]
    angle_lut: tuple[int, ...]
    fit_max_error_deg: float
    sweep_delta_deg: float
    blob: bytes
    blob_crc32: int


def cal_point_step_index(point_index: int) -> int:
    local = point_index % CAL_POINTS_PER_SWEEP
    if (point_index // CAL_POINTS_PER_SWEEP) == 0:
        return local
    return CAL_POINTS_PER_SWEEP - 1 - local


def cal_sweep_dir(point_index: int) -> int:
    return 0 if point_index < CAL_POINTS_PER_SWEEP else 1


def cal_capture_dir(raw_index: int) -> int:
    return 0 if raw_index < CAL_CAPTURE_POINTS_PER_DIR else 1


def cal_capture_rotation(raw_index: int) -> int:
    if cal_capture_dir(raw_index) == 0:
        return raw_index // CAL_POINTS_PER_SWEEP
    return (raw_index - CAL_CAPTURE_POINTS_PER_DIR) // CAL_POINTS_PER_SWEEP


def cal_capture_step_index(raw_index: int) -> int:
    local = raw_index % CAL_POINTS_PER_SWEEP
    if cal_capture_dir(raw_index) == 0:
        return local
    return CAL_POINTS_PER_SWEEP - 1 - local


def pseudo_index(x: int, y: int) -> int:
    ax = abs(x)
    ay = abs(y)
    if ax > ay:
        if x >= 0:
            octant = 0 if y >= 0 else 7
        else:
            octant = 3 if y >= 0 else 4
        ratio = 0 if ax == 0 else (ay * SEGMENT_SIZE) // ax
    else:
        if y >= 0:
            octant = 1 if x >= 0 else 2
        else:
            octant = 6 if x >= 0 else 5
        ratio = 0 if ay == 0 else (ax * SEGMENT_SIZE) // ay

    if ratio >= SEGMENT_SIZE:
        ratio = SEGMENT_SIZE - 1
    if octant & 1:
        ratio = (SEGMENT_SIZE - 1) - ratio
    return (octant << LUT_BITS) | ratio


def apply_affine_q20(x: np.ndarray, y: np.ndarray, coeffs_q20: Sequence[int]) -> tuple[np.ndarray, np.ndarray]:
    coeffs = np.asarray(coeffs_q20, dtype=np.int64)
    u = ((x.astype(np.int64) * coeffs[0]) + (y.astype(np.int64) * coeffs[1]) + coeffs[2]) >> AFFINE_SHIFT
    v = ((x.astype(np.int64) * coeffs[3]) + (y.astype(np.int64) * coeffs[4]) + coeffs[5]) >> AFFINE_SHIFT
    return u.astype(np.int64), v.astype(np.int64)


def turn16_error_deg(measured: int, target: float) -> float:
    delta = ((measured - target + 32768.0) % 65536.0) - 32768.0
    return abs(delta) * 360.0 / 65536.0


def _build_lut(anchor_indices: np.ndarray, anchor_turn16: np.ndarray) -> np.ndarray:
    diffs = np.mod(np.roll(anchor_indices, -1) - anchor_indices, LUT_SIZE)
    if np.any(diffs == 0) or int(diffs.sum()) != LUT_SIZE:
        raise CalibrationError("pseudo-angle ordering is not strictly circular")

    x = np.empty(CAL_POINTS_PER_SWEEP + 1, dtype=np.float64)
    x[0] = float(anchor_indices[0])
    for idx in range(1, CAL_POINTS_PER_SWEEP):
        x[idx] = x[idx - 1] + float(diffs[idx - 1])
    x[-1] = x[0] + LUT_SIZE

    y = np.empty(CAL_POINTS_PER_SWEEP + 1, dtype=np.float64)
    y[:-1] = anchor_turn16
    y[-1] = 65536.0

    lut = np.empty(LUT_SIZE, dtype=np.uint16)
    start = x[0]
    for idx in range(LUT_SIZE):
        sample = float(idx)
        if sample < start:
            sample += LUT_SIZE
        lut[idx] = int(round(np.interp(sample, x, y))) & 0xFFFF
    return lut


def _solve_anchor_set(xs: np.ndarray, ys: np.ndarray) -> tuple[tuple[int, ...], tuple[int, ...], float]:
    if xs.shape != (CAL_POINTS_PER_SWEEP,) or ys.shape != (CAL_POINTS_PER_SWEEP,):
        raise CalibrationError("expected 36 anchors")

    angles = np.arange(CAL_POINTS_PER_SWEEP, dtype=np.float64) * (2.0 * math.pi / CAL_POINTS_PER_SWEEP)
    design = np.column_stack((xs, ys, np.ones(CAL_POINTS_PER_SWEEP, dtype=np.float64)))
    target_u = np.cos(angles) * AFFINE_TARGET_SCALE
    target_v = np.sin(angles) * AFFINE_TARGET_SCALE

    coeff_u = np.linalg.lstsq(design, target_u, rcond=None)[0]
    coeff_v = np.linalg.lstsq(design, target_v, rcond=None)[0]
    coeffs_q20 = tuple(int(round(value * (1 << AFFINE_SHIFT))) for value in np.concatenate((coeff_u, coeff_v)))

    u, v = apply_affine_q20(xs, ys, coeffs_q20)
    anchor_indices = np.array([pseudo_index(int(ui), int(vi)) for ui, vi in zip(u, v)], dtype=np.int64)
    anchor_turn16 = np.arange(CAL_POINTS_PER_SWEEP, dtype=np.float64) * (65536.0 / CAL_POINTS_PER_SWEEP)
    lut = _build_lut(anchor_indices, anchor_turn16)

    corrected_turn16 = (np.arctan2(v.astype(np.float64), u.astype(np.float64)) % (2.0 * math.pi)) * (
        65536.0 / (2.0 * math.pi)
    )
    fit_errors = [turn16_error_deg(int(round(measured)), target) for measured, target in zip(corrected_turn16, anchor_turn16)]
    return coeffs_q20, tuple(int(value) for value in lut.tolist()), max(fit_errors)


def build_blob(
    affine_q20: Sequence[int],
    angle_lut: Sequence[int],
    fit_max_error_deg: float,
    sweep_delta_deg: float,
) -> bytes:
    if len(affine_q20) != 6:
        raise ValueError("expected 6 affine coefficients")
    if len(angle_lut) != LUT_SIZE:
        raise ValueError("expected 256 LUT entries")

    fit_x100 = max(0, min(0xFFFF, int(round(fit_max_error_deg * 100.0))))
    sweep_x100 = max(0, min(0xFFFF, int(round(sweep_delta_deg * 100.0))))

    body = struct.pack(
        "<6i256H2H88x",
        *affine_q20,
        *angle_lut,
        fit_x100,
        sweep_x100,
    )
    crc32 = zlib.crc32(body) & 0xFFFFFFFF
    blob = struct.pack("<IHHI", CAL_MAGIC, CAL_VERSION, CAL_BLOB_SIZE, crc32) + body
    if len(blob) != CAL_BLOB_SIZE:
        raise ValueError("blob size mismatch")
    return blob


def validate_blob(blob: bytes) -> tuple[bool, int]:
    if len(blob) != CAL_BLOB_SIZE:
        return False, 0
    magic, version, size, crc32 = struct.unpack_from("<IHHI", blob, 0)
    if magic != CAL_MAGIC or version != CAL_VERSION or size != CAL_BLOB_SIZE:
        return False, 0
    calc = zlib.crc32(blob[12:]) & 0xFFFFFFFF
    return calc == crc32, crc32


def iter_blob_chunks(blob: bytes, chunk_size: int = DEFAULT_CHUNK_SIZE) -> Iterable[tuple[int, bytes]]:
    if chunk_size <= 0 or chunk_size > (FRAME_MAX_PAYLOAD - 2):
        raise ValueError("invalid chunk size")
    for offset in range(0, len(blob), chunk_size):
        yield offset, blob[offset : offset + chunk_size]


def solve_capture(points: Sequence[CapturePoint]) -> SolvedCalibration:
    if len(points) != CAL_TOTAL_POINTS:
        raise CalibrationError("capture did not return all 72 points")

    ordered = sorted(points, key=lambda point: point.index)
    if [point.index for point in ordered] != list(range(CAL_TOTAL_POINTS)):
        raise CalibrationError("capture indices are incomplete")

    forward_x = np.zeros(CAL_POINTS_PER_SWEEP, dtype=np.int64)
    forward_y = np.zeros(CAL_POINTS_PER_SWEEP, dtype=np.int64)
    reverse_x = np.zeros(CAL_POINTS_PER_SWEEP, dtype=np.int64)
    reverse_y = np.zeros(CAL_POINTS_PER_SWEEP, dtype=np.int64)
    have_forward = [False] * CAL_POINTS_PER_SWEEP
    have_reverse = [False] * CAL_POINTS_PER_SWEEP

    for point in ordered:
        if not 0 <= point.step_index < CAL_POINTS_PER_SWEEP:
            raise CalibrationError("step index out of range")
        if point.sweep_dir == 0:
            forward_x[point.step_index] = point.x
            forward_y[point.step_index] = point.y
            have_forward[point.step_index] = True
        elif point.sweep_dir == 1:
            reverse_x[point.step_index] = point.x
            reverse_y[point.step_index] = point.y
            have_reverse[point.step_index] = True
        else:
            raise CalibrationError("invalid sweep direction")

    if not all(have_forward) or not all(have_reverse):
        raise CalibrationError("missing forward or reverse anchors")

    anchor_x = ((forward_x + reverse_x) // 2).astype(np.int64)
    anchor_y = ((forward_y + reverse_y) // 2).astype(np.int64)
    affine_q20, angle_lut, fit_max_error_deg = _solve_anchor_set(anchor_x, anchor_y)
    lut = np.asarray(angle_lut, dtype=np.uint16)

    sweep_deltas = []
    for step in range(CAL_POINTS_PER_SWEEP):
        raw_fx = np.array([forward_x[step]], dtype=np.int64)
        raw_fy = np.array([forward_y[step]], dtype=np.int64)
        raw_rx = np.array([reverse_x[step]], dtype=np.int64)
        raw_ry = np.array([reverse_y[step]], dtype=np.int64)
        fu, fv = apply_affine_q20(raw_fx, raw_fy, affine_q20)
        ru, rv = apply_affine_q20(raw_rx, raw_ry, affine_q20)
        fa = int(lut[pseudo_index(int(fu[0]), int(fv[0]))])
        ra = int(lut[pseudo_index(int(ru[0]), int(rv[0]))])
        sweep_deltas.append(turn16_error_deg(fa, float(ra)))

    sweep_delta_deg = max(sweep_deltas)
    if fit_max_error_deg > MAX_FIT_ERROR_DEG:
        raise CalibrationError(f"fit residual too high: {fit_max_error_deg:.2f} deg")
    if sweep_delta_deg > MAX_SWEEP_DELTA_DEG:
        raise CalibrationError(f"sweep disagreement too high: {sweep_delta_deg:.2f} deg")

    blob = build_blob(affine_q20, angle_lut, fit_max_error_deg, sweep_delta_deg)
    _, blob_crc32 = validate_blob(blob)
    return SolvedCalibration(
        affine_q20=affine_q20,
        angle_lut=angle_lut,
        fit_max_error_deg=fit_max_error_deg,
        sweep_delta_deg=sweep_delta_deg,
        blob=blob,
        blob_crc32=blob_crc32,
    )


def solve_legacy_csv(path: Path) -> SolvedCalibration:
    step_groups: dict[int, list[tuple[int, int]]] = {idx: [] for idx in range(CAL_POINTS_PER_SWEEP)}
    first_step: int | None = None

    with path.open(newline="") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            step_text = row.get("step_count")
            x_text = row.get("magx")
            y_text = row.get("magy")
            if not step_text or not x_text or not y_text:
                continue
            step_count = int(float(step_text))
            if first_step is None:
                first_step = step_count
            index = (step_count - first_step) % CAL_POINTS_PER_SWEEP
            step_groups[index].append((int(float(x_text)), int(float(y_text))))

    if first_step is None or not all(step_groups.values()):
        raise CalibrationError(f"incomplete legacy csv: {path}")

    xs = np.array([int(round(np.mean([pair[0] for pair in step_groups[idx]]))) for idx in range(CAL_POINTS_PER_SWEEP)])
    ys = np.array([int(round(np.mean([pair[1] for pair in step_groups[idx]]))) for idx in range(CAL_POINTS_PER_SWEEP)])
    affine_q20, angle_lut, fit_max_error_deg = _solve_anchor_set(xs, ys)
    blob = build_blob(affine_q20, angle_lut, fit_max_error_deg, 0.0)
    _, blob_crc32 = validate_blob(blob)
    return SolvedCalibration(
        affine_q20=affine_q20,
        angle_lut=angle_lut,
        fit_max_error_deg=fit_max_error_deg,
        sweep_delta_deg=0.0,
        blob=blob,
        blob_crc32=blob_crc32,
    )


class EspBridge:
    def __init__(self, port: str, baudrate: int = 115200) -> None:
        self.serial = serial.Serial(port=port, baudrate=baudrate, timeout=0.05, write_timeout=1.0)

    def close(self) -> None:
        if self.serial.is_open:
            self.serial.close()

    def __enter__(self) -> "EspBridge":
        return self

    def __exit__(self, *_: object) -> None:
        self.close()

    def _read_line_until(self, predicate: Callable[[str], bool], timeout: float) -> str:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            line = self.serial.readline()
            if not line:
                continue
            text = line.decode("utf-8", errors="replace").strip()
            if predicate(text):
                return text
        raise TimeoutError("bridge shell did not respond")

    def sync_shell(self, timeout: float = 6.0) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.serial.reset_input_buffer()
            self.serial.write(b"\n")
            self.serial.write(b"ping\n")
            self.serial.flush()
            try:
                self._read_line_until(lambda text: text == "pong", timeout=0.8)
                return
            except TimeoutError:
                time.sleep(0.2)
        raise TimeoutError("bridge shell did not respond")

    def enter_bridge(self, mode: str = "app") -> None:
        self.sync_shell()
        self.serial.reset_input_buffer()
        self.serial.reset_output_buffer()
        self.serial.write(f"bridge {mode}\n".encode("utf-8"))
        self.serial.flush()
        self._read_line_until(lambda text: text == f"# bridge={mode}", timeout=2.0)

    def exit_bridge(self) -> None:
        time.sleep(BRIDGE_IDLE_S)
        self.sync_shell(timeout=4.0)


class Stm32Client:
    def __init__(self, port: serial.Serial, address: int = 0) -> None:
        self.port = port
        self.address = address

    def _read_frame(self, expected_cmd: int, timeout: float) -> bytes:
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
                try:
                    address, cmd, payload = decode_frame(bytes(buf))
                except ValueError:
                    buf.clear()
                    expected_len = None
                    continue
                if address != self.address or cmd != expected_cmd:
                    buf.clear()
                    expected_len = None
                    continue
                return payload

        raise TimeoutError(f"timeout waiting for command 0x{expected_cmd:X} response")

    def exchange(self, cmd: int, payload: bytes = b"", timeout: float = 0.5) -> bytes:
        self.port.reset_input_buffer()
        self.port.write(encode_frame(self.address, cmd, payload))
        self.port.flush()
        return self._read_frame(cmd, timeout)

    def exchange_retry(self, cmd: int, payload: bytes = b"", timeout: float = 0.5, attempts: int = 3) -> bytes:
        last_error: Exception | None = None
        for _ in range(attempts):
            try:
                return self.exchange(cmd, payload, timeout=timeout)
            except TimeoutError as exc:
                last_error = exc
                time.sleep(0.02)
        if last_error is not None:
            raise last_error
        raise TimeoutError(f"timeout waiting for command 0x{cmd:X} response")

    def get_status(self) -> Status:
        payload = self.exchange_retry(CMD_GET_STATUS, timeout=0.5)
        return Status(*struct.unpack("<BBBBhhhHiih", payload))

    def set_duty(self, duty: int) -> Status:
        payload = self.exchange_retry(CMD_SET_DUTY, struct.pack("<h", duty), timeout=0.5)
        return Status(*struct.unpack("<BBBBhhhHiih", payload))

    def set_advance_deg(self, advance_deg: int) -> Status:
        payload = self.exchange_retry(CMD_SET_ADVANCE, struct.pack("<h", advance_deg), timeout=0.5)
        return Status(*struct.unpack("<BBBBhhhHiih", payload))

    def set_position(self, position_crad: int) -> Status:
        payload = self.exchange_retry(CMD_SET_POSITION, struct.pack("<i", position_crad), timeout=0.5)
        return Status(*struct.unpack("<BBBBhhhHiih", payload))

    def cal_start(self, sweep_dir: int) -> tuple[int, int]:
        payload = self.exchange_retry(CMD_CAL_START, struct.pack("<B", sweep_dir), timeout=0.5)
        return struct.unpack("<BB", payload)

    def cal_status(self) -> CalStatus:
        payload = self.exchange_retry(CMD_CAL_STATUS, timeout=0.5)
        return CalStatus(*struct.unpack("<BBBB", payload))

    def cal_read_point(self, index: int, timeout: float = 2.0) -> CapturePoint:
        payload = self.exchange_retry(CMD_CAL_READ_POINT, struct.pack("<B", index), timeout=timeout)
        result, point_index, step_index, sweep_dir, x, y, z, xy_radius, sample_spread = struct.unpack(
            "<BBBBhhhHH", payload
        )
        if result != RESULT_OK:
            raise CalibrationError(f"read point {index} failed with result {result}")
        return CapturePoint(point_index, step_index, sweep_dir, x, y, z, xy_radius, sample_spread)

    def cal_write_blob(self, offset: int, chunk: bytes) -> tuple[int, int]:
        payload = self.exchange_retry(CMD_CAL_WRITE_BLOB, struct.pack("<H", offset) + chunk, timeout=0.5)
        return struct.unpack("<BH", payload)

    def cal_commit(self) -> tuple[int, int]:
        payload = self.exchange_retry(CMD_CAL_COMMIT, timeout=0.5)
        return struct.unpack("<BB", payload)

    def cal_clear(self) -> tuple[int, int]:
        payload = self.exchange_retry(CMD_CAL_CLEAR, timeout=0.5)
        return struct.unpack("<BB", payload)

    def cal_info(self) -> CalibrationInfo:
        payload = self.exchange_retry(CMD_CAL_INFO, timeout=0.5)
        return CalibrationInfo(*struct.unpack("<BBHI", payload))

    def wait_until_ready(self, timeout: float = 5.0) -> Status:
        deadline = time.monotonic() + timeout
        last_error: Exception | None = None
        while time.monotonic() < deadline:
            try:
                return self.get_status()
            except Exception as exc:  # noqa: BLE001
                last_error = exc
                time.sleep(0.05)
        raise TimeoutError("stm32 did not come back after commit") from last_error


def print_status(status: Status) -> None:
    print(
        "mode=%d flags=0x%02X faults=0x%02X raw=(%d,%d,%d) angle_turn16=%d pos_crad=%d vel_crads=%d duty=%d"
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
        )
    )


def read_capture_points(
    client: Stm32Client,
    base_index: int,
    point_cb: Callable[[CapturePoint], None] | None = None,
) -> list[CapturePoint]:
    points: list[CapturePoint] = []
    for local_index in range(CAL_POINTS_PER_SWEEP):
        point = None
        last_error: TimeoutError | None = None
        for _ in range(20):
            try:
                point = client.cal_read_point(local_index, timeout=2.0)
                break
            except TimeoutError as exc:
                last_error = exc
                time.sleep(0.05)
        if point is None:
            raise last_error if last_error is not None else CalibrationError("point read failed")
        point = dataclasses.replace(point, index=base_index + local_index)
        points.append(point)
        if point_cb is not None:
            point_cb(point)
        time.sleep(0.05)
    return points


def capture_sweep(
    client: Stm32Client,
    sweep_dir: int,
    base_index: int,
    status_cb: Callable[[CalStatus], None] | None = None,
    sample_cb: Callable[[int, int, int, Status], None] | None = None,
    point_cb: Callable[[CapturePoint], None] | None = None,
) -> list[CapturePoint]:
    result, total_points = client.cal_start(sweep_dir)
    if result != RESULT_OK:
        raise CalibrationError(f"CAL_START failed with result {result}")
    if total_points != CAL_POINTS_PER_SWEEP:
        raise CalibrationError(f"unexpected point count {total_points}")

    last_sample_index = -1
    while True:
        status = None
        last_error: TimeoutError | None = None
        for _ in range(5):
            try:
                status = client.cal_status()
                break
            except TimeoutError as exc:
                last_error = exc
                time.sleep(0.05)
        if status is None:
            raise last_error if last_error is not None else CalibrationError("status read failed")
        if status.result != RESULT_OK:
            raise CalibrationError(f"CAL_STATUS failed with result {status.result}")
        if status_cb is not None:
            status_cb(
                CalStatus(
                    result=status.result,
                    active=status.active,
                    next_index=base_index + status.next_index,
                    total_points=CAL_CAPTURE_TOTAL_POINTS,
                )
            )
        if (
            status.active
            and status.next_index > 0
            and sample_cb is not None
            and status.next_index != last_sample_index
        ):
            try:
                live = client.get_status()
                point_index = base_index + int(status.next_index) - 1
                sample_cb(point_index, cal_capture_step_index(point_index), sweep_dir, live)
                last_sample_index = status.next_index
            except TimeoutError:
                pass
        if not status.active and status.next_index >= CAL_POINTS_PER_SWEEP:
            break
        time.sleep(0.05)
    return read_capture_points(client, base_index=base_index, point_cb=point_cb)


def capture_points(
    client: Stm32Client,
    status_cb: Callable[[CalStatus], None] | None = None,
    sample_cb: Callable[[int, int, int, Status], None] | None = None,
    point_cb: Callable[[CapturePoint], None] | None = None,
) -> list[CapturePoint]:
    points: list[CapturePoint] = []
    raw_index = 0
    for sweep_dir in [0] * CAL_ROTATIONS_PER_DIR + [1] * CAL_ROTATIONS_PER_DIR:
        points.extend(
            capture_sweep(
                client,
                sweep_dir=sweep_dir,
                base_index=raw_index,
                status_cb=status_cb,
                sample_cb=sample_cb,
                point_cb=point_cb,
            )
        )
        raw_index += CAL_POINTS_PER_SWEEP
    return points


def reduce_capture_points(points: Sequence[CapturePoint]) -> list[CapturePoint]:
    ordered = sorted(points, key=lambda point: point.index)
    if len(ordered) != CAL_CAPTURE_TOTAL_POINTS:
        raise CalibrationError(f"capture did not return all {CAL_CAPTURE_TOTAL_POINTS} points")
    if [point.index for point in ordered] != list(range(CAL_CAPTURE_TOTAL_POINTS)):
        raise CalibrationError("capture indices are incomplete")

    data = {}
    for sweep_dir in (0, 1):
        xs = np.zeros((CAL_ROTATIONS_PER_DIR, CAL_POINTS_PER_SWEEP), dtype=np.int64)
        ys = np.zeros((CAL_ROTATIONS_PER_DIR, CAL_POINTS_PER_SWEEP), dtype=np.int64)
        zs = np.zeros((CAL_ROTATIONS_PER_DIR, CAL_POINTS_PER_SWEEP), dtype=np.int64)
        radii = np.zeros((CAL_ROTATIONS_PER_DIR, CAL_POINTS_PER_SWEEP), dtype=np.int64)
        spreads = np.zeros((CAL_ROTATIONS_PER_DIR, CAL_POINTS_PER_SWEEP), dtype=np.int64)
        seen = np.zeros((CAL_ROTATIONS_PER_DIR, CAL_POINTS_PER_SWEEP), dtype=bool)
        data[sweep_dir] = (xs, ys, zs, radii, spreads, seen)

    for point in ordered:
        if not 0 <= point.step_index < CAL_POINTS_PER_SWEEP:
            raise CalibrationError("step index out of range")
        sweep_dir = cal_capture_dir(point.index)
        if point.sweep_dir != sweep_dir:
            raise CalibrationError("capture direction mismatch")
        rotation = cal_capture_rotation(point.index)
        xs, ys, zs, radii, spreads, seen = data[sweep_dir]
        xs[rotation, point.step_index] = point.x
        ys[rotation, point.step_index] = point.y
        zs[rotation, point.step_index] = point.z
        radii[rotation, point.step_index] = point.xy_radius
        spreads[rotation, point.step_index] = point.sample_spread
        seen[rotation, point.step_index] = True

    reduced: list[CapturePoint] = []
    for sweep_dir in (0, 1):
        xs, ys, zs, radii, spreads, seen = data[sweep_dir]
        if not np.all(seen):
            raise CalibrationError("missing sweep samples")

        ref_x = xs[0].astype(np.float64)
        ref_y = ys[0].astype(np.float64)
        for rotation in range(1, CAL_ROTATIONS_PER_DIR):
            best_shift = 0
            best_score: float | None = None
            for shift in range(CAL_POINTS_PER_SWEEP):
                sx = np.roll(xs[rotation], shift)
                sy = np.roll(ys[rotation], shift)
                score = float(np.sum((sx - ref_x) ** 2 + (sy - ref_y) ** 2))
                if best_score is None or score < best_score:
                    best_score = score
                    best_shift = shift
            xs[rotation] = np.roll(xs[rotation], best_shift)
            ys[rotation] = np.roll(ys[rotation], best_shift)
            zs[rotation] = np.roll(zs[rotation], best_shift)
            radii[rotation] = np.roll(radii[rotation], best_shift)
            spreads[rotation] = np.roll(spreads[rotation], best_shift)
            ref_x = np.mean(xs[: rotation + 1], axis=0)
            ref_y = np.mean(ys[: rotation + 1], axis=0)

        avg_x = np.rint(np.mean(xs, axis=0)).astype(np.int64)
        avg_y = np.rint(np.mean(ys, axis=0)).astype(np.int64)
        avg_z = np.rint(np.mean(zs, axis=0)).astype(np.int64)
        avg_radius = np.rint(np.mean(radii, axis=0)).astype(np.int64)
        avg_spread = np.rint(np.mean(spreads, axis=0)).astype(np.int64)

        if sweep_dir == 0:
            for step in range(CAL_POINTS_PER_SWEEP):
                reduced.append(
                    CapturePoint(
                        index=step,
                        step_index=step,
                        sweep_dir=0,
                        x=int(avg_x[step]),
                        y=int(avg_y[step]),
                        z=int(avg_z[step]),
                        xy_radius=int(avg_radius[step]),
                        sample_spread=int(avg_spread[step]),
                    )
                )
        else:
            for local_index in range(CAL_POINTS_PER_SWEEP):
                step = CAL_POINTS_PER_SWEEP - 1 - local_index
                reduced.append(
                    CapturePoint(
                        index=CAL_POINTS_PER_SWEEP + local_index,
                        step_index=step,
                        sweep_dir=1,
                        x=int(avg_x[step]),
                        y=int(avg_y[step]),
                        z=int(avg_z[step]),
                        xy_radius=int(avg_radius[step]),
                        sample_spread=int(avg_spread[step]),
                    )
                )
    return reduced


def upload_calibration(
    client: Stm32Client,
    solved: SolvedCalibration,
    chunk_size: int,
    upload_cb: Callable[[int, int], None] | None = None,
) -> None:
    next_offset = 0
    total = len(solved.blob)
    if upload_cb is not None:
        upload_cb(0, total)
    for offset, chunk in iter_blob_chunks(solved.blob, chunk_size):
        result, next_offset = client.cal_write_blob(offset, chunk)
        if result != RESULT_OK:
            raise CalibrationError(f"CAL_WRITE_BLOB failed at offset {offset} with result {result}")
        if upload_cb is not None:
            upload_cb(next_offset, total)
    if next_offset != total:
        raise CalibrationError(f"blob upload stopped at {next_offset}")


def verify_calibration(client: Stm32Client, solved: SolvedCalibration | None = None) -> CalibrationInfo:
    status = client.wait_until_ready()
    info = client.cal_info()
    if not status.calibrated:
        raise CalibrationError("stm32 came back uncalibrated")
    if not info.valid:
        raise CalibrationError("stm32 reports invalid calibration blob")
    if solved is not None and info.blob_crc32 != solved.blob_crc32:
        raise CalibrationError("blob crc mismatch after commit")
    return info


def run_calibration(
    client: Stm32Client,
    chunk_size: int,
    status_cb: Callable[[CalStatus], None] | None = None,
    sample_cb: Callable[[int, int, int, Status], None] | None = None,
    point_cb: Callable[[CapturePoint], None] | None = None,
    upload_cb: Callable[[int, int], None] | None = None,
) -> SolvedCalibration:
    raw_points = capture_points(client, status_cb=status_cb, sample_cb=sample_cb)
    points = reduce_capture_points(raw_points)
    if point_cb is not None:
        for point in points:
            point_cb(point)

    solved = solve_capture(points)
    upload_calibration(client, solved, chunk_size, upload_cb=upload_cb)

    result, valid = client.cal_commit()
    if result != RESULT_OK or valid != 1:
        raise CalibrationError(f"CAL_COMMIT failed with result {result}")

    verify_calibration(client, solved)
    return solved


def command_calibrate(args: argparse.Namespace) -> int:
    with EspBridge(args.port) as bridge:
        bridge.enter_bridge()
        try:
            client = Stm32Client(bridge.serial, address=args.address)
            solved = run_calibration(
                client,
                chunk_size=args.chunk_size,
                status_cb=lambda status: print(f"capture {status.next_index}/{status.total_points}"),
                upload_cb=lambda done, total: print(f"upload {done}/{total}"),
            )
            print(
                "solve fit_max_error=%.2fdeg sweep_delta=%.2fdeg crc32=0x%08X"
                % (solved.fit_max_error_deg, solved.sweep_delta_deg, solved.blob_crc32)
            )
            print(
                "calibrated crc32=0x%08X fit=%.2fdeg sweep=%.2fdeg"
                % (solved.blob_crc32, solved.fit_max_error_deg, solved.sweep_delta_deg)
            )
        finally:
            bridge.exit_bridge()
    return 0


def command_info(args: argparse.Namespace) -> int:
    with EspBridge(args.port) as bridge:
        bridge.enter_bridge()
        try:
            client = Stm32Client(bridge.serial, address=args.address)
            status = client.get_status()
            info = client.cal_info()
        finally:
            bridge.exit_bridge()

    print_status(status)
    print("blob_valid=%d blob_size=%d blob_crc32=0x%08X" % (info.valid, info.blob_size, info.blob_crc32))
    return 0


def command_clear(args: argparse.Namespace) -> int:
    with EspBridge(args.port) as bridge:
        bridge.enter_bridge()
        try:
            client = Stm32Client(bridge.serial, address=args.address)
            result, valid = client.cal_clear()
            status = client.get_status()
        finally:
            bridge.exit_bridge()

    if result != RESULT_OK or valid != 0:
        raise CalibrationError(f"CAL_CLEAR failed with result {result}")
    print_status(status)
    print("cleared")
    return 0


def command_verify(args: argparse.Namespace) -> int:
    with EspBridge(args.port) as bridge:
        bridge.enter_bridge()
        try:
            client = Stm32Client(bridge.serial, address=args.address)
            status = client.get_status()
            info = client.cal_info()
        finally:
            bridge.exit_bridge()

    if not status.calibrated or not info.valid or status.result != RESULT_OK or info.result != RESULT_OK:
        raise CalibrationError("verification failed")
    if status.faults & (FAULT_SENSOR | FAULT_FLASH):
        raise CalibrationError(f"verification saw faults 0x{status.faults:02X}")

    print_status(status)
    print("verified crc32=0x%08X size=%d" % (info.blob_crc32, info.blob_size))
    return 0


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="pennyesc calibration CLI")
    parser.add_argument("--port", required=True, help="ESP32 USB CDC port")
    parser.add_argument("--address", type=int, default=0, help="ESC address nibble")

    subparsers = parser.add_subparsers(dest="command", required=True)

    calibrate = subparsers.add_parser("calibrate", help="capture, solve, upload, verify")
    calibrate.add_argument("--chunk-size", type=int, default=DEFAULT_CHUNK_SIZE)
    calibrate.set_defaults(func=command_calibrate)

    info = subparsers.add_parser("info", help="show status and stored blob info")
    info.set_defaults(func=command_info)

    clear = subparsers.add_parser("clear", help="erase stored calibration")
    clear.set_defaults(func=command_clear)

    verify = subparsers.add_parser("verify", help="confirm stored calibration is active")
    verify.set_defaults(func=command_verify)

    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    try:
        return args.func(args)
    except (CalibrationError, TimeoutError, serial.SerialException) as exc:
        print(f"error: {exc}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
