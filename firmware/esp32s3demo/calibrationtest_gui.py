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
from typing import Any, Sequence

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
    CAL_CAPTURE_TOTAL_POINTS,
    CAL_POINTS_PER_SWEEP,
    CAL_TOTAL_POINTS,
    DEFAULT_CHUNK_SIZE,
    CalibrationError,
    EspBridge,
    Stm32Client,
    apply_affine_q20,
    capture_points,
    commit_calibration_blob,
    measure_forward_angle_sign,
    pseudo_index,
    reduce_capture_points,
    set_forward_angle_sign,
    solve_capture,
)
from pnyproto import (  # noqa: E402
    RESULT_BAD_ARG,
    RESULT_BAD_STATE,
    RESULT_BUSY,
    RESULT_CRC,
    RESULT_FLASH,
    RESULT_NOT_CALIBRATED,
    RESULT_OK,
    RESULT_RANGE,
)

Signal = getattr(QtCore, "Signal", QtCore.pyqtSignal)

RESULTS_DIR = Path(__file__).resolve().parent / "calibrationtest_sessions"
RAW_PLOT_WINDOW_S = 10.0
RAW_POLL_MS = 100
RAW_STATUS_TIMEOUT_S = 0.12
RAW_STATUS_ATTEMPTS = 2
TURN32_PER_REV = 65536.0
TURN32_TO_RAD = (2.0 * math.pi) / TURN32_PER_REV
RESULT_NAMES = {
    RESULT_OK: "ok",
    RESULT_BAD_STATE: "bad_state",
    RESULT_BAD_ARG: "bad_arg",
    RESULT_NOT_CALIBRATED: "not_calibrated",
    RESULT_BUSY: "busy",
    RESULT_RANGE: "range",
    RESULT_FLASH: "flash",
    RESULT_CRC: "crc",
}


def result_text(result: int) -> str:
    return RESULT_NAMES.get(int(result), f"unknown_{int(result)}")


def format_status(status) -> str:
    return (
        "result=%s(%d) mode=%d flags=0x%02X faults=0x%02X raw=(%d,%d,%d) angle_turn16=%d pos_turn32=%d vel_turn32_s=%d duty=%d mct_faults=%d"
        % (
            result_text(status.result),
            status.result,
            status.mode,
            status.flags,
            status.faults,
            status.x,
            status.y,
            status.z,
            status.angle_turn16,
            status.position_turn32,
            status.velocity_turn32_per_s,
            status.duty,
            status.mct_fault_count,
        )
    )


def calibration_progress_text(next_index: int, total_points: int) -> str:
    if total_points <= 0:
        return "capture"
    capped = max(0, min(next_index, total_points))
    per_dir = CAL_POINTS_PER_SWEEP * 3
    if capped < per_dir:
        direction = "forward"
        local = capped
    else:
        direction = "reverse"
        local = capped - per_dir
    rotation = min(3, (local // CAL_POINTS_PER_SWEEP) + 1)
    step = min(CAL_POINTS_PER_SWEEP, (local % CAL_POINTS_PER_SWEEP) + 1)
    return f"capture {direction} rot {rotation}/3 step {step}/{CAL_POINTS_PER_SWEEP} ({capped}/{total_points})"


def probe_error_text(exc: Exception) -> str:
    if isinstance(exc, serial.SerialException):
        return "motor: no serial port"
    text = str(exc).lower()
    if "no serial port" in text:
        return "motor: no serial port"
    if "bridge" in text:
        return "bridge: not recognized"
    if isinstance(exc, TimeoutError):
        return "motor: no ESC"
    return "motor: probe failed"


def serial_port_allowed(value: str, text: str = "") -> bool:
    combined = f"{value} {text}".lower()
    blocked = ("bluetooth", "debug", "incoming-port", "debug-console")
    return bool(value) and not any(token in combined for token in blocked)


def find_serial_port(pattern: str | None) -> str:
    if pattern:
        matches = [value for value in sorted(glob(pattern)) if serial_port_allowed(value)]
        return matches[0] if matches else (pattern if serial_port_allowed(pattern) else "")

    if list_ports is not None:
        ports = []
        for port_info in list_ports.comports():
            text = " ".join(
                [
                    port_info.device or "",
                    port_info.description or "",
                    port_info.manufacturer or "",
                    port_info.hwid or "",
                ]
            )
            if serial_port_allowed(port_info.device or "", text):
                ports.append(port_info)

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
        matches = [value for value in sorted(glob(port_pattern)) if serial_port_allowed(value)]
        if matches:
            return matches[0]

    return ""


def list_serial_ports(preferred: str | None = None) -> list[str]:
    ports: list[str] = []
    seen: set[str] = set()

    def add_port(value: str) -> None:
        if serial_port_allowed(value) and value not in seen:
            seen.add(value)
            ports.append(value)

    if preferred:
        matches = sorted(glob(preferred)) if any(char in preferred for char in "*?[") else []
        if matches:
            for value in matches:
                add_port(value)
        elif Path(preferred).exists() and serial_port_allowed(preferred):
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
            text = " ".join(
                [
                    item.device or "",
                    item.description or "",
                    item.manufacturer or "",
                    item.hwid or "",
                ]
            )
            if serial_port_allowed(item.device or "", text):
                add_port(item.device or "")

    for port_pattern in ("/dev/cu.usbmodem*", "/dev/ttyACM*", "/dev/ttyUSB*"):
        for value in sorted(glob(port_pattern)):
            add_port(value)

    return ports


def port_available(port: str) -> bool:
    if not serial_port_allowed(port):
        return False
    if any(char in port for char in "*?["):
        return bool(glob(port))
    return Path(port).exists()


def clamp(value: int, low: int, high: int) -> int:
    return max(low, min(high, value))


def signed_turn16_error_deg(measured: int, target: float) -> float:
    delta = ((measured - target + 32768.0) % 65536.0) - 32768.0
    return float(delta) * 360.0 / 65536.0


def angle_from_xy(x: int, y: int, affine_q20: Sequence[int], angle_lut: Sequence[int]) -> int:
    xs = np.array([x], dtype=np.int64)
    ys = np.array([y], dtype=np.int64)
    u, v = apply_affine_q20(xs, ys, affine_q20)
    return int(angle_lut[pseudo_index(int(u[0]), int(v[0]))])


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
class StaticResult:
    raw_points: list[dict[str, Any]]
    reduced_points: list[dict[str, Any]]
    affine_q20: list[int] | None
    angle_lut: list[int] | None
    fit_max_error_deg: float | None
    sweep_delta_deg: float | None
    forward_angle_sign: int | None
    forward_sign_tests: list[dict[str, Any]] | None
    blob_crc32: int | None
    blob_size: int | None


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


@dataclasses.dataclass
class ScanResult:
    port: str
    found: list[int]
    status_text: str
    error: str | None = None


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
        forward_angle_sign=int(solved.forward_angle_sign),
        forward_sign_tests=None,
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
        forward_angle_sign=None,
        forward_sign_tests=None,
        blob_crc32=None,
        blob_size=None,
    )


class ScanWorker(QtCore.QThread):
    done = Signal(object)

    def __init__(self, port: str) -> None:
        super().__init__()
        self.port = port

    def run(self) -> None:
        if not self.port:
            self.done.emit(ScanResult(port="", found=[], status_text="motor: no serial port"))
            return

        found: list[int] = []
        try:
            with EspBridge(self.port) as bridge:
                bridge.enter_bridge("app")
                try:
                    client = Stm32Client(bridge.serial, address=0)
                    for address in range(16):
                        client.address = address
                        try:
                            client.get_status(timeout=RAW_STATUS_TIMEOUT_S, attempts=1)
                        except TimeoutError:
                            continue
                        found.append(address)
                finally:
                    bridge.exit_bridge()
        except Exception as exc:  # noqa: BLE001
            self.done.emit(ScanResult(port=self.port, found=[], status_text=probe_error_text(exc), error=str(exc)))
            return

        status_text = "motor: unknown" if found else "motor: no ESC"
        self.done.emit(ScanResult(port=self.port, found=found, status_text=status_text))


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
    ) -> None:
        super().__init__()
        self.port = port
        self.address = address
        self.chunk_size = chunk_size

    def run(self) -> None:
        stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        export_dir = RESULTS_DIR / stamp
        session = SessionData(
            mode="static",
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
                try:
                    client = Stm32Client(bridge.serial, address=self.address)

                    self.progress.emit(1)
                    raw_points = capture_points(
                        client,
                        status_cb=self.on_capture_status,
                        sample_cb=self.on_capture_sample,
                    )
                    self.log.emit("average capture points")
                    reduced_points = reduce_capture_points(raw_points)
                    session.static = build_partial_static_result(raw_points, reduced_points)
                    self.log.emit("solve static map")
                    solved = solve_capture(reduced_points)
                    session.static = build_static_result(raw_points, reduced_points, solved)
                    self.log.emit(
                        "fit %.2f deg sweep %.2f deg crc32 0x%08X"
                        % (solved.fit_max_error_deg, solved.sweep_delta_deg, solved.blob_crc32)
                    )
                    self.log.emit("upload temporary calibration")
                    info = commit_calibration_blob(client, solved, self.chunk_size, upload_cb=self.on_upload)
                    self.log.emit("measure forward advance sign")
                    forward_sign = measure_forward_angle_sign(client)
                    for test in forward_sign.tests:
                        self.log.emit(
                            "direction sign=%+d advance=%+d delta_turn32=%d"
                            % (test.candidate_sign, test.advance_deg, test.delta_turn32)
                        )
                    solved = set_forward_angle_sign(solved, forward_sign.forward_angle_sign)
                    session.static = build_static_result(raw_points, reduced_points, solved)
                    if session.static is not None:
                        session.static.forward_sign_tests = [
                            dataclasses.asdict(test) for test in forward_sign.tests
                        ]
                    self.log.emit("direction sign=%+d" % solved.forward_angle_sign)
                    self.log.emit("upload final calibration")
                    info = commit_calibration_blob(client, solved, self.chunk_size, upload_cb=self.on_upload)
                    if session.static is not None:
                        session.static.blob_size = int(info.blob_size)
                        session.static.blob_crc32 = int(info.blob_crc32)
                    session.status = "calibrated"
                    self.progress.emit(80)
                    self.log.emit(
                        "calibrated crc32=0x%08X size=%d forward_sign=%+d"
                        % (info.blob_crc32, info.blob_size, solved.forward_angle_sign)
                    )
                finally:
                    bridge.exit_bridge()

            session.ended_at = datetime.now().isoformat(timespec="seconds")
            self.session_ready.emit(session)
            self.done.emit(session.status)
        except Exception as exc:  # noqa: BLE001
            session.error = str(exc)
            if session.status == "not run":
                session.status = "failed"
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
    def __init__(self, port: str, address: int, chunk_size: int, auto: bool) -> None:
        if not 0 <= address <= 0xF:
            raise SystemExit("ESC address must be 0-15")
        self.port = port
        self.address = address
        self.chunk_size = chunk_size
        self.auto = auto
        self.worker: TestWorker | None = None
        self.scan_worker: ScanWorker | None = None
        self.bridge: EspBridge | None = None
        self.client: Stm32Client | None = None
        self.client_busy = False
        self.session: SessionData | None = None
        self.static_live_forward_x: list[int] = []
        self.static_live_forward_y: list[int] = []
        self.static_live_reverse_x: list[int] = []
        self.static_live_reverse_y: list[int] = []
        self.raw_times: deque[float] = deque()
        self.raw_x_vals: deque[int] = deque()
        self.raw_y_vals: deque[int] = deque()
        self.raw_z_vals: deque[int] = deque()
        self.position_vals: deque[float] = deque()
        self.velocity_vals: deque[float] = deque()
        self.raw_start_time: float | None = None
        self.raw_paused = False
        self.raw_last_status = "waiting for data"
        self.raw_last_values = (0, 0, 0, 0, 0)
        self.device_calibration_text = "motor: unknown"

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
        self.scan_button = QtWidgets.QPushButton("Scan")
        self.scan_button.clicked.connect(self.scan_addresses)
        top_row.addWidget(self.scan_button)

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

        self.start_button = QtWidgets.QPushButton("Run Calibration")
        self.start_button.clicked.connect(lambda: self.start_worker())
        controls.addWidget(self.start_button)

        self.clear_button = QtWidgets.QPushButton("Clear Plots")
        self.clear_button.clicked.connect(self.clear_session)
        controls.addWidget(self.clear_button)

        self.device_calibration_label = QtWidgets.QLabel(self.device_calibration_text)
        controls.addWidget(self.device_calibration_label)

        self.check_motor_button = QtWidgets.QPushButton("Check Motor")
        self.check_motor_button.clicked.connect(self.check_motor_calibration)
        controls.addWidget(self.check_motor_button)
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
        self.build_session_tab()
        self.tabs.setCurrentWidget(self.raw_tab)

        terminal = QtWidgets.QHBoxLayout()
        self.command_line = QtWidgets.QLineEdit()
        self.command_line.setPlaceholderText("status | duty 120 | duty -120 | position 6.28 | stop | advance 90")
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
            QtCore.QTimer.singleShot(0, lambda: self.start_worker())
        else:
            QtCore.QTimer.singleShot(0, self.scan_addresses)

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

        plots = QtWidgets.QGridLayout()
        plots.setColumnStretch(0, 1)
        plots.setColumnStretch(1, 1)
        plots.setRowStretch(0, 1)
        plots.setRowStretch(1, 1)
        layout.addLayout(plots, stretch=1)

        self.raw_mag_plot = pg.PlotWidget(title="Magnetic Field")
        self.raw_mag_plot.showGrid(x=True, y=True, alpha=0.25)
        self.raw_mag_plot.setLabel("bottom", "Time (s)")
        self.raw_mag_plot.setLabel("left", "Raw Field")
        self.raw_mag_plot.addLegend()
        plots.addWidget(self.raw_mag_plot, 0, 0, 2, 1)
        self.plot_widgets.append(("raw_magnetic", self.raw_mag_plot))

        self.raw_x_curve = self.raw_mag_plot.plot(name="X", pen=pg.mkPen("#e74c3c", width=2))
        self.raw_y_curve = self.raw_mag_plot.plot(name="Y", pen=pg.mkPen("#2ecc71", width=2))
        self.raw_z_curve = self.raw_mag_plot.plot(name="Z", pen=pg.mkPen("#3498db", width=2))

        self.position_plot = pg.PlotWidget(title="Position")
        self.position_plot.showGrid(x=True, y=True, alpha=0.25)
        self.position_plot.setLabel("bottom", "Time (s)")
        self.position_plot.setLabel("left", "Position (rad)")
        plots.addWidget(self.position_plot, 0, 1)
        self.plot_widgets.append(("position", self.position_plot))
        self.position_curve = self.position_plot.plot(pen=pg.mkPen("#8e44ad", width=2))

        self.velocity_plot = pg.PlotWidget(title="Velocity")
        self.velocity_plot.showGrid(x=True, y=True, alpha=0.25)
        self.velocity_plot.setLabel("bottom", "Time (s)")
        self.velocity_plot.setLabel("left", "Velocity (rpm)")
        plots.addWidget(self.velocity_plot, 1, 1)
        self.plot_widgets.append(("velocity", self.velocity_plot))
        self.velocity_curve = self.velocity_plot.plot(pen=pg.mkPen("#f39c12", width=2))

        self.raw_tab = tab
        self.tabs.addTab(tab, "Telemetry")

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

        self.step_mag_plot = self.add_plot(geometry_grid, 0, 1, "Reduced Field vs Step", "static_step_mag")
        self.step_mag_plot.setLabel("bottom", "Step")
        self.step_mag_plot.setLabel("left", "Raw Field")
        self.step_mag_plot.addLegend()
        subtabs.addTab(geometry_tab, "Geometry")

        residual_tab = QtWidgets.QWidget()
        residual_grid = QtWidgets.QGridLayout(residual_tab)
        self.residual_plot = self.add_plot(residual_grid, 0, 0, "Residual vs Step", "static_residual")
        self.residual_plot.setLabel("bottom", "Step")
        self.residual_plot.setLabel("left", "Error (deg)")
        self.residual_plot.addLegend()
        subtabs.addTab(residual_tab, "Residuals")

        self.tabs.addTab(tab, "Calibration")

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
        self.device_calibration_text = "motor: unknown"
        self.device_calibration_label.setText(self.device_calibration_text)
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
        self.position_curve = self.position_plot.plot(pen=pg.mkPen("#8e44ad", width=2))
        self.velocity_curve = self.velocity_plot.plot(pen=pg.mkPen("#f39c12", width=2))
        self.clear_raw_data()
        self.refresh_status()

    def append_log(self, text: str) -> None:
        self.log_box.appendPlainText(text)
        if self.auto:
            print(text, flush=True)

    def append_status(self, status) -> None:
        self.append_log(format_status(status))
        if status.result != RESULT_OK:
            self.append_log(f"error: command returned {result_text(status.result)}")

    def selected_port(self) -> str:
        value = self.port_box.currentData()
        if isinstance(value, str) and value:
            return value
        text = self.port_box.currentText().strip()
        if text:
            return text
        return self.port if port_available(self.port) else ""

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

    def scan_addresses(self) -> None:
        if self.worker is not None and self.worker.isRunning():
            self.append_log("error: stop current test before scanning")
            return
        if self.scan_worker is not None and self.scan_worker.isRunning():
            return
        if self.client_busy:
            self.append_log("error: serial busy")
            return

        self.client_busy = True
        self.scan_button.setEnabled(False)
        self.close_client()
        port = self.selected_port()
        self.device_calibration_text = "scan: running"
        self.refresh_status()
        self.append_log("scan start")
        self.scan_worker = ScanWorker(port)
        self.scan_worker.done.connect(self.on_scan_done)
        self.scan_worker.start()

    def on_scan_done(self, result: ScanResult) -> None:
        self.client_busy = False
        self.scan_button.setEnabled(True)
        self.device_calibration_text = result.status_text
        self.refresh_status()
        if result.error:
            self.append_log(f"scan error: {result.error}")
            self.scan_worker = None
            return
        if not result.found:
            if result.status_text == "motor: no serial port":
                self.append_log("scan found no serial port")
            elif result.status_text == "motor: no ESC":
                self.append_log("scan found no ESCs")
            self.scan_worker = None
            return

        found = result.found
        current = self.address
        self.esc_box.blockSignals(True)
        self.esc_box.clear()
        for address in found:
            self.esc_box.addItem(str(address), address)
        index = self.esc_box.findData(current)
        if index < 0:
            index = 0
        self.esc_box.setCurrentIndex(index)
        self.esc_box.blockSignals(False)
        self.address = int(self.esc_box.currentData())
        self.clear_raw_data()
        self.refresh_status()
        self.append_log("scan found: " + ", ".join(str(address) for address in found))
        self.client_busy = True
        try:
            self.ensure_client()
            self.refresh_device_calibration()
            self.append_log(f"connected esc {self.address}")
        except Exception as exc:  # noqa: BLE001
            self.close_client()
            self.device_calibration_text = probe_error_text(exc)
            self.refresh_status()
            self.append_log(f"connect error: {exc}")
        finally:
            self.client_busy = False
            self.scan_worker = None

    def on_port_changed(self) -> None:
        self.port = self.selected_port()
        self.close_client()
        self.device_calibration_text = "motor: unknown"
        self.clear_raw_data()
        self.refresh_status()

    def on_esc_changed(self) -> None:
        self.address = int(self.esc_box.currentData())
        self.clear_raw_data()
        if self.client is not None:
            self.client.address = self.address
        self.device_calibration_text = "motor: unknown"
        self.refresh_status()

    def refresh_status(self) -> None:
        self.device_calibration_label.setText(self.device_calibration_text)
        self.status.setText(
            "port=%s | esc=%d | %s | stage=%s | capture=%d/%d | bins=%d/%d"
            % (
                self.selected_port(),
                int(self.esc_box.currentData()),
                self.device_calibration_text,
                self.stage_text,
                self.capture_count,
                CAL_CAPTURE_TOTAL_POINTS,
                self.bin_count,
                CAL_TOTAL_POINTS,
            )
        )

    def set_running(self, running: bool) -> None:
        self.start_button.setEnabled(not running)
        self.clear_button.setEnabled(not running)
        self.port_box.setEnabled(not running)
        self.refresh_ports_button.setEnabled(not running)
        self.scan_button.setEnabled(not running)
        self.esc_box.setEnabled(not running)
        self.check_motor_button.setEnabled(not running)
        self.command_line.setEnabled(not running)
        self.command_button.setEnabled(not running)

    def set_device_calibration_from_status(self, status, info=None) -> None:
        if status.calibrated:
            if info is not None and info.valid:
                self.device_calibration_text = f"motor: calibrated crc32=0x{info.blob_crc32:08X}"
            else:
                self.device_calibration_text = "motor: calibrated"
        else:
            self.device_calibration_text = "motor: uncalibrated"
        self.refresh_status()

    def refresh_device_calibration(self) -> None:
        if not self.selected_port():
            self.device_calibration_text = "motor: no serial port"
            self.refresh_status()
            return
        client = self.ensure_client()
        status = client.get_status(timeout=RAW_STATUS_TIMEOUT_S, attempts=RAW_STATUS_ATTEMPTS)
        info = client.cal_info() if status.calibrated else None
        self.set_device_calibration_from_status(status, info)

    def check_motor_calibration(self) -> None:
        if self.worker is not None and self.worker.isRunning():
            self.append_log("error: stop current test before checking motor")
            return
        if self.client_busy:
            self.append_log("error: serial busy")
            return
        self.client_busy = True
        try:
            self.refresh_device_calibration()
            self.append_log(self.device_calibration_text)
        except Exception as exc:  # noqa: BLE001
            self.close_client()
            self.device_calibration_text = probe_error_text(exc)
            self.refresh_status()
            self.append_log(f"connect error: {exc}")
        finally:
            self.client_busy = False

    def toggle_raw_pause(self) -> None:
        self.raw_paused = not self.raw_paused
        self.raw_pause_button.setText("Resume" if self.raw_paused else "Pause")
        self.update_raw_status_label()

    def clear_raw_data(self) -> None:
        self.raw_times.clear()
        self.raw_x_vals.clear()
        self.raw_y_vals.clear()
        self.raw_z_vals.clear()
        self.position_vals.clear()
        self.velocity_vals.clear()
        self.raw_start_time = None
        self.raw_last_values = (0, 0, 0, 0, 0)
        self.raw_last_status = "waiting for data"
        self.raw_x_curve.setData([], [])
        self.raw_y_curve.setData([], [])
        self.raw_z_curve.setData([], [])
        self.position_curve.setData([], [])
        self.velocity_curve.setData([], [])
        self.update_raw_status_label()

    def update_raw_status_label(self) -> None:
        x, y, z, position, velocity = self.raw_last_values
        paused = "paused | " if self.raw_paused else ""
        self.raw_status_label.setText(
            f"port={self.selected_port()} | esc={int(self.esc_box.currentData())} | "
            f"X={x} Y={y} Z={z} | pos={position} vel={velocity} | {paused}{self.raw_last_status}"
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
        self.position_vals.append(float(status.position_turn32) * TURN32_TO_RAD)
        self.velocity_vals.append(float(status.velocity_turn32_per_s) * 60.0 / TURN32_PER_REV)
        self.raw_last_values = (status.x, status.y, status.z, status.position_turn32, status.velocity_turn32_per_s)
        cutoff = sample_time - RAW_PLOT_WINDOW_S
        while self.raw_times and self.raw_times[0] < cutoff:
            self.raw_times.popleft()
            self.raw_x_vals.popleft()
            self.raw_y_vals.popleft()
            self.raw_z_vals.popleft()
            self.position_vals.popleft()
            self.velocity_vals.popleft()

    def update_raw_plot(self) -> None:
        if not self.raw_times:
            return
        times = list(self.raw_times)
        self.raw_x_curve.setData(times, list(self.raw_x_vals))
        self.raw_y_curve.setData(times, list(self.raw_y_vals))
        self.raw_z_curve.setData(times, list(self.raw_z_vals))
        self.position_curve.setData(times, list(self.position_vals))
        self.velocity_curve.setData(times, list(self.velocity_vals))
        latest = times[-1]
        start = max(0.0, latest - RAW_PLOT_WINDOW_S)
        self.raw_mag_plot.setXRange(start, start + RAW_PLOT_WINDOW_S, padding=0.0)
        self.position_plot.setXRange(start, start + RAW_PLOT_WINDOW_S, padding=0.0)
        self.velocity_plot.setXRange(start, start + RAW_PLOT_WINDOW_S, padding=0.0)

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
            self.device_calibration_text = probe_error_text(exc)
            self.raw_last_status = f"error: {exc}"
            self.update_raw_status_label()
        else:
            self.set_device_calibration_from_status(status)
            self.raw_last_status = "mode=%d flags=0x%02X faults=0x%02X mct_faults=%d" % (
                status.mode,
                status.flags,
                status.faults,
                status.mct_fault_count,
            )
            self.raw_last_values = (status.x, status.y, status.z, status.position_turn32, status.velocity_turn32_per_s)
            if not self.raw_paused:
                self.append_raw_sample(status)
                self.update_raw_plot()
            self.update_raw_status_label()
        finally:
            self.client_busy = False

    def ensure_client(self) -> Stm32Client:
        address = int(self.esc_box.currentData())
        port = self.selected_port()
        if not port:
            raise CalibrationError("no serial port selected")
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
            self.append_log("commands: status | duty <value> | duty -<value> | position <rad> | stop | advance <deg>")
            return

        client = self.ensure_client()

        if cmd == "status":
            status = client.get_status()
            info = client.cal_info() if status.calibrated else None
            self.set_device_calibration_from_status(status, info)
            self.append_status(status)
            self.append_log(self.device_calibration_text)
            return

        if cmd == "stop":
            status = client.set_duty(0)
            self.set_device_calibration_from_status(status)
            self.append_status(status)
            return

        if cmd == "duty":
            if len(parts) != 2:
                raise ValueError("usage: duty <value>")
            duty = int(parts[1], 0)
            status = client.set_duty(duty)
            self.set_device_calibration_from_status(status)
            self.append_status(status)
            return

        if cmd == "advance":
            if len(parts) != 2:
                raise ValueError("usage: advance <deg>")
            advance_deg = int(parts[1], 0)
            status = client.set_advance_deg(advance_deg)
            self.set_device_calibration_from_status(status)
            self.append_log(f"advance={advance_deg}")
            self.append_status(status)
            return

        if cmd in ("position", "pos"):
            if len(parts) != 2:
                raise ValueError("usage: position <rad>")
            status = client.set_position_rad(float(parts[1]))
            self.set_device_calibration_from_status(status)
            self.append_log(format_status(status))
            return

        raise ValueError("unknown command")

    def start_worker(self) -> None:
        if self.worker is not None and self.worker.isRunning():
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
        self.stage_text = "calibration start"
        self.device_calibration_text = "motor: calibrating"
        self.refresh_status()
        self.tabs.setCurrentWidget(self.static_tab)
        self.append_log("start calibration")
        self.set_running(True)

        self.worker = TestWorker(
            port=self.port,
            address=self.address,
            chunk_size=self.chunk_size,
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
        self.stage_text = calibration_progress_text(next_index, CAL_CAPTURE_TOTAL_POINTS) if active else "capture done"
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
        self.update_summary()
        self.export_session()

    def on_done(self, text: str) -> None:
        self.stage_text = text
        if self.session is not None and self.session.static is not None and self.session.static.blob_crc32 is not None:
            self.device_calibration_text = "motor: calibrated crc32=0x%08X" % self.session.static.blob_crc32
        self.progress.setValue(100)
        self.append_log(text)
        self.set_running(False)
        self.refresh_status()
        if self.auto:
            QtCore.QTimer.singleShot(200, self.app.quit)

    def on_failed(self, text: str) -> None:
        self.stage_text = "failed"
        self.device_calibration_text = "motor: unknown"
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

        self.reset_plot(self.step_mag_plot)
        self.reset_plot(self.residual_plot)
        coeffs = static.affine_q20
        lut = static.angle_lut

        step_x = list(range(CAL_POINTS_PER_SWEEP))
        for rows, prefix, x_color, y_color in (
            (fwd, "forward", "#27ae60", "#2e86ab"),
            (rev, "reverse", "#c0392b", "#8e44ad"),
        ):
            self.step_mag_plot.plot(
                step_x,
                [row["x"] for row in rows],
                pen=pg.mkPen(x_color, width=2),
                symbol="o",
                symbolSize=4,
                name=f"{prefix} X",
            )
            self.step_mag_plot.plot(
                step_x,
                [row["y"] for row in rows],
                pen=pg.mkPen(y_color, width=2),
                symbol="t",
                symbolSize=4,
                name=f"{prefix} Y",
            )

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
            if self.session.static.forward_angle_sign is not None:
                lines.append(f"  forward angle sign: {self.session.static.forward_angle_sign:+d}")
            if self.session.static.blob_crc32 is not None:
                lines.append(f"  blob crc32: 0x{self.session.static.blob_crc32:08X}")
            if self.session.static.fit_max_error_deg is None:
                lines.append("  solve: failed before calibration blob was built")
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
                    "forward_angle_sign": self.session.static.forward_angle_sign,
                    "forward_sign_tests": self.session.static.forward_sign_tests,
                    "blob_crc32": self.session.static.blob_crc32,
                    "blob_size": self.session.static.blob_size,
                    "affine_q20": self.session.static.affine_q20,
                }
                with (export_dir / "static_summary.json").open("w", encoding="utf-8") as handle:
                    json.dump(to_serializable(summary), handle, indent=2, sort_keys=True)

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
    args = parser.parse_args()

    port = find_serial_port(args.port)
    window = Window(port, args.address, args.chunk_size, args.auto)
    return window.app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
