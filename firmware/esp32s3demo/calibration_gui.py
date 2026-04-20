#!/usr/bin/env python3.11
from __future__ import annotations

import argparse
import sys
from glob import glob
from pathlib import Path

import serial

try:
    from serial.tools import list_ports
except ImportError:
    list_ports = None

try:
    import pyqtgraph as pg
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
    capture_points,
    reduce_capture_points,
    solve_capture,
    upload_calibration,
    verify_calibration,
)

Signal = getattr(QtCore, "Signal", QtCore.pyqtSignal)


def format_status(status) -> str:
    return (
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

    for pattern in ("/dev/cu.usbmodem*", "/dev/ttyACM*", "/dev/ttyUSB*"):
        matches = sorted(glob(pattern))
        if matches:
            return matches[0]

    raise SystemExit("No serial port found")


class CalibrationWorker(QtCore.QThread):
    log = Signal(str)
    capture_status = Signal(int, int, int)
    sample = Signal(int, int, int, int, int, int)
    point = Signal(int, int, int, int, int, int)
    solved = Signal(float, float, int)
    upload = Signal(int, int)
    done = Signal(str)
    failed = Signal(str)

    def __init__(self, port: str, address: int, chunk_size: int) -> None:
        super().__init__()
        self.port = port
        self.address = address
        self.chunk_size = chunk_size

    def run(self) -> None:
        try:
            self.log.emit(f"open {self.port}")
            with EspBridge(self.port) as bridge:
                bridge.enter_bridge("app")
                self.log.emit("bridge app")
                try:
                    client = Stm32Client(bridge.serial, address=self.address)
                    raw_points = capture_points(client, status_cb=self.on_capture_status, sample_cb=self.on_sample)
                    self.log.emit("average")
                    points = reduce_capture_points(raw_points)
                    for point in points:
                        self.on_point(point)
                    self.log.emit("solve")
                    solved = solve_capture(points)
                    self.solved.emit(solved.fit_max_error_deg, solved.sweep_delta_deg, solved.blob_crc32)
                    self.log.emit(
                        "fit %.2f deg sweep %.2f deg crc32 0x%08X"
                        % (solved.fit_max_error_deg, solved.sweep_delta_deg, solved.blob_crc32)
                    )
                    bridge.exit_bridge()
                    bridge.enter_bridge("app")
                    self.log.emit("upload")
                    client = Stm32Client(bridge.serial, address=self.address)
                    upload_calibration(client, solved, self.chunk_size, upload_cb=self.on_upload)
                    self.log.emit("commit")
                    result, valid = client.cal_commit()
                    if result != 0 or valid != 1:
                        raise CalibrationError(f"CAL_COMMIT failed with result {result}")
                    info = verify_calibration(client, solved)
                    self.done.emit(
                        "calibrated crc32=0x%08X size=%d fit=%.2fdeg sweep=%.2fdeg"
                        % (info.blob_crc32, info.blob_size, solved.fit_max_error_deg, solved.sweep_delta_deg)
                    )
                finally:
                    bridge.exit_bridge()
        except (CalibrationError, TimeoutError, serial.SerialException) as exc:
            self.failed.emit(str(exc))

    def on_capture_status(self, status) -> None:
        self.capture_status.emit(status.active, status.next_index, status.total_points)

    def on_sample(self, index: int, step_index: int, sweep_dir: int, status) -> None:
        self.sample.emit(index, step_index, sweep_dir, status.x, status.y, status.z)

    def on_point(self, point) -> None:
        self.point.emit(point.index, point.step_index, point.sweep_dir, point.x, point.y, point.z)

    def on_upload(self, done: int, total: int) -> None:
        self.upload.emit(done, total)


class Window:
    def __init__(self, port: str, address: int, chunk_size: int, auto: bool) -> None:
        self.port = port
        self.address = address
        self.chunk_size = chunk_size
        self.auto = auto
        self.worker: CalibrationWorker | None = None
        self.bridge: EspBridge | None = None
        self.client: Stm32Client | None = None

        self.capture_count = 0
        self.bin_count = 0
        self.live_index: list[int] = []
        self.live_step: list[int] = []
        self.live_pos: list[float] = []
        self.live_x: list[int] = []
        self.live_y: list[int] = []
        self.live_z: list[int] = []
        self.bin_pos: list[float] = []
        self.bin_x: list[int] = []
        self.bin_y: list[int] = []
        self.bin_z: list[int] = []
        self.fit_text = "-"
        self.sweep_text = "-"
        self.crc_text = "-"
        self.stage_text = "idle"

        self.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv)
        self.app.aboutToQuit.connect(self.close_client)
        self.win = QtWidgets.QWidget()
        self.win.setWindowTitle("pennyesc calibration")
        self.win.resize(1200, 860)

        layout = QtWidgets.QVBoxLayout(self.win)

        self.status = QtWidgets.QLabel()
        layout.addWidget(self.status)

        controls = QtWidgets.QHBoxLayout()
        controls.addWidget(QtWidgets.QLabel("Port"))
        self.port_label = QtWidgets.QLabel(self.port)
        controls.addWidget(self.port_label)
        controls.addWidget(QtWidgets.QLabel("ESC"))
        self.esc_box = QtWidgets.QComboBox()
        for esc in range(3):
            self.esc_box.addItem(str(esc), esc)
        self.esc_box.setCurrentIndex(self.address)
        controls.addWidget(self.esc_box)

        self.start_button = QtWidgets.QPushButton("Start Calibration")
        self.start_button.clicked.connect(self.start_calibration)
        controls.addWidget(self.start_button)

        self.clear_button = QtWidgets.QPushButton("Clear Plot")
        self.clear_button.clicked.connect(self.clear_data)
        controls.addWidget(self.clear_button)
        controls.addStretch(1)
        layout.addLayout(controls)

        self.progress = QtWidgets.QProgressBar()
        self.progress.setRange(0, 100)
        layout.addWidget(self.progress)

        self.field_plot = pg.PlotWidget(title="Magnetic Field By Step")
        self.field_plot.showGrid(x=True, y=True, alpha=0.25)
        self.field_plot.setLabel("bottom", "Step Bin")
        self.field_plot.setLabel("left", "Raw Field")
        self.field_plot.setXRange(-0.5, CAL_POINTS_PER_SWEEP - 0.5)
        self.field_plot.addLegend()
        self.x_live_curve = self.field_plot.plot(
            name="X live",
            pen=None,
            symbol="o",
            symbolSize=5,
            symbolBrush=pg.mkBrush(231, 76, 60, 110),
            symbolPen=None,
        )
        self.y_live_curve = self.field_plot.plot(
            name="Y live",
            pen=None,
            symbol="o",
            symbolSize=5,
            symbolBrush=pg.mkBrush(46, 204, 113, 110),
            symbolPen=None,
        )
        self.z_live_curve = self.field_plot.plot(
            name="Z live",
            pen=None,
            symbol="o",
            symbolSize=5,
            symbolBrush=pg.mkBrush(52, 152, 219, 110),
            symbolPen=None,
        )
        self.x_bin_curve = self.field_plot.plot(
            name="X avg",
            pen=None,
            symbol="x",
            symbolSize=9,
            symbolPen=pg.mkPen("#e74c3c", width=2),
        )
        self.y_bin_curve = self.field_plot.plot(
            name="Y avg",
            pen=None,
            symbol="x",
            symbolSize=9,
            symbolPen=pg.mkPen("#2ecc71", width=2),
        )
        self.z_bin_curve = self.field_plot.plot(
            name="Z avg",
            pen=None,
            symbol="x",
            symbolSize=9,
            symbolPen=pg.mkPen("#3498db", width=2),
        )
        layout.addWidget(self.field_plot, stretch=2)

        self.step_plot = pg.PlotWidget(title="Step Progress")
        self.step_plot.showGrid(x=True, y=True, alpha=0.25)
        self.step_plot.setLabel("bottom", "Capture Index")
        self.step_plot.setLabel("left", "Step")
        self.step_plot.setYRange(-1, 36)
        self.step_curve = self.step_plot.plot(pen=pg.mkPen("#f39c12", width=2), symbol="o", symbolSize=6)
        layout.addWidget(self.step_plot, stretch=1)

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
        layout.addWidget(self.log_box, stretch=1)

        self.win.show()
        self.refresh_status()

        if self.auto:
            QtCore.QTimer.singleShot(0, self.start_calibration)

    def clear_data(self) -> None:
        self.capture_count = 0
        self.bin_count = 0
        self.live_index.clear()
        self.live_step.clear()
        self.live_pos.clear()
        self.live_x.clear()
        self.live_y.clear()
        self.live_z.clear()
        self.bin_pos.clear()
        self.bin_x.clear()
        self.bin_y.clear()
        self.bin_z.clear()
        self.fit_text = "-"
        self.sweep_text = "-"
        self.crc_text = "-"
        self.progress.setValue(0)
        self.update_plot()
        self.refresh_status()

    def append_log(self, text: str) -> None:
        self.log_box.appendPlainText(text)
        if self.auto:
            print(text, flush=True)

    def refresh_status(self) -> None:
        self.status.setText(
            "port=%s | esc=%d | stage=%s | capture=%d/%d | bins=%d/%d | fit=%s | sweep=%s | crc=%s"
            % (
                self.port,
                int(self.esc_box.currentData()),
                self.stage_text,
                self.capture_count,
                CAL_CAPTURE_TOTAL_POINTS,
                self.bin_count,
                CAL_TOTAL_POINTS,
                self.fit_text,
                self.sweep_text,
                self.crc_text,
            )
        )

    def update_plot(self) -> None:
        self.x_live_curve.setData(self.live_pos, self.live_x)
        self.y_live_curve.setData(self.live_pos, self.live_y)
        self.z_live_curve.setData(self.live_pos, self.live_z)
        self.x_bin_curve.setData(self.bin_pos, self.bin_x)
        self.y_bin_curve.setData(self.bin_pos, self.bin_y)
        self.z_bin_curve.setData(self.bin_pos, self.bin_z)
        self.step_curve.setData(self.live_index, self.live_step)

    def set_running(self, running: bool) -> None:
        self.start_button.setEnabled(not running)
        self.esc_box.setEnabled(not running)
        self.command_line.setEnabled(not running)
        self.command_button.setEnabled(not running)

    def ensure_client(self) -> Stm32Client:
        address = int(self.esc_box.currentData())
        if self.bridge is None or self.client is None:
            self.bridge = EspBridge(self.port)
            try:
                self.bridge.enter_bridge("app")
                self.client = Stm32Client(self.bridge.serial, address=address)
            except Exception:
                self.close_client()
                raise
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
            self.append_log("error: stop calibration before sending commands")
            return
        try:
            self.run_terminal_command(text)
        except (CalibrationError, TimeoutError, ValueError, serial.SerialException) as exc:
            if isinstance(exc, (TimeoutError, serial.SerialException)):
                self.close_client()
            self.append_log(f"error: {exc}")

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
            self.append_log(f"advance={advance_deg}")
            self.append_log(format_status(status))
            return

        raise ValueError("unknown command")

    def start_calibration(self) -> None:
        if self.worker is not None and self.worker.isRunning():
            return

        self.close_client()
        self.clear_data()
        self.address = int(self.esc_box.currentData())
        self.stage_text = "starting"
        self.refresh_status()
        self.append_log("start")
        self.set_running(True)

        self.worker = CalibrationWorker(self.port, self.address, self.chunk_size)
        self.worker.log.connect(self.on_log)
        self.worker.capture_status.connect(self.on_capture_status)
        self.worker.sample.connect(self.on_sample)
        self.worker.point.connect(self.on_point)
        self.worker.solved.connect(self.on_solved)
        self.worker.upload.connect(self.on_upload)
        self.worker.done.connect(self.on_done)
        self.worker.failed.connect(self.on_failed)
        self.worker.start()

    def on_log(self, text: str) -> None:
        self.append_log(text)
        self.stage_text = text
        self.refresh_status()

    def on_capture_status(self, active: int, next_index: int, total: int) -> None:
        self.stage_text = "capture active" if active else "capture done"
        self.capture_count = next_index
        if total > 0:
            self.progress.setValue((next_index * 60) // total)
        self.refresh_status()

    def on_sample(self, index: int, step_index: int, sweep_dir: int, x: int, y: int, z: int) -> None:
        self.capture_count = index + 1
        self.live_index.append(index)
        self.live_step.append(step_index)
        self.live_pos.append(step_index + (-0.14 if sweep_dir == 0 else 0.14))
        self.live_x.append(x)
        self.live_y.append(y)
        self.live_z.append(z)
        self.update_plot()
        self.refresh_status()

    def on_point(self, index: int, step_index: int, sweep_dir: int, x: int, y: int, z: int) -> None:
        self.bin_count = index + 1
        self.bin_pos.append(step_index + (-0.22 if sweep_dir == 0 else 0.22))
        self.bin_x.append(x)
        self.bin_y.append(y)
        self.bin_z.append(z)
        self.append_log("point %d step=%d sweep=%d x=%d y=%d z=%d" % (index, step_index, sweep_dir, x, y, z))
        self.progress.setValue(60 + ((index + 1) * 18) // CAL_TOTAL_POINTS)
        self.update_plot()
        self.refresh_status()

    def on_solved(self, fit_error: float, sweep_delta: float, crc32: int) -> None:
        self.stage_text = "solve"
        self.fit_text = "%.2f deg" % fit_error
        self.sweep_text = "%.2f deg" % sweep_delta
        self.crc_text = "0x%08X" % crc32
        self.progress.setValue(80)
        self.refresh_status()

    def on_upload(self, done: int, total: int) -> None:
        self.stage_text = "upload"
        if total > 0:
            self.progress.setValue(80 + (done * 18) // total)
        self.refresh_status()

    def on_done(self, text: str) -> None:
        self.stage_text = "done"
        self.capture_count = CAL_CAPTURE_TOTAL_POINTS
        self.bin_count = CAL_TOTAL_POINTS
        self.progress.setValue(100)
        self.append_log(text)
        self.refresh_status()
        self.set_running(False)
        if self.auto:
            QtCore.QTimer.singleShot(200, self.app.quit)

    def on_failed(self, text: str) -> None:
        self.stage_text = "failed"
        self.append_log(f"error: {text}")
        self.refresh_status()
        self.set_running(False)
        if self.auto:
            self.app.exit(1)


def main() -> int:
    parser = argparse.ArgumentParser(description="pennyesc calibration GUI")
    parser.add_argument("--port")
    parser.add_argument("--address", type=int, default=0)
    parser.add_argument("--chunk-size", type=int, default=DEFAULT_CHUNK_SIZE)
    parser.add_argument("--auto", action="store_true", help="start immediately and exit when complete")
    args = parser.parse_args()

    port = find_serial_port(args.port)
    window = Window(port, args.address, args.chunk_size, args.auto)
    return window.app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
