#!/usr/bin/env python3.11
from __future__ import annotations

import argparse
import math
import queue
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path

import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets

TOOLS_DIR = Path(__file__).resolve().parent / "tools"
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from pennycal import BrushedControl, EspBridge, Stm32Client, find_serial_port  # noqa: E402


Signal = getattr(QtCore, "Signal", QtCore.pyqtSignal)
Slot = getattr(QtCore, "Slot", QtCore.pyqtSlot)
HORIZONTAL = QtCore.Qt.Orientation.Horizontal
DASH_LINE = QtCore.Qt.PenStyle.DashLine
ADDRESSES = (2, 3)
TARGET_RATE_HZ = 100.0
TELEMETRY_RATE_HZ = 25.0
PLOT_WINDOW_S = 10.0


@dataclass(frozen=True)
class ServoConfig:
    amplitude: float = 2.0 * math.pi
    period: float = 2.0
    kp: float = 100.0
    kd: float = 0.1
    clip: int = 500


class ServoWorker(QtCore.QObject):
    connected = Signal(str)
    sample = Signal(object)
    status = Signal(str)
    failed = Signal(str)
    finished = Signal()

    def __init__(self, port: str, configs: dict[int, ServoConfig]):
        super().__init__()
        self.port = port
        self.configs = dict(configs)
        self.commands: queue.SimpleQueue[tuple[str, object | None]] = queue.SimpleQueue()
        self.stop_event = threading.Event()
        self.clients: dict[int, Stm32Client] = {}
        self.active = False
        self.phases = {address: 0.0 for address in ADDRESSES}
        self.targets = {address: 0.0 for address in ADDRESSES}

    def command(self, name: str, value: object | None = None) -> None:
        self.commands.put((name, value))

    def shutdown(self) -> None:
        self.stop_event.set()

    def stop_outputs(self) -> None:
        for client in self.clients.values():
            try:
                client.set_duty(0)
            except Exception:
                pass
        self.active = False
        self.targets = {address: 0.0 for address in ADDRESSES}

    def apply_control(self, address: int) -> None:
        config = self.configs[address]
        control = BrushedControl(kp=config.kp, kd=config.kd, clip=config.clip)
        self.clients[address].set_control(control)

    def handle_commands(self, now: float, last_phase_time: float) -> float:
        new_configs: dict[int, ServoConfig] = {}
        commands: list[str] = []
        while True:
            try:
                name, value = self.commands.get_nowait()
            except queue.Empty:
                break
            if name == "config":
                address, config = value  # type: ignore[misc]
                new_configs[address] = config
            else:
                commands.append(name)

        for address, config in new_configs.items():
            self.configs[address] = config
            if self.active:
                self.apply_control(address)

        for name in commands:
            if name == "start":
                for address, client in self.clients.items():
                    self.phases[address] = 0.0
                    self.targets[address] = 0.0
                    self.apply_control(address)
                    client.set_position_rad(0.0)
                self.active = True
                self.status.emit("Running synchronized sine trajectory")
                last_phase_time = time.monotonic()
            elif name == "stop":
                self.stop_outputs()
                self.status.emit("Stopped; both duties are zero")
            elif name == "zero":
                self.stop_outputs()
                time.sleep(0.1)
                for client in self.clients.values():
                    client.zero_position()
                self.phases = {address: 0.0 for address in ADDRESSES}
                self.status.emit("Stopped and zeroed both shaft positions")
        return last_phase_time

    @Slot()
    def run(self) -> None:
        try:
            port = find_serial_port(self.port)
            with EspBridge(port, baudrate=921600) as bridge:
                try:
                    self.run_connected(bridge, port)
                finally:
                    self.stop_outputs()
        except Exception as exc:
            self.failed.emit(str(exc))
        finally:
            self.finished.emit()

    def run_connected(self, bridge: EspBridge, port: str) -> None:
        bridge.enter_bridge("app")
        self.clients = {address: Stm32Client(bridge.serial, address) for address in ADDRESSES}
        values = {}
        for address, client in self.clients.items():
            client.set_duty(0)
            client.get_status(timeout=0.5, attempts=3)
            values[address] = client.get_pos_vel(timeout=0.2, attempts=3)
            self.status.emit(f"Found servo {address}")
        time.sleep(0.1)
        for client in self.clients.values():
            client.zero_position()

        self.connected.emit(port)
        self.status.emit("Connected, zeroed, and stopped")
        epoch = time.monotonic()
        last_phase_time = epoch
        next_target = epoch
        next_sample = epoch

        while not self.stop_event.is_set():
            now = time.monotonic()
            last_phase_time = self.handle_commands(now, last_phase_time)
            now = time.monotonic()

            if self.active:
                dt = now - last_phase_time
                last_phase_time = now
                for address in ADDRESSES:
                    config = self.configs[address]
                    self.phases[address] = math.fmod(
                        self.phases[address] + (2.0 * math.pi * dt / config.period),
                        2.0 * math.pi,
                    )
                    self.targets[address] = config.amplitude * math.sin(self.phases[address])
                if now >= next_target:
                    for address, client in self.clients.items():
                        client.send_position_rad(self.targets[address])
                    next_target = now + (1.0 / TARGET_RATE_HZ)
            else:
                last_phase_time = now

            if now >= next_sample:
                for address, client in self.clients.items():
                    try:
                        values[address] = client.get_pos_vel(timeout=0.12, attempts=3)
                    except TimeoutError:
                        self.status.emit(f"Missed one telemetry sample from servo {address}")
                self.sample.emit(
                    (
                        now - epoch,
                        self.targets[2],
                        values[2].position_rad,
                        values[2].velocity_rad_s,
                        self.targets[3],
                        values[3].position_rad,
                        values[3].velocity_rad_s,
                    )
                )
                next_sample = now + (1.0 / TELEMETRY_RATE_HZ)
            time.sleep(0.001)


class SliderRow(QtWidgets.QWidget):
    valueChanged = Signal(float)

    def __init__(
        self,
        label: str,
        minimum: float,
        maximum: float,
        step: float,
        value: float,
        decimals: int,
        suffix: str = "",
    ):
        super().__init__()
        self.step = step
        layout = QtWidgets.QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        name = QtWidgets.QLabel(label)
        name.setMinimumWidth(82)
        self.slider = QtWidgets.QSlider(HORIZONTAL)
        self.slider.setRange(round(minimum / step), round(maximum / step))
        self.spin = QtWidgets.QDoubleSpinBox()
        self.spin.setRange(minimum, maximum)
        self.spin.setSingleStep(step)
        self.spin.setDecimals(decimals)
        self.spin.setSuffix(suffix)
        self.spin.setMinimumWidth(112)
        layout.addWidget(name)
        layout.addWidget(self.slider, 1)
        layout.addWidget(self.spin)
        self.slider.valueChanged.connect(self.slider_changed)
        self.spin.valueChanged.connect(self.spin_changed)
        self.setValue(value)

    def value(self) -> float:
        return self.spin.value()

    def setValue(self, value: float) -> None:
        self.spin.setValue(value)
        self.slider.setValue(round(value / self.step))

    def slider_changed(self, value: int) -> None:
        number = value * self.step
        self.spin.blockSignals(True)
        self.spin.setValue(number)
        self.spin.blockSignals(False)
        self.valueChanged.emit(number)

    def spin_changed(self, value: float) -> None:
        self.slider.blockSignals(True)
        self.slider.setValue(round(value / self.step))
        self.slider.blockSignals(False)
        self.valueChanged.emit(value)


class ServoControls(QtWidgets.QGroupBox):
    changed = Signal()

    def __init__(self, address: int):
        super().__init__(f"Servo address {address}")
        layout = QtWidgets.QVBoxLayout(self)
        self.amplitude = SliderRow("Amplitude", 0.0, 4.0 * math.pi, 0.01, 2.0 * math.pi, 2, " rad")
        self.period = SliderRow("Period", 0.25, 20.0, 0.01, 2.0, 2, " s")
        self.kp = SliderRow("Kp", 0.0, 300.0, 0.1, 100.0, 1)
        self.kd = SliderRow("Kd", 0.0, 127.9, 0.1, 0.1, 1)
        self.clip = SliderRow("Clip", 0.0, 799.0, 1.0, 500.0, 0)
        for row in (self.amplitude, self.period, self.kp, self.kd, self.clip):
            row.valueChanged.connect(lambda _value: self.changed.emit())
            layout.addWidget(row)

    def config(self) -> ServoConfig:
        return ServoConfig(
            amplitude=self.amplitude.value(),
            period=self.period.value(),
            kp=self.kp.value(),
            kd=self.kd.value(),
            clip=round(self.clip.value()),
        )


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, port: str):
        super().__init__()
        self.worker: ServoWorker | None = None
        self.thread: QtCore.QThread | None = None
        self.times: deque[float] = deque(maxlen=2500)
        self.desired = {2: deque(maxlen=2500), 3: deque(maxlen=2500)}
        self.positions = {2: deque(maxlen=2500), 3: deque(maxlen=2500)}
        self.last_sample: tuple[float, ...] | None = None

        self.setWindowTitle("PennyESC dual brushed-servo sine control")
        self.resize(1220, 900)
        root = QtWidgets.QWidget()
        self.setCentralWidget(root)
        layout = QtWidgets.QVBoxLayout(root)

        connection = QtWidgets.QHBoxLayout()
        connection.addWidget(QtWidgets.QLabel("Bridge port"))
        self.port = QtWidgets.QLineEdit(port)
        connection.addWidget(self.port, 1)
        self.connect_button = QtWidgets.QPushButton("Connect")
        self.connect_button.clicked.connect(self.toggle_connection)
        connection.addWidget(self.connect_button)
        layout.addLayout(connection)

        self.controls = {address: ServoControls(address) for address in ADDRESSES}
        for controls in self.controls.values():
            controls.changed.connect(self.config_changed)

        buttons = QtWidgets.QHBoxLayout()
        self.zero_button = QtWidgets.QPushButton("Stop && Zero")
        self.start_button = QtWidgets.QPushButton("Start")
        self.stop_button = QtWidgets.QPushButton("Stop")
        self.zero_button.clicked.connect(lambda: self.send_command("zero"))
        self.start_button.clicked.connect(lambda: self.send_command("start"))
        self.stop_button.clicked.connect(lambda: self.send_command("stop"))
        for button in (self.zero_button, self.start_button, self.stop_button):
            button.setEnabled(False)
            buttons.addWidget(button)
        layout.addLayout(buttons)

        self.servo_status = QtWidgets.QLabel("Disconnected")
        layout.addWidget(self.servo_status)

        servo_layout = QtWidgets.QHBoxLayout()
        self.plots: dict[int, pg.PlotWidget] = {}
        self.curves: dict[int, tuple[pg.PlotDataItem, pg.PlotDataItem]] = {}
        for address, color in ((2, "#00c8ff"), (3, "#ff66cc")):
            column = QtWidgets.QVBoxLayout()
            column.addWidget(self.controls[address])
            plot = pg.PlotWidget(title=f"Servo address {address}")
            plot.showGrid(x=True, y=True, alpha=0.25)
            plot.setLabel("left", "Angle", units="rad")
            plot.setLabel("bottom", "Time", units="s")
            plot.addLegend()
            desired = plot.plot(
                name="Desired",
                pen=pg.mkPen("#ffd166", width=2, style=DASH_LINE),
            )
            current = plot.plot(name="Current", pen=pg.mkPen(color, width=2))
            column.addWidget(plot, 1)
            servo_layout.addLayout(column, 1)
            self.plots[address] = plot
            self.curves[address] = (desired, current)
        layout.addLayout(servo_layout, 1)

        self.config_timer = QtCore.QTimer(self)
        self.config_timer.setSingleShot(True)
        self.config_timer.timeout.connect(self.apply_config)
        self.plot_timer = QtCore.QTimer(self)
        self.plot_timer.timeout.connect(self.update_plots)
        self.plot_timer.start(50)

    def current_configs(self) -> dict[int, ServoConfig]:
        return {address: controls.config() for address, controls in self.controls.items()}

    def config_changed(self) -> None:
        self.config_timer.start(100)

    def apply_config(self) -> None:
        if self.worker is not None:
            for address, config in self.current_configs().items():
                self.worker.command("config", (address, config))

    def toggle_connection(self) -> None:
        if self.worker is not None:
            self.disconnect()
            return
        self.clear_plot()
        self.worker = ServoWorker(self.port.text().strip() or "auto", self.current_configs())
        self.thread = QtCore.QThread(self)
        self.worker.moveToThread(self.thread)
        self.thread.started.connect(self.worker.run)
        self.worker.connected.connect(self.on_connected)
        self.worker.sample.connect(self.on_sample)
        self.worker.status.connect(self.servo_status.setText)
        self.worker.failed.connect(self.on_failed)
        self.worker.finished.connect(self.thread.quit)
        self.thread.finished.connect(self.on_disconnected)
        self.connect_button.setText("Connecting…")
        self.connect_button.setEnabled(False)
        self.thread.start()

    def on_connected(self, port: str) -> None:
        self.port.setText(port)
        self.connect_button.setText("Disconnect")
        self.connect_button.setEnabled(True)
        for button in (self.zero_button, self.start_button, self.stop_button):
            button.setEnabled(True)

    def on_failed(self, message: str) -> None:
        self.servo_status.setText(f"Error: {message}")

    def disconnect(self) -> None:
        if self.worker is not None:
            self.connect_button.setEnabled(False)
            self.servo_status.setText("Stopping outputs and disconnecting…")
            self.worker.shutdown()

    def on_disconnected(self) -> None:
        if not self.servo_status.text().startswith("Error:"):
            self.servo_status.setText("Disconnected; outputs stopped")
        self.worker = None
        self.thread = None
        self.connect_button.setText("Connect")
        self.connect_button.setEnabled(True)
        for button in (self.zero_button, self.start_button, self.stop_button):
            button.setEnabled(False)

    def send_command(self, name: str) -> None:
        if self.worker is not None:
            self.worker.command(name)

    def on_sample(self, sample: tuple[float, ...]) -> None:
        elapsed, desired2, pos2, vel2, desired3, pos3, vel3 = sample
        self.times.append(elapsed)
        self.desired[2].append(desired2)
        self.desired[3].append(desired3)
        self.positions[2].append(pos2)
        self.positions[3].append(pos3)
        self.last_sample = sample
        self.servo_status.setText(
            f"Servo 2: {pos2:+.3f} rad, {vel2:+.2f} rad/s    |    "
            f"Servo 3: {pos3:+.3f} rad, {vel3:+.2f} rad/s"
        )

    def update_plots(self) -> None:
        if not self.times:
            return
        times = list(self.times)
        start = max(0.0, times[-1] - PLOT_WINDOW_S)
        first = 0
        while first < len(times) and times[first] < start:
            first += 1
        x = times[first:]
        for address in ADDRESSES:
            desired_curve, current_curve = self.curves[address]
            desired_curve.setData(x, list(self.desired[address])[first:])
            current_curve.setData(x, list(self.positions[address])[first:])
            self.plots[address].setXRange(start, max(PLOT_WINDOW_S, times[-1]), padding=0.0)

    def clear_plot(self) -> None:
        self.times.clear()
        for values in self.desired.values():
            values.clear()
        for values in self.positions.values():
            values.clear()
        for desired, current in self.curves.values():
            desired.setData([], [])
            current.setData([], [])

    def closeEvent(self, event) -> None:  # noqa: N802
        if self.worker is not None and self.thread is not None:
            self.worker.shutdown()
            self.thread.wait(8000)
        event.accept()


def main() -> int:
    parser = argparse.ArgumentParser(description="Control PennyESC servos 2 and 3 with a sine trajectory")
    parser.add_argument("--port", default="auto")
    args = parser.parse_args()

    pg.setConfigOptions(antialias=True)
    app = QtWidgets.QApplication(sys.argv[:1])
    window = MainWindow(args.port)
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
