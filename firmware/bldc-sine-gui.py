#!/usr/bin/env python3.11
from __future__ import annotations

import argparse
import math
import queue
import select
import struct
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path

from PyQt6 import QtCore, QtWidgets
import pyqtgraph as pg

sys.path.insert(0, str(Path(__file__).resolve().parent / 'tools'))
from pennycal import Control, EspBridge, Stm32Client, find_serial_port, RAD_TO_TURN32, TURN32_TO_RAD, unpack_status
from pnyproto import CMD_GET_POS_VEL, CMD_GET_STATUS, CMD_SEND_POSITION, encode_frame

RATE_HZ = 500
PLOT_HZ = 30
PLOT_SAMPLE_HZ = 100


@dataclass(frozen=True)
class MotorConfig:
    amplitude: float = 1.0
    period: float = 4.0
    kp: float = 100.0
    kd: float = 2.0
    clip: int = 150
    deadband: float = 0.01
    friction: int = 75
    smoothing: float = 1.0


class MotorWorker(QtCore.QObject):
    connected = QtCore.pyqtSignal(str)
    sample = QtCore.pyqtSignal(object)
    rates = QtCore.pyqtSignal(float, float)
    status = QtCore.pyqtSignal(str)
    failed = QtCore.pyqtSignal(str)
    finished = QtCore.pyqtSignal()

    def __init__(self, port: str, address: int, config: MotorConfig):
        super().__init__()
        self.port = port
        self.address = address
        self.config = config
        self.commands = queue.SimpleQueue()
        self.stop_event = threading.Event()
        self.active = False
        self.phase = 0.0
        self.target = 0.0
        self.crc_errors = 0

    def command(self, name: str, value=None) -> None:
        self.commands.put((name, value))

    def shutdown(self) -> None:
        self.stop_event.set()

    def check(self, status) -> None:
        if status.result != 0 or status.faults:
            raise RuntimeError(f'ESC {self.address}: result {status.result}, faults 0x{status.faults:02X}')
        if not status.calibrated:
            raise RuntimeError('Calibrate the motor in Penny GUI first')

    def apply_control(self, client: Stm32Client) -> None:
        self.check(client.set_control(Control(
            kp=self.config.kp, kd=self.config.kd, clip=self.config.clip,
            deadband_turn32=round(self.config.deadband * RAD_TO_TURN32),
        )))

    @QtCore.pyqtSlot()
    def run(self) -> None:
        try:
            port = find_serial_port(self.port)
            with EspBridge(port) as bridge:
                bridge.enter_bridge('app')
                client = Stm32Client(bridge.serial, self.address)
                try:
                    self.check(client.brake())
                    time.sleep(0.1)
                    self.check(client.zero_position())
                    self.connected.emit(port)
                    self.status.emit('Connected, zeroed, and stopped')
                    self.run_motor(client)
                finally:
                    try:
                        self.check(client.brake())
                    finally:
                        bridge.exit_bridge()
        except Exception as exc:
            self.failed.emit(str(exc))
        finally:
            self.finished.emit()

    def run_motor(self, client: Stm32Client) -> None:
        epoch = last_phase = time.monotonic()
        next_tick = next_status = next_plot = rate_time = epoch
        last_feedback = epoch
        position_rad = 0.0
        sent = received = 0
        samples = []
        poll_frame = encode_frame(self.address, CMD_GET_POS_VEL)
        status_frame = encode_frame(self.address, CMD_GET_STATUS)
        client.port.timeout = 0
        while not self.stop_event.is_set():
            config = None
            actions = []
            while True:
                try:
                    name, value = self.commands.get_nowait()
                except queue.Empty:
                    break
                if name == 'config':
                    config = value
                else:
                    actions.append(name)
            if config is not None:
                self.config = config
                if self.active:
                    self.apply_control(client)
            for name in actions:
                if name == 'start':
                    self.phase = 0.0
                    self.target = 0.0
                    client.send_trajectory(0.0, 0.0, 0)
                    self.apply_control(client)
                    self.active = True
                    last_phase = time.monotonic()
                    self.status.emit('Running')
                elif name in ('stop', 'zero'):
                    self.active = False
                    self.check(client.brake())
                    if name == 'zero':
                        time.sleep(0.1)
                        self.check(client.zero_position())
                        self.target = 0.0
                    else:
                        self.target = client.get_pos_vel(attempts=1).position_rad
                    self.status.emit('Stopped and zeroed' if name == 'zero' else 'Stopped; motor braked')
            now = time.monotonic()
            if now >= next_tick:
                packet = bytearray()
                if self.active:
                    self.phase = (self.phase + 2 * math.pi * (now - last_phase) / self.config.period) % (2 * math.pi)
                    self.target = self.config.amplitude * math.sin(self.phase)
                    velocity = self.config.amplitude * (2 * math.pi / self.config.period) * math.cos(self.phase)
                    feedforward = round(self.config.friction * math.tanh(velocity / self.config.smoothing))
                    error = self.target - position_rad
                    toward_target = error if velocity > 0 else -error
                    fade = max(self.config.deadband, 0.01)
                    # Friction assist must not push against the position correction.
                    feedforward = round(feedforward * max(0.0, math.tanh(toward_target / fade)))
                    payload = struct.pack('<iih', round(self.target * RAD_TO_TURN32),
                                          round(velocity * RAD_TO_TURN32), feedforward)
                    packet.extend(encode_frame(self.address, CMD_SEND_POSITION, payload))
                last_phase = now
                packet.extend(poll_frame)
                if now >= next_status:
                    packet.extend(status_frame)
                    next_status = now + 0.5
                client.port.write(packet)
                sent += 1
                next_tick += 1 / RATE_HZ
                # Catch up after short GUI stalls; discard work older than 20 ms.
                if next_tick < now - 10 / RATE_HZ:
                    next_tick = now
            for address, command, payload in client.reader.poll():
                if address != self.address:
                    continue
                if command == CMD_GET_STATUS:
                    self.check(unpack_status(payload))
                elif command == CMD_GET_POS_VEL:
                    position, velocity = struct.unpack('<ii', payload)
                    last_feedback = time.monotonic()
                    position_rad = position * TURN32_TO_RAD
                    received += 1
                    samples.append((last_feedback - epoch, self.target,
                                    position_rad, velocity * TURN32_TO_RAD))
            if now - last_feedback > 0.25:
                raise TimeoutError('No motor feedback for 250 ms; stopping')
            if now >= next_plot and samples:
                self.sample.emit(samples)
                samples = []
                next_plot = now + 1 / PLOT_HZ
            if now - rate_time >= 1:
                elapsed = now - rate_time
                self.rates.emit(sent / elapsed, received / elapsed)
                self.crc_errors = client.reader.crc_errors
                sent = received = 0
                rate_time = now
            select.select([client.port.fileno()], [], [], max(0, next_tick - time.monotonic()))


class SliderRow(QtWidgets.QWidget):
    changed = QtCore.pyqtSignal()

    def __init__(self, label, minimum, maximum, step, value, decimals, suffix=''):
        super().__init__()
        self.step = step
        layout = QtWidgets.QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        name = QtWidgets.QLabel(label)
        name.setMinimumWidth(90)
        self.slider = QtWidgets.QSlider(QtCore.Qt.Orientation.Horizontal)
        self.slider.setRange(round(minimum / step), round(maximum / step))
        self.spin = QtWidgets.QDoubleSpinBox()
        self.spin.setRange(minimum, maximum)
        self.spin.setSingleStep(step)
        self.spin.setDecimals(decimals)
        self.spin.setSuffix(suffix)
        self.spin.setMinimumWidth(110)
        self.spin.setValue(value)
        self.slider.setValue(round(value / step))
        layout.addWidget(name)
        layout.addWidget(self.slider, 1)
        layout.addWidget(self.spin)
        self.slider.valueChanged.connect(self.slider_changed)
        self.spin.valueChanged.connect(self.spin_changed)

    def slider_changed(self, value):
        self.spin.blockSignals(True)
        self.spin.setValue(value * self.step)
        self.spin.blockSignals(False)
        self.changed.emit()

    def spin_changed(self, value):
        self.slider.blockSignals(True)
        self.slider.setValue(round(value / self.step))
        self.slider.blockSignals(False)
        self.changed.emit()

    def value(self):
        return self.spin.value()


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, port: str, address: int):
        super().__init__()
        self.worker = self.thread = None
        self.times = deque(maxlen=1000)
        self.targets = deque(maxlen=1000)
        self.positions = deque(maxlen=1000)
        self.plot_next = 0.0
        self.setWindowTitle('PennyESC BLDC sine control')
        self.resize(920, 740)
        root = QtWidgets.QWidget()
        self.setCentralWidget(root)
        layout = QtWidgets.QVBoxLayout(root)
        connection = QtWidgets.QHBoxLayout()
        connection.addWidget(QtWidgets.QLabel('Bridge port'))
        self.port = QtWidgets.QLineEdit(port)
        connection.addWidget(self.port, 1)
        connection.addWidget(QtWidgets.QLabel('ESC'))
        self.address = QtWidgets.QSpinBox()
        self.address.setRange(0, 15)
        self.address.setValue(address)
        connection.addWidget(self.address)
        self.connect_button = QtWidgets.QPushButton('Connect')
        self.connect_button.clicked.connect(self.toggle_connection)
        connection.addWidget(self.connect_button)
        layout.addLayout(connection)
        defaults = MotorConfig()
        self.amplitude = SliderRow('Amplitude', 0, 4 * math.pi, 0.01, defaults.amplitude, 2, ' rad')
        self.period = SliderRow('Period', 0.25, 20, 0.01, defaults.period, 2, ' s')
        self.kp = SliderRow('Kp', 0, 127.9, 0.1, defaults.kp, 1)
        self.kd = SliderRow('Kd', 0, 127.9, 0.01, defaults.kd, 2)
        self.clip = SliderRow('Duty limit', 0, 799, 1, defaults.clip, 0)
        self.deadband = SliderRow('Deadband', 0, 0.25, 0.001, defaults.deadband, 3, ' rad')
        self.friction = SliderRow('Feedforward', 0, 150, 1, defaults.friction, 0)
        self.smoothing = SliderRow('FF smoothing', 0.05, 5, 0.01, defaults.smoothing, 2, ' rad/s')
        self.deadband.setToolTip('Position tolerance for proportional correction; damping stays active.')
        self.friction.setToolTip('Friction assist toward the target, faded near the target and sine reversals.')
        self.smoothing.setToolTip('Increase for softer feedforward near reversals.')
        self.config_timer = QtCore.QTimer(self)
        self.config_timer.setSingleShot(True)
        self.config_timer.timeout.connect(self.apply_config)
        for row in (self.amplitude, self.period, self.kp, self.kd, self.clip, self.deadband, self.friction, self.smoothing):
            row.changed.connect(lambda: self.config_timer.start(100))
            layout.addWidget(row)
        buttons = QtWidgets.QHBoxLayout()
        self.zero_button = QtWidgets.QPushButton('Stop && Zero')
        self.start_button = QtWidgets.QPushButton('Start')
        self.stop_button = QtWidgets.QPushButton('Stop')
        for button, name in ((self.zero_button, 'zero'), (self.start_button, 'start'), (self.stop_button, 'stop')):
            button.setEnabled(False)
            button.clicked.connect(lambda checked, action=name: self.send_command(action))
            buttons.addWidget(button)
        layout.addLayout(buttons)
        self.message = QtWidgets.QLabel('Disconnected')
        self.values = QtWidgets.QLabel('Position: —    Velocity: —')
        self.rate_label = QtWidgets.QLabel('Commands: — Hz    Feedback: — Hz')
        self.plot_dirty = False
        layout.addWidget(self.message)
        layout.addWidget(self.values)
        layout.addWidget(self.rate_label)
        self.plot = pg.PlotWidget()
        self.plot.showGrid(x=True, y=True, alpha=0.25)
        self.plot.setLabel('left', 'Angle', units='rad')
        self.plot.setLabel('bottom', 'Time', units='s')
        self.plot.addLegend()
        self.target_curve = self.plot.plot(name='Target', pen=pg.mkPen('#ffd166', width=1, style=QtCore.Qt.PenStyle.DashLine))
        self.position_curve = self.plot.plot(name='Position', pen=pg.mkPen('#00c8ff', width=1))
        for curve in (self.target_curve, self.position_curve):
            curve.setClipToView(True)
            curve.setDownsampling(auto=True, method='peak')
        layout.addWidget(self.plot, 1)
        self.plot_timer = QtCore.QTimer(self)
        self.plot_timer.timeout.connect(self.update_plot)
        self.plot_timer.start(round(1000 / PLOT_HZ))

    def config(self):
        return MotorConfig(
            self.amplitude.value(), self.period.value(), self.kp.value(), self.kd.value(),
            round(self.clip.value()), self.deadband.value(), round(self.friction.value()), self.smoothing.value(),
        )

    def apply_config(self):
        if self.worker is not None:
            self.worker.command('config', self.config())

    def toggle_connection(self):
        if self.worker is not None:
            self.connect_button.setEnabled(False)
            self.worker.shutdown()
            return
        for values in (self.times, self.targets, self.positions):
            values.clear()
        self.plot_next = 0.0
        self.target_curve.setData([], [])
        self.position_curve.setData([], [])
        self.worker = MotorWorker(self.port.text().strip() or 'auto', self.address.value(), self.config())
        self.thread = QtCore.QThread(self)
        self.thread.setServiceLevel(QtCore.QThread.QualityOfService.High)
        self.worker.moveToThread(self.thread)
        self.thread.started.connect(self.worker.run)
        self.worker.connected.connect(self.on_connected)
        self.worker.sample.connect(self.on_sample)
        self.worker.rates.connect(self.on_rates)
        self.worker.status.connect(self.message.setText)
        self.worker.failed.connect(lambda message: self.message.setText(f'Error: {message}'))
        self.worker.finished.connect(self.thread.quit, QtCore.Qt.ConnectionType.DirectConnection)
        self.worker.finished.connect(self.worker.deleteLater)
        self.thread.finished.connect(self.on_disconnected)
        self.thread.finished.connect(self.thread.deleteLater)
        self.connect_button.setText('Connecting…')
        self.connect_button.setEnabled(False)
        self.port.setEnabled(False)
        self.address.setEnabled(False)
        self.thread.start()

    def on_connected(self, port):
        self.port.setText(port)
        self.connect_button.setText('Disconnect')
        self.connect_button.setEnabled(True)
        for button in (self.zero_button, self.start_button, self.stop_button):
            button.setEnabled(True)

    def on_disconnected(self):
        self.worker = self.thread = None
        self.connect_button.setText('Connect')
        self.connect_button.setEnabled(True)
        self.port.setEnabled(True)
        self.address.setEnabled(True)
        if not self.message.text().startswith('Error:'):
            self.message.setText('Disconnected; motor braked')
        for button in (self.zero_button, self.start_button, self.stop_button):
            button.setEnabled(False)

    def send_command(self, name):
        if self.worker is not None:
            self.worker.command('config', self.config())
            self.worker.command(name)

    def on_rates(self, sent, received):
        self.rate_label.setText(f'Commands: {sent:.0f} Hz    Feedback: {received:.0f} Hz')

    def on_sample(self, samples):
        for elapsed, target, position, velocity in samples:
            if elapsed < self.plot_next:
                continue
            self.times.append(elapsed)
            self.targets.append(target)
            self.positions.append(position)
            self.plot_next = elapsed + 1 / PLOT_SAMPLE_HZ
        _, _, position, velocity = samples[-1]
        self.values.setText(f'Position: {position:+.3f} rad    Velocity: {velocity:+.2f} rad/s')
        self.plot_dirty = True

    def update_plot(self):
        if self.times and self.plot_dirty:
            self.plot_dirty = False
            self.target_curve.setData(list(self.times), list(self.targets))
            self.position_curve.setData(list(self.times), list(self.positions))
            end = max(10, self.times[-1])
            self.plot.setXRange(end - 10, end, padding=0)

    def closeEvent(self, event):
        if self.worker is not None:
            self.worker.shutdown()
            if not self.thread.wait(3000):
                self.message.setText('Waiting for motor to stop…')
                event.ignore()
                return
        event.accept()


def main():
    parser = argparse.ArgumentParser(description='Single BLDC motor sine position control')
    parser.add_argument('--port', default='auto')
    parser.add_argument('--address', type=int, choices=range(16), default=2)
    args = parser.parse_args()
    pg.setConfigOptions(antialias=False)
    app = QtWidgets.QApplication(sys.argv[:1])
    window = MainWindow(args.port, args.address)
    window.show()
    return app.exec()


if __name__ == '__main__':
    raise SystemExit(main())
