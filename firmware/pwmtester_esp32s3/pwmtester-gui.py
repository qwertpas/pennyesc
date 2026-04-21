#!/usr/bin/env python3
# Requires: pip install pyqt5 pyserial

import sys

import serial
import serial.tools.list_ports
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QSlider,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)


class PwmTesterWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("PWM tester (1000–2000 µs)")
        self._ser: serial.Serial | None = None

        self._slider = QSlider(Qt.Horizontal)
        self._slider.setRange(1000, 2000)
        self._slider.setValue(1500)
        self._slider.setTickPosition(QSlider.TicksBelow)
        self._slider.setTickInterval(100)

        self._spin = QSpinBox()
        self._spin.setRange(1000, 2000)
        self._spin.setValue(1500)
        self._spin.setSuffix(" µs")
        self._spin.setKeyboardTracking(False)
        self._spin.setMinimumWidth(100)

        self._port_combo = QComboBox()
        self._port_combo.setMinimumWidth(260)
        self._refresh_ports()

        self._connect_btn = QPushButton("Connect")
        self._connect_btn.clicked.connect(self._toggle_connect)

        self._gpio13 = QCheckBox("GPIO 13 HIGH")
        self._gpio13.stateChanged.connect(self._on_gpio13)

        port_row = QHBoxLayout()
        port_row.addWidget(self._port_combo, stretch=1)
        port_row.addWidget(self._connect_btn)

        slider_row = QHBoxLayout()
        slider_row.addWidget(self._slider, stretch=1)
        slider_row.addWidget(self._spin)

        conn_box = QGroupBox("Serial")
        conn_form = QFormLayout(conn_box)
        conn_form.addRow(port_row)
        conn_form.addRow(slider_row)
        conn_form.addRow(self._gpio13)

        central = QWidget()
        layout = QVBoxLayout(central)
        layout.addWidget(conn_box)
        self.setCentralWidget(central)

        self._slider.valueChanged.connect(self._on_slider)
        self._spin.valueChanged.connect(self._on_spin)
        self._apply_pwm_value(self._slider.value(), source_slider=True)

        self._poll_timer = QTimer(self)
        self._poll_timer.timeout.connect(self._refresh_ports_silent)
        self._poll_timer.start(2000)

    def _refresh_ports(self) -> None:
        current = self._port_combo.currentData()
        self._port_combo.clear()
        for p in serial.tools.list_ports.comports():
            label = f"{p.device} — {p.description}"
            self._port_combo.addItem(label, p.device)
        if current:
            idx = self._port_combo.findData(current)
            if idx >= 0:
                self._port_combo.setCurrentIndex(idx)

    def _refresh_ports_silent(self) -> None:
        ports = [p.device for p in serial.tools.list_ports.comports()]
        listed = [
            self._port_combo.itemData(i) for i in range(self._port_combo.count())
        ]
        if ports != listed:
            self._refresh_ports()

    def _toggle_connect(self) -> None:
        if self._ser and self._ser.is_open:
            self._ser.close()
            self._ser = None
            self._connect_btn.setText("Connect")
            return
        device = self._port_combo.currentData()
        if not device:
            QMessageBox.warning(self, "Serial", "No port selected.")
            return
        try:
            self._ser = serial.Serial(device, 115200, timeout=0.1)
        except serial.SerialException as e:
            QMessageBox.critical(self, "Serial", str(e))
            self._ser = None
            return
        self._connect_btn.setText("Disconnect")
        self._send_pulse(self._spin.value())
        self._send_gpio13()

    def _send_line(self, line: str) -> None:
        if not self._ser or not self._ser.is_open:
            return
        try:
            self._ser.write((line + "\n").encode("ascii"))
        except serial.SerialException:
            self._toggle_connect()

    def _send_pulse(self, us: int) -> None:
        self._send_line(f"P{int(us)}")

    def _send_gpio13(self) -> None:
        self._send_line("D1" if self._gpio13.isChecked() else "D0")

    def _apply_pwm_value(self, value: int, *, source_slider: bool) -> None:
        if source_slider:
            self._spin.blockSignals(True)
            self._spin.setValue(value)
            self._spin.blockSignals(False)
        else:
            self._slider.blockSignals(True)
            self._slider.setValue(value)
            self._slider.blockSignals(False)
        self._send_pulse(value)

    def _on_slider(self, value: int) -> None:
        self._apply_pwm_value(value, source_slider=True)

    def _on_spin(self, value: int) -> None:
        self._apply_pwm_value(value, source_slider=False)

    def _on_gpio13(self) -> None:
        self._send_gpio13()

    def closeEvent(self, event) -> None:
        if self._ser and self._ser.is_open:
            self._ser.close()
        event.accept()


def main() -> None:
    app = QApplication(sys.argv)
    w = PwmTesterWindow()
    w.resize(480, 140)
    w.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
