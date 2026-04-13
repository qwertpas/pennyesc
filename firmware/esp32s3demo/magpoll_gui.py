#!/usr/bin/env python3
import argparse
import queue
import sys
import threading
import time
from collections import deque
from glob import glob

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


BAUD = 115200
MAX_POINTS = 1500


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


class Reader(threading.Thread):
    def __init__(self, ser: serial.Serial, data_q: queue.Queue, msg_q: queue.Queue) -> None:
        super().__init__(daemon=True)
        self.ser = ser
        self.data_q = data_q
        self.msg_q = msg_q
        self.stop_event = threading.Event()

    def run(self) -> None:
        while not self.stop_event.is_set():
            try:
                raw = self.ser.readline()
            except (serial.SerialException, OSError) as exc:
                self.msg_q.put(f"serial error: {exc}")
                return
            if not raw:
                continue

            line = raw.decode("utf-8", errors="replace").strip()
            if not line:
                continue
            if line.startswith("#"):
                self.msg_q.put(line[1:].strip())
                continue

            parts = line.split(",")
            if len(parts) != 4:
                continue

            try:
                esc, x, y, z = (int(part) for part in parts)
            except ValueError:
                continue

            self.data_q.put((time.monotonic(), esc, x, y, z))

    def stop(self) -> None:
        self.stop_event.set()


class Window:
    def __init__(self, ser: serial.Serial, port: str) -> None:
        self.ser = ser
        self.port = port
        self.data_q: queue.Queue = queue.Queue()
        self.msg_q: queue.Queue = queue.Queue()
        self.reader = Reader(ser, self.data_q, self.msg_q)
        self.reader.start()

        self.times = deque(maxlen=MAX_POINTS)
        self.x_vals = deque(maxlen=MAX_POINTS)
        self.y_vals = deque(maxlen=MAX_POINTS)
        self.z_vals = deque(maxlen=MAX_POINTS)
        self.selected_esc = 1
        self.paused = False
        self.last_status = "waiting for data"
        self.last_values = (0, 0, 0)

        self.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv)
        self.win = QtWidgets.QWidget()
        self.win.setWindowTitle("pennyesc magpoll")
        self.win.resize(1100, 700)

        layout = QtWidgets.QVBoxLayout(self.win)

        self.status = QtWidgets.QLabel()
        layout.addWidget(self.status)

        controls = QtWidgets.QHBoxLayout()
        controls.addWidget(QtWidgets.QLabel("ESC"))
        self.esc_box = QtWidgets.QComboBox()
        for esc in range(3):
            self.esc_box.addItem(str(esc), esc)
        self.esc_box.setCurrentIndex(self.selected_esc)
        self.esc_box.currentIndexChanged.connect(self.change_esc)
        controls.addWidget(self.esc_box)

        self.clear_button = QtWidgets.QPushButton("Clear")
        self.clear_button.clicked.connect(self.clear_data)
        controls.addWidget(self.clear_button)

        self.pause_button = QtWidgets.QPushButton("Pause")
        self.pause_button.clicked.connect(self.toggle_pause)
        controls.addWidget(self.pause_button)

        controls.addStretch(1)
        layout.addLayout(controls)

        self.plot = pg.PlotWidget(title="Magnetic Field")
        self.plot.showGrid(x=True, y=True, alpha=0.25)
        self.plot.setLabel("bottom", "Time (s)")
        self.plot.setLabel("left", "Raw Field")
        self.plot.addLegend()
        layout.addWidget(self.plot)

        self.x_curve = self.plot.plot(name="X", pen=pg.mkPen("#e74c3c", width=2))
        self.y_curve = self.plot.plot(name="Y", pen=pg.mkPen("#2ecc71", width=2))
        self.z_curve = self.plot.plot(name="Z", pen=pg.mkPen("#3498db", width=2))

        self.win.show()
        self.send(f"e{self.selected_esc}\n")

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.tick)
        self.timer.start(30)

    def send(self, text: str) -> None:
        try:
            self.ser.write(text.encode("utf-8"))
            self.ser.flush()
        except (serial.SerialException, OSError) as exc:
            self.last_status = f"serial error: {exc}"

    def change_esc(self) -> None:
        self.selected_esc = int(self.esc_box.currentData())
        self.clear_data()
        self.send(f"e{self.selected_esc}\n")

    def toggle_pause(self) -> None:
        self.paused = not self.paused
        self.pause_button.setText("Resume" if self.paused else "Pause")

    def clear_data(self) -> None:
        self.times.clear()
        self.x_vals.clear()
        self.y_vals.clear()
        self.z_vals.clear()

    def append_sample(self, stamp: float, x: int, y: int, z: int) -> None:
        self.times.append(stamp)
        self.x_vals.append(x)
        self.y_vals.append(y)
        self.z_vals.append(z)
        self.last_values = (x, y, z)

    def update_plot(self) -> None:
        if not self.times:
            return
        t0 = self.times[0]
        xs = [t - t0 for t in self.times]
        self.x_curve.setData(xs, list(self.x_vals))
        self.y_curve.setData(xs, list(self.y_vals))
        self.z_curve.setData(xs, list(self.z_vals))

    def tick(self) -> None:
        while not self.msg_q.empty():
            self.last_status = self.msg_q.get_nowait()

        while not self.data_q.empty():
            stamp, esc, x, y, z = self.data_q.get_nowait()
            if esc != self.selected_esc:
                continue
            if not self.paused:
                self.append_sample(stamp, x, y, z)

        if not self.paused:
            self.update_plot()

        x, y, z = self.last_values
        self.status.setText(
            f"port={self.port} | esc={self.selected_esc} | X={x} Y={y} Z={z} | "
            f"{'paused | ' if self.paused else ''}{self.last_status}"
        )

    def close(self) -> None:
        self.reader.stop()
        self.reader.join(timeout=1.0)
        self.ser.close()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port")
    parser.add_argument("--baud", type=int, default=BAUD)
    args = parser.parse_args()

    port = find_serial_port(args.port)
    try:
        ser = serial.Serial(port, args.baud, timeout=0.2)
    except serial.SerialException as exc:
        raise SystemExit(str(exc))

    ser.reset_input_buffer()
    ser.reset_output_buffer()
    window = Window(ser, port)
    app = window.app
    app.aboutToQuit.connect(window.close)
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
