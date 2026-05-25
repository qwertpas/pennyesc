#!/usr/bin/env python3.11
import argparse
import sys
from collections import deque

import pyqtgraph as pg
import serial
from pyqtgraph.Qt import QtCore
from serial.tools import list_ports

BAUD = 921600
ADDRS = (7, 8, 9)
WINDOW_S = 5.0


def find_port():
    ports = [
        p
        for p in list_ports.comports()
        if p.device and "bluetooth" not in p.device.lower() and "debug-console" not in p.device.lower()
    ]
    if not ports:
        raise SystemExit("No USB serial port found")

    def score(p):
        text = f"{p.device} {p.description} {p.manufacturer} {p.hwid}".lower()
        return (
            100 * (getattr(p, "vid", None) == 0x303A)
            + 60 * ("esp32" in text or "espressif" in text)
            + 20 * ("usbmodem" in text or "usbserial" in text or "com" in text)
        )

    return max(ports, key=score).device


parser = argparse.ArgumentParser()
parser.add_argument("--port")
args = parser.parse_args()
port = args.port or find_port()

ser = serial.Serial(port, BAUD, timeout=0, write_timeout=0.2)
ser.dtr = False
ser.rts = False
ser.reset_input_buffer()

app = pg.mkQApp("PennyESC 3 Motor Sweep")
win = pg.GraphicsLayoutWidget(show=True, title=f"PennyESC 3 Motor Sweep - {port}")
win.resize(1100, 700)

pos_plot = win.addPlot(title="Position")
win.nextRow()
rpm_plot = win.addPlot(title="Velocity")
for plot, label, unit in ((pos_plot, "Position", "rad"), (rpm_plot, "Velocity", "rpm")):
    plot.addLegend()
    plot.setLabel("bottom", "Time", units="s")
    plot.setLabel("left", label, units=unit)
rpm_plot.getAxis("left").enableAutoSIPrefix(False)

colors = {7: "#e76f51", 8: "#2a9d8f", 9: "#457b9d"}
data = {a: {k: deque() for k in ("t", "target", "pos", "rpm")} for a in ADDRS}
curves = {}
for address in ADDRS:
    color = colors[address]
    curves[address, "target"] = pos_plot.plot(
        name=f"{address} target",
        pen=pg.mkPen(color, width=1, style=QtCore.Qt.DashLine),
    )
    curves[address, "pos"] = pos_plot.plot(name=f"{address} pos", pen=pg.mkPen(color, width=2))
    curves[address, "rpm"] = rpm_plot.plot(name=f"{address} rpm", pen=pg.mkPen(color, width=2))

buf = bytearray()
start_ms = None


def poll():
    global start_ms
    if ser.in_waiting:
        buf.extend(ser.read(ser.in_waiting))

    changed = False
    while b"\n" in buf:
        raw, _, rest = buf.partition(b"\n")
        buf[:] = rest
        line = raw.decode("utf-8", errors="replace").strip()
        if not line or line.startswith("#"):
            continue
        try:
            ms_s, address_s, target_s, pos_s, rpm_s = line.split(",", 4)
            ms = int(ms_s)
            address = int(address_s)
            if address not in data:
                continue
            if start_ms is None:
                start_ms = ms
            data[address]["t"].append((ms - start_ms) / 1000.0)
            data[address]["target"].append(float(target_s))
            data[address]["pos"].append(float(pos_s))
            data[address]["rpm"].append(float(rpm_s))
            changed = True
        except ValueError:
            pass

    if not changed:
        return

    latest = 0.0
    for address in ADDRS:
        t = list(data[address]["t"])
        if not t:
            continue
        latest = max(latest, t[-1])
    cutoff = max(0.0, latest - WINDOW_S)
    for address in ADDRS:
        while data[address]["t"] and data[address]["t"][0] < cutoff:
            for key in ("t", "target", "pos", "rpm"):
                data[address][key].popleft()

        t = [value - cutoff for value in data[address]["t"]]
        curves[address, "target"].setData(t, list(data[address]["target"]))
        curves[address, "pos"].setData(t, list(data[address]["pos"]))
        curves[address, "rpm"].setData(t, list(data[address]["rpm"]))

    pos_plot.setXRange(0, WINDOW_S, padding=0)
    rpm_plot.setXRange(0, WINDOW_S, padding=0)


timer = QtCore.QTimer()
timer.timeout.connect(poll)
timer.start(30)
app.aboutToQuit.connect(ser.close)
sys.exit(app.exec())
