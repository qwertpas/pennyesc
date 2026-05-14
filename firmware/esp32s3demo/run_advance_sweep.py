#!/usr/bin/env python3.11
import argparse
import datetime as dt
import sys
import time
from pathlib import Path

import serial
from serial.tools import list_ports


BAUD = 921600
EXPECTED_ROWS = 418 * 200


def find_port():
    ports = list(list_ports.comports())
    usbmodem = [p.device for p in ports if "usbmodem" in p.device]
    if len(usbmodem) == 1:
        return usbmodem[0]
    if len(usbmodem) > 1:
        raise SystemExit("Multiple usbmodem ports found: " + ", ".join(usbmodem) + ". Use --port.")

    serial_ports = [p.device for p in ports if "usbserial" in p.device]
    if len(serial_ports) == 1:
        return serial_ports[0]
    if serial_ports:
        raise SystemExit("Multiple USB serial ports found: " + ", ".join(serial_ports) + ". Use --port.")

    raise SystemExit("No USB serial port found.")


def main():
    parser = argparse.ArgumentParser(description="Run the ESP32 advance sweep and save CSV data.")
    parser.add_argument("--port", help="Serial port, for example /dev/cu.usbmodem1301")
    parser.add_argument("--out-dir", default="advance_sweeps", help="Output directory")
    args = parser.parse_args()

    port = args.port or find_port()
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    out = out_dir / f"advance_transition_fine_{dt.datetime.now():%Y%m%d-%H%M%S}.csv"

    ser = serial.Serial(port, BAUD, timeout=0.05)
    rows = 0
    header_seen = False

    print(f"port {port}")
    print(f"output {out}")
    print("starting sweep; press Ctrl-C to send stop and close the port")

    try:
        time.sleep(2.0)
        ser.reset_input_buffer()
        ser.write(b"sweep\n")
        ser.flush()

        last_progress = time.monotonic()
        with out.open("w", newline="") as f:
            while True:
                raw = ser.readline()
                if not raw:
                    continue

                line = raw.decode("utf-8", "replace").strip()
                if not line:
                    continue

                if line.startswith("#"):
                    print(line, flush=True)
                    if line == "# sweep_done":
                        break
                    continue

                if line.startswith("run_id,"):
                    if not header_seen:
                        f.write(line + "\n")
                        header_seen = True
                    continue

                f.write(line + "\n")
                rows += 1

                now = time.monotonic()
                if now - last_progress >= 30:
                    print(f"rows {rows}/{EXPECTED_ROWS}", flush=True)
                    last_progress = now

    except KeyboardInterrupt:
        print("\nstopping")
        ser.write(b"stop\n")
        ser.flush()
        time.sleep(0.2)
        return 130
    finally:
        ser.close()
        print(f"saved {out} rows={rows}")
        print("port closed")

    if rows != EXPECTED_ROWS:
        print(f"warning: expected {EXPECTED_ROWS} rows", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
