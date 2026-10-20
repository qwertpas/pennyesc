"""Build and package the firmware offered by the browser flasher."""

import argparse
import hashlib
import json
import re
import shutil
import struct
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
PIO = Path.home() / ".platformio/penv/bin/pio"
OUTPUT = ROOT / "docs/firmware"
S3_PINS = [*range(19), 21, *range(35, 49)]
H2_PINS = [*range(6), *range(8, 13), 22, 24, 25]
BOARDS = [
    ("xiao_bridge", "xiao-esp32s3", "XIAO ESP32-S3", "ESP32-S3", "8MB", 7, 44, -1, [*range(1, 10), 43, 44]),
    ("bootbridge", "qtpy-esp32s3", "ESP32S3 SuperMini", "ESP32-S3", "4MB", 13, 12, -1, S3_PINS),
    ("h2_bridge", "esp32-h2", "ESP32-H2", "ESP32-H2", "4MB", 0, 1, 2, H2_PINS),
]


def copy_image(source, path, address):
    data = source.read_bytes()
    if not data:
        raise ValueError(f"Empty firmware: {source}")
    target = OUTPUT / path
    target.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(source, target)
    return {"path": path, "address": address, "bytes": len(data), "sha256": hashlib.sha256(data).hexdigest()}


def app_partition(path):
    data = path.read_bytes()
    for offset in range(0, len(data) - 31, 32):
        magic, kind, subtype, address, size, _, _ = struct.unpack_from("<HBBII16sI", data, offset)
        if magic == 0x50AA and kind == 0 and subtype == 0x10:
            return address, size
    raise ValueError(f"No OTA-0 application partition in {path}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--skip-build", action="store_true", help="Package existing build outputs")
    args = parser.parse_args()
    if not args.skip_build:
        subprocess.run([str(PIO), "run", "-d", str(ROOT / "firmware/esp32s3demo"),
                        *[value for board in BOARDS for value in ("-e", board[0])]], check=True)
        subprocess.run([str(PIO), "run", "-d", str(ROOT / "firmware/pennyesc_libopencm3"),
                        "-e", "pennyesc_uart"], check=True)

    boot_app = Path.home() / ".platformio/packages/framework-arduinoespressif32/tools/partitions/boot_app0.bin"
    bridges = []
    for env, board_id, name, chip, flash_size, rx, tx, gnd, pins in BOARDS:
        build = ROOT / "firmware/esp32s3demo/.pio/build" / env
        address, capacity = app_partition(build / "partitions.bin")
        if (build / "firmware.bin").stat().st_size > capacity:
            raise ValueError(f"Firmware exceeds partition: {board_id}")
        straps = {8, 9, 25} if chip == "ESP32-H2" else {0, 3, 45, 46}
        bridges.append({
            "id": board_id, "name": name, "chip": chip, "flashSize": flash_size,
            "rx": rx, "tx": tx, "gnd": gnd, "inputPins": pins, "outputPins": pins,
            "groundPins": [pin for pin in pins if pin not in straps],
            "flash": [
                copy_image(build / "bootloader.bin", f"{board_id}/bootloader.bin", 0),
                copy_image(build / "partitions.bin", f"{board_id}/partitions.bin", 0x8000),
                copy_image(boot_app, f"{board_id}/boot_app0.bin", 0xE000),
                copy_image(build / "firmware.bin", f"{board_id}/firmware.bin", address),
            ],
        })

    project = ROOT / "firmware/pennyesc_libopencm3"
    linker = (project / "ld/uart_app_cal.ld").read_text()
    region = re.search(r"rom \(rx\) : ORIGIN = (0x[0-9A-Fa-f]+), LENGTH = (\d+)", linker)
    if not region:
        raise ValueError("Application ROM region was not found")
    address, capacity = int(region[1], 16), int(region[2])
    app = copy_image(project / ".pio/build/pennyesc_uart/firmware.bin", "pennyesc.bin", address)
    if app["bytes"] > capacity:
        raise ValueError("pennyesc firmware exceeds the application ROM region")
    default_address = re.search(r"#define PNY_ESC_ADDRESS (\d+)", (project / "src/main.c").read_text())
    boot = (project / "src/uart_boot.c").read_text()
    app_bytes = int(re.search(r"#define PENNYESC_APP_BYTES (\d+)u", boot)[1])
    page_bytes = int(re.search(r"#define PENNYESC_PAGE_BYTES (\d+)u", boot)[1])
    app.update(name="pennyesc", escAddress=int(default_address[1]), maxBytes=app_bytes, pageBytes=page_bytes)
    manifest = {"version": 1, "bridges": bridges, "pennyesc": app}
    (OUTPUT / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(f"Packaged {len(bridges)} bridges and pennyesc in {OUTPUT}")


if __name__ == "__main__":
    main()
