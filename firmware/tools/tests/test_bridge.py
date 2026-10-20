"""Exercise the real bridge USB shell and binary forwarding without hardware."""

import json
import hashlib
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]

ARDUINO = r'''
#pragma once
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <map>
#include <sstream>
#include <string>
#define SERIAL_8N1 1
#define SERIAL_8E1 2
#define INPUT 0
#define OUTPUT 1
#define LOW 0
static uint32_t now_ms = 100;
static std::map<int, int> modes;
inline uint32_t millis() { return now_ms; }
inline uint32_t micros() { return ++now_ms * 1000; }
inline void delay(uint32_t n) { now_ms += n; }
inline void pinMode(int pin, int mode) { modes[pin] = mode; }
inline void digitalWrite(int, int) {}
class Stream {
public:
    std::deque<uint8_t> input;
    std::string output;
    int available() { return input.size(); }
    int peek() { return input.empty() ? -1 : input.front(); }
    int read() { if (input.empty()) return -1; int n = input.front(); input.pop_front(); return n; }
    size_t write(const uint8_t *data, size_t n) { output.append((const char *)data, n); return n; }
    template<class T> void print(T value) { std::ostringstream s; s << value; output += s.str(); }
    void print(uint8_t value) { print((unsigned)value); }
    void print(float value, int) { print(value); }
    void println() { output += '\n'; }
    template<class T> void println(T value) { print(value); println(); }
    void println(float value, int digits) { print(value, digits); println(); }
    void feed(const std::string &data) { for (uint8_t byte : data) input.push_back(byte); }
};
class HardwareSerial : public Stream {
public:
    uint32_t baud = 0, config = 0;
    int rx = -1, tx = -1;
    void end() {}
    void flush() {}
    void begin(uint32_t b, uint32_t c, int r, int t) { baud=b; config=c; rx=r; tx=t; }
};
static HardwareSerial Serial, Serial1;
'''

PREFERENCES = r'''
#pragma once
#include <cstdint>
static bool stored = false, save_failed = false;
static uint32_t saved_pins;
class Preferences {
public:
    bool begin(const char *, bool readonly) { return !save_failed && (!readonly || stored); }
    void end() {}
    uint32_t getUInt(const char *, uint32_t value) { return stored ? saved_pins : value; }
    size_t putUInt(const char *, uint32_t value) { saved_pins = value; stored = true; return 4; }
};
'''

GPIO = r'''
#pragma once
#if defined(CONFIG_IDF_TARGET_ESP32H2)
#define SOC_GPIO_PIN_COUNT 28
#define GPIO_IS_VALID_GPIO(pin) ((pin) >= 0 && (pin) < 28)
#else
#define SOC_GPIO_PIN_COUNT 49
#define GPIO_IS_VALID_GPIO(pin) ((pin) >= 0 && (pin) < 49 && ((pin) < 22 || (pin) > 25))
#endif
#define GPIO_IS_VALID_OUTPUT_GPIO(pin) GPIO_IS_VALID_GPIO(pin)
typedef int gpio_num_t;
inline void gpio_pullup_en(gpio_num_t) {}
'''

CHECK = r'''
#include <cassert>
#include "pennyesc_arduino.h"
static void send(PennyEscBridge &bridge, Stream &usb, const std::string &line) {
    now_ms += 30;
    usb.feed(line);
    while (usb.available()) bridge.poll();
}
int main() {
    Stream usb;
    HardwareSerial uart;
    PennyEscBridge bridge;
    bridge.begin(usb, uart, DEFAULT_RX, DEFAULT_TX, 1, DEFAULT_GND);
    send(bridge, usb, "pins\n");
    assert(usb.output.find("# pins board=" BOARD_ID) != std::string::npos);
    fputs(usb.output.c_str(), stdout);
    send(bridge, usb, "pins 4 5 2\n");
    assert(uart.rx == 4 && uart.tx == 5 && modes[2] == OUTPUT && stored);
    uint32_t first = saved_pins;
    for (const char *line : {"pins 4 4 2\n", "pins 4 5 4\n", "pins -1 5 2\n",
                            "pins 4 5 26\n", "pins 4 5 2x\n", "pins 4 5 2 0\n", INVALID_GROUND}) {
        usb.output.clear();
        send(bridge, usb, line);
        assert(usb.output == "# pins error=invalid\n");
        assert(uart.rx == 4 && uart.tx == 5 && saved_pins == first);
    }
    PennyEscBridge restored;
    restored.begin(usb, uart, DEFAULT_RX, DEFAULT_TX, 1, DEFAULT_GND);
    assert(uart.rx == 4 && uart.tx == 5);
    send(restored, usb, "pins 5 4 -1\n");
    assert(uart.rx == 5 && uart.tx == 4 && modes[2] == INPUT);
    save_failed = true;
    usb.output.clear();
    send(restored, usb, "pins 4 5 2\n");
    assert(usb.output == "# pins error=save\n" && uart.rx == 5);
    save_failed = false;
    send(restored, usb, "bridge rom\n");
    send(restored, usb, "!#pins 4 5 2\n");
    assert(uart.baud == 115200 && uart.config == SERIAL_8E1 && uart.rx == 4);
    send(restored, usb, "!#bridge upload\n");
    assert(uart.baud == 115200 && uart.config == SERIAL_8N1);
    uart.output.clear();
    std::string binary;
    for (int i=0; i<256; i++) binary += (char)i;
    send(restored, usb, binary);
    send(restored, usb, "!#payload\r\n");
    assert(uart.output == binary + "!#payload\r\n");
    send(restored, usb, "!#bridge handoff\n");
    assert(uart.baud == 921600 && uart.config == SERIAL_8N1);
    send(restored, usb, "!#bridge off\n");
    send(restored, usb, "pins default\n");
    assert(uart.rx == DEFAULT_RX && uart.tx == DEFAULT_TX && uart.baud == 921600);
}
'''


class BridgeTests(unittest.TestCase):
    def test_shell_pins_and_binary_upload(self):
        boards = [
            ("ARDUINO_XIAO_ESP32S3", "CONFIG_IDF_TARGET_ESP32S3", "xiao-esp32s3", 7, 44, -1, 3),
            ("ARDUINO_ADAFRUIT_QTPY_ESP32S3_N4R2", "CONFIG_IDF_TARGET_ESP32S3", "qtpy-esp32s3", 13, 12, -1, 0),
            ("TEST_H2", "CONFIG_IDF_TARGET_ESP32H2", "esp32-h2", 0, 1, 2, 9),
        ]
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            (folder / "driver").mkdir()
            for name, source in [("Arduino.h", ARDUINO), ("Preferences.h", PREFERENCES),
                                 ("driver/gpio.h", GPIO), ("check.cpp", CHECK)]:
                (folder / name).write_text(source)
            for board, chip, name, rx, tx, gnd, strap in boards:
                with self.subTest(board=name):
                    binary = folder / name
                    includes = []
                    if sys.platform == "darwin":
                        sdk = subprocess.check_output(["xcrun", "--show-sdk-path"], text=True).strip()
                        includes = ["-isystem", f"{sdk}/usr/include/c++/v1"]
                    subprocess.run(["c++", "-std=c++11", "-O1", "-fsanitize=undefined",
                                    "-fno-sanitize-recover=all", f"-D{board}", f"-D{chip}",
                                    f'-DBOARD_ID="{name}"', f"-DDEFAULT_RX={rx}", f"-DDEFAULT_TX={tx}",
                                    f"-DDEFAULT_GND={gnd}", f'-DINVALID_GROUND="pins 4 5 {strap}\\n"', *includes,
                                    "-I", str(folder), "-I", str(ROOT / "firmware/Lib"),
                                    str(folder / "check.cpp"), "-o", str(binary)], check=True)
                    output = subprocess.check_output([str(binary)], text=True)
                    line = next(line for line in output.splitlines() if line.startswith("# pins "))
                    fields = dict(field.split("=", 1) for field in line[7:].split())
                    manifest = json.loads((ROOT / "docs/firmware/manifest.json").read_text())
                    expected = next(board for board in manifest["bridges"] if board["id"] == name)
                    for field, key in [("input", "inputPins"), ("output", "outputPins"), ("ground", "groundPins")]:
                        self.assertEqual([int(pin) for pin in fields[field].split(",")], expected[key])

    def test_browser_images(self):
        folder = ROOT / "docs/firmware"
        manifest = json.loads((folder / "manifest.json").read_text())
        for entry in [manifest["pennyesc"], *[image for board in manifest["bridges"] for image in board["flash"]]]:
            data = (folder / entry["path"]).read_bytes()
            self.assertEqual(len(data), entry["bytes"])
            self.assertEqual(hashlib.sha256(data).hexdigest(), entry["sha256"])
        self.assertEqual(manifest["pennyesc"]["address"], 0x08000580)
        self.assertEqual(manifest["pennyesc"]["escAddress"], 1)
