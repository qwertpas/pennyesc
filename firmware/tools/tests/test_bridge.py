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
#include <algorithm>
#include <vector>
#include <cstring>
using std::min;
#define SERIAL_8N1 1
#define SERIAL_8E1 2
#define INPUT 0
#define OUTPUT 1
#define LOW 0
static uint32_t now_ms = 100;
static bool usb_plugged = true;
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
    virtual int available() { return input.size(); }
    virtual int peek() { return input.empty() ? -1 : input.front(); }
    virtual int read() { if (input.empty()) return -1; int n = input.front(); input.pop_front(); return n; }
    virtual void flush() {}
    virtual size_t write(uint8_t byte) { output += (char)byte; return 1; }
    virtual size_t write(const uint8_t *data, size_t n) { output.append((const char *)data, n); return n; }
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
    bool isPlugged() { return usb_plugged; }
    void end() {}
    void flush() {}
    void begin(uint32_t b, uint32_t c, int r, int t) { baud=b; config=c; rx=r; tx=t; }
};
static HardwareSerial Serial, Serial1;
struct Queue { size_t size; std::deque<std::vector<uint8_t>> items; };
using QueueHandle_t = Queue *;
static const int pdTRUE = 1;
inline QueueHandle_t xQueueCreate(int, size_t size) { return new Queue{size, {}}; }
inline void xQueueReset(QueueHandle_t queue) { queue->items.clear(); }
inline int xQueueSend(QueueHandle_t queue, const void *data, int) {
    const auto *bytes = (const uint8_t *)data;
    queue->items.emplace_back(bytes, bytes + queue->size); return pdTRUE;
}
inline int xQueueReceive(QueueHandle_t queue, void *data, int) {
    if (queue->items.empty()) return 0;
    memcpy(data, queue->items.front().data(), queue->size); queue->items.pop_front(); return pdTRUE;
}
'''

BLE = r'''
#pragma once
#include <Arduino.h>
#define BLE_GAP_LE_PHY_2M_MASK 2
#define BLE_HS_ADV_F_DISC_GEN 2
#define BLE_HS_ADV_F_BREDR_UNSUP 4
struct ble_gap_conn_desc { int conn_handle; };
struct ble_uuid_any_t { int u128; };
struct ble_hs_adv_fields {
    int flags, name_len, name_is_complete, num_uuids128, uuids128_is_complete;
    const uint8_t *name; const int *uuids128;
};
inline void ble_gap_set_prefered_default_le_phy(int, int) {}
inline void ble_gap_set_data_len(int, int, int) {}
inline void ble_gap_adv_set_fields(ble_hs_adv_fields *) {}
class BLEUUID {
    ble_uuid_any_t uuid{};
public: BLEUUID(const char *) {} const ble_uuid_any_t *getNative() const { return &uuid; }
};
class BLEServer;
class BLECharacteristic;
class BLEServerCallbacks {
public:
    virtual void onConnect(BLEServer *, ble_gap_conn_desc *) {}
    virtual void onDisconnect(BLEServer *) {}
    virtual void onMtuChanged(BLEServer *, ble_gap_conn_desc *, uint16_t) {}
};
class BLECharacteristicCallbacks {
public: virtual void onWrite(BLECharacteristic *) {}
};
class BLECharacteristic {
public:
    enum { PROPERTY_NOTIFY=1, PROPERTY_WRITE=2, PROPERTY_WRITE_NR=4 };
    BLECharacteristicCallbacks *callbacks{};
    std::string data;
    size_t getLength() { return data.size(); }
    uint8_t *getData() { return (uint8_t *)data.data(); }
    void setCallbacks(BLECharacteristicCallbacks *value) { callbacks=value; }
    void setValue(const uint8_t *, size_t) {}
    void notify() {}
};
static BLECharacteristic input;
class BLEService {
public:
    BLECharacteristic *createCharacteristic(const char *, int) { return &input; }
    void start() {}
};
class BLEAdvertisementData {
public: void setFlags(int) {} void setCompleteServices(const BLEUUID &) {} void setName(const char *) {}
};
class BLEAdvertising {
public: void setAdvertisementData(BLEAdvertisementData &) {}
};
class BLEServer {
public:
    BLEServerCallbacks *callbacks{};
    void setCallbacks(BLEServerCallbacks *value) { callbacks=value; }
    BLEService *createService(const char *) { static BLEService service; return &service; }
    void disconnect(int) {} int getConnId() { return 0; }
    void startAdvertising() {} void updateConnParams(int, int, int, int, int) {}
};
static BLEServer server;
class BLEDevice {
public:
    static void init(const char *) {} static void setMTU(int) {} static void startAdvertising() {}
    static BLEServer *createServer() { return &server; }
    static BLEAdvertising *getAdvertising() { static BLEAdvertising advertising; return &advertising; }
};
'''

USB_CHECK = r'''
#include <cassert>
#define ARDUINO_BOARD "test"
#include "pennyesc_ble.h"
static void send(PennyEscBleBridge &bridge, const std::string &bytes) {
    now_ms += 30; Serial.feed(bytes); bridge.poll();
}
int main() {
    PennyEscBleBridge bridge;
    bridge.begin(Serial, Serial1, 12, 13);
    bridge.beginBle();
    send(bridge, "bridge app\n");
    assert(Serial.output.find("# bridge=app") != std::string::npos);
    Serial.output.clear(); Serial1.output.clear();

    // A command is in flight when the USB link monitor briefly reports suspend.
    const std::string command("\xaa\x24\x01\x02\x90", 5);
    send(bridge, command);
    assert(Serial1.output == command);
    usb_plugged = false;
    bridge.poll();
    // The UART reply arrives later, with no further host writes to wake the bridge.
    const std::string reply("\xaa\x24\x04\x00\x01\x04\x24\x02", 8);
    Serial1.feed(reply); bridge.poll();
    assert(Serial.output == reply);
    assert(Serial1.baud == 921600 && pennyesc_bridge_active);

    usb_plugged = true;
    Serial1.output.clear();
    send(bridge, command);
    assert(Serial1.output == command);
    // Firmware upload retains its baud and receives no application stop frame.
    send(bridge, "!#bridge upload\n");
    Serial1.output.clear(); usb_plugged = false;
    bridge.poll();
    assert(Serial1.baud == 115200 && Serial1.output.empty());
    Serial.output.clear(); Serial1.feed(std::string(1, '\x79')); bridge.poll();
    assert(Serial.output == std::string(1, '\x79'));

    // A real BLE takeover resets UART mode explicitly, before its first command.
    input.data = std::string("\xaa\x2f\x02\x00\x00", 5);
    input.data += (char)PennyEsc::crc8((const uint8_t *)input.data.data(), input.data.size());
    server.callbacks->onConnect(&server, nullptr);
    input.callbacks->onWrite(&input); bridge.poll();
    assert(Serial1.baud == 921600 && !pennyesc_bridge_active);
    assert(Serial1.output == input.data);
}
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

LEDC = r'''
#pragma once
#define ESP_OK 0
#define LEDC_LOW_SPEED_MODE 0
#define LEDC_TIMER_14_BIT 14
#define LEDC_TIMER_0 0
#define LEDC_CHANNEL_0 0
#define LEDC_AUTO_CLK 0
struct ledc_timer_config_t { int speed_mode, duty_resolution, timer_num, freq_hz, clk_cfg; };
struct ledc_channel_config_t { int gpio_num, speed_mode, channel, timer_sel; uint32_t duty; };
static int pwm_pin = -1, pwm_hz = 0;
static uint32_t pwm_duty = 0;
inline int ledc_timer_config(ledc_timer_config_t *t) {
    if (t->duty_resolution != 14) return -1;
    pwm_hz = t->freq_hz; return ESP_OK;
}
inline int ledc_channel_config(ledc_channel_config_t *c) {
    pwm_pin = c->gpio_num; pwm_duty = c->duty; return ESP_OK;
}
inline int ledc_set_duty(int, int, uint32_t duty) { pwm_duty = duty; return ESP_OK; }
inline int ledc_update_duty(int, int) { return ESP_OK; }
inline int ledc_stop(int, int, int) { pwm_pin = -1; pwm_duty = 0; return ESP_OK; }
inline uint32_t ledc_get_freq(int, int) { return pwm_hz; }
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
    usb.output.clear();
    send(bridge, usb, "aux\n");
    assert(usb.output == "# aux pin=11 pulse=1500 hz=200 enabled=0\n");
    send(bridge, usb, "aux 3 1000\n");
    assert(pwm_pin == 3 && pwm_hz == 200 && pwm_duty == 3276);
    send(bridge, usb, "aux 3 2000\n");
    assert(pwm_pin == 3 && pwm_duty == 6553);
    for (const char *line : {"aux 3 999\n", "aux 3 2001\n", "aux 26 1500\n",
                            "aux 3 1000x\n", "aux 3 1000 1\n", "pins 3 5 2\n"}) {
        usb.output.clear();
        send(bridge, usb, line);
        assert(usb.output.find("error=invalid") != std::string::npos);
        assert(pwm_pin == 3 && pwm_duty == 6553 && !stored);
    }
    send(bridge, usb, "aux " + std::to_string(DEFAULT_RX) + " 1500\n");
    assert(pwm_pin == 3 && pwm_duty == 6553);
    send(bridge, usb, "aux " + std::to_string(DEFAULT_TX) + " 1500\n");
    assert(pwm_pin == 3 && pwm_duty == 6553);
    send(bridge, usb, "aux 4 1500\n");
    assert(pwm_pin == 4 && modes[3] == INPUT && pwm_duty == 4915);
    send(bridge, usb, "aux off\n");
    assert(pwm_pin == -1 && modes[4] == INPUT);
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
    uart.output.clear();
    send(restored, usb, "!#aux 3 1125\n");
    assert(pwm_pin == 3 && pwm_duty == 3686 && uart.output.empty());
    send(restored, usb, "!#aux off\n");
    assert(pwm_pin == -1 && uart.output.empty());
    send(restored, usb, "!#bridge off\n");
    send(restored, usb, "pins default\n");
    assert(uart.rx == DEFAULT_RX && uart.tx == DEFAULT_TX && uart.baud == 921600);
}
'''


class BridgeTests(unittest.TestCase):
    def test_usb_suspend_preserves_uart_session_until_ble_takeover(self):
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            (folder / "driver").mkdir()
            (folder / "host").mkdir()
            files = {"Arduino.h": ARDUINO, "Preferences.h": PREFERENCES,
                     "driver/gpio.h": GPIO, "driver/ledc.h": LEDC,
                     "BLEDevice.h": BLE, "BLEServer.h": '#include "BLEDevice.h"',
                     "host/ble_hs_adv.h": '#include "BLEDevice.h"', "check.cpp": USB_CHECK}
            for name, source in files.items():
                (folder / name).write_text(source)
            includes = []
            if sys.platform == "darwin":
                sdk = subprocess.check_output(["xcrun", "--show-sdk-path"], text=True).strip()
                includes = ["-isystem", f"{sdk}/usr/include/c++/v1"]
            binary = folder / "check"
            subprocess.run(["c++", "-std=c++11", "-O1", "-fsanitize=undefined",
                            "-fno-sanitize-recover=all", *includes, "-I", str(folder),
                            "-I", str(ROOT / "firmware/Lib"), str(folder / "check.cpp"),
                            "-o", str(binary)], check=True)
            subprocess.run([str(binary)], check=True)

    def test_shell_pins_and_binary_upload(self):
        boards = [
            ("ARDUINO_XIAO_ESP32S3", "CONFIG_IDF_TARGET_ESP32S3", "xiao-esp32s3", 7, 44, -1, 3),
            ("ARDUINO_ADAFRUIT_QTPY_ESP32S3_N4R2", "CONFIG_IDF_TARGET_ESP32S3", "qtpy-esp32s3", 12, 13, -1, 0),
            ("TEST_H2", "CONFIG_IDF_TARGET_ESP32H2", "esp32-h2", 0, 1, 2, 9),
        ]
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            (folder / "driver").mkdir()
            for name, source in [("Arduino.h", ARDUINO), ("Preferences.h", PREFERENCES),
                                 ("driver/gpio.h", GPIO), ("driver/ledc.h", LEDC), ("check.cpp", CHECK)]:
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
