"""Compile the real bridge firmware against Arduino stand-ins and exercise its packet protocol."""

import hashlib
import json
import os
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
#include <cstring>
#include <deque>
#include <map>
#include <string>
#include <vector>
#define SERIAL_8N1 1
#define INPUT 0
#define OUTPUT 1
#define LOW 0
#define ARDUINO_BOARD "test"
static uint32_t now_us = 1000000;
static std::map<int, int> modes;
inline uint32_t millis() { return now_us / 1000; }
inline uint32_t micros() { return now_us; }
inline void delay(uint32_t ms) { now_us += ms * 1000; }
inline void pinMode(int pin, int mode) { modes[pin] = mode; }
inline void digitalWrite(int, int) {}
class Stream {
public:
    std::deque<uint8_t> input;
    std::string output;
    virtual ~Stream() {}
    virtual int available() { return input.size(); }
    virtual int read() { if (input.empty()) return -1; int n = input.front(); input.pop_front(); return n; }
    virtual void flush() {}
    virtual size_t write(uint8_t byte) { output += (char)byte; return 1; }
    virtual size_t write(const uint8_t *data, size_t n) { output.append((const char *)data, n); return n; }
    void feed(const std::string &data) { for (uint8_t byte : data) input.push_back(byte); }
};
class HardwareSerial : public Stream {
public:
    uint32_t baud = 0, config = 0, begins = 0;
    int rx = -1, tx = -1;
    void end() {}
    void setRxBufferSize(size_t) {}
    void begin(uint32_t b, uint32_t c, int r, int t) { baud = b; config = c; rx = r; tx = t; begins++; }
};
static HardwareSerial Serial, Serial1;
typedef void *TaskHandle_t;
inline int xTaskCreatePinnedToCore(void (*)(void *), const char *, int, void *, int, TaskHandle_t *, int) { return 1; }
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
inline int ledc_timer_config(ledc_timer_config_t *t) { pwm_hz = t->freq_hz; return ESP_OK; }
inline int ledc_channel_config(ledc_channel_config_t *c) { pwm_pin = c->gpio_num; pwm_duty = c->duty; return ESP_OK; }
inline int ledc_set_duty(int, int, uint32_t duty) { pwm_duty = duty; return ESP_OK; }
inline int ledc_update_duty(int, int) { return ESP_OK; }
inline int ledc_stop(int, int, int) { pwm_pin = -1; pwm_duty = 0; return ESP_OK; }
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
static std::vector<std::string> notified;
class BLECharacteristic {
public:
    enum { PROPERTY_NOTIFY=1, PROPERTY_WRITE=2, PROPERTY_WRITE_NR=4 };
    BLECharacteristicCallbacks *callbacks{};
    std::string data, value;
    size_t getLength() { return data.size(); }
    uint8_t *getData() { return (uint8_t *)data.data(); }
    void setCallbacks(BLECharacteristicCallbacks *callback) { callbacks = callback; }
    void setValue(uint8_t *bytes, size_t length) { value.assign((const char *)bytes, length); }
    void notify() { notified.push_back(value); }
};
static BLECharacteristic input, reply;
class BLEService {
public:
    BLECharacteristic *createCharacteristic(const char *, int properties) {
        return (properties & BLECharacteristic::PROPERTY_NOTIFY) ? &reply : &input;
    }
    void start() {}
};
class BLEAdvertisementData {
public: void setFlags(int) {} void setCompleteServices(const BLEUUID &) {} void setName(const char *) {}
};
class BLEAdvertising {
public: void setAdvertisementData(BLEAdvertisementData &) {}
};
static int ble_kicks = 0;
class BLEServer {
public:
    BLEServerCallbacks *callbacks{};
    void setCallbacks(BLEServerCallbacks *callback) { callbacks = callback; }
    BLEService *createService(const char *) { static BLEService service; return &service; }
    void disconnect(int) { ble_kicks++; } int getConnId() { return 0; }
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

# Helpers shared by the C++ checks.
HELPERS = r'''
#include <cassert>
#include <cstdio>
typedef std::vector<uint8_t> Bytes;
static Bytes packet(uint8_t type, Bytes payload = {}) {
    Bytes data{type};
    data.insert(data.end(), payload.begin(), payload.end());
    data.push_back(PennyBridge::crc8(data.data(), data.size()));
    return data;
}
static std::string usb(uint8_t type, Bytes payload = {}) {
    Bytes data = packet(type, payload), encoded(data.size() + 4);
    size_t count = PennyBridge::cobsEncode(data.data(), data.size(), encoded.data());
    return std::string(1, '\0') + std::string((const char *)encoded.data(), count) + std::string(1, '\0');
}
// Packets the bridge sent on USB, decoded and with their CRC checked.
static std::vector<Bytes> sent(Stream &stream) {
    std::vector<Bytes> packets;
    std::string chunk;
    for (char c : stream.output) {
        if (c != 0) { chunk += c; continue; }
        if (chunk.empty()) continue;
        Bytes decoded(chunk.size());
        int length = PennyBridge::cobsDecode((const uint8_t *)chunk.data(), chunk.size(), decoded.data());
        assert(length >= 2);
        decoded.resize(length);
        assert(PennyBridge::crc8(decoded.data(), length - 1) == decoded[length - 1]);
        decoded.pop_back();
        packets.push_back(decoded);
        chunk.clear();
    }
    stream.output.clear();
    return packets;
}
static Bytes frame(uint8_t address, uint8_t command, Bytes payload = {}) {
    Bytes data{0xAA, (uint8_t)(address << 4 | command), (uint8_t)payload.size()};
    data.insert(data.end(), payload.begin(), payload.end());
    data.push_back(PennyEsc::crc8(data.data(), data.size()));
    return data;
}
static std::string text(const Bytes &bytes) { return std::string(bytes.begin(), bytes.end()); }
'''

CHECK = r'''
#include "pennyesc_bridge.h"
''' + HELPERS + r'''
struct TestBridge : PennyBridge {
    using PennyBridge::validPin;
    using PennyBridge::validGround;
};
static Bytes command(TestBridge &bridge, Stream &host, uint8_t type, Bytes payload = {}) {
    host.feed(usb(type, payload));
    bridge.poll();
    std::vector<Bytes> packets = sent(host);
    assert(packets.size() == 1 && packets[0][0] == PENNY_PKT_STATE && packets[0][2] == PENNY_BRIDGE_VERSION);
    return Bytes(packets[0].begin() + 1, packets[0].end());
}
int main() {
    for (int kind = 0; kind < 3; kind++) {
        printf(kind == 0 ? "input=" : kind == 1 ? " output=" : " ground=");
        bool comma = false;
        for (int pin = 0; pin < SOC_GPIO_PIN_COUNT; pin++) {
            if (kind == 2 ? TestBridge::validGround(pin) : TestBridge::validPin(pin, kind != 0)) {
                printf(comma ? ",%d" : "%d", pin);
                comma = true;
            }
        }
    }
    printf("\n");

    Stream host;
    HardwareSerial uart;
    TestBridge bridge;
    bridge.begin(host, uart, DEFAULT_RX, DEFAULT_TX, DEFAULT_GND);
    assert(uart.baud == 921600 && uart.rx == DEFAULT_RX);

    // HELLO reports version, pins, aux and board; text noise before it is ignored.
    host.feed("err\npong\n");
    Bytes state = command(bridge, host, PENNY_PKT_HELLO);
    assert(state[0] == 0 && (int8_t)state[2] == DEFAULT_RX && (int8_t)state[3] == DEFAULT_TX && (int8_t)state[4] == DEFAULT_GND);
    assert(text(Bytes(state.begin() + 9, state.end())) == BOARD_ID);

    // Aux PWM: 200 Hz, pulse in microseconds, 0 turns it off; bad values change nothing.
    state = command(bridge, host, PENNY_PKT_AUX, {3, 0xE8, 0x03});
    assert(state[0] == 0 && pwm_pin == 3 && pwm_hz == 200 && pwm_duty == 3276 && state[8] == 1);
    command(bridge, host, PENNY_PKT_AUX, {3, 0xD0, 0x07});
    assert(pwm_duty == 6553);
    for (Bytes bad : {Bytes{3, 0xE7, 0x03}, Bytes{3, 0xD1, 0x07}, Bytes{26, 0xDC, 0x05}, Bytes{(uint8_t)DEFAULT_RX, 0xDC, 0x05}, Bytes{3, 1}}) {
        assert(command(bridge, host, PENNY_PKT_AUX, bad)[0] == PENNY_BRIDGE_INVALID);
        assert(pwm_pin == 3 && pwm_duty == 6553);
    }
    command(bridge, host, PENNY_PKT_AUX, {4, 0xDC, 0x05});
    assert(pwm_pin == 4 && modes[3] == INPUT && pwm_duty == 4915);
    state = command(bridge, host, PENNY_PKT_AUX, {4, 0, 0});
    assert(pwm_pin == -1 && modes[4] == INPUT && state[8] == 0);

    // Pins are validated, saved, and applied to the UART.
    state = command(bridge, host, PENNY_PKT_PINS, {4, 5, 2});
    assert(state[0] == 0 && uart.rx == 4 && uart.tx == 5 && modes[2] == OUTPUT && stored);
    uint32_t first = saved_pins;
    for (Bytes bad : {Bytes{4, 4, 2}, Bytes{4, 5, 4}, Bytes{0xFF, 5, 2}, Bytes{4, 5, 26}, Bytes{4, 5, INVALID_GROUND}, Bytes{4, 5}}) {
        assert(command(bridge, host, PENNY_PKT_PINS, bad)[0] == PENNY_BRIDGE_INVALID);
        assert(uart.rx == 4 && uart.tx == 5 && saved_pins == first);
    }
    TestBridge restored;
    restored.begin(host, uart, DEFAULT_RX, DEFAULT_TX, DEFAULT_GND);
    assert(uart.rx == 4 && uart.tx == 5);
    save_failed = true;
    assert(command(restored, host, PENNY_PKT_PINS, {5, 4, 0xFF})[0] == PENNY_BRIDGE_SAVE_FAILED);
    save_failed = false;
    state = command(restored, host, PENNY_PKT_PINS, {5, 4, 0xFF});
    assert(state[0] == 0 && uart.rx == 5 && modes[2] == INPUT);
    assert(command(restored, host, 0x55)[0] == PENNY_BRIDGE_UNKNOWN);

    // UART packets carry their baud; every byte value passes through unchanged.
    Bytes status = frame(1, 1);
    host.feed(usb(PENNY_PKT_UART, Bytes{PENNY_BAUD_APP}) + usb(PENNY_PKT_UART, [&] { Bytes b{PENNY_BAUD_APP}; b.insert(b.end(), status.begin(), status.end()); return b; }()));
    uart.output.clear();
    restored.poll();
    assert(uart.output == text(status) && uart.baud == 921600 && pennyesc_bridge_active);
    Bytes binary{PENNY_BAUD_BOOT};
    for (int i = 0; i < 200; i++) binary.push_back(i);
    for (char c : std::string("!#bridge off\n")) binary.push_back(c);
    uart.output.clear();
    host.feed(usb(PENNY_PKT_UART, binary));
    restored.poll();
    assert(uart.baud == 115200 && uart.output == text(Bytes(binary.begin() + 1, binary.end())));
    uart.feed(std::string(1, '\x79'));
    restored.poll();
    std::vector<Bytes> packets = sent(host);
    assert(packets.size() == 1 && packets[0] == (Bytes{PENNY_PKT_UART_RX, PENNY_BAUD_BOOT, 0x79}));

    // A corrupt packet is dropped and the next one still works.
    std::string corrupt = usb(PENNY_PKT_HELLO);
    corrupt[2] ^= 1;
    host.feed(corrupt);
    restored.poll();
    assert(sent(host).empty());
    command(restored, host, PENNY_PKT_HELLO);

    // Streaming: the bridge polls status itself, stamps replies, and forwards other frames.
    uart.output.clear();
    state = command(restored, host, PENNY_PKT_STREAM, {2, 200});
    assert(state[0] == 0 && uart.baud == 921600);
    assert(uart.output == text(frame(2, 1)));
    uart.output.clear();
    now_us += 5000;
    restored.poll();
    assert(uart.output == text(frame(2, 1)));
    Bytes reply = frame(2, 1, Bytes(60, 7));
    Bytes duty = frame(2, 3, Bytes(60, 9));
    Bytes host_status = frame(2, 1, Bytes(60, 8));
    // The first status reply answers the bridge's poll; a second one answers the host.
    uart.feed(text(reply) + text(duty) + text(host_status));
    restored.poll();
    packets = sent(host);
    assert(packets.size() == 2 && packets[0][0] == PENNY_PKT_UART_RX && text(Bytes(packets[0].begin() + 2, packets[0].end())) == text(duty));
    assert(packets[1][0] == PENNY_PKT_UART_RX && text(Bytes(packets[1].begin() + 2, packets[1].end())) == text(host_status));
    now_us += 11000;
    restored.poll();
    packets = sent(host);
    assert(packets.size() == 1 && packets[0][0] == PENNY_PKT_TELEMETRY && packets[0].size() == 1 + 4 + reply.size());
    assert(text(Bytes(packets[0].begin() + 5, packets[0].end())) == text(reply));
    uart.output.clear();
    command(restored, host, PENNY_PKT_STREAM, {0, 0});
    now_us += 20000;
    restored.poll();
    assert(uart.output.empty());
    // HELLO from a fresh host also stops a stream left running by a closed page.
    command(restored, host, PENNY_PKT_STREAM, {2, 100});
    command(restored, host, PENNY_PKT_HELLO);
    uart.output.clear();
    now_us += 20000;
    restored.poll();
    assert(uart.output.empty());
    for (uint8_t bad : {16, 255}) assert(command(restored, host, PENNY_PKT_STREAM, {bad, 10})[0] == PENNY_BRIDGE_INVALID);
}
'''

BLE_CHECK = r'''
#include "pennyesc_ble.h"
''' + HELPERS + r'''
static void ble_write(PennyBleBridge &bridge, uint8_t type, Bytes payload = {}) {
    input.data = text(packet(type, payload));
    input.callbacks->onWrite(&input);
    bridge.poll();
}
int main() {
    PennyBleBridge bridge;
    bridge.begin(Serial, Serial1, 12, 13);
    bridge.beginBle();
    server.callbacks->onConnect(&server, nullptr);
    ble_gap_conn_desc desc{1};
    server.callbacks->onMtuChanged(&server, &desc, 247);

    // BLE packets need no COBS; replies come back as notifications.
    ble_write(bridge, PENNY_PKT_HELLO);
    assert(notified.size() == 1 && (uint8_t)notified[0][0] == PENNY_PKT_STATE && notified[0][1] == 0);
    Bytes duty = frame(3, 3, {100, 0});
    Bytes payload{PENNY_BAUD_APP};
    payload.insert(payload.end(), duty.begin(), duty.end());
    ble_write(bridge, PENNY_PKT_UART, payload);
    assert(Serial1.output == text(duty));
    Serial1.feed(text(frame(3, 3, Bytes(60, 1))));
    bridge.poll();
    assert(notified.size() == 2 && (uint8_t)notified[1][0] == PENNY_PKT_UART_RX);

    // USB takes priority: BLE is disconnected and refused for a second.
    Serial1.output.clear();
    Serial.feed(usb(PENNY_PKT_HELLO));
    bridge.poll();
    assert(ble_kicks == 1 && sent(Serial).size() == 1);
    // Handing the UART to USB stops whatever BLE was driving.
    assert(Serial1.output == text(frame(3, 15, {0, 0})));
    notified.clear();
    ble_write(bridge, PENNY_PKT_HELLO);
    assert(notified.size() == 1 && notified[0][1] == PENNY_BRIDGE_BUSY);
    now_us += 1100000;
    ble_write(bridge, PENNY_PKT_AUX, {11, 0xDC, 0x05});
    assert(notified.size() == 2 && notified[1][1] == 0 && pwm_pin == 11);

    // Losing BLE stops its outputs.
    ble_write(bridge, PENNY_PKT_UART, payload);
    Serial1.output.clear();
    server.callbacks->onDisconnect(&server);
    bridge.poll();
    assert(Serial1.output == text(frame(3, 15, {0, 0})) && pwm_pin == -1);
}
'''


def compile_and_run(folder: Path, name: str, source: str, defines: list[str]) -> str:
    (folder / f"{name}.cpp").write_text(source)
    includes = []
    if sys.platform == "darwin":
        sdk = subprocess.check_output(["xcrun", "--show-sdk-path"], text=True).strip()
        includes = ["-isystem", f"{sdk}/usr/include/c++/v1"]
    binary = folder / name
    subprocess.run(["c++", "-std=c++14", "-O1", "-fsanitize=address,undefined", "-fno-sanitize-recover=all",
                    *defines, *includes, "-I", str(folder), "-I", str(ROOT / "firmware/Lib"),
                    str(folder / f"{name}.cpp"), "-o", str(binary)], check=True)
    # The firmware allocates its queue once for its lifetime, which LeakSanitizer reports on Linux.
    return subprocess.check_output([str(binary)], text=True, env={**os.environ, "ASAN_OPTIONS": "detect_leaks=0"})


def stand_ins(folder: Path) -> None:
    for sub in ("driver", "host"):
        (folder / sub).mkdir(exist_ok=True)
    for name, source in {"Arduino.h": ARDUINO, "Preferences.h": PREFERENCES, "driver/gpio.h": GPIO,
                         "driver/ledc.h": LEDC, "BLEDevice.h": BLE, "BLEServer.h": '#include "BLEDevice.h"',
                         "host/ble_hs_adv.h": '#include "BLEDevice.h"'}.items():
        (folder / name).write_text(source)


class BridgeTests(unittest.TestCase):
    def test_packet_protocol_pins_aux_uart_and_stream(self):
        boards = [
            ("ARDUINO_XIAO_ESP32S3", "CONFIG_IDF_TARGET_ESP32S3", "xiao-esp32s3", 7, 44, -1, 3),
            ("ARDUINO_ADAFRUIT_QTPY_ESP32S3_N4R2", "CONFIG_IDF_TARGET_ESP32S3", "qtpy-esp32s3", 12, 13, -1, 0),
            ("TEST_H2", "CONFIG_IDF_TARGET_ESP32H2", "esp32-h2", 0, 1, 2, 9),
        ]
        manifest = json.loads((ROOT / "docs/firmware/manifest.json").read_text())
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            stand_ins(folder)
            for board, chip, name, rx, tx, gnd, strap in boards:
                with self.subTest(board=name):
                    output = compile_and_run(folder, name, CHECK, [
                        f"-D{board}", f"-D{chip}", f'-DBOARD_ID="{name}"', f"-DDEFAULT_RX={rx}",
                        f"-DDEFAULT_TX={tx}", f"-DDEFAULT_GND={gnd}", f"-DINVALID_GROUND={strap}"])
                    fields = dict(field.split("=", 1) for field in output.split())
                    expected = next(item for item in manifest["bridges"] if item["id"] == name)
                    for field, key in [("input", "inputPins"), ("output", "outputPins"), ("ground", "groundPins")]:
                        self.assertEqual([int(pin) for pin in fields[field].split(",")], expected[key])

    def test_ble_link_and_usb_priority(self):
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            stand_ins(folder)
            compile_and_run(folder, "ble", BLE_CHECK, ["-DCONFIG_IDF_TARGET_ESP32S3"])

    def test_browser_images(self):
        folder = ROOT / "docs/firmware"
        manifest = json.loads((folder / "manifest.json").read_text())
        for entry in [manifest["pennyesc"], *[image for board in manifest["bridges"] for image in board["flash"]]]:
            data = (folder / entry["path"]).read_bytes()
            self.assertEqual(len(data), entry["bytes"])
            self.assertEqual(hashlib.sha256(data).hexdigest(), entry["sha256"])
        self.assertEqual(manifest["pennyesc"]["address"], 0x08000580)
        self.assertEqual(manifest["pennyesc"]["escAddress"], 1)


if __name__ == "__main__":
    unittest.main()
