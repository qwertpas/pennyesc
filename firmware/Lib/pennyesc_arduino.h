#ifndef PENNYESC_ARDUINO_H
#define PENNYESC_ARDUINO_H

#include <Arduino.h>
#include <string.h>
#include "pennyesc_protocol.h"

static const uint32_t PENNYESC_BAUD_UPDATE = 115200u;
static const uint32_t PENNYESC_BAUD_FAST = 921600u;
static const uint32_t PENNYESC_ROM_BAUD = 115200u;

struct PennyEscPins {
    int rx = 12;
    int tx = 13;
    int gnd = 11;
    int pullup = 9;
};

struct PennyEscStatus {
    bool valid = false;
    uint8_t result = 0;
    uint8_t mode = 0;
    uint8_t flags = 0;
    uint8_t faults = 0;
    int16_t x = 0;
    int16_t y = 0;
    int16_t z = 0;
    uint16_t angle_turn16 = 0;
    int32_t position_crad = 0;
    int32_t velocity_crads = 0;
    int16_t duty = 0;

    bool sensorOk() const { return (flags & PNY_FLAG_SENSOR_OK) != 0u; }
    bool calValid() const { return (flags & PNY_FLAG_CAL_VALID) != 0u; }
    bool busy() const { return (flags & PNY_FLAG_BUSY) != 0u; }
    bool positionReached() const { return (flags & PNY_FLAG_POSITION_REACHED) != 0u; }
    bool hasFault() const { return (flags & PNY_FLAG_FAULT) != 0u; }
    float angleRad() const { return ((float)angle_turn16 * 6.2831853f) / 65536.0f; }
    float positionRad() const { return (float)position_crad * 0.01f; }
    float velocityRadS() const { return (float)velocity_crads * 0.01f; }
    float velocityRpm() const { return velocityRadS() * 9.5492966f; }
};

struct PennyEscEncoderData {
    bool valid = false;
    int16_t x = 0;
    int16_t y = 0;
    int16_t z = 0;
    int32_t position_crad = 0;
    int32_t velocity_crads = 0;

    float positionRad() const { return (float)position_crad * 0.01f; }
    float velocityRadS() const { return (float)velocity_crads * 0.01f; }
    float velocityRpm() const { return velocityRadS() * 9.5492966f; }
};

class PennyEsc {
public:
    PennyEsc(uint8_t address = 0) : address_(address) {}

    void begin(
        HardwareSerial &serial = Serial1,
        const PennyEscPins &pins = PennyEscPins(),
        uint32_t baud = PENNYESC_BAUD_UPDATE,
        bool drive_power_pins = true
    )
    {
        serial_ = &serial;
        pins_ = pins;
        if (drive_power_pins) {
            pinMode(pins_.gnd, OUTPUT);
            digitalWrite(pins_.gnd, LOW);
            pinMode(pins_.pullup, OUTPUT);
            digitalWrite(pins_.pullup, HIGH);
        }
        beginUart(baud, SERIAL_8N1);
    }

    void beginUart(uint32_t baud, uint32_t config)
    {
        baud_ = baud;
        config_ = config;
        serial().end();
        serial().begin(baud_, config_, pins_.rx, pins_.tx);
    }

    void clearRx()
    {
        while (serial().available() > 0) {
            serial().read();
        }
    }

    uint32_t baud() const { return baud_; }
    uint8_t address() const { return address_; }
    void setAddress(uint8_t address) { address_ = (uint8_t)(address & 0x0Fu); }
    HardwareSerial &serial() const { return *serial_; }

    bool getStatus(PennyEscStatus &status, uint32_t timeout_ms = 20u)
    {
        uint8_t frame[PNY_FRAME_MAX_PAYLOAD + 4u];
        uint8_t frame_len = 0u;

        if (!sendFrame(PNY_CMD_GET_STATUS, 0, 0)) {
            return false;
        }
        if (!readFrame(PNY_CMD_GET_STATUS, frame, sizeof(frame), frame_len, timeout_ms)) {
            return false;
        }
        return parseStatus(frame, frame_len, status);
    }

    bool setDuty(int16_t duty, PennyEscStatus *status = 0, uint32_t timeout_ms = 20u)
    {
        uint8_t payload[2];
        uint8_t frame[PNY_FRAME_MAX_PAYLOAD + 4u];
        uint8_t frame_len = 0u;

        memcpy(payload, &duty, sizeof(duty));
        if (!sendFrame(PNY_CMD_SET_DUTY, payload, sizeof(payload))) {
            return false;
        }
        if (!readFrame(PNY_CMD_SET_DUTY, frame, sizeof(frame), frame_len, timeout_ms)) {
            return false;
        }
        if (status != 0) {
            return parseStatus(frame, frame_len, *status);
        }
        PennyEscStatus ignore;
        return parseStatus(frame, frame_len, ignore);
    }

    bool setPositionCrad(int32_t position_crad, PennyEscStatus *status = 0, uint32_t timeout_ms = 20u)
    {
        uint8_t payload[4];
        uint8_t frame[PNY_FRAME_MAX_PAYLOAD + 4u];
        uint8_t frame_len = 0u;

        memcpy(payload, &position_crad, sizeof(position_crad));
        if (!sendFrame(PNY_CMD_SET_POSITION, payload, sizeof(payload))) {
            return false;
        }
        if (!readFrame(PNY_CMD_SET_POSITION, frame, sizeof(frame), frame_len, timeout_ms)) {
            return false;
        }
        if (status != 0) {
            return parseStatus(frame, frame_len, *status);
        }
        PennyEscStatus ignore;
        return parseStatus(frame, frame_len, ignore);
    }

    bool setPositionRad(float position_rad, PennyEscStatus *status = 0, uint32_t timeout_ms = 20u)
    {
        return setPositionCrad((int32_t)(position_rad * 100.0f), status, timeout_ms);
    }

    bool setAdvanceDeg(int16_t advance_deg, PennyEscStatus *status = 0, uint32_t timeout_ms = 20u)
    {
        uint8_t payload[2];
        uint8_t frame[PNY_FRAME_MAX_PAYLOAD + 4u];
        uint8_t frame_len = 0u;

        memcpy(payload, &advance_deg, sizeof(advance_deg));
        if (!sendFrame(PNY_CMD_SET_ADVANCE, payload, sizeof(payload))) {
            return false;
        }
        if (!readFrame(PNY_CMD_SET_ADVANCE, frame, sizeof(frame), frame_len, timeout_ms)) {
            return false;
        }
        if (status != 0) {
            return parseStatus(frame, frame_len, *status);
        }
        PennyEscStatus ignore;
        return parseStatus(frame, frame_len, ignore);
    }

    bool enterBootloader(uint32_t timeout_ms = 800u)
    {
        uint8_t payload[4];
        uint8_t frame[PNY_FRAME_MAX_PAYLOAD + 4u];
        uint8_t frame_len = 0u;
        uint32_t magic = PNY_BOOT_MAGIC;

        memcpy(payload, &magic, sizeof(magic));
        if (!sendFrame(PNY_CMD_ENTER_BOOT, payload, sizeof(payload))) {
            return false;
        }
        if (!readFrame(PNY_CMD_ENTER_BOOT, frame, sizeof(frame), frame_len, timeout_ms)) {
            return false;
        }
        return frame_len >= 5u && frame[3] == PNY_RESULT_OK;
    }

    bool pollEncoder(PennyEscEncoderData &data, uint32_t timeout_ms = 20u)
    {
        uint8_t tx[3];
        uint8_t rx[17];
        uint8_t idx = 0u;
        uint32_t deadline;

        tx[0] = PNY_FRAME_START;
        tx[1] = (uint8_t)((address_ << 4) | 0x03u);
        tx[2] = crc8(tx, 2u);

        clearRx();
        serial().write(tx, sizeof(tx));
        serial().flush();

        deadline = millis() + timeout_ms;
        while ((int32_t)(millis() - deadline) < 0 && idx < sizeof(rx)) {
            if (serial().available() <= 0) {
                continue;
            }
            uint8_t byte = (uint8_t)serial().read();
            if (byte == PNY_FRAME_START) {
                idx = 0u;
                rx[idx++] = byte;
                continue;
            }
            if (idx == 0u) {
                continue;
            }
            rx[idx++] = byte;
        }

        if (idx != sizeof(rx) || crc8(rx, sizeof(rx) - 1u) != rx[sizeof(rx) - 1u]) {
            data.valid = false;
            return false;
        }

        data.valid = ((rx[1] >> 4) & 0x0Fu) == address_;
        data.x = (int16_t)(rx[2] | ((int16_t)rx[3] << 8));
        data.y = (int16_t)(rx[4] | ((int16_t)rx[5] << 8));
        data.z = (int16_t)(rx[6] | ((int16_t)rx[7] << 8));
        memcpy(&data.position_crad, &rx[8], sizeof(data.position_crad));
        memcpy(&data.velocity_crads, &rx[12], sizeof(data.velocity_crads));
        return data.valid;
    }

    static uint8_t crc8(const uint8_t *data, size_t len)
    {
        uint8_t crc = 0u;

        while (len-- > 0u) {
            crc ^= *data++;
            for (uint8_t bit = 0; bit < 8u; bit++) {
                crc = (crc & 0x80u) ? (uint8_t)((crc << 1) ^ 0x07u) : (uint8_t)(crc << 1);
            }
        }

        return crc;
    }

private:
    bool sendFrame(uint8_t cmd, const void *payload, uint8_t payload_len)
    {
        uint8_t frame[PNY_FRAME_MAX_PAYLOAD + 4u];

        if (payload_len > PNY_FRAME_MAX_PAYLOAD) {
            return false;
        }

        frame[0] = PNY_FRAME_START;
        frame[1] = (uint8_t)((address_ << 4) | (cmd & 0x0Fu));
        frame[2] = payload_len;
        if (payload_len != 0u && payload != 0) {
            memcpy(&frame[3], payload, payload_len);
        }
        frame[3 + payload_len] = crc8(frame, (size_t)(3u + payload_len));

        clearRx();
        serial().write(frame, (size_t)(4u + payload_len));
        serial().flush();
        return true;
    }

    bool readFrame(uint8_t expected_cmd, uint8_t *out, size_t out_cap, uint8_t &out_len, uint32_t timeout_ms)
    {
        uint8_t idx = 0u;
        uint8_t expected = 0u;
        uint32_t deadline = millis() + timeout_ms;

        out_len = 0u;
        while ((int32_t)(millis() - deadline) < 0) {
            if (serial().available() <= 0) {
                continue;
            }

            uint8_t byte = (uint8_t)serial().read();
            if (idx == 0u) {
                if (byte == PNY_FRAME_START) {
                    out[idx++] = byte;
                }
                continue;
            }

            if (idx < 3u && byte == PNY_FRAME_START) {
                out[0] = byte;
                idx = 1u;
                expected = 0u;
                continue;
            }

            if (idx >= out_cap) {
                idx = 0u;
                expected = 0u;
                continue;
            }

            out[idx++] = byte;
            if (idx == 3u) {
                if (out[2] > PNY_FRAME_MAX_PAYLOAD) {
                    idx = 0u;
                    expected = 0u;
                    continue;
                }
                expected = (uint8_t)(out[2] + 4u);
            }

            if (expected != 0u && idx == expected) {
                if (crc8(out, (size_t)(expected - 1u)) != out[expected - 1u]) {
                    idx = 0u;
                    expected = 0u;
                    continue;
                }
                if ((out[1] >> 4) != address_ || (out[1] & 0x0Fu) != expected_cmd) {
                    idx = 0u;
                    expected = 0u;
                    continue;
                }
                out_len = expected;
                return true;
            }
        }

        return false;
    }

    static bool parseStatus(const uint8_t *frame, uint8_t frame_len, PennyEscStatus &status)
    {
        pny_status_payload_t payload;

        if (frame_len != (uint8_t)(sizeof(payload) + 4u) || frame[2] != sizeof(payload)) {
            status.valid = false;
            return false;
        }

        memcpy(&payload, &frame[3], sizeof(payload));
        status.valid = true;
        status.result = payload.result;
        status.mode = payload.mode;
        status.flags = payload.flags;
        status.faults = payload.faults;
        status.x = payload.x;
        status.y = payload.y;
        status.z = payload.z;
        status.angle_turn16 = payload.angle_turn16;
        status.position_crad = payload.position_crad;
        status.velocity_crads = payload.velocity_crads;
        status.duty = payload.duty;
        return true;
    }

    HardwareSerial *serial_ = &Serial1;
    PennyEscPins pins_;
    uint32_t baud_ = PENNYESC_BAUD_UPDATE;
    uint32_t config_ = SERIAL_8N1;
    uint8_t address_ = 0u;
};

class PennyEscBridge {
public:
    void begin(
        Stream &usb,
        HardwareSerial &uart = Serial1,
        const PennyEscPins &pins = PennyEscPins(),
        uint32_t app_baud = PENNYESC_BAUD_UPDATE,
        bool drive_power_pins = true
    )
    {
        usb_ = &usb;
        uart_ = &uart;
        pins_ = pins;
        app_baud_ = app_baud;
        if (drive_power_pins) {
            pinMode(pins_.gnd, OUTPUT);
            digitalWrite(pins_.gnd, LOW);
            pinMode(pins_.pullup, OUTPUT);
            digitalWrite(pins_.pullup, HIGH);
        }
        beginApp();
        delay(50);
        usb_->println("pennyesc-bootbridge");
        printHelp();
    }

    void poll()
    {
        if (bridge_mode_ == 0u) {
            shellPoll();
        } else {
            bridgePoll();
        }
    }

private:
    void setUart(uint32_t baud, uint32_t config)
    {
        uart().end();
        uart().begin(baud, config, pins_.rx, pins_.tx);
    }

    void beginApp()
    {
        setUart(app_baud_, SERIAL_8N1);
        uart_is_rom_ = false;
    }

    void beginBoot()
    {
        setUart(PENNYESC_BAUD_UPDATE, SERIAL_8N1);
        uart_is_rom_ = false;
    }

    void beginRom()
    {
        setUart(PENNYESC_ROM_BAUD, SERIAL_8E1);
        uart_is_rom_ = true;
    }

    void flushUartRx()
    {
        while (uart().available() > 0) {
            uart().read();
        }
    }

    void printHelp()
    {
        usb_->println("commands: help ping bridge app bridge upload bridge rom");
    }

    void bridgeEnter(uint8_t mode)
    {
        while (usb_->available() > 0) {
            usb_->read();
        }
        flushUartRx();
        boot_probe_idx_ = 0u;
        upload_switch_armed_ = false;
        bridge_mode_ = mode;
        bridge_last_activity_ms_ = millis();

        if (mode == 3u) {
            beginRom();
            usb_->println("# bridge=rom");
        } else if (mode == 2u) {
            beginBoot();
            usb_->println("# bridge=upload");
        } else {
            beginApp();
            usb_->println("# bridge=app");
        }
    }

    void bridgeExit()
    {
        bridge_mode_ = 0u;
        upload_switch_armed_ = false;
        boot_probe_idx_ = 0u;
        beginApp();
        usb_->println("# bridge=off");
    }

    void bootProbeByte(uint8_t byte)
    {
        if (upload_switch_armed_ || bridge_mode_ != 2u) {
            return;
        }

        if (boot_probe_idx_ == 0u) {
            if (byte == PNY_FRAME_START) {
                boot_probe_[boot_probe_idx_++] = byte;
            }
            return;
        }

        if (byte == PNY_FRAME_START) {
            boot_probe_[0] = byte;
            boot_probe_idx_ = 1u;
            return;
        }

        if (boot_probe_idx_ >= sizeof(boot_probe_)) {
            boot_probe_idx_ = 0u;
            return;
        }

        boot_probe_[boot_probe_idx_++] = byte;
        if (boot_probe_idx_ != sizeof(boot_probe_)) {
            return;
        }

        uint32_t magic = (uint32_t)boot_probe_[3] |
                         ((uint32_t)boot_probe_[4] << 8) |
                         ((uint32_t)boot_probe_[5] << 16) |
                         ((uint32_t)boot_probe_[6] << 24);
        if (boot_probe_[2] == 4u &&
            (boot_probe_[1] & 0x0Fu) == PNY_CMD_ENTER_BOOT &&
            magic == PNY_BOOT_MAGIC &&
            PennyEsc::crc8(boot_probe_, 7u) == boot_probe_[7]) {
            upload_switch_armed_ = true;
            upload_switch_ms_ = millis() + 160u;
        }

        boot_probe_idx_ = 0u;
    }

    void handleLine()
    {
        line_buf_[line_len_] = '\0';

        if (line_len_ == 0u) {
            return;
        }
        if (strcmp(line_buf_, "help") == 0) {
            printHelp();
            return;
        }
        if (strcmp(line_buf_, "ping") == 0) {
            usb_->println("pong");
            return;
        }
        if (strcmp(line_buf_, "bridge app") == 0) {
            bridgeEnter(1u);
            return;
        }
        if (strcmp(line_buf_, "bridge upload") == 0) {
            bridgeEnter(2u);
            return;
        }
        if (strcmp(line_buf_, "bridge rom") == 0) {
            bridgeEnter(3u);
            return;
        }
        usb_->println("err");
    }

    void shellPoll()
    {
        while (usb_->available() > 0) {
            int value = usb_->read();
            if (value < 0) {
                break;
            }

            char ch = (char)value;
            if (ch == '\r') {
                continue;
            }
            if (ch == '\n') {
                handleLine();
                line_len_ = 0u;
                continue;
            }
            if (line_len_ + 1u < sizeof(line_buf_)) {
                line_buf_[line_len_++] = ch;
            }
        }
    }

    void bridgePoll()
    {
        if (upload_switch_armed_ && (int32_t)(millis() - upload_switch_ms_) >= 0) {
            flushUartRx();
            beginRom();
            upload_switch_armed_ = false;
        }

        while (usb_->available() > 0) {
            uint8_t byte = (uint8_t)usb_->read();
            if (!(bridge_mode_ == 2u && upload_switch_armed_)) {
                uart().write(&byte, 1u);
            }
            if (bridge_mode_ == 2u && !uart_is_rom_) {
                bootProbeByte(byte);
            }
            bridge_last_activity_ms_ = millis();
        }

        while (uart().available() > 0) {
            uint8_t byte = (uint8_t)uart().read();
            usb_->write(&byte, 1u);
            bridge_last_activity_ms_ = millis();
        }

    }

    HardwareSerial &uart() const { return *uart_; }

    Stream *usb_ = &Serial;
    HardwareSerial *uart_ = &Serial1;
    PennyEscPins pins_;
    uint32_t app_baud_ = PENNYESC_BAUD_UPDATE;
    uint8_t bridge_mode_ = 0u;
    bool uart_is_rom_ = false;
    bool upload_switch_armed_ = false;
    uint32_t upload_switch_ms_ = 0u;
    uint32_t bridge_last_activity_ms_ = 0u;
    char line_buf_[48];
    uint8_t line_len_ = 0u;
    uint8_t boot_probe_[8];
    uint8_t boot_probe_idx_ = 0u;
};

#endif
