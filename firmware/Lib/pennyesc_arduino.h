#ifndef PENNYESC_ARDUINO_H
#define PENNYESC_ARDUINO_H

#include <Arduino.h>
#include <string.h>
#include "pennyesc_protocol.h"

static const uint32_t PENNYESC_BAUD_UPDATE = 115200u;
static const uint32_t PENNYESC_BAUD_FAST = 2000000u;
static const uint32_t PENNYESC_ROM_BAUD = 115200u;
static const float PENNYESC_TURN32_PER_REV = 65536.0f;
static const float PENNYESC_TURN32_TO_RAD = 6.2831853f / PENNYESC_TURN32_PER_REV;
static const float PENNYESC_RAD_TO_TURN32 = PENNYESC_TURN32_PER_REV / 6.2831853f;

static volatile bool pennyesc_bridge_active = false;

static float pennyEscTurn16ToRad(uint16_t value)
{
    return ((float)value * 6.2831853f) / 65536.0f;
}

static float pennyEscTurn32ToRad(int32_t value)
{
    return (float)value * PENNYESC_TURN32_TO_RAD;
}

static float pennyEscTurn32ToRpm(int32_t value)
{
    return (float)value * 60.0f / PENNYESC_TURN32_PER_REV;
}

static int32_t pennyEscRadToTurn32(float value)
{
    return (int32_t)(value * PENNYESC_RAD_TO_TURN32);
}

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
    int32_t position_turn32 = 0;
    int32_t velocity_turn32_per_s = 0;
    int16_t duty = 0;
    uint16_t mct_fault_count = 0;
    uint16_t isr_us = 0;
    uint16_t isr_max_us = 0;
    uint16_t i2c_us = 0;
    uint16_t i2c_start_us = 0;
    uint16_t i2c_end_us = 0;
    uint32_t isr_overrun_count = 0;
    uint32_t i2c_timeout_count = 0;
    uint32_t i2c_nack_count = 0;
    uint32_t i2c_recover_count = 0;
    uint32_t uart_overrun_errors = 0;
    uint32_t tmag_sample_count = 0;
    uint16_t tmag_sample_dt_us = 0;

    bool sensorOk() const { return (flags & PNY_FLAG_SENSOR_OK) != 0u; }
    bool calValid() const { return (flags & PNY_FLAG_CAL_VALID) != 0u; }
    bool busy() const { return (flags & PNY_FLAG_BUSY) != 0u; }
    bool positionReached() const { return (flags & PNY_FLAG_POSITION_REACHED) != 0u; }
    bool hasFault() const { return (flags & PNY_FLAG_FAULT) != 0u; }
    float angleRad() const { return pennyEscTurn16ToRad(angle_turn16); }
    float positionRad() const { return pennyEscTurn32ToRad(position_turn32); }
    float velocityRadS() const { return pennyEscTurn32ToRad(velocity_turn32_per_s); }
    float velocityRpm() const { return pennyEscTurn32ToRpm(velocity_turn32_per_s); }
};

struct PennyEscEncoderData {
    bool valid = false;
    int16_t x = 0;
    int16_t y = 0;
    int16_t z = 0;
    int32_t position_turn32 = 0;
    int32_t velocity_turn32_per_s = 0;

    float positionRad() const { return pennyEscTurn32ToRad(position_turn32); }
    float velocityRadS() const { return pennyEscTurn32ToRad(velocity_turn32_per_s); }
    float velocityRpm() const { return pennyEscTurn32ToRpm(velocity_turn32_per_s); }
};

class PennyEsc {
public:
    PennyEsc(uint8_t address = 1) : address_(address) {}

    void begin(
        HardwareSerial &serial = Serial1,
        int rx = 12,
        int tx = 13
    )
    {
        serial_ = &serial;
        rx_ = rx;
        tx_ = tx;
        beginUart(PENNYESC_BAUD_FAST, SERIAL_8N1);
    }

    void beginUart(uint32_t baud, uint32_t config)
    {
        baud_ = baud;
        serial().end();
        serial().begin(baud, config, rx_, tx_);
    }

    void clearRx()
    {
        while (serial().available() > 0) {
            serial().read();
        }
        rx_index_ = 0u;
        rx_expected_ = 0u;
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

    bool getPosVel(PennyEscEncoderData &data, uint32_t timeout_ms = 20u)
    {
        uint8_t frame[sizeof(pny_pos_vel_payload_t) + 4u];
        uint8_t frame_len = 0u;

        if (!sendFrame(PNY_CMD_GET_POS_VEL, 0, 0)) {
            data.valid = false;
            return false;
        }
        if (!readFrame(PNY_CMD_GET_POS_VEL, frame, sizeof(frame), frame_len, timeout_ms)) {
            data.valid = false;
            return false;
        }
        return parsePosVel(frame, frame_len, data);
    }

    bool requestPosVel()
    {
        return sendFrame(PNY_CMD_GET_POS_VEL, 0, 0, false);
    }

    bool readPosVelAvailable(PennyEscEncoderData &data)
    {
        uint8_t frame[sizeof(pny_pos_vel_payload_t) + 4u];
        uint8_t frame_len = 0u;

        if (!readFrameAvailable(PNY_CMD_GET_POS_VEL, frame, sizeof(frame), frame_len)) {
            return false;
        }
        return parsePosVel(frame, frame_len, data);
    }

    bool setDuty(int16_t duty, PennyEscStatus *status = 0, uint32_t timeout_ms = 20u)
    {
        uint8_t payload[2];

        memcpy(payload, &duty, sizeof(duty));
        return sendStatusCommand(PNY_CMD_SET_DUTY, payload, sizeof(payload), status, timeout_ms);
    }

    bool sendDuty(int16_t duty)
    {
        uint8_t payload[2];

        memcpy(payload, &duty, sizeof(duty));
        return sendFrame(PNY_CMD_SEND_DUTY, payload, sizeof(payload), false);
    }

    bool stop(PennyEscStatus *status = 0, uint32_t timeout_ms = 20u)
    {
        return sendStatusCommand(PNY_CMD_STOP, 0, 0u, status, timeout_ms);
    }

    bool setPositionTurn32(int32_t position_turn32, PennyEscStatus *status = 0, uint32_t timeout_ms = 20u)
    {
        uint8_t payload[4];

        memcpy(payload, &position_turn32, sizeof(position_turn32));
        return sendStatusCommand(PNY_CMD_SET_POSITION, payload, sizeof(payload), status, timeout_ms);
    }

    bool setPositionRad(float position_rad, PennyEscStatus *status = 0, uint32_t timeout_ms = 20u)
    {
        return setPositionTurn32(pennyEscRadToTurn32(position_rad), status, timeout_ms);
    }

    bool sendPositionTurn32(int32_t position_turn32)
    {
        uint8_t payload[4];

        memcpy(payload, &position_turn32, sizeof(position_turn32));
        return sendFrame(PNY_CMD_SEND_POSITION, payload, sizeof(payload), false);
    }

    bool sendPositionRad(float position_rad)
    {
        return sendPositionTurn32(pennyEscRadToTurn32(position_rad));
    }

    bool zeroPosition(PennyEscStatus *status = 0, uint32_t timeout_ms = 20u)
    {
        return sendStatusCommand(PNY_CMD_ZERO_POSITION, 0, 0u, status, timeout_ms);
    }

    bool setAdvanceDeg(int16_t advance_deg, PennyEscStatus *status = 0, uint32_t timeout_ms = 20u)
    {
        uint8_t payload[2];

        memcpy(payload, &advance_deg, sizeof(advance_deg));
        return sendStatusCommand(PNY_CMD_SET_ADVANCE, payload, sizeof(payload), status, timeout_ms);
    }

    bool setControl(
        float kp,
        float kd,
        float kv,
        int16_t kf,
        int16_t clip,
        PennyEscStatus *status = 0,
        uint32_t timeout_ms = 20u
    )
    {
        pny_control_payload_t payload;

        payload.kp_q8 = (int16_t)(kp * 256.0f);
        payload.kd_q8 = (int16_t)(kd * 256.0f);
        payload.kv_q8 = (int16_t)(kv * 256.0f);
        payload.kf = kf;
        payload.clip = clip;
        return sendStatusCommand(PNY_CMD_SET_CONTROL, &payload, sizeof(payload), status, timeout_ms);
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
        return getPosVel(data, timeout_ms);
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

protected:
    bool sendFrame(uint8_t cmd, const void *payload, uint8_t payload_len, bool wait_for_tx = true)
    {
        uint8_t frame[PNY_FRAME_MAX_PAYLOAD + 4u];

        if (pennyesc_bridge_active) {
            return false;
        }
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

        serial().write(frame, (size_t)(4u + payload_len));
        if (wait_for_tx) {
            serial().flush();
        }
        return true;
    }

    bool readFrame(uint8_t expected_cmd, uint8_t *out, size_t out_cap, uint8_t &out_len, uint32_t timeout_ms)
    {
        uint32_t deadline = micros() + (timeout_ms * 1000u);

        out_len = 0u;
        while ((int32_t)(micros() - deadline) < 0) {
            if (readFrameAvailable(expected_cmd, out, out_cap, out_len)) {
                return true;
            }
        }

        return false;
    }

    bool readFrameAvailable(uint8_t expected_cmd, uint8_t *out, size_t out_cap, uint8_t &out_len)
    {
        out_len = 0u;
        while (serial().available() > 0) {
            uint8_t byte = (uint8_t)serial().read();
            if (rx_index_ == 0u) {
                if (byte == PNY_FRAME_START) {
                    rx_buf_[rx_index_++] = byte;
                }
                continue;
            }

            if (rx_index_ < 3u && byte == PNY_FRAME_START) {
                rx_buf_[0] = byte;
                rx_index_ = 1u;
                rx_expected_ = 0u;
                continue;
            }

            if (rx_index_ >= sizeof(rx_buf_)) {
                rx_index_ = 0u;
                rx_expected_ = 0u;
                continue;
            }

            rx_buf_[rx_index_++] = byte;
            if (rx_index_ == 3u) {
                if (rx_buf_[2] > PNY_FRAME_MAX_PAYLOAD) {
                    rx_index_ = 0u;
                    rx_expected_ = 0u;
                    continue;
                }
                rx_expected_ = (uint8_t)(rx_buf_[2] + 4u);
            }

            if (rx_expected_ == 0u || rx_index_ != rx_expected_) {
                continue;
            }

            uint8_t frame_len = rx_expected_;
            rx_index_ = 0u;
            rx_expected_ = 0u;
            if (crc8(rx_buf_, (size_t)(frame_len - 1u)) != rx_buf_[frame_len - 1u]) {
                continue;
            }
            if ((rx_buf_[1] >> 4) != address_ || (rx_buf_[1] & 0x0Fu) != expected_cmd) {
                continue;
            }
            if (frame_len > out_cap) {
                continue;
            }
            memcpy(out, rx_buf_, frame_len);
            out_len = frame_len;
            return true;
        }
        return false;
    }

private:
    bool sendStatusCommand(uint8_t cmd, const void *payload, uint8_t payload_len, PennyEscStatus *status, uint32_t timeout_ms)
    {
        uint8_t frame[PNY_FRAME_MAX_PAYLOAD + 4u];
        uint8_t frame_len = 0u;

        if (!sendFrame(cmd, payload, payload_len)) {
            return false;
        }
        if (!readFrame(cmd, frame, sizeof(frame), frame_len, timeout_ms)) {
            return false;
        }
        if (status != 0) {
            return parseStatus(frame, frame_len, *status);
        }
        PennyEscStatus ignore;
        return parseStatus(frame, frame_len, ignore);
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
        status.position_turn32 = payload.position_turn32;
        status.velocity_turn32_per_s = payload.velocity_turn32_per_s;
        status.duty = payload.duty;
        status.mct_fault_count = payload.mct_fault_count;
        status.isr_us = payload.isr_us;
        status.isr_max_us = payload.isr_max_us;
        status.i2c_us = payload.i2c_us;
        status.i2c_start_us = payload.i2c_start_us;
        status.i2c_end_us = payload.i2c_end_us;
        status.isr_overrun_count = payload.isr_overrun_count;
        status.i2c_timeout_count = payload.i2c_timeout_count;
        status.i2c_nack_count = payload.i2c_nack_count;
        status.i2c_recover_count = payload.i2c_recover_count;
        status.uart_overrun_errors = payload.uart_overrun_errors;
        status.tmag_sample_count = payload.tmag_sample_count;
        status.tmag_sample_dt_us = payload.tmag_sample_dt_us;
        return true;
    }

    static bool parsePosVel(const uint8_t *frame, uint8_t frame_len, PennyEscEncoderData &data)
    {
        pny_pos_vel_payload_t payload;

        if (frame_len != (uint8_t)(sizeof(payload) + 4u) || frame[2] != sizeof(payload)) {
            data.valid = false;
            return false;
        }

        memcpy(&payload, &frame[3], sizeof(payload));
        data.valid = true;
        data.x = 0;
        data.y = 0;
        data.z = 0;
        data.position_turn32 = payload.position_turn32;
        data.velocity_turn32_per_s = payload.velocity_turn32_per_s;
        return true;
    }

    HardwareSerial *serial_ = &Serial1;
    int rx_ = 12;
    int tx_ = 13;
    uint32_t baud_ = PENNYESC_BAUD_FAST;
    uint8_t address_ = 1u;
    uint8_t rx_buf_[PNY_FRAME_MAX_PAYLOAD + 4u];
    uint8_t rx_index_ = 0u;
    uint8_t rx_expected_ = 0u;
};

class PennyEscBridge {
protected:
    enum BridgeMode : uint8_t {
        ModeShell = 0u,
        ModeApp = 1u,
        ModeUpload = 2u,
        ModeRom = 3u,
        ModeHandoff = 4u,
    };

public:
    void setAddress(uint8_t address)
    {
        address_ = (uint8_t)(address & 0x0Fu);
        esc_.setAddress(address_);
    }

    void begin(
        Stream &usb,
        HardwareSerial &uart = Serial1,
        int rx = 12,
        int tx = 13,
        uint8_t address = 1
    )
    {
        beginMode(usb, uart, rx, tx, address, false, false);
    }

    void beginBackground(
        Stream &usb,
        HardwareSerial &uart = Serial1,
        int rx = 12,
        int tx = 13,
        uint8_t address = 1
    )
    {
        beginMode(usb, uart, rx, tx, address, true, true);
    }

    bool poll()
    {
        if (bridge_mode_ == ModeShell) {
            return shellPoll();
        } else if (bridge_mode_ == ModeUpload) {
            rawBridgePoll();
        } else {
            bridgePoll();
        }
        return true;
    }

protected:
    void beginMode(
        Stream &usb,
        HardwareSerial &uart,
        int rx,
        int tx,
        uint8_t address,
        bool background,
        bool prefix_only
    )
    {
        usb_ = &usb;
        uart_ = &uart;
        rx_ = rx;
        tx_ = tx;
        prefix_only_ = prefix_only;
        setAddress(address);
        beginApp();
        esc_.begin(*uart_, rx_, tx_);
        delay(50);
        if (background) {
            startTask();
        } else {
            usb_->println("pennyesc-bootbridge");
            printHelp();
        }
    }
#if defined(ARDUINO_ARCH_ESP32)
    static void taskMain(void *arg)
    {
        PennyEscBridge *bridge = (PennyEscBridge *)arg;
        for (;;) {
            bridge->poll();
            delay(1);
        }
    }

    void startTask()
    {
        if (task_ != 0) {
            return;
        }
        xTaskCreatePinnedToCore(taskMain, "pnybridge", 4096, this, 1, &task_, 0);
    }
#else
    void startTask() {}
#endif

    void setUart(uint32_t baud, uint32_t config)
    {
        uart().end();
        uart().begin(baud, config, rx_, tx_);
    }

    void beginApp()
    {
        setUart(PENNYESC_BAUD_FAST, SERIAL_8N1);
    }

    void beginBoot()
    {
        setUart(PENNYESC_BAUD_UPDATE, SERIAL_8N1);
    }

    void beginRom()
    {
        setUart(PENNYESC_ROM_BAUD, SERIAL_8E1);
    }

    void flushUartRx()
    {
        while (uart().available() > 0) {
            uart().read();
        }
    }

    virtual void printHelp()
    {
        usb_->println("commands: help ping e<addr> s d<duty> bridge app bridge handoff bridge upload bridge rom bridge off");
    }

    virtual bool handleDebugLine(const char *line)
    {
        (void)line;
        return false;
    }

    void printStatus(const PennyEscStatus &status)
    {
        usb_->print("result=");
        usb_->print(status.result);
        usb_->print(" duty=");
        usb_->print(status.duty);
        usb_->print(" pos=");
        usb_->print(status.positionRad(), 3);
        usb_->print(" vel=");
        usb_->print(status.velocityRpm(), 1);
        usb_->print(" x=");
        usb_->print(status.x);
        usb_->print(" y=");
        usb_->print(status.y);
        usb_->print(" z=");
        usb_->print(status.z);
        usb_->print(" mct_faults=");
        usb_->print(status.mct_fault_count);
        usb_->print(" isr=");
        usb_->print(status.isr_us);
        usb_->print("us max=");
        usb_->print(status.isr_max_us);
        usb_->print("us overruns=");
        usb_->print(status.isr_overrun_count);
        usb_->print(" i2c=");
        usb_->print(status.i2c_us);
        usb_->print("us phase=");
        usb_->print(status.i2c_start_us);
        usb_->print("->");
        usb_->print(status.i2c_end_us);
        usb_->print(" i2c_err=");
        usb_->print(status.i2c_timeout_count);
        usb_->print("/");
        usb_->print(status.i2c_nack_count);
        usb_->print("/");
        usb_->print(status.i2c_recover_count);
        usb_->print(" tmag_dt=");
        usb_->println(status.tmag_sample_dt_us);
    }

    void bridgeEnter(BridgeMode mode)
    {
        while (usb_->available() > 0) {
            usb_->read();
        }
        flushUartRx();
        bridge_mode_ = mode;
        pennyesc_bridge_active = true;

        if (mode == ModeHandoff) {
            beginApp();
            usb_->println("# bridge=handoff");
        } else if (mode == ModeRom) {
            beginRom();
            usb_->println("# bridge=rom");
        } else if (mode == ModeUpload) {
            beginBoot();
            usb_->println("# bridge=upload");
        } else {
            beginApp();
            usb_->println("# bridge=app");
        }
    }

    void bridgeExit()
    {
        bridge_mode_ = ModeShell;
        pennyesc_bridge_active = false;
        beginApp();
        usb_->println("# bridge=off");
    }

    void handleLine(const char *line)
    {
        if (line == 0 || line[0] == '\0') {
            return;
        }
        if (strcmp(line, "help") == 0) {
            printHelp();
            return;
        }
        if (strcmp(line, "ping") == 0) {
            usb_->println("pong");
            return;
        }
        if (line[0] == 'e') {
            setAddress((uint8_t)atoi(line + 1));
            PennyEscStatus status;
            usb_->print("esc=");
            usb_->print(address_);
            if (esc_.getStatus(status, 100u)) {
                usb_->print(" ok ");
                printStatus(status);
            } else {
                usb_->println(" fail");
            }
            return;
        }
        if (strcmp(line, "s") == 0) {
            PennyEscStatus status;
            if (esc_.getStatus(status, 100u)) {
                printStatus(status);
            } else {
                usb_->println("fail");
            }
            return;
        }
        if (line[0] == 'd') {
            PennyEscStatus status;
            if (esc_.setDuty((int16_t)atoi(line + 1), &status, 100u)) {
                printStatus(status);
            } else {
                usb_->println("fail");
            }
            return;
        }
        if (handleDebugLine(line)) {
            return;
        }
        if (strcmp(line, "bridge app") == 0) {
            bridgeEnter(ModeApp);
            return;
        }
        if (strcmp(line, "bridge handoff") == 0) {
            bridgeEnter(ModeHandoff);
            return;
        }
        if (strcmp(line, "bridge upload") == 0) {
            bridgeEnter(ModeUpload);
            return;
        }
        if (strcmp(line, "bridge rom") == 0) {
            bridgeEnter(ModeRom);
            return;
        }
        if (strcmp(line, "bridge off") == 0) {
            bridgeExit();
            return;
        }
        usb_->println("err");
    }

    bool shellPoll()
    {
        if (usb_->available() <= 0) {
            return line_len_ != 0u;
        }
        if (prefix_only_ && line_len_ == 0u && usb_->peek() != '!') {
            return false;
        }

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
                line_buf_[line_len_] = '\0';
                if (prefix_only_ && line_len_ >= 2u && line_buf_[0] == '!' && line_buf_[1] == '#') {
                    handleLine(line_buf_ + 2u);
                } else if (!prefix_only_) {
                    handleLine(line_buf_);
                }
                line_len_ = 0u;
                return true;
            }
            if (line_len_ + 1u < sizeof(line_buf_)) {
                line_buf_[line_len_++] = ch;
            }
        }

        return line_len_ != 0u;
    }

    void flushBridgeEscape()
    {
        if (line_len_ == 0u) {
            return;
        }
        uart().write((const uint8_t *)line_buf_, line_len_);
        bridge_escape_ms_ = 0u;
        line_len_ = 0u;
    }

    bool handleBridgeEscape()
    {
        line_buf_[line_len_] = '\0';
        if (line_len_ < 2u || line_buf_[0] != '!' || line_buf_[1] != '#') {
            return false;
        }
        handleLine(line_buf_ + 2u);
        return true;
    }

    void bridgePoll()
    {
        while (usb_->available() > 0) {
            int value = usb_->read();
            if (value < 0) {
                break;
            }

            char ch = (char)value;
            if (line_len_ == 0u) {
                if (ch == '!' && (millis() - bridge_last_usb_ms_) > 20u) {
                    line_buf_[line_len_++] = ch;
                    bridge_escape_ms_ = millis();
                    bridge_last_usb_ms_ = millis();
                    continue;
                }
                uint8_t byte = (uint8_t)ch;
                uart().write(&byte, 1u);
                bridge_last_usb_ms_ = millis();
                continue;
            }

            if (line_len_ == 1u && line_buf_[0] == '!' && ch != '#') {
                flushBridgeEscape();
                uint8_t byte = (uint8_t)ch;
                uart().write(&byte, 1u);
                bridge_last_usb_ms_ = millis();
                continue;
            }

            if (ch == '\r') {
                continue;
            }
            if (ch == '\n') {
                if (!handleBridgeEscape()) {
                    flushBridgeEscape();
                    uint8_t byte = '\n';
                    uart().write(&byte, 1u);
                    bridge_last_usb_ms_ = millis();
                } else {
                    bridge_escape_ms_ = 0u;
                }
                line_len_ = 0u;
                continue;
            }
            if (ch >= 0x20 && ch <= 0x7E && line_len_ + 1u < sizeof(line_buf_)) {
                line_buf_[line_len_++] = ch;
                bridge_escape_ms_ = millis();
                continue;
            }

            flushBridgeEscape();
            uint8_t byte = (uint8_t)ch;
            uart().write(&byte, 1u);
            bridge_last_usb_ms_ = millis();
        }

        if (line_len_ != 0u && (millis() - bridge_escape_ms_) > 2u) {
            flushBridgeEscape();
        }

        while (uart().available() > 0) {
            uint8_t byte = (uint8_t)uart().read();
            usb_->write(&byte, 1u);
        }
    }

    void rawBridgePoll()
    {
        while (usb_->available() > 0) {
            uint8_t byte = (uint8_t)usb_->read();
            uart().write(&byte, 1u);
        }

        while (uart().available() > 0) {
            uint8_t byte = (uint8_t)uart().read();
            usb_->write(&byte, 1u);
        }
    }

    HardwareSerial &uart() const { return *uart_; }

    Stream *usb_ = &Serial;
    HardwareSerial *uart_ = &Serial1;
    PennyEsc esc_;
    int rx_ = 12;
    int tx_ = 13;
    uint8_t address_ = 1u;
    BridgeMode bridge_mode_ = ModeShell;
    bool prefix_only_ = true;
    uint32_t bridge_escape_ms_ = 0u;
    uint32_t bridge_last_usb_ms_ = 0u;
    char line_buf_[48];
    uint8_t line_len_ = 0u;
#if defined(ARDUINO_ARCH_ESP32)
    TaskHandle_t task_ = 0;
#endif
};

#endif
