#ifndef PENNYESC_ARDUINO_H
#define PENNYESC_ARDUINO_H

#include <Arduino.h>
#include <limits.h>
#include <string.h>
#include "pennyesc_protocol.h"

static const uint32_t PENNYESC_BAUD_UPDATE = 115200u;
static const uint32_t PENNYESC_BAUD_FAST = 2000000u;
static const uint32_t PENNYESC_ROM_BAUD = 115200u;
static const float PENNYESC_TURN32_PER_REV = 65536.0f;
static const float PENNYESC_TURN32_TO_RAD = 6.2831853f / PENNYESC_TURN32_PER_REV;
static const float PENNYESC_RAD_TO_TURN32 = PENNYESC_TURN32_PER_REV / 6.2831853f;
static const uint32_t PENNYESC_RATE_LOG_MAX = 2400u;

static volatile bool pennyesc_bridge_active = false;

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
    float angleRad() const { return ((float)angle_turn16 * 6.2831853f) / 65536.0f; }
    float positionRad() const { return (float)position_turn32 * PENNYESC_TURN32_TO_RAD; }
    float velocityRadS() const { return (float)velocity_turn32_per_s * PENNYESC_TURN32_TO_RAD; }
    float velocityRpm() const { return (float)velocity_turn32_per_s * 60.0f / PENNYESC_TURN32_PER_REV; }
};

struct PennyEscEncoderData {
    bool valid = false;
    int16_t x = 0;
    int16_t y = 0;
    int16_t z = 0;
    int32_t position_turn32 = 0;
    int32_t velocity_turn32_per_s = 0;

    float positionRad() const { return (float)position_turn32 * PENNYESC_TURN32_TO_RAD; }
    float velocityRadS() const { return (float)velocity_turn32_per_s * PENNYESC_TURN32_TO_RAD; }
    float velocityRpm() const { return (float)velocity_turn32_per_s * 60.0f / PENNYESC_TURN32_PER_REV; }
};

struct PennyEscRateSample {
    uint32_t t_us;
    int32_t position_turn32;
    int32_t velocity_turn32_per_s;
    uint16_t loop_us;
    uint8_t ok;
};

struct PennyEscPollFastSample {
    uint32_t t_us;
    uint32_t sample_age_us;
    int32_t position_turn32;
    int32_t velocity_turn32_per_s;
    uint16_t loop_us;
    uint8_t fresh;
    uint8_t valid;
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
        config_ = config;
        serial().end();
        serial().begin(baud_, config_, rx_, tx_);
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

    bool sendDuty(int16_t duty)
    {
        uint8_t payload[2];

        memcpy(payload, &duty, sizeof(duty));
        return sendFrame(PNY_CMD_SEND_DUTY, payload, sizeof(payload), false);
    }

    bool stop(PennyEscStatus *status = 0, uint32_t timeout_ms = 20u)
    {
        uint8_t frame[PNY_FRAME_MAX_PAYLOAD + 4u];
        uint8_t frame_len = 0u;

        if (!sendFrame(PNY_CMD_STOP, 0, 0u)) {
            return false;
        }
        if (!readFrame(PNY_CMD_STOP, frame, sizeof(frame), frame_len, timeout_ms)) {
            return false;
        }
        if (status != 0) {
            return parseStatus(frame, frame_len, *status);
        }
        PennyEscStatus ignore;
        return parseStatus(frame, frame_len, ignore);
    }

    bool setPositionTurn32(int32_t position_turn32, PennyEscStatus *status = 0, uint32_t timeout_ms = 20u)
    {
        uint8_t payload[4];
        uint8_t frame[PNY_FRAME_MAX_PAYLOAD + 4u];
        uint8_t frame_len = 0u;

        memcpy(payload, &position_turn32, sizeof(position_turn32));
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
        return setPositionTurn32((int32_t)(position_rad * PENNYESC_RAD_TO_TURN32), status, timeout_ms);
    }

    bool sendPositionTurn32(int32_t position_turn32)
    {
        uint8_t payload[4];

        memcpy(payload, &position_turn32, sizeof(position_turn32));
        return sendFrame(PNY_CMD_SEND_POSITION, payload, sizeof(payload), false);
    }

    bool sendPositionRad(float position_rad)
    {
        return sendPositionTurn32((int32_t)(position_rad * PENNYESC_RAD_TO_TURN32));
    }

    bool zeroPosition(PennyEscStatus *status = 0, uint32_t timeout_ms = 20u)
    {
        uint8_t frame[PNY_FRAME_MAX_PAYLOAD + 4u];
        uint8_t frame_len = 0u;

        if (!sendFrame(PNY_CMD_ZERO_POSITION, 0, 0)) {
            return false;
        }
        if (!readFrame(PNY_CMD_ZERO_POSITION, frame, sizeof(frame), frame_len, timeout_ms)) {
            return false;
        }
        if (status != 0) {
            return parseStatus(frame, frame_len, *status);
        }
        PennyEscStatus ignore;
        return parseStatus(frame, frame_len, ignore);
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
        uint8_t frame[PNY_FRAME_MAX_PAYLOAD + 4u];
        uint8_t frame_len = 0u;

        payload.kp_q8 = (int16_t)(kp * 256.0f);
        payload.kd_q8 = (int16_t)(kd * 256.0f);
        payload.kv_q8 = (int16_t)(kv * 256.0f);
        payload.kf = kf;
        payload.clip = clip;
        if (!sendFrame(PNY_CMD_SET_CONTROL, &payload, sizeof(payload))) {
            return false;
        }
        if (!readFrame(PNY_CMD_SET_CONTROL, frame, sizeof(frame), frame_len, timeout_ms)) {
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

private:
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
    uint32_t config_ = SERIAL_8N1;
    uint8_t address_ = 1u;
    uint8_t rx_buf_[PNY_FRAME_MAX_PAYLOAD + 4u];
    uint8_t rx_index_ = 0u;
    uint8_t rx_expected_ = 0u;
};

class PennyEscBridge {
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
        if (bridge_mode_ == 0u) {
            return shellPoll();
        } else if (bridge_mode_ == 2u) {
            rawBridgePoll();
        } else {
            bridgePoll();
        }
        return true;
    }

private:
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

    void printHelp()
    {
        usb_->println("commands: help ping e<addr> s d<duty> rate <ms> <hz> <duty> <spinup_ms> <timeout_ms> pollfast <ms> <hz> <duty> <spinup_ms> bridge app bridge handoff bridge upload bridge rom");
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

    void runRateTest(uint32_t duration_ms, uint32_t hz, int16_t duty, uint32_t spinup_ms, uint32_t timeout_ms)
    {
        PennyEsc esc(address_);
        PennyEscStatus status;
        PennyEscEncoderData data;
        uint32_t period_us;
        uint32_t start_us;
        uint32_t next_us;
        uint32_t elapsed_us;
        uint32_t target_loops;
        uint32_t loops = 0u;
        uint32_t missed = 0u;
        uint32_t worst_us = 0u;
        uint32_t worst_late_us = 0u;
        uint32_t fail = 0u;
        uint32_t late_response = 0u;
        uint32_t lost_response = 0u;
        uint32_t worst_late_response_us = 0u;
        uint32_t over_start = 0u;
        uint32_t over_end = 0u;
        bool status_ok = false;
        uint32_t log_count = 0u;
        uint32_t ok_samples = 0u;
        int32_t rpm_sum = 0;
        int32_t rpm_min = INT32_MAX;
        int32_t rpm_max = INT32_MIN;
        static PennyEscRateSample log[PENNYESC_RATE_LOG_MAX];

        if (duration_ms == 0u) {
            duration_ms = 10000u;
        }
        if (hz == 0u) {
            hz = 500u;
        }
        if (timeout_ms == 0u) {
            timeout_ms = 5u;
        }
        period_us = 1000000u / hz;
        target_loops = (duration_ms * hz) / 1000u;

        beginApp();
        esc.begin(uart(), rx_, tx_);
        if (esc.getStatus(status, 100u)) {
            over_start = status.uart_overrun_errors;
        } else {
            fail++;
        }
        if (duty != 0) {
            int16_t clip = (duty < 0) ? (int16_t)(-duty) : duty;
            if (!esc.setControl(0.0f, 0.0f, 0.0f, 0, clip, &status, 100u)) {
                fail++;
            }
            if (!esc.setDuty(duty, &status, 100u)) {
                fail++;
            }
            while (spinup_ms > 0u) {
                uint32_t step_ms = (spinup_ms > 50u) ? 50u : spinup_ms;
                delay(step_ms);
                spinup_ms -= step_ms;
            }
        }

        start_us = micros();
        next_us = start_us + period_us;
        while (loops < target_loops) {
            uint32_t loop_us = micros();

            bool ok = esc.getPosVel(data, timeout_ms);
            if (!ok) {
                uint32_t late_start_us = micros();
                bool got_late = false;
                fail++;
                while ((uint32_t)(micros() - late_start_us) < 20000u) {
                    if (esc.readPosVelAvailable(data)) {
                        uint32_t late_us = micros() - late_start_us;
                        late_response++;
                        got_late = true;
                        if (late_us > worst_late_response_us) {
                            worst_late_response_us = late_us;
                        }
                        break;
                    }
                }
                if (!got_late) {
                    lost_response++;
                    esc.clearRx();
                }
            } else {
                ok_samples++;
            }

            loops++;
            loop_us = micros() - loop_us;
            if (log_count < PENNYESC_RATE_LOG_MAX) {
                log[log_count].t_us = (uint32_t)(micros() - start_us);
                log[log_count].position_turn32 = data.position_turn32;
                log[log_count].velocity_turn32_per_s = data.velocity_turn32_per_s;
                log[log_count].loop_us = (uint16_t)((loop_us > 0xffffu) ? 0xffffu : loop_us);
                log[log_count].ok = ok ? 1u : 0u;
                log_count++;
            }
            if (ok) {
                int32_t rpm = (data.velocity_turn32_per_s * 60L) / 65536L;
                rpm_sum += rpm;
                if (rpm < rpm_min) {
                    rpm_min = rpm;
                }
                if (rpm > rpm_max) {
                    rpm_max = rpm;
                }
            }
            if (loop_us > worst_us) {
                worst_us = loop_us;
            }

            int32_t wait_us = (int32_t)(next_us - micros());
            if (wait_us > 0) {
                while ((int32_t)(next_us - micros()) > 0) {
                }
            } else {
                uint32_t late_us = (uint32_t)(0 - wait_us);
                missed++;
                if (late_us > worst_late_us) {
                    worst_late_us = late_us;
                }
                next_us = micros();
            }
            next_us += period_us;
        }
        elapsed_us = micros() - start_us;

        if (esc.getStatus(status, 100u)) {
            over_end = status.uart_overrun_errors;
            status_ok = true;
        } else {
            fail++;
        }
        if (duty != 0) {
            PennyEscStatus stop_status;
            (void)esc.stop(&stop_status, 100u);
        }

        usb_->print("# rate_summary duration_ms=");
        usb_->print(duration_ms);
        usb_->print(" hz=");
        usb_->print(hz);
        usb_->print(" duty=");
        usb_->print(duty);
        usb_->print(" loops=");
        usb_->print(loops);
        usb_->print(" elapsed_ms=");
        usb_->print((elapsed_us + 500u) / 1000u);
        usb_->print(" missed=");
        usb_->print(missed);
        usb_->print(" worst_us=");
        usb_->print(worst_us);
        usb_->print(" worst_late_us=");
        usb_->println(worst_late_us);
        usb_->print("# rate_address address=");
        usb_->print(address_);
        usb_->print(" cycles=");
        usb_->print(loops);
        usb_->print(" fail=");
        usb_->print(fail);
        usb_->print(" late_response=");
        usb_->print(late_response);
        usb_->print(" lost_response=");
        usb_->print(lost_response);
        usb_->print(" worst_late_response_us=");
        usb_->print(worst_late_response_us);
        usb_->print(" uart_overruns_delta=");
        usb_->println(over_end - over_start);
        usb_->print("# rate_rpm address=");
        usb_->print(address_);
        usb_->print(" samples=");
        usb_->print(ok_samples);
        usb_->print(" avg=");
        usb_->print((ok_samples != 0u) ? (rpm_sum / (int32_t)ok_samples) : 0);
        usb_->print(" min=");
        usb_->print((rpm_min == INT32_MAX) ? 0 : rpm_min);
        usb_->print(" max=");
        usb_->println((rpm_max == INT32_MIN) ? 0 : rpm_max);
        if (status_ok) {
            int32_t rpm = (status.velocity_turn32_per_s * 60L) / 65536L;
            usb_->print("# rate_status address=");
            usb_->print(address_);
            usb_->print(" rpm=");
            usb_->print(rpm);
            usb_->print(" isr_max=");
            usb_->print(status.isr_max_us);
            usb_->print(" overruns=");
            usb_->print(status.isr_overrun_count);
            usb_->print(" uart_ore=");
            usb_->print(status.uart_overrun_errors);
            usb_->print(" tmag_dt=");
            usb_->print(status.tmag_sample_dt_us);
            usb_->print(" tmag_samples=");
            usb_->print(status.tmag_sample_count);
            usb_->print(" faults=");
            usb_->println(status.faults);
        }
        usb_->println("# rate_csv sample,t_us,loop_us,ok,position_turn32,velocity_turn32_per_s,rpm");
        for (uint32_t i = 0u; i < log_count; i++) {
            int32_t rpm = (log[i].velocity_turn32_per_s * 60L) / 65536L;
            usb_->print("# rate_row ");
            usb_->print(i);
            usb_->print(",");
            usb_->print(log[i].t_us);
            usb_->print(",");
            usb_->print(log[i].loop_us);
            usb_->print(",");
            usb_->print(log[i].ok);
            usb_->print(",");
            usb_->print(log[i].position_turn32);
            usb_->print(",");
            usb_->print(log[i].velocity_turn32_per_s);
            usb_->print(",");
            usb_->println(rpm);
        }
        usb_->println("# rate_done");
    }

    void runPollFastTest(uint32_t duration_ms, uint32_t control_hz, int16_t duty, uint32_t spinup_ms)
    {
        PennyEsc esc(address_);
        PennyEscStatus status;
        PennyEscEncoderData data;
        PennyEscEncoderData latest;
        uint32_t period_us;
        uint32_t start_us;
        uint32_t next_tick_us;
        uint32_t last_sample_us = 0u;
        uint32_t request_us = 0u;
        uint32_t target_ticks;
        uint32_t ticks = 0u;
        uint32_t fresh_ticks = 0u;
        uint32_t stale_ticks = 0u;
        uint32_t missed = 0u;
        uint32_t poll_ok = 0u;
        uint32_t poll_fail = 0u;
        uint32_t timeouts = 0u;
        uint32_t fail = 0u;
        uint32_t worst_loop_us = 0u;
        uint32_t worst_age_us = 0u;
        uint32_t worst_late_us = 0u;
        uint32_t over_start = 0u;
        uint32_t over_end = 0u;
        uint32_t log_count = 0u;
        uint32_t rpm_samples = 0u;
        int32_t rpm_sum = 0;
        int32_t rpm_min = INT32_MAX;
        int32_t rpm_max = INT32_MIN;
        bool request_pending = false;
        bool latest_valid = false;
        bool tick_fresh = false;
        bool status_ok = false;
        static PennyEscPollFastSample log[PENNYESC_RATE_LOG_MAX];

        if (duration_ms == 0u) {
            duration_ms = 10000u;
        }
        if (control_hz == 0u) {
            control_hz = 600u;
        }
        period_us = 1000000u / control_hz;
        target_ticks = (duration_ms * control_hz) / 1000u;

        beginApp();
        esc.begin(uart(), rx_, tx_);
        if (esc.getStatus(status, 100u)) {
            over_start = status.uart_overrun_errors;
        } else {
            fail++;
        }
        if (duty != 0) {
            int16_t clip = (duty < 0) ? (int16_t)(-duty) : duty;
            if (!esc.setControl(0.0f, 0.0f, 0.0f, 0, clip, &status, 100u)) {
                fail++;
            }
            if (!esc.setDuty(duty, &status, 100u)) {
                fail++;
            }
            while (spinup_ms > 0u) {
                uint32_t step_ms = (spinup_ms > 50u) ? 50u : spinup_ms;
                delay(step_ms);
                spinup_ms -= step_ms;
            }
        }
        if (esc.getPosVel(latest, 20u)) {
            latest_valid = true;
            last_sample_us = micros();
        } else {
            fail++;
        }

        start_us = micros();
        next_tick_us = start_us + period_us;
        while (ticks < target_ticks) {
            uint32_t loop_start_us = micros();

            if (!request_pending) {
                if (esc.requestPosVel()) {
                    request_pending = true;
                    request_us = micros();
                } else {
                    poll_fail++;
                }
            }

            if (request_pending && esc.readPosVelAvailable(data)) {
                latest = data;
                latest_valid = true;
                last_sample_us = micros();
                request_pending = false;
                tick_fresh = true;
                poll_ok++;
                int32_t rpm = (data.velocity_turn32_per_s * 60L) / 65536L;
                rpm_sum += rpm;
                rpm_samples++;
                if (rpm < rpm_min) {
                    rpm_min = rpm;
                }
                if (rpm > rpm_max) {
                    rpm_max = rpm;
                }
            }

            if (request_pending && (uint32_t)(micros() - request_us) > 5000u) {
                request_pending = false;
                timeouts++;
                poll_fail++;
                esc.clearRx();
            }

            uint32_t now = micros();
            if ((int32_t)(now - next_tick_us) >= 0) {
                uint32_t age_us = latest_valid ? (uint32_t)(now - last_sample_us) : 0xffffffffu;
                uint32_t loop_us = now - loop_start_us;
                if (loop_us > worst_loop_us) {
                    worst_loop_us = loop_us;
                }
                if (age_us > worst_age_us && age_us != 0xffffffffu) {
                    worst_age_us = age_us;
                }
                if (latest_valid && tick_fresh) {
                    fresh_ticks++;
                } else {
                    stale_ticks++;
                }
                if (log_count < PENNYESC_RATE_LOG_MAX) {
                    log[log_count].t_us = now - start_us;
                    log[log_count].sample_age_us = age_us;
                    log[log_count].position_turn32 = latest.position_turn32;
                    log[log_count].velocity_turn32_per_s = latest.velocity_turn32_per_s;
                    log[log_count].loop_us = (uint16_t)((loop_us > 0xffffu) ? 0xffffu : loop_us);
                    log[log_count].fresh = (latest_valid && tick_fresh) ? 1u : 0u;
                    log[log_count].valid = latest_valid ? 1u : 0u;
                    log_count++;
                }
                tick_fresh = false;
                ticks++;

                int32_t wait_us = (int32_t)(next_tick_us + period_us - micros());
                if (wait_us < 0) {
                    uint32_t late_us = (uint32_t)(0 - wait_us);
                    missed++;
                    if (late_us > worst_late_us) {
                        worst_late_us = late_us;
                    }
                    next_tick_us = micros();
                } else {
                    next_tick_us += period_us;
                }
            }
        }

        if (esc.getStatus(status, 100u)) {
            over_end = status.uart_overrun_errors;
            status_ok = true;
        } else {
            fail++;
        }
        if (duty != 0) {
            PennyEscStatus stop_status;
            (void)esc.stop(&stop_status, 100u);
        }

        uint32_t elapsed_us = micros() - start_us;
        usb_->print("# pollfast_summary duration_ms=");
        usb_->print(duration_ms);
        usb_->print(" control_hz=");
        usb_->print(control_hz);
        usb_->print(" duty=");
        usb_->print(duty);
        usb_->print(" ticks=");
        usb_->print(ticks);
        usb_->print(" elapsed_ms=");
        usb_->print((elapsed_us + 500u) / 1000u);
        usb_->print(" fresh_ticks=");
        usb_->print(fresh_ticks);
        usb_->print(" stale_ticks=");
        usb_->print(stale_ticks);
        usb_->print(" missed=");
        usb_->print(missed);
        usb_->print(" poll_ok=");
        usb_->print(poll_ok);
        usb_->print(" poll_fail=");
        usb_->print(poll_fail);
        usb_->print(" timeouts=");
        usb_->print(timeouts);
        usb_->print(" worst_age_us=");
        usb_->print(worst_age_us);
        usb_->print(" worst_loop_us=");
        usb_->print(worst_loop_us);
        usb_->print(" worst_late_us=");
        usb_->println(worst_late_us);
        usb_->print("# pollfast_address address=");
        usb_->print(address_);
        usb_->print(" fail=");
        usb_->print(fail);
        usb_->print(" uart_overruns_delta=");
        usb_->println(over_end - over_start);
        usb_->print("# pollfast_rpm address=");
        usb_->print(address_);
        usb_->print(" samples=");
        usb_->print(rpm_samples);
        usb_->print(" avg=");
        usb_->print((rpm_samples != 0u) ? (rpm_sum / (int32_t)rpm_samples) : 0);
        usb_->print(" min=");
        usb_->print((rpm_min == INT32_MAX) ? 0 : rpm_min);
        usb_->print(" max=");
        usb_->println((rpm_max == INT32_MIN) ? 0 : rpm_max);
        if (status_ok) {
            int32_t rpm = (status.velocity_turn32_per_s * 60L) / 65536L;
            usb_->print("# pollfast_status address=");
            usb_->print(address_);
            usb_->print(" rpm=");
            usb_->print(rpm);
            usb_->print(" isr_max=");
            usb_->print(status.isr_max_us);
            usb_->print(" overruns=");
            usb_->print(status.isr_overrun_count);
            usb_->print(" uart_ore=");
            usb_->print(status.uart_overrun_errors);
            usb_->print(" tmag_dt=");
            usb_->print(status.tmag_sample_dt_us);
            usb_->print(" tmag_samples=");
            usb_->print(status.tmag_sample_count);
            usb_->print(" faults=");
            usb_->println(status.faults);
        }
        usb_->println("# pollfast_csv sample,t_us,sample_age_us,loop_us,fresh,valid,position_turn32,velocity_turn32_per_s,rpm");
        for (uint32_t i = 0u; i < log_count; i++) {
            int32_t rpm = (log[i].velocity_turn32_per_s * 60L) / 65536L;
            usb_->print("# pollfast_row ");
            usb_->print(i);
            usb_->print(",");
            usb_->print(log[i].t_us);
            usb_->print(",");
            usb_->print(log[i].sample_age_us);
            usb_->print(",");
            usb_->print(log[i].loop_us);
            usb_->print(",");
            usb_->print(log[i].fresh);
            usb_->print(",");
            usb_->print(log[i].valid);
            usb_->print(",");
            usb_->print(log[i].position_turn32);
            usb_->print(",");
            usb_->print(log[i].velocity_turn32_per_s);
            usb_->print(",");
            usb_->println(rpm);
        }
        usb_->println("# pollfast_done");
    }

    void bridgeEnter(uint8_t mode)
    {
        while (usb_->available() > 0) {
            usb_->read();
        }
        flushUartRx();
        bridge_mode_ = mode;
        pennyesc_bridge_active = true;

        if (mode == 4u) {
            beginApp();
            usb_->println("# bridge=handoff");
        } else if (mode == 3u) {
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
        if (strncmp(line, "rate", 4) == 0) {
            uint32_t duration_ms = 10000u;
            uint32_t hz = 500u;
            uint32_t spinup_ms = 0u;
            uint32_t timeout_ms = 5u;
            int duty = 0;
            sscanf(line + 4, "%lu %lu %d %lu %lu", &duration_ms, &hz, &duty, &spinup_ms, &timeout_ms);
            runRateTest(duration_ms, hz, (int16_t)duty, spinup_ms, timeout_ms);
            return;
        }
        if (strncmp(line, "pollfast", 8) == 0) {
            uint32_t duration_ms = 10000u;
            uint32_t hz = 600u;
            uint32_t spinup_ms = 0u;
            int duty = 0;
            sscanf(line + 8, "%lu %lu %d %lu", &duration_ms, &hz, &duty, &spinup_ms);
            runPollFastTest(duration_ms, hz, (int16_t)duty, spinup_ms);
            return;
        }
        if (strcmp(line, "bridge app") == 0) {
            bridgeEnter(1u);
            return;
        }
        if (strcmp(line, "bridge handoff") == 0) {
            bridgeEnter(4u);
            return;
        }
        if (strcmp(line, "bridge upload") == 0) {
            bridgeEnter(2u);
            return;
        }
        if (strcmp(line, "bridge rom") == 0) {
            bridgeEnter(3u);
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
    uint8_t bridge_mode_ = 0u;
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
