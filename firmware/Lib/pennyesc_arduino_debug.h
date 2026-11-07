#ifndef PENNYESC_ARDUINO_DEBUG_H
#define PENNYESC_ARDUINO_DEBUG_H

#include <limits.h>
#include <stdio.h>
#include <string.h>
#include "pennyesc_arduino.h"

struct PennyEscCaptureSample {
    uint16_t angle_turn16 = 0;
    int16_t rpm = 0;

    float angleRad() const { return pennyEscTurn16ToRad(angle_turn16); }
};

struct PennyEscCaptureStatus {
    bool valid = false;
    uint8_t result = 0;
    bool active = false;
    bool done = false;
    uint16_t sample_hz = 0;
    uint16_t duration_ms = 0;
    uint16_t elapsed_ms = 0;
    uint16_t sample_count = 0;
    uint16_t missed_count = 0;
    uint16_t mct_fault_count = 0;
};

class PennyEscDebug : public PennyEsc {
public:
    PennyEscDebug(uint8_t address = 1) : PennyEsc(address) {}

    bool sendDebugCommand(
        const void *payload,
        uint8_t payload_len,
        uint8_t *response,
        uint8_t &response_len,
        uint32_t timeout_ms = 200u
    )
    {
        uint8_t frame[PNY_FRAME_MAX_PAYLOAD + 4u];
        uint8_t frame_len = 0u;

        response_len = 0u;
        if (response == 0 || !sendFrame(PNY_CMD_DEBUG, payload, payload_len)) {
            return false;
        }
        if (!readFrame(PNY_CMD_DEBUG, frame, sizeof(frame), frame_len, timeout_ms)) {
            return false;
        }
        if (frame_len < 4u || frame[2] > PNY_FRAME_MAX_PAYLOAD) {
            return false;
        }
        memcpy(response, &frame[3], frame[2]);
        response_len = frame[2];
        return true;
    }

    bool startCapture(
        int16_t duty,
        int16_t advance_deg,
        uint16_t duration_ms,
        uint16_t sample_hz,
        PennyEscCaptureStatus &status,
        uint32_t timeout_ms = 200u
    )
    {
        pny_capture_start_payload_t request;
        uint8_t response[PNY_FRAME_MAX_PAYLOAD];
        uint8_t response_len = 0u;

        status.valid = false;
        request.subcmd = PNY_DEBUG_CAPTURE_START;
        request.duty = duty;
        request.advance_deg = advance_deg;
        request.duration_ms = duration_ms;
        request.sample_hz = sample_hz;
        if (!sendDebugCommand(&request, sizeof(request), response, response_len, timeout_ms)) {
            return false;
        }
        return parseCaptureStatus(response, response_len, status);
    }

    bool getCaptureStatus(PennyEscCaptureStatus &status, uint32_t timeout_ms = 200u)
    {
        uint8_t request = PNY_DEBUG_CAPTURE_STATUS;
        uint8_t response[PNY_FRAME_MAX_PAYLOAD];
        uint8_t response_len = 0u;

        status.valid = false;
        if (!sendDebugCommand(&request, sizeof(request), response, response_len, timeout_ms)) {
            return false;
        }
        return parseCaptureStatus(response, response_len, status);
    }

    bool readCapture(
        uint16_t offset,
        PennyEscCaptureSample *samples,
        uint8_t count,
        uint8_t &got,
        uint32_t timeout_ms = 200u
    )
    {
        uint8_t request[4];
        uint8_t response[PNY_FRAME_MAX_PAYLOAD];
        uint8_t response_len = 0u;

        got = 0u;
        if (samples == 0 || count == 0u || count > PNY_CAPTURE_READ_MAX_SAMPLES) {
            return false;
        }
        request[0] = PNY_DEBUG_CAPTURE_READ;
        memcpy(&request[1], &offset, sizeof(offset));
        request[3] = count;
        if (!sendDebugCommand(request, sizeof(request), response, response_len, timeout_ms)) {
            return false;
        }
        return parseCaptureRead(offset, response, response_len, samples, got);
    }

    bool setObserver(
        int16_t lead_us,
        uint8_t mode,
        PennyEscCaptureStatus *status = 0,
        uint32_t timeout_ms = 200u
    )
    {
        pny_observer_payload_t request;
        uint8_t response[PNY_FRAME_MAX_PAYLOAD];
        uint8_t response_len = 0u;
        PennyEscCaptureStatus local_status;

        request.subcmd = PNY_DEBUG_SET_OBSERVER;
        request.lead_us = lead_us;
        request.mode = mode;
        if (!sendDebugCommand(&request, sizeof(request), response, response_len, timeout_ms)) {
            return false;
        }
        if (!parseCaptureStatus(response, response_len, local_status)) {
            return false;
        }
        if (status != 0) {
            *status = local_status;
        }
        return local_status.result == PNY_RESULT_OK;
    }

private:
    static bool parseCaptureStatus(const uint8_t *response, uint8_t response_len, PennyEscCaptureStatus &status)
    {
        pny_capture_status_payload_t payload;

        status.valid = false;
        if (response_len != sizeof(payload)) {
            return false;
        }
        memcpy(&payload, response, sizeof(payload));
        if (payload.subcmd != PNY_DEBUG_CAPTURE_STATUS) {
            return false;
        }
        status.valid = true;
        status.result = payload.result;
        status.active = payload.active != 0u;
        status.done = payload.done != 0u;
        status.sample_hz = payload.sample_hz;
        status.duration_ms = payload.duration_ms;
        status.elapsed_ms = payload.elapsed_ms;
        status.sample_count = payload.sample_count;
        status.missed_count = payload.missed_count;
        status.mct_fault_count = payload.mct_fault_count;
        return true;
    }

    static bool parseCaptureRead(
        uint16_t offset,
        const uint8_t *response,
        uint8_t response_len,
        PennyEscCaptureSample *samples,
        uint8_t &got
    )
    {
        uint16_t response_offset;
        uint8_t count;

        if (response_len < 5u || response[0] != PNY_DEBUG_CAPTURE_READ || response[1] != PNY_RESULT_OK) {
            return false;
        }
        memcpy(&response_offset, &response[2], sizeof(response_offset));
        count = response[4];
        if (response_offset != offset || count > PNY_CAPTURE_READ_MAX_SAMPLES) {
            return false;
        }
        if (response_len != (uint8_t)(5u + (count * sizeof(pny_capture_sample_t)))) {
            return false;
        }
        for (uint8_t i = 0u; i < count; i++) {
            pny_capture_sample_t sample;
            memcpy(&sample, &response[5u + (i * sizeof(sample))], sizeof(sample));
            samples[i].angle_turn16 = sample.angle_turn16;
            samples[i].rpm = sample.rpm;
        }
        got = count;
        return true;
    }
};

#endif
