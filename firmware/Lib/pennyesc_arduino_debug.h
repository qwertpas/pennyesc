#ifndef PENNYESC_ARDUINO_DEBUG_H
#define PENNYESC_ARDUINO_DEBUG_H

#include <limits.h>
#include <stdio.h>
#include <string.h>
#include "pennyesc_arduino.h"

static const uint32_t PENNYESC_RATE_LOG_MAX = 2400u;

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

class PennyEscDebugBridge : public PennyEscBridge {
protected:
    void printHelp() override
    {
        PennyEscBridge::printHelp();
        usb_->println("debug commands: rate <ms> <hz> <duty> <spinup_ms> <timeout_ms> pollfast <ms> <hz> <duty> <spinup_ms>");
    }

    bool handleDebugLine(const char *line) override
    {
        if (strncmp(line, "rate", 4) == 0) {
            uint32_t duration_ms = 10000u;
            uint32_t hz = 500u;
            uint32_t spinup_ms = 0u;
            uint32_t timeout_ms = 5u;
            int duty = 0;
            sscanf(line + 4, "%lu %lu %d %lu %lu", &duration_ms, &hz, &duty, &spinup_ms, &timeout_ms);
            runRateTest(duration_ms, hz, (int16_t)duty, spinup_ms, timeout_ms);
            return true;
        }
        if (strncmp(line, "pollfast", 8) == 0) {
            uint32_t duration_ms = 10000u;
            uint32_t hz = 600u;
            uint32_t spinup_ms = 0u;
            int duty = 0;
            sscanf(line + 8, "%lu %lu %d %lu", &duration_ms, &hz, &duty, &spinup_ms);
            runPollFastTest(duration_ms, hz, (int16_t)duty, spinup_ms);
            return true;
        }
        return false;
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

};

#endif
