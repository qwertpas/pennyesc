#include <Arduino.h>
#include "pennyesc_arduino_debug.h"

static PennyEscDebug esc(1);

static void printCaptureSamples(const PennyEscCaptureStatus &capture)
{
    PennyEscCaptureSample samples[PNY_CAPTURE_READ_MAX_SAMPLES];

    Serial.println("index,t_ms,angle_turn16,angle_rad,rpm");
    for (uint16_t offset = 0; offset < capture.sample_count;) {
        uint16_t left = capture.sample_count - offset;
        uint8_t want = PNY_CAPTURE_READ_MAX_SAMPLES;
        if (left < want) {
            want = (uint8_t)left;
        }

        uint8_t got = 0;
        if (!esc.readCapture(offset, samples, want, got)) {
            Serial.println("# capture_read_failed");
            break;
        }

        for (uint8_t i = 0; i < got; i++) {
            uint16_t index = offset + i;
            float t_ms = ((float)index * 1000.0f) / (float)capture.sample_hz;
            Serial.printf(
                "%u,%.3f,%u,%.6f,%d\n",
                index,
                t_ms,
                samples[i].angle_turn16,
                samples[i].angleRad(),
                samples[i].rpm
            );
        }
        offset += got;
    }
}

static void runCapture()
{
    PennyEscCaptureStatus capture;

    if (!esc.startCapture(120, 90, 200, 1000, capture) || !capture.valid) {
        Serial.println("# capture_start_timeout");
        return;
    }
    if (capture.result != PNY_RESULT_OK) {
        Serial.printf("# capture_start_failed result=%u\n", capture.result);
        return;
    }

    while (capture.active) {
        delay(5);
        if (!esc.getCaptureStatus(capture)) {
            Serial.println("# capture_status_timeout");
            return;
        }
    }

    Serial.printf(
        "# capture_done samples=%u missed=%u hz=%u duration_ms=%u\n",
        capture.sample_count,
        capture.missed_count,
        capture.sample_hz,
        capture.duration_ms
    );
    printCaptureSamples(capture);
}

void setup()
{
    Serial.begin(115200);
    esc.begin(Serial1, 13, 12);
    delay(300);

    Serial.println("# pennyesc capture example");
    Serial.println("# send c to capture, s to stop");
}

void loop()
{
    while (Serial.available() > 0) {
        char ch = (char)Serial.read();
        if (ch == 'c') {
            runCapture();
        } else if (ch == 's') {
            esc.setDuty(0);
            Serial.println("# stopped");
        }
    }
}
