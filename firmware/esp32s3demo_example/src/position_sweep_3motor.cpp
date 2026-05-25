#include <Arduino.h>
#include "pennyesc_arduino.h"

#define RX_PIN 1
#define TX_PIN 2
#define GND_PIN 3

static PennyEsc esc7(7);
static PennyEsc esc8(8);
static PennyEsc esc9(9);
static PennyEsc *escs[] = {&esc7, &esc8, &esc9};
static const uint8_t ESC_ADDRESSES[] = {7, 8, 9};
static const float AMPLITUDE_RAD = 30.0f * PI;
static const uint32_t PERIOD_MS = 2000;
static const uint32_t PLOT_MS = 20;

void setup()
{
    Serial.begin(921600);
    pinMode(GND_PIN, OUTPUT);
    digitalWrite(GND_PIN, LOW);

    delay(1000);

    for (PennyEsc *esc : escs) {
        PennyEscStatus status;
        esc->begin(Serial1, RX_PIN, TX_PIN);
        esc->sendDuty(0);
        esc->zeroPosition(&status);
        float kp = 50.0f;
        float kd = 1.0f;
        float kv = 0.0f;
        int16_t kf = 0;
        int16_t clip = 100;
        esc->setControl(kp, kd, kv, kf, clip, &status);
    }
    delay(100);
    Serial.println("# ms,esc,target_rad,pos_rad,rpm");
}

void loop()
{
    static uint32_t last_plot_ms = 0;
    uint32_t now = millis();
    float targets[3];

    for (uint8_t i = 0; i < 3; i++) {
        float phase = 2.0f * PI * ((float)(now % PERIOD_MS) / (float)PERIOD_MS + (float)i / 3.0f);
        targets[i] = AMPLITUDE_RAD * sinf(phase);
        escs[i]->sendPositionRad(targets[i]);
    }

    if ((uint32_t)(now - last_plot_ms) >= PLOT_MS) {
        last_plot_ms = now;
        for (uint8_t i = 0; i < 3; i++) {
            PennyEscEncoderData data;
            if (escs[i]->getPosVel(data, 5)) {
                Serial.printf(
                    "%lu,%u,%.4f,%.4f,%.2f\n",
                    (unsigned long)now,
                    ESC_ADDRESSES[i],
                    targets[i],
                    data.positionRad(),
                    data.velocityRpm()
                );
            }
        }
    }
    delay(1);
}
