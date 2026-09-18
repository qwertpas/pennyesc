#include <Arduino.h>
#include "pennyesc_arduino.h"

#define RX_PIN 1
#define TX_PIN 2
#define GND_PIN 3
#define ESC_ADDRESS 7

static PennyEsc esc(ESC_ADDRESS);

void setup()
{
    Serial.begin(921600);
    pinMode(GND_PIN, OUTPUT);
    digitalWrite(GND_PIN, LOW);

    esc.begin(Serial1, RX_PIN, TX_PIN);
    delay(300);

    PennyEscStatus status;
    esc.zeroPosition(&status);
    esc.setControl(80.0f, 1.0f, 0.0f, 0, 60, &status);
}

void loop()
{
    static uint32_t last_move_ms;
    static uint32_t last_status_ms;
    static bool positive;
    uint32_t now = millis();

    if ((uint32_t)(now - last_move_ms) >= 1000u) {
        last_move_ms = now;
        positive = !positive;
        esc.sendPositionRad(positive ? 0.2f : -0.2f);
    }

    if ((uint32_t)(now - last_status_ms) >= 20u) {
        last_status_ms = now;
        PennyEscEncoderData data;
        if (esc.getPosVel(data, 5u)) {
            Serial.printf("pos=%.4f vel=%.2f rpm\n", data.positionRad(), data.velocityRpm());
        }
    }
    delay(1);
}
