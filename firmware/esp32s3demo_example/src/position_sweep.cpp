#include <Arduino.h>
#include "pennyesc_arduino.h"

#define RX_PIN 1
#define TX_PIN 2
#define GND_PIN 3

#define ESC_ADDRESS 7

static PennyEsc esc(ESC_ADDRESS);
static PennyEscStatus status;

void setup()
{
    Serial.begin(921600);

    pinMode(GND_PIN, OUTPUT);
    digitalWrite(GND_PIN, LOW);

    esc.begin(Serial1, RX_PIN, TX_PIN);

    delay(300);

    esc.zeroPosition(&status);

    float kp = 100.0f;
    float kd = 1.0f;
    float kv = 0.0f;
    int16_t kf = 0;
    int16_t clip = 100;
    esc.setControl(kp, kd, kv, kf, clip, &status);
}

static uint32_t last_status_ms = 0;
static uint32_t last_sweep_ms = 0;
void loop()
{

    // Change position every 500ms
    if (millis() - last_sweep_ms < 500) {
        esc.sendPositionRad(3.0f * 2*PI);
    } else if (millis() - last_sweep_ms < 1000) {
        esc.sendPositionRad(0.0f);
    } else {
        last_sweep_ms = millis();
    }

    // Print status every 10ms (100Hz)
    if (millis() - last_status_ms >= 10) {
        last_status_ms = millis();
        PennyEscEncoderData data;
        if (esc.getPosVel(data, 5)) { //5ms timeout
            float pos = data.positionRad();
            float rpm = data.velocityRpm();
            Serial.printf("pos=%.3f vel=%.2f\n", pos, rpm);
        }
    }
    
    delay(1);
}
