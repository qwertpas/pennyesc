#include <Arduino.h>
#include "pennyesc_arduino.h"

static PennyEsc esc(2);
static PennyEscStatus status;

void setup()
{
    Serial.begin(115200);
    esc.begin(Serial1, 13, 12);
    delay(300);

    esc.zeroPosition(&status);
    esc.setControl(100.0f, 0.10f, 0.0f, 0, 100, &status);

    Serial.println("position sweep");
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

    // Print status every 50ms
    if (millis() - last_status_ms >= 50) {
        last_status_ms = millis();
        esc.getStatus(status);
        Serial.printf("pos=%.3f vel=%.2f duty=%d\n",
            status.positionRad(),
            status.velocityRpm(),
            status.duty
        );
    }
    
    delay(1);
}
