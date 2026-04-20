#include <Arduino.h>
#include "esc_address.h"
#include "pennyesc_arduino.h"

static PennyEsc esc(ESC0);

void setup()
{
    Serial.begin(115200);
    esc.begin(Serial1, PennyEscPins(), PENNYESC_BAUD_UPDATE);
    delay(100);
    Serial.println("pennyesc demo");
}

void loop()
{
    static uint32_t last_print_ms = 0u;
    PennyEscStatus status;

    if ((millis() - last_print_ms) < 250u) {
        return;
    }
    last_print_ms = millis();

    if (!esc.getStatus(status)) {
        Serial.println("status timeout");
        return;
    }

    Serial.printf(
        "pos=%.3f vel=%.2f duty=%d x=%d y=%d z=%d mode=%u\n",
        status.positionRad(),
        status.velocityRpm(),
        status.duty,
        status.x,
        status.y,
        status.z,
        status.mode
    );
}
