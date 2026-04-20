#include <Arduino.h>
#include "pennyesc_arduino.h"

static PennyEscBridge bridge;

void setup()
{
    Serial.begin(115200);
    bridge.begin(Serial, Serial1, PennyEscPins(), PENNYESC_BAUD_UPDATE);
}

void loop()
{
    bridge.poll();
}
