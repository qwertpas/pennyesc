#include <Arduino.h>
#include "pennyesc_arduino.h"

static PennyEscBridge bridge;

void setup()
{
    PennyEscPins pins;
    pins.rx = 13;
    pins.tx = 12;

    Serial.begin(115200);
    bridge.begin(Serial, Serial1, pins, PENNYESC_BAUD_FAST);
}

void loop()
{
    bridge.poll();
}
