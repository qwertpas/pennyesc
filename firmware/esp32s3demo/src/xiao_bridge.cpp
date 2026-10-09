#include <Arduino.h>
#include "pennyesc_arduino.h"

static PennyEscBridge bridge;

void setup()
{
    Serial.begin(115200);
    bridge.begin(Serial, Serial1, D8, D7);
}

void loop()
{
    bridge.poll();
}
