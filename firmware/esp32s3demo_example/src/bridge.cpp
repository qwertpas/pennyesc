#include <Arduino.h>
#include "pennyesc_arduino.h"

PennyEscBridge bridge;

void setup()
{
    Serial.begin(921600);
    bridge.begin(Serial, Serial1, 13, 12, 1);
}

void loop()
{
    bridge.poll();
}
