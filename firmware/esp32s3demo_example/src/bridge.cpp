#include <Arduino.h>
#include "pennyesc_arduino.h"

PennyEscBridge bridge;

void setup()
{
    Serial.begin(115200);
    bridge.begin(Serial, Serial1, 13, 12, 2);
}

void loop()
{
    bridge.poll();
}
