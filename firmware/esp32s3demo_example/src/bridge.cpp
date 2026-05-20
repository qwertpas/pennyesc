#include <Arduino.h>
#include "pennyesc_arduino_debug.h"

PennyEscDebugBridge bridge;

void setup()
{
    Serial.begin(921600);
    pinMode(43, OUTPUT);
    digitalWrite(43, LOW);
    bridge.begin(Serial, Serial1, 44, 1, 1);
}

void loop()
{
    bridge.poll();
}
