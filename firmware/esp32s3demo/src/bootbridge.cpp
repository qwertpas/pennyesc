#include <Arduino.h>
#include "pennyesc_arduino_debug.h"

static PennyEscDebugBridge bridge;

void setup()
{
    Serial.begin(115200);
    bridge.begin(Serial, Serial1, 12, 13);
}

void loop()
{
    bridge.poll();
}
