#include <Arduino.h>
#include <BLEDevice.h>
#include "pennyesc_ble.h"

static PennyEscBleBridge bridge;

void setup()
{
    Serial.begin(115200);
    bridge.begin(Serial, Serial1, D8, D7);
    bridge.beginBle();
}

void loop()
{
    bridge.poll();
    delay(1);
}
