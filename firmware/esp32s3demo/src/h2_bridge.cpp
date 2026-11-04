#include <Arduino.h>
#include <BLEDevice.h>
#include "pennyesc_ble.h"

static PennyEscBleBridge bridge;

static const int RX_PIN = 0;
static const int TX_PIN = 1;
static const int GND_PIN = 2;

void setup()
{
    Serial.begin(115200);
    bridge.begin(Serial, Serial1, RX_PIN, TX_PIN, 1, GND_PIN);
    bridge.beginBle();
}

void loop()
{
    bridge.poll();
    delay(1);
}
