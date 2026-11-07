#include <Arduino.h>
#include "pennyesc_bridge.h"

PennyBridge bridge;

#define TX_PIN 13
#define RX_PIN 12
#define GND_PIN 11


void setup()
{
    Serial.begin(921600);
    bridge.begin(Serial, Serial1, RX_PIN, TX_PIN, GND_PIN);
}

void loop()
{
    bridge.poll();
}
