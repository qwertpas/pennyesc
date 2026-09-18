#include <Arduino.h>
#include "pennyesc_arduino.h"

PennyEscBridge bridge;

#define TX_PIN 13
#define RX_PIN 12
#define GND_PIN 11


void setup()
{
    Serial.begin(921600);
    pinMode(GND_PIN, OUTPUT);
    digitalWrite(GND_PIN, LOW);
    bridge.begin(Serial, Serial1, RX_PIN, TX_PIN);
}

void loop()
{
    bridge.poll();
}
