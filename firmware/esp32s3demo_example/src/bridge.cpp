#include <Arduino.h>
#include "pennyesc_arduino.h"

PennyEscBridge bridge;

#define RX_PIN 1
#define TX_PIN 2
#define GND_PIN 3


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
