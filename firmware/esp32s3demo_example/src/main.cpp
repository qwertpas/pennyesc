#include <Arduino.h>
#include "pennyesc_arduino.h"

PennyEscBridge bridge;

void setup()
{
    PennyEscPins pins;
    pins.rx = 13; // UART Serial1: PennyESC to ESP32
    pins.tx = 12;
    bridge.begin(Serial, Serial1, pins, PENNYESC_BAUD_FAST, true, false, false);
    bridge.setAddress(2);

    Serial.begin(115200); // Serial monitor to your laptop
}

void loop()
{
    bridge.poll();

    /*
    In serial monitor:
        s       status
        d100    set duty
        d0      stop
        e1      change to ESC 2 and verify it responds
    */
}
