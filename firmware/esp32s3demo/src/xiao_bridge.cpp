#include <Arduino.h>
#include <BLEDevice.h>  // lets PlatformIO find the BLE library
#include "pennyesc_ble.h"

static PennyBleBridge bridge;

void setup()
{
    // Large buffers let a host send a whole bootloader write in one packet.
    Serial.setRxBufferSize(1024);
    Serial.setTxBufferSize(4096);
    Serial.begin(115200);
    Serial.setTxTimeoutMs(0);
    bridge.begin(Serial, Serial1, D8, D7);
    bridge.beginBle();
}

void loop()
{
    bridge.poll();
    delay(1);
}
