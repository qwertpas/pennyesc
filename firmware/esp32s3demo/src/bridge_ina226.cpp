#include <Arduino.h>
#include <Wire.h>
#include "pennyesc_arduino.h"

static const int ESC_RX_PIN = 13;
static const int ESC_TX_PIN = 12;
static const int INA_GND_PIN = 7;
static const int INA_PWR_PIN = 6;
static const int INA_SDA_PIN = 5;
static const int INA_SCL_PIN = 4;
static const uint8_t INA_ADDR = 0x40;
static const float SHUNT_OHMS = 0.1f;
static const float CURRENT_LSB_A = 0.0001f;

static PennyEscBridge bridge;

static bool writeReg(uint8_t reg, uint16_t value)
{
    Wire.beginTransmission(INA_ADDR);
    Wire.write(reg);
    Wire.write((uint8_t)(value >> 8));
    Wire.write((uint8_t)value);
    return Wire.endTransmission() == 0;
}

static bool readReg(uint8_t reg, uint16_t &value)
{
    Wire.beginTransmission(INA_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) {
        return false;
    }
    if (Wire.requestFrom((int)INA_ADDR, 2) != 2) {
        return false;
    }
    value = (uint16_t)Wire.read() << 8;
    value |= (uint16_t)Wire.read();
    return true;
}

static bool initIna()
{
    uint16_t cal = (uint16_t)(0.00512f / (CURRENT_LSB_A * SHUNT_OHMS));
    return writeReg(0x00, 0x4927) && writeReg(0x05, cal);
}

static void printIna()
{
    uint16_t raw_shunt = 0;
    uint16_t raw_bus = 0;
    uint16_t raw_power = 0;
    uint16_t raw_current = 0;

    if (!readReg(0x01, raw_shunt) || !readReg(0x02, raw_bus) ||
        !readReg(0x03, raw_power) || !readReg(0x04, raw_current)) {
        Serial.println("# ina226 read fail");
        return;
    }

    float shunt_v = (float)(int16_t)raw_shunt * 0.0000025f;
    float bus_v = (float)raw_bus * 0.00125f;
    float shunt_current_a = shunt_v / SHUNT_OHMS;
    float current_a = (float)(int16_t)raw_current * CURRENT_LSB_A;
    float power_w = (float)raw_power * CURRENT_LSB_A * 25.0f;

    Serial.printf(
        "# ina226 bus=%.4fV shunt=%.6fV current=%.4fA shunt_current=%.4fA power=%.4fW\n",
        bus_v,
        shunt_v,
        current_a,
        shunt_current_a,
        power_w
    );
}

void setup()
{
    Serial.begin(115200);

    pinMode(INA_GND_PIN, OUTPUT);
    digitalWrite(INA_GND_PIN, LOW);
    pinMode(INA_PWR_PIN, OUTPUT);
    digitalWrite(INA_PWR_PIN, HIGH);
    delay(20);

    Wire.begin(INA_SDA_PIN, INA_SCL_PIN);
    Wire.setClock(100000);

    bridge.begin(Serial, Serial1, ESC_RX_PIN, ESC_TX_PIN, 2);
    delay(100);

    Serial.println("# bridge_ina226");
    if (initIna()) {
        Serial.println("# ina226 ok");
    } else {
        Serial.println("# ina226 init fail");
    }
}

void loop()
{
    static uint32_t last_ms = 0;

    bridge.poll();

    if (!pennyesc_bridge_active && (uint32_t)(millis() - last_ms) >= 500u) {
        last_ms = millis();
        printIna();
    }

    delay(1);
}
