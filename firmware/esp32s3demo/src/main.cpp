#include <Arduino.h>
#include "esc_address.h"

/* ============================================================================
 * Protocol Constants
 * ============================================================================ */
#define BLDC_RX 13
#define BLDC_TX 12
#define BLDC_GND 11
#define BLDC_BAUD 921600

#define START_BYTE 0xAA
#define CMD_SET_POSITION 0x01
#define CMD_SET_DUTY 0x02
#define CMD_POLL 0x03

#define RESPONSE_LEN 11

/* Status flags */
#define STATUS_POSITION_REACHED 0x01
#define STATUS_ERROR 0x02

//esc_address.h
#define TEST_ESC ESC1

/* ============================================================================
 * CRC-8 Lookup Table (polynomial 0x07, CRC-8/CCITT)
 * ============================================================================ */
static const uint8_t crc8_table[256] PROGMEM = {
    0x00,0x07,0x0E,0x09,0x1C,0x1B,0x12,0x15,0x38,0x3F,0x36,0x31,0x24,0x23,0x2A,0x2D,
    0x70,0x77,0x7E,0x79,0x6C,0x6B,0x62,0x65,0x48,0x4F,0x46,0x41,0x54,0x53,0x5A,0x5D,
    0xE0,0xE7,0xEE,0xE9,0xFC,0xFB,0xF2,0xF5,0xD8,0xDF,0xD6,0xD1,0xC4,0xC3,0xCA,0xCD,
    0x90,0x97,0x9E,0x99,0x8C,0x8B,0x82,0x85,0xA8,0xAF,0xA6,0xA1,0xB4,0xB3,0xBA,0xBD,
    0xC7,0xC0,0xC9,0xCE,0xDB,0xDC,0xD5,0xD2,0xFF,0xF8,0xF1,0xF6,0xE3,0xE4,0xED,0xEA,
    0xB7,0xB0,0xB9,0xBE,0xAB,0xAC,0xA5,0xA2,0x8F,0x88,0x81,0x86,0x93,0x94,0x9D,0x9A,
    0x27,0x20,0x29,0x2E,0x3B,0x3C,0x35,0x32,0x1F,0x18,0x11,0x16,0x03,0x04,0x0D,0x0A,
    0x57,0x50,0x59,0x5E,0x4B,0x4C,0x45,0x42,0x6F,0x68,0x61,0x66,0x73,0x74,0x7D,0x7A,
    0x89,0x8E,0x87,0x80,0x95,0x92,0x9B,0x9C,0xB1,0xB6,0xBF,0xB8,0xAD,0xAA,0xA3,0xA4,
    0xF9,0xFE,0xF7,0xF0,0xE5,0xE2,0xEB,0xEC,0xC1,0xC6,0xCF,0xC8,0xDD,0xDA,0xD3,0xD4,
    0x69,0x6E,0x67,0x60,0x75,0x72,0x7B,0x7C,0x51,0x56,0x5F,0x58,0x4D,0x4A,0x43,0x44,
    0x19,0x1E,0x17,0x10,0x05,0x02,0x0B,0x0C,0x21,0x26,0x2F,0x28,0x3D,0x3A,0x33,0x34,
    0x4E,0x49,0x40,0x47,0x52,0x55,0x5C,0x5B,0x76,0x71,0x78,0x7F,0x6A,0x6D,0x64,0x63,
    0x3E,0x39,0x30,0x37,0x22,0x25,0x2C,0x2B,0x06,0x01,0x08,0x0F,0x1A,0x1D,0x14,0x13,
    0xAE,0xA9,0xA0,0xA7,0xB2,0xB5,0xBC,0xBB,0x96,0x91,0x98,0x9F,0x8A,0x8D,0x84,0x83,
    0xDE,0xD9,0xD0,0xD7,0xC2,0xC5,0xCC,0xCB,0xE6,0xE1,0xE8,0xEF,0xFA,0xFD,0xF4,0xF3
};

//static const uint8_t wake_stream[255] PROGMEM = {0xFF};

static uint8_t crc8(const uint8_t* data, size_t len) {
    uint8_t crc = 0x00;
    while (len--) {
        crc = pgm_read_byte(&crc8_table[crc ^ *data++]);
    }
    return crc;
}

/* ============================================================================
 * Response Structure
 * ============================================================================ */
struct BLDCResponse {
    bool valid;
    uint8_t status;
    int32_t position_crad;   // centiradians
    int32_t velocity_crads;  // centiradians/second
    
    bool positionReached() { return status & STATUS_POSITION_REACHED; }
    bool hasError() { return status & STATUS_ERROR; }
    float positionRad() { return position_crad * 0.01f; }
    float velocityRadS() { return velocity_crads * 0.01f; }
    float velocityRPM() { return velocity_crads * 0.001591549f; }  // /(2π*100)*60
};

/* ============================================================================
 * Communication Statistics
 * ============================================================================ */
struct EscStats {
    uint32_t sent = 0;
    uint32_t received = 0;
    uint32_t crcErrors = 0;
    uint32_t timeouts = 0;
    uint32_t latencyUs = 0;
    uint32_t latencyMaxUs = 0;
    uint32_t latencyAvgUs = 0;
};

static EscStats stats[ESC_COUNT];

/* ============================================================================
 * Send Command and Wait for Response
 * ============================================================================ */
static BLDCResponse sendCommand(uint8_t address, uint8_t cmd, const uint8_t* payload, uint8_t payloadLen) {
    BLDCResponse resp = {false, 0, 0, 0};
    uint32_t start = micros();
    
    assert(address <= 0xF);

    /* Build and send packet */
    uint8_t txBuf[8];
    txBuf[0] = START_BYTE;
    txBuf[1] = (address << 4) | (cmd & 0xF);
    if (payload && payloadLen > 0) {
        memcpy(&txBuf[2], payload, payloadLen);
    }
    txBuf[2 + payloadLen] = crc8(txBuf, 2 + payloadLen);
    Serial1.write(txBuf, 3 + payloadLen);
    stats[address].sent++;
    
    /* Wait for response (11 bytes, timeout 5ms) */
    uint8_t rxBuf[RESPONSE_LEN];
    uint8_t rxIdx = 0;
    uint32_t timeout = millis() + 5;
    
    while (rxIdx < RESPONSE_LEN && millis() < timeout) {
        if (Serial1.available()) {
            uint8_t b = Serial1.read();
            
            /* Resync: START_BYTE always starts a new packet (handles mid-packet corruption) */
            if (b == START_BYTE) {
                rxIdx = 0;
                rxBuf[rxIdx++] = b;
                continue;
            }
            
            /* Not START_BYTE - only accept if we're mid-packet */
            if (rxIdx == 0) continue;
            
            rxBuf[rxIdx++] = b;
        }
    }
    
    if (rxIdx < RESPONSE_LEN) {
        stats[address].timeouts++;
        return resp;
    }
    
    /* Verify CRC */
    if (crc8(rxBuf, RESPONSE_LEN - 1) != rxBuf[RESPONSE_LEN - 1]) {
        stats[address].crcErrors++;
        return resp;
    }
    
    /* Parse response */
    resp.valid = true;
    resp.status = rxBuf[1];
    memcpy(&resp.position_crad, &rxBuf[2], 4);
    memcpy(&resp.velocity_crads, &rxBuf[6], 4);
    stats[address].received++;

    /* Update latency stats */
    uint32_t elapsed = micros() - start;
    stats[address].latencyUs = elapsed;
    if (elapsed > stats[address].latencyMaxUs) {
        stats[address].latencyMaxUs = elapsed;
    }
    stats[address].latencyAvgUs = (stats[address].latencyAvgUs * 7 + elapsed) / 8;  /* EMA */
    
    return resp;
}

/* ============================================================================
 * High-Level API Functions
 * ============================================================================ */
static BLDCResponse setPosition(uint8_t address, int32_t position_crad) {
    uint8_t payload[4];
    memcpy(payload, &position_crad, 4);
    return sendCommand(address, CMD_SET_POSITION, payload, 4);
}

static BLDCResponse setDuty(uint8_t address, int16_t duty) {
    uint8_t payload[2];
    memcpy(payload, &duty, 2);
    return sendCommand(address, CMD_SET_DUTY, payload, 2);
}

static BLDCResponse poll(uint8_t address) {
    return sendCommand(address, CMD_POLL, nullptr, 0);
}

/* Convenience: set position in radians */
static BLDCResponse setPositionRad(uint8_t address, float rad) {
    return setPosition(address, (int32_t)(rad * 100.0f));
}

//send a constant stream of high values over RX line to wake any connected ESCs
/*void wakeESC() {
    sendCommand(0xF, 0xF, wake_stream,sizeof(wake_stream)/sizeof(wake_stream[0]));
}*/

/* ============================================================================
 * Setup and Main Loop
 * ============================================================================ */
void setup() {
    pinMode(BLDC_GND, OUTPUT);
    digitalWrite(BLDC_GND, LOW);
    delay(100);

    Serial.begin(115200);  /* USB debug */
    Serial1.begin(BLDC_BAUD, SERIAL_8N1, BLDC_RX, BLDC_TX);
    
    delay(100);
    Serial.println("BLDC Controller Ready");
    Serial.println("Commands: d[duty], t[target rad], p[pulse duty], ?=stats");
}

BLDCResponse r[ESC_COUNT] = {};

void loop() {
    static uint32_t lastPollUs = 0;
    static uint32_t loopCount = 0;
    static uint32_t pulseEndTime = 0;
    static bool pulseActive = false;
    
    /* Handle pulse timeout */
    /*if (pulseActive && millis() >= pulseEndTime) {
        setDuty(TEST_ESC,0);
        pulseActive = false;
        Serial.println("Pulse finished: duty=0");
    }*/

    /* Poll at 1kHz using micros() for precise timing */
    uint32_t now = micros();
    if (now - lastPollUs >= 2000) {  /* 2000µs = 2ms = 500Hz */
        lastPollUs = now;
        loopCount++;
        
        //Poll every ESC
        for (int i = 0; i < ESC_COUNT; i++) {
            r[i] = poll(i);
        }
        
         /* Print stats every 100 loops (0.2 second at 500Hz) */
         //and try to wake any new devices
        /*if (loopCount % 100 == 0) { //&& r.valid) {
             //Serial.printf("Pos: %.2f rad, Vel: %.1f RPM | Latency: %lu/%lu/%lu us | Errs: %lu CRC, %lu TO\n",
                 r.positionRad(), r.velocityRPM(), 
                 stats.latencyUs, stats.latencyAvgUs, stats.latencyMaxUs,
                 stats.crcErrors, stats.timeouts);
            //wakeESC();
        }*/
    }

        /* Handle serial commands */
    static char inputBuffer[128];
    static uint8_t inputIdx = 0;
    
    while (Serial.available()) {
        char inChar = (char)Serial.read();
        if (inChar == '\n' || inChar == '\r' || inputIdx == sizeof(inputBuffer)/sizeof(inputBuffer[0]) - 1) {
            if (inputIdx > 0) {
                inputBuffer[inputIdx] = '\0';  /* Null-terminate */
                
                /* Parse command and arguments */
                char* token = strtok(inputBuffer, " \t");
                if (token != nullptr) {
                    char cmd = token[0];
                    char* arg1 = strtok(nullptr, " \t");
                    char* arg2 = strtok(nullptr, " \t");
                    
                    if (cmd == 'd' && arg1) {
                        int16_t duty = (int16_t)atoi(arg1);
                        uint8_t esc = arg2 ? (uint8_t)atoi(arg2) : TEST_ESC;
                        setDuty(esc, duty);
                        Serial.printf("Set Duty: %d on ESC %d\n", duty, esc);
                    } else if (cmd == 't' && arg1) {
                        float pos = atof(arg1);
                        uint8_t esc = arg2 ? (uint8_t)atoi(arg2) : TEST_ESC;
                        setPositionRad(esc, pos);
                        Serial.printf("Set Target Position: %.3f rad on ESC %d\n", pos, esc);
                    } else if (cmd == 'p' && arg1) {
                        int16_t pulseDuty = (int16_t)atoi(arg1);
                        uint8_t esc = arg2 ? (uint8_t)atoi(arg2) : TEST_ESC;
                        setDuty(esc, pulseDuty);
                        pulseEndTime = millis() + 500;
                        pulseActive = true;
                        Serial.printf("Pulse Started: duty=%d on ESC %d for 0.5s\n", pulseDuty, esc);
                    } else if (cmd == '?') {
                        Serial.printf("%s\n","============================");
                        for (int i = 0; i < ESC_COUNT; i++) {
                            Serial.printf("[ESC %d] Sent=%lu, Recv=%lu, CRC Err=%lu, Timeouts=%lu\n",
                                i, stats[i].sent, stats[i].received, stats[i].crcErrors, stats[i].timeouts);
                            Serial.printf("[ESC %d] Latency: cur=%lu us, avg=%lu us, max=%lu us\n",
                                i, stats[i].latencyUs, stats[i].latencyAvgUs, stats[i].latencyMaxUs);
                            Serial.printf("[ESC %d] Pos: %.2f rad, Vel: %.1f RPM\n",
                                i, r[i].positionRad(), r[i].velocityRPM());
                        }
                        Serial.printf("%s\n\n","============================");
                    } else if (cmd == 'h') {
                        Serial.println("Commands:");
                        Serial.println("  d <duty> [esc]    - Set duty cycle");
                        Serial.println("  t <position> [esc] - Set target position (rad)");
                        Serial.println("  p <duty> [esc]    - Pulse duty for 0.5s");
                        Serial.println("  ? - Show stats");
                    } else {
                        Serial.println("Unknown command. Use h for help");
                    }
                }
                inputIdx = 0;
            }
        } else if (inputIdx < sizeof(inputBuffer) - 1) {
            inputBuffer[inputIdx++] = inChar;
        }
    }
}
