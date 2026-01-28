// Dual Motor Dyno: PennyESC + Vertiq with WiFi/UDP control
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <iq_module_communication.hpp>

/* ============================================================================
 * Pin Configuration
 * ============================================================================ */
// PennyESC on Serial1
#define PENNY_RX 13
#define PENNY_TX 12
#define PENNY_GND 11
#define PENNY_BAUD 921600

// Vertiq on Serial2
#define VERTIQ_TX 10
#define VERTIQ_RX 9
#define VERTIQ_GND 8
#define VERTIQ_BAUD 921600

#define LED_PIN 48

/* ============================================================================
 * PennyESC Protocol Constants
 * ============================================================================ */
#define START_BYTE 0xAA
#define CMD_SET_POSITION 0x01
#define CMD_SET_DUTY 0x02
#define CMD_POLL 0x03
#define RESPONSE_LEN 11

#define STATUS_POSITION_REACHED 0x01
#define STATUS_ERROR 0x02

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

static uint8_t crc8(const uint8_t* data, size_t len) {
    uint8_t crc = 0x00;
    while (len--) {
        crc = pgm_read_byte(&crc8_table[crc ^ *data++]);
    }
    return crc;
}

/* ============================================================================
 * Base64 Encoding
 * ============================================================================ */
static const char base64_chars[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

size_t base64_encode(const uint8_t *input, size_t inputLen, char *output) {
    size_t i = 0, j = 0;
    uint8_t arr3[3], arr4[4];
    size_t outputLen = 0;
    
    while (inputLen--) {
        arr3[i++] = *(input++);
        if (i == 3) {
            arr4[0] = (arr3[0] & 0xfc) >> 2;
            arr4[1] = ((arr3[0] & 0x03) << 4) + ((arr3[1] & 0xf0) >> 4);
            arr4[2] = ((arr3[1] & 0x0f) << 2) + ((arr3[2] & 0xc0) >> 6);
            arr4[3] = arr3[2] & 0x3f;
            for (i = 0; i < 4; i++) {
                output[outputLen++] = base64_chars[arr4[i]];
            }
            i = 0;
        }
    }
    
    if (i) {
        for (j = i; j < 3; j++) arr3[j] = '\0';
        arr4[0] = (arr3[0] & 0xfc) >> 2;
        arr4[1] = ((arr3[0] & 0x03) << 4) + ((arr3[1] & 0xf0) >> 4);
        arr4[2] = ((arr3[1] & 0x0f) << 2) + ((arr3[2] & 0xc0) >> 6);
        for (j = 0; j < i + 1; j++) {
            output[outputLen++] = base64_chars[arr4[j]];
        }
        while (i++ < 3) {
            output[outputLen++] = '=';
        }
    }
    
    output[outputLen] = '\0';
    return outputLen;
}

/* ============================================================================
 * Loop Timing
 * ============================================================================ */
const uint32_t LOOP_FREQ_HZ = 500;
const uint32_t LOOP_PERIOD_US = 1000000 / LOOP_FREQ_HZ;  // 2000µs = 2ms

/* ============================================================================
 * Logging Buffer - Combined data from both motors
 * ============================================================================ */
const uint32_t LOG_BUFFER_SIZE = 6000;  // 12 seconds at 500Hz

struct LogSample {
    // Vertiq data
    float v_angle;
    float v_vel;
    float v_vbat;
    float v_set_volts;
    // PennyESC data
    float p_position;    // radians
    float p_velocity;    // rad/s
    // Timing
    uint32_t time_us;
};  // 28 bytes per sample

LogSample logBuffer[LOG_BUFFER_SIZE];
uint32_t logHead = 0;
uint32_t logCount = 0;
uint32_t logStartTime = 0;
uint32_t logStopTime = 0;
bool logging = false;
bool logReady = false;

/* ============================================================================
 * PennyESC Response Structure
 * ============================================================================ */
struct PennyResponse {
    bool valid;
    uint8_t status;
    int32_t position_crad;
    int32_t velocity_crads;
    
    bool positionReached() { return status & STATUS_POSITION_REACHED; }
    bool hasError() { return status & STATUS_ERROR; }
    float positionRad() { return position_crad * 0.01f; }
    float velocityRadS() { return velocity_crads * 0.01f; }
};

/* ============================================================================
 * PennyESC Communication
 * ============================================================================ */
static PennyResponse pennySendCommand(uint8_t cmd, const uint8_t* payload, uint8_t payloadLen) {
    PennyResponse resp = {false, 0, 0, 0};
    
    // Build and send packet
    uint8_t txBuf[8];
    txBuf[0] = START_BYTE;
    txBuf[1] = cmd;
    if (payload && payloadLen > 0) {
        memcpy(&txBuf[2], payload, payloadLen);
    }
    txBuf[2 + payloadLen] = crc8(txBuf, 2 + payloadLen);
    Serial1.write(txBuf, 3 + payloadLen);
    
    // Wait for response (11 bytes, timeout 2ms)
    uint8_t rxBuf[RESPONSE_LEN];
    uint8_t rxIdx = 0;
    uint32_t timeout = millis() + 2;
    
    while (rxIdx < RESPONSE_LEN && millis() < timeout) {
        if (Serial1.available()) {
            uint8_t b = Serial1.read();
            if (b == START_BYTE) {
                rxIdx = 0;
                rxBuf[rxIdx++] = b;
                continue;
            }
            if (rxIdx == 0) continue;
            rxBuf[rxIdx++] = b;
        }
    }
    
    if (rxIdx < RESPONSE_LEN) return resp;
    if (crc8(rxBuf, RESPONSE_LEN - 1) != rxBuf[RESPONSE_LEN - 1]) return resp;
    
    resp.valid = true;
    resp.status = rxBuf[1];
    memcpy(&resp.position_crad, &rxBuf[2], 4);
    memcpy(&resp.velocity_crads, &rxBuf[6], 4);
    return resp;
}

static PennyResponse pennySetDuty(int16_t duty) {
    uint8_t payload[2];
    memcpy(payload, &duty, 2);
    return pennySendCommand(CMD_SET_DUTY, payload, 2);
}

static PennyResponse pennySetPosition(int32_t position_crad) {
    uint8_t payload[4];
    memcpy(payload, &position_crad, 4);
    return pennySendCommand(CMD_SET_POSITION, payload, 4);
}

static PennyResponse pennySetPositionRad(float rad) {
    return pennySetPosition((int32_t)(rad * 100.0f));
}

static PennyResponse pennyPoll() {
    return pennySendCommand(CMD_POLL, nullptr, 0);
}

/* ============================================================================
 * Vertiq Motor Setup (IQ Module Communication)
 * ============================================================================ */
GenericInterface vertiqCom;
PowerMonitorClient power(0);
BrushlessDriveClient mot(0);
MultiTurnAngleControlClient multi(0);

void vertiqSend() {
    uint8_t buf[128];
    uint8_t len;
    if (vertiqCom.GetTxBytes(buf, len)) {
        Serial2.write(buf, len);
    }
}

void vertiqRead() {
    uint8_t buf[128];
    uint8_t len;
    while (Serial2.available()) {
        len = Serial2.readBytes(buf, min((int)sizeof(buf), Serial2.available()));
        vertiqCom.SetRxBytes(buf, len);
        
        uint8_t *packet;
        uint8_t pLen;
        while (vertiqCom.PeekPacket(&packet, &pLen)) {
            multi.ReadMsg(packet, pLen);
            power.ReadMsg(packet, pLen);
            vertiqCom.DropPacket();
        }
    }
}

/* ============================================================================
 * Wi-Fi SoftAP + UDP
 * ============================================================================ */
WiFiUDP udp;
const char *SSID = "vertiqdyno";
const uint16_t PORT = 9870;

void udpReply(const String &message) {
    udp.beginPacket(udp.remoteIP(), udp.remotePort());
    udp.write((const uint8_t *)message.c_str(), message.length());
    udp.endPacket();
}

// State for LOG command flow
bool awaitingLogConfirm = false;
IPAddress logRequesterIP;
uint16_t logRequesterPort;

/* ============================================================================
 * Motor State Variables
 * ============================================================================ */
// Vertiq state
float v_raw_angle = 0;
float v_zero_angle = 0;
float v_angle = 0;
float v_target = 0;
bool v_spinning = false;
float v_start_err = 1;
float v_set_voltage = 0.2f;
float v_vbat = 0;
float v_vel = 0;

// PennyESC state
float p_zero_angle = 0;
float p_angle = 0;
float p_vel = 0;
int16_t p_duty = 0;

// Chained command queue (Vertiq only)
enum ChainCmdType { CHAIN_VOLTAGE, CHAIN_TARGET };
struct ChainCmd { ChainCmdType type; float value; };
const int MAX_CHAIN_LEN = 10;
ChainCmd chainQueue[MAX_CHAIN_LEN];
int chainLen = 0;
int chainIdx = 0;

bool parseChain(const String &cmd) {
    chainLen = 0;
    chainIdx = 0;
    int pos = 0;
    int len = cmd.length();
    
    while (pos < len && chainLen < MAX_CHAIN_LEN) {
        char c = cmd.charAt(pos);
        if (c == 'V' || c == 'v') {
            int numStart = pos + 1;
            int numEnd = numStart;
            while (numEnd < len) {
                char nc = cmd.charAt(numEnd);
                if ((nc >= '0' && nc <= '9') || nc == '.' || nc == '-') {
                    numEnd++;
                } else {
                    break;
                }
            }
            if (numEnd > numStart) {
                chainQueue[chainLen].type = CHAIN_VOLTAGE;
                chainQueue[chainLen].value = cmd.substring(numStart, numEnd).toFloat();
                chainLen++;
            }
            pos = numEnd;
        } else if (c == 'T' || c == 't') {
            int numStart = pos + 1;
            int numEnd = numStart;
            while (numEnd < len) {
                char nc = cmd.charAt(numEnd);
                if ((nc >= '0' && nc <= '9') || nc == '.' || nc == '-') {
                    numEnd++;
                } else {
                    break;
                }
            }
            if (numEnd > numStart) {
                chainQueue[chainLen].type = CHAIN_TARGET;
                chainQueue[chainLen].value = cmd.substring(numStart, numEnd).toFloat();
                chainLen++;
            }
            pos = numEnd;
        } else {
            pos++;
        }
    }
    return chainLen > 0;
}

// Control mode (Vertiq)
enum ControlMode { MODE_VOLTAGE, MODE_VELOCITY };
ControlMode control_mode = MODE_VOLTAGE;
float v_set_velocity = 20.0f;

// Homing state (Vertiq)
bool homing = false;
ControlMode pre_home_mode = MODE_VOLTAGE;
float pre_home_velocity = 20.0f;

uint32_t lastLoopTime = 0;
uint32_t lastVertiqUartUs = 0;
uint32_t lastPennyUartUs = 0;

/* ============================================================================
 * Send Log Data
 * ============================================================================ */
void sendLogData() {
    // Each LogSample is 28 bytes
    // Send in chunks of 21 samples = 588 bytes raw -> 784 bytes base64
    const uint32_t SAMPLES_PER_CHUNK = 21;
    char base64Chunk[801];
    LogSample tempChunk[SAMPLES_PER_CHUNK];
    
    uint32_t totalSamples = min(logCount, LOG_BUFFER_SIZE);
    uint32_t startIdx = (logCount >= LOG_BUFFER_SIZE) ? logHead : 0;
    
    uint32_t samplesSent = 0;
    int chunkNum = 0;
    
    while (samplesSent < totalSamples) {
        uint32_t samplesToProcess = min(SAMPLES_PER_CHUNK, totalSamples - samplesSent);
        
        for (uint32_t i = 0; i < samplesToProcess; i++) {
            uint32_t idx = (startIdx + samplesSent + i) % LOG_BUFFER_SIZE;
            tempChunk[i] = logBuffer[idx];
        }
        
        size_t bytesToProcess = samplesToProcess * sizeof(LogSample);
        base64_encode((uint8_t *)tempChunk, bytesToProcess, base64Chunk);
        
        udp.beginPacket(logRequesterIP, logRequesterPort);
        udp.write((uint8_t *)base64Chunk, strlen(base64Chunk));
        udp.endPacket();
        
        samplesSent += samplesToProcess;
        chunkNum++;
        delay(8);
    }
    
    delay(20);
    udpReply("END\n");
    
    Serial.printf("Sent %lu samples in %d chunks\n", samplesSent, chunkNum);
}

/* ============================================================================
 * Setup
 * ============================================================================ */
void setup() {
    setCpuFrequencyMhz(240);

    Serial.begin(115200);
    
    // PennyESC GND pin
    pinMode(PENNY_GND, OUTPUT);
    digitalWrite(PENNY_GND, LOW);
    
    // Vertiq GND pin
    pinMode(VERTIQ_GND, OUTPUT);
    digitalWrite(VERTIQ_GND, LOW);
    
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
    
    delay(1000);
    Serial.println("\nDual Motor Dyno: PennyESC + Vertiq");

    // Wi-Fi Access Point
    WiFi.mode(WIFI_AP);
    WiFi.softAP(SSID, nullptr, 6);
    IPAddress ip = WiFi.softAPIP();
    Serial.printf("SoftAP '%s' active\n", SSID);
    Serial.printf("ESP32 IP: %s\n", ip.toString().c_str());

    // UDP listener
    udp.begin(PORT);
    Serial.printf("UDP port %d\n", PORT);

    // PennyESC on Serial1
    Serial1.begin(PENNY_BAUD, SERIAL_8N1, PENNY_RX, PENNY_TX);
    Serial.println("PennyESC on Serial1 (RX=13, TX=12)");

    // Vertiq on Serial2
    Serial2.begin(VERTIQ_BAUD, SERIAL_8N1, VERTIQ_RX, VERTIQ_TX);
    multi.ctrl_volts_.set(vertiqCom, 0.0f);
    vertiqSend();
    Serial.println("Vertiq on Serial2 (RX=9, TX=10)");
    
    Serial.printf("Log buffer: %lu samples (%lu bytes)\n", LOG_BUFFER_SIZE, LOG_BUFFER_SIZE * sizeof(LogSample));
}

/* ============================================================================
 * Main Loop
 * ============================================================================ */
long loopcount = 0;

void loop() {
    uint32_t now = micros();
    if (now - lastLoopTime < LOOP_PERIOD_US) {
        return;
    }
    lastLoopTime = now;
    loopcount++;
    
    // ---- Poll PennyESC ----
    uint32_t pennyStart = micros();
    PennyResponse pennyResp = pennyPoll();
    lastPennyUartUs = micros() - pennyStart;
    
    if (pennyResp.valid) {
        p_angle = pennyResp.positionRad() - p_zero_angle;
        p_vel = pennyResp.velocityRadS();
    }
    
    // ---- Vertiq: Request sensor data ----
    power.volts_.get(vertiqCom);
    multi.obs_angular_velocity_.get(vertiqCom);
    multi.obs_angular_displacement_.get(vertiqCom);
    
    float v_error = v_target - v_angle;
    float v_current_set_volts = 0;

    // ---- Vertiq control logic ----
    if (v_spinning && v_start_err * v_error > 0) {
        if (control_mode == MODE_VOLTAGE) {
            v_current_set_volts = (v_error > 0) ? v_set_voltage : -v_set_voltage;
            multi.ctrl_volts_.set(vertiqCom, v_current_set_volts);
        } else {
            float vel_cmd = (v_error > 0) ? v_set_velocity : -v_set_velocity;
            multi.ctrl_velocity_.set(vertiqCom, vel_cmd);
        }
        digitalWrite(LED_PIN, HIGH);
    } else {
        // Target reached or not spinning - check for chained commands
        bool chainContinued = false;
        if (v_spinning && chainIdx < chainLen) {
            while (chainIdx < chainLen) {
                ChainCmd &cmd = chainQueue[chainIdx];
                chainIdx++;
                if (cmd.type == CHAIN_VOLTAGE) {
                    v_set_voltage = constrain(cmd.value, 0, v_vbat);
                    Serial.printf("CHAIN: V%.2f\n", v_set_voltage);
                } else if (cmd.type == CHAIN_TARGET) {
                    v_target = constrain(cmd.value, -188, 188);
                    v_start_err = v_target - v_angle;
                    Serial.printf("CHAIN: T%.1f (err %.1f)\n", v_target, v_start_err);
                    chainContinued = true;
                    break;
                }
            }
        }
        
        if (!chainContinued) {
            bool wasHoming = homing;
            if (homing) {
                control_mode = pre_home_mode;
                v_set_velocity = pre_home_velocity;
                homing = false;
                Serial.println("HOME COMPLETE");
            }
            
            if (control_mode == MODE_VELOCITY && !wasHoming) {
                multi.ctrl_angle_.set(vertiqCom, v_target + v_zero_angle);
            } else {
                multi.ctrl_volts_.set(vertiqCom, 0.0f);
                multi.ctrl_brake_.set(vertiqCom);
            }

            if (v_spinning && logging && logStopTime == 0) {
                logStopTime = now + 100000;  // 0.1s after target reached
            }
            v_spinning = false;
            chainLen = 0;
            chainIdx = 0;
            digitalWrite(LED_PIN, LOW);
        }
    }
    
    // Stop logging after post-target delay
    if (logStopTime != 0 && (int32_t)(now - logStopTime) >= 0) {
        logging = false;
        logReady = true;
        logStopTime = 0;
        Serial.printf("Log stopped: %lu samples\n", min(logCount, LOG_BUFFER_SIZE));
    }
    
    // Send Vertiq requests
    uint32_t vertiqStart = micros();
    vertiqSend();
    
    // Wait for Vertiq replies (1.5ms timeout)
    while (micros() - vertiqStart < 1500) {
        vertiqRead();
        if (power.volts_.IsFresh() && 
            multi.obs_angular_velocity_.IsFresh() && 
            multi.obs_angular_displacement_.IsFresh()) {
            break;
        }
    }
    lastVertiqUartUs = micros() - vertiqStart;
    
    // Get Vertiq values
    if (power.volts_.IsFresh()) v_vbat = power.volts_.get_reply();
    if (multi.obs_angular_velocity_.IsFresh()) v_vel = multi.obs_angular_velocity_.get_reply();
    if (multi.obs_angular_displacement_.IsFresh()) {
        v_raw_angle = multi.obs_angular_displacement_.get_reply();
        v_angle = v_raw_angle - v_zero_angle;
    }

    // ---- Logging (circular buffer) ----
    if (logging) {
        logBuffer[logHead].v_angle = v_angle;
        logBuffer[logHead].v_vel = v_vel;
        logBuffer[logHead].v_vbat = v_vbat;
        logBuffer[logHead].v_set_volts = v_current_set_volts;
        logBuffer[logHead].p_position = p_angle;
        logBuffer[logHead].p_velocity = p_vel;
        logBuffer[logHead].time_us = now - logStartTime;
        logHead = (logHead + 1) % LOG_BUFFER_SIZE;
        logCount++;
    }

    // ---- Handle UDP commands (every 10 loops = 20ms) ----
    if (loopcount % 10 == 0) {
        int packetSize = udp.parsePacket();
        if (packetSize) {
            char buf[64];
            int len = udp.read(buf, sizeof(buf) - 1);
            buf[len] = '\0';
            String cmd(buf);
            cmd.trim();
            String cmdUpper = cmd;
            cmdUpper.toUpperCase();

            // Handle Y/N/C confirmation for log download
            if (awaitingLogConfirm) {
                if (cmdUpper == "Y" || cmdUpper == "YES" || cmdUpper.startsWith("Y ")) {
                    awaitingLogConfirm = false;
                    udpReply("SENDING...\n");
                    delay(50);
                    sendLogData();
                    logHead = 0;
                    logCount = 0;
                    logStartTime = 0;
                } else if (cmdUpper == "C" || cmdUpper == "CLEAR") {
                    awaitingLogConfirm = false;
                    logHead = 0;
                    logCount = 0;
                    logStartTime = 0;
                    udpReply("LOG CLEARED\n");
                } else {
                    awaitingLogConfirm = false;
                    udpReply("CANCELLED\n");
                }
            }
            // ---- PennyESC commands (P:D, P:T) ----
            else if (cmdUpper.startsWith("P:D")) {
                int16_t duty = (int16_t)cmdUpper.substring(3).toInt();
                duty = constrain(duty, -799, 799);
                p_duty = duty;
                pennySetDuty(duty);
                
                // Start logging
                if (logStartTime == 0) {
                    logStartTime = micros();
                }
                logging = true;
                logReady = false;
                logStopTime = 0;
                
                Serial.printf("PENNY DUTY: %d (logging)\n", duty);
                udpReply("PENNY DUTY: " + String(duty) + " (logging)\n");

            } else if (cmdUpper.startsWith("P:T")) {
                float pos = cmdUpper.substring(3).toFloat();
                pennySetPositionRad(pos);
                
                if (logStartTime == 0) {
                    logStartTime = micros();
                }
                logging = true;
                logReady = false;
                logStopTime = 0;
                
                Serial.printf("PENNY TARGET: %.2f rad (logging)\n", pos);
                udpReply("PENNY TARGET: " + String(pos, 2) + " rad (logging)\n");

            }
            // ---- Chained command detection (Vertiq) ----
            else if ((cmdUpper.startsWith("V") || cmdUpper.startsWith("T")) && 
                     cmdUpper.indexOf('V') >= 0 && cmdUpper.indexOf('T') >= 0 &&
                     !cmdUpper.startsWith("VOLT") && !cmdUpper.startsWith("VEL")) {
                if (parseChain(cmdUpper)) {
                    if (logStartTime == 0) {
                        logStartTime = micros();
                    }
                    logging = true;
                    logReady = false;
                    logStopTime = 0;
                    
                    String response = "CHAIN[" + String(chainLen) + "]: ";
                    while (chainIdx < chainLen) {
                        ChainCmd &c = chainQueue[chainIdx];
                        chainIdx++;
                        if (c.type == CHAIN_VOLTAGE) {
                            v_set_voltage = constrain(c.value, 0, v_vbat);
                            response += "V" + String(c.value, 1) + " ";
                        } else if (c.type == CHAIN_TARGET) {
                            v_target = constrain(c.value, -188, 188);
                            v_start_err = v_target - v_angle;
                            v_spinning = true;
                            response += "T" + String(c.value, 1) + " ";
                            break;
                        }
                    }
                    response += "(logging)\n";
                    Serial.print(response);
                    udpReply(response);
                } else {
                    udpReply("INVALID CHAIN\n");
                }

            } else if (cmdUpper.startsWith("T")) {
                float val = cmdUpper.substring(1).toFloat();
                v_target = constrain(val, -188, 188);
                v_start_err = v_target - v_angle;
                v_spinning = true;
                chainLen = 0;
                chainIdx = 0;
                
                if (logStartTime == 0) {
                    logStartTime = micros();
                }
                logging = true;
                logReady = false;
                logStopTime = 0;
                
                Serial.printf("VERTIQ TARGET: %.1f (logging)\n", v_target);
                udpReply("VERTIQ TARGET: " + String(v_target, 1) + " (logging)\n");

            } else if (cmdUpper.startsWith("VOLT")) {
                float val = cmdUpper.substring(4).toFloat();
                v_set_voltage = constrain(val, 0, v_vbat);
                control_mode = MODE_VOLTAGE;
                Serial.printf("MODE: VOLTAGE %.2fV\n", v_set_voltage);
                udpReply("MODE: VOLTAGE " + String(v_set_voltage, 2) + "V\n");

            } else if (cmdUpper.startsWith("VEL")) {
                float val = cmdUpper.substring(3).toFloat();
                v_set_velocity = constrain(val, 0.1f, 100.0f);
                control_mode = MODE_VELOCITY;
                Serial.printf("MODE: VELOCITY %.1f rad/s\n", v_set_velocity);
                udpReply("MODE: VELOCITY " + String(v_set_velocity, 1) + " rad/s\n");

            } else if (cmdUpper.startsWith("V")) {
                float val = cmdUpper.substring(1).toFloat();
                v_set_voltage = constrain(val, 0, v_vbat);
                chainLen = 0;
                chainIdx = 0;
                Serial.printf("VOLTAGE: %.2f / %.2f\n", v_set_voltage, v_vbat);
                udpReply("VOLTAGE: " + String(v_set_voltage, 2) + " / " + String(v_vbat, 2) + "\n");

            } else if (cmdUpper.startsWith("STOP") || cmdUpper.startsWith("OFF")) {
                // Stop both motors
                v_spinning = false;
                logging = false;
                homing = false;
                chainLen = 0;
                chainIdx = 0;
                
                // Stop Vertiq
                multi.ctrl_volts_.set(vertiqCom, 0.0f);
                multi.ctrl_brake_.set(vertiqCom);
                vertiqSend();
                
                // Stop PennyESC
                p_duty = 0;
                pennySetDuty(0);
                
                udpReply("STOPPED BOTH\n");

            } else if (cmdUpper.startsWith("ZERO") || cmdUpper == "Z") {
                // Zero both motors
                v_zero_angle = v_raw_angle;
                v_target = 0;
                p_zero_angle = p_angle + p_zero_angle;  // Cumulative zero
                Serial.println("ZEROED BOTH");
                udpReply("ZEROED BOTH\n");

            } else if (cmdUpper == "H" || cmdUpper == "HOME") {
                pre_home_mode = control_mode;
                pre_home_velocity = v_set_velocity;
                control_mode = MODE_VELOCITY;
                v_set_velocity = 5.0f;
                v_target = 0;
                v_start_err = v_target - v_angle;
                v_spinning = true;
                homing = true;
                chainLen = 0;
                chainIdx = 0;
                Serial.printf("HOMING Vertiq from %.1f rad\n", v_angle);
                udpReply("HOMING from " + String(v_angle, 1) + " rad\n");

            } else if (cmdUpper.startsWith("STATUS") || cmdUpper == "S") {
                String modeStr = (control_mode == MODE_VOLTAGE) ? "VOLT" : "VEL";
                udpReply(
                    "STATUS\n"
                    " VERTIQ: angle=" + String(v_angle, 1) + " vel=" + String(v_vel, 1) + 
                    " vbat=" + String(v_vbat, 2) + " target=" + String(v_target, 1) +
                    " spinning=" + String(v_spinning ? 1 : 0) + " mode=" + modeStr +
                    " uart_us=" + String(lastVertiqUartUs) + "\n"
                    " PENNY: angle=" + String(p_angle, 2) + " vel=" + String(p_vel, 1) +
                    " duty=" + String(p_duty) + " uart_us=" + String(lastPennyUartUs) + "\n"
                );

            } else if (cmdUpper == "LOG") {
                uint32_t totalSamples = min(logCount, LOG_BUFFER_SIZE);
                if (totalSamples == 0) {
                    udpReply("NO LOG DATA\n");
                } else {
                    uint32_t newestIdx = (logHead + LOG_BUFFER_SIZE - 1) % LOG_BUFFER_SIZE;
                    uint32_t duration_ms = logBuffer[newestIdx].time_us / 1000;

                    String info = "LOG: " + String(totalSamples) + " samples, " +
                                String(duration_ms) + "ms duration\n" +
                                "Download? (Y/N/C)\n";
                    udpReply(info);

                    logRequesterIP = udp.remoteIP();
                    logRequesterPort = udp.remotePort();
                    awaitingLogConfirm = true;
                }

            } else {
                Serial.printf("Unknown cmd: '%s'\n", cmd.c_str());
                udpReply("UNKNOWN CMD\n");
            }
        }
    }
}
