
// //Same setup with wifi
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <iq_module_communication.hpp>

// ---- Base64 encoding ----
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

size_t base64_encoded_length(size_t inputLen) {
    return 4 * ((inputLen + 2) / 3);
}

// ---- PRBS RNG ----
static uint32_t xorshift32(uint32_t &state) {
    state ^= state << 13;
    state ^= state >> 17;
    state ^= state << 5;
    return state;
}

// ---- PRBS state ----
bool prbs_running = false;
uint32_t prbs_rng = 0x12345678u;
uint32_t prbs_start_time = 0;
uint32_t prbs_next_bit_time = 0;
float prbs_amplitude = 5.0f;
uint32_t prbs_bit_us = 20000;
uint32_t prbs_duration_us = 10000000;
float prbs_v_cmd = 0.0f;

// ---- Loop timing ----
const uint32_t LOOP_FREQ_HZ = 2000;
const uint32_t LOOP_PERIOD_US = 1000000 / LOOP_FREQ_HZ;  // 500µs

// ---- Logging buffer ----
const uint32_t LOG_DURATION_S = 5;
// const uint32_t LOG_BUFFER_SIZE = LOOP_FREQ_HZ * LOG_DURATION_S;  // 10000 samples
const uint32_t LOG_BUFFER_SIZE = 12000;  // a bit more

struct LogSample {
    float angle;
    float vel;
    float vbat;
    float set_volts;
    uint32_t time_us;
};

LogSample logBuffer[LOG_BUFFER_SIZE];
uint32_t logHead = 0;      // Next write position (circular)
uint32_t logCount = 0;     // Total samples written (for knowing if wrapped)
uint32_t logStartTime = 0;
uint32_t logStopTime = 0;  // When to actually stop logging (for post-target delay)
bool logging = false;
bool logReady = false;  // true when a complete log is ready to download

// ---- Motor setup ----
const int TX_PIN = 13; // ESP32 TX → Vertiq RX
const int RX_PIN = 12; // ESP32 RX → Vertiq TX
const int MTR_GND = 11;
const int LED_PIN = 48;

GenericInterface com; // Use GenericInterface directly for batching
PowerMonitorClient power(0);
BrushlessDriveClient mot(0);
MultiTurnAngleControlClient multi(0);

// ---- Wi-Fi SoftAP + UDP ----
WiFiUDP udp;
const char *SSID = "dnajumper";
const uint16_t PORT = 9870;

void udpReply(const String &message) {
    udp.beginPacket(udp.remoteIP(), udp.remotePort());
    udp.write((const uint8_t *)message.c_str(), message.length());
    udp.endPacket();
}

// Helper to send all bytes in the com TX queue
void comSend() {
    uint8_t buf[128];
    uint8_t len;
    if (com.GetTxBytes(buf, len)) {
        Serial1.write(buf, len);
    }
}

// Helper to read and parse packets
void comRead() {
    uint8_t buf[128];
    uint8_t len;
    while (Serial1.available()) {
        len = Serial1.readBytes(buf, min((int)sizeof(buf), Serial1.available()));
        com.SetRxBytes(buf, len);
        
        uint8_t *packet;
        uint8_t pLen;
        while (com.PeekPacket(&packet, &pLen)) {
            multi.ReadMsg(packet, pLen);
            power.ReadMsg(packet, pLen);
            com.DropPacket();
        }
    }
}

// State for LOG command flow
bool awaitingLogConfirm = false;
IPAddress logRequesterIP;
uint16_t logRequesterPort;

void setup() {
    // setCpuFrequencyMhz(80);
    setCpuFrequencyMhz(240);

    Serial.begin(115200);
    pinMode(MTR_GND, OUTPUT);
    digitalWrite(MTR_GND, LOW);
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
    delay(1000);
    Serial.println("\nStarting Vertiq Wi-Fi Control (SoftAP mode)");

    // ---- Wi-Fi Access Point ----
    WiFi.mode(WIFI_AP);
    WiFi.softAP(SSID, nullptr, 6);
    IPAddress ip = WiFi.softAPIP();
    Serial.printf("SoftAP '%s' active\nConnect your laptop to Wi-Fi SSID: %s\n", SSID, SSID);
    Serial.printf("ESP32 IP address: %s\n", ip.toString().c_str());

    // ---- UDP listener ----
    udp.begin(PORT);
    Serial.printf("Listening for UDP commands on port %d\n", PORT);

    // ---- Motor UART ----
    Serial1.begin(921600, SERIAL_8N1, RX_PIN, TX_PIN);
    multi.ctrl_volts_.set(com, 0.0f);
    comSend();
    Serial.println("Motor interface initialized at 921600 baud");
    
    Serial.printf("Log buffer: %lu samples (%lu bytes)\n", LOG_BUFFER_SIZE, LOG_BUFFER_SIZE * sizeof(LogSample));
}

float raw_angle = 0;
float zero_angle = 0;
float angle = 0;
float target = 0;
bool spinning = false;
float start_err = 1;
float set_voltage = 0.2f;

// ---- Chained command queue ----
enum ChainCmdType { CHAIN_VOLTAGE, CHAIN_TARGET };
struct ChainCmd { ChainCmdType type; float value; };
const int MAX_CHAIN_LEN = 10;
ChainCmd chainQueue[MAX_CHAIN_LEN];
int chainLen = 0;   // Total commands in queue
int chainIdx = 0;   // Next command to execute

// Parse chained command string like "v4t5v6t15" into chainQueue
// Returns true if valid chain with at least one command parsed
bool parseChain(const String &cmd) {
    chainLen = 0;
    chainIdx = 0;
    int pos = 0;
    int len = cmd.length();
    
    while (pos < len && chainLen < MAX_CHAIN_LEN) {
        char c = cmd.charAt(pos);
        if (c == 'V' || c == 'v') {
            // Find end of number
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
            pos++;  // Skip unknown characters
        }
    }
    return chainLen > 0;
}

// ---- Control mode ----
enum ControlMode { MODE_VOLTAGE, MODE_VELOCITY };
ControlMode control_mode = MODE_VOLTAGE;
float set_velocity = 20.0f;  // rad/s for velocity mode

// ---- Homing state ----
bool homing = false;
ControlMode pre_home_mode = MODE_VOLTAGE;
float pre_home_velocity = 20.0f;

float vbat = 0;
float vel = 0;

uint32_t lastLoopTime = 0;
uint32_t lastUartTimeUs = 0;

void sendLogData() {
    // Each LogSample is 20 bytes. 
    // We will send in chunks of 30 samples = 600 bytes raw.
    // 600 bytes raw becomes 800 bytes in Base64 (well within UDP MTU).
    const uint32_t SAMPLES_PER_CHUNK = 30;
    char base64Chunk[801]; // 4 * (600/3) + 1 for null
    LogSample tempChunk[SAMPLES_PER_CHUNK];
    
    uint32_t totalSamples = min(logCount, LOG_BUFFER_SIZE);
    // Start index: if wrapped, start from logHead (oldest), else start from 0
    uint32_t startIdx = (logCount >= LOG_BUFFER_SIZE) ? logHead : 0;
    
    uint32_t samplesSent = 0;
    int chunkNum = 0;
    
    while (samplesSent < totalSamples) {
        uint32_t samplesToProcess = min(SAMPLES_PER_CHUNK, totalSamples - samplesSent);
        
        // Copy samples to temp buffer in order (handling circular wrap)
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
        delay(8); // Small delay to prevent packet loss
    }
    
    // Send end marker
    delay(20);
    udpReply("END\n");
    
    Serial.printf("Sent %lu samples in %d chunks\n", samplesSent, chunkNum);
}

long loopcount = 0;
void loop() {
    // ---- Precise loop timing ----
    uint32_t now = micros();
    if (now - lastLoopTime < LOOP_PERIOD_US) {
        return;  // Not time yet
    }
    lastLoopTime = now;
    loopcount++;
    
    // ---- Batch read sensors and set control ----
    power.volts_.get(com);
    multi.obs_angular_velocity_.get(com);
    multi.obs_angular_displacement_.get(com);
    
    float error = target - angle;
    float current_set_volts = 0;

    // ---- PRBS mode ----
    if (prbs_running) {
        uint32_t now_prbs = micros();
        // Check duration
        if ((now_prbs - prbs_start_time) >= prbs_duration_us) {
            prbs_running = false;
            prbs_v_cmd = 0.0f;
            multi.ctrl_volts_.set(com, 0.0f);
            multi.ctrl_brake_.set(com);
            logging = false;
            logReady = true;
            digitalWrite(LED_PIN, LOW);
        } else {
            // Update bit
            if ((int32_t)(now_prbs - prbs_next_bit_time) >= 0) {
                prbs_next_bit_time += prbs_bit_us;
                uint32_t r = xorshift32(prbs_rng);
                prbs_v_cmd = (r & 1u) ? prbs_amplitude : -prbs_amplitude;
            }
            current_set_volts = prbs_v_cmd;
            multi.ctrl_volts_.set(com, current_set_volts);
            digitalWrite(LED_PIN, HIGH);
        }
    }
    // ---- Normal spinning mode ----
    else if(spinning && start_err*error>0) {
        if (control_mode == MODE_VOLTAGE) {
            current_set_volts = (error > 0) ? set_voltage : -set_voltage;
            multi.ctrl_volts_.set(com, current_set_volts);
        } else {
            // Velocity control mode
            float vel_cmd = (error > 0) ? set_velocity : -set_velocity;
            multi.ctrl_velocity_.set(com, vel_cmd);
        }
        digitalWrite(LED_PIN, HIGH);
    } else {
        // Target reached (or not spinning) - check for chained commands
        bool chainContinued = false;
        if (spinning && chainIdx < chainLen) {
            // Execute pending chain commands
            while (chainIdx < chainLen) {
                ChainCmd &cmd = chainQueue[chainIdx];
                chainIdx++;
                if (cmd.type == CHAIN_VOLTAGE) {
                    set_voltage = constrain(cmd.value, 0, vbat);
                    Serial.printf("CHAIN: V%.2f\n", set_voltage);
                } else if (cmd.type == CHAIN_TARGET) {
                    target = constrain(cmd.value, -188, 188);
                    start_err = target - angle;
                    Serial.printf("CHAIN: T%.1f (err %.1f)\n", target, start_err);
                    chainContinued = true;
                    break;  // Continue spinning to new target
                }
            }
        }
        
        if (!chainContinued) {
            // Handle homing completion - restore previous mode, don't hold position
            bool wasHoming = homing;
            if (homing) {
                control_mode = pre_home_mode;
                set_velocity = pre_home_velocity;
                homing = false;
                Serial.println("HOME COMPLETE");
            }
            
            if (control_mode == MODE_VELOCITY && !wasHoming) {
                // Hold position using angle control (but not after homing)
                multi.ctrl_angle_.set(com, target + zero_angle);
            } else {
                multi.ctrl_volts_.set(com, 0.0f);
                multi.ctrl_brake_.set(com);
            }

            // When movement ends, schedule logging to stop after 0.1s
            if (spinning && logging && logStopTime == 0) {
                logStopTime = now + 100000;  // 0.1 seconds after target reached
            }
            spinning = false;
            chainLen = 0;  // Clear chain
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
    
    // Send all requests in one burst
    uint32_t uartStart = micros();
    comSend();
    
    // Wait for replies (with 1.5ms timeout)
    while (micros() - uartStart < 1500) {
        comRead();
        if (power.volts_.IsFresh() && 
            multi.obs_angular_velocity_.IsFresh() && 
            multi.obs_angular_displacement_.IsFresh()) {
            break;
        }
    }
    lastUartTimeUs = micros() - uartStart;
    
    // Get values from entries
    if (power.volts_.IsFresh()) vbat = power.volts_.get_reply();
    if (multi.obs_angular_velocity_.IsFresh()) vel = multi.obs_angular_velocity_.get_reply();
    if (multi.obs_angular_displacement_.IsFresh()) {
        raw_angle = multi.obs_angular_displacement_.get_reply();
        angle = raw_angle - zero_angle;
    }

    // ---- Logging (circular buffer) ----
    if (logging) {
        logBuffer[logHead].angle = angle;
        logBuffer[logHead].vel = vel;
        logBuffer[logHead].vbat = vbat;
        logBuffer[logHead].set_volts = current_set_volts;
        logBuffer[logHead].time_us = now - logStartTime;
        logHead = (logHead + 1) % LOG_BUFFER_SIZE;
        logCount++;
    }

    // ---- Handle UDP commands ----
    if(loopcount % 10 == 0){

    
        int packetSize = udp.parsePacket();
        if (packetSize) {
            char buf[64];
            int len = udp.read(buf, sizeof(buf) - 1);
            buf[len] = '\0';
            String cmd(buf);
            cmd.trim();
            cmd.toUpperCase();

            // Handle Y/N/C confirmation for log download
            if (awaitingLogConfirm) {
                if (cmd == "Y" || cmd == "YES" || cmd.startsWith("Y ")) {
                    awaitingLogConfirm = false;
                    udpReply("SENDING...\n");
                    delay(50);
                    sendLogData();
                    // Clear log after successful download
                    logHead = 0;
                    logCount = 0;
                    logStartTime = 0;
                } else if (cmd == "C" || cmd == "CLEAR") {
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
            // ---- Chained command detection (e.g., V4T5V6T15) ----
            else if ((cmd.startsWith("V") || cmd.startsWith("T")) && 
                     cmd.indexOf('V') >= 0 && cmd.indexOf('T') >= 0 &&
                     !cmd.startsWith("VOLT") && !cmd.startsWith("VEL")) {
                // Parse as chained command
                if (parseChain(cmd)) {
                    // Start logging
                    if (logStartTime == 0) {
                        logStartTime = micros();
                    }
                    logging = true;
                    logReady = false;
                    logStopTime = 0;
                    
                    // Execute commands until we hit a target
                    String response = "CHAIN[" + String(chainLen) + "]: ";
                    while (chainIdx < chainLen) {
                        ChainCmd &c = chainQueue[chainIdx];
                        chainIdx++;
                        if (c.type == CHAIN_VOLTAGE) {
                            set_voltage = constrain(c.value, 0, vbat);
                            response += "V" + String(c.value, 1) + " ";
                        } else if (c.type == CHAIN_TARGET) {
                            target = constrain(c.value, -188, 188);
                            start_err = target - angle;
                            spinning = true;
                            response += "T" + String(c.value, 1) + " ";
                            break;  // Start spinning, remaining commands execute on target reach
                        }
                    }
                    response += "(logging)\n";
                    Serial.print(response);
                    udpReply(response);
                } else {
                    udpReply("INVALID CHAIN\n");
                }

            } else if (cmd.startsWith("T")) {
                float val = cmd.substring(1).toFloat();
                target = constrain(val, -188, 188); //30 rotations
                start_err = target - angle;
                spinning = true;
                chainLen = 0;  // Clear any pending chain
                chainIdx = 0;
                
                // Start logging (only set start time if buffer was cleared)
                if (logStartTime == 0) {
                    logStartTime = micros();
                }
                logging = true;
                logReady = false;
                logStopTime = 0;  // Cancel any pending log stop
                
                Serial.printf("TARGET SET %.1f, ERR %.1f (logging)\n", target, start_err);
                udpReply("TARGET SET " + String(target, 1) + ", ERR " + String(start_err, 1) + " (logging)\n");

            } else if(cmd.startsWith("VOLT")) {
                float val = cmd.substring(4).toFloat();
                set_voltage = constrain(val, 0, vbat);
                control_mode = MODE_VOLTAGE;
                Serial.printf("MODE: VOLTAGE %.2fV\n", set_voltage);
                udpReply("MODE: VOLTAGE " + String(set_voltage, 2) + "V\n");

            } else if(cmd.startsWith("VEL")) {
                float val = cmd.substring(3).toFloat();
                set_velocity = constrain(val, 0.1f, 100.0f);
                control_mode = MODE_VELOCITY;
                Serial.printf("MODE: VELOCITY %.1f rad/s\n", set_velocity);
                udpReply("MODE: VELOCITY " + String(set_velocity, 1) + " rad/s\n");

            } else if(cmd.startsWith("V")) {
                float val = cmd.substring(1).toFloat();
                set_voltage = constrain(val, 0, vbat);
                chainLen = 0;  // Clear any pending chain
                chainIdx = 0;
                Serial.printf("VOLTAGE SET %.2f / %.2f\n", set_voltage, vbat);
                udpReply("VOLTAGE SET " + String(set_voltage, 2) + " / " + String(vbat, 2) + "\n");

            } else if(cmd.startsWith("STOP") || cmd.startsWith("OFF")){
                spinning = false;
                prbs_running = false;
                prbs_v_cmd = 0.0f;
                logging = false;
                homing = false;
                chainLen = 0;  // Clear chain
                chainIdx = 0;
                multi.ctrl_volts_.set(com, 0.0f);
                multi.ctrl_brake_.set(com);
                comSend();
                udpReply("STOPPED\n");

            }else if (cmd.startsWith("ZERO") || cmd == "Z") {
                zero_angle = raw_angle;
                target = 0;
                Serial.println("ZEROED");
                udpReply("ZEROED\n");

            } else if (cmd == "H" || cmd == "HOME") {
                // Home: velocity control at 5 rad/s to 0, then restore previous mode
                pre_home_mode = control_mode;
                pre_home_velocity = set_velocity;
                control_mode = MODE_VELOCITY;
                set_velocity = 5.0f;
                target = 0;
                start_err = target - angle;
                spinning = true;
                homing = true;
                chainLen = 0;
                chainIdx = 0;
                Serial.printf("HOMING from %.1f rad\n", angle);
                udpReply("HOMING from " + String(angle, 1) + " rad\n");

            } else if (cmd.startsWith("STATUS") || cmd == "S") {
                String modeStr = (control_mode == MODE_VOLTAGE) ? "VOLT" : "VEL";
                udpReply(
                    "STATUS " +
                    String("SPINNING:") + String(spinning ? 1 : 0) +
                    " " + String("TARGET:") + String(target, 1) +
                    " " + String("ANGLE:") + String(angle, 1) +
                    " " + String("MODE:") + modeStr +
                    " " + String("VSET:") + String(set_voltage, 2) +
                    " " + String("VELSET:") + String(set_velocity, 1) +
                    " " + String("VBAT:") + String(vbat, 2) +
                    " " + String("VEL:") + String(vel, 1) +
                    " " + String("UART_US:") + String(lastUartTimeUs) +
                    "\n"
                );

            } else if (cmd == "LOG") {
                uint32_t totalSamples = min(logCount, LOG_BUFFER_SIZE);
                if (totalSamples == 0) {
                    udpReply("NO LOG DATA\n");
                } else {
                    // Calculate duration from newest sample
                    uint32_t newestIdx = (logHead + LOG_BUFFER_SIZE - 1) % LOG_BUFFER_SIZE;
                    uint32_t duration_ms = logBuffer[newestIdx].time_us / 1000;

                    String info = "LOG: " + String(totalSamples) + " samples, " +
                                String(duration_ms) + "ms duration\n" +
                                "Download? (Y/N/C)\n";
                    udpReply(info);

                    // Remember who asked, for the confirmation
                    logRequesterIP = udp.remoteIP();
                    logRequesterPort = udp.remotePort();
                    awaitingLogConfirm = true;
                }

            } else if (cmd.startsWith("PRBS")) {
                // Parse: PRBS [amplitude] [bit_ms] [duration_s]
                String args = cmd.substring(4);
                args.trim();
                if (args.length() > 0) {
                    int sp1 = args.indexOf(' ');
                    if (sp1 > 0) {
                        prbs_amplitude = constrain(args.substring(0, sp1).toFloat(), 0.1f, 7.0f);
                        String rest = args.substring(sp1 + 1);
                        rest.trim();
                        int sp2 = rest.indexOf(' ');
                        if (sp2 > 0) {
                            prbs_bit_us = rest.substring(0, sp2).toInt() * 1000;
                            prbs_duration_us = rest.substring(sp2 + 1).toInt() * 1000000;
                        } else {
                            prbs_bit_us = rest.toInt() * 1000;
                        }
                    } else {
                        prbs_amplitude = constrain(args.toFloat(), 0.1f, 7.0f);
                    }
                }
                // Start PRBS
                prbs_rng = 0x12345678u;
                prbs_start_time = micros();
                prbs_next_bit_time = prbs_start_time;
                prbs_v_cmd = 0.0f;
                prbs_running = true;
                spinning = false;
                // Start logging (reset buffer for PRBS sysid)
                logHead = 0;
                logCount = 0;
                logStartTime = prbs_start_time;
                logging = true;
                logReady = false;

                udpReply("PRBS: " + String(prbs_amplitude, 1) + "V, " +
                    String(prbs_bit_us/1000) + "ms, " + String(prbs_duration_us/1000000) + "s\n");

            } else {
                Serial.printf("Unknown cmd: '%s'\n", cmd.c_str());
                udpReply("UNKNOWN CMD\n");
            }
        }
    }
}
