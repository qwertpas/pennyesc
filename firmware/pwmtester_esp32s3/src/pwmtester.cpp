#include <Arduino.h>

// QT Py ESP32-S3: PWM pulse width 1000–2000 µs @ 50 Hz on GPIO12; GPIO11 driven LOW;
// GPIO13 digital out from host commands.
static constexpr int kPinPwm = 12;
static constexpr int kPinGndRef = 11;
static constexpr int kPinAux = 13;

static constexpr uint32_t kPwmFreqHz = 50;
// ESP32-S3 LEDC max duty resolution is 14 bits (SOC_LEDC_TIMER_BIT_WIDE_NUM); 16 fails ledcSetup().
static constexpr uint8_t kPwmResolutionBits = 14;
static constexpr uint32_t kPeriodUs = 1000000U / kPwmFreqHz;
static constexpr uint8_t kPwmChannel = 0;

static String serialLine;

static void setPulseWidthUs(uint16_t us) {
    if (us < 1000) {
        us = 1000;
    }
    if (us > 2000) {
        us = 2000;
    }
    const uint32_t maxDuty = (1U << kPwmResolutionBits) - 1U;
    const uint32_t duty = (static_cast<uint32_t>(us) * maxDuty) / kPeriodUs;
    ledcWrite(kPwmChannel, duty);
}

static void handleLine(const char *line) {
    while (*line == ' ' || *line == '\t' || *line == '\r') {
        ++line;
    }
    if (*line == '\0') {
        return;
    }
    const char cmd = static_cast<char>(toupper(static_cast<unsigned char>(*line)));
    if (cmd == 'P') {
        const int v = atoi(line + 1);
        setPulseWidthUs(static_cast<uint16_t>(v));
    } else if (cmd == 'D') {
        const int v = atoi(line + 1);
        digitalWrite(kPinAux, v ? HIGH : LOW);
    }
}

void setup() {
    Serial.begin(115200);

    pinMode(kPinGndRef, OUTPUT);
    digitalWrite(kPinGndRef, LOW);

    pinMode(kPinAux, OUTPUT);
    digitalWrite(kPinAux, LOW);

    if (ledcSetup(kPwmChannel, kPwmFreqHz, kPwmResolutionBits) == 0) {
        Serial.println("LEDC setup failed (PWM)");
    }
    ledcAttachPin(kPinPwm, kPwmChannel);
    setPulseWidthUs(1500);
}

void loop() {
    while (Serial.available() > 0) {
        const char ch = static_cast<char>(Serial.read());
        if (ch == '\n' || ch == '\r') {
            if (serialLine.length() > 0) {
                handleLine(serialLine.c_str());
                serialLine = "";
            }
        } else {
            serialLine += ch;
            if (serialLine.length() > 64) {
                serialLine = "";
            }
        }
    }
}
