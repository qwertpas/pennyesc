#include <Arduino.h>
#include "esc_address.h"
#include "pennyesc_arduino.h"

static PennyEsc esc(ESC0);
static PennyEscStatus last_status;
static String input_line;
static uint32_t pulse_end_ms = 0u;
static bool pulse_active = false;

static void print_status(const PennyEscStatus &status)
{
    Serial.printf(
        "mode=%u result=%u duty=%d pos=%.3f vel=%.2f x=%d y=%d z=%d flags=0x%02X faults=0x%02X\n",
        status.mode,
        status.result,
        status.duty,
        status.positionRad(),
        status.velocityRpm(),
        status.x,
        status.y,
        status.z,
        status.flags,
        status.faults
    );
}

static void print_help()
{
    Serial.println("commands: d<duty>, t<rad>, p<duty>, s, b, h");
}

static void handle_line(String line)
{
    line.trim();
    if (!line.length()) {
        return;
    }

    char cmd = line.charAt(0);
    String value = line.substring(1);

    if (cmd == 'd') {
        if (esc.setDuty((int16_t)value.toInt(), &last_status)) {
            print_status(last_status);
        } else {
            Serial.println("duty failed");
        }
        return;
    }

    if (cmd == 't') {
        if (esc.setPositionRad(value.toFloat(), &last_status)) {
            print_status(last_status);
        } else {
            Serial.println("position failed");
        }
        return;
    }

    if (cmd == 'p') {
        int16_t duty = (int16_t)value.toInt();
        if (esc.setDuty(duty, &last_status)) {
            pulse_end_ms = millis() + 500u;
            pulse_active = true;
            print_status(last_status);
        } else {
            Serial.println("pulse failed");
        }
        return;
    }

    if (cmd == 's') {
        if (esc.getStatus(last_status)) {
            print_status(last_status);
        } else {
            Serial.println("status failed");
        }
        return;
    }

    if (cmd == 'b') {
        Serial.println(esc.enterBootloader() ? "boot ok" : "boot failed");
        return;
    }

    if (cmd == 'h' || cmd == '?') {
        print_help();
        return;
    }

    Serial.println("unknown");
}

void setup()
{
    Serial.begin(115200);
    esc.begin(Serial1, PennyEscPins(), PENNYESC_BAUD_UPDATE);
    delay(100);
    Serial.println("pennyesc motortest");
    print_help();
}

void loop()
{
    if (pulse_active && (int32_t)(millis() - pulse_end_ms) >= 0) {
        pulse_active = false;
        esc.setDuty(0, &last_status);
        print_status(last_status);
    }

    while (Serial.available() > 0) {
        char ch = (char)Serial.read();
        if (ch == '\n' || ch == '\r') {
            if (input_line.length()) {
                handle_line(input_line);
                input_line = "";
            }
        } else if (input_line.length() < 32) {
            input_line += ch;
        }
    }
}
