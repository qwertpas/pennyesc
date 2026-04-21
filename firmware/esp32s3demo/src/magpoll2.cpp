#include <Arduino.h>
#include "esc_address.h"
#include "pennyesc_arduino.h"

static PennyEsc esc(1);
static String input_line;

static void print_help()
{
    Serial.println("# magpoll2");
    Serial.println("# commands: e<esc>, ?, h");
}

static void handle_line(String line)
{
    line.trim();
    if (!line.length()) {
        return;
    }

    char cmd = line.charAt(0);
    if (cmd == 'e' || cmd == 'E') {
        int esc_id = line.substring(1).toInt();
        if (esc_id >= 0 && esc_id <= 0xF) {
            esc.setAddress((uint8_t)esc_id);
            Serial.printf("# esc=%u\n", esc.address());
        } else {
            Serial.println("# invalid esc");
        }
        return;
    }

    if (cmd == '?') {
        PennyEscStatus status;
        if (esc.getStatus(status)) {
            Serial.printf(
                "# esc=%u baud=%lu mode=%u flags=0x%02X faults=0x%02X\n",
                esc.address(),
                (unsigned long)esc.baud(),
                status.mode,
                status.flags,
                status.faults
            );
        } else {
            Serial.printf("# esc=%u baud=%lu timeout\n", esc.address(), (unsigned long)esc.baud());
        }
        return;
    }

    if (cmd == 'h' || cmd == 'H') {
        print_help();
        return;
    }

    Serial.println("# unknown");
}

void setup()
{
    Serial.begin(115200);
    esc.begin(Serial1, PennyEscPins(), PENNYESC_BAUD_FAST);
    delay(100);
    print_help();
    Serial.printf("# esc=%u\n", esc.address());
}

void loop()
{
    static uint32_t last_poll_us = 0u;
    PennyEscStatus status;

    if ((uint32_t)(micros() - last_poll_us) >= 2000u) {
        last_poll_us = micros();
        if (esc.getStatus(status)) {
            Serial.printf("%u,%d,%d,%d\n", esc.address(), status.x, status.y, status.z);
        }
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
