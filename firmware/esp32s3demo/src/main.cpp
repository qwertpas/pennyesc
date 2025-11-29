#include <Arduino.h>

// Define pins for the UART connection to the other MCU
// ESP32 TX pin (connects to other MCU RX)
#define TX_PIN 13
// ESP32 RX pin (connects to other MCU TX)
#define RX_PIN 12

// Target baudrate
#define BAUDRATE 921600

void setup() {
  // Initialize USB Serial (Serial)
  // Note: Baud rate is ignored for native USB CDC, but we set it for consistency
  Serial.begin(BAUDRATE);

  // Initialize Hardware Serial (Serial1) for communication with the other MCU
  // Syntax: Serial1.begin(baud, config, rxPin, txPin)
  // SERIAL_8N1 (8 data bits, No parity, 1 stop bit) is the default and idles HIGH.
  Serial1.begin(BAUDRATE, SERIAL_8N1, RX_PIN, TX_PIN);
}

void loop() {
  // Forward data from USB Serial to Hardware Serial (Laptop -> Other MCU)
  while (Serial.available()) {
    Serial1.write(Serial.read());
  }

  // Forward data from Hardware Serial to USB Serial (Other MCU -> Laptop)
  while (Serial1.available()) {
    Serial.write(Serial1.read());
  }
}
