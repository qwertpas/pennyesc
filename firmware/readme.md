# PennyESC Firmware

## Wiring

PennyESC requires 4 wires (in order on the PennyESC PCB):

- GND
- UART RX (RX of ESC, TX of your microcontroller)
- UART TX (TX of ESC, RX of your microcontroller)
- 5-15V power input

Multiple PennyESCs can be daisy chained on the same UART bus because it is implemented with open-drain outputs. So, the TX line requires a pull-up resistor to 3.3V. Each PennyESC on the same bus must have a unique address. See the Flashing section on changing the address.

A large capacitor (>100uF) is recommended between the power input and GND. Without it, the driver may turn off during high current (mct_faults will increment in the motor status in the GUI).



## Arduino API

PennyESC can be commanded over UART and I've provided a Arduino API.

Minimal code to spin:

```cpp
#include <Arduino.h>
#include "pennyesc_arduino.h"
PennyEsc esc(1); // ESC address 1
void setup() {
  esc.begin(Serial1, 13, 12); // RX pin 13, TX pin 12
}
void loop() {
  esc.setDuty(100); // duty ranges from -799 to 799
  delay(1000);
  esc.setDuty(0);
  delay(1000);
}
```

Encoder read:

```cpp
PennyEscEncoderData data;
esc.getPosVel(data);
float pos = data.positionRad();
float rpm = data.velocityRpm();
```

Other examples in `firmware/esp32s3demo_example`:

- `src/bridge.cpp`: USB serial bridge using `PennyEscDebugBridge`.
- `src/position_sweep.cpp`: zeros the ESC, sets control gains, alternates position targets, and prints status.
- `src/capture.cpp`: captures angle and RPM in firmware at up to 1000 Hz for 200 ms, then prints CSV.

Common calls:


| Call                                               | Use                                                            |
| -------------------------------------------------- | -------------------------------------------------------------- |
| `esc.begin(Serial1, RX_pin, TX_pin)`               | Start ESC UART on custom pins.                                 |
| `esc.getStatus(status)`                            | Read position, velocity, duty, sensor data, flags, and faults. |
| `esc.getPosVel(data)`                              | Fast read of only encoder position and velocity.               |
| `esc.setDuty(duty)`                                | Set open-loop duty, `-799..799`. Start low.                    |
| `esc.sendPositionRad(rad)`                         | Move to an absolute position relative to current zero.         |
| `esc.zeroPosition()`                               | Set the current shaft position as zero.                        |


## Arduino Bridge and Debug Helpers

The normal bridge is part of the main Arduino header:

```cpp
#include "pennyesc_arduino.h"
PennyEscBridge bridge;
```

Use the debug header only for development-only capture and UART rate tests:

```cpp
#include "pennyesc_arduino_debug.h"
PennyEscDebug esc(1);
PennyEscDebugBridge bridge;
```

Debug helper calls:

| Call                                                       | Use                                      |
| ---------------------------------------------------------- | ---------------------------------------- |
| `esc.startCapture(duty, advance, ms, hz, capture)`         | Start firmware RPM capture.              |
| `esc.getCaptureStatus(capture)`                            | Poll capture progress and sample count.  |
| `esc.readCapture(offset, samples, count, got)`             | Read up to 14 captured samples.          |
| `esc.setObserver(lead_us, mode)`                           | Set the development observer mode.       |
| `PennyEscDebugBridge` command `rate ...` / `pollfast ...`  | Run UART timing tests from the USB shell. |


## Calibration and Test GUI

I recommend using the GUI for calibration and testing. Calibration must be done whenever the encoder magnet is moved. Calibration persists between power cycles.

Run the main calibration and test GUI:

```bash
python3 firmware/penny-gui.py
```

The GUI can run static calibration, status checks, duty commands, stop, and advance commands. Sessions are saved under `firmware/penny-gui-sessions/`.



Command line calibration is also available:

```bash
python3 firmware/tools/pennycal.py --address 1 info
python3 firmware/tools/pennycal.py --address 1 calibrate
python3 firmware/tools/pennycal.py --address 1 verify
```

## Flashing:

To update the firmware running on the PennyESC itself, it is meant to be flashed over UART. To do so, it must be connected to an ESP32 with the bridge firmware. 

If you need to calibrate or configure the pennyesc, flash the ESP32 bridge firmware from `firmware/esp32s3demo_example/src/bridge.cpp`:

```bash
python3 -m platformio run -d firmware/esp32s3demo_example -e bridge -t upload
```

Scan for connected PennyESC addresses:

```bash
python3 firmware/tools/pnyboot.py scan
```

Update PennyESC firmware over UART after it has been seeded and connected to the ESP32 bridge:

```bash
ESC_ADDRESS=1 \
python3 -m platformio run -d firmware/pennyesc_libopencm3 -e pennyesc_uart -t uart_upload
```

Change a board from one ESC address to another, then use the new address for later commands:

```bash
CURRENT_ESC_ADDRESS=1 NEW_ESC_ADDRESS=2 \
python3 -m platformio run -d firmware/pennyesc_libopencm3 -e readdress -t uart_readdress
```

Seed a PennyESC over an STLink. Use this for a fresh board or a board that is bricked and needs the UART bootloader restored:

```bash
ESC_ADDRESS=1 \
python3 -m platformio run -d firmware/pennyesc_libopencm3 -e seed -t seed_upload
```

## Low-level Protocol

If you only need to communicate with the ESC using the Arduino API or calibration/testing GUI, you do not need to worry about this protocol. 

Frames are addressed, length-prefixed, and CRC checked:

```text
[0]  0xAA start byte
[1]  header: high nibble = address, low nibble = command
[2]  payload length, 0..64
[3..] payload bytes
[last] CRC-8 over every previous byte, polynomial 0x07, initial 0x00
```

Commands are defined in `firmware/Lib/pennyesc_protocol.h`.


| Command                 | Code  | Request                       | Response                |
| ----------------------- | ----- | ----------------------------- | ----------------------- |
| `PNY_CMD_GET_STATUS`    | `0x1` | none                          | `pny_status_payload_t`  |
| `PNY_CMD_SET_POSITION`  | `0x2` | `int32 position_turn32`       | status                  |
| `PNY_CMD_SET_DUTY`      | `0x3` | `int16 duty`                  | status                  |
| `PNY_CMD_CAL`           | `0x4` | calibration subcommand        | calibration payload     |
| `PNY_CMD_DEBUG`         | `0x5` | debug subcommand              | debug payload           |
| `PNY_CMD_ZERO_POSITION` | `0x6` | none                          | status                  |
| `PNY_CMD_SET_VELOCITY`  | `0x7` | `int32 velocity_turn32_per_s` | status                  |
| `PNY_CMD_SET_CONTROL`   | `0x8` | `pny_control_payload_t`       | status                  |
| `PNY_CMD_STOP`          | `0x9` | none                          | status                  |
| `PNY_CMD_SEND_POSITION` | `0xA` | `int32 position_turn32`       | none                    |
| `PNY_CMD_ENTER_BOOT`    | `0xB` | `uint32 PNY_BOOT_MAGIC`       | result byte             |
| `PNY_CMD_SET_ADVANCE`   | `0xC` | `int16 advance_deg`           | status                  |
| `PNY_CMD_SET_QUIET`     | `0xD` | `uint16 hold_ms`              | result byte             |
| `PNY_CMD_GET_POS_VEL`   | `0xE` | none                          | `pny_pos_vel_payload_t` |


All multi-byte fields are little-endian.

Debug capture subcommands:


| Subcommand                 | Code  | Request                                    | Response                           |
| -------------------------- | ----- | ------------------------------------------ | ---------------------------------- |
| `PNY_DEBUG_CAPTURE_START`  | `0x1` | `pny_capture_start_payload_t`              | `pny_capture_status_payload_t`     |
| `PNY_DEBUG_CAPTURE_STATUS` | `0x2` | subcommand byte                            | `pny_capture_status_payload_t`     |
| `PNY_DEBUG_CAPTURE_READ`   | `0x3` | subcommand, `uint16 offset`, `uint8 count` | `pny_capture_read_payload_t` chunk |


The capture buffer stores 200 samples. `PNY_DEBUG_CAPTURE_READ` returns at most 14 samples per frame.

## Units


| Value         | Unit           | Conversion                        |
| ------------- | -------------- | --------------------------------- |
| Position      | `turn32`       | `65536 = 1 mechanical revolution` |
| Radians       | radians        | `rad = turn32 * 2*pi / 65536`     |
| Velocity      | `turn32/s`     | `rpm = turn32_per_s * 60 / 65536` |
| Duty          | raw PWM        | `-799..799`                       |
| Control gains | Q8 fixed-point | `256 = 1.0`                       |


## Files


| Path                                          | Purpose                                      |
| --------------------------------------------- | -------------------------------------------- |
| `firmware/penny-gui.py`                       | Main calibration and test GUI.               |
| `firmware/Lib/pennyesc_arduino.h`             | ESP32 Arduino API.                           |
| `firmware/Lib/pennyesc_arduino_debug.h`       | Development capture, observer, and rate helpers. |
| `firmware/Lib/pennyesc_protocol.h`            | Shared packet protocol.                      |
| `firmware/esp32s3demo_example/src/bridge.cpp` | ESP32 bridge firmware with debug commands.   |
| `firmware/pennyesc_libopencm3/src/main.c`     | STM32 PennyESC app.                          |
| `firmware/tools/pennycal.py`                  | Host calibration CLI and shared GUI backend. |
| `firmware/tools/pnyboot.py`                   | Host UART boot/update tool.                  |

