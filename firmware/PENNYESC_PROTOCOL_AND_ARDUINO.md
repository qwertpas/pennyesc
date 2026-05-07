# PennyESC Protocol and Arduino Guide

This is the current guide for controlling PennyESC from an ESP32 Arduino sketch.

## Start Here

1. Wire ESP32-S3 `RX=GPIO12` to PennyESC `PA9/TX`.
2. Wire ESP32-S3 `TX=GPIO13` to PennyESC `PA10/RX`.
3. Share ground. The default `PennyEscPins` also drives `GPIO11` low and `GPIO9` high for the existing test wiring.
4. Use `firmware/Lib/pennyesc_arduino.h` from the ESP32 PlatformIO project.
5. Match the ESC address. The STM32 firmware default is address `1` unless built with a different `PNY_ESC_ADDRESS`.

Minimal sketch:

```cpp
#include <Arduino.h>
#include "pennyesc_arduino.h"

PennyEsc esc(1);
PennyEscBridge bridge;

void setup()
{
    Serial.begin(115200);
    esc.begin(Serial1, PennyEscPins(), PENNYESC_BAUD_FAST);
    bridge.begin(Serial, Serial1, PennyEscPins(), PENNYESC_BAUD_FAST);
}

void loop()
{
    PennyEscStatus status;
    if (esc.getStatus(status) && status.result == PNY_RESULT_OK) {
        Serial.printf(
            "pos=%.3f rad vel=%.1f rpm duty=%d sensor=%d cal=%d faults=0x%02X\n",
            status.positionRad(),
            status.velocityRpm(),
            status.duty,
            status.sensorOk(),
            status.calValid(),
            status.faults
        );
    }
    delay(250);
}
```

## User-Level Arduino API

Use `PennyEsc` unless you are writing host tools or bootloader code.

| Call | What it does |
| --- | --- |
| `begin(serial, pins, baud)` | Starts the ESC UART. Use `PENNYESC_BAUD_FAST` for the normal app. |
| `getStatus(status)` | Reads position, velocity, duty, flags, faults, and sensor values. |
| `setDuty(duty)` | Runs open-loop duty from `-799` to `799`. Start low. |
| `setPositionRad(rad)` | Commands an absolute position in radians relative to the current zero. |
| `zeroPosition()` | Sets the current shaft position as zero. |
| `setAdvanceDeg(deg)` | Sets commutation advance from `-180` to `180` degrees. |
| `pollEncoder(data)` | Convenience wrapper around `getStatus()` for encoder-style reads. |
| `enterBootloader()` | Requests the STM32 UART boot path. |

Useful status helpers:

| Helper | Meaning |
| --- | --- |
| `status.sensorOk()` | Magnetic sensor is responding. |
| `status.calValid()` | Calibration blob is loaded. |
| `status.busy()` | ESC is in a busy operation. |
| `status.positionReached()` | Position target is within deadband. |
| `status.hasFault()` | One or more fault bits are set. |

## Safe First Motor Test

1. Flash the STM32 `pennyesc_uart` app.
2. Flash the ESP32 `main` environment.
3. Open the ESP32 monitor at `115200`.
4. Confirm `getStatus()` works before commanding motion.
5. Call `zeroPosition()`.
6. Try `setDuty(50)`, wait briefly, then `setDuty(0)`.
7. Increase duty slowly only after sensor and calibration status look correct.
8. Use `setPositionRad(6.2831853f)` for one mechanical turn.

## Current Wire Protocol

The active protocol is framed, addressed, and length-prefixed:

```text
[0]  0xAA start byte
[1]  header: high nibble = address, low nibble = command
[2]  payload length, 0..64
[3..] payload bytes
[last] CRC-8 over every previous byte, polynomial 0x07, initial 0x00
```

Commands are defined in `firmware/Lib/pennyesc_protocol.h`.

| Command | Code | Request payload | Normal response |
| --- | ---: | --- | --- |
| `PNY_CMD_GET_STATUS` | `0x1` | none | `pny_status_payload_t` |
| `PNY_CMD_SET_POSITION` | `0x2` | `int32 position_turn32` | `pny_status_payload_t` |
| `PNY_CMD_SET_DUTY` | `0x3` | `int16 duty` | `pny_status_payload_t` |
| `PNY_CMD_CAL` | `0x4` | calibration subcommand | calibration payload |
| `PNY_CMD_DEBUG` | `0x5` | debug subcommand | debug payload |
| `PNY_CMD_ZERO_POSITION` | `0x6` | none | `pny_status_payload_t` |
| `PNY_CMD_SET_VELOCITY` | `0x7` | `int32 velocity_turn32_per_s` | `pny_status_payload_t` |
| `PNY_CMD_SET_CONTROL` | `0x8` | `pny_control_payload_t` | `pny_status_payload_t` |
| `PNY_CMD_STOP` | `0x9` | none | `pny_status_payload_t` |
| `PNY_CMD_ENTER_BOOT` | `0xB` | `uint32 PNY_BOOT_MAGIC` | result byte |
| `PNY_CMD_SET_ADVANCE` | `0xC` | `int16 advance_deg` | `pny_status_payload_t` |
| `PNY_CMD_SET_QUIET` | `0xD` | `uint16 hold_ms` | result byte |

All multi-byte fields are little-endian.

## Units

| Value | Unit | Conversion |
| --- | --- | --- |
| Position | `turn32` | `65536 = 1 mechanical revolution` |
| Position radians | radians | `rad = turn32 * 2*pi / 65536` |
| Velocity | `turn32/s` | `rpm = turn32_per_s * 60 / 65536` |
| Duty | raw PWM | valid range is `-799..799` |
| Control gains | Q8 fixed-point | `value 256 = 1.0` |

## Learning Outline

1. **Get status only**: build the ESP32 demo, call `getStatus()`, print position, velocity, flags, and faults.
2. **Understand frames**: read `pnyproto.py` and encode one `GET_STATUS` request by hand.
3. **Use safe motion**: call `zeroPosition()`, then pulse small duties and stop.
4. **Position control**: command one turn with `setPositionRad(2*pi)` and watch `positionReached()`.
5. **Calibration awareness**: learn what `calValid()`, `PNY_FAULT_UNCALIBRATED`, and calibration tools mean before relying on position.
6. **Advanced control**: use Python `pennycal.py` for velocity, control gains, calibration, and debug capture until the Arduino wrapper grows those helpers.
7. **Boot/update path**: use `PennyEscBridge` only when updating STM32 firmware through the ESP32.

## Current Gaps

- The Arduino wrapper covers common status, duty, position, zero, advance, and boot commands, but not `STOP`, velocity, control gains, calibration, quiet mode, or debug capture yet.
- `PennyEsc` defaults to address `1`, matching the STM32 firmware default. Pass the address explicitly when using multiple ESCs.
- The wrapper drops bytes before each send. That is simple, but less robust than the Python client when a delayed reply arrives after a timeout.
- The ESP32 demo is status-only. It is safe, but it is not a complete interactive motor-control example.

## Calibration GUI Bridge

Add one global bridge object and call `bridge.begin(...)` in `setup()`. On ESP32 this starts a background bridge task, so the user sketch does not need bridge code in `loop()`.

The bridge listens for reserved `!#...` commands from the host. Normal sketch serial input is left alone unless it starts with that bridge prefix. While bridge mode is active, `PennyEsc` commands from the sketch return `false` instead of writing to the ESC UART, leaving the calibration GUI in control until it sends `!#bridge off`.
