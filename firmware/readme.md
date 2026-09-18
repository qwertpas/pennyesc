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

Other examples:

- `firmware/esp32s3demo_example/src/bridge.cpp`: USB serial bridge using `PennyEscBridge`.
- `firmware/esp32s3demo_example/src/brushed_position.cpp`: low-duty brushed servo position test.
- `firmware/esp32s3demo_example/src/position_sweep.cpp`: zeros the ESC, sets control gains, alternates position targets, and prints status.
- `firmware/esp32s3demo/src/capture.cpp`: captures angle and RPM in firmware at up to 1000 Hz for 200 ms, then prints CSV.

Common calls:


| Call                                               | Use                                                            |
| -------------------------------------------------- | -------------------------------------------------------------- |
| `esc.begin(Serial1, RX_pin, TX_pin)`               | Start ESC UART on custom pins.                                 |
| `esc.getStatus(status)`                            | Read position, velocity, duty, sensor data, flags, and faults. |
| `esc.getPosVel(data)`                              | Fast read of only encoder position and velocity.               |
| `esc.setDuty(duty)`                                | Set open-loop duty, `-799..799`. Start low.                    |
| `esc.sendPositionRad(rad)`                         | Move to an absolute position relative to current zero.         |
| `esc.zeroPosition()`                               | Set the current shaft position as zero.                        |

## BLDC commutation timing

The `pennyesc_uart` build uses TIM21 as a free-running 1 MHz clock and its second compare channel for sector edges. SysTick services sensor and control work every 100 us and divides ten ticks into the application's millisecond clock. It keeps running while the motor is idle. TIM21 has the highest interrupt priority, I²C is next, and SysTick is lower. The STM32L011 has TIM2 and TIM21; it does not have TIM22.

For the six-pole-pair motor, a sector lasts 98.04 us at 17,000 rpm and 33.33 us at 50,000 rpm. Each edge advances the sector sequence and schedules the next edge from the previous deadline, carrying fractional microseconds forward. Sensor corrections move a deadline by at most a quarter-sector. If a sector ISR fires during the calculation, its edge count carries that correction forward to the currently upcoming edge. Corrections within 4 us of an edge are applied to the following edge; discarding them caused nearly a full sector of error in an accelerating simulation near harmonics of the sensor tick. Expired compares generate an immediate timer event instead of waiting for the 16-bit timer to wrap. Adjacent Hall sectors change a single pin without first clearing all three inputs.

The BLDC sensor profile retains the committed firmware's continuous XY conversion, 1x averaging, and 8-bit X/Y results. Reads also include conversion status so duplicate, incomplete, reset, and diagnostic-failure results are rejected. Reads are paced by the 100 us control tick; accepted sample intervals on ESC1 are usually about 100 us. The I²C clock remains below the sensor's 1 MHz limit. This is faster acquisition than the discarded triggered XYX experiment, which produced fresh samples about 400 us apart and changed the motor's tuning.

The observer uses **I²C read start** as its timestamp reference. Fast alpha-beta gains are `(1/4, 1/32)` to reduce acceleration lag; mid and slow retain `(1/12, 1/192)` and `(1/16, 1/256)`. The default residual lead is **145 us**, with **90 electrical degrees** of advance. These combine into one phase: `pole_pairs * (estimated_angle + estimated_speed * (sample_age + lead)) + alignment + advance`. The 145 us value was selected from the measured 140–150 us working range on ESC1 after checking alignment; it is not a measured sensor-only delay. The legacy secant modes use five sample endpoints with actual timestamps.

Sensor initialization restores standard register reads before checking the device ID, since fast-read mode can survive an MCU reset. It acknowledges the latched undervoltage flag from the previous supply ramp. Other diagnostic bits remain set, and fresh conversion results still reject active faults. See the [TI TMAG5273 datasheet](https://www.ti.com/lit/ds/symlink/tmag5273.pdf) for read modes, conversion timing, and diagnostic registers.

An entire missed sector, an impossible speed estimate, or stale measurements shuts off PWM. Above low speed, the stale limit is half a predicted mechanical revolution (about 594 us at 50,000 rpm); otherwise it is 10 ms. Sensor cleanup happens outside the commutation interrupt. Mechanical phase wraps separately from the saturating accumulated position counter.

ESC1's static phase check found a −23.7° electrical alignment bias, consistent in forward and reverse sweeps. Its existing calibration was backed up and only the alignment field and CRC were changed (4669 → 8986); the affine transform and angle lookup table were preserved. The backup and corrected blobs are in `pennyesc_libopencm3/data/esc1-calibration-{before-alignment,aligned}-2026-09-17.bin`. This board-specific correction is stored in ESC1, not hard-coded into the firmware.

With the corrected alignment, lead was swept at fixed 90° advance using 1 kHz on-board speed capture and repeated duty ±400 acceleration runs. The selected 145 us setting completed three runs per direction without a speed collapse up to the test cutoff near 28,000 rpm. Relative to 120 us with the same corrected calibration and observer, median acceleration improved about 19% forward / 27% reverse over 18,000–22,000 rpm, and 37% / 72% over 22,000–26,000 rpm. There were only two reference runs and three selected-setting runs per direction; these are bench comparisons, not a global torque optimum. The 160 us setting collapsed in forward runs and 180 us collapsed in both directions, so neither was retained. See `pennyesc_libopencm3/data/esc1-torque-timing-2026-09-17.png` and `esc1-torque-timing-summary-2026-09-17.json`. Earlier duty-only comparisons are retained in `esc1-timing-comparison-2026-09-17.json`.

Brief steady checks at 145 us and 90° held approximately +26,100 / −26,800 rpm at duty ±300. Duty ±350 continued accelerating to the 29,000 rpm test cutoff. No collapse or reported fault occurred in those runs. These checks lasted 350 ms per command and do not establish thermal performance or a stable maximum speed. The final flashed default image was then checked without a lead/advance override through the GUI’s `SET_CONTROL` duty path: ±100 spun reliably, ±300 held about +26,200 / −27,000 rpm, and ±350 continued to the speed cutoff. That final check had no sensor, I²C, UART, or driver faults, and ended braked; details are in `esc1-final-torque-defaults-2026-09-17.json`.

The capture command releases the brake at startup, switches duty off at its duration limit, and becomes inactive on a commutation fault. `tools/pny_accel.py` randomizes short runs, stops after a speed drop or near 28,000 rpm, waits for rest, records calibration/image identifiers, and rejects collapsed runs from acceleration scoring. It restores 145 us lead and 90° advance afterward:

```bash
python3 firmware/tools/pny_accel.py --port /dev/cu.usbmodem101 --duty 400 --leads 140 145 150 --advances 90 --repeats 3 --output /tmp/esc1-acceleration.json
```

Acceleration is compared through matched RPM bands after 9 ms centered smoothing and persistent threshold crossings. It is a net torque proxy at the same load and inertia, not a measurement of torque in Nm, efficiency, or heating. Control ISR overruns of the 100 us budget still occur in short high-speed runs; the dedicated sector ISR has priority.

For further bench validation, log full status velocity, accepted sample intervals, I²C errors, and overruns. Scope Hall outputs and I²C to measure edge phase and jitter, and compare current at the same speed and load. Compact capture packets saturate above 32,767 rpm; use status or `getPosVel()` for higher speeds. Status `isr_us`/`isr_max_us` measure the control ISR including preemption, not the short sector ISR. **50,000 rpm remains a software timing test point, not a verified motor speed.**

## Development checks

Run from the repository root; these commands build and test without uploading:

```bash
python3 -m pytest firmware/tools/tests -q
pio run -d firmware/pennyesc_libopencm3 -e pennyesc_uart -e pennyesc_brushed_uart -e stepper_swd -e seed -e readdress -t buildprog
pio run -d firmware/esp32s3demo_example -t buildprog
```

The C checks compile production timing, sensor, and framing functions against mocked registers with undefined-behavior checks. They cover startup control dispatch, the idle clock, target timer availability, 17,000/50,000 rpm sector scheduling, timer wrap, late compares, corrections across ISR preemption and imminent edges, a combined 0–50,000 rpm / 50 ms ramp with simulated 0–60 us calculation time in both directions, observer prediction, sensor modes, initialization after reset, freshness/errors, and every address/command header. Observer simulations assume a specified sample delay; they do not determine the board's actual delay, interrupt latency, or motor dynamics.

The BLDC build uses 13,540 of 13,696 application flash bytes and 1,292 of 1,792 application RAM bytes; the remaining 500 RAM bytes hold the stack. Keep the startup parser and large reply buffers off the running control stack, and check nested interrupt stack use when changing these paths. Historical simulations such as `tools/observer_sim.py` describe earlier firmware and are retained with experimental sources and captured data.

## Brushed Motor Firmware

The `pennyesc_brushed_uart` build drives a brushed motor from OUTA to OUTB. Leave OUTC disconnected. Positive duty drives OUTA to OUTB, negative duty drives OUTB to OUTA, zero duty coasts, and `brake()` turns on the driver's low-side brake.

The brushed build is intended for a magnetic encoder on the geared output shaft. It reads full-resolution X/Y/Z field data and services sensor reads and motor control at 10 kHz; angle and velocity update only when a fresh conversion is accepted. It uses a small integer angle conversion directly from X/Y, so the output does not need to rotate through a full turn for calibration. `zeroPosition()`, position commands, status packets, addressing, daisy chaining, and UART firmware updates use the same API as the normal firmware.

Place the servo output away from its mechanical stops before zeroing it. First apply a small positive duty and confirm the reported position increases; swap the two motor leads if it decreases. Start with a current-limited 4.8-8.4V supply and a low control clip; the `brushed_position` example uses `60/799` and moves only `0.2` radians from zero.

Build the example controller:

```bash
pio run -d firmware/esp32s3demo_example -e brushed_position -t upload
```

Build or update the brushed PennyESC firmware:

```bash
ESC_ADDRESS=1 \
pio run -d firmware/pennyesc_libopencm3 -e pennyesc_brushed_uart -t uart_upload
```


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

The GUI can run static calibration, status checks, duty commands, brake, and advance commands. Sessions are saved under `firmware/penny-gui-sessions/`.



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
pio run -d firmware/esp32s3demo_example -e bridge -t upload
```

Scan for connected PennyESC addresses:

```bash
python3 firmware/tools/pnyboot.py scan
```

Update PennyESC firmware over UART after it has been seeded and connected to the ESP32 bridge:

```bash
ESC_ADDRESS=1 \
pio run -d firmware/pennyesc_libopencm3 -e pennyesc_uart -t uart_upload
```

Change a board from one ESC address to another, then use the new address for later commands:

```bash
CURRENT_ESC_ADDRESS=1 NEW_ESC_ADDRESS=2 \
pio run -d firmware/pennyesc_libopencm3 -e readdress -t uart_readdress
```

Seed a PennyESC over an STLink. Use this for a fresh board or a board that is bricked and needs the UART bootloader restored:

```bash
ESC_ADDRESS=1 \
pio run -d firmware/pennyesc_libopencm3 -e seed -t seed_upload
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
| `PNY_CMD_BRAKE`         | `0x9` | none                          | status                  |
| `PNY_CMD_SEND_POSITION` | `0xA` | `int32 position_turn32`       | none                    |
| `PNY_CMD_ENTER_BOOT`    | `0xB` | `uint32 PNY_BOOT_MAGIC`       | result byte             |
| `PNY_CMD_SET_ADVANCE`   | `0xC` | `int16 advance_deg`           | status                  |
| `PNY_CMD_SET_QUIET`     | `0xD` | `uint16 hold_ms`              | result byte             |
| `PNY_CMD_GET_POS_VEL`   | `0xE` | none                          | `pny_pos_vel_payload_t` |


All multi-byte fields are little-endian. `PNY_CAL_READ_BLOB` (`0x8` under `PNY_CMD_CAL`) takes the subcommand byte, a `uint16` offset, and a `uint8` count (1–63). It returns a result byte followed by the requested calibration bytes on success. Invalid ranges and missing calibration return only an error byte. `Stm32Client.cal_read_blob()` assembles all 640 bytes and verifies the CRC, allowing a calibration backup before changes.

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
| `firmware/esp32s3demo_example/src/bridge.cpp` | ESP32 bridge firmware.                       |
| `firmware/pennyesc_libopencm3/src/main.c`     | STM32 PennyESC app.                          |
| `firmware/tools/pennycal.py`                  | Host calibration CLI and shared GUI backend. |
| `firmware/tools/pnyboot.py`                   | Host UART boot/update tool.                  |
