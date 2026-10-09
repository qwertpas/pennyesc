# PennyESC Firmware

## Wiring

PennyESC requires 4 wires (in order on the PennyESC PCB):

- GND
- UART RX (RX of ESC, TX of your microcontroller)
- UART TX (TX of ESC, RX of your microcontroller)
- 5-15V power input

Multiple PennyESCs can be daisy chained on the same UART bus because it is implemented with open-drain outputs. So, the TX line requires a pull-up resistor to 3.3V. Each PennyESC on the same bus must have a unique address. See the Flashing section on changing the address.

A large capacitor (>100uF) is recommended between the power input and GND. Without it, the driver may turn off during high current (mct_faults will increment in the motor status in the GUI).

ESP32-H2-DevKitM-1 bridge: GPIO 0 receives ESC TX, GPIO 1 sends to ESC RX,
and GPIO 2 is driven low for the UART ground connection. Power the ESC separately.
Connect the H2's native USB port to the host. The `h2_bridge` environment uses
pioarduino for H2 Arduino support, pinned to a release supported by PlatformIO Core 6.1.19.

```bash
pio run -d firmware/esp32s3demo -e h2_bridge -t upload
python3 firmware/tools/calibrate.py --port /dev/cu.usbmodemPORT --address ADDRESS calibrate
python3 firmware/tools/calibrate.py --port /dev/cu.usbmodemPORT --address ADDRESS verify
```

Calibration uses six pole pairs (12 motor poles). Run only one process on the USB port at a time.
The Maxon ECX FLAT 22 setup uses ESC address 2 and the BLDC `maxon_esc`
environment. After uploading the H2 bridge, use Penny GUI on `/dev/cu.usbmodem1101`
with address 2 to calibrate. The previous ESC application is backed up in
`pennyesc_libopencm3/data/esc2-app-before-maxon-2026-10-01.bin`.

On October 1, address 2 was calibrated with a 0.86° maximum fit error and 1.11°
forward/reverse disagreement (blob CRC `C5158565`). Exact flash readback matched,
and brief duty ±80 checks ran in both directions without reported faults. Capture
data and the blob are in `pennyesc_libopencm3/data/esc2-maxon-calibration-2026-10-01.*`.
The brake command now succeeds without calibration; encoder speed tracking starts
only when available, so Penny GUI can brake a fresh BLDC ESC before calibration.



## Arduino API

For a single BLDC motor with live sine sliders, run:

```bash
python3 firmware/bldc-sine-gui.py --port /dev/cu.usbmodem1101 --address 2
```

Connect brakes and zeros the motor. Start runs the sine; Stop, disconnect, and
window close brake it. Amplitude and period update live, alongside Kp, Kd, duty
limit, position deadband, friction feedforward, and feedforward smoothing.
Defaults for the Maxon are 1 rad amplitude, 4 s period, Kp 100, Kd 2, duty limit
150, deadband 0.01 rad, and feedforward 75. Larger smoothing values soften the
feedforward around reversals. Friction assist also fades near the target and
becomes zero when the requested velocity would push away from the target.
PD torque remains active during stalls; the GUI does not stop for a stall or
position tracking error. The position deadband smoothly removes the
proportional correction near the target; derivative damping remains active.
The existing 0.06 rad reached-position flag tolerance does not suppress PD duty.
Firmware `maxon_esc` must include the adjustable deadband and trajectory support.
The GUI reports measured command and feedback rates. Its serial worker streams
without waiting for each reply, preserves partial frames, and batches display
updates. The plot redraws at 30 fps with 100 samples/s. Isolated late replies
keep the connection alive; 250 ms without valid feedback stops the worker and
attempts to brake. Torque reversals switch commutation directly without restarting
the encoder. UART remains at 921600 baud.

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

Address 3 was recalibrated twice more. The first new blob (`DBBD2D73`) completed three two-second duty 500 runs near 38,500 rpm. A following calibration capture failed to solve because the prior brake command left the brake pin asserted; all 216 points had nearly identical X/Y values. Calibration start now releases that pin before the first motor step. With the fix flashed, a calibration started from the braked state produced a valid second blob (`4A25FAA1`). It completed three duty 500, two duty 400, and one reverse duty 500 two-second runs without a speed collapse or reported fault. The XIAO bridge was reflashed afterward, and address 3 still reported the second blob as valid. Calibration backups and run summaries are in `pennyesc_libopencm3/data/esc3-calibration-*-2026-09-24.bin` and `esc3-recalibration-2026-09-24.json`.

On September 24, ESC address 3 was flashed with the absolute-phase scheduler and tuned on hardware. Its observer lead defaults to **100 us**; other addresses retain 120 us. At 120 us, duty 400 repeatedly reached about 38,500 rpm and then lost more than 5,000 rpm. Fast, mid, and slow observer gains all showed that drop. At 100 us, three two-second duty 500 runs held about 39,450 rpm and three duty 400 runs held about 28,550 rpm; one reverse duty 500 run held about 34,670 rpm. None reported a driver or I²C fault. The 100 us runs still accumulated control-service overruns (about 3,000 per two-second duty 500 run), so the overrun counter alone does not identify the cause of the speed collapse. These are short unloaded runs measured by the on-board sensor, without an independent tachometer. The connected STP2S displayed 12.00 V and about 0.44–0.46 A at idle and duty 300; its reading has not been established as motor current. See `pennyesc_libopencm3/data/esc3-commutation-2026-09-24.json`.

**Experimental changes remain unvalidated. ESC1 was restored to `d8a6f2b` after the September 17 tests.** Absolute phase scheduling passed three short 600-duty tests but later collapsed at 700 duty with both tested observer gains. A conversion-synchronized SCL/XYX experiment sampled correctly while braked and ran at duty 100, but lost communication at duty 600, including brake acknowledgment; the user power-cycled ESC1. That sensor change was removed from maintained code and preserved as a patch and image in `data/esc1-scl-experiment-2026-09-17.*`. Do not interpret any earlier individual passing run as validation across duty or speed. No maximum-torque claim is established.

The `pennyesc_uart` build uses TIM21 as a free-running 1 MHz clock and its second compare channel for sector edges. SysTick services sensor and control work every 100 us and divides ten ticks into the application's millisecond clock. It keeps running while the motor is idle. TIM21 has the highest interrupt priority, I²C is next, and SysTick is lower. The STM32L011 has TIM2 and TIM21; it does not have TIM22.

For the six-pole-pair motor, a sector lasts 98.04 us at 17,000 rpm and 33.33 us at 50,000 rpm. The sensor service publishes an electrical phase, its timestamp, and its velocity together with TIM21 masked. Each sector event evaluates `phase(t) = phase_at_update + phase_rate * (t - update_time)` and selects `floor(6 * wrapped_phase / 65536)`. It schedules the next boundary from that absolute phase. Time spent calculating the update is included before selecting the output sector. A delayed interrupt selects the current sector instead of replaying an obsolete event. This replaces the free-running sector counter and its quarter-sector correction limit, which could remain out of phase with the observer. Integer period rounding introduces less than 1 us of edge-time error, not cumulative phase drift. Expired compares generate an immediate event instead of waiting for the timer to wrap. Adjacent Hall sectors change a single pin without first clearing all three inputs.

The BLDC sensor profile retains the committed firmware's continuous XY conversion, 1x averaging, and 8-bit X/Y results. Reads also include conversion status so duplicate, incomplete, reset, and diagnostic-failure results are rejected. Reads are paced by the 100 us control tick; accepted sample intervals on ESC1 are usually about 100 us. The I²C clock remains below the sensor's 1 MHz limit. This is faster acquisition than the discarded triggered XYX experiment, which produced fresh samples about 400 us apart and changed the motor's tuning.

The observer uses **I²C read start** as its timestamp reference. Fast alpha-beta gains are `(1/8, 1/128)`; mid and slow use `(1/12, 1/192)` and `(1/16, 1/256)`. The residual lead is **100 us for address 3** and **120 us for other addresses**, with **90 electrical degrees** of advance. These combine into one phase: `pole_pairs * (estimated_angle + estimated_speed * (sample_age + lead)) + alignment + advance`. The lead is an empirical total compensation, not a measured sensor-only delay. The legacy secant modes use five sample endpoints with actual timestamps.

Sensor initialization restores standard register reads before checking the device ID, since fast-read mode can survive an MCU reset. It acknowledges the latched undervoltage flag from the previous supply ramp. Other diagnostic bits remain set, and fresh conversion results still reject active faults. See the [TI TMAG5273 datasheet](https://www.ti.com/lit/ds/symlink/tmag5273.pdf) for read modes, conversion timing, and diagnostic registers.

An impossible speed estimate or stale measurements shuts off PWM. An interrupt delayed by a sector is recovered using the current absolute phase; it no longer shuts off output solely for missing that old deadline. Above low speed, the stale limit is half a predicted mechanical revolution (about 594 us at 50,000 rpm); otherwise it is 10 ms. Sensor cleanup happens outside the commutation interrupt. Mechanical phase wraps separately from the saturating accumulated position counter.

ESC1's static phase check found a −23.7° electrical alignment bias, consistent in forward and reverse sweeps. Its existing calibration was backed up and only the alignment field and CRC were changed (4669 → 8986); the affine transform and angle lookup table were preserved. The backup and corrected blobs are in `pennyesc_libopencm3/data/esc1-calibration-{before-alignment,aligned}-2026-09-17.bin`. This was an experimental board-specific correction. ESC1 now uses the original alignment (4669, CRC `e835e070`), restored before the duty-500 comparison below. Both backups are retained; firmware does not rewrite calibration.

Earlier, with the experimental corrected alignment, lead was swept at fixed 90° advance using 1 kHz on-board speed capture and repeated duty ±400 acceleration runs. The then-selected 145 us setting completed three runs per direction without a speed collapse up to the test cutoff near 28,000 rpm. Relative to 120 us with the same corrected calibration and observer, median acceleration improved about 19% forward / 27% reverse over 18,000–22,000 rpm, and 37% / 72% over 22,000–26,000 rpm. There were only two reference runs and three selected-setting runs per direction; these are bench comparisons, not a global torque optimum. The 160 us setting collapsed in forward runs and 180 us collapsed in both directions, so neither was retained. See `pennyesc_libopencm3/data/esc1-torque-timing-2026-09-17.png` and `esc1-torque-timing-summary-2026-09-17.json`. Earlier duty-only comparisons are retained in `esc1-timing-comparison-2026-09-17.json`.

Brief steady checks at 145 us and 90° held approximately +26,100 / −26,800 rpm at duty ±300. Duty ±350 continued accelerating to the 29,000 rpm test cutoff. No collapse or reported fault occurred in those runs. These checks lasted 350 ms per command and do not establish thermal performance or a stable maximum speed. That earlier default image was then checked without a lead/advance override through the GUI’s `SET_CONTROL` duty path: ±100 spun reliably, ±300 held about +26,200 / −27,000 rpm, and ±350 continued to the speed cutoff. That final check had no sensor, I²C, UART, or driver faults, and ended braked; details are in `esc1-final-torque-defaults-2026-09-17.json`.

The capture command releases the brake at startup, switches duty off at its duration limit, and becomes inactive on a commutation fault. `tools/pny_accel.py` randomizes short runs, stops after a speed drop or near 28,000 rpm, waits for rest, records calibration/image identifiers, and rejects collapsed runs from acceleration scoring. It restores 120 us lead and 90° advance afterward:

```bash
python3 firmware/tools/pny_accel.py --port /dev/cu.usbmodem101 --duty 400 --leads 100 120 140 --advances 90 --repeats 3 --output /tmp/esc1-acceleration.json
```

Acceleration is compared through matched RPM bands after 9 ms centered smoothing and persistent threshold crossings. It is a net torque proxy at the same load and inertia, not a measurement of torque in Nm, efficiency, or heating. Control ISR overruns of the 100 us budget still occur in short high-speed runs; the dedicated sector ISR has priority.

The initial duty-500 comparison held calibration at the original CRC `e835e070`. Reducing lead from 145 to 120 us allowed short runs near 35,000 rpm, but a later 600-duty test still stopped output near 41,150 rpm. Thus the earlier inference that excessive lead alone explained the regression was incorrect. The old firmware (`d8a6f2b`) had held about 28,000 rpm at duty 500; its absolute sector selection was an important behavior lost in the rewrite.

A temporary diagnostic build of the counter-based scheduler measured up to 35 us of disagreement between its next deadline and the observer's requested deadline above 20,000 rpm. At 45,000 rpm that is approximately 57 electrical degrees, almost a complete sector. Observer innovations also reached about 135 electrical degrees. These are internal disagreements, not an independent rotor-angle measurement. The diagnostic code changes execution timing and was not used as the final firmware. The uninstrumented counter-based scheduler reproduced the 600-duty output shutdown; absolute phase selection then completed three three-second 600-duty runs near 45,000–46,000 rpm, with lead still at 120 us, advance still at 90°, the same observer gains, and the original calibration. The first two-second 700-duty run completed near 53,500 reported rpm. Diagnostics, source, comparisons, and subsequent failures are retained in `pennyesc_libopencm3/data/esc1-sector-2026-09-17.json`.

The change restores a scheduling invariant: every event selects the sector corresponding to the published angle prediction at the current timer time. It does not eliminate sensor measurement-age uncertainty or establish maximum torque/current alignment. The sensor INT pin is unconnected on this PCB, and continuous XY conversions are not synchronized to I²C reads. Read-start lead remains an empirical compensation. Control ISR overruns remain measurable at high speed.

Use full-width status telemetry for this check: the compact capture saturates at 32,767 rpm and the earlier acceleration tool stops near 28,000 rpm, below the observed failure. `tools/pny_speed.py` uses the GUI’s control command, preserves flashed timing defaults, records actual duty/mode and wall-clock sample times, stops on a persistent 5,000 rpm drop or output shutdown, and brakes on completion or error. Run only one bridge/serial process at a time; close Penny GUI first.

```bash
python3 firmware/tools/pny_speed.py --port /dev/cu.usbmodem101 --duty 500 --duration 3 --repeats 3 --image firmware/pennyesc_libopencm3/.pio/build/pennyesc_uart/firmware.bin --output /tmp/esc1-speed.json
```

The earlier duty-500 data remains in `pennyesc_libopencm3/data/esc1-duty500-2026-09-17.json`. It is superseded as evidence of a general fix. These are short unloaded bench tests, not proof of stability at every duty, load, temperature or direction. ISR overruns must not be mistaken for driver fault counts.

For further bench validation, log full status velocity, accepted sample intervals, I²C errors, and overruns. Scope Hall outputs and I²C to measure edge phase and jitter, and compare current at the same speed and load. Compact capture packets saturate above 32,767 rpm; use status or `getPosVel()` for higher speeds. Status `isr_us`/`isr_max_us` measure the control ISR including preemption, not the short sector ISR. Speeds above 50,000 rpm have been reported in the 700-duty bench run, but have not been independently verified with a tachometer.

## Development checks

Run from the repository root; these commands build and test without uploading:

```bash
python3 -m pytest firmware/tools/tests -q
pio run -d firmware/pennyesc_libopencm3 -e pennyesc_uart -e pennyesc_brushed_uart -e stepper_swd -e seed -e readdress -t buildprog
pio run -d firmware/esp32s3demo_example -t buildprog
```

The C checks compile production timing, sensor, and framing functions against mocked registers with undefined-behavior checks. They cover startup control dispatch, the idle clock, target timer availability, 17,000/50,000/60,000 rpm sector scheduling, timer wrap, late compares, immediate sector correction after large phase steps and ISR preemption, recovery after a delayed sector event, a combined 0–50,000 rpm / 50 ms ramp with simulated 0–60 us calculation time in both directions, observer prediction, sensor modes, initialization after reset, freshness/errors, and every address/command header. Observer simulations assume a specified sample delay; they do not determine the board's actual delay, interrupt latency, or motor dynamics.

The BLDC build uses 13,336 of 13,696 application flash bytes and 1,292 of 1,792 application RAM bytes; the remaining 500 RAM bytes hold the stack. Keep the startup parser and large reply buffers off the running control stack, and check nested interrupt stack use when changing these paths. Historical simulations such as `tools/observer_sim.py` describe earlier firmware and are retained with experimental sources and captured data.

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

BLDC `SET_CONTROL` accepts an optional trailing `uint16` position deadband in
turn32 units; the usual 10-byte payload sets that deadband to zero.
`SEND_POSITION` also accepts a 10-byte trajectory payload (`int32 position`,
`int32 velocity`, `int16 feedforward duty`) in addition to its usual 4-byte
position. The trajectory fields update together without a reply. The sine GUI
sends these and requests position/velocity at 500 Hz. The `maxon_esc` controller
runs at 5 kHz to leave CPU time for UART service; its former 10 kHz loop dropped
received frames under motion. Friction
feedforward is `strength * tanh(target_velocity / smoothing)`, so it transitions
continuously through zero at each reversal.

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
