# PennyESC Motor Quickstart Guide


## 1. Wiring

| ESP32-S3 | STM32L011 (PennyESC) |
|----------|----------------------|
| GPIO 12 (RX) | PA9 (USART2 TX) |
| GPIO 13 (TX) | PA10 (USART2 RX) |
| GPIO 11 | GND |

The current UART app uses **230400 baud, 8N1** by default. The update/boot path uses **115200 baud**.

## 2. Flash Firmware

**STM32 (PennyESC):**

```bash
cd pennyesc_libopencm3
pio run -t upload
```

**ESP32-S3:**

```bash
cd esp32s3demo
pio run -t upload
```

**UART bootloader path:**

```bash
cd pennyesc_libopencm3
pio run -e pennyesc_uart -t seed_upload

cd ../esp32s3demo
pio run -e bootbridge -t upload --upload-port /dev/cu.usbmodem101

cd ..
BRIDGE_PORT=/dev/cu.usbmodem1301 ESC_ADDRESS=3 \
python3.11 -m platformio run -d firmware/pennyesc_libopencm3 -e pennyesc_uart -t uart_upload
```

`uart_upload` now checks the bridge port, bridge shell, and target address before it starts a build. If the normal app-to-bootloader handoff is unavailable, it fails quickly and leaves recovery to `uart_recover`.

See [`UART_UPDATE.md`](UART_UPDATE.md) for the reusable setup.

## ESP32 Library

ESP32 sketches can use [`Lib/pennyesc_arduino.h`](Lib/pennyesc_arduino.h) directly:

```cpp
#include "pennyesc_arduino.h"

PennyEsc esc(1);

void setup() {
    Serial.begin(115200);
    esc.begin(Serial1, PennyEscPins(), PENNYESC_BAUD_FAST);
}

void loop() {
    PennyEscStatus status;
    if (esc.getStatus(status)) {
        Serial.println(status.positionRad());
    }
}
```

The same header also provides `esc.setDuty(...)`, `esc.setPositionRad(...)`, `esc.pollEncoder(...)`, and `PennyEscBridge` for the USB boot/update bridge.

## 3. Spin the Motor

Open ESP32 serial monitor at 115200 baud:

```bash
pio device monitor -b 115200
```

The default `main` environment prints periodic status. For an interactive motor test, build and flash the `motortest` environment.

### Command Reference

| Command | Description |
|---------|-------------|
| `d<duty>` | Set duty cycle continuously (-799 to 799) |
| `p<duty>` | Pulse: run at duty for 0.5s then stop |
| `t<rad>` | Move to target position in radians |
| `?` | Show current stats |
| `h` | Show help |

### Example Commands

**Basic motor test:**

```
p100          # Pulse forward at low power for 0.5s
p-100         # Pulse reverse
```

**Continuous spinning:**

```
d150          # Spin forward at duty=150
d300          # Faster
d-200         # Reverse direction
d0            # Stop
```

**Position control:**

```
t0            # Go to zero position
t3.14         # Go to pi radians (half turn)
t6.28         # Go to 2*pi radians (one full turn)
t-6.28        # Go to -2*pi (one turn backwards)
t62.8         # Go to 10 full turns
```

**Position control with custom speed:**

The position controller uses the last set duty as its speed. Set duty first, then position:

```
d100          # Set slow speed
t6.28         # Move to 2*pi slowly

d200          # Set faster speed  
t0            # Return to zero faster
```

If no duty was set, position control defaults to duty=150.

**Pulse with specific duty and duration:**

The `p` command runs for a fixed 0.5 seconds:

```
p50           # Gentle pulse (low duty cycle)
p200          # Medium pulse
p500          # Strong pulse
p-300         # Reverse pulse
```

**Check status:**

```
?             # Shows: Pos: 6.28 rad, Vel: 0.0 RPM | Latency: 850/920/1200 us | Errs: 0 CRC, 0 TO
```

### Quick Test Sequence

1. `p100` - Verify motor spins (short pulse)
2. `p-100` - Verify reverse works
3. `d150` - Continuous spin, observe
4. `d0` - Stop
5. `?` - Check position value
6. `t0` - Command return to zero
7. `?` - Verify position reached ~0

## 4. Serial Protocol Reference

See [`PENNYESC_PROTOCOL_AND_ARDUINO.md`](PENNYESC_PROTOCOL_AND_ARDUINO.md) for the current ESP32 Arduino API, packet format, command table, safe first tests, and learning outline.

### Units

| Value | Unit | Conversion |
|-------|------|------------|
| Position | turn32 | radians = turn32 * 2*pi / 65536 |
| Velocity | turn32/s | RPM = turn32/s * 60 / 65536 |
| Duty | raw PWM | Range: -799 to +799 |

### Resync Behavior

Both sides treat `0xAA` as a packet-start marker. If received mid-packet, the parser resets and starts a new packet. This enables recovery from corruption or partial packets.

## 5. Magnetic Sensor Calibration

The TMAG5273 magnetic angle sensor requires calibration to correct for magnet mounting errors (tilt, offset, non-orthogonality). This produces a lookup table (LUT) that maps raw X/Y readings to corrected angles.

### Current Calibration Process

```mermaid
flowchart LR
    A[Flash Calibration FW] --> B[Log with MCUViewer]
    B --> C[Export CSV]
    C --> D[Run octant_centroid.py]
    D --> E[Copy LUT to angleLUT.c]
    E --> F[Flash Main FW]
```

**Step 1: Flash calibration firmware**

```bash
cd pennyesc_libopencm3
pio run -e calibration -t upload
```

This flashes `main_calibration.c` which spins the motor open-loop at fixed duty, stepping through 6 electrical states every 150ms. It reverses direction every 20 seconds.

**Step 2: Log sensor data**

Connect MCUViewer to the STM32 and add these variables to watch:
- `magx` - Raw X-axis reading
- `magy` - Raw Y-axis reading  
- `step_count` - Current commutation step

Start logging and let it run for at least one full cycle (40+ seconds) to capture both directions.

**Step 3: Export and process**

Export the MCUViewer log to CSV, then run the centroid script:

```bash
cd pennyesc_libopencm3/data
# Update TRAIN_CSV_FILENAME in the script to match your exported file
python octant_centroid.py
```

The script:
1. Groups sensor readings by `step_count`
2. Calculates centroid pseudo-index for each step
3. Maps centroids to true angles (known from step position)
4. Interpolates a smooth 2048-entry LUT

**Step 4: Update the LUT**

Copy the generated `OCTANT_LUT[2048]` array from the script output into:

```
pennyesc_libopencm3/src/angleLUT.c
```

**Step 5: Flash main firmware**

```bash
pio run -t upload  # Uses default environment (main.c)
```

### Future Improvements

**Automated calibration (no external logger):**
- Add UART command to trigger calibration mode
- Stream X/Y/step data over existing protocol
- Process on ESP32 or host PC

**Closed-loop calibration:**
- Use external encoder as ground truth
- Spin at constant velocity, correlate sensor angle with encoder
- Eliminates dependency on open-loop step timing

**Temperature compensation:**
- Log temperature during calibration runs at different temps
- Generate temp-indexed LUT or polynomial correction

**Finer resolution:**
- Increase commutation steps (e.g., 72 instead of 36 per revolution)
- Use slower stepping for more samples per step
- Improves interpolation accuracy at octant boundaries


## Key Files

- [`pennyesc_libopencm3/src/main.c`](pennyesc_libopencm3/src/main.c) - STM32 motor controller firmware
- [`pennyesc_libopencm3/src/main_calibration.c`](pennyesc_libopencm3/src/main_calibration.c) - Calibration firmware (open-loop stepping)
- [`pennyesc_libopencm3/src/angleLUT.c`](pennyesc_libopencm3/src/angleLUT.c) - Generated angle correction LUT
- [`pennyesc_libopencm3/data/octant_centroid.py`](pennyesc_libopencm3/data/octant_centroid.py) - LUT generation script
- [`esp32s3demo/src/main.cpp`](esp32s3demo/src/main.cpp) - ESP32 host controller with serial CLI
