# PennyESC Motor Quickstart Guide


## 1. Wiring

| ESP32-S3 | STM32L011 (PennyESC) |
|----------|----------------------|
| GPIO 13 (TX) | PA10 (USART2 RX) |
| GPIO 12 (RX) | PA9 (USART2 TX) |
| GND | GND |

Both devices use **921600 baud, 8N1** between them.

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

## 3. Spin the Motor

Open ESP32 serial monitor at 115200 baud:

```bash
pio device monitor -b 115200
```

You should see: `BLDC Controller Ready`

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

**UART Settings:** 921600 baud, 8N1, no flow control

**CRC-8:** Polynomial 0x07 (CRC-8/CCITT), initial value 0x00

### Packet Structure

All packets start with `0xAA` and end with a CRC-8 byte computed over all preceding bytes.

```
TX: [START=0xAA] [CMD] [PAYLOAD...] [CRC8]
RX: [START=0xAA] [STATUS] [POS_0..3] [VEL_0..3] [CRC8]
```

### Commands (Host to PennyESC)

| Command | Code | Payload | Total Length |
|---------|------|---------|--------------|
| Poll | 0x03 | none | 3 bytes |
| Set Duty | 0x02 | int16 (little-endian) | 5 bytes |
| Set Position | 0x01 | int32 centiradians (LE) | 7 bytes |

### Response (PennyESC to Host)

Always 11 bytes:

| Byte | Field | Description |
|------|-------|-------------|
| 0 | START | 0xAA |
| 1 | STATUS | Bit 0: position reached, Bit 1: error |
| 2-5 | position | int32, centiradians (1 crad = 0.01 rad) |
| 6-9 | velocity | int32, centiradians/second |
| 10 | CRC8 | CRC of bytes 0-9 |

### Byte-Level Examples

**Example 1: Poll Command**

Request current position and velocity:

```
TX: AA 03 8E
     │  │  └── CRC8 of [AA 03]
     │  └───── CMD_POLL
     └──────── START_BYTE
```

**Example 2: Set Duty = 150**

Duty 150 = 0x0096 (little-endian: 96 00):

```
TX: AA 02 96 00 E7
     │  │  └──┴── int16 duty = 150
     │  └──────── CMD_SET_DUTY
     └─────────── START_BYTE
```

**Example 3: Set Duty = -150**

Duty -150 = 0xFF6A (little-endian: 6A FF):

```
TX: AA 02 6A FF 2C
     │  │  └──┴── int16 duty = -150
     │  └──────── CMD_SET_DUTY
     └─────────── START_BYTE
```

**Example 4: Set Position = 628 crad (2*pi radians)**

Position 628 = 0x00000274 (little-endian: 74 02 00 00):

```
TX: AA 01 74 02 00 00 9A
     │  │  └────────┴── int32 position = 628 crad
     │  └────────────── CMD_SET_POSITION
     └───────────────── START_BYTE
```

**Example 5: Response Parsing**

Response when motor is at position 1234 crad, velocity 5000 crad/s:

```
RX: AA 00 D2 04 00 00 88 13 00 00 xx
     │  │  └────────┴── position = 0x000004D2 = 1234 crad (12.34 rad)
     │  │              └────────┴── velocity = 0x00001388 = 5000 crad/s
     │  └── status = 0 (moving)
     └───── START_BYTE

Position reached (status bit 0 set):
RX: AA 01 ...
     │  └── status = 1 (position reached)
```

### Units

| Value | Unit | Conversion |
|-------|------|------------|
| Position | centiradians | 1 crad = 0.01 rad = 0.573° |
| Velocity | centiradians/s | 1 crad/s = 0.00955 RPM |
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

