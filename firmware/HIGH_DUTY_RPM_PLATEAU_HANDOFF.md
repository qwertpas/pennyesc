# High Duty RPM Plateau Handoff

## 2026-04-30 Update

Implemented a commutation observer timestamp fix in `firmware/pennyesc_libopencm3/src/main.c`.

The async TMAG angle sample was previously treated as if it occurred at I2C read completion (`sensor_i2c_end_us`). The run observer now timestamps that measured angle at async read start (`sensor_i2c_start_us`) for velocity and position prediction. This removes about `138-141us` of commutation lag from the high-speed path while keeping the reported I2C timing fields unchanged.

Build and upload:

- `python3.11 -m platformio run -d firmware/pennyesc_libopencm3 -e pennyesc_uart` passed.
- `BRIDGE_PORT=/dev/cu.usbmodem1301 ESC_ADDRESS=3 python3.11 -m platformio run -d firmware/pennyesc_libopencm3 -e pennyesc_uart -t uart_upload` passed and verified `image_len=12892`.

Live status sweep after flashing, with observer `LP4`, lead `0us`, advance `90deg`:

| Duty | Mean RPM | Clean |
| ---: | ---: | :--- |
| 75 | 3028 | yes |
| 100 | 6441 | yes |
| 125 | 10096 | yes |
| 150 | 12591 | yes |
| 175 | 14215 | yes |
| 200 | 15496 | yes |
| 225 | 16642 | yes |
| 250 | 17625 | yes |
| 300 | 20107 | yes |
| 350 | 22073 | yes |
| 400 | 22918 | no, stopped with `mct_delta=1` |

New data:

- `firmware/dynamic_metrics/high_duty_timestamp_start_20260430/sweep.csv`
- `firmware/dynamic_metrics/high_duty_timestamp_start_20260430/status_samples.csv`
- `firmware/dynamic_metrics/high_duty_timestamp_start_adv_sweep_20260430/summary.csv`

Advance spot check after the timestamp fix:

- `70deg` stayed clean through duty `400`, but only reached about `18.7k rpm`.
- `50deg` stayed clean through duty `400`, but only reached about `12.3k rpm`.
- `90deg` remains much faster, so the plateau was primarily timestamp/latency error, not static advance being too high.

## 2026-04-30 CC2 Scheduler Update

Implemented compare-scheduled sector switching in `firmware/pennyesc_libopencm3/src/main.c`.

- TIM21 CC1 still owns the deterministic `100us` TMAG/sensor cadence.
- TIM21 CC2 now schedules the next Hall sector switch at the observer-estimated phase boundary instead of waiting for the next 10 kHz sensor tick.
- The next async TMAG read is started earlier in the CC1 ISR, tied to the scheduled sensor tick.
- Removed the commutation velocity sign gate. Positive duty on this motor reports negative angle velocity, so the old gate disabled prediction and prevented CC2 scheduling from working.

Build and upload:

- `python3.11 -m platformio run -d firmware/pennyesc_libopencm3 -e pennyesc_uart` passed.
- `BRIDGE_PORT=/dev/cu.usbmodem1301 ESC_ADDRESS=3 python3.11 -m platformio run -d firmware/pennyesc_libopencm3 -e pennyesc_uart -t uart_upload` passed and verified `image_len=13276`.

Live status sweep after flashing, with observer `LP4`, lead `0us`, advance `90deg`:

| Duty | Mean RPM | Clean |
| ---: | ---: | :--- |
| 75 | 2961 | yes |
| 100 | 7092 | yes |
| 150 | 16240 | yes |
| 200 | 21274 | yes |
| 250 | 26037 | yes |
| 300 | 27681 | no, stopped with `mct_delta=1` |

New data:

- `firmware/dynamic_metrics/high_duty_cc2_signed_velocity_20260430/sweep.csv`
- `firmware/dynamic_metrics/high_duty_cc2_signed_velocity_20260430/status_samples.csv`

Notes:

- ISR max rose to about `152us` at the high-speed end, but `isr_overrun_count` stayed `0`.
- A first CC2 attempt that kept the old sign gate was stable but slow, topping out around `9k rpm`; that data is in `firmware/dynamic_metrics/high_duty_cc2_sector_sched_20260430/`.

## Summary

With the better power supply connected, the motor now reaches much higher speed than the earlier regression data, but RPM gain flattens strongly above roughly duty `200`. The user suspects angle estimation or commutation quality is degrading at higher speed, causing poor efficiency while commanded duty/current continue increasing.

Do not recalibrate the motor. ESC address is `3`; bridge port is `/dev/cu.usbmodem1301`.

## Current Firmware State

The test was run on the current uncommitted `firmware/pennyesc_libopencm3/src/main.c` firmware after commutation changes in this working tree.

Relevant observed behavior from this firmware:

- Runtime MCT guard is debounced.
- Low-latency commutation velocity path is present again.
- Default observer lead is `0us`.
- Status velocity overflow fix is present.

## Latest Data

Data and plot:

- `firmware/dynamic_metrics/rpm_vs_duty_status_continuous_better_supply_20260430_155906/sweep_corrected.csv`
- `firmware/dynamic_metrics/rpm_vs_duty_status_continuous_better_supply_20260430_155906/status_samples.csv`
- `firmware/dynamic_metrics/rpm_vs_duty_status_continuous_better_supply_20260430_155906/rpm_vs_duty_corrected.png`

Measurement method:

- Better power supply was connected.
- Ran a continuous ascending duty sweep so the motor stayed spinning between duty points.
- Used status velocity samples for the sweep because the capture command stops the motor at the end of each point.
- Each duty point settled for about `0.9s`.
- Each duty point then sampled status velocity for about `0.55s`.
- RPM below is mechanical RPM computed from `status.velocity_turn32_per_s`.
- Duty was set to `0` at the end.

## Results

| Duty | Mean RPM | Min RPM | Max RPM | Clean |
| ---: | ---: | ---: | ---: | :--- |
| 75 | 2731 | 2381 | 3210 | yes |
| 100 | 6027 | 5850 | 6132 | yes |
| 125 | 10173 | 9868 | 10380 | yes |
| 150 | 12627 | 12538 | 12704 | yes |
| 175 | 13691 | 13606 | 13782 | yes |
| 200 | 14278 | 14192 | 14359 | yes |
| 225 | 14758 | 14707 | 14848 | yes |
| 250 | 15093 | 14984 | 15194 | yes |
| 275 | 15382 | 15321 | 15453 | yes |
| 300 | 15567 | 15505 | 15667 | yes |
| 350 | 15856 | 15740 | 15935 | yes |
| 400 | 16138 | 16034 | 16233 | yes |
| 450 | 16156 | 16040 | 16230 | yes |
| 500 | 15849 | 15849 | 15849 | no, stopped with `mct_delta=1` |

## Symptom

RPM rises quickly up to around duty `200`, reaching about `14.3k rpm`. From duty `200` to duty `450`, commanded duty more than doubles, but mean RPM only increases from about `14.3k rpm` to about `16.2k rpm`.

The slope change is the main symptom:

- Duty `75 -> 200`: about `+11.5k rpm`.
- Duty `200 -> 450`: about `+1.9k rpm`.
- Duty `400 -> 450`: effectively flat within measurement noise.

At duty `500`, the motor did not remain in run mode. The recorded row ended with `mode=0`, `duty=0`, and `mct_delta=1`.

## Notes For The Next Agent

- The latest high-duty plateau data uses the better power supply.
- Do not compare these RPM numbers directly to the older stop-and-capture sweep without accounting for the different measurement method.
- The capture-based method is still useful for angle-slope RPM, but it stops the motor at each point and currently interacts with restart/recovery behavior.
- The user specifically suspects high-speed angle estimation or commutation quality because current continues increasing while RPM gain is small.
- This document intentionally does not propose fixes.
