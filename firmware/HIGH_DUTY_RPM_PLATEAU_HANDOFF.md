# High Duty RPM Plateau Handoff

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
- RPM below is mechanical RPM computed from `status.velocity_crads`.
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
