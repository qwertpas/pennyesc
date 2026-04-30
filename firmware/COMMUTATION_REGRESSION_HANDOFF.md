# Commutation Regression Handoff

## Summary

The current uncommitted `main.c` firmware spins slower for the same duty than previous commit `edfde39` (`use fresh commutation timestamp`). This supports the user observation that the new firmware draws more current for the same commanded duty and likely has worse commutation timing/efficiency.

Do not recalibrate the motor. ESC address is `3`, bridge is `/dev/cu.usbmodem1301`.

## Measured A/B

Data and plot:

- `firmware/dynamic_metrics/duty_rpm_compare_20260430_144348/combined.csv`
- `firmware/dynamic_metrics/duty_rpm_compare_20260430_144348/rpm_vs_duty.png`

Measurement method:

- Flashed current uncommitted firmware, swept duties `[25, 50, 75, 100, 125, 150, 175, 200]`.
- Flashed previous firmware `edfde39`, repeated same sweep.
- For each point: duty 0, wait, set duty, settle `0.8s`, capture angle at `500Hz` for `320ms`.
- RPM was computed offline from captured angle slope, not from `status.velocity_crads`, to avoid status overflow/wrap artifacts.
- Duty was set back to 0 after every point.

Key results, absolute mechanical RPM:

| Duty | Current firmware | Previous `edfde39` |
| ---: | ---: | ---: |
| 75 | 1706 | 1457 |
| 100 | 3170 | 3032 |
| 125 | 3307 | 4030 |
| 150 | 3334 | 4234 |
| 175 | 3422 | 4441 |
| 200 | 3449 | 4478 |

Current firmware plateaus around `3.4k rpm` by duty 125. Previous firmware keeps increasing to about `4.5k rpm` by duty 200.

## Relevant Firmware State

Current branch HEAD is `edfde39`, but `firmware/pennyesc_libopencm3/src/main.c` has uncommitted edits. The relevant current changes are:

- Fixed FIR32 velocity overflow by avoiding `sum * 10000` 32-bit overflow.
- Removed the 3-sample commutation velocity path.
- Runtime MCT check stops the motor if MCT config is lost.
- MCT config writes are now exact writes, not read-modify-write.
- `MCT_EXPECTED_CONTROL2A` was changed back to open-drain SDO mode.

Previous firmware for comparison:

- Commit `edfde39`.
- It uses the scheduled 1 MHz TIM21 / CC1 sensor-tick architecture, but without the later velocity overflow fix and MCT stop-only runtime guard.

## Important Findings

1. The velocity overflow was real.
   Before the fix, status/capture behavior showed apparent sign reversal around the old 32-bit overflow region. After fixing, duty 100 could report roughly `8k rpm` without the velocity estimate wrapping.

2. The high-duty MCT reset/config-loss is real.
   At high duty, SWD-read debug values showed MCT registers back at reset/default-like values:
   `bad_mask=0x7f`, `IC=0x0000`, `CTRL2A=0x0060`, `CTRL3=0x0046`, `CTRL4=0x0010`, `CTRL7=0x0001`, `CTRL8=0x0000`.
   The runtime guard now stops instead of continuing with unknown MCT state.

3. The commutation regression is separate from MCT reset.
   The duty-vs-rpm comparison used duties up to 200 and showed `mct_delta=0` for both current and previous firmware at all measured points. The current firmware is still slower, so the efficiency loss is likely from commutation timing/angle/velocity behavior, not from MCT reset handling.

4. Direction/sign conventions are still easy to misread.
   Previous firmware measured negative RPM for positive duty, while current measured positive RPM in the A/B sweep. Use absolute RPM for speed comparisons unless explicitly checking direction behavior.

## Most Likely Regression Areas

Focus on differences in commutation angle prediction and velocity filtering between current uncommitted `main.c` and `edfde39`:

- `comm_velocity_for_direction()` currently clamps lead prediction to zero when velocity sign does not match command direction. This may be wrong because measured velocity sign can differ from commanded direction depending on sensor/LUT sign convention.
- Current commutation velocity now uses FIR32 plus additional LP filtering for `PNY_OBSERVER_LP4`. This adds more delay than previous behavior and may make sector selection late.
- Current `observer_position_at_tick()` predicts from `comm_scheduler.position_tick` using `observer_lead_us`, but the timestamp/source of `observer_state.position_turn32` and `comm_velocity_turn32_per_s` may not match the sensor sample timing.
- Capture and run paths update `current_advance_turn16`; make sure the A/B tests use the same advance (`90 deg`) and observer lead (`350 us`).
- Current `MCT_EXPECTED_CONTROL2A` is open drain, while older `edfde39` expected push-pull. User had previously said open drain was known working, but verify this is not altering SPI read reliability or MCT behavior.

## Suggested Next Steps

1. Keep the velocity overflow fix. It is a real bug.
2. Temporarily disable only the sign clamp in `comm_velocity_for_direction()` and rerun the same duty-vs-rpm sweep.
3. Compare sector timing from current vs `edfde39`: log estimated electrical phase, selected sector, `comm_velocity_turn32_per_s`, `velocity_turn32_per_s`, and sample age for a fixed duty 150 run.
4. Try using raw FIR32 velocity directly for commutation lead with no extra LP4 filtering, then sweep duty-vs-rpm again.
5. If RPM improves, retune `observer_lead_us`; the old optimum around `350us` may no longer apply after timing/filter changes.
6. Keep runtime MCT stop guard for safety, but do not let it rewrite/blank PWM during normal low-duty comparison tests.

## Reproduction Commands

Current firmware upload:

```sh
BRIDGE_PORT=/dev/cu.usbmodem1301 ESC_ADDRESS=3 \
python3.11 -m platformio run -d /Users/chris/Kicad/pennyesc/firmware/pennyesc_libopencm3 -e pennyesc_uart -t uart_upload
```

Previous firmware upload:

```sh
rm -rf /tmp/pennyesc-edfde39
git worktree add --detach /tmp/pennyesc-edfde39 edfde39
BRIDGE_PORT=/dev/cu.usbmodem1301 ESC_ADDRESS=3 \
python3.11 -m platformio run -d /tmp/pennyesc-edfde39/firmware/pennyesc_libopencm3 -e pennyesc_uart -t uart_upload
```

Always send duty 0 at the end of tests.
