# Incremental Hardware Plan For Observer Metrics

## Summary
Measure candidate torque/angle-quality metrics first using the current raw-angle commutation, then compare metric consistency before adding the simple velocity-lead observer. All tests use short windows to avoid overheating and always end with `duty 0`.

## Baseline Sanity Checks
- Before metric tests, run a low-risk sensor sanity pass at duties `[100, 150]`, max `1 s` each.
- Verify position and velocity are reasonable:
  - `position_crad` changes monotonically for a steady direction.
  - `velocity_crads` sign matches commanded duty direction.
  - rpm trend rises after duty is applied and falls/stabilizes after `duty 0`.
  - `angle_turn16` changes smoothly without large discontinuities except normal wrap.
  - `faults=0`, `mct_faults` does not increment unexpectedly.
- If these fail, stop and debug sensor/calibration before metric collection.

## Metric Collection
- Use duties `[100, 150, 200, 250]`.
- Use advances `[30, 50, 70, 90, 110, 130, 150]`.
- For each direction, duty, and advance:
  - set advance
  - set duty
  - collect for at most `1.5 s`
  - set `duty 0`
  - wait a short cooldown, at least `0.5 s`
- Record raw samples at `100 Hz`: timestamp, angle, position, velocity, rpm, duty, advance, direction, faults, mct_faults.
- Compute metrics:
  - acceleration from rpm slope over the first `200-500 ms`
  - final speed from mean rpm over the last `300 ms`
  - velocity ripple from steady-state `std(rpm) / mean(rpm)`
  - phase dither result by comparing neighboring advances around each local best
  - forward/reverse symmetry by comparing best advance and score per duty

## Consistency Analysis
- Compare metrics after all baseline runs before choosing a tuning target.
- Mark a metric as consistent if it chooses similar best advances across adjacent duties and both directions.
- Prefer metrics in this order unless data contradicts it:
  - acceleration consistency
  - speed reached consistency
  - low ripple as a stability veto
  - forward/reverse symmetry as a bias check
- Do not auto-tune from a metric unless it agrees with at least one other metric and does not land in a high-ripple region.

## Observer Test
- Add only a simple velocity-lead observer after baseline metrics are reviewed.
- Observer form:
  `estimated_angle = measured_angle + velocity * lead_time`
- Keep TMAG in `1x XY`.
- Sweep lead times `[-200, -100, 0, 100, 200, 300, 400, 600, 800] us`.
- Re-run the same duty/advance metric flow, but with shorter focused tests around the best baseline advances.
- Compare raw-angle vs observer results using the same consistency analysis.

## Assumptions
- Hardware target is ESC address `3` on `/dev/cu.usbmodem1301`.
- No coast-deceleration metric in this pass.
- No calibration blob changes during metric testing.
- Every hardware test command sequence must force `duty 0` in cleanup, and no powered hold exceeds `2 s`.
