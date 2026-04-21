## `calibrationtest_gui.py` Host-Only Static + Dynamic Calibration Harness

**Summary**
- Add a new host-side GUI script, `firmware/esp32s3demo/calibrationtest_gui.py`, that runs the existing static calibration flow first, commits the static blob exactly as today, then runs the proposed dynamic free-spin tests and visualizes the results.
- Do **not** change STM32 or ESP32 firmware, protocol, flash blob format, or runtime commutation behavior in this step. Dynamic results are diagnostic-only.
- Leave `calibration_gui.py` unchanged as the current production static-calibration tool. The new GUI is an experimental test harness.
- Auto-save each session to a timestamped results folder so the user can compare runs offline before deciding what firmware changes to make later.
- Use `90` electrical degrees as the default runtime advance for dynamic testing, but allow the user to change it from the GUI terminal command line before starting or rerunning sweeps.

**Implementation Changes**
- Create `calibrationtest_gui.py` as a separate entrypoint beside the current GUI. It should reuse existing bridge/client/static-solver code from `tools/pennycal.py` rather than duplicating protocol logic.
- Extend the host-side Python tooling with non-firmware-facing helpers for:
  - dynamic sweep execution over existing `GET_STATUS`, `SET_DUTY`, and `SET_ADVANCE`
  - result aggregation for static and dynamic plots
  - timestamped session export
- Keep the static phase identical to current behavior:
  - capture forward/reverse anchors
  - reduce and solve affine + LUT
  - upload and commit the static blob
  - verify CRC/info after reboot
- Run dynamic tests only **after** the static blob has been committed and verified, so all speed/angle telemetry reflects the newly calibrated static mapping.
- Dynamic test procedure:
  - the user enters a comma-separated rpm list in one GUI text box before starting the dynamic stage, for example `1500,2500,3500`
  - parse the entered list into target speeds in ascending order and require at least `2` positive values; if the entered values are invalid, mark dynamic test incomplete and do not run sweeps
  - for each of `{forward, reverse} × {entered speeds}`, drive the motor toward the selected speed by adjusting duty from the host, then sweep advance values `40, 60, 80, 100, 120, 140, 160` electrical degrees
  - at each advance, wait `500 ms`, then collect `500 ms` of `GET_STATUS` samples at `100 Hz`
  - reject sweep points with any fault, any sign flip, or relative ripple above `8%`
  - score each candidate with \(J=\bar{\omega}-0.2\sigma_\omega\)
  - refine around the best coarse point using `±10°` in `2°` steps
  - if two candidates are within `1%` of best score, choose the lower advance
- Fit and report, but do **not** upload or apply, these dynamic summaries:
  - best forward/reverse advances for every entered speed
  - averaged advance for every entered speed
  - simple line fit \(a^*(n_m)=\phi_0 + kn_m\), where \(n_m\) is mechanical speed in rpm and \(k\) is in electrical degrees per rpm
  - equivalent inferred delay \( \tau = k / (6p) \) seconds with \(p=6\)
  - forward/reverse asymmetry across the full entered speed range
- Session export:
  - create one timestamped folder per run
  - save raw static capture, reduced static points, solved static summary, parsed dynamic speed settings, all dynamic sweep samples, best-point summary, fitted line summary, and exported plot images
  - use simple machine-readable formats: CSV for time series and points, JSON for run metadata and fit summaries, PNG for plots

**GUI and Interfaces**
- Add a new experimental GUI only; do not alter the public STM32 command set or calibration blob interface.
- `calibrationtest_gui.py` should have three sections:
  - `Static`
  - `Dynamic`
  - `Session`
- Static section plots:
  - raw XY forward/reverse loci
  - reduced anchors and affine-corrected loci with unit-circle overlay
  - residual vs step with forward, reverse, even, and hysteresis components
- Dynamic section plots:
  - score vs advance, with phase advance on the x-axis and score on the y-axis, using one panel per direction and one colored line per entered speed so all speeds are visible together
  - mean rpm vs advance, with one panel per direction and one colored line per entered speed, so flat or noisy optima can be distinguished from genuine speed gains
  - ripple vs advance, with one panel per direction and one colored line per entered speed, so unstable regions are obvious even when score looks acceptable
  - best advance vs achieved rpm plot with forward points, reverse points, averaged points, and fitted line using all entered speeds
  - asymmetry vs rpm plot showing forward-minus-reverse best advance difference across all entered speeds
  - target rpm vs achieved rpm plot so the user can see whether the host-side duty search actually reached each requested speed before interpreting the sweep result
  - before/after comparison plot is **not** included yet, because firmware behavior is unchanged by the dynamic stage in this iteration
- Session section:
  - display run metadata, static fit error, static sweep delta, entered rpm list, fitted slope, inferred delay, asymmetry summary, and export folder path
  - show a clear badge state: `static committed`, `dynamic complete`, `dynamic incomplete`, or `dynamic suspect`
- User controls:
  - `Start Full Test`
  - `Run Static Only`
  - `Run Dynamic Only` using the currently flashed static calibration
  - `Clear Plots`
  - one text box for comma-separated dynamic target rpm values
  - terminal-style commands retained from the current GUI: `status`, `duty`, `stop`, `advance`
- Defaults:
  - full test runs static commit first, then dynamic test
  - session auto-saves on every completed or failed run
  - dynamic stage starts from `90` electrical degrees unless the user has changed it with the terminal command
  - original `calibration_gui.py` remains the default production tool

**Test Plan**
- Regression-test that the static solve/upload path in the new GUI produces the same static blob and CRC as the existing GUI for identical captured points.
- Unit-test host-side dynamic helpers with synthetic traces:
  - comma-separated speed parsing, validation, and sorting with acceptance of `2+` speeds
  - score computation
  - coarse+refine best-advance selection
  - fitted slope and inferred delay
  - incomplete dynamic run when entered speeds are invalid
- Smoke-test host-only integration against the existing bridge/client:
  - `Run Static Only` commits a valid blob
  - `Run Dynamic Only` uses current calibration and produces exported artifacts without firmware mutation
  - all plots populate from saved session data when replayed or loaded in-memory
- Manual acceptance on hardware:
  - new GUI can perform static commit exactly like today
  - dynamic sweep runs without new firmware support
  - exported folder contains CSV, JSON, and PNG outputs
  - plots make error types visually distinguishable: geometry distortion, hysteresis, flat/ambiguous dynamic optima, and forward/reverse asymmetry

**Assumptions and Defaults**
- No STM32 firmware changes, no ESP32 bridge changes, and no protocol additions are allowed in this step.
- Dynamic results are exploratory and must not affect flash contents or runtime behavior beyond temporary `SET_DUTY` and `SET_ADVANCE` commands during the session.
- The motor is calibrated free-spin and unloaded.
- The current static calibration must succeed before the default dynamic test runs.
- `calibrationtest_gui.py` is intentionally separate so the current production `calibration_gui.py` stays simple and stable.
- Host-side dynamic control is responsible for choosing duty to reach the full user-entered speed list because there is no firmware speed-control command in this step.
- Dynamic plots and fit reporting use all parsed speeds rather than collapsing the data to only low/mid/high summary points.
- The dynamic fit uses mechanical speed in rpm. With advance in electrical degrees and mechanical speed in rpm, \(k\) maps to delay by \(a = 6pn_m\tau\), so \( \tau = k / (6p) \).
