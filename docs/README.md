# penny-gui

[![penny-gui calibration with recorded capture data](screenshot.png)](https://qwertpas.github.io/pennyesc/)

[Open penny-gui](https://qwertpas.github.io/pennyesc/)

Browser control, calibration, live position and velocity plots, and firmware flashing for PennyESC.

Open in your browser. Choose **USB** or **BLE**, connect, and select the ESC address. BLE supports controls, PWM, and 200 samples/s telemetry. USB takes priority and is required for firmware flashing, pin changes, and calibration. Use the controls or commands to set duty, position, and velocity. Duty starts at 100; click **Set** or enable **Key control** and hold `−` or `+` to drive; release to stop.

Enable **PWM output (1-9)** beside **Key control** to start 200 Hz servo PWM and use keys `1–9` for 1000–2000 µs in 125 µs steps from any view. Use **PWM** to change the pulse or pin (default GPIO 11). The output stays enabled across view and window switches. Bridge defaults: ESP32S3 SuperMini, RX 12, TX 13, GND.
