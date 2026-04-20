# UART Update

This flow lets the ESP32-S3 update PennyESC firmware over the existing UART wiring.

## Wiring

| ESP32-S3 | PennyESC |
|----------|----------|
| GPIO 12 (RX) | PA9 (USART2 TX) |
| GPIO 13 (TX) | PA10 (USART2 RX) |
| GPIO 11 | GND |

## One-time setup

1. Flash the ESP32 bridge:

```bash
cd /Users/chris/Kicad/pennyesc/firmware/esp32s3demo
pio run -e bootbridge -t upload --upload-port /dev/cu.usbmodem101
```

2. Flash one UART-updateable STM32 build once over SWD:

```bash
cd /Users/chris/Kicad/pennyesc/firmware/pennyesc_libopencm3
pio run -e pennyesc_uart -t upload
```

## Normal UART upload

From then on, update over the ESP32 bridge:

```bash
cd /Users/chris/Kicad/pennyesc/firmware/pennyesc_libopencm3
pio run -e pennyesc_uart -t uart_upload
pio run -e encoder_uart -t uart_upload
```

`uart_upload` builds the selected env and runs `tools/pnyboot.py upload` with that env's image.

## Recovery upload

If the app is not running, use the reset window:

```bash
cd /Users/chris/Kicad/pennyesc/firmware/pennyesc_libopencm3
pio run -e pennyesc_uart -t uart_recover
```

When prompted, short `NRST` to `GND`, press Enter, then release `NRST` within 2 seconds.

## Add it to a new STM32 env

1. Include the shared build flag and upload script:

```ini
[env:my_app_uart]
build_src_filter = ...
build_flags =
    ${env.build_flags}
    -DPNY_UART_UPDATE=1
extra_scripts = post:extra_uart_update.py
custom_bridge_port = /dev/cu.usbmodem101
custom_esc_address = 0
```

2. In the app:

```c
#include "pennyesc_uart_update.h"
```

3. Start USART2 at the boot baud, open the reset window, then switch to the app baud:

```c
usart2_setup(PNY_UART_UPDATE_BOOT_BAUD);
pennyesc_uart_update_boot_window(&system_millis, ESC_ADDRESS);
usart2_setup(pennyesc_uart_update_app_baud(MY_APP_BAUD));
```

4. Feed every received byte into the hidden update channel:

```c
if (pennyesc_uart_update_feed_byte(
        byte,
        ESC_ADDRESS,
        system_millis,
        fill_update_status,
        prepare_update_boot)) {
    reset_your_app_parser();
    continue;
}
```

5. Call the boot poll in the main loop:

```c
pennyesc_uart_update_poll(system_millis);
```

6. Provide:

- `fill_update_status()` to fill `pny_status_payload_t`
- `prepare_update_boot()` to stop the app safely and return a `PNY_RESULT_*` code
