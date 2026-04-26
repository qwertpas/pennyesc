# Flashing Commands

Use `python3.11` for all commands.

## ESP32 bridge

Build the bootbridge firmware:

```bash
python3.11 -m platformio run -d /Users/chris/Kicad/pennyesc/firmware/esp32s3demo -e bootbridge
```

Flash the bootbridge firmware over USB:

```bash
python3.11 -m platformio run -d /Users/chris/Kicad/pennyesc/firmware/esp32s3demo -e bootbridge -t upload --upload-port /dev/cu.usbmodem1301
```

If ESP32 upload is unreliable, lower the upload speed:

```bash
python3.11 -m platformio run -c /tmp/bootbridge-upload.ini -d /Users/chris/Kicad/pennyesc/firmware/esp32s3demo -e bootbridge -t upload --upload-port /dev/cu.usbmodem1301
```

## STM32 over STLink

Build the UART-updateable STM32 app:

```bash
python3.11 -m platformio run -d /Users/chris/Kicad/pennyesc/firmware/pennyesc_libopencm3 -e pennyesc_uart
```

Seed the resident UART bootloader plus app over STLink:

```bash
BRIDGE_PORT=/dev/cu.usbmodem1301 ESC_ADDRESS=3 \
python3.11 -m platformio run -d /Users/chris/Kicad/pennyesc/firmware/pennyesc_libopencm3 -e seed -t seed_upload
```

## UART update

Check that the bridge and target ESC are reachable before building:

```bash
BRIDGE_PORT=/dev/cu.usbmodem1301 ESC_ADDRESS=3 \
python3.11 -m platformio run -d /Users/chris/Kicad/pennyesc/firmware/pennyesc_libopencm3 -e pennyesc_uart -t uart_check
```

Normal UART firmware upload:

```bash
BRIDGE_PORT=/dev/cu.usbmodem1301 ESC_ADDRESS=3 \
python3.11 -m platformio run -d /Users/chris/Kicad/pennyesc/firmware/pennyesc_libopencm3 -e pennyesc_uart -t uart_upload
```

Recovery upload if the app is dead or half-flashed:

```bash
BRIDGE_PORT=/dev/cu.usbmodem1301 ESC_ADDRESS=3 \
python3.11 -m platformio run -d /Users/chris/Kicad/pennyesc/firmware/pennyesc_libopencm3 -e pennyesc_uart -t uart_recover
```

## Change ESC address

Flash a device that currently answers on one address so it comes back on a new address:

```bash
BRIDGE_PORT=/dev/cu.usbmodem1301 \
CURRENT_ESC_ADDRESS=3 \
NEW_ESC_ADDRESS=5 \
python3.11 -m platformio run -d /Users/chris/Kicad/pennyesc/firmware/pennyesc_libopencm3 -e readdress -t uart_readdress
```

Verify the new address:

```bash
BRIDGE_PORT=/dev/cu.usbmodem1301 ESC_ADDRESS=5 \
python3.11 -m platformio run -d /Users/chris/Kicad/pennyesc/firmware/pennyesc_libopencm3 -e pennyesc_uart -t uart_check
```

## Notes

- `seed_upload` uses STLink and the UART bridge in one flow.
- `uart_upload` is strict and fails fast if the target is not connected.
- `uart_recover` is the slow/manual recovery path.
- Only run one serial or STLink flashing process at a time.
