# Commutation bench

Scripts used to find and fix the high-speed commutation collapse and to tune
for torque per amp. They drive an ESC through the stateless bridge and log a
SKY TOPPOWER STP2S supply over Modbus. Run them from a scratch directory: they
write JSON logs to the current directory.

Environment: `PNY_PORT` (bridge, default `/dev/cu.usbmodem101`), `PNY_ADDRESS`
(ESC, default 5), `PSU_PORT` (supply, default `/dev/cu.usbserial-110`).
Close the web GUI first: it holds the bridge port.

| Script | Use |
|---|---|
| `pny.py` | ESC client: status, duty (feedforward with a full clip, like the GUI), commutation settings, comm stats, capture. `PsuLogger` samples supply V/A at ~10 Hz. |
| `psu.py` | STP2S Modbus client. `python3 psu.py` prints the output state. |
| `seq.py`, `stress.py` | Duty sequences with a one-line stability summary: speed per step, slip-guard events, faults, missed/rejected samples. |
| `paired.py`, `mtpa.py` | Torque per amp: hold a speed with a host loop on duty and compare supply current. `paired.py` measures each candidate between two references in one spin, because slow drift (heat, load) is larger than the effect. |
| `flash.sh` | Build `pennyesc_uart`, patch the image address byte, upload through the bridge (one retry). |
| `recover.py` | Discharge with the supply off, start `pnyboot recover`, power on inside its sync window. |
| `escsim.py`, `escsim2.py`, `obs_bias.py` | Models (need numba): motor, six-step drive, TMAG timing and the old/new estimators; the alpha-beta observer's commutation bias under acceleration. |

## Results behind the firmware defaults (1104 motor, 12 V)

- The old firmware collapsed from ~40k rpm to ~3k at duty 400. Its
  alpha-beta observer lagged under acceleration (about +50 electrical degrees
  per 1M rpm/s of deceleration), so braking over-advanced commutation and the
  braking sustained itself. Sample timestamps also jittered by about 16 us.
- Triggered TMAG sampling on a fixed 125 us tick, DMA receive, and the I2C
  interrupt above TIM21: 1 missed sample per ~24k at 44k rpm (was 430-740).
- Observer gains 3/6/12 (alpha 1/8, beta 1/64, gamma 1/4096) were steadier and
  faster at fixed duty than 2/5/10.
- Torque per amp (paired, held speed, drag load): `lead_us` -10 to -20 us is
  best; +20 us costs 1-3%, -60 us 6-9%. Field weakening (+30 us) drew 3.8x the
  current at 35k rpm unloaded and made speed hunt at a few Hz, so it is off by
  default (`fw_us` 0).
- The drag load eases off over its first ~8 s of spinning: warm up before
  measuring.
- Without the bulk capacitor the ESC reset under sustained load (RCC flags
  0x04). That was hardware, not firmware.
