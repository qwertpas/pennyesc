#!/bin/sh
# Build, patch the image address byte, flash through the bridge.
set -e
H="$(cd "$(dirname "$0")" && pwd)"
OUT="${TMPDIR:-/tmp}"
PORT="${PNY_PORT:-/dev/cu.usbmodem101}"
ADDRESS="${PNY_ADDRESS:-5}"
cd "$H/../../pennyesc_libopencm3"
if ! pio run -e pennyesc_uart > "$OUT/pio_build.log" 2>&1; then grep -E "error|overflowed" "$OUT/pio_build.log"; echo "BUILD FAILED, not flashing"; exit 1; fi
grep -E "Flash:" "$OUT/pio_build.log"
python3 - "$OUT" "$ADDRESS" <<'PY'
import sys
d=bytearray(open('.pio/build/pennyesc_uart/firmware.bin','rb').read())
i=d.find(b'PNYADDR'); assert i>=0 and d.count(b'PNYADDR')==1
d[i+7]=int(sys.argv[2]); open(sys.argv[1]+'/fw_addr.bin','wb').write(d)
PY
cd ../tools
for attempt in 1 2; do
  if python3 pnyboot.py upload --port "$PORT" --address "$ADDRESS" --image "$OUT/fw_addr.bin" > "$OUT/upload.log" 2>&1; then
    tail -1 "$OUT/upload.log"; exit 0
  fi
  echo "upload attempt $attempt failed: $(tail -1 "$OUT/upload.log")"; sleep 7
done
exit 1
