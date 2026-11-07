import subprocess, sys, time
from psu import PSU
import os, pathlib
H = os.environ.get('TMPDIR', '/tmp')
TOOLS = str(pathlib.Path(__file__).resolve().parent.parent)
proc = subprocess.Popen([sys.executable, 'pnyboot.py', 'recover', '--port', os.environ.get('PNY_PORT', '/dev/cu.usbmodem101'), '--address', os.environ.get('PNY_ADDRESS', '5'),
                         '--image', H + '/fw_addr.bin'], cwd=TOOLS,
                        stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
proc.stdin.write('\n'); proc.stdin.flush()
time.sleep(1.0)
p = PSU()
p.write(0x0001, 0)
t = time.monotonic()
while time.monotonic() - t < 15:
    v = p.snapshot()['v']
    if v < 0.5:
        break
    time.sleep(0.2)
print(f'off: rail {v:.2f} V after {time.monotonic()-t:.1f} s', flush=True)
time.sleep(0.3)
p.write(0x0001, 1)
out, _ = proc.communicate(timeout=60)
print(out[-600:])
print(p.snapshot())
