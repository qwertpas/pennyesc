"""Bench harness: PennyESC over the stateless bridge, plus the STP2S supply."""
import json
import os
import struct
import sys
import threading
import time

HERE = __import__('pathlib').Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
sys.path.insert(0, str(HERE))

from pnybridge import Bridge  # noqa: E402
from pnyproto import (CMD_BRAKE, CMD_DEBUG, CMD_GET_STATUS, CMD_SET_DUTY, CMD_SET_CONTROL,  # noqa: E402
                      CMD_SET_ADVANCE, FrameReader, encode_frame)
from pennycal import unpack_status  # noqa: E402
from psu import PSU  # noqa: E402

RPM_PER_TURN16_S = 60 / 65536
STATS_FMT = '<BB hBBBBhh IIHHHHhhHBBH'
STATS_KEYS = ('subcmd result lead_us flags alpha beta gamma fw_us fw_max_deg samples missed '
              'guard_events comm_faults innov_mean_x16 innov_max accel_krpm_s fw_deg proc_max_us '
              'reset_flags tick_us rejected').split()


class Esc:
    def __init__(self, port=None, address=None):
        port = port or os.environ.get('PNY_PORT', '/dev/cu.usbmodem101')
        address = int(address if address is not None else os.environ.get('PNY_ADDRESS', '5'))
        self.bridge = Bridge(port)
        self.reader = FrameReader(self.bridge)
        self.address = address

    def x(self, cmd, payload=b'', timeout=0.25):
        self.reader.poll()
        self.bridge.write(encode_frame(self.address, cmd, payload))
        return self.reader.read(self.address, cmd, timeout)

    def status(self, retries=2):
        # Read-only, so a dropped reply is simply asked again.
        for attempt in range(retries + 1):
            try:
                return unpack_status(self.x(CMD_GET_STATUS))
            except TimeoutError:
                if attempt == retries:
                    raise

    def duty(self, duty, clip=799):
        # Same as the GUI's duty mode: feedforward only, with a full clip.
        t = time.monotonic()
        payload = struct.pack('<5h', 0, 0, 0, duty, clip)
        try:
            r = unpack_status(self.x(CMD_SET_CONTROL, payload, timeout=1.0))
        except TimeoutError:   # setting the same duty again is harmless
            r = unpack_status(self.x(CMD_SET_CONTROL, payload, timeout=1.0))
        if time.monotonic() - t > 0.1:
            print(f'  slow SET_CONTROL reply: {1000*(time.monotonic()-t):.0f} ms', flush=True)
        return r

    def brake(self):
        return unpack_status(self.x(CMD_BRAKE))

    def advance(self, deg):
        return unpack_status(self.x(CMD_SET_ADVANCE, struct.pack('<h', deg)))

    def stats(self):
        data = self.x(CMD_DEBUG, bytes([7]))
        return dict(zip(STATS_KEYS, struct.unpack(STATS_FMT, data)))

    def settings(self, lead_us=-15, flags=6, alpha=3, beta=6, gamma=12, fw_us=0, fw_max_deg=30):
        data = self.x(CMD_DEBUG, struct.pack('<BhBBBBhh', 4, lead_us, flags, alpha, beta, gamma, fw_us, fw_max_deg))
        return dict(zip(STATS_KEYS, struct.unpack(STATS_FMT, data)))

    def capture_start(self, duty=0, advance=90, duration_ms=0, decimation=1):
        data = self.x(CMD_DEBUG, struct.pack('<BhhHH', 1, duty, advance, duration_ms, decimation))
        return struct.unpack('<BBBB6H', data)

    def capture_status(self, freeze=False):
        data = self.x(CMD_DEBUG, bytes([2, 1]) if freeze else bytes([2]))
        keys = 'subcmd result active done decimation duration_ms elapsed_ms count trigger mct'.split()
        return dict(zip(keys, struct.unpack('<BBBB6H', data)))

    def capture_read(self, count):
        out = []
        while len(out) < count:
            want = min(14, count - len(out))
            data = self.x(CMD_DEBUG, struct.pack('<BHB', 3, len(out), want))
            sub, result, offset, n = struct.unpack('<BBHB', data[:5])
            if result:
                raise RuntimeError(f'capture read result {result}')
            out.extend(struct.iter_unpack('<Hh', data[5:5 + 4 * n]))
        return out


class PsuLogger(threading.Thread):
    def __init__(self):
        super().__init__(daemon=True)
        self.psu = PSU()
        self.rows = []
        self.running = True

    def run(self):
        while self.running:
            try:
                v, a = self.psu.read(0x0010, 2)
                self.rows.append((time.monotonic(), v / 100, a / 1000))
            except Exception:
                pass

    def stop(self):
        self.running = False
        self.join(1)


def run_profile(esc, profile, log_path, psu=None, stats_every=None):
    """profile: list of (seconds, duty). Logs status as fast as the link allows."""
    rows = []
    t0 = time.monotonic()
    try:
        for seconds, duty in profile:
            esc.duty(duty)
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                try:
                    s = esc.status()
                except TimeoutError:
                    rows.append(dict(t=time.monotonic() - t0, timeout=1, cmd=duty))
                    continue
                rows.append(dict(t=time.monotonic() - t0, cmd=duty, mode=s.mode, duty=s.duty,
                                 rpm=s.velocity_turn32_per_s * RPM_PER_TURN16_S, z=s.z,
                                 pos=s.position_turn32, ovr=s.isr_overrun_count, isr=s.isr_us,
                                 isr_max=s.isr_max_us, i2c=s.i2c_us, dt=s.tmag_sample_dt_us,
                                 n=s.tmag_sample_count, to=s.i2c_timeout_count, nack=s.i2c_nack_count,
                                 mct=s.mct_fault_count, faults=s.faults))
    finally:
        for _ in range(3):
            try:
                esc.duty(0)
                break
            except Exception:
                time.sleep(0.05)
    if psu is not None:
        for t, v, a in psu.rows:
            rows.append(dict(t=t - t0, psu_v=v, psu_a=a))
    with open(log_path, 'w') as f:
        json.dump(rows, f)
    return rows


def summarize(rows, step=0.1):
    st = [r for r in rows if 'rpm' in r]
    ps = [r for r in rows if 'psu_a' in r]
    last = -1
    for r in st:
        if r['t'] - last >= step:
            last = r['t']
            near = [p for p in ps if abs(p['t'] - r['t']) < step / 2]
            amp = max((p['psu_a'] for p in near), default=float('nan'))
            volt = max((p['psu_v'] for p in near), default=float('nan'))
            print(f"t={r['t']:6.2f} cmd={r['cmd']:4d} mode={r['mode']} rpm={r['rpm']:8.0f} z={r['z']:6d} "
                  f"ovr={r['ovr']:6d} isrmax={r['isr_max']:4d} dt={r['dt']:4d} I={amp:5.2f}A V={volt:5.2f}")
