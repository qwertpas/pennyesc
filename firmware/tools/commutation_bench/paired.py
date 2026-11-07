"""Paired torque-per-amp comparison at one held speed.

Slow drift (heating, load) is larger than the effect, so each candidate is
measured between two reference measurements in the same spin, settings
changed live. Reports current relative to the reference (lower is better).
"""
import json
import statistics
import sys
import time

from pny import Esc, PsuLogger, RPM_PER_TURN16_S
from mtpa import hold_speed

REF = dict(lead_us=0, fw_us=0, fw_max_deg=0)


def paired(e, psu, target, candidates, settle_s=1.2, measure_s=1.0):
    rows = []
    e.settings(**REF)
    r = hold_speed(e, psu, target, settle_s=2.0, measure_s=measure_s)
    duty = int(r['duty'])
    ref_prev = r['amps']
    for cand in candidates:
        s = dict(REF); s.update(cand)
        e.settings(**s)
        c = hold_speed(e, psu, target, settle_s=settle_s, measure_s=measure_s, duty0=duty)
        e.settings(**REF)
        ref_next = hold_speed(e, psu, target, settle_s=settle_s, measure_s=measure_s, duty0=duty)['amps']
        ratio = c['amps'] / ((ref_prev + ref_next) / 2)
        rows.append(dict(cand, target=target, amps=c['amps'], ref=(ref_prev + ref_next) / 2, ratio=ratio,
                         duty=c['duty'], rpm=c['rpm']))
        print(f"  {target:6d} rpm {json.dumps(cand):42s} I={c['amps']:.3f} ref={((ref_prev + ref_next) / 2):.3f} "
              f"ratio={ratio:.3f} duty={c['duty']:.0f}", flush=True)
        ref_prev = ref_next
    return rows


if __name__ == '__main__':
    candidates = json.loads(sys.argv[1])
    speeds = json.loads(sys.argv[2])
    tag = sys.argv[3] if len(sys.argv) > 3 else 'paired'
    e = Esc()
    psu = PsuLogger(); psu.start()
    out = []
    try:
        for i, rpm in enumerate(speeds):
            if i:
                e.duty(0); time.sleep(25)
            out += paired(e, psu, rpm, candidates)
    finally:
        e.duty(0); psu.stop(); e.settings()
    json.dump(out, open(tag + '.json', 'w'), indent=1)
