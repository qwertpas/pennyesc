"""Repeated duty sequence; summarize stability per run."""
import json, sys, time
from pny import *

def run_seq(e, seq, tag, settings=None, psu=None):
    if settings:
        r = e.settings(**settings)
        assert r['result'] == 0, r
    s0 = e.stats()
    rows = []
    t0 = time.monotonic()
    armed = False
    try:
        for seconds, duty in seq:
            e.duty(duty)
            if not armed and duty:
                e.capture_start(duty=0, duration_ms=0, decimation=1); armed = True
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                try:
                    s = e.status()
                except TimeoutError:
                    continue
                rows.append((time.monotonic() - t0, duty, s.mode, s.velocity_turn32_per_s * RPM_PER_TURN16_S, s.z))
    finally:
        e.duty(0)
    cs = e.capture_status(freeze=True)
    s1 = e.stats()
    recs = e.capture_read(cs['count'])
    json.dump(dict(rows=rows, cap=cs, recs=recs, s0=s0, s1=s1, settings=settings), open(tag + '.json', 'w'))
    # per-segment steady speed: median over last half of each segment
    out = []
    t = 0
    for seconds, duty in seq:
        seg = [r[3] for r in rows if t + seconds / 2 <= r[0] < t + seconds and r[2] == 1]
        if duty:
            out.append(f"d{duty}:{(sorted(seg)[len(seg)//2] if seg else 0):6.0f}/min{(min(seg) if seg else 0):6.0f}")
        t += seconds
    died = any(r[2] == 0 for r in rows if r[1] != 0)
    print(f"{tag}: {' '.join(out)} died={died} guard={s1['guard_events']-s0['guard_events']} "
          f"faults={s1['comm_faults']-s0['comm_faults']} missed={s1['missed']-s0['missed']}/{s1['samples']-s0['samples']} "
          f"innov_mean={s1['innov_mean_x16']/16:.0f} innov_max={s1['innov_max']} rej={s1['rejected']-s0['rejected']} proc_max={s1['proc_max_us']} fw={s1['fw_deg']}",
          flush=True)
    return rows

SEQ = [(0.8, 250), (1.5, 400), (0.6, 0)]
if __name__ == '__main__':
    e = Esc()
    settings = json.loads(sys.argv[2]) if len(sys.argv) > 2 else None
    for i in range(int(sys.argv[1]) if len(sys.argv) > 1 else 3):
        run_seq(e, SEQ, f'seq_{i}', settings if i == 0 else None)
