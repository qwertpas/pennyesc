"""Torque per amp at fixed speed.

Unloaded, the load at a fixed speed is drag, which does not depend on the
commutation angle. So the setting that holds a speed on the least supply
current gives the most torque per amp there. A host-side integral loop on
duty holds the speed; the STP2S supplies current at about 10 Hz.
"""
import json
import statistics
import sys
import time

from pny import Esc, PsuLogger, RPM_PER_TURN16_S


def hold_speed(e, psu, rpm_target, settle_s=1.6, measure_s=1.2, duty0=None):
    duty = duty0 if duty0 is not None else max(60, int(rpm_target / 105))
    e.duty(duty)
    t0 = time.monotonic()
    speeds, duties = [], []
    acc = float(duty)
    while time.monotonic() - t0 < settle_s + measure_s:
        s = e.status()
        rpm = s.velocity_turn32_per_s * RPM_PER_TURN16_S
        if s.mode != 1:
            raise RuntimeError('ESC stopped')
        acc += 0.002 * (rpm_target - rpm)
        acc = min(max(acc, 30.0), 700.0)
        if int(round(acc)) != duty:
            duty = int(round(acc))
            e.duty(duty)
        if time.monotonic() - t0 >= settle_s:
            speeds.append(rpm)
            duties.append(duty)
    t1 = time.monotonic()
    amps = [a for t, _, a in psu.rows if t1 - measure_s <= t <= t1]
    return dict(rpm=statistics.median(speeds), duty=statistics.mean(duties),
                amps=statistics.median(amps) if amps else float('nan'), n_amps=len(amps))


def sweep(e, settings_list, speeds, tag, cool_s=25.0):
    psu = PsuLogger()
    psu.start()
    results = []
    try:
        for i, settings in enumerate(settings_list):
            adv = settings.pop('advance', None)
            if adv is not None:
                e.advance(adv)
            e.settings(**settings)
            duty = None
            for rpm in speeds:
                r = hold_speed(e, psu, rpm, duty0=duty)
                duty = int(r['duty'])
                r.update(settings, advance=adv, target=rpm)
                results.append(r)
                print(f"  adv={adv} fw_us={settings.get('fw_us')} cap={settings.get('fw_max_deg')} "
                      f"target={rpm:6d} rpm={r['rpm']:7.0f} duty={r['duty']:6.1f} I={r['amps']:.3f} A (n={r['n_amps']})",
                      flush=True)
            e.duty(0)
            if i + 1 < len(settings_list):
                time.sleep(cool_s)
    finally:
        e.duty(0)
        psu.stop()
        e.advance(90)
        e.settings()
    json.dump(results, open(tag + '.json', 'w'), indent=1)
    return results


if __name__ == '__main__':
    e = Esc()
    plan = json.loads(sys.argv[1])
    speeds = json.loads(sys.argv[2])
    sweep(e, plan, speeds, sys.argv[3] if len(sys.argv) > 3 else 'mtpa')
