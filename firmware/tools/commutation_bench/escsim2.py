"""Model of the proposed firmware: trigger-mode TMAG sampling on a fixed tick,
fixed-point alpha-beta-gamma observer (shift gains), capped field-weakening
advance, slip guard. Same plant as escsim.py."""
import math
import numpy as np
from numba import njit

TWO_PI = 2 * math.pi


@njit(cache=True)
def wrap(x):
    return (x + 0.5) % 1.0 - 0.5


@njit(cache=True)
def simulate(duty, vbus, tick_us, sensor_mode, stamp_err_us, a_sh, b_sh, g_sh,
             lead_us, fw_us, fw_max_deg, adv_deg, guard_acc_rpm_s, guard_ms,
             xy_skew_corr, dur_s, R, L, lam, J, ilim, load_c, load_b, eps_sensor,
             seed, kick_t, kick_rpm, read_us, legacy_age_us):
    np.random.seed(seed)
    pp = 6
    dt = 1e-6
    n = int(dur_s / dt)
    th = 0.0
    w = 0.0
    ia = 0.0
    ib = 0.0
    H = 1024
    hist = np.zeros(H)
    hidx = 0
    # continuous-mode sensor state (sensor_mode 0)
    Tc = 75.0 * (1.0 + eps_sensor)
    next_conv = 3.0
    conv_k = 0
    stored_ang = 0.0
    stored_cnt = -1
    read_active = False
    read_start = -1e9
    read_end = -1e9
    read_ang = 0.0
    read_cnt = -2
    last_cnt = -3
    # trigger-mode state (sensor_mode 1): conversion started at trig_t, read next tick
    trig_t = -1e9
    pend_valid = False
    # observer (units: revs, revs/tick, revs/tick^2), state at grid time pos_t
    T = tick_us
    have_obs = False
    pos = 0.0
    vel = 0.0
    acc = 0.0
    pos_t = 0.0
    last_tick_index = 0
    anchor = 0.0
    anchor_t = 0.0
    rate = 0.0
    guard_until = -1.0
    guard_events = 0
    vmag = duty * vbus / math.sqrt(3.0)
    ax_c = 60.0
    ay_c = 97.0
    next_tick = 50.0
    tick_index = 0
    m = int(dur_s * 1000)
    log_rpm = np.zeros(m)
    log_est = np.zeros(m)
    log_adv = np.zeros(m)
    log_iq = np.zeros(m)
    log_acc = np.zeros(m)
    adv_acc = 0.0
    adv_n = 0
    iq_acc = 0.0
    li = 0
    kicked = False
    meas = 0.0
    for s in range(n):
        t_us = s * 1.0
        if sensor_mode == 0:
            if t_us >= next_conv + 100.0:
                tx = next_conv + 12.5
                ty = next_conv + 37.5
                thx = hist[(hidx - 1 - int(t_us - tx)) % H]
                thy = hist[(hidx - 1 - int(t_us - ty)) % H]
                busy = read_active and t_us >= read_start and t_us <= read_end
                if not busy:
                    xq = round(ax_c * math.cos(TWO_PI * thx))
                    yq = round(ay_c * math.sin(TWO_PI * thy))
                    stored_ang = math.atan2(yq / ay_c, xq / ax_c) / TWO_PI
                    stored_cnt = conv_k
                conv_k += 1
                next_conv += Tc
            if read_active and t_us >= read_end:
                read_active = False
        if t_us >= next_tick:
            next_tick += T
            have = False
            stamp = 0.0
            if sensor_mode == 1:
                # read result of conversion triggered last tick, trigger the next
                if pend_valid:
                    tx = trig_t + 20.0 + 12.5
                    ty = trig_t + 20.0 + 37.5
                    thx = hist[(hidx - 1 - int(t_us - tx)) % H]
                    thy = hist[(hidx - 1 - int(t_us - ty)) % H]
                    xq = round(ax_c * math.cos(TWO_PI * thx))
                    yq = round(ay_c * math.sin(TWO_PI * thy))
                    if xy_skew_corr:
                        # y was sampled 25 us after x: rotate it back using the estimate
                        dth = vel / T * 25.0 * TWO_PI
                        yq = yq - dth * ay_c / ax_c * xq
                    meas = math.atan2(yq / ay_c, xq / ax_c) / TWO_PI
                    have = True
                    stamp = trig_t + 20.0 + 25.0 + stamp_err_us
                trig_t = t_us
                pend_valid = True
            else:
                if (not read_active) and read_cnt != last_cnt and read_cnt >= 0:
                    have = True
                    last_cnt = read_cnt
                    meas = read_ang
                    stamp = read_start - legacy_age_us
                if not read_active:
                    read_active = True
                    read_start = t_us
                    read_end = t_us + read_us
                    read_ang = stored_ang
                    read_cnt = stored_cnt
            tick_index += 1
            if have:
                if not have_obs:
                    pos = meas
                    have_obs = True
                    k = 1
                else:
                    k = int(round((stamp - pos_t) / T))
                    if k < 1:
                        k = 1
                    if k > 2:
                        k = 2
                    pred = pos + vel * k + 0.5 * acc * k * k
                    pvel = vel + acc * k
                    e = wrap(meas - pred)
                    pos = pred + e * 2.0 ** (-a_sh)
                    vel = pvel + e * 2.0 ** (-b_sh) / k
                    acc = acc + e * 2.0 ** (-g_sh) / (k * k)
                pos_t = stamp
                # advance law
                acc_rpm_s = acc / (T * 1e-6) ** 2 * 60.0
                ctl = t_us + 10.0
                dirn = 1.0
                if guard_acc_rpm_s > 0 and acc_rpm_s * dirn < -guard_acc_rpm_s and abs(vel) / (T * 1e-6) * 60 > 5000:
                    if ctl > guard_until:
                        guard_events += 1
                    guard_until = ctl + guard_ms * 1000.0
                fw = 0.0
                if ctl > guard_until:
                    fw = abs(vel) / T * fw_us * pp
                    if fw > fw_max_deg / 360.0:
                        fw = fw_max_deg / 360.0
                h = (ctl - pos_t + lead_us) / T
                anchor = (pos + vel * h + 0.5 * acc * h * h) * pp + fw
                anchor_t = ctl
                rate = (vel + acc * h) / T * pp
        if have_obs:
            ph = anchor + rate * (t_us - anchor_t) + adv_deg / 360.0
        else:
            ph = th * pp + adv_deg / 360.0
        sector = math.floor(ph * 6.0)
        gam = (sector / 6.0 + 1.0 / 12.0) * TWO_PI
        va = vmag * math.cos(gam)
        vb = vmag * math.sin(gam)
        proj = ia * math.cos(gam) + ib * math.sin(gam)
        if proj > ilim:
            va = 0.0
            vb = 0.0
        the = TWO_PI * th * pp
        we = TWO_PI * w * pp
        ea = -lam * we * math.sin(the)
        eb = lam * we * math.cos(the)
        ia += (va - R * ia - ea) / L * dt
        ib += (vb - R * ib - eb) / L * dt
        iq = -ia * math.sin(the) + ib * math.cos(the)
        torque = 1.5 * pp * lam * iq
        wr = TWO_PI * w
        tl = J * (load_c + load_b * abs(wr))
        if wr < 0:
            tl = -tl
        wr += (torque - tl) / J * dt
        if (not kicked) and kick_t > 0 and t_us * 1e-6 >= kick_t:
            wr -= TWO_PI * kick_rpm / 60.0
            kicked = True
        w = wr / TWO_PI
        th += w * dt
        hist[hidx % H] = th
        hidx += 1
        adv_acc += wrap(gam / TWO_PI - the / TWO_PI)
        adv_n += 1
        iq_acc += iq
        if s % 1000 == 999:
            log_rpm[li] = w * 60.0
            log_est[li] = vel / (T * 1e-6) * 60.0
            log_adv[li] = adv_acc / adv_n * 360.0
            log_iq[li] = iq_acc / adv_n
            log_acc[li] = acc / (T * 1e-6) ** 2 * 60.0
            adv_acc = 0.0
            adv_n = 0
            iq_acc = 0.0
            li += 1
    return log_rpm, log_est, log_adv, log_iq, log_acc, guard_events


DEFAULTS = dict(duty=0.5, vbus=12.0, tick_us=125.0, sensor_mode=1, stamp_err_us=0.0,
                a_sh=2, b_sh=5, g_sh=10, lead_us=0.0, fw_us=20.0, fw_max_deg=30.0,
                adv_deg=90.0, guard_acc_rpm_s=0.0, guard_ms=20.0, xy_skew_corr=False,
                dur_s=1.5, R=0.2, L=8e-6, lam=1.53e-4, J=9.4e-8, ilim=2.7,
                load_c=1508.0, load_b=0.83, eps_sensor=0.001, seed=1, kick_t=0.0,
                kick_rpm=0.0, read_us=45.0, legacy_age_us=100.0)


def run(**kw):
    p = dict(DEFAULTS)
    p.update(kw)
    return simulate(*[p[k] for k in DEFAULTS])
