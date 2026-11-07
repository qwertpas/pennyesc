"""Behavioural model of PennyESC commutation at high speed.

Plant: 6-pole-pair SPM motor in the stationary alpha/beta frame, averaged PWM,
120-degree six-step voltage vector selected from emulated Hall sector,
MCT8316Z-style cycle-by-cycle limit on motoring current only.
Sensor: TMAG5273 continuous XY at 75 us/conversion, X and Y sampled 25 us
apart, 8-bit MSB-only reads, results discarded when a conversion completes
while the MCU is reading (datasheet table 6-1, INT_MODE 000b).
Firmware: SysTick 100 us, observer_ab_update replica (alpha 1/8, beta 1/128),
stamp = I2C read start, scheduler extrapolates phase with observer velocity.

Parameters are plausible 1104-class values, not measured ones.
"""
import math
import numpy as np
from numba import njit

TWO_PI = 2 * math.pi


@njit(cache=True)
def wrap(x):
    return (x + 0.5) % 1.0 - 0.5


@njit(cache=True)
def simulate(duty, vbus, lead_us, adv_deg, obs_mode, sensor_sync, eps_sensor,
             tick_us, read_us, isr_pre_us, dur_s, R, L, lam, J, ilim,
             load_c, load_b, alpha_div, beta_div, gamma_div, w0_rpm, seed,
             kick_t, kick_rpm):
    np.random.seed(seed)
    pp = 6
    dt = 1e-6
    n = int(dur_s / dt)
    # rotor
    th = 0.0                       # mechanical revs
    w = w0_rpm / 60.0              # rev/s
    ia = 0.0
    ib = 0.0
    # position history ring (mech revs) per microsecond
    H = 512
    hist = np.zeros(H)
    for k in range(H):
        hist[k] = th - w * (H - k) * dt
    hidx = 0
    # sensor state
    Tc = 75.0 * (1.0 + eps_sensor)
    next_conv = 3.0
    conv_k = 0
    stored_ang = 0.0
    stored_cnt = -1
    stored_tsamp = 0.0
    # pending X/Y sample instants for in-flight conversions (complete 100 us after start)
    # MCU state
    next_tick = 50.0
    read_active = False
    read_start = -1e9
    read_end = -1e9
    read_ang = 0.0
    read_cnt = -2
    read_tsamp = 0.0
    last_cnt = -3
    have_obs = False
    pos = 0.0
    vel = w0_rpm / 60.0
    acc = 0.0
    pos_tick = 0.0
    anchor = 0.0
    anchor_t = 0.0
    rate = 0.0
    adv = adv_deg / 360.0
    vmag = duty * vbus / math.sqrt(3.0)
    ax_c = 60.0
    ay_c = 97.0
    # logging every 1 ms
    m = int(dur_s * 1000)
    log_t = np.zeros(m)
    log_rpm = np.zeros(m)
    log_est = np.zeros(m)
    log_adv = np.zeros(m)
    log_iq = np.zeros(m)
    log_id = np.zeros(m)
    adv_acc = 0.0
    adv_n = 0
    iq_acc = 0.0
    id_acc = 0.0
    li = 0
    kicked = False
    meas = 0.0
    stamp = 0.0
    for s in range(n):
        t_us = s * 1.0
        # ---- sensor conversions (start every Tc; X at +12.5, Y at +37.5, done at +100)
        if t_us >= next_conv + 100.0:
            tx = next_conv + 12.5
            ty = next_conv + 37.5
            # look up history
            bx = hidx - 1 - int(t_us - tx)
            by = hidx - 1 - int(t_us - ty)
            thx = hist[bx % H]
            thy = hist[by % H]
            busy = read_active and t_us >= read_start and t_us <= read_end
            if not busy:
                xq = round(ax_c * math.cos(TWO_PI * thx))
                yq = round(ay_c * math.sin(TWO_PI * thy))
                stored_ang = math.atan2(yq / ay_c, xq / ax_c) / TWO_PI
                stored_cnt = conv_k
                stored_tsamp = 0.5 * (tx + ty)
            conv_k += 1
            next_conv += Tc
        # ---- I2C read completes
        if read_active and t_us >= read_end:
            read_active = False
        # ---- SysTick ISR
        if t_us >= next_tick:
            next_tick += tick_us
            have = False
            if (not read_active) and read_cnt != last_cnt and read_cnt >= 0:
                have = True
                last_cnt = read_cnt
                meas = read_ang
                stamp = read_start if not sensor_sync else read_tsamp
            if have:
                if not have_obs:
                    pos = meas
                    pos_tick = stamp
                    have_obs = True
                else:
                    dts = (stamp - pos_tick) * 1e-6
                    if obs_mode == 2:
                        pos = meas
                        # ideal velocity
                        vel = w
                    elif dts > 0:
                        pred = pos + vel * dts + 0.5 * acc * dts * dts
                        e = wrap(meas - pred)
                        pos = pred + e / alpha_div
                        vel = vel + acc * dts + e / beta_div / dts
                        if obs_mode == 1:
                            acc = acc + e / gamma_div / (dts * dts)
                    pos_tick = stamp
            ctl = t_us + isr_pre_us
            if not read_active:
                read_active = True
                read_start = ctl
                read_end = ctl + read_us
                read_ang = stored_ang
                read_cnt = stored_cnt
                read_tsamp = stored_tsamp
            if have:
                # comm_scheduler_schedule_sector_event at control tick
                h = (ctl - pos_tick + lead_us) * 1e-6
                anchor = (pos + vel * h + (0.5 * acc * h * h if obs_mode == 1 else 0.0)) * pp
                anchor_t = ctl
                rate = vel * pp
        # ---- commutation (absolute phase extrapolation)
        if t_us >= anchor_t:
            ph = anchor + rate * (t_us - anchor_t) * 1e-6 + adv
        else:
            ph = anchor + adv
        sector = math.floor(ph * 6.0)
        gam = (sector / 6.0 + 1.0 / 12.0) * TWO_PI
        va = vmag * math.cos(gam)
        vb = vmag * math.sin(gam)
        # cycle-by-cycle limit on motoring current (current along drive vector)
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
        idd = ia * math.cos(the) + ib * math.sin(the)
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
        # effective advance of applied vector vs rotor q-axis-relative d-axis
        adv_acc += wrap(gam / TWO_PI - (the / TWO_PI))
        adv_n += 1
        iq_acc += iq
        id_acc += idd
        if s % 1000 == 999:
            log_t[li] = (s + 1) * dt
            log_rpm[li] = w * 60.0
            log_est[li] = vel * 60.0
            log_adv[li] = adv_acc / adv_n * 360.0
            log_iq[li] = iq_acc / adv_n
            log_id[li] = id_acc / adv_n
            adv_acc = 0.0
            adv_n = 0
            iq_acc = 0.0
            id_acc = 0.0
            li += 1
    return log_t, log_rpm, log_est, log_adv, log_iq, log_id


DEFAULTS = dict(duty=0.5, vbus=12.0, lead_us=120.0, adv_deg=90.0, obs_mode=0,
                sensor_sync=False, eps_sensor=0.003, tick_us=100.0, read_us=45.0,
                isr_pre_us=15.0, dur_s=1.0, R=0.2, L=8e-6, lam=1.53e-4, J=9.4e-8,
                ilim=2.7, load_c=1508.0, load_b=0.83, alpha_div=8.0, beta_div=128.0,
                gamma_div=4096.0, w0_rpm=0.0, seed=1, kick_t=0.0, kick_rpm=0.0)


def run(**kw):
    p = dict(DEFAULTS)
    p.update(kw)
    return simulate(*[p[k] for k in DEFAULTS])
