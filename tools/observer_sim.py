from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable

import numpy as np


POLE_PAIRS = 6
SECTORS_PER_MECH_TURN = 6 * POLE_PAIRS

LEGACY_CONTROL_HZ = 5000.0
LEGACY_SENSOR_HZ = 5700.0
CURRENT_CONTROL_HZ = 10000.0
CURRENT_SENSOR_HZ = FAST_XY_SENSOR_HZ = 13300.0
READ_DELAY_S = 25e-6

# Runtime angle path is a 256-entry LUT with no interpolation.
ANGLE_LUT_BINS = 256


def wrap_turn(value: float) -> float:
    return value - math.floor(value)


def wrap_turn_signed(value: float) -> float:
    return ((value + 0.5) % 1.0) - 0.5


def mech_turns_to_elec_deg(turns: np.ndarray | float) -> np.ndarray | float:
    return np.asarray(turns) * 360.0 * POLE_PAIRS


def sector_index(turns: np.ndarray) -> np.ndarray:
    electrical = np.mod(turns * POLE_PAIRS, 1.0)
    return np.floor(electrical * 6.0).astype(np.int32)


def quantize_turn(turns: float, bins: int = ANGLE_LUT_BINS) -> float:
    return round(wrap_turn(turns) * bins) / bins


@dataclass(frozen=True)
class Scenario:
    rpm: float
    noise_deg: float
    ripple_frac: float
    ripple_hz: float
    static_bias_deg: float = 0.0
    duration_s: float = 0.25
    control_hz: float = CURRENT_CONTROL_HZ
    sensor_hz: float = CURRENT_SENSOR_HZ
    read_delay_s: float = READ_DELAY_S
    triggered: bool = True

    @property
    def mech_turns_per_s(self) -> float:
        return self.rpm / 60.0

    @property
    def noise_turns(self) -> float:
        return self.noise_deg / 360.0

    @property
    def static_bias_turns(self) -> float:
        return self.static_bias_deg / 360.0

    @property
    def control_dt(self) -> float:
        return 1.0 / self.control_hz

    @property
    def sensor_dt(self) -> float:
        return 1.0 / self.sensor_hz

    @property
    def sensor_group_delay_s(self) -> float:
        return 0.5 * self.sensor_dt

    @property
    def triggered_age_s(self) -> float:
        return self.read_delay_s + self.sensor_group_delay_s + 0.5 * self.control_dt


@dataclass(frozen=True)
class Metrics:
    rms_elec_deg: float
    p95_elec_deg: float
    max_elec_deg: float
    wrong_sector_rate: float
    missed_sector_rate: float
    spurious_transitions_per_s: float


class BaseObserver:
    name = "base"

    def advance(self, dt: float) -> None:
        raise NotImplementedError

    def update(self, z_wrapped: float, age_s: float, timestamp_s: float) -> None:
        raise NotImplementedError

    def angle_turns(self) -> float:
        raise NotImplementedError


class RawObserver(BaseObserver):
    name = "raw"

    def __init__(self) -> None:
        self.ready = False
        self.latest = 0.0

    def advance(self, dt: float) -> None:
        return

    def update(self, z_wrapped: float, age_s: float, timestamp_s: float) -> None:
        if not self.ready:
            self.latest = z_wrapped
            self.ready = True
            return
        self.latest += wrap_turn_signed(z_wrapped - self.latest)

    def angle_turns(self) -> float:
        return self.latest


class VelocityLeadObserver(BaseObserver):
    name = "velocity_lead"

    def __init__(self, window_s: float = 1e-3) -> None:
        self.window_s = window_s
        self.ready = False
        self.latest = 0.0
        self.latest_time = 0.0
        self.history: list[tuple[float, float]] = []
        self.omega_turn_s = 0.0
        self.current_age_s = 0.0

    def advance(self, dt: float) -> None:
        self.current_age_s += dt

    def update(self, z_wrapped: float, age_s: float, timestamp_s: float) -> None:
        if not self.ready:
            self.latest = z_wrapped
            self.latest_time = timestamp_s
            self.history = [(timestamp_s, z_wrapped)]
            self.ready = True
            self.current_age_s = age_s
            return

        unwrapped = self.latest + wrap_turn_signed(z_wrapped - self.latest)
        self.latest = unwrapped
        self.latest_time = timestamp_s
        self.current_age_s = age_s
        self.history.append((timestamp_s, unwrapped))

        while len(self.history) > 2 and (timestamp_s - self.history[0][0]) > self.window_s:
            self.history.pop(0)

        oldest_t, oldest_theta = self.history[0]
        dt = timestamp_s - oldest_t
        if dt > 0.0:
            self.omega_turn_s = (unwrapped - oldest_theta) / dt

    def angle_turns(self) -> float:
        return self.latest + self.omega_turn_s * self.current_age_s


class DelayAlphaBeta(BaseObserver):
    name = "alpha_beta"

    def __init__(self, dt_s: float, alpha: float, beta: float, age_s: float) -> None:
        self.dt_s = dt_s
        self.alpha = alpha
        self.beta = beta
        self.age_s = age_s
        self.ready = False
        self.x = 0.0
        self.v = 0.0

    def advance(self, dt: float) -> None:
        if self.ready:
            self.x += self.v * dt

    def update(self, z_wrapped: float, age_s: float, timestamp_s: float) -> None:
        if not self.ready:
            self.x = z_wrapped
            self.ready = True
            return

        z_pred = self.x - self.v * self.age_s
        err = wrap_turn_signed(z_wrapped - z_pred)
        self.x += self.alpha * err
        self.v += (self.beta / self.dt_s) * err

    def angle_turns(self) -> float:
        return self.x


class DelayKalmanCV(BaseObserver):
    name = "kalman_cv"

    def __init__(self, sigma_meas_turns: float, sigma_accel_turn_s2: float) -> None:
        self.ready = False
        self.x = np.zeros(2, dtype=np.float64)
        self.p = np.diag([0.25, 100.0]).astype(np.float64)
        self.r = float(sigma_meas_turns**2)
        self.sigma_accel = float(sigma_accel_turn_s2)

    def advance(self, dt: float) -> None:
        if not self.ready:
            return
        f = np.array([[1.0, dt], [0.0, 1.0]], dtype=np.float64)
        q = (self.sigma_accel**2) * np.array(
            [[0.25 * dt**4, 0.5 * dt**3], [0.5 * dt**3, dt**2]],
            dtype=np.float64,
        )
        self.x = f @ self.x
        self.p = f @ self.p @ f.T + q

    def update(self, z_wrapped: float, age_s: float, timestamp_s: float) -> None:
        if not self.ready:
            self.x[0] = z_wrapped
            self.ready = True
            return

        h = np.array([[1.0, -age_s]], dtype=np.float64)
        z_pred = float((h @ self.x)[0])
        z = z_pred + wrap_turn_signed(z_wrapped - z_pred)
        y = z - z_pred

        s = float((h @ self.p @ h.T)[0, 0] + self.r)
        k = (self.p @ h.T)[:, 0] / s
        i = np.eye(2, dtype=np.float64)
        kh = np.outer(k, h[0])

        self.x = self.x + k * y
        self.p = (i - kh) @ self.p @ (i - kh).T + np.outer(k, k) * self.r

    def angle_turns(self) -> float:
        return float(self.x[0])


class DelayKalmanCA(BaseObserver):
    name = "kalman_ca"

    def __init__(self, sigma_meas_turns: float, sigma_jerk_turn_s3: float) -> None:
        self.ready = False
        self.x = np.zeros(3, dtype=np.float64)
        self.p = np.diag([0.25, 100.0, 20000.0]).astype(np.float64)
        self.r = float(sigma_meas_turns**2)
        self.sigma_jerk = float(sigma_jerk_turn_s3)

    def advance(self, dt: float) -> None:
        if not self.ready:
            return
        f = np.array(
            [
                [1.0, dt, 0.5 * dt**2],
                [0.0, 1.0, dt],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )
        q = (self.sigma_jerk**2) * np.array(
            [
                [dt**5 / 20.0, dt**4 / 8.0, dt**3 / 6.0],
                [dt**4 / 8.0, dt**3 / 3.0, dt**2 / 2.0],
                [dt**3 / 6.0, dt**2 / 2.0, dt],
            ],
            dtype=np.float64,
        )
        self.x = f @ self.x
        self.p = f @ self.p @ f.T + q

    def update(self, z_wrapped: float, age_s: float, timestamp_s: float) -> None:
        if not self.ready:
            self.x[0] = z_wrapped
            self.ready = True
            return

        h = np.array([[1.0, -age_s, 0.5 * age_s**2]], dtype=np.float64)
        z_pred = float((h @ self.x)[0])
        z = z_pred + wrap_turn_signed(z_wrapped - z_pred)
        y = z - z_pred

        s = float((h @ self.p @ h.T)[0, 0] + self.r)
        k = (self.p @ h.T)[:, 0] / s
        i = np.eye(3, dtype=np.float64)
        kh = np.outer(k, h[0])

        self.x = self.x + k * y
        self.p = (i - kh) @ self.p @ (i - kh).T + np.outer(k, k) * self.r

    def angle_turns(self) -> float:
        return float(self.x[0])


class OracleLeadObserver(BaseObserver):
    name = "oracle"

    def __init__(self, omega_fn) -> None:
        self.ready = False
        self.latest = 0.0
        self.control_time = 0.0
        self.current_age_s = 0.0
        self.omega_fn = omega_fn

    def advance(self, dt: float) -> None:
        self.control_time += dt
        self.current_age_s += dt

    def update(self, z_wrapped: float, age_s: float, timestamp_s: float) -> None:
        if not self.ready:
            self.latest = z_wrapped
            self.ready = True
        else:
            self.latest += wrap_turn_signed(z_wrapped - self.latest)
        self.current_age_s = age_s

    def angle_turns(self) -> float:
        return self.latest + self.omega_fn(self.control_time) * self.current_age_s


def make_speed_profile(turns_per_s: float, ripple_frac: float, ripple_hz: float):
    tau = turns_per_s * ripple_frac / (2.0 * math.pi * ripple_hz) if ripple_hz > 0.0 else 0.0

    def angle_fn(t: float) -> float:
        if ripple_hz <= 0.0:
            return turns_per_s * t
        return turns_per_s * t + tau * (1.0 - math.cos(2.0 * math.pi * ripple_hz * t))

    def omega_fn(t: float) -> float:
        if ripple_hz <= 0.0:
            return turns_per_s
        return turns_per_s * (1.0 + ripple_frac * math.sin(2.0 * math.pi * ripple_hz * t))

    return angle_fn, omega_fn


def measurement_variance_turns(noise_turns: float) -> float:
    quant = (1.0 / ANGLE_LUT_BINS) / math.sqrt(12.0)
    return noise_turns**2 + quant**2


def run_observer(
    observer: BaseObserver,
    scenario: Scenario,
    angle_fn,
    sensor_completion_times: np.ndarray,
    sensor_timestamp_times: np.ndarray,
    sensor_measurements: np.ndarray,
) -> Metrics:
    control_times = np.arange(
        scenario.control_dt,
        scenario.duration_s + scenario.control_dt * 0.5,
        scenario.control_dt,
    )

    measurement_index = 0
    errors_turn = []
    est_sectors = []
    true_sectors = []
    spurious_transitions = 0

    prev_est_sector: int | None = None
    prev_true_sector: int | None = None

    for control_time in control_times:
        observer.advance(scenario.control_dt)

        while measurement_index < len(sensor_completion_times) and sensor_completion_times[measurement_index] <= control_time:
            age_s = control_time - sensor_timestamp_times[measurement_index]
            observer.update(
                float(sensor_measurements[measurement_index]),
                age_s=age_s,
                timestamp_s=float(sensor_timestamp_times[measurement_index]),
            )
            measurement_index += 1

        estimate_turn = observer.angle_turns()
        truth_turn = angle_fn(float(control_time))

        err_turn = wrap_turn_signed(estimate_turn - truth_turn)
        errors_turn.append(err_turn)

        est_sector = int(sector_index(np.asarray([estimate_turn]))[0])
        true_sector = int(sector_index(np.asarray([truth_turn]))[0])
        est_sectors.append(est_sector)
        true_sectors.append(true_sector)

        if prev_est_sector is not None and prev_true_sector is not None:
            if est_sector != prev_est_sector and true_sector == prev_true_sector:
                spurious_transitions += 1
        prev_est_sector = est_sector
        prev_true_sector = true_sector

    errors_turn_arr = np.asarray(errors_turn)
    abs_elec_deg = np.abs(mech_turns_to_elec_deg(errors_turn_arr))
    est_sector_arr = np.asarray(est_sectors)
    true_sector_arr = np.asarray(true_sectors)

    return Metrics(
        rms_elec_deg=float(np.sqrt(np.mean(abs_elec_deg**2))),
        p95_elec_deg=float(np.percentile(abs_elec_deg, 95.0)),
        max_elec_deg=float(np.max(abs_elec_deg)),
        wrong_sector_rate=float(np.mean(est_sector_arr != true_sector_arr)),
        missed_sector_rate=float(
            np.mean(
                (est_sector_arr[1:] == est_sector_arr[:-1])
                & (true_sector_arr[1:] != true_sector_arr[:-1])
            )
        ),
        spurious_transitions_per_s=float(spurious_transitions / scenario.duration_s),
    )


def simulate_scenario(scenario: Scenario) -> dict[str, Metrics]:
    angle_fn, omega_fn = make_speed_profile(
        turns_per_s=scenario.mech_turns_per_s,
        ripple_frac=scenario.ripple_frac,
        ripple_hz=scenario.ripple_hz,
    )

    if scenario.triggered:
        sensor_completion_times = np.arange(
            scenario.control_dt,
            scenario.duration_s + scenario.control_dt * 0.5,
            scenario.control_dt,
        )
        sensor_timestamp_times = sensor_completion_times - scenario.triggered_age_s
    else:
        sensor_completion_times = np.arange(
            scenario.sensor_dt,
            scenario.duration_s + scenario.sensor_dt,
            scenario.sensor_dt,
        )
        sensor_timestamp_times = sensor_completion_times - scenario.sensor_group_delay_s - scenario.read_delay_s

    rng = np.random.default_rng(12345)
    sensor_measurements = []
    for sample_time in sensor_timestamp_times:
        truth = angle_fn(float(sample_time)) + scenario.static_bias_turns
        noisy = truth + rng.normal(0.0, scenario.noise_turns)
        sensor_measurements.append(quantize_turn(noisy))
    sensor_measurements_arr = np.asarray(sensor_measurements, dtype=np.float64)

    sigma_meas_turns = math.sqrt(measurement_variance_turns(scenario.noise_turns))

    observers: Iterable[BaseObserver] = (
        RawObserver(),
        VelocityLeadObserver(window_s=1.0e-3),
        DelayAlphaBeta(
            dt_s=scenario.control_dt,
            alpha=3.0 / 8.0,
            beta=1.0 / 16.0,
            age_s=scenario.triggered_age_s if scenario.triggered else scenario.read_delay_s + scenario.sensor_group_delay_s + 0.5 * scenario.control_dt,
        ),
        DelayKalmanCV(
            sigma_meas_turns=sigma_meas_turns,
            sigma_accel_turn_s2=scenario.mech_turns_per_s * scenario.ripple_frac * 2.0 * math.pi * scenario.ripple_hz,
        ),
        DelayKalmanCA(
            sigma_meas_turns=sigma_meas_turns,
            sigma_jerk_turn_s3=scenario.mech_turns_per_s
            * scenario.ripple_frac
            * (2.0 * math.pi * scenario.ripple_hz) ** 2,
        ),
        OracleLeadObserver(omega_fn),
    )

    results: dict[str, Metrics] = {}
    for observer in observers:
        results[observer.name] = run_observer(
            observer=observer,
            scenario=scenario,
            angle_fn=angle_fn,
            sensor_completion_times=sensor_completion_times,
            sensor_timestamp_times=sensor_timestamp_times,
            sensor_measurements=sensor_measurements_arr,
        )
    return results


def print_loop_timing() -> None:
    print("Loop timing")
    for label, control_hz, sensor_hz, triggered in (
        ("legacy main.c", LEGACY_CONTROL_HZ, LEGACY_SENSOR_HZ, False),
        ("legacy + XY 1x", LEGACY_CONTROL_HZ, FAST_XY_SENSOR_HZ, False),
        ("current firmware", CURRENT_CONTROL_HZ, CURRENT_SENSOR_HZ, True),
    ):
        control_dt = 1.0 / control_hz
        sensor_dt = 1.0 / sensor_hz
        if triggered:
            min_age = max_age = (READ_DELAY_S + 0.5 * sensor_dt + 0.5 * control_dt) * 1e6
        else:
            min_age = (0.5 * sensor_dt + READ_DELAY_S) * 1e6
            max_age = (1.5 * sensor_dt + READ_DELAY_S) * 1e6
        one_sample_rpm = control_hz * 60.0 / SECTORS_PER_MECH_TURN
        two_sample_rpm = one_sample_rpm / 2.0
        three_sample_rpm = one_sample_rpm / 3.0

        print(f"  {label}:")
        print(f"    control loop: {control_hz:.0f} Hz ({control_dt * 1e6:.1f} us)")
        print(f"    sensor update: {sensor_hz:.0f} Hz ({sensor_dt * 1e6:.1f} us)")
        if triggered:
            print(f"    sample timestamp age: {min_age:.1f} us")
        else:
            print(f"    sample timestamp age: {min_age:.1f} to {max_age:.1f} us")
        print(f"    1 control tick / sector: {one_sample_rpm:.0f} rpm")
        print(f"    2 control ticks / sector: {two_sample_rpm:.0f} rpm")
        print(f"    3 control ticks / sector: {three_sample_rpm:.0f} rpm")
    print()


def print_results(title: str, results: dict[str, Metrics]) -> None:
    print(title)
    print(
        "  %-14s %10s %10s %10s %10s %10s %10s"
        % (
            "observer",
            "rms_e_deg",
            "p95_e_deg",
            "max_e_deg",
            "wrong_%", 
            "missed_%",
            "spur/s",
        )
    )
    for name, metrics in results.items():
        print(
            "  %-14s %10.2f %10.2f %10.2f %10.2f %10.2f %10.1f"
            % (
                name,
                metrics.rms_elec_deg,
                metrics.p95_elec_deg,
                metrics.max_elec_deg,
                metrics.wrong_sector_rate * 100.0,
                metrics.missed_sector_rate * 100.0,
                metrics.spurious_transitions_per_s,
            )
        )
    print()


def run_sweeps() -> None:
    print_loop_timing()

    base_cases = [
        Scenario(rpm=1000.0, noise_deg=0.5, ripple_frac=0.02, ripple_hz=120.0),
        Scenario(rpm=3000.0, noise_deg=0.5, ripple_frac=0.02, ripple_hz=120.0),
        Scenario(rpm=6000.0, noise_deg=0.5, ripple_frac=0.02, ripple_hz=120.0),
        Scenario(rpm=10000.0, noise_deg=0.5, ripple_frac=0.02, ripple_hz=120.0),
    ]
    for case in base_cases:
        print_results(
            "Scenario: %.0f rpm, %.1f deg noise, %.0f%% ripple @ %.0f Hz"
            % (case.rpm, case.noise_deg, case.ripple_frac * 100.0, case.ripple_hz),
            simulate_scenario(case),
        )

    noisy_case = Scenario(rpm=3000.0, noise_deg=2.0, ripple_frac=0.02, ripple_hz=120.0)
    print_results(
        "Scenario: %.0f rpm, %.1f deg noise, %.0f%% ripple @ %.0f Hz"
        % (noisy_case.rpm, noisy_case.noise_deg, noisy_case.ripple_frac * 100.0, noisy_case.ripple_hz),
        simulate_scenario(noisy_case),
    )

    bias_case = Scenario(
        rpm=3000.0,
        noise_deg=0.5,
        ripple_frac=0.02,
        ripple_hz=120.0,
        static_bias_deg=5.0,
    )
    print_results(
        "Scenario: %.0f rpm, %.1f deg static bias, %.1f deg noise"
        % (bias_case.rpm, bias_case.static_bias_deg, bias_case.noise_deg),
        simulate_scenario(bias_case),
    )

    timing_cases = [
        Scenario(
            rpm=10000.0,
            noise_deg=0.5,
            ripple_frac=0.02,
            ripple_hz=120.0,
            control_hz=LEGACY_CONTROL_HZ,
            sensor_hz=LEGACY_SENSOR_HZ,
            triggered=False,
        ),
        Scenario(
            rpm=10000.0,
            noise_deg=0.5,
            ripple_frac=0.02,
            ripple_hz=120.0,
            control_hz=LEGACY_CONTROL_HZ,
            sensor_hz=FAST_XY_SENSOR_HZ,
            triggered=False,
        ),
        Scenario(rpm=10000.0, noise_deg=0.5, ripple_frac=0.02, ripple_hz=120.0),
    ]
    for case, label in zip(
        timing_cases,
        ("legacy main.c timing", "legacy + XY 1x sensor", "current firmware timing"),
        strict=True,
    ):
        print_results(
            "Timing comparison: %s at %.0f rpm" % (label, case.rpm),
            simulate_scenario(case),
        )


if __name__ == "__main__":
    run_sweeps()
