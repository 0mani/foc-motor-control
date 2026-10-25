"""Closed-loop simulation: C FOC controller + PMSM plant + sensors."""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Callable

import numpy as np

from .foc_lib import MODE_SPEED, FocController
from .motor import ideal_sensors, MotorParams, Pmsm, SensorModel, inverter_voltage

RPM_PER_RADS = 60.0 / (2.0 * math.pi)


@dataclass
class SimConfig:
    motor: MotorParams = field(default_factory=MotorParams)
    vdc: float = 24.0
    f_pwm: float = 20e3           # current loop / PWM frequency [Hz]
    speed_div: int = 10           # speed loop at f_pwm / speed_div
    bw_current_hz: float = 1000.0
    bw_speed_hz: float = 50.0
    iq_max: float = 10.0
    substeps: int = 4             # plant RK4 sub-steps per PWM period
    sensors: SensorModel | None = None   # None = realistic defaults
    decouple: bool = True
    speed_slew: float = 0.0       # [rad/s^2], 0 = off

    @property
    def ts(self) -> float:
        return 1.0 / self.f_pwm


def run(cfg: SimConfig, t_end: float,
        speed_ref: Callable[[float], float],
        load: Callable[[float], float] = lambda t: 0.0,
        gains: dict | None = None) -> dict:
    """Simulate and return logged signals as numpy arrays.

    speed_ref(t) is in rad/s (mechanical), load(t) in N*m.
    The controller output is applied one PWM period later, like a real
    MCU that computes in the ADC interrupt and updates the timer at the
    next reload.
    """
    sensors = cfg.sensors if cfg.sensors is not None else SensorModel()
    motor = Pmsm(cfg.motor)
    ctrl = FocController(cfg.motor, cfg.ts, cfg.speed_div,
                         cfg.bw_current_hz, cfg.bw_speed_hz, cfg.iq_max)
    if gains:
        g = ctrl.gains
        g.update(gains)
        ctrl.set_gains(g["kp_current"], g["ki_current"],
                       g["kp_speed"], g["ki_speed"])
    ctrl.set_decouple(cfg.decouple)
    ctrl.set_slew(cfg.speed_slew)
    ctrl.set_mode(MODE_SPEED)

    n = int(round(t_end / cfg.ts))
    log = {k: np.empty(n) for k in (
        "t", "speed_ref", "speed", "speed_meas", "id", "iq", "iq_ref",
        "vd", "vq", "duty_a", "duty_b", "duty_c", "load", "torque",
        "v_limited", "theta_e")}

    duty = (0.5, 0.5, 0.5)
    pp = cfg.motor.pole_pairs
    dt = cfg.ts / cfg.substeps
    for k in range(n):
        t = k * cfg.ts
        w_ref = speed_ref(t)
        tl = load(t)
        ctrl.set_speed(w_ref)

        # sample (start of period)
        ia, ib, _ = motor.phase_currents()
        th_meas = sensors.angle(motor.s.theta_m)
        w_meas = sensors.speed(th_meas, cfg.ts)
        out = ctrl.step(sensors.current(ia), sensors.current(ib),
                        pp * th_meas, w_meas, cfg.vdc)

        # previous period's duty is what the timer is applying now
        v_alpha, v_beta = inverter_voltage(*duty, cfg.vdc)
        for _ in range(cfg.substeps):
            motor.step(v_alpha, v_beta, tl, dt)
        duty = out[0:3]

        log["t"][k] = t
        log["speed_ref"][k] = w_ref
        log["speed"][k] = motor.s.omega_m
        log["speed_meas"][k] = w_meas
        log["id"][k] = motor.s.id
        log["iq"][k] = motor.s.iq
        log["iq_ref"][k] = out[7]
        log["vd"][k] = out[5]
        log["vq"][k] = out[6]
        log["duty_a"][k], log["duty_b"][k], log["duty_c"][k] = out[0:3]
        log["load"][k] = tl
        log["torque"][k] = motor.torque
        log["v_limited"][k] = out[8]
        log["theta_e"][k] = pp * motor.s.theta_m
    log["gains"] = ctrl.gains
    return log


def step_metrics(t, y, t0, y0, y1, band=0.02):
    """Rise time (10-90%), overshoot [%] and settling time for a step y0->y1 at t0."""
    m = t >= t0
    t, y = t[m] - t0, y[m]
    span = y1 - y0
    frac = (y - y0) / span
    try:
        t10 = t[np.argmax(frac >= 0.1)]
        t90 = t[np.argmax(frac >= 0.9)]
        rise = t90 - t10
    except ValueError:
        rise = float("nan")
    overshoot = max(0.0, (frac.max() - 1.0) * 100.0)
    outside = np.nonzero(np.abs(frac - 1.0) > band)[0]
    settle = t[outside[-1]] if len(outside) else 0.0
    if len(outside) and outside[-1] == len(t) - 1:
        settle = float("nan")
    return {"rise_s": float(rise), "overshoot_pct": float(overshoot),
            "settle_s": float(settle)}


__all__ = ["SimConfig", "run", "step_metrics", "RPM_PER_RADS", "ideal_sensors"]
