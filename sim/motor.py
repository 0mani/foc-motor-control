"""Surface-mounted PMSM / BLDC plant model, averaged inverter and sensors.

The plant is integrated with RK4 at several sub-steps per PWM period so the
controller under test (the C library) sees realistic continuous dynamics.
"""
from __future__ import annotations

import math
import random
from dataclasses import dataclass, field

SQRT3 = math.sqrt(3.0)


@dataclass
class MotorParams:
    """SI-unit parameters of a small 24 V BLDC motor with load."""

    rs: float = 0.5          # phase resistance [ohm]
    ls: float = 0.5e-3       # phase inductance [H]
    flux: float = 0.008      # PM flux linkage [V*s/rad]
    pole_pairs: int = 4
    inertia: float = 5e-5    # rotor + load [kg*m^2]
    friction: float = 1e-5   # viscous [N*m*s/rad]

    @property
    def kt(self) -> float:
        return 1.5 * self.pole_pairs * self.flux


@dataclass
class MotorState:
    id: float = 0.0
    iq: float = 0.0
    omega_m: float = 0.0
    theta_m: float = 0.0


class Pmsm:
    """dq-frame PMSM with Ld = Lq, driven by stationary-frame voltages."""

    def __init__(self, p: MotorParams):
        self.p = p
        self.s = MotorState()

    def _deriv(self, x, v_alpha, v_beta, load):
        p = self.p
        i_d, i_q, w_m, th_m = x
        th_e = p.pole_pairs * th_m
        c, s = math.cos(th_e), math.sin(th_e)
        v_d = v_alpha * c + v_beta * s
        v_q = -v_alpha * s + v_beta * c
        w_e = p.pole_pairs * w_m
        did = (v_d - p.rs * i_d + w_e * p.ls * i_q) / p.ls
        diq = (v_q - p.rs * i_q - w_e * p.ls * i_d - w_e * p.flux) / p.ls
        torque = p.kt * i_q
        dw = (torque - load - p.friction * w_m) / p.inertia
        return (did, diq, dw, w_m)

    def step(self, v_alpha: float, v_beta: float, load: float, dt: float):
        x = (self.s.id, self.s.iq, self.s.omega_m, self.s.theta_m)
        k1 = self._deriv(x, v_alpha, v_beta, load)
        x2 = tuple(a + 0.5 * dt * b for a, b in zip(x, k1))
        k2 = self._deriv(x2, v_alpha, v_beta, load)
        x3 = tuple(a + 0.5 * dt * b for a, b in zip(x, k2))
        k3 = self._deriv(x3, v_alpha, v_beta, load)
        x4 = tuple(a + dt * b for a, b in zip(x, k3))
        k4 = self._deriv(x4, v_alpha, v_beta, load)
        x = tuple(
            a + dt / 6.0 * (b1 + 2 * b2 + 2 * b3 + b4)
            for a, b1, b2, b3, b4 in zip(x, k1, k2, k3, k4)
        )
        self.s = MotorState(*x)

    def phase_currents(self):
        """Phase currents a, b, c from the dq state."""
        th_e = self.p.pole_pairs * self.s.theta_m
        c, s = math.cos(th_e), math.sin(th_e)
        i_alpha = self.s.id * c - self.s.iq * s
        i_beta = self.s.id * s + self.s.iq * c
        ia = i_alpha
        ib = -0.5 * i_alpha + 0.5 * SQRT3 * i_beta
        return ia, ib, -ia - ib

    @property
    def torque(self) -> float:
        return self.p.kt * self.s.iq


def inverter_voltage(duty_a, duty_b, duty_c, vdc):
    """Averaged 2-level inverter into a star-connected (isolated neutral) motor.

    Returns the alpha/beta stator voltage. The common-mode part injected by
    SVPWM cancels here, exactly as it does on real hardware.
    """
    cm = (duty_a + duty_b + duty_c) / 3.0
    va = (duty_a - cm) * vdc
    vb = (duty_b - cm) * vdc
    return va, (va + 2.0 * vb) / SQRT3


@dataclass
class SensorModel:
    """Current ADC and incremental encoder imperfections."""

    current_noise_a: float = 0.02   # 1-sigma current noise [A]
    adc_lsb_a: float = 0.01         # current quantisation step [A]
    encoder_cpr: int = 16384        # 14-bit magnetic encoder (AS5047-class)
    speed_window: int = 10          # M-method window [current periods]
    seed: int = 1
    _hist: list = field(default_factory=list, repr=False)

    def __post_init__(self):
        self._rng = random.Random(self.seed)

    def current(self, i: float) -> float:
        i = i + self._rng.gauss(0.0, self.current_noise_a)
        return round(i / self.adc_lsb_a) * self.adc_lsb_a if self.adc_lsb_a else i

    def angle(self, theta_m: float) -> float:
        if not self.encoder_cpr:
            return theta_m
        lsb = 2.0 * math.pi / self.encoder_cpr
        return math.floor(theta_m / lsb) * lsb

    def speed(self, theta_meas: float, ts: float) -> float:
        """M-method: angle change over a fixed window of samples."""
        self._hist.append(theta_meas)
        if len(self._hist) > self.speed_window + 1:
            self._hist.pop(0)
        n = len(self._hist) - 1
        if n == 0:
            return 0.0
        return (self._hist[-1] - self._hist[0]) / (n * ts)


def ideal_sensors() -> SensorModel:
    """Noise-free, unquantised sensors (fresh instance: the model has state)."""
    return SensorModel(current_noise_a=0.0, adc_lsb_a=0.0, encoder_cpr=0,
                       speed_window=1)
