"""ctypes binding to the C FOC library (build/libfoc_sim.so).

The Python simulator drives the exact C code that would run on the MCU.
"""
from __future__ import annotations

import ctypes
import os
from pathlib import Path

from .motor import MotorParams

MODE_DISABLED, MODE_TORQUE, MODE_SPEED = 0, 1, 2

_F = ctypes.c_float
_FP = ctypes.POINTER(ctypes.c_float)


def _find_lib() -> Path:
    env = os.environ.get("FOC_SIM_LIB")
    if env:
        return Path(env)
    root = Path(__file__).resolve().parent.parent
    for name in ("libfoc_sim.so", "libfoc_sim.dylib", "foc_sim.dll"):
        p = root / "build" / name
        if p.exists():
            return p
    raise FileNotFoundError(
        "libfoc_sim not found: run `cmake -S . -B build && cmake --build build` "
        "or set FOC_SIM_LIB"
    )


_lib = ctypes.CDLL(str(_find_lib()))
_lib.focsim_create.restype = ctypes.c_void_p
_lib.focsim_create.argtypes = [_F, _F, _F, ctypes.c_int, _F, _F, _F,
                               ctypes.c_int, _F, _F, _F]
_lib.focsim_destroy.argtypes = [ctypes.c_void_p]
_lib.focsim_set_mode.argtypes = [ctypes.c_void_p, ctypes.c_int]
_lib.focsim_set_speed.argtypes = [ctypes.c_void_p, _F]
_lib.focsim_set_current.argtypes = [ctypes.c_void_p, _F, _F]
_lib.focsim_set_slew.argtypes = [ctypes.c_void_p, _F]
_lib.focsim_set_decouple.argtypes = [ctypes.c_void_p, ctypes.c_int]
_lib.focsim_set_gains.argtypes = [ctypes.c_void_p, _F, _F, _F, _F]
_lib.focsim_get_gains.argtypes = [ctypes.c_void_p, _FP]
_lib.focsim_step.argtypes = [ctypes.c_void_p, _F, _F, _F, _F, _F, _FP]


class FocController:
    """Thin object wrapper around one C controller instance."""

    def __init__(self, motor: MotorParams, ts: float, speed_div: int,
                 bw_current_hz: float, bw_speed_hz: float, iq_max: float):
        self._h = _lib.focsim_create(
            motor.rs, motor.ls, motor.flux, motor.pole_pairs, motor.inertia,
            motor.friction, ts, speed_div, bw_current_hz, bw_speed_hz, iq_max)
        if not self._h:
            raise ValueError("foc_tune rejected these bandwidths "
                             "(time-scale separation rule violated)")
        self._out = (ctypes.c_float * 9)()

    def __del__(self):
        h = getattr(self, "_h", None)
        if h:
            _lib.focsim_destroy(h)
            self._h = None

    def set_mode(self, mode: int):
        _lib.focsim_set_mode(self._h, mode)

    def set_speed(self, omega_m: float):
        _lib.focsim_set_speed(self._h, omega_m)

    def set_current(self, id_ref: float, iq_ref: float):
        _lib.focsim_set_current(self._h, id_ref, iq_ref)

    def set_slew(self, slew: float):
        _lib.focsim_set_slew(self._h, slew)

    def set_decouple(self, on: bool):
        _lib.focsim_set_decouple(self._h, int(on))

    def set_gains(self, kp_i, ki_i, kp_w, ki_w):
        _lib.focsim_set_gains(self._h, kp_i, ki_i, kp_w, ki_w)

    @property
    def gains(self):
        g = (ctypes.c_float * 4)()
        _lib.focsim_get_gains(self._h, g)
        return {"kp_current": g[0], "ki_current": g[1],
                "kp_speed": g[2], "ki_speed": g[3]}

    def step(self, ia, ib, theta_e, omega_m, vdc):
        """Returns (duty_a, duty_b, duty_c, id, iq, vd, vq, iq_ref, v_limited)."""
        _lib.focsim_step(self._h, ia, ib, theta_e, omega_m, vdc, self._out)
        return tuple(self._out)
