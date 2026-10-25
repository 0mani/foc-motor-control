"""Closed-loop regression tests: the C controller against the PMSM model."""
import numpy as np
import pytest

from sim.motor import ideal_sensors
from sim.simulate import RPM_PER_RADS, SimConfig, run, step_metrics

W = 1500 / RPM_PER_RADS


def test_speed_step_tracks_reference():
    r = run(SimConfig(), 0.25, lambda t: 0.0 if t < 0.01 else W)
    m = step_metrics(r["t"], r["speed"], 0.01, 0.0, W)
    assert m["overshoot_pct"] < 10.0
    assert m["rise_s"] < 0.03
    tail = r["speed"][r["t"] > 0.2]
    assert abs(tail.mean() - W) < 0.005 * W


def test_load_rejected_with_zero_steady_state_error():
    r = run(SimConfig(), 0.4, lambda t: W, lambda t: 0.15 if t >= 0.2 else 0.0)
    tail = r["speed"][r["t"] > 0.35]
    assert abs(tail.mean() - W) < 0.005 * W
    # torque balances the load plus friction once settled
    torque = r["torque"][r["t"] > 0.35].mean()
    assert torque == pytest.approx(0.15 + 1e-5 * W, rel=0.05)


def test_current_limit_respected():
    cfg = SimConfig(iq_max=6.0)
    r = run(cfg, 0.1, lambda t: 0.0 if t < 0.01 else 3000 / RPM_PER_RADS)
    assert np.abs(r["iq_ref"]).max() <= 6.0 + 1e-6
    assert np.abs(r["iq"]).max() < 6.0 * 1.1


def test_id_held_near_zero():
    r = run(SimConfig(sensors=ideal_sensors()), 0.2, lambda t: W,
            lambda t: 0.1 if t >= 0.1 else 0.0)
    assert np.abs(r["id"][r["t"] > 0.02]).max() < 0.2


def test_duties_within_bounds():
    r = run(SimConfig(), 0.15, lambda t: 2500 / RPM_PER_RADS, lambda t: 0.2)
    for ph in ("duty_a", "duty_b", "duty_c"):
        assert r[ph].min() >= 0.0 and r[ph].max() <= 1.0


def test_decoupling_reduces_id_disturbance():
    peaks = []
    for dec in (False, True):
        r = run(SimConfig(decouple=dec, sensors=ideal_sensors()), 0.25,
                lambda t: 2500 / RPM_PER_RADS,
                lambda t: 0.2 if t >= 0.2 else 0.0)
        peaks.append(np.abs(r["id"][r["t"] >= 0.2]).max())
    assert peaks[1] < 0.6 * peaks[0]


def test_tuner_rejects_unsafe_bandwidth():
    with pytest.raises(ValueError):
        run(SimConfig(bw_current_hz=5000.0), 0.01, lambda t: 0.0)
