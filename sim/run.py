"""Run the simulation scenarios and render the figures in docs/.

    python -m sim.run            # all scenarios
    python -m sim.run overview   # one scenario
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from .motor import ideal_sensors, SensorModel  # noqa: E402
from .simulate import RPM_PER_RADS, SimConfig, run, step_metrics  # noqa: E402

DOCS = Path(__file__).resolve().parent.parent / "docs"

# chart palette (categorical order is fixed: blue, orange, aqua)
BLUE, ORANGE, AQUA = "#2a78d6", "#eb6834", "#1baf7a"
INK, INK2, MUTED = "#0b0b0b", "#52514e", "#898781"
GRID, AXIS, SURFACE = "#e1e0d9", "#c3c2b7", "#fcfcfb"


def _style():
    plt.rcParams.update({
        "figure.facecolor": SURFACE, "axes.facecolor": SURFACE,
        "savefig.facecolor": SURFACE, "font.size": 10,
        "axes.edgecolor": AXIS, "axes.labelcolor": INK2,
        "axes.titlesize": 11, "axes.titleweight": "bold", "axes.titlecolor": INK,
        "axes.titlelocation": "left", "axes.spines.top": False,
        "axes.spines.right": False, "axes.grid": True, "grid.color": GRID,
        "grid.linewidth": 0.6, "xtick.color": MUTED, "ytick.color": MUTED,
        "legend.frameon": False, "legend.labelcolor": INK2,
        "lines.linewidth": 1.6, "lines.solid_capstyle": "round",
    })


def _rpm(x):
    return x * RPM_PER_RADS


def _event(ax, t, text, y=0.97):
    ax.axvline(t, color=AXIS, lw=0.8, ls=(0, (3, 3)), zorder=0)
    ax.text(t, y, f" {text}", transform=ax.get_xaxis_transform(),
            color=INK2, fontsize=8.5, va="top")


def scenario_overview():
    """Start-up, load step, reversal and load release on one timeline."""
    w1, w2 = 2000 / RPM_PER_RADS, -1500 / RPM_PER_RADS

    def ref(t):
        return 0.0 if t < 0.02 else (w1 if t < 0.35 else w2)

    def load(t):
        return 0.15 if 0.20 <= t < 0.50 else 0.0

    r = run(SimConfig(), 0.6, ref, load)
    t = r["t"] * 1e3

    fig, (a1, a2) = plt.subplots(2, 1, figsize=(9, 6), sharex=True,
                                 gridspec_kw={"height_ratios": [1.3, 1]})
    a1.plot(t, _rpm(r["speed_ref"]), color=MUTED, lw=1.2, ls=(0, (4, 3)),
            label="reference")
    a1.plot(t, _rpm(r["speed"]), color=BLUE, label="rotor speed")
    a1.set_ylabel("speed [rpm]")
    a1.set_ylim(-1900, 2750)
    a1.set_title("Speed control: start-up, 0.15 N·m load step, reversal")
    a1.legend(loc="lower left")
    for tt, txt in ((20, "step to 2000 rpm"), (200, "load on"),
                    (350, "reverse to −1500 rpm"), (500, "load off")):
        _event(a1, tt, txt)

    a2.plot(t, r["iq_ref"], color=MUTED, lw=1.0, ls=(0, (4, 3)),
            label="iq reference")
    a2.plot(t, r["iq"], color=BLUE, label="iq (torque)")
    a2.plot(t, r["id"], color=ORANGE, label="id (flux)")
    a2.set_ylabel("current [A]")
    a2.set_xlabel("time [ms]")
    a2.legend(loc="lower left", ncol=3)
    fig.tight_layout()
    fig.savefig(DOCS / "overview.png", dpi=150)
    plt.close(fig)

    m = step_metrics(r["t"], r["speed"], 0.02, 0.0, w1)
    k = (r["t"] > 0.2) & (r["t"] < 0.3)
    dip = _rpm(w1 - r["speed"][k].min())
    return {"start-up rise time 10–90 %": f"{m['rise_s'] * 1e3:.1f} ms",
            "start-up overshoot": f"{m['overshoot_pct']:.1f} %",
            "speed dip on 0.15 N·m load step": f"{dip:.0f} rpm",
            "auto-tuned gains": r["gains"]}


def scenario_tuning():
    """Speed-loop bandwidth sweep with the model-based auto-tuner."""
    w0, w1 = 1000 / RPM_PER_RADS, 1200 / RPM_PER_RADS
    bws = (20.0, 50.0, 100.0)
    colors = (BLUE, ORANGE, AQUA)
    rows = []

    fig, (a1, a2) = plt.subplots(1, 2, figsize=(10, 4))
    for bw, col in zip(bws, colors):
        cfg = SimConfig(bw_speed_hz=bw)
        r = run(cfg, 0.5, lambda t: w0 if t < 0.25 else w1,
                lambda t: 0.1 if t >= 0.40 else 0.0)
        t = r["t"]
        k = (t >= 0.24) & (t < 0.36)
        a1.plot((t[k] - 0.25) * 1e3, _rpm(r["speed"][k]), color=col,
                label=f"{bw:.0f} Hz")
        k2 = (t >= 0.39) & (t < 0.5)
        a2.plot((t[k2] - 0.40) * 1e3, _rpm(r["speed"][k2] - w1), color=col,
                label=f"{bw:.0f} Hz")
        sm = step_metrics(t[t < 0.40], r["speed"][t < 0.40], 0.25, w0, w1)
        dip = _rpm(w1 - r["speed"][k2].min())
        rows.append((bw, sm["rise_s"] * 1e3, sm["overshoot_pct"],
                     sm["settle_s"] * 1e3, dip, r["gains"]["kp_speed"],
                     r["gains"]["ki_speed"]))

    a1.axhline(1200, color=MUTED, lw=1.0, ls=(0, (4, 3)))
    a1.set_title("Setpoint step 1000 → 1200 rpm")
    a1.set_xlabel("time after step [ms]")
    a1.set_ylabel("speed [rpm]")
    a1.set_ylim(-1900, 2750)
    a1.legend(title="speed-loop bandwidth", title_fontsize=9, loc="lower right")
    a2.axhline(0, color=AXIS, lw=0.8)
    a2.set_title("Disturbance: 0.1 N·m load step")
    a2.set_xlabel("time after load step [ms]")
    a2.set_ylabel("speed error [rpm]")
    fig.tight_layout()
    fig.savefig(DOCS / "tuning.png", dpi=150)
    plt.close(fig)
    return rows


def scenario_decoupling():
    """dq cross-coupling with and without feed-forward decoupling."""
    w = 2500 / RPM_PER_RADS
    fig, ax = plt.subplots(figsize=(9, 3.4))
    peaks = {}
    for dec, col, name in ((False, ORANGE, "without decoupling"),
                           (True, BLUE, "with decoupling feed-forward")):
        r = run(SimConfig(decouple=dec, sensors=SensorModel(current_noise_a=0.0,
                                                             adc_lsb_a=0.0)),
                0.26,
                lambda t: 0.0 if t < 0.01 else w,
                lambda t: 0.2 if t >= 0.20 else 0.0)
        t = r["t"]
        k = (t >= 0.195) & (t < 0.26)
        ax.plot((t[k] - 0.2) * 1e3, r["id"][k], color=col, label=name)
        peaks[name] = float(np.abs(r["id"][(t >= 0.2)]).max())
    ax.axhline(0, color=AXIS, lw=0.8)
    ax.set_title("id disturbance on a 0.2 N·m load step at 2500 rpm (id ref = 0)")
    ax.set_xlabel("time after load step [ms]")
    ax.set_ylabel("id [A]")
    ax.legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(DOCS / "decoupling.png", dpi=150)
    plt.close(fig)
    return peaks


def scenario_svpwm():
    """SVPWM duty waveforms over one electrical revolution at steady state."""
    w = 2000 / RPM_PER_RADS
    r = run(SimConfig(sensors=ideal_sensors()), 0.2, lambda t: w,
            lambda t: 0.15)
    t = r["t"]
    f_e = 4 * w / (2 * math.pi)
    k = np.nonzero(t >= (t[-1] - 1.0 / f_e))[0]
    ang = np.degrees(r["theta_e"][k]) % 360.0
    order = np.argsort(ang)
    k, x = k[order], ang[order]
    fig, ax = plt.subplots(figsize=(9, 3.4))
    for key, col, lab in (("duty_a", BLUE, "phase A"),
                          ("duty_b", ORANGE, "phase B"),
                          ("duty_c", AQUA, "phase C")):
        ax.plot(x, r[key][k], color=col, label=lab)
    ax.set_xlim(0, 360)
    ax.set_xticks(range(0, 361, 60))
    ax.set_ylim(0, 1)
    ax.set_title("SVPWM duty cycles over one electrical cycle (min/max injection)")
    ax.set_xlabel("electrical angle [deg]")
    ax.set_ylabel("duty cycle")
    ax.legend(loc="upper right", ncol=3)
    fig.tight_layout()
    fig.savefig(DOCS / "svpwm.png", dpi=150)
    plt.close(fig)


SCENARIOS = {
    "overview": scenario_overview,
    "tuning": scenario_tuning,
    "decoupling": scenario_decoupling,
    "svpwm": scenario_svpwm,
}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("scenario", nargs="?", choices=[*SCENARIOS, "all"],
                    default="all")
    args = ap.parse_args(argv)
    DOCS.mkdir(exist_ok=True)
    _style()

    names = list(SCENARIOS) if args.scenario == "all" else [args.scenario]
    for name in names:
        res = SCENARIOS[name]()
        print(f"\n== {name} ==")
        if name == "tuning":
            print("bw [Hz] | rise [ms] | overshoot [%] | settle 2% [ms] | "
                  "load dip [rpm] | Kp_w | Ki_w")
            for row in res:
                print(f"{row[0]:7.0f} | {row[1]:9.1f} | {row[2]:13.1f} | "
                      f"{row[3]:14.1f} | {row[4]:14.0f} | {row[5]:.3f} | "
                      f"{row[6]:.1f}")
        elif isinstance(res, dict):
            for k, v in res.items():
                print(f"  {k}: {v}")
    print(f"\nfigures written to {DOCS}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
