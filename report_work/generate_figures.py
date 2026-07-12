from __future__ import annotations

import csv
import re
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch


ROOT = Path(__file__).resolve().parents[1]
TOOLS = ROOT / "project" / "tools"
OUT = Path(__file__).resolve().parent / "figures"
OUT.mkdir(parents=True, exist_ok=True)

plt.rcParams.update({
    "font.family": "DejaVu Sans",
    "axes.spines.top": False,
    "axes.spines.right": False,
    "axes.grid": True,
    "grid.color": "#D9DEE7",
    "grid.linewidth": 0.7,
    "figure.dpi": 160,
})


def read_height_log() -> list[dict[str, float]]:
    path = TOOLS / "height_log.txt"
    lines = path.read_text(encoding="utf-8", errors="ignore").splitlines()
    begin = next(i for i, line in enumerate(lines) if line.startswith("seq,"))
    end = next(i for i, line in enumerate(lines[begin + 1 :], begin + 1) if line.startswith("HEIGHT_LOG_END"))
    return list(csv.DictReader(lines[begin:end]))


def save(fig: plt.Figure, name: str) -> None:
    fig.tight_layout()
    fig.savefig(OUT / name, bbox_inches="tight", facecolor="white")
    plt.close(fig)


def plot_height() -> None:
    rows = read_height_log()
    t0 = float(rows[0]["time_us"])
    t = [(float(r["time_us"]) - t0) / 1e6 for r in rows]
    tof = [float(r["tof_cm"]) for r in rows]
    ekf = [float(r["ekf_z_cm"]) for r in rows]
    final = [float(r["final_height_cm"]) for r in rows]
    profile = [float(r["profile_height_cm"]) for r in rows]

    fig, ax = plt.subplots(figsize=(8.3, 4.5))
    ax.plot(t, tof, color="#B4BCCB", lw=1.0, label="ToF measurement")
    ax.plot(t, ekf, color="#165DAB", lw=2.0, label="EKF height")
    ax.plot(t, final, color="#F08A24", lw=1.6, label="Control feedback height")
    ax.plot(t, profile, color="#555D6E", lw=1.4, ls="--", label="Profile reference")
    ax.set(title="Altitude-estimation log", xlabel="Time (s)", ylabel="Height (cm)")
    ax.legend(ncol=2, frameon=False, loc="upper left")
    save(fig, "fig_height_estimation.png")

    vz = [float(r["ekf_vz_cm_s"]) for r in rows]
    ref_vz = [float(r["profile_vz_cm_s"]) for r in rows]
    throttle = [float(r["throttle"]) for r in rows]
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8.3, 5.5), sharex=True,
                                   gridspec_kw={"height_ratios": [2, 1]})
    ax1.plot(t, vz, color="#165DAB", lw=1.6, label="EKF vertical velocity")
    ax1.plot(t, ref_vz, color="#F08A24", lw=1.4, ls="--", label="Profile vertical velocity")
    ax1.axhline(0, color="#555D6E", lw=0.8)
    ax1.set(title="Vertical-state and throttle log", ylabel="Velocity (cm/s)")
    ax1.legend(frameon=False, loc="upper left")
    ax2.plot(t, throttle, color="#6D5BA8", lw=1.5)
    ax2.set(xlabel="Time (s)", ylabel="Throttle")
    save(fig, "fig_vertical_state.png")


def plot_attitude() -> None:
    text = (TOOLS / "debug_log_2.txt").read_text(encoding="utf-8", errors="ignore")
    pat = re.compile(r"roll=([-+0-9.]+),\s*pitch=([-+0-9.]+),yaw=([-+0-9.]+)")
    values = [tuple(map(float, match.groups())) for match in pat.finditer(text)]
    if not values:
        return
    samples = list(range(len(values)))
    roll = [v[0] for v in values]
    pitch = [v[1] for v in values]
    fig, ax = plt.subplots(figsize=(8.3, 4.3))
    ax.plot(samples, roll, color="#165DAB", marker="o", ms=2.5, lw=1.4, label="Roll")
    ax.plot(samples, pitch, color="#F08A24", marker="o", ms=2.5, lw=1.4, label="Pitch")
    ax.axhline(0, color="#555D6E", lw=0.8)
    ax.set(title="Attitude samples from debug log", xlabel="Debug sample index", ylabel="Angle (deg)")
    ax.legend(frameon=False, loc="upper right")
    save(fig, "fig_attitude_debug.png")


def plot_closure() -> None:
    # Values are the recorded end-point residuals of the documented out-and-back test.
    body = (-1.43, -3.14)
    earth = (1.51, -9.42)
    fig, ax = plt.subplots(figsize=(6.8, 5.0))
    ax.axhline(0, color="#7A8291", lw=0.9)
    ax.axvline(0, color="#7A8291", lw=0.9)
    ax.scatter([0], [0], s=100, marker="*", color="#313947", label="Ideal return")
    ax.scatter([body[0]], [body[1]], s=90, color="#165DAB", label="Body-frame residual")
    ax.scatter([earth[0]], [earth[1]], s=90, color="#F08A24", label="Earth-frame residual")
    ax.annotate("(-1.43, -3.14) cm", body, xytext=(8, 8), textcoords="offset points", color="#165DAB")
    ax.annotate("(1.51, -9.42) cm", earth, xytext=(8, -14), textcoords="offset points", color="#B86610")
    ax.set(title="Out-and-back position closure residual", xlabel="X residual (cm)", ylabel="Y residual (cm)")
    ax.set_aspect("equal", adjustable="box")
    ax.legend(frameon=False, loc="upper right")
    save(fig, "fig_position_closure.png")


def plot_architecture() -> None:
    fig, ax = plt.subplots(figsize=(9.2, 4.8))
    ax.set_axis_off()
    boxes = [
        (0.05, 0.72, "IMU\nICM42688", "#E9F2FF"),
        (0.05, 0.42, "ToF height\nVL53L8 / DL1B", "#FFF1DF"),
        (0.05, 0.12, "Optical flow\nLC302", "#EAF7EC"),
        (0.33, 0.48, "State estimation\nMahony + EKF-lite", "#EDF0F5"),
        (0.59, 0.48, "Cascade control\nposition / velocity / attitude", "#E9F2FF"),
        (0.84, 0.48, "Motor mixing\nand ESC output", "#FFF1DF"),
    ]
    for x, y, label, color in boxes:
        ax.add_patch(FancyBboxPatch((x, y), 0.13, 0.18, boxstyle="round,pad=0.02", fc=color, ec="#516072", lw=1.1))
        ax.text(x + 0.065, y + 0.09, label, ha="center", va="center", fontsize=9)
    arrows = [(0.18, 0.81, 0.33, 0.57), (0.18, 0.51, 0.33, 0.57), (0.18, 0.21, 0.33, 0.57),
              (0.46, 0.57, 0.59, 0.57), (0.72, 0.57, 0.84, 0.57)]
    for x1, y1, x2, y2 in arrows:
        ax.annotate("", xy=(x2, y2), xytext=(x1, y1), arrowprops={"arrowstyle": "->", "lw": 1.3, "color": "#516072"})
    ax.set_title("Flight-control software architecture", fontsize=14, pad=12)
    save(fig, "fig_control_architecture.png")


if __name__ == "__main__":
    plot_height()
    plot_attitude()
    plot_closure()
    plot_architecture()
