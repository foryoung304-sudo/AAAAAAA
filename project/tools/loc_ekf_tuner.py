#!/usr/bin/env python3
"""Offline XY EKF replay/tuner for LOC_LOG CSV captures.

Requires: Python 3.10+ and NumPy.
"""

from __future__ import annotations

import argparse
import csv
import itertools
import math
import re
from dataclasses import dataclass
from pathlib import Path

import numpy as np


COLUMNS = (
    "seq", "time_us",
    "raw_dx_cm", "raw_dy_cm",
    "true_dx_body_cm", "true_dy_body_cm",
    "earth_dx_cm", "earth_dy_cm",
    "flow_x_cm", "flow_y_cm",
    "flow_vx_cm_s", "flow_vy_cm_s",
    "ekf_x_cm", "ekf_y_cm",
    "ekf_vx_cm_s", "ekf_vy_cm_s",
    "innov_vx_cm_s", "innov_vy_cm_s",
    "roll_deg", "pitch_deg", "yaw_deg", "height_cm",
    "raw_valid", "obs_valid", "ekf_used", "gate_reject", "quality", "armed",
)

NEW_COLUMNS = (
    "seq", "time_us",
    "raw_dx_cm", "raw_dy_cm",
    "true_dx_body_cm", "true_dy_body_cm",
    "earth_dx_cm", "earth_dy_cm",
    "flow_x_cm", "flow_y_cm",
    "flow_vx_cm_s", "flow_vy_cm_s",
    "obs_vx_cm_s", "obs_vy_cm_s",
    "ekf_x_cm", "ekf_y_cm",
    "ekf_vx_cm_s", "ekf_vy_cm_s",
    "innov_vx_cm_s", "innov_vy_cm_s",
    "roll_deg", "pitch_deg", "yaw_deg", "height_cm",
    "raw_valid", "obs_valid", "ekf_used", "gate_reject", "quality", "armed",
)


@dataclass(frozen=True)
class Params:
    q_pos: float = 5.0
    q_vel: float = 120.0
    r_vel: float = 900.0
    vel_gate: float = 180.0


def parse_number(token: str) -> float:
    match = re.match(r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)", token.strip())
    if not match:
        raise ValueError(f"not numeric: {token!r}")
    return float(match.group(0))


def parse_loc_log(path: Path) -> dict[str, np.ndarray]:
    text = path.read_text(encoding="utf-8", errors="ignore")
    begin = text.find("LOC_LOG_BEGIN")
    if begin < 0:
        begin = text.find("OC_LOG_BEGIN")
    if begin < 0:
        raise ValueError("missing LOC_LOG_BEGIN")
    end = text.find("LOC_LOG_END", begin)
    if end < 0:
        end = len(text)

    block = text[begin:end]
    header_pos = block.find("seq,time_us")
    if header_pos < 0:
        raise ValueError("missing LOC_LOG CSV header")

    payload = block[header_pos:]
    header_line = payload[: payload.find("\n")].strip()
    columns = NEW_COLUMNS if "obs_vx_cm_s" in header_line else COLUMNS
    payload = payload[payload.find("\n") + 1 :]
    starts = list(re.finditer(r"(?m)(?:^|\n)\s*(\d+),(\d+),", payload))
    rows: list[list[float]] = []

    for index, start in enumerate(starts):
        stop = starts[index + 1].start() if index + 1 < len(starts) else len(payload)
        chunk = payload[start.start() : stop]
        chunk = re.sub(r"\s+", "", chunk)
        first_digit = re.search(r"\d", chunk)
        if not first_digit:
            continue
        tokens = chunk[first_digit.start() :].split(",")
        if len(tokens) < len(columns):
            continue
        try:
            row = [parse_number(token) for token in tokens[: len(columns)]]
        except ValueError:
            continue
        rows.append(row)

    if len(rows) < 20:
        raise ValueError(f"only {len(rows)} records recovered")

    matrix = np.asarray(rows, dtype=float)
    order = np.argsort(matrix[:, 0])
    matrix = matrix[order]
    _, unique_indices = np.unique(matrix[:, 0], return_index=True)
    matrix = matrix[np.sort(unique_indices)]
    data = {name: matrix[:, i] for i, name in enumerate(columns)}
    if "obs_vx_cm_s" not in data:
        data["obs_vx_cm_s"] = data["flow_vx_cm_s"]
        data["obs_vy_cm_s"] = data["flow_vy_cm_s"]

    dt = np.diff(data["time_us"]) * 1e-6
    valid_dt = (dt > 0.005) & (dt < 0.2)
    if np.count_nonzero(valid_dt) < len(dt) * 0.9:
        raise ValueError("too many invalid/non-monotonic timestamps")
    return data


def replay(data: dict[str, np.ndarray], params: Params) -> tuple[np.ndarray, ...]:
    time_s = (data["time_us"] - data["time_us"][0]) * 1e-6
    obs_vx = data["obs_vx_cm_s"]
    obs_vy = data["obs_vy_cm_s"]
    obs_valid = data["obs_valid"] > 0.5
    roll_ok = np.abs(data["roll_deg"]) <= 25.0
    pitch_ok = np.abs(data["pitch_deg"]) <= 25.0

    x = float(data["flow_x_cm"][0])
    y = float(data["flow_y_cm"][0])
    vx = 0.0
    vy = 0.0
    p_x = p_y = 100.0
    p_vx = p_vy = 100.0

    out_x = np.empty(len(time_s))
    out_y = np.empty(len(time_s))
    out_vx = np.empty(len(time_s))
    out_vy = np.empty(len(time_s))
    used = np.zeros(len(time_s))
    rejected = np.zeros(len(time_s))
    innov_x = np.empty(len(time_s))
    innov_y = np.empty(len(time_s))

    median_dt = float(np.median(np.diff(time_s)))
    for i in range(len(time_s)):
        dt = median_dt if i == 0 else float(time_s[i] - time_s[i - 1])
        dt = min(max(dt, 0.005), 0.1)

        x += vx * dt
        y += vy * dt
        p_x += p_vx * dt * dt + params.q_pos
        p_y += p_vy * dt * dt + params.q_pos
        p_vx += params.q_vel * dt
        p_vy += params.q_vel * dt

        ix = float(obs_vx[i]) - vx
        iy = float(obs_vy[i]) - vy
        innov_x[i] = ix
        innov_y[i] = iy

        if obs_valid[i] and roll_ok[i] and pitch_ok[i]:
            if abs(ix) > params.vel_gate or abs(iy) > params.vel_gate:
                rejected[i] = 1.0
            else:
                kx = p_vx / (p_vx + params.r_vel)
                ky = p_vy / (p_vy + params.r_vel)
                vx += kx * ix
                vy += ky * iy
                p_vx *= 1.0 - kx
                p_vy *= 1.0 - ky
                used[i] = 1.0

        x = float(np.clip(x, -500.0, 500.0))
        y = float(np.clip(y, -500.0, 500.0))
        vx = float(np.clip(vx, -350.0, 350.0))
        vy = float(np.clip(vy, -350.0, 350.0))
        out_x[i] = x
        out_y[i] = y
        out_vx[i] = vx
        out_vy[i] = vy

    return out_x, out_y, out_vx, out_vy, used, rejected, innov_x, innov_y


def estimate_lag_ms(time_s: np.ndarray, estimate: np.ndarray, truth: np.ndarray) -> float:
    dt = float(np.median(np.diff(time_s)))
    estimate = estimate - np.mean(estimate)
    truth = truth - np.mean(truth)
    max_shift = max(1, int(round(0.5 / dt)))
    best_shift = 0
    best_score = -math.inf
    for shift in range(-max_shift, max_shift + 1):
        if shift < 0:
            a, b = estimate[-shift:], truth[:shift]
        elif shift > 0:
            a, b = estimate[:-shift], truth[shift:]
        else:
            a, b = estimate, truth
        if len(a) < 10:
            continue
        denom = np.linalg.norm(a) * np.linalg.norm(b)
        score = float(np.dot(a, b) / denom) if denom > 1e-9 else -math.inf
        if score > best_score:
            best_score = score
            best_shift = shift
    return -best_shift * dt * 1000.0


def metrics(
    data: dict[str, np.ndarray],
    x: np.ndarray,
    y: np.ndarray,
    vx: np.ndarray,
    vy: np.ndarray,
    used: np.ndarray | None = None,
    rejected: np.ndarray | None = None,
) -> tuple[float, float, float, float, float, float, float, float, float]:
    time_s = (data["time_us"] - data["time_us"][0]) * 1e-6
    flow_x = data["flow_x_cm"]
    flow_y = data["flow_y_cm"]
    flow_vx = data["flow_vx_cm_s"]
    flow_vy = data["flow_vy_cm_s"]

    trim = max(2, int(round(0.25 / np.median(np.diff(time_s)))))
    sl = slice(trim, -trim) if len(time_s) > 2 * trim else slice(None)

    pos_rmse = float(np.sqrt(np.mean((x[sl] - flow_x[sl]) ** 2 + (y[sl] - flow_y[sl]) ** 2)))
    vel_rmse = float(np.sqrt(np.mean((vx[sl] - flow_vx[sl]) ** 2 + (vy[sl] - flow_vy[sl]) ** 2)))
    lag_x = estimate_lag_ms(time_s[sl], x[sl], flow_x[sl])
    lag_y = estimate_lag_ms(time_s[sl], y[sl], flow_y[sl])

    speed = np.hypot(flow_vx, flow_vy)
    quiet = speed < 2.0
    quiet_vel = float(np.sqrt(np.mean(vx[quiet] ** 2 + vy[quiet] ** 2))) if np.any(quiet) else 0.0
    closure = float(np.hypot(x[-1] - x[0], y[-1] - y[0]))
    used_rate = float(np.mean(used)) if used is not None else float(np.mean(data["ekf_used"] > 0.5))
    reject_rate = float(np.mean(rejected)) if rejected is not None else float(np.mean(data["gate_reject"] > 0.5))

    cost = (
        pos_rmse
        + 0.12 * vel_rmse
        + 0.010 * (abs(lag_x) + abs(lag_y))
        + 0.20 * quiet_vel
        + 0.08 * closure
        + 20.0 * reject_rate
        + 8.0 * max(0.0, 0.75 - used_rate)
    )
    return cost, pos_rmse, vel_rmse, lag_x, lag_y, quiet_vel, closure, used_rate, reject_rate


def parameter_grid() -> list[Params]:
    values = {
        "q_pos": (1.0, 3.0, 5.0, 10.0, 20.0),
        "q_vel": (40.0, 80.0, 120.0, 200.0, 350.0),
        "r_vel": (300.0, 600.0, 900.0, 1400.0, 2200.0),
        "vel_gate": (90.0, 140.0, 180.0, 240.0),
    }
    keys = tuple(values)
    return [
        Params(**dict(zip(keys, combination)))
        for combination in itertools.product(*(values[key] for key in keys))
    ]


def write_series(
    path: Path,
    data: dict[str, np.ndarray],
    replay_out: tuple[np.ndarray, ...],
) -> None:
    x, y, vx, vy, used, rejected, innov_x, innov_y = replay_out
    time_s = (data["time_us"] - data["time_us"][0]) * 1e-6
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow((
            "time_s", "flow_x_cm", "flow_y_cm", "flow_vx_cm_s", "flow_vy_cm_s",
            "obs_vx_cm_s", "obs_vy_cm_s",
            "logged_ekf_x_cm", "logged_ekf_y_cm", "logged_ekf_vx_cm_s", "logged_ekf_vy_cm_s",
            "replay_x_cm", "replay_y_cm", "replay_vx_cm_s", "replay_vy_cm_s",
            "replay_used", "replay_rejected", "replay_innov_x_cm_s", "replay_innov_y_cm_s",
        ))
        for row in zip(
            time_s,
            data["flow_x_cm"], data["flow_y_cm"], data["flow_vx_cm_s"], data["flow_vy_cm_s"],
            data["obs_vx_cm_s"], data["obs_vy_cm_s"],
            data["ekf_x_cm"], data["ekf_y_cm"], data["ekf_vx_cm_s"], data["ekf_vy_cm_s"],
            x, y, vx, vy, used, rejected, innov_x, innov_y,
        ):
            writer.writerow((f"{value:.6f}" for value in row))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("log", type=Path)
    parser.add_argument("--out-dir", type=Path, default=Path("project/tools/loc_ekf_tuner_out"))
    parser.add_argument("--top", type=int, default=20)
    parser.add_argument("--no-scan", action="store_true")
    args = parser.parse_args()

    data = parse_loc_log(args.log)
    args.out_dir.mkdir(parents=True, exist_ok=True)
    time_s = (data["time_us"] - data["time_us"][0]) * 1e-6

    logged = metrics(
        data,
        data["ekf_x_cm"], data["ekf_y_cm"],
        data["ekf_vx_cm_s"], data["ekf_vy_cm_s"],
    )
    baseline = Params()
    baseline_out = replay(data, baseline)
    baseline_metrics = metrics(data, *baseline_out[:6])

    print(f"samples={len(time_s)} duration={time_s[-1]:.3f}s median_dt={np.median(np.diff(time_s))*1000:.2f}ms")
    print(f"flow_delta=({data['flow_x_cm'][-1] - data['flow_x_cm'][0]:.2f}, {data['flow_y_cm'][-1] - data['flow_y_cm'][0]:.2f})cm")
    print(f"logged_delta=({data['ekf_x_cm'][-1] - data['ekf_x_cm'][0]:.2f}, {data['ekf_y_cm'][-1] - data['ekf_y_cm'][0]:.2f})cm")
    print(
        "logged : cost={:.3f} pos_rmse={:.3f} vel_rmse={:.3f} "
        "lag=({:.1f},{:.1f})ms quiet_v={:.3f} closure={:.3f} used={:.2%} reject={:.2%}".format(*logged)
    )
    print(
        "replay : cost={:.3f} pos_rmse={:.3f} vel_rmse={:.3f} "
        "lag=({:.1f},{:.1f})ms quiet_v={:.3f} closure={:.3f} used={:.2%} reject={:.2%}".format(*baseline_metrics)
    )

    write_series(args.out_dir / "baseline_series.csv", data, baseline_out)

    if args.no_scan:
        return 0

    ranked: list[tuple[float, float, float, float, float, float, float, float, float, Params]] = []
    for params in parameter_grid():
        out = replay(data, params)
        ranked.append((*metrics(data, *out[:6]), params))
    ranked.sort(key=lambda item: item[0])

    results_path = args.out_dir / "grid_results.csv"
    with results_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow((
            "cost", "pos_rmse", "vel_rmse", "lag_x_ms", "lag_y_ms", "quiet_vel_rms",
            "closure_cm", "used_rate", "reject_rate", "q_pos", "q_vel", "r_vel", "vel_gate",
        ))
        for row in ranked:
            writer.writerow((*row[:-1], row[-1].q_pos, row[-1].q_vel, row[-1].r_vel, row[-1].vel_gate))

    print("\nTop candidates:")
    for index, item in enumerate(ranked[: args.top], start=1):
        cost, pos_rmse, vel_rmse, lag_x, lag_y, quiet_vel, closure, used_rate, reject_rate, params = item
        print(
            f"{index:2d} cost={cost:.3f} pos={pos_rmse:.3f} vel={vel_rmse:.3f} "
            f"lag=({lag_x:.1f},{lag_y:.1f})ms quiet_v={quiet_vel:.3f} "
            f"closure={closure:.3f} used={used_rate:.2%} reject={reject_rate:.2%} {params}"
        )

    best = ranked[0][-1]
    best_out = replay(data, best)
    write_series(args.out_dir / "best_series.csv", data, best_out)
    print(f"\nWrote {results_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
