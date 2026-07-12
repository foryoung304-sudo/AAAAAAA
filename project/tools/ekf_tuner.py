#!/usr/bin/env python3
"""Offline vertical EKF replay/tuner for HEIGHT_LOG CSV captures.

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
    "seq",
    "time_us",
    "tof_cm",
    "height_cm",
    "ekf_z_cm",
    "ekf_vz_cm_s",
    "acc_raw_cm_s2",
    "acc_corrected_cm_s2",
    "acc_lpf_cm_s2",
    "acc_ekf_cm_s2",
    "tof_diff_cm_s",
    "innov_cm",
    "throttle",
    "target_vz_cm_s",
    "tof_used",
    "armed",
)


@dataclass(frozen=True)
class Params:
    acc_gain: float = 0.20
    acc_alpha: float = 0.12
    q_z: float = 40.0
    q_vz: float = 300.0
    r_z: float = 25.0
    vel_obs: int = 1
    vel_alpha: float = 0.35
    r_vz: float = 350.0


FIRMWARE_CURRENT = Params(
    acc_gain=0.20,
    acc_alpha=0.18,
    q_z=25.0,
    q_vz=400.0,
    r_z=25.0,
    vel_obs=1,
    vel_alpha=0.55,
    r_vz=200.0,
)


def parse_number(token: str) -> float:
    match = re.match(r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)", token.strip())
    if not match:
        raise ValueError(f"not numeric: {token!r}")
    return float(match.group(0))


def parse_height_log(path: Path) -> dict[str, np.ndarray]:
    text = path.read_text(encoding="utf-8", errors="ignore")
    begin = text.find("HEIGHT_LOG_BEGIN")
    end = text.find("HEIGHT_LOG_END", begin)
    if begin < 0 or end < 0:
        raise ValueError("missing HEIGHT_LOG_BEGIN/HEIGHT_LOG_END")

    block = text[begin:end]
    header_pos = block.find("seq,time_us")
    if header_pos < 0:
        raise ValueError("missing HEIGHT_LOG CSV header")

    payload = block[header_pos:]
    payload = payload[payload.find("\n") + 1 :]
    # Wireless transport may wrap records. Record boundaries remain seq,time_us.
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
        if len(tokens) < len(COLUMNS):
            continue
        try:
            row = [parse_number(token) for token in tokens[: len(COLUMNS)]]
        except ValueError:
            continue
        if int(row[0]) != len(rows):
            # Keep valid non-contiguous rows; continuity checked below.
            pass
        rows.append(row)

    if len(rows) < 20:
        raise ValueError(f"only {len(rows)} records recovered")

    matrix = np.asarray(rows, dtype=float)
    order = np.argsort(matrix[:, 0])
    matrix = matrix[order]
    _, unique_indices = np.unique(matrix[:, 0], return_index=True)
    matrix = matrix[np.sort(unique_indices)]
    data = {name: matrix[:, i] for i, name in enumerate(COLUMNS)}

    dt = np.diff(data["time_us"]) * 1e-6
    valid_dt = (dt > 0.005) & (dt < 0.2)
    if np.count_nonzero(valid_dt) < len(dt) * 0.9:
        raise ValueError("too many invalid/non-monotonic timestamps")
    return data


def local_poly_truth(
    time_s: np.ndarray,
    height_cm: np.ndarray,
    window_s: float,
    poly_order: int,
) -> tuple[np.ndarray, np.ndarray]:
    """Zero-phase local polynomial height and derivative on nonuniform timestamps."""
    n = len(time_s)
    smooth = np.empty(n)
    velocity = np.empty(n)
    half = window_s * 0.5

    for i in range(n):
        mask = np.abs(time_s - time_s[i]) <= half
        indices = np.flatnonzero(mask)
        minimum = poly_order + 2
        if len(indices) < minimum:
            nearest = np.argsort(np.abs(time_s - time_s[i]))[:minimum]
            indices = np.sort(nearest)
        x = time_s[indices] - time_s[i]
        y = height_cm[indices]
        degree = min(poly_order, len(indices) - 1)
        coeff = np.polynomial.polynomial.polyfit(x, y, degree)
        smooth[i] = coeff[0]
        velocity[i] = coeff[1] if len(coeff) > 1 else 0.0
    return smooth, velocity


def replay(data: dict[str, np.ndarray], params: Params) -> tuple[np.ndarray, np.ndarray]:
    time_s = (data["time_us"] - data["time_us"][0]) * 1e-6
    z_meas = data["height_cm"]
    acc_raw = data["acc_corrected_cm_s2"]

    z = float(z_meas[0])
    vz = 0.0
    p00 = p11 = 100.0
    p01 = p10 = 0.0
    acc_lpf = float(acc_raw[0])
    tof_vel_lpf = 0.0
    last_z = float(z_meas[0])
    out_z = np.empty(len(z_meas))
    out_vz = np.empty(len(z_meas))

    for i in range(len(z_meas)):
        if i == 0:
            dt = float(np.median(np.diff(time_s)))
        else:
            dt = float(time_s[i] - time_s[i - 1])
        dt = min(max(dt, 0.005), 0.1)

        acc_lpf += params.acc_alpha * (float(acc_raw[i]) - acc_lpf)
        acc = np.clip(acc_lpf, -300.0, 300.0) * params.acc_gain

        z += vz * dt + 0.5 * acc * dt * dt
        vz += acc * dt
        p00 += p11 * dt * dt + params.q_z * dt
        p01 += p11 * dt
        p10 = p01
        p11 += params.q_vz * dt

        innovation = float(z_meas[i]) - z
        s = p00 + params.r_z
        k0 = p00 / s
        k1 = p10 / s
        old = p00, p01, p10, p11
        z += k0 * innovation
        vz += k1 * innovation
        p00 = old[0] - k0 * old[0]
        p01 = old[1] - k0 * old[1]
        p10 = old[2] - k1 * old[0]
        p11 = old[3] - k1 * old[1]

        if params.vel_obs and i > 0:
            tof_vel_raw = np.clip((float(z_meas[i]) - last_z) / dt, -80.0, 80.0)
            tof_vel_lpf += params.vel_alpha * (tof_vel_raw - tof_vel_lpf)
            vel_innovation = tof_vel_lpf - vz
            vel_s = p11 + params.r_vz
            old = p00, p01, p10, p11
            kv0 = p01 / vel_s
            kv1 = p11 / vel_s
            z += kv0 * vel_innovation
            vz += kv1 * vel_innovation
            p00 = old[0] - kv0 * old[2]
            p01 = old[1] - kv0 * old[3]
            p10 = old[2] - kv1 * old[2]
            p11 = old[3] - kv1 * old[3]

        last_z = float(z_meas[i])
        z = float(np.clip(z, 0.0, 220.0))
        vz = float(np.clip(vz, -200.0, 200.0))
        out_z[i] = z
        out_vz[i] = vz

    return out_z, out_vz


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
    # Positive means estimate occurs later than truth.
    return -best_shift * dt * 1000.0


def score(
    time_s: np.ndarray, estimate: np.ndarray, truth: np.ndarray
) -> tuple[float, float, float, float]:
    trim = max(2, int(round(0.25 / np.median(np.diff(time_s)))))
    e = estimate[trim:-trim] if len(estimate) > 2 * trim else estimate
    t = truth[trim:-trim] if len(truth) > 2 * trim else truth
    tt = time_s[trim:-trim] if len(time_s) > 2 * trim else time_s
    rmse = float(np.sqrt(np.mean((e - t) ** 2)))
    lag_ms = estimate_lag_ms(tt, e, t)
    quiet = np.abs(t) < 1.0
    drift = float(np.sqrt(np.mean(e[quiet] ** 2))) if np.any(quiet) else 0.0
    cost = rmse + 0.015 * abs(lag_ms) + 0.35 * drift
    return cost, rmse, lag_ms, drift


def parameter_grid() -> list[Params]:
    values = {
        "acc_gain": (0.10, 0.20, 0.35),
        "acc_alpha": (0.12, 0.18, 0.35),
        "q_z": (15.0, 25.0, 40.0),
        "q_vz": (300.0, 400.0, 600.0),
        "r_z": (12.5, 25.0),
        "vel_obs": (1,),
        "vel_alpha": (0.35, 0.55),
        "r_vz": (150.0, 200.0, 350.0),
    }
    keys = tuple(values)
    grid = [
        Params(**dict(zip(keys, combination)))
        for combination in itertools.product(*(values[key] for key in keys))
    ]
    if FIRMWARE_CURRENT not in grid:
        grid.append(FIRMWARE_CURRENT)
    return grid


def format_metrics(metrics: tuple[float, float, float, float]) -> str:
    return "cost={:.3f} rmse={:.3f} lag={:.1f}ms quiet={:.3f}".format(*metrics)


def write_series(
    path: Path,
    data: dict[str, np.ndarray],
    truth_height: np.ndarray,
    truth_vz: np.ndarray,
    replay_z: np.ndarray,
    replay_vz: np.ndarray,
) -> None:
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            ("time_s", "height_cm", "truth_height_cm", "logged_vz", "truth_vz", "replay_z", "replay_vz")
        )
        time_s = (data["time_us"] - data["time_us"][0]) * 1e-6
        for row in zip(
            time_s,
            data["height_cm"],
            truth_height,
            data["ekf_vz_cm_s"],
            truth_vz,
            replay_z,
            replay_vz,
        ):
            writer.writerow((f"{value:.6f}" for value in row))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("log", type=Path)
    parser.add_argument("--out-dir", type=Path, default=Path("project/tools/ekf_tuner_out"))
    parser.add_argument("--window", type=float, default=0.55, help="truth smoothing window, seconds")
    parser.add_argument("--poly", type=int, default=3)
    parser.add_argument("--top", type=int, default=20)
    parser.add_argument("--no-scan", action="store_true")
    args = parser.parse_args()

    data = parse_height_log(args.log)
    args.out_dir.mkdir(parents=True, exist_ok=True)
    time_s = (data["time_us"] - data["time_us"][0]) * 1e-6
    truth_height, truth_vz = local_poly_truth(
        time_s, data["height_cm"], args.window, args.poly
    )

    baseline = Params()
    baseline_z, baseline_vz = replay(data, baseline)
    firmware_z, firmware_vz = replay(data, FIRMWARE_CURRENT)
    logged_metrics = score(time_s, data["ekf_vz_cm_s"], truth_vz)
    baseline_metrics = score(time_s, baseline_vz, truth_vz)
    firmware_metrics = score(time_s, firmware_vz, truth_vz)

    print(f"samples={len(time_s)} duration={time_s[-1]:.3f}s median_dt={np.median(np.diff(time_s))*1000:.2f}ms")
    print(f"logged           : {format_metrics(logged_metrics)}")
    print(f"baseline replay  : {format_metrics(baseline_metrics)} {baseline}")
    print(f"firmware replay  : {format_metrics(firmware_metrics)} {FIRMWARE_CURRENT}")

    write_series(
        args.out_dir / "baseline_series.csv",
        data,
        truth_height,
        truth_vz,
        baseline_z,
        baseline_vz,
    )

    if args.no_scan:
        return 0

    ranked: list[tuple[float, float, float, float, Params]] = []
    for params in parameter_grid():
        _, vz = replay(data, params)
        metrics = score(time_s, vz, truth_vz)
        ranked.append((*metrics, params))
    ranked.sort(key=lambda item: item[0])
    firmware_rank = next(
        (
            index
            for index, item in enumerate(ranked, start=1)
            if item[4] == FIRMWARE_CURRENT
        ),
        None,
    )

    results_path = args.out_dir / "grid_results.csv"
    with results_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            (
                "cost", "rmse", "lag_ms", "quiet_rms", "acc_gain", "acc_alpha",
                "q_z", "q_vz", "r_z", "vel_obs", "vel_alpha", "r_vz",
            )
        )
        for cost_value, rmse, lag_ms, drift, params in ranked:
            writer.writerow(
                (
                    cost_value, rmse, lag_ms, drift, params.acc_gain,
                    params.acc_alpha, params.q_z, params.q_vz, params.r_z,
                    params.vel_obs, params.vel_alpha, params.r_vz,
                )
            )

    print("\nTop candidates:")
    for index, item in enumerate(ranked[: args.top], start=1):
        cost_value, rmse, lag_ms, drift, params = item
        label = " <- firmware_current" if params == FIRMWARE_CURRENT else ""
        print(
            f"{index:2d} cost={cost_value:.3f} rmse={rmse:.3f} "
            f"lag={lag_ms:.1f}ms quiet={drift:.3f} {params}{label}"
        )

    if firmware_rank is not None and firmware_rank > args.top:
        cost_value, rmse, lag_ms, drift, params = ranked[firmware_rank - 1]
        print(
            f"\nfirmware_current rank={firmware_rank}/{len(ranked)} "
            f"cost={cost_value:.3f} rmse={rmse:.3f} "
            f"lag={lag_ms:.1f}ms quiet={drift:.3f} {params}"
        )

    best = ranked[0][4]
    best_z, best_vz = replay(data, best)
    write_series(
        args.out_dir / "best_series.csv",
        data,
        truth_height,
        truth_vz,
        best_z,
        best_vz,
    )
    print(f"\nWrote {results_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
