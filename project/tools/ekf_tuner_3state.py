#!/usr/bin/env python3
"""Offline vertical EKF replay/tuner for 3-state evaluation.

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
    "seq", "time_us", "tof_cm", "height_cm", "ekf_z_cm", "ekf_vz_cm_s",
    "acc_raw_cm_s2", "acc_corrected_cm_s2", "acc_lpf_cm_s2", "acc_ekf_cm_s2",
    "tof_diff_cm_s", "innov_cm", "throttle", "target_vz_cm_s", "tof_used", "armed",
)

@dataclass(frozen=True)
class Params:
    acc_gain: float = 0.20
    acc_alpha: float = 0.12
    q_z: float = 40.0
    q_vz: float = 300.0
    q_bias: float = 0.0  # Used in 3-state
    r_z: float = 25.0
    vel_obs: int = 1
    vel_alpha: float = 0.35
    r_vz: float = 350.0

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

def replay_2state(data: dict[str, np.ndarray], params: Params) -> tuple[np.ndarray, np.ndarray]:
    time_s = (data["time_us"] - data["time_us"][0]) * 1e-6
    z_meas = data["height_cm"]
    acc_raw = data["acc_raw_cm_s2"]
    tof_used = data["tof_used"]

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

        if tof_used[i] > 0.5:
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


def replay_3state(data: dict[str, np.ndarray], params: Params) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    time_s = (data["time_us"] - data["time_us"][0]) * 1e-6
    z_meas = data["height_cm"]
    acc_raw = data["acc_raw_cm_s2"]
    tof_used = data["tof_used"]

    X = np.array([float(z_meas[0]), 0.0, 0.0], dtype=float)
    P = np.array([
        [100.0, 0.0, 0.0],
        [0.0, 100.0, 0.0],
        [0.0, 0.0, 100.0]
    ], dtype=float)

    acc_lpf = float(acc_raw[0])
    tof_vel_lpf = 0.0
    last_z = float(z_meas[0])
    
    out_z = np.empty(len(z_meas))
    out_vz = np.empty(len(z_meas))
    out_bias = np.empty(len(z_meas))

    for i in range(len(z_meas)):
        if i == 0:
            dt = float(np.median(np.diff(time_s)))
        else:
            dt = float(time_s[i] - time_s[i - 1])
        dt = min(max(dt, 0.005), 0.1)

        acc_lpf += params.acc_alpha * (float(acc_raw[i]) - acc_lpf)
        acc_c = np.clip(acc_lpf, -300.0, 300.0) * params.acc_gain

        F = np.array([
            [1.0, dt, -0.5 * dt**2],
            [0.0, 1.0, -dt],
            [0.0, 0.0, 1.0]
        ], dtype=float)
        
        Q = np.array([
            [params.q_z * dt, 0.0, 0.0],
            [0.0, params.q_vz * dt, 0.0],
            [0.0, 0.0, params.q_bias * dt]
        ], dtype=float)
        
        X[0] = X[0] + X[1] * dt + 0.5 * (acc_c - X[2]) * dt**2
        X[1] = X[1] + (acc_c - X[2]) * dt
        # X[2] remains unchanged
        
        P = F @ P @ F.T + Q

        if tof_used[i] > 0.5:
            H_z = np.array([[1.0, 0.0, 0.0]], dtype=float)
            innov_z = float(z_meas[i]) - X[0]
            S_z = (H_z @ P @ H_z.T)[0, 0] + params.r_z
            K_z = (P @ H_z.T) / S_z
            X = X + (K_z * innov_z).flatten()
            P = (np.eye(3) - K_z @ H_z) @ P

            if params.vel_obs and i > 0:
                tof_vel_raw = np.clip((float(z_meas[i]) - last_z) / dt, -80.0, 80.0)
                tof_vel_lpf += params.vel_alpha * (tof_vel_raw - tof_vel_lpf)
                vel_innov = tof_vel_lpf - X[1]
                H_v = np.array([[0.0, 1.0, 0.0]], dtype=float)
                S_v = (H_v @ P @ H_v.T)[0, 0] + params.r_vz
                K_v = (P @ H_v.T) / S_v
                X = X + (K_v * vel_innov).flatten()
                P = (np.eye(3) - K_v @ H_v) @ P

        last_z = float(z_meas[i])
        
        X[0] = float(np.clip(X[0], 0.0, 220.0))
        X[1] = float(np.clip(X[1], -200.0, 200.0))
        X[2] = float(np.clip(X[2], -200.0, 200.0))
        
        out_z[i] = X[0]
        out_vz[i] = X[1]
        out_bias[i] = X[2]

    return out_z, out_vz, out_bias

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

def parameter_grid_3state() -> list[Params]:
    values = {
        "acc_gain": (0.20,), # Fixed per user request for strict comparison
        "acc_alpha": (0.08, 0.12, 0.22, 0.35),
        "q_z": (15.0, 40.0, 80.0),
        "q_vz": (100.0, 300.0, 600.0),
        "q_bias": (0.5, 2.0, 5.0, 10.0),
        "r_z": (12.5, 25.0, 50.0),
        "vel_obs": (1,),
        "vel_alpha": (0.25, 0.35, 0.55),
        "r_vz": (150.0, 350.0, 700.0),
    }
    keys = tuple(values)
    return [
        Params(**dict(zip(keys, combination)))
        for combination in itertools.product(*(values[key] for key in keys))
    ]

def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("log", type=Path)
    args = parser.parse_args()

    data = parse_height_log(args.log)
    time_s = (data["time_us"] - data["time_us"][0]) * 1e-6
    truth_height, truth_vz = local_poly_truth(
        time_s, data["height_cm"], window_s=0.55, poly_order=3
    )

    baseline = Params()
    _, baseline_vz = replay_2state(data, baseline)
    logged_metrics = score(time_s, data["ekf_vz_cm_s"], truth_vz)
    baseline_metrics = score(time_s, baseline_vz, truth_vz)

    print(f"Log: {args.log.name}")
    print(f"samples={len(time_s)} duration={time_s[-1]:.3f}s")
    print("logged (2-state): cost={:.3f} rmse={:.3f} lag={:.1f}ms quiet={:.3f}".format(*logged_metrics))
    print("replay (2-state): cost={:.3f} rmse={:.3f} lag={:.1f}ms quiet={:.3f}".format(*baseline_metrics))

    print("\nRunning 3-state grid search...")
    ranked: list[tuple[float, float, float, float, Params]] = []
    for params in parameter_grid_3state():
        _, vz, _ = replay_3state(data, params)
        metrics = score(time_s, vz, truth_vz)
        ranked.append((*metrics, params))
    ranked.sort(key=lambda item: item[0])

    print("\nTop 5 3-state candidates:")
    for index, item in enumerate(ranked[:5], start=1):
        cost_value, rmse, lag_ms, drift, params = item
        print(
            f"{index:2d} cost={cost_value:.3f} rmse={rmse:.3f} "
            f"lag={lag_ms:.1f}ms quiet={drift:.3f} q_bias={params.q_bias} "
            f"q_vz={params.q_vz} acc_alpha={params.acc_alpha}"
        )

    return 0

if __name__ == "__main__":
    raise SystemExit(main())
