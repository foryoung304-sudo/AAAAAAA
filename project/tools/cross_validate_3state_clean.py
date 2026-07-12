#!/usr/bin/env python3
"""Clean 2-state vs 3-state validation using explicit original log paths."""

from __future__ import annotations

import argparse
import hashlib
from pathlib import Path

import numpy as np

from ekf_tuner_3state import (
    Params,
    local_poly_truth,
    parse_height_log,
    replay_2state,
    replay_3state,
    score,
)


def identity(path: Path, data: dict[str, np.ndarray]) -> str:
    digest = hashlib.sha256(path.read_bytes()).hexdigest()[:16]
    return (
        f"sha256={digest} samples={len(data['time_us'])} "
        f"time={int(data['time_us'][0])}->{int(data['time_us'][-1])} "
        f"height={data['height_cm'][0]:.2f}->{data['height_cm'][-1]:.2f}"
    )


def metrics(data, params, three_state: bool, window: float):
    time_s = (data["time_us"] - data["time_us"][0]) * 1e-6
    _, truth_vz = local_poly_truth(time_s, data["height_cm"], window, 3)
    if three_state:
        _, vz, bias = replay_3state(data, params)
    else:
        _, vz = replay_2state(data, params)
        bias = np.zeros_like(vz)
    return score(time_s, vz, truth_vz), bias


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("logs", type=Path, nargs="+")
    args = parser.parse_args()

    middle = Params(
        acc_gain=0.20, acc_alpha=0.18, q_z=25.0, q_vz=400.0,
        q_bias=0.0, r_z=20.0, vel_obs=1, vel_alpha=0.45, r_vz=250.0,
    )
    same_3 = Params(**{**middle.__dict__, "q_bias": 10.0})
    gemini_3 = Params(
        acc_gain=0.20, acc_alpha=0.12, q_z=15.0, q_vz=600.0,
        q_bias=10.0, r_z=12.5, vel_obs=1, vel_alpha=0.55, r_vz=150.0,
    )
    gemini_2 = Params(**{**gemini_3.__dict__, "q_bias": 0.0})

    for path in args.logs:
        data = parse_height_log(path)
        print(f"\n{path}")
        print(identity(path, data))
        for window in (0.40, 0.55, 0.70):
            m2, _ = metrics(data, middle, False, window)
            m3_same, bias_same = metrics(data, same_3, True, window)
            m2_gem, _ = metrics(data, gemini_2, False, window)
            m3_gem, bias_gem = metrics(data, gemini_3, True, window)
            print(
                f"w={window:.2f} "
                f"2S rmse={m2[1]:.3f} lag={m2[2]:.0f} quiet={m2[3]:.3f} | "
                f"3S-only rmse={m3_same[1]:.3f} lag={m3_same[2]:.0f} quiet={m3_same[3]:.3f} "
                f"bias_std={np.std(bias_same):.2f} | "
                f"2S-gem rmse={m2_gem[1]:.3f} lag={m2_gem[2]:.0f} quiet={m2_gem[3]:.3f} | "
                f"3S-gem rmse={m3_gem[1]:.3f} lag={m3_gem[2]:.0f} quiet={m3_gem[3]:.3f} "
                f"bias_std={np.std(bias_gem):.2f}"
            )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
