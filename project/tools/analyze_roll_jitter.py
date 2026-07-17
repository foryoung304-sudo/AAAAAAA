import math
import sys

from analyze_history_compact import corr, load_detailed, mean


def rms(values):
    return math.sqrt(mean([value * value for value in values])) if values else float("nan")


def diffs(values):
    return [values[i] - values[i - 1] for i in range(1, len(values))]


def large_error_boost(error, gain):
    excess = abs(error) - 0.5
    if excess <= 0.0:
        return 0.0
    blend = 1.0
    if excess < 0.5:
        t = excess / 0.5
        blend = t * t * (3.0 - 2.0 * t)
    boost = excess * gain * blend
    if error < 0.0:
        boost = -boost
    return max(-3.0, min(3.0, boost))


def report(name, rows, gain):
    if len(rows) < 3:
        print(f"{name}: insufficient rows")
        return

    rate_target = [row["rate_tgt_r"] for row in rows]
    rate_feedback = [row["rate_fb_r"] for row in rows]
    feedforward = [row["sp_rate_ff_roll"] for row in rows]
    angle_error = [row["roll_tgt"] - row["roll_cur"] for row in rows]
    boost = [large_error_boost(error, gain) for error in angle_error]
    rate_error = [target - feedback for target, feedback in zip(rate_target, rate_feedback)]
    rate_p = [row["rate_p_r"] for row in rows]
    rate_i = [row["rate_i_r"] for row in rows]
    rate_d = [row["rate_d_r"] for row in rows]
    rate_out = [row["rate_out_r"] for row in rows]

    d_target = diffs(rate_target)
    d_feedback = diffs(rate_feedback)
    d_feedforward = diffs(feedforward)
    d_boost = diffs(boost)
    active_transitions = sum(
        (boost[i] == 0.0) != (boost[i - 1] == 0.0)
        for i in range(1, len(boost))
    )

    print(f"[{name}] rows={len(rows)}")
    print(
        f"  rate target/fb rms={rms(rate_target):.2f}/{rms(rate_feedback):.2f} dps "
        f"tracking-error-rms={rms(rate_error):.2f} dps"
    )
    print(
        f"  frame-diff rms: target={rms(d_target):.2f} fb={rms(d_feedback):.2f} "
        f"ff={rms(d_feedforward):.2f} boost={rms(d_boost):.2f} dps/frame"
    )
    print(
        f"  PID/out rms: P={rms(rate_p):.3f} I={rms(rate_i):.3f} "
        f"D={rms(rate_d):.3f} out={rms(rate_out):.3f}; "
        f"D frame-diff rms={rms(diffs(rate_d)):.3f}"
    )
    print(
        f"  corr(diff target, diff ff/boost)="
        f"{corr(d_target, d_feedforward):.3f}/{corr(d_target, d_boost):.3f} "
        f"boost-active={sum(value != 0.0 for value in boost) / len(boost) * 100:.1f}% "
        f"boost-boundary-crossings={active_transitions}"
    )


def main(path, gain):
    rows = [
        row
        for row in load_detailed(path)
        if row["loc_ready"] != 0.0 and row["loc_hold"] != 0.0
    ]
    low_speed = [
        row
        for row in rows
        if math.hypot(row["vel_x_e"], row["vel_y_e"]) <= 8.0
    ]
    high_speed = [
        row
        for row in rows
        if math.hypot(row["vel_x_e"], row["vel_y_e"]) > 8.0
    ]
    small_error = [
        row for row in rows
        if abs(row["roll_tgt"] - row["roll_cur"]) <= 0.5
    ]
    blend_error = [
        row for row in rows
        if 0.5 < abs(row["roll_tgt"] - row["roll_cur"]) <= 1.0
    ]
    large_error = [
        row for row in rows
        if abs(row["roll_tgt"] - row["roll_cur"]) > 1.0
    ]
    report("all hold", rows, gain)
    report("small angle error (<=0.5 deg)", small_error, gain)
    report("blend angle error (0.5..1.0 deg)", blend_error, gain)
    report("large angle error (>1.0 deg)", large_error, gain)
    report("low speed (new damping is zero)", low_speed, gain)
    report("high speed", high_speed, gain)


if __name__ == "__main__":
    main(sys.argv[1], float(sys.argv[2]) if len(sys.argv) > 2 else 2.0)
