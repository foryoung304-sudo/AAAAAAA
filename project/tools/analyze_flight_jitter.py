#!/usr/bin/env python3
"""Separate commanded flight motion from uncommanded attitude jitter."""

from __future__ import annotations

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))
from flight_log_replay import integer, load_rows, number


AXES = (
    ("roll", "target_roll_deg", "roll_deg", "rate_tgt_roll_dps", "rate_fb_roll_dps", "rate_out_roll"),
    ("pitch", "target_pitch_deg", "pitch_deg", "rate_tgt_pitch_dps", "rate_fb_pitch_dps", "rate_out_pitch"),
)


def finite(row: dict[str, str], field: str) -> float | None:
    value = number(row.get(field, ""))
    return value if math.isfinite(value) else None


def rms(values: list[float]) -> float:
    return math.sqrt(sum(value * value for value in values) / len(values)) if values else math.nan


def p95_abs(values: list[float]) -> float:
    if not values:
        return math.nan
    ordered = sorted(abs(value) for value in values)
    return ordered[min(len(ordered) - 1, int(0.95 * len(ordered)))]


def first_time(rows: list[dict[str, str]], predicate, consecutive: int = 1) -> float | None:
    if not rows:
        return None
    t0 = integer(rows[0].get("time_us", "0"))
    run = 0
    for row in rows:
        if predicate(row):
            run += 1
            if run >= consecutive:
                return (integer(row.get("time_us", "0")) - t0) / 1_000_000.0
        else:
            run = 0
    return None


def print_event_rows(rows: list[dict[str, str]]) -> None:
    indices = [
        index for index, row in enumerate(rows)
        if integer(row.get("flow_gate_clipped", "0")) != 0
    ]
    if not indices:
        return
    first = indices[0]
    print("\n[FIRST FLOW CLIP WINDOW]")
    print("t_s mode pos_xy goal_xy vel_xy flow_vel_xy tgt_rp actual_rp clip")
    for row in rows[max(0, first - 2):min(len(rows), first + 4)]:
        values = [finite(row, field) or 0.0 for field in (
            "pos_x_cm", "pos_y_cm", "goal_x_cm", "goal_y_cm",
            "vel_x_cm_s", "vel_y_cm_s", "flow_vel_x_cm_s", "flow_vel_y_cm_s",
            "target_roll_deg", "target_pitch_deg", "roll_deg", "pitch_deg",
        )]
        print(
            f"{integer(row.get('time_us', '0')) / 1_000_000.0:6.3f} "
            f"{integer(row.get('mode', '0'))} "
            f"({values[0]:5.1f},{values[1]:5.1f}) "
            f"({values[2]:5.1f},{values[3]:5.1f}) "
            f"({values[4]:5.1f},{values[5]:5.1f}) "
            f"({values[6]:6.1f},{values[7]:6.1f}) "
            f"({values[8]:5.1f},{values[9]:5.1f}) "
            f"({values[10]:5.1f},{values[11]:5.1f}) "
            f"{integer(row.get('flow_gate_clipped', '0'))}"
        )


def replay_horizontal_guard(rows: list[dict[str, str]]) -> None:
    active = False
    bad_frames = 0
    good_frames = 0
    transitions: list[tuple[float, str]] = []
    for row in rows:
        mode = integer(row.get("mode", "-1"))
        if mode not in (0, 1):
            active = False
            bad_frames = 0
            good_frames = 0
            continue
        speed = math.hypot(finite(row, "vel_x_cm_s") or 0.0,
                           finite(row, "vel_y_cm_s") or 0.0)
        clipped = integer(row.get("flow_gate_clipped", "0")) != 0
        if active:
            if integer(row.get("flow", "0")) != 0 and not clipped and speed <= 20.0:
                good_frames += 1
                if good_frames >= 25:
                    active = False
                    bad_frames = 0
                    good_frames = 0
                    transitions.append((integer(row.get("time_us", "0")) / 1e6, "CLEAR"))
            else:
                good_frames = 0
        else:
            bad_frames = bad_frames + 1 if clipped or speed >= 55.0 else 0
            if speed >= 90.0 or bad_frames >= 3:
                active = True
                good_frames = 0
                transitions.append((integer(row.get("time_us", "0")) / 1e6, "BRAKE"))
    print("\n[CURRENT HORIZONTAL GUARD REPLAY]")
    if transitions:
        for time_s, state in transitions:
            print(f"{time_s:.3f}s -> {state}")
    else:
        print("no guard transition")


def phase_stats(name: str, rows: list[dict[str, str]]) -> None:
    if not rows:
        return
    t0 = integer(rows[0].get("time_us", "0")) / 1_000_000.0
    t1 = integer(rows[-1].get("time_us", "0")) / 1_000_000.0
    print(f"\n[{name}] samples={len(rows)} time={t0:.3f}..{t1:.3f}s")
    for axis, target_field, actual_field, rate_target_field, rate_fb_field, output_field in AXES:
        targets = [v for row in rows if (v := finite(row, target_field)) is not None]
        actuals = [v for row in rows if (v := finite(row, actual_field)) is not None]
        errors = []
        rate_errors = []
        outputs = []
        for row in rows:
            target = finite(row, target_field)
            actual = finite(row, actual_field)
            rate_target = finite(row, rate_target_field)
            rate_fb = finite(row, rate_fb_field)
            output = finite(row, output_field)
            if target is not None and actual is not None:
                errors.append(target - actual)
            if rate_target is not None and rate_fb is not None:
                rate_errors.append(rate_target - rate_fb)
            if output is not None:
                outputs.append(output)
        print(
            f"{axis:5s}: target_rms={rms(targets):5.2f}deg "
            f"actual_rms={rms(actuals):5.2f}deg "
            f"att_err_p95={p95_abs(errors):5.2f}deg "
            f"rate_err_p95={p95_abs(rate_errors):6.2f}dps "
            f"rate_out_p95={p95_abs(outputs):5.2f}"
        )

    motor_spreads = []
    for row in rows:
        motors = [finite(row, field) for field in ("m1", "m2", "m3", "m4")]
        if all(value is not None for value in motors):
            motor_spreads.append(max(motors) - min(motors))
    print(f"motor_spread: rms={rms(motor_spreads):.0f} p95={p95_abs(motor_spreads):.0f}")


def main() -> int:
    if len(sys.argv) != 2:
        print("usage: analyze_flight_jitter.py <pasted-text.txt>")
        return 2
    rows = load_rows(Path(sys.argv[1]))
    if not rows:
        print("No intact MISSION_LOG rows found.")
        return 1

    mode0 = [row for row in rows if integer(row.get("mode", "-1")) == 0]
    mode1 = [row for row in rows if integer(row.get("mode", "-1")) == 1]
    before_beacon = [row for row in rows if integer(row.get("beacon", "0")) == 0]
    after_beacon = [row for row in rows if integer(row.get("beacon", "0")) != 0]
    fault = [row for row in rows if integer(row.get("flow_gate_clipped", "0")) != 0]

    phase_stats("ALL", rows)
    phase_stats("TAKEOFF_MODE_0", mode0)
    phase_stats("MISSION_MODE_1", mode1)
    phase_stats("BEFORE_BEACON", before_beacon)
    phase_stats("BEACON_VISIBLE", after_beacon)
    phase_stats("FLOW_CLIPPED", fault)
    print_event_rows(rows)
    replay_horizontal_guard(rows)

    t_command = first_time(
        rows,
        lambda row: max(abs(finite(row, "target_roll_deg") or 0.0),
                        abs(finite(row, "target_pitch_deg") or 0.0)) >= 3.0,
        consecutive=3,
    )
    t_actual = first_time(
        rows,
        lambda row: max(abs(finite(row, "roll_deg") or 0.0),
                        abs(finite(row, "pitch_deg") or 0.0)) >= 3.0,
        consecutive=3,
    )
    t_speed = first_time(
        rows,
        lambda row: math.hypot(finite(row, "vel_x_cm_s") or 0.0,
                               finite(row, "vel_y_cm_s") or 0.0) >= 30.0,
        consecutive=3,
    )
    t_clip = first_time(rows, lambda row: integer(row.get("flow_gate_clipped", "0")) != 0)
    t_tracking_error = first_time(
        rows,
        lambda row: max(
            abs((finite(row, "target_roll_deg") or 0.0) -
                (finite(row, "roll_deg") or 0.0)),
            abs((finite(row, "target_pitch_deg") or 0.0) -
                (finite(row, "pitch_deg") or 0.0)),
        ) >= 3.0,
        consecutive=3,
    )

    print("\n[ONSET FROM FIRST LOGGED SAMPLE]")
    print(f"horizontal_speed>=30cm/s: {t_speed}")
    print(f"commanded_attitude>=3deg: {t_command}")
    print(f"actual_attitude>=3deg:    {t_actual}")
    print(f"tracking_error>=3deg:     {t_tracking_error}")
    print(f"flow_gate_clipped:         {t_clip}")
    if (t_clip is not None and t_command is not None and
            (t_tracking_error is None or t_clip <= t_tracking_error)):
        print("classification: FLOW/LOC ESCALATION PRECEDES PERSISTENT ATTITUDE ERROR")
    elif t_command is not None and t_actual is not None and t_command <= t_actual:
        print("classification: COMMAND/ESTIMATOR ESCALATION PRECEDES LARGE ATTITUDE")
    elif t_tracking_error is not None:
        print("classification: UNCOMMANDED ATTITUDE PRECEDES LARGE TARGET; inspect airframe/IMU/motors")
    else:
        print("classification: no >=3deg attitude event in capture")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
