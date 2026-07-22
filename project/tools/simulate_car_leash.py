#!/usr/bin/env python3
"""Regression simulation for aircraft/Y-car mission waypoint ownership."""

from __future__ import annotations

import csv
import math
from dataclasses import dataclass
from pathlib import Path


DT_S = 0.02
FOLLOW_START_CM = 85.0
FOLLOW_STOP_CM = 60.0
FOLLOW_TARGET_STEP_CM = 8.0
FOLLOW_SPEED_LIMIT_CM_S = 22.0
SEARCH_SPEED_LIMIT_CM_S = 30.0
SEARCH_CENTER = (100.0, 150.0)
SEARCH_LOST_FRAMES = 3
SEARCH_REACQUIRE_FRAMES = 2


def distance(a: tuple[float, float], b: tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def move_toward(
    current: tuple[float, float],
    target: tuple[float, float],
    max_step: float,
) -> tuple[float, float]:
    dx = target[0] - current[0]
    dy = target[1] - current[1]
    length = math.hypot(dx, dy)
    if length <= max_step or length == 0.0:
        return target
    scale = max_step / length
    return current[0] + dx * scale, current[1] + dy * scale


@dataclass
class MissionState:
    drone: tuple[float, float] = (0.0, 0.0)
    drone_goal: tuple[float, float] = (0.0, 0.0)
    car: tuple[float, float] = (35.0, 0.0)
    beacon: tuple[float, float] = (180.0, 20.0)
    follow_active: bool = False
    search_active: bool = False
    lost_frames: int = 0
    found_frames: int = 0
    mode: str = "hold"


def update_search(state: MissionState, beacon_visible: bool) -> None:
    if beacon_visible:
        state.lost_frames = 0
        state.found_frames = min(255, state.found_frames + 1)
        if state.search_active and state.found_frames >= SEARCH_REACQUIRE_FRAMES:
            state.search_active = False
            state.drone_goal = state.drone
    else:
        state.found_frames = 0
        state.lost_frames = min(255, state.lost_frames + 1)
        if state.lost_frames >= SEARCH_LOST_FRAMES:
            state.search_active = True

    if state.search_active:
        state.drone_goal = SEARCH_CENTER


def update_car_leash(
    state: MissionState,
    beacon_visible: bool,
    car_visible: bool,
) -> None:
    # Search owns the aircraft goal. The leash must yield without overwriting it.
    if state.search_active:
        state.follow_active = False
        return

    car_distance = distance(state.drone, state.car)
    eligible = beacon_visible and car_visible
    if not eligible:
        if state.follow_active:
            state.follow_active = False
            state.drone_goal = state.drone
        return

    if not state.follow_active:
        if car_distance <= FOLLOW_START_CM:
            return
        state.follow_active = True
        first_step = min(
            FOLLOW_TARGET_STEP_CM,
            car_distance - FOLLOW_STOP_CM,
        )
        state.drone_goal = move_toward(state.drone, state.car, first_step)
        return

    if car_distance <= FOLLOW_STOP_CM:
        state.follow_active = False
        state.drone_goal = state.drone
        return

    # Desired point lies FOLLOW_STOP_CM from the car on the car-to-drone line.
    ratio = (car_distance - FOLLOW_STOP_CM) / car_distance
    desired = (
        state.drone[0] + (state.car[0] - state.drone[0]) * ratio,
        state.drone[1] + (state.car[1] - state.drone[1]) * ratio,
    )
    state.drone_goal = move_toward(
        state.drone_goal,
        desired,
        FOLLOW_TARGET_STEP_CM,
    )


def scenario_inputs(t_s: float) -> tuple[bool, bool]:
    # 0-8 s: normal chase; 8-10 s: beacon lost; after 10 s: reacquired.
    beacon_visible = not (8.0 <= t_s < 10.0)
    car_visible = True
    return beacon_visible, car_visible


def run_simulation(duration_s: float = 18.0) -> list[dict[str, object]]:
    state = MissionState()
    rows: list[dict[str, object]] = []
    steps = int(duration_s / DT_S)

    for step in range(steps):
        t_s = step * DT_S
        beacon_visible, car_visible = scenario_inputs(t_s)

        update_search(state, beacon_visible)
        update_car_leash(state, beacon_visible, car_visible)

        if state.search_active:
            state.mode = "search_center"
            drone_speed = SEARCH_SPEED_LIMIT_CM_S
            # During center return the car follows the aircraft projection.
            car_goal = state.drone
            car_speed = 24.0
        else:
            state.mode = "follow_car" if state.follow_active else "hold"
            drone_speed = (
                FOLLOW_SPEED_LIMIT_CM_S if state.follow_active else 15.0
            )
            car_goal = state.beacon
            car_speed = 28.0

        previous_drone = state.drone
        state.drone = move_toward(
            state.drone,
            state.drone_goal,
            drone_speed * DT_S,
        )
        state.car = move_toward(state.car, car_goal, car_speed * DT_S)
        actual_drone_speed = distance(previous_drone, state.drone) / DT_S

        rows.append(
            {
                "time_s": round(t_s, 3),
                "beacon_visible": int(beacon_visible),
                "search_active": int(state.search_active),
                "follow_active": int(state.follow_active),
                "mode": state.mode,
                "drone_x_cm": round(state.drone[0], 3),
                "drone_y_cm": round(state.drone[1], 3),
                "goal_x_cm": round(state.drone_goal[0], 3),
                "goal_y_cm": round(state.drone_goal[1], 3),
                "car_x_cm": round(state.car[0], 3),
                "car_y_cm": round(state.car[1], 3),
                "car_distance_cm": round(distance(state.drone, state.car), 3),
                "drone_speed_cm_s": round(actual_drone_speed, 3),
            }
        )
    return rows


def verify(rows: list[dict[str, object]]) -> list[str]:
    failures: list[str] = []
    follow_rows = [row for row in rows if row["follow_active"] == 1]
    search_rows = [row for row in rows if row["search_active"] == 1]
    reacquired_rows = [
        row for row in rows
        if float(row["time_s"]) >= 10.1 and row["search_active"] == 0
    ]

    if not follow_rows:
        failures.append("aircraft leash never activated")
    if not search_rows:
        failures.append("lost beacon never activated center return")
    if any(row["follow_active"] == 1 for row in search_rows):
        failures.append("car leash overrode center-return ownership")
    if any(
        float(row["drone_speed_cm_s"]) > FOLLOW_SPEED_LIMIT_CM_S + 1e-6
        for row in follow_rows
    ):
        failures.append("car-follow speed exceeded configured limit")
    if not reacquired_rows:
        failures.append("center return did not exit after beacon reacquisition")
    if search_rows and any(
        (float(row["goal_x_cm"]), float(row["goal_y_cm"])) != SEARCH_CENTER
        for row in search_rows
    ):
        failures.append("center-return goal was overwritten")
    if follow_rows:
        first_follow = follow_rows[0]
        first_goal_step = math.hypot(
            float(first_follow["goal_x_cm"]) - float(first_follow["drone_x_cm"]),
            float(first_follow["goal_y_cm"]) - float(first_follow["drone_y_cm"]),
        )
        if first_goal_step > FOLLOW_TARGET_STEP_CM + 0.5:
            failures.append("first follow target jumped by more than 8 cm")

    return failures


def main() -> int:
    rows = run_simulation()
    output_path = Path(__file__).with_name("car_leash_simulation.csv")
    with output_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    failures = verify(rows)
    minimum_distance = min(float(row["car_distance_cm"]) for row in rows)
    maximum_distance = max(float(row["car_distance_cm"]) for row in rows)
    follow_time = sum(row["follow_active"] == 1 for row in rows) * DT_S
    search_time = sum(row["search_active"] == 1 for row in rows) * DT_S

    print("CAR_LEASH_SIMULATION")
    print(f"result={'PASS' if not failures else 'FAIL'}")
    print(f"distance_range_cm={minimum_distance:.1f}..{maximum_distance:.1f}")
    print(f"follow_time_s={follow_time:.2f}")
    print(f"search_time_s={search_time:.2f}")
    print(f"csv={output_path}")
    for failure in failures:
        print(f"failure={failure}")
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
