#!/usr/bin/env python3
"""Static contract checks plus a readable simple_finish_v2 mission replay."""

from pathlib import Path
import re


ROOT = Path(__file__).resolve().parents[2]
VISION_C = (ROOT / "project/code/vision_nav.c").read_text(encoding="utf-8")
VISION_H = (ROOT / "project/code/vision_nav.h").read_text(encoding="utf-8")
LOC_C = (ROOT / "project/code/loc_ctrl.c").read_text(encoding="utf-8")
LOC_H = (ROOT / "project/code/loc_ctrl.h").read_text(encoding="utf-8")
BEACON_C = (ROOT / "project/code/beacon.c").read_text(encoding="utf-8")
PARAM_C = (ROOT / "project/code/param.c").read_text(encoding="utf-8")
PARAM_H = (ROOT / "project/code/param.h").read_text(encoding="utf-8")
REMOTE_C = (ROOT / "project/code/remote_ctrl.c").read_text(encoding="utf-8")


def function_body(source: str, signature: str) -> str:
    start = source.index(signature)
    brace = source.index("{", start)
    depth = 0
    for index in range(brace, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return source[brace + 1:index]
    raise AssertionError(f"unterminated function: {signature}")


def macro_us(name: str) -> int:
    match = re.search(rf"^#define\s+{name}\s+(\d+)u", VISION_H, re.MULTILINE)
    assert match, f"missing macro {name}"
    return int(match.group(1))


def macro_int(source: str, name: str) -> int:
    match = re.search(rf"^#define\s+{name}\s+(\d+)", source, re.MULTILINE)
    assert match, f"missing macro {name}"
    return int(match.group(1))


def macro_float(source: str, name: str) -> float:
    match = re.search(
        rf"^#define\s+{name}\s+(-?\d+(?:\.\d+)?)f",
        source,
        re.MULTILINE,
    )
    assert match, f"missing macro {name}"
    return float(match.group(1))


direct = function_body(VISION_C, "static void vision_nav_direct_car_follow_update")
horizontal_fault = function_body(VISION_C, "static void vision_nav_update_horizontal_fault")
yaw_spin = function_body(VISION_C, "static uint8_t vision_nav_yaw_spin_update")
loc_position = function_body(LOC_C, "void loc_2level_ctrl")
nav_update = function_body(VISION_C, "void vision_nav_update(void)")
start_search = function_body(VISION_C, "static void vision_nav_start_search")
candidate_select = function_body(BEACON_C, "static uint8_t mission_select_beacon_candidate")
param_update = function_body(PARAM_C, "void param_update(float dT_s)")

# Source-level ownership and safety contracts.
assert "vision_nav_obs.beacon_loss_brake_active = 0u" not in direct
assert "vision_nav_obs.beacon_loss_brake_active == 0u" in direct
assert nav_update.index("vision_nav_obs.beacon_loss_brake_active =") < nav_update.index(
    "vision_nav_follow_car_update"
)
assert "vision_nav_yaw_spin_is_done() != 0u" in param_update
assert "uint8_t mission_yaw_ready = 1u" in horizontal_fault or \
    "uint8_t mission_yaw_ready = 1u" in VISION_C
assert "#if VISION_NAV_YAW_SPIN_ENABLE" in VISION_C
assert "mission_task_requested = 1u" in param_update
assert "auto_landing_request = 1u" not in nav_update
assert "vehicle_state.flight_mode != FLY_AUTOTAKEOFF" in horizontal_fault
assert "vehicle_state.flight_mode == FLY_AUTOFLY" in horizontal_fault
assert macro_float(PARAM_H, "FLIGHT_FAILSAFE_IMU_CONFIRM_S") == 0.30
assert macro_float(PARAM_H, "FLIGHT_FAILSAFE_TOF_CONFIRM_S") == 2.00
assert macro_float(PARAM_H, "FLIGHT_FAILSAFE_FLOW_CONFIRM_S") == 4.00
assert macro_float(VISION_H, "VISION_NAV_HORIZONTAL_FAULT_SPEED_CM_S") == 75.0
assert macro_float(VISION_H, "VISION_NAV_HORIZONTAL_FAULT_CRITICAL_SPEED_CM_S") == 120.0
assert macro_int(VISION_H, "VISION_NAV_HORIZONTAL_FAULT_BAD_FRAMES") == 10
assert macro_float(VISION_H, "VISION_NAV_HORIZONTAL_FAULT_CLEAR_SPEED_CM_S") == 30.0
assert macro_int(VISION_H, "VISION_NAV_HORIZONTAL_FAULT_CLEAR_FRAMES") == 15
assert macro_us("VISION_NAV_HORIZONTAL_FAULT_LAND_US") == 6000000
assert PARAM_C.count("FAILSAFECFG,imu_confirm_ms=") == 2
assert "remote_land_switch=immediate" in PARAM_C
assert REMOTE_C.count("auto_landing_request = 1u;") == 2
assert "vision_nav_yaw_spin_done = 0u" in yaw_spin
assert "vision_nav_obs.horizontal_fault_active == 0u" in param_update
assert "vision_nav_center_return_required = 1u" in start_search
assert "vision_nav_search_center_x_cm = vehicle_state.current_pos_x" not in start_search
assert "vision_nav_search_center_y_cm = vehicle_state.current_pos_y" not in start_search
assert "VISION_NAV_CENTER_ONLY_CONFIRM_FRAMES" in candidate_select
assert "vision_nav_beacon_selection_blocked()" in candidate_select
assert "vision_nav_locked_target_is_center()" in candidate_select
assert "VISION_NAV_CENTER_OBSERVATION_BACKOFF_CM" in VISION_C
assert "BEACONPLANCFG,center_policy=%u" in PARAM_C
assert "vision_nav_yaw_spin_is_active() != 0u" in LOC_C
assert "position_gain_scale = LOC_YAW_SPIN_POS_GAIN_SCALE" in LOC_C
assert "YAWPOSCFG,pos_gain_scale=%.2f" in PARAM_C
assert "YAWLOADCFG,enabled=%u,target_rate_dps=%.1f" in PARAM_C
assert "TAKEOFFPOSCFG,damping_enable_cm=%.1f" in PARAM_C
assert "preserves the target captured on" in LOC_C
assert "vehicle_setpoint.target_pos_x = vehicle_state.current_pos_x" not in loc_position

yaw_pos_gain = macro_float(LOC_H, "LOC_YAW_SPIN_POS_GAIN_SCALE")
yaw_enabled = macro_int(VISION_H, "VISION_NAV_YAW_SPIN_ENABLE")
yaw_pos_limit = macro_float(
    LOC_H, "LOC_YAW_SPIN_POS_CORRECTION_LIMIT_CM_S"
)
yaw_vel_limit = macro_float(LOC_H, "LOC_YAW_SPIN_TOTAL_VEL_LIMIT_CM_S")
yaw_angle_limit = macro_float(VISION_H, "VISION_NAV_YAW_SPIN_LOC_ANGLE_DEG")
yaw_target_rate = macro_float(VISION_H, "VISION_NAV_YAW_SPIN_RATE_DPS")
yaw_max_rate = macro_float(VISION_H, "VISION_NAV_YAW_SPIN_MAX_RATE_DPS")
yaw_ct_limit = macro_float(VISION_H, "VISION_NAV_YAW_SPIN_CT_LIMIT")
yaw_total_deg = macro_float(VISION_H, "VISION_NAV_YAW_SPIN_TOTAL_DEG")
yaw_segment_deg = macro_float(VISION_H, "VISION_NAV_YAW_SPIN_SEGMENT_DEG")
yaw_finish_tolerance = macro_float(
    VISION_H, "VISION_NAV_YAW_SPIN_FINISH_TOLERANCE_DEG"
)
yaw_pause_ms = macro_us("VISION_NAV_YAW_SPIN_SEGMENT_PAUSE_US") // 1000
yaw_settle_ms = macro_us("VISION_NAV_YAW_SPIN_SETTLE_US") // 1000
takeoff_damping_cm = macro_float(
    LOC_H, "LOC_AUTO_TAKEOFF_HORIZONTAL_ENABLE_HEIGHT_CM"
)
takeoff_hold_cm = macro_float(LOC_H, "LOC_AUTO_LOW_HOLD_HEIGHT_CM")
takeoff_release_cm = macro_float(
    LOC_H, "LOC_AUTO_TAKEOFF_HORIZONTAL_RELEASE_HEIGHT_CM"
)
assert yaw_pos_gain > 1.0
assert yaw_pos_limit >= yaw_vel_limit
assert 3.0 < yaw_angle_limit <= 4.0
assert yaw_target_rate == -24.0
assert yaw_max_rate == 26.0
assert yaw_ct_limit == 5.0
assert "vision_nav_yaw_spin_height_recovery_active" in yaw_spin
assert "VISION_NAV_YAW_SPIN_RECOVERY_RESUME_HEIGHT_CM" in yaw_spin
assert "vision_nav_yaw_spin_aborted = 1u" not in yaw_spin
assert yaw_segment_deg == 120.0
assert yaw_pause_ms == 250
assert 0.0 < yaw_finish_tolerance <= 10.0
assert yaw_settle_ms <= 100
assert 16.0 < takeoff_damping_cm < takeoff_hold_cm
assert 16.0 < takeoff_release_cm < takeoff_damping_cm

brake_ms = macro_us("VISION_NAV_BEACON_LOSS_BRAKE_DELAY_US") // 1000
reset_ms = macro_us("VISION_NAV_TARGET_LOST_RESET_US") // 1000
assert brake_ms < reset_ms

spin_replay = (
    [
        ("CRUISE_READY", "start segmented ~350 deg yaw spin; task blocked"),
        ("SPIN_SEGMENT_PAUSE", "Yaw zero for 250ms after each 120 deg segment"),
        ("SPIN_HEIGHT_RECOVERY", "pause Yaw during drop; resume above 70cm"),
        ("SPIN_DONE", "capture local origin; task may enter with valid Y-car"),
    ]
    if yaw_enabled
    else [("YAW_DISABLED", "skip spin; climb to cruise mission gate")]
)

replay = [
    ("DISARMED", "aircraft hold; Y-car STOP"),
    ("TAKEOFF", "damp after 20cm reading; hold launch point after 24cm; Y-car STOP"),
    ("TAKEOFF_FLOW_FAULT", "fixed-point brake; <=5 deg; block spin/task; no new auto-land"),
] + spin_replay + [
    ("SEARCH", "fly fixed first leg; Y-car follows aircraft"),
    ("RAW_CANDIDATE", "aircraft brakes immediately; Y-car STOP"),
    ("CONFIRMED", "Y-car follows lamp; aircraft follows Y-car"),
    (f"LAMP_LOST_0..{brake_ms}ms", "bounded old car command; aircraft follows"),
    (f"LAMP_LOST_{brake_ms}..{reset_ms}ms", "Y-car STOP; aircraft fixed-point brake"),
    (f"LAMP_LOST_{reset_ms}ms", "hold, then aircraft leads Y-car to centre observation point"),
    ("CENTRE_SURVEY_MULTI", "keep centre lamp; select an outer lamp"),
    ("CENTRE_SURVEY_SINGLE", "confirm only one lamp remains; extinguish it directly"),
    ("TASK_FINISH", "operator-controlled stop/disarm; no mission auto-land"),
]

print("MISSION FLOW CONTRACT: PASS")
for phase, action in replay:
    print(f"{phase:24s} -> {action}")
