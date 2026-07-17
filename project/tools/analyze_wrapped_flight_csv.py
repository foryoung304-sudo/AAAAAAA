import csv
import re
import sys


def main(path: str) -> None:
    text = open(path, "r", encoding="utf-8", errors="ignore").read()
    # Attachments can contain hard wraps in the middle of CSV records.
    flat = text.replace("\r", "").replace("\n", "")
    starts = list(re.finditer(
        r"(\d{1,5}),(\d{7,10}),(19\.\d{2}|20\.\d{2}),([01]),(\d+),(\d+\.\d+),",
        flat,
    ))
    rows = []
    expected_columns = 68
    for index, match in enumerate(starts):
        end = starts[index + 1].start() if index + 1 < len(starts) else len(flat)
        fields = next(csv.reader([flat[match.start():end]]))
        if len(fields) >= expected_columns:
            rows.append(fields[:expected_columns])

    if not rows:
        raise SystemExit("no flight rows parsed")

    names = [
        "seq", "time_us", "dt_ms", "armed", "phase", "voltage",
        "loc_ready", "loc_hold", "loc_weight", "yaw_deg",
        "pos_err_x_e", "pos_err_y_e", "vel_x_e", "vel_y_e",
        "vel_tgt_x_e", "vel_tgt_y_e", "vel_err_x_b", "vel_err_y_b",
        "loc_raw_roll", "loc_raw_pitch", "loc_trim_roll", "loc_trim_pitch",
        "loc_ramped_roll", "loc_ramped_pitch", "loc_vel_p_x", "loc_vel_i_x",
        "loc_vel_d_x", "loc_vel_p_y", "loc_vel_i_y", "loc_vel_d_y",
        "roll_tgt", "roll_cur", "pitch_tgt", "pitch_cur",
        "sp_rate_ff_roll", "sp_rate_ff_pitch", "rate_tgt_r", "rate_fb_r",
        "rate_out_r", "rate_p_r", "rate_i_r", "rate_d_r", "rate_tgt_p",
        "rate_fb_p", "rate_out_p", "imu_acc_body_y_m_s2", "flow_obs_vx_e",
        "flow_obs_vy_e", "ekf_vx_e", "ekf_vy_e", "flow_valid",
        "flow_obs_valid", "flow_ekf_used", "flow_gate_clipped", "throttle",
        "height_cm", "vel_z_cm_s", "m1", "m2", "m3", "m4",
        "att_voltage_scale", "goal_pos_x_e", "goal_pos_y_e", "profile_pos_x_e",
        "profile_pos_y_e", "profile_vel_x_e", "profile_vel_y_e",
    ]
    data = [dict(zip(names, row)) for row in rows]

    print(f"rows={len(data)} seq={data[0]['seq']}..{data[-1]['seq']}")
    for name in ("loc_ready", "loc_hold", "flow_valid", "flow_obs_valid", "flow_ekf_used", "flow_gate_clipped"):
        counts = {}
        for row in data:
            value = row[name]
            counts[value] = counts.get(value, 0) + 1
        print(f"{name}: {counts}")
    for name in ("height_cm", "voltage", "loc_weight", "pos_err_x_e", "pos_err_y_e", "vel_x_e", "vel_y_e", "loc_raw_roll", "loc_raw_pitch"):
        values = [float(row[name]) for row in data]
        print(f"{name}: min={min(values):.3f} max={max(values):.3f}")

    changes = []
    last = None
    for row in data:
        state = (row["loc_ready"], row["loc_hold"], row["flow_valid"], row["flow_obs_valid"], row["flow_ekf_used"])
        if state != last:
            changes.append((row["seq"], row["height_cm"], state))
            last = state
    print("state changes:")
    for change in changes[:30]:
        print(change)


if __name__ == "__main__":
    main(sys.argv[1])
