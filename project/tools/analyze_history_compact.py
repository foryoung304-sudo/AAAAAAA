import csv
import math
import re
import sys


def load_history(path):
    lines = open(path, encoding="utf-8", errors="ignore").read().splitlines()
    start = next(i for i, line in enumerate(lines) if "HISTORY_STATE_BEGIN" in line)
    header_i = next(i for i in range(start + 1, len(lines)) if lines[i].startswith("hseq,"))
    header = lines[header_i]
    i = header_i + 1
    while header.count(",") < 27 and i < len(lines):
        header += lines[i]
        i += 1

    records = []
    current = ""
    row_start = re.compile(r"^(\d+),(\d{6,}),")
    for line in lines[i:]:
        if line.startswith(("FLIGHTCFG", "TRIMCFG", "DEBUG_STATE_END")):
            break
        if row_start.match(line):
            if current:
                records.append(current)
            current = line
        else:
            current += line
    if current:
        records.append(current)

    names = next(csv.reader([header]))
    rows = []
    for record in records:
        values = next(csv.reader([record]))
        if len(values) != len(names):
            continue
        try:
            rows.append({name: int(value) for name, value in zip(names, values)})
        except ValueError:
            pass
    return rows


def load_detailed(path):
    lines = open(path, encoding="utf-8", errors="ignore").read().splitlines()
    header_i = next(
        i for i, line in enumerate(lines)
        if line.startswith("seq,time")
        or (line == "s" and i + 1 < len(lines) and lines[i + 1].startswith("eq,time_us,"))
    )
    if lines[header_i] == "s":
        header = lines[header_i] + lines[header_i + 1]
        i = header_i + 2
    else:
        header = lines[header_i]
        i = header_i + 1
    while ("profile_vel_y_e" not in header) and i < len(lines):
        header += lines[i]
        i += 1
    names = next(csv.reader([header]))
    records = []
    current = ""
    row_start = re.compile(r"^(\d+),(\d{6,}),")
    for line in lines[i:]:
        if "HISTORY_STATE_BEGIN" in line:
            break
        if row_start.match(line):
            if current:
                records.append(current)
            current = line
        else:
            current += line
    if current:
        records.append(current)
    rows = []
    for record in records:
        values = next(csv.reader([record]))
        if len(values) != len(names):
            continue
        try:
            rows.append({name: float(value) for name, value in zip(names, values)})
        except ValueError:
            pass
    return rows


def mean(xs):
    return sum(xs) / len(xs) if xs else float("nan")


def span(xs):
    return max(xs) - min(xs) if xs else float("nan")


def corr(xs, ys):
    mx, my = mean(xs), mean(ys)
    dx = [x - mx for x in xs]
    dy = [y - my for y in ys]
    den = math.sqrt(sum(x*x for x in dx) * sum(y*y for y in dy))
    return sum(x*y for x, y in zip(dx, dy)) / den if den else float("nan")


def main(path, large_error_gain=1.5, position_velocity_limit=5.0):
    rows = load_history(path)
    if not rows:
        raise SystemExit("no HISTORY rows parsed")
    t0 = rows[0]["time_us"]
    time_s = [(r["time_us"] - t0) / 1e6 for r in rows]
    pos = [r["pos_err_y_dcm"] / 10 for r in rows]
    vel = [r["ekf_vy_dcms"] / 10 for r in rows]
    flow = [r["flow_obs_vy_dcms"] / 10 for r in rows]
    yaw = [r["yaw_cdeg"] / 100 for r in rows]
    integ = [r["loc_vel_i_y_mdeg"] / 1000 for r in rows]
    voltage = [r["voltage_cV"] / 100 for r in rows]
    raw_roll = [r["raw_roll_cdeg"] / 100 for r in rows]
    roll = [r["roll_cur_cdeg"] / 100 for r in rows]
    valid = [i for i, r in enumerate(rows) if r["flow_obs_valid"] and r["flow_ekf_used"]]
    mid = len(rows) // 2

    print(f"rows={len(rows)} duration={time_s[-1]:.2f}s")
    print(f"pos_y: start={pos[0]:+.1f} end={pos[-1]:+.1f} range=[{min(pos):+.1f},{max(pos):+.1f}] span={span(pos):.1f}")
    print(f"pos span first/second={span(pos[:mid]):.1f}/{span(pos[mid:]):.1f} cm")
    print(f"vel_y: start={vel[0]:+.1f} end={vel[-1]:+.1f} range=[{min(vel):+.1f},{max(vel):+.1f}]")
    print(f"yaw: start={yaw[0]:+.2f} end={yaw[-1]:+.2f} range=[{min(yaw):+.2f},{max(yaw):+.2f}]")
    print(f"voltage: start={voltage[0]:.2f} end={voltage[-1]:.2f} mean={mean(voltage):.2f} range=[{min(voltage):.2f},{max(voltage):.2f}]")
    print(f"loc_i_y: start={integ[0]:+.3f} end={integ[-1]:+.3f} range=[{min(integ):+.3f},{max(integ):+.3f}]")
    print(f"raw/actual roll span={span(raw_roll):.2f}/{span(roll):.2f} deg")
    moving = [i for i, v in enumerate(vel) if abs(v) >= 3.0]
    raw_brake = mean([1.0 if raw_roll[i] * vel[i] < 0 else 0.0 for i in moving])
    act_brake = mean([1.0 if roll[i] * vel[i] < 0 else 0.0 for i in moving])
    print(f"braking sign when |Vy|>=3: raw={raw_brake*100:.1f}% actual={act_brake*100:.1f}%")
    print(f"flow used={len(valid)}/{len(rows)} corr(flow,ekf)={corr([flow[i] for i in valid], [vel[i] for i in valid]):.3f}")
    print("segments:")
    for seg in range(5):
        a = len(rows) * seg // 5
        b = len(rows) * (seg + 1) // 5
        print(
            f"  {time_s[a]:4.1f}-{time_s[b-1]:4.1f}s "
            f"pos_mean={mean(pos[a:b]):+6.1f} span={span(pos[a:b]):5.1f} "
            f"vel_mean={mean(vel[a:b]):+5.1f} span={span(vel[a:b]):5.1f} "
            f"yaw_mean={mean(yaw[a:b]):+5.2f} "
            f"I_mean={mean(integ[a:b]):+.3f}"
        )

    baseline_n = min(len(rows), 100)
    baseline_pos = mean(pos[:baseline_n])
    onset = None
    for i in range(baseline_n, len(rows) - 24):
        if all(abs(pos[j] - baseline_pos) >= 8.0 for j in range(i, i + 25)):
            onset = i
            break
    if onset is not None:
        print(f"sustained departure onset={time_s[onset]:.2f}s baseline_pos={baseline_pos:+.1f}cm")
        print("  t    pos    vel   flow   yaw      I   rawR   curR obs used")
        for i in range(max(0, onset - 50), min(len(rows), onset + 101), 10):
            r = rows[i]
            print(
                f" {time_s[i]:4.1f} {pos[i]:+6.1f} {vel[i]:+6.1f} {flow[i]:+6.1f} "
                f"{yaw[i]:+5.2f} {integ[i]:+6.3f} {raw_roll[i]:+6.2f} {roll[i]:+6.2f} "
                f" {r['flow_obs_valid']}    {r['flow_ekf_used']}"
            )

    detailed = load_detailed(path)
    if detailed:
        angle_errors = [r["roll_tgt"] - r["roll_cur"] for r in detailed]
        boosts = []
        for error in angle_errors:
            if error > 0.5:
                boost = (error - 0.5) * large_error_gain
            elif error < -0.5:
                boost = (error + 0.5) * large_error_gain
            else:
                boost = 0.0
            boosts.append(max(-3.0, min(3.0, boost)))
        active = [i for i, boost in enumerate(boosts) if boost != 0.0]
        motors = [
            r[name]
            for r in detailed
            for name in ("m1", "m2", "m3", "m4")
            if 0.0 <= r[name] <= 10000.0
        ]
        rms = lambda values: math.sqrt(mean([value * value for value in values]))
        print(
            f"large-error boost(gain={large_error_gain:.2f}) "
            f"active={len(active)}/{len(detailed)} "
            f"({len(active) / len(detailed) * 100:.1f}%) "
            f"range=[{min(boosts):+.2f},{max(boosts):+.2f}]dps"
        )
        print(
            f"roll angle-error rms={rms(angle_errors):.2f}deg "
            f"rate target/fb range="
            f"[{min(r['rate_tgt_r'] for r in detailed):+.2f},{max(r['rate_tgt_r'] for r in detailed):+.2f}]/"
            f"[{min(r['rate_fb_r'] for r in detailed):+.2f},{max(r['rate_fb_r'] for r in detailed):+.2f}]dps "
            f"motors=[{min(motors):.0f},{max(motors):.0f}]"
        )
        large_pos = [i for i, r in enumerate(detailed) if abs(r["pos_err_y_e"]) >= 10.0]
        if large_pos:
            target_toward = mean([
                1.0 if detailed[i]["vel_tgt_y_e"] * detailed[i]["pos_err_y_e"] > 0.0 else 0.0
                for i in large_pos
            ])
            actual_toward = mean([
                1.0 if detailed[i]["vel_y_e"] * detailed[i]["pos_err_y_e"] > 0.0 else 0.0
                for i in large_pos
            ])
            target_saturated = mean([
                1.0 if abs(detailed[i]["vel_tgt_y_e"]) >= position_velocity_limit - 0.1 else 0.0
                for i in large_pos
            ])
            raw_roll_toward = mean([
                1.0 if detailed[i]["loc_raw_roll"] * detailed[i]["pos_err_y_e"] > 0.0 else 0.0
                for i in large_pos
            ])
            actual_roll_toward = mean([
                1.0 if detailed[i]["roll_cur"] * detailed[i]["pos_err_y_e"] > 0.0 else 0.0
                for i in large_pos
            ])
            print(
                f"position recovery when |ErrY|>=10cm: "
                f"target-toward={target_toward * 100:.1f}% "
                f"actual-toward={actual_toward * 100:.1f}% "
                f"vel-target-at-{position_velocity_limit:g}cm/s-limit="
                f"{target_saturated * 100:.1f}% "
                f"raw/actual-roll-toward={raw_roll_toward * 100:.1f}%/"
                f"{actual_roll_toward * 100:.1f}%"
            )
    mismatch = [r for r in detailed
                if abs(r.get("roll_tgt", 0.0)) >= 0.3
                and r.get("roll_tgt", 0.0) * r.get("roll_cur", 0.0) < 0]
    print(f"detailed_rows={len(detailed)} roll_sign_mismatch={len(mismatch)}")
    if mismatch:
        print("  seq rawR tgtR curR rate_tgt rate_fb rate_out rateP rateI rateD motors")
        step = max(1, len(mismatch) // 8)
        for r in mismatch[::step][:8]:
            print(
                f" {int(r['seq']):4d} {r['loc_raw_roll']:+5.2f} {r['roll_tgt']:+5.2f} {r['roll_cur']:+5.2f} "
                f"{r['rate_tgt_r']:+6.2f} {r['rate_fb_r']:+6.2f} {r['rate_out_r']:+6.2f} "
                f"{r['rate_p_r']:+5.2f} {r['rate_i_r']:+5.2f} {r['rate_d_r']:+5.2f} "
                f"{int(r['m1'])}/{int(r['m2'])}/{int(r['m3'])}/{int(r['m4'])}"
            )


if __name__ == "__main__":
    gain = float(sys.argv[2]) if len(sys.argv) > 2 else 1.5
    position_velocity_limit = float(sys.argv[3]) if len(sys.argv) > 3 else 5.0
    main(sys.argv[1], gain, position_velocity_limit)
