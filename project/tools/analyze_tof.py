import math
import sys

# 默认偏移，可通过命令行覆盖
TOF_OFFSET_X = 8.5
TOF_OFFSET_Y = -0.5

def deg2rad(d):
    return d * math.pi / 180.0

def parse_lines(lines):
    frames = []
    pending_angle = None
    for ln in lines:
        ln = ln.strip()
        if not ln:
            continue
        if not ln.startswith("channel_data:"):
            continue
        vals = ln.split("channel_data:",1)[1].split(",")
        nums = []
        for v in vals:
            try:
                nums.append(float(v))
            except:
                pass
        if not nums:
            continue
        # Heuristic: 距离通常以 cm 为单位，值会较大（> 20）；角度通常在 -180..180 范围
        first = nums[0]
        if first > 20.0:
            # 这是距离行
            if pending_angle is not None:
                frames.append((pending_angle, nums))
                pending_angle = None
            else:
                # 未知顺序，保存 distance as standalone (angle missing)
                frames.append(([0.0,0.0,0.0], nums))
        else:
            # 这是角度行，保存为 pending，等待下一条距离
            pending_angle = nums
    # 如果末尾仍有 pending_angle，忽略
    return frames


def compute_expected_height(a, b, offx, offy):
    # a: list of three numbers, 假设 a = [pitch_deg, roll_deg, yaw_deg] or [x,y,z]
    # b: list where first element是距离（cm）
    dist_cm = b[0]
    # 尝试识别角度字段：若 a 的范围像角度（-180~180），使用 a[0], a[1]
    pitch = a[0]
    roll = a[1] if len(a)>1 else 0.0
    pitch_sin = math.sin(deg2rad(pitch))
    roll_sin = math.sin(deg2rad(roll))
    pitch_cos = math.cos(deg2rad(pitch))
    roll_cos = math.cos(deg2rad(roll))
    height_cm = dist_cm * pitch_cos * roll_cos
    lever_arm_z = offx * pitch_sin - offy * roll_sin
    corrected = height_cm - lever_arm_z
    return {
        'dist_cm': dist_cm,
        'height_cm': height_cm,
        'lever_arm_z': lever_arm_z,
        'corrected': corrected,
        'pitch': pitch,
        'roll': roll
    }

def fit_offsets(frames):
    # Solve for params [offx, offy, H0] minimizing ||A params - y||
    # A rows: [pitch_sin, -roll_sin, 1], y = height_cm
    rows = []
    ys = []
    for a,b in frames:
        pitch = a[0]
        roll = a[1] if len(a)>1 else 0.0
        ps = math.sin(deg2rad(pitch))
        rs = math.sin(deg2rad(roll))
        pc = math.cos(deg2rad(pitch))
        rc = math.cos(deg2rad(roll))
        dist = b[0]
        y = dist * pc * rc
        rows.append((ps, -rs, 1.0))
        ys.append(y)

    # Build normal equations: (AtA) params = AtY
    AtA = [[0.0]*3 for _ in range(3)]
    AtY = [0.0]*3
    for (r0,r1,r2), y in zip(rows, ys):
        v = [r0, r1, r2]
        for i in range(3):
            for j in range(3):
                AtA[i][j] += v[i]*v[j]
            AtY[i] += v[i]*y

    # Solve 3x3 linear system AtA * params = AtY via Gaussian elimination
    M = [row[:] for row in AtA]
    for i in range(3):
        M[i].append(AtY[i])

    # Gaussian elimination
    n = 3
    for i in range(n):
        # pivot
        piv = i
        for k in range(i, n):
            if abs(M[k][i]) > abs(M[piv][i]):
                piv = k
        if abs(M[piv][i]) < 1e-12:
            return None
        M[i], M[piv] = M[piv], M[i]
        # normalize
        pivval = M[i][i]
        for j in range(i, n+1):
            M[i][j] /= pivval
        # eliminate
        for k in range(n):
            if k == i: continue
            factor = M[k][i]
            for j in range(i, n+1):
                M[k][j] -= factor * M[i][j]

    params = [M[i][n] for i in range(n)]
    # params = [offx, offy_neg, H0] because we used -rs in matrix for offy
    offx = params[0]
    offy = -params[1]
    H0 = params[2]
    return offx, offy, H0

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python analyze_tof.py path/to/log.txt [TOF_OFFSET_X_cm] [TOF_OFFSET_Y_cm]")
        sys.exit(1)
    logpath = sys.argv[1]
    if len(sys.argv) >= 3:
        TOF_OFFSET_X = float(sys.argv[2])
    if len(sys.argv) >= 4:
        TOF_OFFSET_Y = float(sys.argv[3])

    with open(logpath, 'r', encoding='utf-8') as f:
        lines = f.readlines()
    frames = parse_lines(lines)
    print("i,pitch,roll,dist_cm,height_cm,lever_arm_z,corrected")
    for i,(a,b) in enumerate(frames):
        r = compute_expected_height(a,b,TOF_OFFSET_X,TOF_OFFSET_Y)
        print(f"{i},{r['pitch']:.3f},{r['roll']:.3f},{r['dist_cm']:.3f},{r['height_cm']:.3f},{r['lever_arm_z']:.3f},{r['corrected']:.3f}")
    # 拟合偏移量
    fit = fit_offsets(frames)
    if fit is not None:
        fx, fy, H0 = fit
        print()
        print(f"Fitted TOF_OFFSET_X = {fx:.3f} cm")
        print(f"Fitted TOF_OFFSET_Y = {fy:.3f} cm")
        print(f"Estimated mean height H0 = {H0:.3f} cm")
