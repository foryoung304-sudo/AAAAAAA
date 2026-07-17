import re
import numpy as np

log_file = "E:/CYT4BB7_Library-master/CYT4BB7_Library-master/LOGS/latest.txt"
data = { 'time': [], 'pos_x': [], 'pos_y': [], 'vel_x': [], 'vel_y': [], 'tgt_x': [], 'tgt_y': [] }
time_ms = 0
with open(log_file, 'r', encoding='utf-8', errors='ignore') as f:
    cur_frame = {}
    for line in f:
        line = line.strip()
        if "DESKTOP DEBUG DASHBOARD" in line:
            if 'pos_x' in cur_frame:
                data['time'].append(time_ms / 1000.0)
                data['pos_x'].append(cur_frame.get('pos_x', 0))
                data['pos_y'].append(cur_frame.get('pos_y', 0))
                data['vel_x'].append(cur_frame.get('vel_x', 0))
                data['vel_y'].append(cur_frame.get('vel_y', 0))
                data['tgt_x'].append(cur_frame.get('tgt_x', 0))
                data['tgt_y'].append(cur_frame.get('tgt_y', 0))
                time_ms += 20.8
            cur_frame = {}
        m = re.search(r"Pos X \(cm\)\s*:\s*Cur:\s*([-.\d]+)\s*\|\s*Tgt:\s*([-.\d]+)", line)
        if m: cur_frame['pos_x'], cur_frame['tgt_x'] = float(m.group(1)), float(m.group(2))
        m = re.search(r"Pos Y \(cm\)\s*:\s*Cur:\s*([-.\d]+)\s*\|\s*Tgt:\s*([-.\d]+)", line)
        if m: cur_frame['pos_y'], cur_frame['tgt_y'] = float(m.group(1)), float(m.group(2))
        m = re.search(r"Vel X \(cm/s\)\s*:\s*Cur:\s*([-.\d]+)", line)
        if m: cur_frame['vel_x'] = float(m.group(1))
        m = re.search(r"Vel Y \(cm/s\)\s*:\s*Cur:\s*([-.\d]+)", line)
        if m: cur_frame['vel_y'] = float(m.group(1))

import pandas as pd
df = pd.DataFrame(data)

df['mean_x'] = df['pos_x'].mean()
df['mean_y'] = df['pos_y'].mean()
df['rx'] = df['pos_x'] - df['mean_x']
df['ry'] = df['pos_y'] - df['mean_y']
df['r'] = np.sqrt(df['rx']**2 + df['ry']**2)
df['speed'] = np.sqrt(df['vel_x']**2 + df['vel_y']**2)
df['vr'] = (df['rx']*df['vel_x'] + df['ry']*df['vel_y']) / df['r']
df['vt'] = (df['rx']*df['vel_y'] - df['ry']*df['vel_x']) / df['r']

print("\n[径向/切向速度数据 (R极值点附近)]")
max_r_idx = df['r'].idxmax()
print("time | r (cm) | speed | v_radial | v_tangent")
for i in range(max(0, max_r_idx - 3), min(len(df), max_r_idx + 4)):
    print(f"{df['time'].iloc[i]:.2f} | {df['r'].iloc[i]:.2f} | {df['speed'].iloc[i]:.2f} | {df['vr'].iloc[i]:.2f} | {df['vt'].iloc[i]:.2f}")
