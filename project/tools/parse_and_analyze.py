import re
import numpy as np

log_file = "E:/CYT4BB7_Library-master/CYT4BB7_Library-master/LOGS/latest.txt"

data = {
    'time': [],
    'pos_x': [], 'pos_y': [],
    'vel_x': [], 'vel_y': [],
    'tgt_x': [], 'tgt_y': [],
    'obs_x': [], 'obs_y': [],
    'innov_x': [], 'innov_y': []
}

time_ms = 0

with open(log_file, 'r', encoding='utf-8', errors='ignore') as f:
    cur_frame = {}
    for line in f:
        line = line.strip()
        
        # New frame start approx
        if "DESKTOP DEBUG DASHBOARD" in line:
            if 'pos_x' in cur_frame:
                data['time'].append(time_ms / 1000.0)
                data['pos_x'].append(cur_frame.get('pos_x', 0))
                data['pos_y'].append(cur_frame.get('pos_y', 0))
                data['vel_x'].append(cur_frame.get('vel_x', 0))
                data['vel_y'].append(cur_frame.get('vel_y', 0))
                data['tgt_x'].append(cur_frame.get('tgt_x', 0))
                data['tgt_y'].append(cur_frame.get('tgt_y', 0))
                data['obs_x'].append(cur_frame.get('obs_x', 0))
                data['obs_y'].append(cur_frame.get('obs_y', 0))
                data['innov_x'].append(cur_frame.get('innov_x', 0))
                data['innov_y'].append(cur_frame.get('innov_y', 0))
                time_ms += 20.8 # approx 50Hz
            cur_frame = {}
            
        # Pos X (cm)  : Cur: -46.389 | Tgt: -46.389 | Err: 0.000
        m = re.search(r"Pos X \(cm\)\s*:\s*Cur:\s*([-.\d]+)\s*\|\s*Tgt:\s*([-.\d]+)", line)
        if m:
            cur_frame['pos_x'] = float(m.group(1))
            cur_frame['tgt_x'] = float(m.group(2))
            
        m = re.search(r"Pos Y \(cm\)\s*:\s*Cur:\s*([-.\d]+)\s*\|\s*Tgt:\s*([-.\d]+)", line)
        if m:
            cur_frame['pos_y'] = float(m.group(1))
            cur_frame['tgt_y'] = float(m.group(2))
            
        m = re.search(r"Vel X \(cm/s\)\s*:\s*Cur:\s*([-.\d]+)", line)
        if m:
            cur_frame['vel_x'] = float(m.group(1))
            
        m = re.search(r"Vel Y \(cm/s\)\s*:\s*Cur:\s*([-.\d]+)", line)
        if m:
            cur_frame['vel_y'] = float(m.group(1))
            
        # [FLOWOBS] gate:31.24 obs_xy:1.93/-6.97 innov_xy:-11.62/-4.66
        m = re.search(r"obs_xy:([-.\d]+)/([-.\d]+)\s*innov_xy:([-.\d]+)/([-.\d]+)", line)
        if m:
            cur_frame['obs_x'] = float(m.group(1))
            cur_frame['obs_y'] = float(m.group(2))
            cur_frame['innov_x'] = float(m.group(3))
            cur_frame['innov_y'] = float(m.group(4))

import pandas as pd
df = pd.DataFrame(data)

print(f"Parsed {len(df)} frames.")
if len(df) > 0:
    dt = 0.0208
    
    # 1. Pos vs Int(Vel) residual
    pos_y_pred = np.zeros(len(df))
    pos_y_pred[0] = df['pos_y'].iloc[0]
    for k in range(1, len(df)):
        pos_y_pred[k] = pos_y_pred[k-1] + df['vel_y'].iloc[k-1] * dt
    
    df['res_y'] = df['pos_y'] - pos_y_pred
    
    print("\n[1] Pos vs Int(Vel) Residual")
    print(f"res_y mean: {df['res_y'].mean():.4f} cm, RMS: {np.sqrt((df['res_y']**2).mean()):.4f} cm, End: {df['res_y'].iloc[-1]:.4f} cm")
    
    print("Sample: time | pos_y | vel_y | pos_y_pred | res_y")
    for i in range(200, 205):
        if i < len(df):
            print(f"{df['time'].iloc[i]:.2f} | {df['pos_y'].iloc[i]:.2f} | {df['vel_y'].iloc[i]:.2f} | {pos_y_pred[i]:.2f} | {df['res_y'].iloc[i]:.2f}")

    # 2. Low speed pos drift rate
    print("\n[2] Low Speed Pos Drift Rate")
    df['speed'] = np.sqrt(df['vel_x']**2 + df['vel_y']**2)
    low_speed = df['speed'] < 2.0
    
    import itertools
    regions = []
    for k, g in itertools.groupby(enumerate(low_speed), key=lambda x: x[1]):
        if k:
            indices = list(x[0] for x in g)
            dur = (indices[-1] - indices[0]) * dt
            if dur >= 0.3:
                dx = df['pos_x'].iloc[indices[-1]] - df['pos_x'].iloc[indices[0]]
                dy = df['pos_y'].iloc[indices[-1]] - df['pos_y'].iloc[indices[0]]
                drift = np.sqrt(dx**2 + dy**2) / dur
                print(f"t={df['time'].iloc[indices[0]]:.2f}s, dur={dur:.2f}s, dx={dx:.2f}cm, dy={dy:.2f}cm -> drift={drift:.2f} cm/s")

    # 3. Flow obs vs EKF vel directional bias
    print("\n[3] Flow Obs vs EKF Vel Directional Bias")
    df['bias_x'] = df['obs_x'] - df['vel_x']
    df['bias_y'] = df['obs_y'] - df['vel_y']
    
    pos_y_mask = df['vel_y'] > 5.0
    neg_y_mask = df['vel_y'] < -5.0
    print(f"When vel_y > 5 cm/s: Mean(Flow - EKF) Y = {df['bias_y'][pos_y_mask].mean():.2f} cm/s")
    print(f"When vel_y < -5 cm/s: Mean(Flow - EKF) Y = {df['bias_y'][neg_y_mask].mean():.2f} cm/s")

    # 4. R vs Vel phase relationship
    print("\n[4] R vs Vel Phase Relationship")
    df['pos_err_y'] = df['pos_y'] - df['tgt_y']
    df['pos_err_x'] = df['pos_x'] - df['tgt_x']
    df['r'] = np.sqrt(df['pos_err_x']**2 + df['pos_err_y']**2)
    
    # print sample of peaks/valleys
    print("Sample around max R:")
    max_r_idx = df['r'].idxmax()
    start_idx = max(0, max_r_idx - 5)
    end_idx = min(len(df), max_r_idx + 6)
    print("time | r | speed | pos_err_y | vel_y")
    for i in range(start_idx, end_idx):
        print(f"{df['time'].iloc[i]:.2f} | {df['r'].iloc[i]:.2f} | {df['speed'].iloc[i]:.2f} | {df['pos_err_y'].iloc[i]:.2f} | {df['vel_y'].iloc[i]:.2f}")

    # 5. Sensitivity of PE with anchor offset
    print("\n[5] Target/Anchor Offset Analysis")
    mean_pos_y = df['pos_y'].mean()
    print(f"Initial target_y: {df['tgt_y'].iloc[0]:.2f}, Mean pos_y: {mean_pos_y:.2f}")
    df['r_anchor'] = df['pos_y'] - mean_pos_y
    
    g_L = 980.0 / 100.0
    pe_target = 0.5 * g_L * df['pos_err_y']**2
    pe_anchor = 0.5 * g_L * df['r_anchor']**2
    
    print(f"Max PE using target: {pe_target.max():.2f} (err_y max: {df['pos_err_y'].abs().max():.2f})")
    print(f"Max PE using mean pos: {pe_anchor.max():.2f} (r_anchor max: {df['r_anchor'].abs().max():.2f})")
    print("Notice how PE amplitude drastically changes if the anchor is offset from the target.")
