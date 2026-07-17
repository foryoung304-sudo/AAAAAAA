import pandas as pd
import numpy as np

# Load the latest log
csv_file = "bias_trajectory_latest_log_3.csv"
try:
    df = pd.read_csv(csv_file)
except Exception as e:
    csv_file = "parsed_log.csv"
    df = pd.read_csv(csv_file)

# Ensure time is sorted and calculate dt
if 'time' not in df.columns and 'timestamp' in df.columns:
    df['time'] = df['timestamp'] / 1000.0 # assume ms
elif 'time' not in df.columns:
    df['time'] = np.arange(len(df)) * 0.01 # assume 100Hz

df = df.sort_values('time').reset_index(drop=True)
df['dt'] = df['time'].diff().fillna(0.01)

print("Columns:", df.columns.tolist())

# Basic columns needed
# 'ekf_y_cm', 'ekf_vy_cm_s', 'ekf_x_cm', 'ekf_vx_cm_s'
# 'obs_y_cm', 'obs_vy_cm_s', 'obs_x_cm', 'obs_vx_cm_s'
# 'target_x_cm', 'target_y_cm'

# Find actual column names
cx = [c for c in df.columns if 'ekf' in c and 'x' in c and 'cm' in c and 'v' not in c]
cvx = [c for c in df.columns if 'ekf' in c and 'v' in c and 'x' in c and 'cm' in c]
cy = [c for c in df.columns if 'ekf' in c and 'y' in c and 'cm' in c and 'v' not in c]
cvy = [c for c in df.columns if 'ekf' in c and 'v' in c and 'y' in c and 'cm' in c]

flow_vx = [c for c in df.columns if 'flow' in c and 'vx' in c]
flow_vy = [c for c in df.columns if 'flow' in c and 'vy' in c]
obs_vx = [c for c in df.columns if 'obs' in c and 'vx' in c]
obs_vy = [c for c in df.columns if 'obs' in c and 'vy' in c]

tar_x = [c for c in df.columns if 'target' in c and 'x' in c]
tar_y = [c for c in df.columns if 'target' in c and 'y' in c]

print(f"Mapped: X={cx}, VX={cvx}, Y={cy}, VY={cvy}, F_VX={flow_vx}, F_VY={flow_vy}, O_VX={obs_vx}, O_VY={obs_vy}")

if cy and cvy:
    pos_y = df[cy[0]].values
    vel_y = df[cvy[0]].values
    
    # 1. Position integral consistency
    pos_y_pred = np.zeros_like(pos_y)
    pos_y_pred[0] = pos_y[0]
    for k in range(1, len(pos_y)):
        pos_y_pred[k] = pos_y_pred[k-1] + vel_y[k-1] * df['dt'].values[k]
    
    residual_y = pos_y - pos_y_pred
    print("\n--- 1. Position Integral Residual ---")
    print(f"Mean residual Y: {np.mean(residual_y):.4f} cm")
    print(f"RMS residual Y: {np.sqrt(np.mean(residual_y**2)):.4f} cm")
    print(f"End residual Y: {residual_y[-1]:.4f} cm")
    
    # Snippet
    print("Residual slice (t=5s to 5.1s):")
    slice_idx = (df['time'] >= 5.0) & (df['time'] <= 5.1)
    if slice_idx.sum() > 0:
        res_slice = pd.DataFrame({'time': df.loc[slice_idx, 'time'], 'pos_y': pos_y[slice_idx], 'vel_y': vel_y[slice_idx], 'pos_pred': pos_y_pred[slice_idx], 'res': residual_y[slice_idx]})
        print(res_slice.head())

if cx and cvx and cy and cvy:
    # 2. Low speed drift
    vx = df[cvx[0]].values
    vy = df[cvy[0]].values
    pos_x = df[cx[0]].values
    pos_y = df[cy[0]].values
    
    speed = np.sqrt(vx**2 + vy**2)
    low_speed_mask = speed < 2.0
    
    print("\n--- 2. Low Speed Drift ---")
    
    # Find contiguous low speed regions
    import itertools
    regions = []
    for k, g in itertools.groupby(enumerate(low_speed_mask), key=lambda x: x[1]):
        if k:
            indices = list(x[0] for x in g)
            duration = df['time'].iloc[indices[-1]] - df['time'].iloc[indices[0]]
            if duration >= 0.3:
                dx = pos_x[indices[-1]] - pos_x[indices[0]]
                dy = pos_y[indices[-1]] - pos_y[indices[0]]
                regions.append({'start_t': df['time'].iloc[indices[0]], 'duration': duration, 'dx': dx, 'dy': dy, 'drift_rate': np.sqrt(dx**2+dy**2)/duration})
                
    if regions:
        print(f"Found {len(regions)} low speed regions (>0.3s):")
        for r in regions[:5]:
            print(f"  t={r['start_t']:.2f}s, dur={r['duration']:.2f}s, dx={r['dx']:.2f}cm, dy={r['dy']:.2f}cm, drift={r['drift_rate']:.2f} cm/s")
    else:
        print("No low speed regions > 0.3s found.")

if flow_vy and cvy:
    print("\n--- 3. Flow vs EKF Vel Directional Bias ---")
    fvy = df[flow_vy[0]].values if flow_vy else df[obs_vy[0]].values
    evy = df[cvy[0]].values
    
    diff = fvy - evy
    pos_vy_mask = evy > 5.0
    neg_vy_mask = evy < -5.0
    
    print(f"Mean (Flow - EKF) when VY > 5 cm/s: {np.mean(diff[pos_vy_mask]):.4f} cm/s")
    print(f"Mean (Flow - EKF) when VY < -5 cm/s: {np.mean(diff[neg_vy_mask]):.4f} cm/s")

if tar_y and cy:
    print("\n--- 4. PE Anchor Analysis ---")
    target_y = df[tar_y[0]].values
    target_y_0 = target_y[0]
    pos_err = pos_y - target_y
    r_anchor = pos_y - np.mean(pos_y) # proxy for true anchor if it's oscillating around mean
    print(f"Initial target: {target_y_0:.2f}, Mean pos: {np.mean(pos_y):.2f}")
    
    # Sample PE calculation
    pe_err = 0.5 * 980 / 100 * pos_err**2
    pe_anchor = 0.5 * 980 / 100 * r_anchor**2
    
    print("PE snippet (t=10s to 10.1s):")
    slice_idx = (df['time'] >= 10.0) & (df['time'] <= 10.1)
    if slice_idx.sum() > 0:
        res_slice = pd.DataFrame({'time': df.loc[slice_idx, 'time'], 'pos_y': pos_y[slice_idx], 'tar_y': target_y[slice_idx], 'pos_err': pos_err[slice_idx], 'pe_err': pe_err[slice_idx], 'pe_anchor': pe_anchor[slice_idx]})
        print(res_slice.head())
