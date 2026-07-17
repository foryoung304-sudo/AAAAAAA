import sys
import numpy as np
import pandas as pd
from scipy import signal

log_file = sys.argv[1] if len(sys.argv) > 1 else "E:/CYT4BB7_Library-master/CYT4BB7_Library-master/LOGS/latest.txt"

if log_file.endswith('.csv'):
    df = pd.read_csv(log_file)
    fs = 1.0 / (df['time_us'].diff().mean() / 1e6)
    duration = len(df) / fs
else:
    # Text fallback
    data = {
        'time': [], 'pos_err_y_e': [], 'vel_tgt_y_e': [], 'ekf_vy_e': [], 'vel_err_y_b': [],
        'roll_tgt': [], 'roll_cur': [], 'flow_obs_vy_e': [], 'imu_acc_body_y_m_s2': []
    }
    time_ms = 0
    import re
    cur_frame = {}
    with open(log_file, 'r', encoding='utf-8', errors='ignore') as f:
        for line in f:
            line = line.strip()
            if "DESKTOP DEBUG DASHBOARD" in line:
                if 'pos_err_y_e' in cur_frame:
                    data['time'].append(time_ms / 1000.0)
                    data['pos_err_y_e'].append(cur_frame.get('pos_err_y_e', 0))
                    data['vel_tgt_y_e'].append(cur_frame.get('vel_tgt_y_e', 0))
                    data['ekf_vy_e'].append(cur_frame.get('ekf_vy_e', 0))
                    data['vel_err_y_b'].append(cur_frame.get('vel_err_y_b', 0))
                    data['roll_tgt'].append(cur_frame.get('roll_tgt', 0))
                    data['roll_cur'].append(cur_frame.get('roll_cur', 0))
                    data['flow_obs_vy_e'].append(cur_frame.get('flow_obs_vy_e', 0))
                    data['imu_acc_body_y_m_s2'].append(cur_frame.get('imu_acc_body_y_m_s2', 0))
                    time_ms += 20.8
                cur_frame = {}
                
            m = re.search(r"Pos Y \(cm\)\s*:\s*Cur:\s*([-.\d]+)\s*\|\s*Tgt:\s*([-.\d]+)\s*\|\s*Err:\s*([-.\d]+)", line)
            if m: cur_frame['pos_err_y_e'] = float(m.group(3))
            
            m = re.search(r"Vel Y \(cm/s\)\s*:\s*Cur:\s*([-.\d]+)\s*\|\s*Tgt:\s*([-.\d]+)\s*\|\s*Err:\s*([-.\d]+)", line)
            if m:
                cur_frame['ekf_vy_e'] = float(m.group(1))
                cur_frame['vel_tgt_y_e'] = float(m.group(2))
                cur_frame['vel_err_y_b'] = float(m.group(3))
                
            m = re.search(r"Roll \(deg\)\s*:\s*Cur:\s*([-.\d]+)\s*\|\s*Tgt:\s*([-.\d]+)", line)
            if m:
                cur_frame['roll_cur'] = float(m.group(1))
                cur_frame['roll_tgt'] = float(m.group(2))
                
            m = re.search(r"obs_xy:([-.\d]+)/([-.\d]+)", line)
            if m:
                cur_frame['flow_obs_vy_e'] = float(m.group(2))
                
            m = re.search(r"linear_acc_body_y_m_s2:\s*([-.\d]+)", line)
            if m:
                cur_frame['imu_acc_body_y_m_s2'] = float(m.group(1))
    
    df = pd.DataFrame(data)
    fs = 48.0
    duration = len(df) / fs

def calc_metrics_at_freq(s1, s2, fs, target_f=None):
    s1 = np.asarray(s1)
    s2 = np.asarray(s2)
    s1 = s1 - np.mean(s1)
    s2 = s2 - np.mean(s2)
    
    f, pxy = signal.csd(s1, s2, fs=fs, nperseg=256)
    f, pxx = signal.welch(s1, fs=fs, nperseg=256)
    f, pyy = signal.welch(s2, fs=fs, nperseg=256)
    
    coh = np.abs(pxy)**2 / (pxx * pyy)
    phase = np.angle(pxy, deg=True)
    gain = np.abs(pxy) / pxx
    
    if target_f is None:
        idx = np.argmax(pxx) # Use dominant frequency of s1
    else:
        idx = np.argmin(np.abs(f - target_f))
        
    return f[idx], phase[idx], coh[idx], gain[idx]

print("\n--- 相位频域诊断 ---")
print(f"数据时长: {duration:.2f}s, 采样率: {fs:.2f}Hz")

# Find dominant frequency using Welch on pos_err_y
pos_err = np.asarray(df['pos_err_y_e'], dtype=float)
pos_err = pos_err - np.mean(pos_err)
f_dom, pxx_dom = signal.welch(pos_err, fs=fs, nperseg=256)
idx_dom = np.argmax(pxx_dom)
dom_f = f_dom[idx_dom]
dom_cycles = dom_f * duration

print(f"局部峰值频率: {dom_f:.3f} Hz (包含 {dom_cycles:.1f} 个周期)")
if dom_cycles < 3.0:
    print("!! 警告: 该频率有效周期数 < 3.0，样本不足，无法稳定估计，不输出此频率的诊断结论 !!")
    eval_freqs = [0.341]
else:
    eval_freqs = [0.341, dom_f]

for eval_f in eval_freqs:
    freq_cycles = eval_f * duration
    print(f"\n[评估频率: {eval_f:.3f} Hz] - 有效周期数: {freq_cycles:.1f}")
    if freq_cycles < 3.0:
        print("!! 样本不足 !! 数据量不足以在此频率进行稳定可靠的闭环相移推断。")
    
    # 1
    f1, ph1, coh1, g1 = calc_metrics_at_freq(df['pos_err_y_e'], df['vel_tgt_y_e'], fs, eval_f)
    print(f"1. Pos_Err -> Vel_Tgt : 基线 0° | 测得 {ph1:7.2f}° | 额外 {ph1-0:7.2f}° | coh={coh1:.2f} | gain={g1:.3f}")
    # 2
    f2, ph2, coh2, g2 = calc_metrics_at_freq(df['vel_err_y_b'], df['roll_tgt'], fs, eval_f)
    print(f"2. Vel_Err -> Roll_Tgt: 基线 0° | 测得 {ph2:7.2f}° | 额外 {ph2-0:7.2f}° | coh={coh2:.2f} | gain={g2:.3f}")
    # 3
    f3, ph3, coh3, g3 = calc_metrics_at_freq(df['roll_tgt'], df['roll_cur'], fs, eval_f)
    print(f"3. Roll_Tgt -> Roll_Cur:基线 0° | 测得 {ph3:7.2f}° | 额外 {ph3-0:7.2f}° | coh={coh3:.2f} | gain={g3:.3f}")
    
    # NEW: Roll_Cur -> IMU_Acc_Body_Y (-180 baseline)
    f_imu, ph_imu, coh_imu, g_imu = calc_metrics_at_freq(df['roll_cur'], df['imu_acc_body_y_m_s2'], fs, eval_f)
    ph_imu_norm = ph_imu if ph_imu > 0 else ph_imu + 360 # shift to 0-360 for relative math
    ext_imu = ph_imu - (-180)
    if ext_imu > 180: ext_imu -= 360
    if ext_imu < -180: ext_imu += 360
    print(f"*. Roll_Cur->IMU_Acc_Y : 基线 -180°| 测得 {ph_imu:7.2f}° | 额外 {ext_imu:7.2f}° | coh={coh_imu:.2f} | gain={g_imu:.3f}")

    # 4
    f4, ph4, coh4, g4 = calc_metrics_at_freq(df['roll_cur'], df['flow_obs_vy_e'], fs, eval_f)
    ph4_norm = ph4 if ph4 > -180 else ph4 + 360
    ext4 = ph4_norm - (-90)
    if ext4 > 180: ext4 -= 360
    if ext4 < -180: ext4 += 360
    print(f"4. Roll_Cur -> Flow_Vy: 基线 -90° | 测得 {ph4_norm:7.2f}° | 额外 {ext4:7.2f}° | coh={coh4:.2f} | gain={g4:.3f}")
    # 5
    f5, ph5, coh5, g5 = calc_metrics_at_freq(df['flow_obs_vy_e'], df['ekf_vy_e'] if 'ekf_vy_e' in df.columns else df['ekf_vy_e'], fs, eval_f)
    print(f"5. Flow_Vy -> Vel_Cur : 基线 0° | 测得 {ph5:7.2f}° | 额外 {ph5-0:7.2f}° | coh={coh5:.2f} | gain={g5:.3f}")


