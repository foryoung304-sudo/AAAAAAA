import re
import math
import numpy as np
from scipy.optimize import brent

# 您提供的 debug 数据
data_str = """
DEBUG: tof_dist_cm=72.90, cos_r=0.999, cos_p=0.999, current_height=72.38, pitch=2.23, roll=1.81
DEBUG: tof_dist_cm=72.80, cos_r=1.000, cos_p=0.999, current_height=72.28, pitch=2.23, roll=1.81
DEBUG: tof_dist_cm=72.70, cos_r=1.000, cos_p=0.999, current_height=72.28, pitch=2.23, roll=1.81
DEBUG: tof_dist_cm=72.70, cos_r=1.000, cos_p=0.999, current_height=72.24, pitch=2.23, roll=1.81
DEBUG: tof_dist_cm=72.70, cos_r=1.000, cos_p=0.999, current_height=72.31, pitch=2.23, roll=1.81
DEBUG: tof_dist_cm=72.80, cos_r=1.000, cos_p=0.999, current_height=72.30, pitch=2.24, roll=1.80
DEBUG: tof_dist_cm=72.70, cos_r=1.000, cos_p=0.999, current_height=72.42, pitch=2.23, roll=1.80
DEBUG: tof_dist_cm=72.30, cos_r=1.000, cos_p=0.999, current_height=72.13, pitch=2.24, roll=1.80
DEBUG: tof_dist_cm=73.00, cos_r=1.000, cos_p=0.999, current_height=72.25, pitch=2.24, roll=1.80
DEBUG: tof_dist_cm=73.20, cos_r=1.000, cos_p=0.999, current_height=72.44, pitch=2.24, roll=1.80
DEBUG: tof_dist_cm=73.10, cos_r=1.000, cos_p=0.999, current_height=72.36, pitch=2.24, roll=1.80
DEBUG: tof_dist_cm=72.50, cos_r=1.000, cos_p=0.999, current_height=72.19, pitch=2.24, roll=1.80
DEBUG: tof_dist_cm=73.00, cos_r=1.000, cos_p=0.999, current_height=72.36, pitch=2.24, roll=1.80
DEBUG: tof_dist_cm=72.50, cos_r=1.000, cos_p=0.999, current_height=72.24, pitch=2.24, roll=1.81
DEBUG: tof_dist_cm=72.60, cos_r=1.000, cos_p=0.999, current_height=72.29, pitch=2.24, roll=1.80
DEBUG: tof_dist_cm=72.60, cos_r=1.000, cos_p=0.999, current_height=72.35, pitch=2.24, roll=1.80
DEBUG: tof_dist_cm=72.40, cos_r=1.000, cos_p=0.999, current_height=72.19, pitch=2.24, roll=1.80
DEBUG: tof_dist_cm=72.30, cos_r=1.000, cos_p=0.999, current_height=72.18, pitch=2.24, roll=1.80
DEBUG: tof_dist_cm=73.10, cos_r=1.000, cos_p=0.999, current_height=72.42, pitch=2.24, roll=1.80
DEBUG: tof_dist_cm=73.00, cos_r=1.000, cos_p=0.998, current_height=72.48, pitch=3.23, roll=1.03
DEBUG: tof_dist_cm=72.90, cos_r=1.000, cos_p=0.999, current_height=72.47, pitch=3.03, roll=1.19
DEBUG: tof_dist_cm=72.50, cos_r=1.000, cos_p=0.999, current_height=72.18, pitch=3.03, roll=1.24
DEBUG: tof_dist_cm=72.50, cos_r=1.000, cos_p=0.999, current_height=72.18, pitch=3.03, roll=1.20
DEBUG: tof_dist_cm=72.80, cos_r=1.000, cos_p=0.999, current_height=72.48, pitch=3.01, roll=1.15
DEBUG: tof_dist_cm=73.40, cos_r=1.000, cos_p=0.998, current_height=72.61, pitch=3.17, roll=0.64
DEBUG: tof_dist_cm=72.70, cos_r=1.000, cos_p=0.999, current_height=72.39, pitch=3.11, roll=0.88
DEBUG: tof_dist_cm=73.20, cos_r=1.000, cos_p=0.999, current_height=72.55, pitch=3.11, roll=0.98
DEBUG: tof_dist_cm=73.10, cos_r=1.000, cos_p=0.998, current_height=72.43, pitch=3.87, roll=0.92
DEBUG: tof_dist_cm=74.40, cos_r=1.000, cos_p=0.995, current_height=72.80, pitch=5.83, roll=1.29
DEBUG: tof_dist_cm=75.10, cos_r=1.000, cos_p=0.992, current_height=73.13, pitch=7.21, roll=1.65
DEBUG: tof_dist_cm=75.70, cos_r=0.999, cos_p=0.986, current_height=73.14, pitch=9.69, roll=1.95
DEBUG: tof_dist_cm=76.60, cos_r=0.999, cos_p=0.981, current_height=73.16, pitch=11.23, roll=1.97
DEBUG: tof_dist_cm=77.10, cos_r=0.999, cos_p=0.976, current_height=73.32, pitch=12.56, roll=1.85
DEBUG: tof_dist_cm=78.40, cos_r=1.000, cos_p=0.969, current_height=73.48, pitch=14.40, roll=1.72
DEBUG: tof_dist_cm=79.20, cos_r=0.999, cos_p=0.960, current_height=73.43, pitch=16.19, roll=1.88
DEBUG: tof_dist_cm=80.40, cos_r=0.999, cos_p=0.953, current_height=73.71, pitch=17.68, roll=1.83
DEBUG: tof_dist_cm=81.00, cos_r=0.999, cos_p=0.947, current_height=73.70, pitch=18.82, roll=1.83
DEBUG: tof_dist_cm=80.90, cos_r=1.000, cos_p=0.940, current_height=73.53, pitch=19.99, roll=1.79
DEBUG: tof_dist_cm=83.30, cos_r=1.000, cos_p=0.931, current_height=74.03, pitch=21.37, roll=1.76
DEBUG: tof_dist_cm=83.90, cos_r=1.000, cos_p=0.923, current_height=73.97, pitch=22.57, roll=1.67
DEBUG: tof_dist_cm=85.50, cos_r=0.999, cos_p=0.917, current_height=74.32, pitch=23.44, roll=1.82
DEBUG: tof_dist_cm=84.90, cos_r=1.000, cos_p=0.914, current_height=74.07, pitch=23.97, roll=1.73
DEBUG: tof_dist_cm=85.60, cos_r=0.999, cos_p=0.908, current_height=74.12, pitch=24.74, roll=1.91
DEBUG: tof_dist_cm=86.60, cos_r=0.999, cos_p=0.906, current_height=74.41, pitch=25.02, roll=1.97
DEBUG: tof_dist_cm=85.30, cos_r=0.999, cos_p=0.909, current_height=74.05, pitch=24.57, roll=1.84
DEBUG: tof_dist_cm=83.30, cos_r=0.999, cos_p=0.926, current_height=73.82, pitch=22.12, roll=1.94
DEBUG: tof_dist_cm=81.70, cos_r=1.000, cos_p=0.939, current_height=73.73, pitch=20.13, roll=1.69
DEBUG: tof_dist_cm=80.90, cos_r=1.000, cos_p=0.951, current_height=73.77, pitch=18.07, roll=1.51
DEBUG: tof_dist_cm=78.70, cos_r=1.000, cos_p=0.964, current_height=73.39, pitch=15.39, roll=1.17
DEBUG: tof_dist_cm=76.30, cos_r=1.000, cos_p=0.980, current_height=73.16, pitch=11.40, roll=1.22
DEBUG: tof_dist_cm=75.10, cos_r=1.000, cos_p=0.990, current_height=72.98, pitch=8.21, roll=1.60
DEBUG: tof_dist_cm=73.80, cos_r=1.000, cos_p=0.996, current_height=72.89, pitch=5.32, roll=1.78
DEBUG: tof_dist_cm=71.80, cos_r=0.999, cos_p=1.000, current_height=72.03, pitch=0.46, roll=2.19
DEBUG: tof_dist_cm=70.60, cos_r=0.999, cos_p=0.999, current_height=71.07, pitch=-2.55, roll=2.37
DEBUG: tof_dist_cm=70.20, cos_r=0.999, cos_p=0.996, current_height=70.75, pitch=-4.92, roll=2.49
DEBUG: tof_dist_cm=69.70, cos_r=0.999, cos_p=0.989, current_height=70.34, pitch=-8.37, roll=2.11
DEBUG: tof_dist_cm=70.40, cos_r=0.999, cos_p=0.986, current_height=70.93, pitch=-9.65, roll=1.95
DEBUG: tof_dist_cm=70.80, cos_r=0.999, cos_p=0.984, current_height=71.24, pitch=-10.35, roll=2.02
DEBUG: tof_dist_cm=70.80, cos_r=0.999, cos_p=0.985, current_height=71.24, pitch=-9.81, roll=1.99
DEBUG: tof_dist_cm=69.40, cos_r=0.999, cos_p=0.995, current_height=70.18, pitch=-5.97, roll=2.05
DEBUG: tof_dist_cm=71.10, cos_r=0.999, cos_p=0.999, current_height=71.11, pitch=-2.52, roll=2.03
DEBUG: tof_dist_cm=71.80, cos_r=0.999, cos_p=1.000, current_height=71.67, pitch=-0.10, roll=2.06
DEBUG: tof_dist_cm=70.30, cos_r=0.999, cos_p=0.997, current_height=70.94, pitch=-4.75, roll=2.40
DEBUG: tof_dist_cm=70.10, cos_r=0.999, cos_p=0.988, current_height=70.41, pitch=-8.70, roll=2.96
DEBUG: tof_dist_cm=71.00, cos_r=0.998, cos_p=0.982, current_height=71.15, pitch=-10.81, roll=3.48
DEBUG: tof_dist_cm=71.40, cos_r=0.998, cos_p=0.979, current_height=71.45, pitch=-11.87, roll=3.55
DEBUG: tof_dist_cm=71.00, cos_r=0.998, cos_p=0.974, current_height=71.24, pitch=-13.18, roll=3.69
DEBUG: tof_dist_cm=71.10, cos_r=0.998, cos_p=0.972, current_height=71.18, pitch=-13.61, roll=3.64
DEBUG: tof_dist_cm=71.50, cos_r=0.998, cos_p=0.974, current_height=71.16, pitch=-13.16, roll=3.61
DEBUG: tof_dist_cm=70.30, cos_r=0.998, cos_p=0.986, current_height=71.06, pitch=-9.67, roll=3.17
DEBUG: tof_dist_cm=70.40, cos_r=0.999, cos_p=0.997, current_height=70.54, pitch=-4.27, roll=2.84
DEBUG: tof_dist_cm=72.20, cos_r=0.999, cos_p=0.999, current_height=71.43, pitch=1.85, roll=2.89
DEBUG: tof_dist_cm=76.50, cos_r=0.999, cos_p=0.979, current_height=72.92, pitch=11.63, roll=3.01
DEBUG: tof_dist_cm=79.10, cos_r=0.999, cos_p=0.964, current_height=73.50, pitch=15.48, roll=2.85
DEBUG: tof_dist_cm=80.60, cos_r=0.999, cos_p=0.948, current_height=73.61, pitch=18.53, roll=2.94
DEBUG: tof_dist_cm=83.00, cos_r=0.999, cos_p=0.932, current_height=74.01, pitch=21.26, roll=2.82
DEBUG: tof_dist_cm=83.70, cos_r=0.999, cos_p=0.921, current_height=73.78, pitch=22.99, roll=2.65
DEBUG: tof_dist_cm=86.50, cos_r=0.999, cos_p=0.908, current_height=74.18, pitch=24.76, roll=2.45
DEBUG: tof_dist_cm=87.50, cos_r=0.999, cos_p=0.897, current_height=74.42, pitch=26.28, roll=2.29
DEBUG: tof_dist_cm=85.00, cos_r=0.999, cos_p=0.919, current_height=74.50, pitch=23.23, roll=2.43
DEBUG: tof_dist_cm=81.50, cos_r=0.999, cos_p=0.944, current_height=73.89, pitch=19.24, roll=2.31
DEBUG: tof_dist_cm=79.30, cos_r=0.999, cos_p=0.956, current_height=73.48, pitch=16.96, roll=1.82
DEBUG: tof_dist_cm=78.90, cos_r=0.999, cos_p=0.959, current_height=73.46, pitch=16.37, roll=1.86
DEBUG: tof_dist_cm=79.10, cos_r=0.999, cos_p=0.960, current_height=73.36, pitch=16.25, roll=1.92
DEBUG: tof_dist_cm=79.10, cos_r=0.999, cos_p=0.961, current_height=73.51, pitch=16.15, roll=1.94
DEBUG: tof_dist_cm=79.70, cos_r=0.999, cos_p=0.961, current_height=73.72, pitch=16.06, roll=1.96
DEBUG: tof_dist_cm=79.10, cos_r=0.999, cos_p=0.961, current_height=73.53, pitch=15.99, roll=1.97
DEBUG: tof_dist_cm=78.80, cos_r=0.999, cos_p=0.962, current_height=73.43, pitch=15.94, roll=1.97
DEBUG: tof_dist_cm=79.20, cos_r=0.999, cos_p=0.962, current_height=73.62, pitch=15.89, roll=1.97
DEBUG: tof_dist_cm=78.70, cos_r=0.999, cos_p=0.962, current_height=73.53, pitch=15.85, roll=1.97
DEBUG: tof_dist_cm=78.60, cos_r=0.999, cos_p=0.962, current_height=73.44, pitch=15.83, roll=1.97
DEBUG: tof_dist_cm=78.70, cos_r=0.999, cos_p=0.962, current_height=73.40, pitch=15.81, roll=1.96
DEBUG: tof_dist_cm=78.50, cos_r=0.999, cos_p=0.962, current_height=73.23, pitch=15.79, roll=1.96
DEBUG: tof_dist_cm=78.80, cos_r=0.999, cos_p=0.962, current_height=73.44, pitch=15.79, roll=1.95
DEBUG: tof_dist_cm=79.30, cos_r=0.999, cos_p=0.962, current_height=73.52, pitch=15.78, roll=1.95
DEBUG: tof_dist_cm=79.10, cos_r=0.999, cos_p=0.962, current_height=73.43, pitch=15.77, roll=1.94
DEBUG: tof_dist_cm=79.00, cos_r=0.999, cos_p=0.962, current_height=73.49, pitch=15.76, roll=1.94
"""

# 解析数据
pitches = []
heights = []
rolls = []

for line in data_str.strip().split('\n'):
    match = re.search(r'current_height=([\d.]+), pitch=([-\d.]+), roll=([-\d.]+)', line)
    if match:
        heights.append(float(match.group(1)))
        pitches.append(float(match.group(2)))
        rolls.append(float(match.group(3)))

pitches = np.array(pitches)
heights = np.array(heights)
rolls = np.array(rolls)

print(f"数据点数: {len(pitches)}")
print(f"pitch 范围: {pitches.min():.2f}° ~ {pitches.max():.2f}°")
print(f"roll 范围: {rolls.min():.2f}° ~ {rolls.max():.2f}°")
print(f"height 范围: {heights.min():.2f} ~ {heights.max():.2f} cm")
print(f"height 变化: {heights.max() - heights.min():.2f} cm")

# 转换为弧度
pitch_rad = np.deg2rad(pitches)
roll_rad = np.deg2rad(rolls)

# 假设高度补偿公式：
# height_compensated = height_raw - (TOF_OFFSET_X * sin(pitch) - TOF_OFFSET_Y * sin(roll))
# 由于 current_height 已经是补偿后的值，我们需要反推最优的 TOF_OFFSET_X
# 目标：最小化 compensated_height 随 pitch 的变化

TOF_OFFSET_Y = -0.5  # 已知

def objective(offset_x):
    # 反补偿：恢复到原始高度
    height_raw = heights + offset_x * np.sin(pitch_rad) - TOF_OFFSET_Y * np.sin(roll_rad)
    
    # 拟合 height_raw = a + b*pitch_rad 的形式，最小化 b 的绝对值
    coeffs = np.polyfit(pitch_rad, height_raw, 1)
    slope = coeffs[0]  # 高度对 pitch 的斜率
    
    return slope ** 2  # 目标：使斜率接近 0

# 搜索最优值
result = brent(objective, (5, 15), full_output=True)
optimal_offset_x = result[0]
min_cost = result[1]

print(f"\n=== 最优补偿参数 ===")
print(f"TOF_OFFSET_X (最优): {optimal_offset_x:.3f} cm")
print(f"当前设置: 9.0 cm")
print(f"建议调整: {optimal_offset_x:.1f} cm")

# 验证效果
height_raw_optimal = heights + optimal_offset_x * np.sin(pitch_rad) - TOF_OFFSET_Y * np.sin(roll_rad)
coeffs_opt = np.polyfit(pitch_rad, height_raw_optimal, 1)
print(f"\n调整后的 height-pitch 斜率: {coeffs_opt[0]:.4f} cm/rad ({coeffs_opt[0]*57.3:.4f} cm/deg)")
print(f"调整后的高度变化: {height_raw_optimal.max() - height_raw_optimal.min():.2f} cm")

# 对比当前设置
height_raw_current = heights + 9.0 * np.sin(pitch_rad) - TOF_OFFSET_Y * np.sin(roll_rad)
coeffs_current = np.polyfit(pitch_rad, height_raw_current, 1)
print(f"\n当前设置 (9.0 cm) 的 height-pitch 斜率: {coeffs_current[0]:.4f} cm/rad ({coeffs_current[0]*57.3:.4f} cm/deg)")
print(f"当前设置的高度变化: {height_raw_current.max() - height_raw_current.min():.2f} cm")
