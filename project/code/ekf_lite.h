#include "zf_common_headfile.h"

#define EKF_LITE_TOF_GATE_CM 35.0f
#define EKF_LITE_FLOW_GATE_CMS 180.0f

typedef struct {
    float x;           // X轴位置估计值
    float y;           // Y轴位置估计值
    float z;           // Z轴高度估计值
    float vx;          // X轴速度估计值
    float vy;          // Y轴速度估计值
    float vz;          // Z轴速度估计值
    float tof_bias;    // TOF传感器高度偏差
    float flow_scale;  // 光流传感器比例因子
} ekf_lite_state_t;

typedef struct {
    uint8_t inited;        // EKF初始化标志位
    uint8_t tof_used;      // 本次更新是否使用了TOF数据
    uint8_t flow_used;     // 本次更新是否使用了光流数据
    uint8_t healthy;       // EKF状态是否健康正常
    float tof_innov;       // TOF高度新息(预测误差)
    float flow_innov_x;    // 光流X轴速度新息
    float flow_innov_y;    // 光流Y轴速度新息
} ekf_lite_health_t;


extern ekf_lite_state_t ekf_lite_state;
extern ekf_lite_health_t ekf_lite_health;

// Z轴(高度和垂直速度)的协方差矩阵 P (2x2矩阵)
extern float ekf_lite_p_z[2][2];
// XY轴(水平位置和速度)的协方差矩阵 P 的对角线元素 (x, y, vx, vy)
extern float ekf_lite_p_xy[4] ;
