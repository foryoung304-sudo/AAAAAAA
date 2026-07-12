#ifndef __LOC_CTRL_H__
#define __LOC_CTRL_H__

#include "zf_common_headfile.h"

// 控制限制参数 (保留)
#define MAX_HORIZONTAL_SPEED  50.0f     // 最大水平速度 (cm/s)
#define MAX_POS_CT_VAL         30.0f      // 最大位置控制输出 (即最大倾斜角度)
#define LOC_MAX_OUTPUT_ANGLE_DEG   8.0f
#define MAX_VEL_CT_VAL        50.0f

#define LOC_TARGET_VEL_SLEW_CM_S2  30.0f
#define LOC_TRAJ_ACCEL_CM_S2       30.0f
#define LOC_TARGET_ANGEL_SLEW_DEG_S2  40.0f
#define LOC_HOLD_TARGET_ANGEL_SLEW_DEG_S2  30.0f
#define LOC_HOLD_VEL_ERR_LPF_ALPHA_MIN 0.30f
#define LOC_HOLD_VEL_ERR_LPF_ALPHA_MAX 0.60f
#define LOC_HOLD_VEL_ERR_DEADBAND_MAX_CM_S 1.0f
#define LOC_HOLD_VEL_ERR_DEADBAND_MIN_CM_S 0.5f
#define LOC_HOLD_ERR_RELAX_CM 8.0f
#define LOC_HOLD_ERR_ACTIVE_CM 18.0f
#define LOC_ENABLE_HEIGHT_CM       50.0f
#define LOC_ENABLE_TARGET_MARGIN_CM 5.0f
#define LOC_ENABLE_VZ_MAX_CM_S     4.0f
#define LOC_ENABLE_HORIZ_VEL_MAX_CM_S  8.0f  /* hold 进入时最大横向合速度 */
#define LOC_HOLD_RELEASE_MARGIN_CM 8.0f
#define LOC_DAMPING_MAX_OUTPUT_ANGLE_DEG 5.0f
#define LOC_DAMPING_MIN_OUTPUT_ANGLE_DEG 2.0f
#define LOC_DAMPING_VZ_FULL_CM_S   4.0f
#define LOC_DAMPING_VZ_REDUCE_CM_S 10.0f
#define LOC_DAMPING_VEL_ERR_SCALE   1.00f
#define LOC_BLEND_TIME_S           0.5f
#define LOC_BRAKE_TIME_S           0.6f
#define LOC_BRAKE_BLEND_TIME_S     0.45f

// 位置控制结构体
typedef struct {
    // 位置环PID控制器（外环）
    pid_param_t pos_pid[2];   // X, Y
    
    // 速度环PID控制器（内环）
    pid_param_t vel_pid[2];   // Vx, Vy

    
} location_control_t;

extern location_control_t loc_ctrl;
    
// 位置环数据结构（外环）
typedef struct {
    float exp_pos_x;
    float exp_pos_y;
    float fb_pos_x;
    float fb_pos_y;
    float exp_vel_x;
    float exp_vel_y;
} loc_2l_ct_t;
extern loc_2l_ct_t loc_2l_ct;

// 速度环数据结构（内环）
typedef struct {
    float exp_vel_x;
    float exp_vel_y;
    float fb_vel_x;
    float fb_vel_y;
    float vel_err_x_body;
    float vel_err_y_body;
    float raw_target_roll;
    float raw_target_pitch;
    float loc_weight;
    uint8_t loc_ready;
    uint8_t loc_hold_ready;
} loc_1l_ct_t;
extern loc_1l_ct_t loc_1l_ct;
// 控制输出
typedef struct {
    float roll_adj;   // Roll修正量
    float pitch_adj;  // Pitch修正量
} loc_output_t;

extern loc_output_t loc_output;
// 函数声明

// 初始化
void loc_ctrl_init(void);

    // 外环：位置控制
void loc_2level_ctrl(float dT_s);
    
    // 内环：速度控制
void loc_1level_ctrl(float dT_s);
// 主控制更新
void loc_ctrl_update(float dT_s);



#endif /* __LOC_CTRL_H__ */
