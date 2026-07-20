#ifndef __LOC_CTRL_H__
#define __LOC_CTRL_H__

#include "zf_common_headfile.h"

// 控制限制参数 (保留)
#define MAX_HORIZONTAL_SPEED  50.0f     // 最大水平速度 (cm/s)
#define MAX_POS_CT_VAL         30.0f      // 最大位置控制输出 (即最大倾斜角度)
#define LOC_MAX_OUTPUT_ANGLE_DEG   8.0f
#define MAX_VEL_CT_VAL        50.0f
#define LOC_GRAVITY_CM_S2          980.665f
#define LOC_MAX_HORIZONTAL_ACCEL_CM_S2  138.0f

#define LOC_POS_CORRECTION_LIMIT_CM_S 15.0f

#define LOC_TERMINAL_POS_CORRECTION_LIMIT_CM_S 12.0f

#define LOC_TERMINAL_TOTAL_VEL_LIMIT_CM_S       15.0f
#define LOC_TERMINAL_APPROACH_RADIUS_CM         40.0f
#define LOC_TERMINAL_CRUISE_RADIUS_CM          100.0f
#define LOC_TERMINAL_VEL_DAMPING_SCALE           1.65f

#define LOC_PROFILE_NEAR_SPEED_CM_S    5.0f
#define LOC_PROFILE_FAR_SPEED_CM_S    45.0f
#define LOC_PROFILE_BLEND_START_CM    20.0f
#define LOC_PROFILE_BLEND_END_CM      80.0f
#define LOC_TOTAL_VEL_LIMIT_CM_S      45.0f
#define LOC_PROFILE_LEASH_START_CM    12.0f
#define LOC_PROFILE_LEASH_MAX_CM      40.0f

#define LOC_PROFILE_LEASH_MIN_SPEED_SCALE 0.40f
#define LOC_TARGET_VEL_SLEW_CM_S2  30.0f

#define LOC_RECOVERY_BRAKE_MARGIN_CM_S      3.0f
#define LOC_RECOVERY_BRAKE_VEL_SLEW_CM_S2 120.0f
#define LOC_TRAJ_ACCEL_CM_S2       30.0f
#define LOC_TARGET_ANGEL_SLEW_DEG_S2  40.0f
#define LOC_HOLD_TARGET_ANGEL_SLEW_DEG_S2  30.0f
#define LOC_HOLD_VEL_ERR_LPF_ALPHA_MIN 0.45f
#define LOC_HOLD_VEL_ERR_LPF_ALPHA_MAX 0.60f
#define LOC_HOLD_VEL_ERR_DEADBAND_MAX_CM_S 1.0f
#define LOC_HOLD_VEL_ERR_DEADBAND_MIN_CM_S 0.5f
#define LOC_VEL_I_OPPOSE_DECAY             0.96f
#define LOC_HOLD_ERR_RELAX_CM 8.0f
#define LOC_HOLD_ERR_ACTIVE_CM 18.0f
#define LOC_ENABLE_HEIGHT_CM       50.0f
#define LOC_HOLD_ENABLE_HEIGHT_CM  65.0f
#define LOC_AUTO_LOW_ENABLE_HEIGHT_CM      20.0f
#define LOC_AUTO_LOW_HOLD_HEIGHT_CM        35.0f
#define LOC_AUTO_LOW_MAX_OUTPUT_ANGLE_DEG   3.0f
#define LOC_AUTO_LOW_ENABLE_VZ_MAX_CM_S    35.0f
#define LOC_ENABLE_VZ_MAX_CM_S    12.0f
#define LOC_ENABLE_HORIZ_VEL_MAX_CM_S  8.0f  /* hold 进入时最大横向合速度 */
#define LOC_ENABLE_ATT_MAX_DEG     8.0f
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
    float profile_vel_x;
    float profile_vel_y;
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
    float dynamic_target_roll;
    float dynamic_target_pitch;
    float target_accel_x_body;
    float target_accel_y_body;
    float horizontal_bias_roll;
    float horizontal_bias_pitch;
    float loc_weight;
    float recovery_closing_speed;
    float recovery_stop_speed;
    uint8_t recovery_brake_active;
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
