#ifndef __ALT_CTRL_H__
#define __ALT_CTRL_H__

#include "zf_common_headfile.h"


// 控制限制参数
#define MAX_CLIMB_SPEED    15.0f     // 最大上升速度 (cm/s)
#define MAX_DESCEND_SPEED  10.0f     // 最大下降速度 (cm/s)
#define MAX_THROTTLE_OUTPUT  10.0f      // 高度环相对悬停油门的最大修正量
#define ALT_LANDED_THR_MARGIN  3.0f
#define ALT_BRAKE_BELOW_TARGET_ALLOW_CM  0.5f
#define ALT_BRAKE_THROTTLE_DROP          4.0f
#define ALT_TAKEOFF_HEIGHT_CM            10.0f
#define ALT_TRANSITION_HEIGHT_CM         30.0f
#define ALT_AUTHORITY_MIN                0.85f
#define ALT_THROTTLE_AUTHORITY_RANGE     5.0f
#define ALT_MAX_THROTTLE_HEADROOM        4.0f
#define ALT_VEL_DAMPING_GAIN             0.036f
#define ALT_DYN_GAIN_REF_VOLTAGE         10.5f
#define ALT_DYN_GAIN_SCALE_MIN           0.75f
#define ALT_DYN_GAIN_SCALE_MAX           1.20f
#define ALT_VEL_CMD_LIMIT_CM_S           10.0f
#define ALT_VEL_CMD_SLEW_CM_S2           8.0f
#define ALT_VEL_I_DISABLED_DECAY         1.0f
// Only unload an I term that opposes the current vertical-velocity error.
// 0.99 at 50 Hz is deliberately gentle: it preserves normal descent/climb
// bias while preventing stale I from carrying through a height crossing.
#define ALT_VEL_I_OPPOSE_DECAY           0.99f
#define ALT_TRAJ_MAX_CLIMB_CM_S          10.0f
#define ALT_TRAJ_MAX_DESCEND_CM_S        8.0f
#define ALT_TRAJ_ACCEL_CM_S2             20.0f
#define ALT_TERMINAL_DECAY_DIST_CM       4.0f
#define ALT_POS_CORR_LIMIT_CM_S          6.0f
#define ALT_PROFILE_MAX_LEAD_CM          0.0f
#define ALT_TRAJ_START_THR_MARGIN        0.5f
#define ALT_TRAJ_START_READY_S           0.10f
#define ALT_SINK_ARREST_ERR_CM           2.0f
#define ALT_SINK_ARREST_VEL_CM_S         2.0f
#define ALT_SINK_ARREST_MIN_CMD_CM_S     4.0f
#define ALT_SINK_ARREST_MAX_CMD_CM_S     7.0f

#define ALT_THROTTLE_SLEW                50.0f

#define ALT_HOVER_TRIM_VOLT_GAIN         3.0f
#define ALT_HOVER_TRIM_MAX               4.5f
#define ALT_HOVER_TRIM_SLEW              0.5f
#define ALT_NEAR_GROUND_BRAKE_HEIGHT_CM  13.0f
#define ALT_NEAR_GROUND_BRAKE_GAIN       0.0f
#define ALT_NEAR_GROUND_BRAKE_LIMIT      1.20f
#define ALT_VEL_FB_START_HEIGHT_CM       8.0f
#define ALT_VEL_FB_FULL_HEIGHT_CM        16.0f
#define ALT_VEL_FB_MIN_WEIGHT            1.00f
#define ALT_VEL_FEEDFORWARD_GAIN         0.0f
#define ALT_VEL_FEEDFORWARD_LIMIT        0.8f
#define ALT_ACC_FEEDFORWARD_GAIN         0.03f
#define ALT_ACC_FEEDFORWARD_LIMIT        0.5f
#define ALT_TAKEOFF_BOOST_HEIGHT_CM      8.5f
#define ALT_TAKEOFF_MIN_HEADROOM         1.2f
#define ALT_TAKEOFF_HEADROOM_SLEW        2.0f
#define ALT_TAKEOFF_BOOST_FADE_VEL_CM_S  0.8f
#define ALT_TAKEOFF_VEL_CMD_LIMIT_CM_S   3.0f
#define ALT_TAKEOFF_VEL_LIMIT_HEIGHT_CM  13.0f
#define ALT_TAKEOFF_BOOST_MIN_ERR_CM     0.8f
#define ALT_TAKEOFF_COMPLETE_ERR_CM      0.4f
#define ALT_TAKEOFF_COMPLETE_HEIGHT_CM   10.0f
#define ALT_LANDED_DEBOUNCE_S            0.35f
#define ALT_SPOOLING_TIMEOUT_S           0.50f
#define AUTO_LAND_DESCEND_SPEED_CM_S     10.0f
#define AUTO_LAND_MIN_DESCEND_CMD_CM_S   3.0f
#define AUTO_LAND_MIN_DESCEND_HEIGHT_CM  8.0f
#define ALT_BRAKE_ACCEL_CM_S2            15.0f

typedef enum
{
    ALT_PHASE_LANDED = 0,
    ALT_PHASE_SPOOLING,
    ALT_PHASE_TAKEOFF,
    ALT_PHASE_HOLD,
    ALT_PHASE_LANDING
} alt_phase_t;

typedef struct {
    // 高度环PID控制器（外环）
    pid_param_t height_pid;

    // 速度环PID控制器（内环）
    pid_param_t vel_pid;

} altitude_control_t;

extern altitude_control_t alt_ctrl;

// 函数声明
typedef struct
{
    float exp_height;
    float fb_height;
    float height_err;
    float exp_vel;
} alt_2l_ct_t;

typedef struct
{
    float exp_climb_rate;
    float fb_climb_rate;
    float vel_err;
} alt_1l_ct_t;

extern volatile alt_2l_ct_t alt_2l_ct;
extern volatile alt_1l_ct_t alt_1l_ct;
extern volatile alt_phase_t alt_phase;

typedef struct
{
    float authority;
    float vel_feedback_weight;
    float pid_raw;
    float pid_weighted;
    float vel_feedforward;
    float dyn_gain_scale;
    float hover_trim;
    float hover_base_with_trim;
    float vel_damping;
    float near_ground_brake;
    float throttle_pre_limit;
    float throttle_post_limit;
    float throttle_final;
    float min_throttle;
    float max_throttle;
    float final_height;
    float profile_height;
    float profile_velocity;
    float profile_acceleration;
    float profile_error;
    float position_correction;
    float velocity_command;
} alt_ctrl_debug_t;



extern volatile alt_ctrl_debug_t alt_ctrl_debug;

// 初始化
void alt_ctrl_init(void);
void alt_2level_ctrl(float dT_s);
void alt_1level_ctrl(float dT_s);


// 主控制更新
void alt_ctrl_update(float dT_s);





#endif /* __ALT_CTRL_H__ */
