#include "zf_common_headfile.h"


static float alt_target_vel_z_ramped = 0.0f;
static float alt_target_height_ramped=0.0f;

uint8_t locked_flag = 0;
uint8_t auto_landing_request = 0;
uint8_t auto_landing_active = 0;
volatile uint8_t preflight_error_flags = 0u;
param_t param = {0};
flight_mode_t flight_mode = {0};
vehicle_setpoint_t vehicle_setpoint={0}; 
vehicle_state_t vehicle_state={0};
manual_input_t manual_input = {0};

uint8_t base_image[MT9V03X_H][MT9V03X_W];
uint8_t binary_image[MT9V03X_H][MT9V03X_W];
uint8_t undistorted_image[MT9V03X_H][MT9V03X_W]; // 定义去畸变后的图像缓冲区
uint8_t decoupled_image[MT9V03X_H][MT9V03X_W];   // 定义姿态解耦后的图像缓冲区
volatile uint32_t imu_update_cnt = 0;
volatile uint32_t att_ctrl_cnt = 0;
volatile uint32_t motor_mix_cnt = 0;
// 封装图像和查找表，供 image_remap8 使用
image_t base_img = 
{
    .data = (uint8_t *)base_image,
    .width = MT9V03X_W,
    .height = MT9V03X_H,
    .step = MT9V03X_W
};
image_t binary_img = 
{
    .data = (uint8_t *)binary_image,
    .width = MT9V03X_W,
    .height = MT9V03X_H,
    .step = MT9V03X_W
};

image_t undistorted_img = 
{
    .data = (uint8_t *)undistorted_image,
    .width = MT9V03X_W,
    .height = MT9V03X_H,
    .step = MT9V03X_W
};
image_t decoupled_img = 
{
    .data = (uint8_t *)decoupled_image,
    .width = MT9V03X_W,
    .height = MT9V03X_H,
    .step = MT9V03X_W
};
image_t mapx_img = 
{
    .data = (uint8_t *)lut_mapX, // 强转丢弃const，底层 image_remap8 仅作安全读取
    .width = LUT_WIDTH,
    .height = LUT_HEIGHT,
    .step = LUT_WIDTH
};
image_t mapy_img = 
{
    .data = (uint8_t *)lut_mapY,
    .width = LUT_WIDTH,
    .height = LUT_HEIGHT,
    .step = LUT_WIDTH
};



uint8_t preflight_check(void)
{
    uint8_t errors = 0u;

    if(imu_is_valid() == 0u)
    {
        errors |= PREFLIGHT_ERR_IMU;
    }

    if(tof_health_ok() == 0u)
    {
        errors |= PREFLIGHT_ERR_TOF;
    }

    preflight_error_flags = errors;
    return (errors == 0u) ? 1u : 0u;
}


void param_init(void)
{

}

#define DEBUG_STATE_LOG_SAMPLE_COUNT      600u
#define DEBUG_STATE_LOG_SAMPLE_RATE_HZ    50u
#define DEBUG_STATE_LOG_START_HEIGHT_CM   45.0f

typedef struct
{
    uint32_t time_us;
    float height_cm;
    float target_height_cm;
    float vel_z_cm_s;
    float target_vz_cm_s;
    float throttle;
    float throttle_ramped;
    float hover_base;
    float raw_tof_cm;
    float tof_innov_cm;
    float tof_diff_vel;
    float height_tof_vel_dt_ms;
    float height_tof_vel_raw;
    float height_tof_vel_filtered;
    float height_vel_innov;
    float acc_ekf_cm_s2;
    float acc_raw_cm_s2;
    float acc_bias_cm_s2;
    float acc_corrected_cm_s2;
    float acc_lpf_cm_s2;
    float profile_height_cm;
    float profile_vz_cm_s;
    float profile_accel_cm_s2;
    float profile_error_cm;
    float position_correction_cm_s;
    float velocity_command_cm_s;
    float vel_feedforward;
    float pid_raw;
    float pid_weighted;
    float alt_vel_i;
    float authority;
    float vel_feedback_weight;
    float hover_trim;
    float hover_base_with_trim;
    float vel_damping;
    float near_ground_brake;
    float throttle_pre_limit;
    float throttle_post_limit;
    float throttle_final;
    float min_throttle;
    float max_throttle;
    float voltage;
    float pos_x_cm;
    float pos_y_cm;
    float target_pos_x_cm;
    float target_pos_y_cm;
    float vel_x_cm_s;
    float vel_y_cm_s;
    float target_vel_x_cm_s;
    float target_vel_y_cm_s;
    float loc_exp_pos_x_cm;
    float loc_exp_pos_y_cm;
    float loc_fb_pos_x_cm;
    float loc_fb_pos_y_cm;
    float loc_pos_err_x_cm;
    float loc_pos_err_y_cm;
    float loc_exp_vel_x_cm_s;
    float loc_exp_vel_y_cm_s;
    float loc_fb_vel_x_cm_s;
    float loc_fb_vel_y_cm_s;
    float loc_vel_err_x_cm_s;
    float loc_vel_err_y_cm_s;
    float loc_vel_err_x_body_cm_s;
    float loc_vel_err_y_body_cm_s;
    float loc_roll_out_deg;
    float loc_pitch_out_deg;
    float loc_raw_target_roll_deg;
    float loc_raw_target_pitch_deg;
    float loc_weight;
    float loc_pos_p_x;
    float loc_pos_i_x;
    float loc_pos_d_x;
    float loc_pos_p_y;
    float loc_pos_i_y;
    float loc_pos_d_y;
    float loc_vel_p_x;
    float loc_vel_i_x;
    float loc_vel_d_x;
    float loc_vel_p_y;
    float loc_vel_i_y;
    float loc_vel_d_y;
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    float target_roll_deg;
    float target_pitch_deg;
    float target_yaw_rate_deg_s;
    float gyro_z_deg_s;
    float ct_roll;
    float ct_pitch;
    float ct_yaw;
    float rate_exp_r;
    float rate_exp_p;
    float rate_exp_y;
    float rate_fb_r;
    float rate_fb_p;
    float rate_fb_y;
    float rate_p_r;
    float rate_p_p;
    float rate_p_y;
    float rate_i_r;
    float rate_i_p;
    float rate_i_y;
    float rate_d_r;
    float rate_d_p;
    float rate_d_y;
    int16_t m1;
    int16_t m2;
    int16_t m3;
    int16_t m4;
    uint8_t phase;
    uint8_t tof_used;
    uint8_t vel_used;
    uint8_t loc_ready;
    uint8_t loc_hold_ready;
    uint8_t armed;
    uint8_t flight_mode;
    uint8_t flow_valid;
    uint8_t flow_raw_valid;
    uint8_t flow_quality_valid;
    uint8_t flow_obs_valid;
    uint8_t flow_ekf_used;
    uint8_t flow_gate_clipped;
    uint8_t flow_quality;
    uint16_t flow_accum_count;
    uint32_t flow_integration_us;
    uint32_t flow_frame_count;
    float flow_obs_vx_cm_s;
    float flow_obs_vy_cm_s;
    float flow_innov_vx_cm_s;
    float flow_innov_vy_cm_s;
    float flow_obs_dt_ms;
    float flow_gate_limit_cm_s;
} debug_state_sample_t;

static debug_state_sample_t debug_state_buf[DEBUG_STATE_LOG_SAMPLE_COUNT];
static uint16_t debug_state_count = 0u;
static uint16_t debug_state_dump_index = 0u;
static uint8_t debug_state_recording = 0u;
static uint8_t debug_state_ready = 0u;
static uint16_t debug_state_wr_idx = 0u;
static uint8_t debug_state_buffer_full = 0u;

/**
 * @brief Capture one coherent 20 ms control snapshot into RAM.
 * @note Called only at the end of the 20 ms control section. Never prints.
 */
void debug_capture_states_20ms(void)
{
    uint32_t now_us = system_time_us();
    uint8_t armed = vehicle_state.armed;

    if((armed != 0u) &&
       (alt_phase != ALT_PHASE_LANDING) &&
       (vehicle_state.current_height >= DEBUG_STATE_LOG_START_HEIGHT_CM) &&
       (debug_state_recording == 0u))
    {
        // A new flight owns the buffer. Never resume an old partial dump.
        debug_state_ready = 0u;
        debug_state_count = 0u;
        debug_state_dump_index = 0u;
        debug_state_wr_idx = 0u;
        debug_state_buffer_full = 0u;
        debug_state_recording = 1u;
    }

    if(((armed == 0u) || (alt_phase == ALT_PHASE_LANDING)) &&
       (debug_state_recording != 0u))
    {
        debug_state_recording = 0u;
        debug_state_ready = (debug_state_count > 0u) ? 1u : 0u;
        debug_state_dump_index = 0u;
    }

    // Exactly one sample per caller invocation.  The caller is the 20 ms
    // control section, so a second wall-clock gate would turn a 19.999 ms
    // tick into an artificial 40 ms logging gap.
    if(debug_state_recording != 0u)
    {
        debug_state_sample_t *sample = &debug_state_buf[debug_state_wr_idx];

        sample->time_us = now_us;
        sample->height_cm = vehicle_state.current_height;
        sample->target_height_cm = vehicle_setpoint.target_height;
        sample->vel_z_cm_s = vehicle_state.current_vel_z;
        sample->target_vz_cm_s = vehicle_setpoint.target_vel_z;
        sample->throttle = vehicle_setpoint.target_throttle;
        sample->throttle_ramped = throttle_ramped_debug;
        sample->hover_base = system_get_hover_throttle_base();
        sample->raw_tof_cm = tof_dist_cm;
        sample->tof_innov_cm = ekf_lite_health.tof_innov;
        sample->tof_diff_vel = tof_diff_vel_debug;
        sample->height_tof_vel_dt_ms = height_tof_vel_dt_ms_debug;
        sample->height_tof_vel_raw = height_tof_vel_raw_debug;
        sample->height_tof_vel_filtered = height_tof_vel_filtered_debug;
        sample->height_vel_innov = height_vel_innov_debug;
        sample->acc_ekf_cm_s2 = acc_z_cm_s2;
        sample->acc_raw_cm_s2 = acc_z_world_raw_cm_s2;
        sample->acc_bias_cm_s2 = acc_z_world_bias_cm_s2;
        sample->acc_corrected_cm_s2 = acc_z_world_corrected_cm_s2;
        sample->acc_lpf_cm_s2 = acc_z_world_lpf_cm_s2;
        sample->profile_height_cm = alt_ctrl_debug.profile_height;
        sample->profile_vz_cm_s = alt_ctrl_debug.profile_velocity;
        sample->profile_accel_cm_s2 = alt_ctrl_debug.profile_acceleration;
        sample->profile_error_cm = alt_ctrl_debug.profile_error;
        sample->position_correction_cm_s = alt_ctrl_debug.position_correction;
        sample->velocity_command_cm_s = alt_ctrl_debug.velocity_command;
        sample->vel_feedforward = alt_ctrl_debug.vel_feedforward;
        sample->pid_raw = alt_ctrl_debug.pid_raw;
        sample->pid_weighted = alt_ctrl_debug.pid_weighted;
        sample->alt_vel_i = alt_ctrl.vel_pid.out_i;
        sample->authority = alt_ctrl_debug.authority;
        sample->vel_feedback_weight = alt_ctrl_debug.vel_feedback_weight;
        sample->hover_trim = alt_ctrl_debug.hover_trim;
        sample->hover_base_with_trim = alt_ctrl_debug.hover_base_with_trim;
        sample->vel_damping = alt_ctrl_debug.vel_damping;
        sample->near_ground_brake = alt_ctrl_debug.near_ground_brake;
        sample->throttle_pre_limit = alt_ctrl_debug.throttle_pre_limit;
        sample->throttle_post_limit = alt_ctrl_debug.throttle_post_limit;
        sample->throttle_final = alt_ctrl_debug.throttle_final;
        sample->min_throttle = alt_ctrl_debug.min_throttle;
        sample->max_throttle = alt_ctrl_debug.max_throttle;
        sample->voltage = vehicle_state.battery_voltage_filtered;
        sample->pos_x_cm = vehicle_state.current_pos_x;
        sample->pos_y_cm = vehicle_state.current_pos_y;
        sample->target_pos_x_cm = vehicle_setpoint.target_pos_x;
        sample->target_pos_y_cm = vehicle_setpoint.target_pos_y;
        sample->vel_x_cm_s = vehicle_state.current_vel_x;
        sample->vel_y_cm_s = vehicle_state.current_vel_y;
        sample->target_vel_x_cm_s = vehicle_setpoint.target_vel_x;
        sample->target_vel_y_cm_s = vehicle_setpoint.target_vel_y;
        sample->loc_exp_pos_x_cm = loc_2l_ct.exp_pos_x;
        sample->loc_exp_pos_y_cm = loc_2l_ct.exp_pos_y;
        sample->loc_fb_pos_x_cm = loc_2l_ct.fb_pos_x;
        sample->loc_fb_pos_y_cm = loc_2l_ct.fb_pos_y;
        sample->loc_pos_err_x_cm = loc_2l_ct.exp_pos_x - loc_2l_ct.fb_pos_x;
        sample->loc_pos_err_y_cm = loc_2l_ct.exp_pos_y - loc_2l_ct.fb_pos_y;
        sample->loc_exp_vel_x_cm_s = loc_1l_ct.exp_vel_x;
        sample->loc_exp_vel_y_cm_s = loc_1l_ct.exp_vel_y;
        sample->loc_fb_vel_x_cm_s = loc_1l_ct.fb_vel_x;
        sample->loc_fb_vel_y_cm_s = loc_1l_ct.fb_vel_y;
        sample->loc_vel_err_x_cm_s = loc_1l_ct.exp_vel_x - loc_1l_ct.fb_vel_x;
        sample->loc_vel_err_y_cm_s = loc_1l_ct.exp_vel_y - loc_1l_ct.fb_vel_y;
        sample->loc_vel_err_x_body_cm_s = loc_1l_ct.vel_err_x_body;
        sample->loc_vel_err_y_body_cm_s = loc_1l_ct.vel_err_y_body;
        sample->loc_roll_out_deg = loc_output.roll_adj;
        sample->loc_pitch_out_deg = loc_output.pitch_adj;
        sample->loc_raw_target_roll_deg = loc_1l_ct.raw_target_roll;
        sample->loc_raw_target_pitch_deg = loc_1l_ct.raw_target_pitch;
        sample->loc_weight = loc_1l_ct.loc_weight;
        sample->loc_pos_p_x = loc_ctrl.pos_pid[0].out_p;
        sample->loc_pos_i_x = loc_ctrl.pos_pid[0].out_i;
        sample->loc_pos_d_x = loc_ctrl.pos_pid[0].out_d;
        sample->loc_pos_p_y = loc_ctrl.pos_pid[1].out_p;
        sample->loc_pos_i_y = loc_ctrl.pos_pid[1].out_i;
        sample->loc_pos_d_y = loc_ctrl.pos_pid[1].out_d;
        sample->loc_vel_p_x = loc_ctrl.vel_pid[0].out_p;
        sample->loc_vel_i_x = loc_ctrl.vel_pid[0].out_i;
        sample->loc_vel_d_x = loc_ctrl.vel_pid[0].out_d;
        sample->loc_vel_p_y = loc_ctrl.vel_pid[1].out_p;
        sample->loc_vel_i_y = loc_ctrl.vel_pid[1].out_i;
        sample->loc_vel_d_y = loc_ctrl.vel_pid[1].out_d;
        sample->roll_deg = vehicle_state.current_roll;
        sample->pitch_deg = vehicle_state.current_pitch;
        sample->yaw_deg = vehicle_state.current_yaw;
        sample->target_roll_deg = vehicle_setpoint.target_roll;
        sample->target_pitch_deg = vehicle_setpoint.target_pitch;
        sample->target_yaw_rate_deg_s = vehicle_setpoint.target_yaw_rate;
        sample->gyro_z_deg_s = imu_data.gyro_actual[2];
        sample->ct_roll = ct_val.rol;
        sample->ct_pitch = ct_val.pit;
        sample->ct_yaw = ct_val.yaw;
        sample->rate_exp_r = att_1l_ct.exp_ang_vel[0];
        sample->rate_exp_p = att_1l_ct.exp_ang_vel[1];
        sample->rate_exp_y = att_1l_ct.exp_ang_vel[2];
        sample->rate_fb_r = att_1l_ct.fb_ang_vel[0];
        sample->rate_fb_p = att_1l_ct.fb_ang_vel[1];
        sample->rate_fb_y = att_1l_ct.fb_ang_vel[2];
        sample->rate_p_r = att_ctrl.rate_pid[0].out_p;
        sample->rate_p_p = att_ctrl.rate_pid[1].out_p;
        sample->rate_p_y = att_ctrl.rate_pid[2].out_p;
        sample->rate_i_r = att_ctrl.rate_pid[0].out_i;
        sample->rate_i_p = att_ctrl.rate_pid[1].out_i;
        sample->rate_i_y = att_ctrl.rate_pid[2].out_i;
        sample->rate_d_r = att_ctrl.rate_pid[0].out_d;
        sample->rate_d_p = att_ctrl.rate_pid[1].out_d;
        sample->rate_d_y = att_ctrl.rate_pid[2].out_d;
        sample->m1 = motor_out.m1;
        sample->m2 = motor_out.m2;
        sample->m3 = motor_out.m3;
        sample->m4 = motor_out.m4;
        sample->phase = (uint8_t)alt_phase;
        sample->tof_used = ekf_lite_health.tof_used;
        sample->vel_used = height_vel_update_used_debug;
        sample->loc_ready = loc_1l_ct.loc_ready;
        sample->loc_hold_ready = loc_1l_ct.loc_hold_ready;
        sample->armed = armed;
        sample->flight_mode = (uint8_t)vehicle_state.flight_mode;
        sample->flow_valid = vehicle_state.flow_valid;
        sample->flow_raw_valid = flow_health.raw_valid;
        sample->flow_quality_valid = flow_health.quality_valid;
        sample->flow_obs_valid = flow_health.obs_valid;
        sample->flow_ekf_used = flow_health.ekf_used;
        sample->flow_gate_clipped = flow_health.gate_clipped;
        sample->flow_quality = flow_health.quality;
        sample->flow_accum_count = flow_health.lc302_accum_count;
        sample->flow_integration_us = flow_health.lc302_integration_us;
        sample->flow_frame_count = flow_health.lc302_frame_count;
        sample->flow_obs_vx_cm_s = flow_health.obs_vx_cm_s;
        sample->flow_obs_vy_cm_s = flow_health.obs_vy_cm_s;
        sample->flow_innov_vx_cm_s = flow_health.innov_vx_cm_s;
        sample->flow_innov_vy_cm_s = flow_health.innov_vy_cm_s;
        sample->flow_obs_dt_ms = flow_health.obs_dt_s * 1000.0f;
        sample->flow_gate_limit_cm_s = flow_health.gate_limit_cm_s;
        
        debug_state_wr_idx++;
        if(debug_state_wr_idx >= DEBUG_STATE_LOG_SAMPLE_COUNT)
        {
            debug_state_wr_idx = 0u;
            debug_state_buffer_full = 1u;
        }
        
        if (debug_state_buffer_full) {
            debug_state_count = DEBUG_STATE_LOG_SAMPLE_COUNT;
        } else {
            debug_state_count = debug_state_wr_idx;
        }
    }

}

/**
 * @brief Dump the captured RAM buffer after disarm.
 * @note Called from the main loop. Never captures live flight state.
 */
void debug_print_states(void)
{

    if((debug_state_ready == 0u) || (vehicle_state.armed != 0u))
    {
        return;
    }

    if(debug_state_dump_index == 0u)
    {
        printf("DEBUG_STATE_BEGIN,count=%u,fs=%u\r\n",
               debug_state_count,
               DEBUG_STATE_LOG_SAMPLE_RATE_HZ);
    }

    uint8_t lines = 0u;
    while((debug_state_dump_index < debug_state_count) && (lines < 1u))
    {
        uint16_t start_idx = debug_state_buffer_full ? debug_state_wr_idx : 0u;
        uint16_t real_idx = (start_idx + debug_state_dump_index) % DEBUG_STATE_LOG_SAMPLE_COUNT;
        const debug_state_sample_t *sample = &debug_state_buf[real_idx];

        printf("========== DESKTOP DEBUG DASHBOARD [%u/%u] ==========\r\n",
               debug_state_dump_index,
               debug_state_count);
        printf("time_us:%lu voltage:%.3f armed:%u phase:%u\r\n",
               (unsigned long)sample->time_us,
               sample->voltage,
               sample->armed,
               sample->phase);
        printf("[FLOWDBG] mode:%u valid:%u raw:%u quality:%u(q=%u) obs:%u ekf:%u clip:%u frame:%lu accum:%u int_us:%lu dt_ms:%.2f\r\n",
               sample->flight_mode,
               sample->flow_valid,
               sample->flow_raw_valid,
               sample->flow_quality_valid,
               sample->flow_quality,
               sample->flow_obs_valid,
               sample->flow_ekf_used,
               sample->flow_gate_clipped,
               (unsigned long)sample->flow_frame_count,
               sample->flow_accum_count,
               (unsigned long)sample->flow_integration_us,
               sample->flow_obs_dt_ms);
        printf("[FLOWOBS] gate:%.2f obs_xy:%.2f/%.2f innov_xy:%.2f/%.2f\r\n",
               sample->flow_gate_limit_cm_s,
               sample->flow_obs_vx_cm_s,
               sample->flow_obs_vy_cm_s,
               sample->flow_innov_vx_cm_s,
               sample->flow_innov_vy_cm_s);
       printf("[1. ALTITUDE LOOP]\r\n");
        printf("Height (cm) : Cur: %.3f | Tgt: %.3f\r\n",
               sample->height_cm,
               sample->target_height_cm);
        printf("Vel Z (cm/s): Cur: %.3f | Tgt: %.3f\r\n",
               sample->vel_z_cm_s,
               sample->target_vz_cm_s);
        printf("[ALTTRAJ] z:%.3f v:%.3f a:%.3f err:%.3f corr:%.3f cmd:%.3f\r\n",
               sample->profile_height_cm,
               sample->profile_vz_cm_s,
               sample->profile_accel_cm_s2,
               sample->profile_error_cm,
               sample->position_correction_cm_s,
               sample->velocity_command_cm_s);
        printf("raw_dist_cm=%.2f\r\n", sample->raw_tof_cm);
        printf("throttle_out:%.2f | throttle_ramped:%.2f | hover_base:%.2f | hover_trim:%.2f | hover_ff:%.2f\r\n",
               sample->throttle,
               sample->throttle_ramped,
               sample->hover_base,
               sample->hover_trim,
               sample->hover_base_with_trim);
        printf("[ALTDBG] ff:%.3f pid_raw:%.3f pid_w:%.3f auth:%.3f fb_w:%.3f\r\n",
               sample->vel_feedforward,
               sample->pid_raw,
               sample->pid_weighted,
               sample->authority,
               sample->vel_feedback_weight);
        printf("[ALTDBG] damp:%.3f brake:%.3f pre:%.3f post:%.3f final:%.3f lim:[%.3f,%.3f]\r\n",
               sample->vel_damping,
               sample->near_ground_brake,
               sample->throttle_pre_limit,
               sample->throttle_post_limit,
               sample->throttle_final,
               sample->min_throttle,
               sample->max_throttle);
        printf("[ALTDBG] vel_i:%.3f\r\n",
               sample->alt_vel_i);
       /* printf("[ACCBIAS] raw:%.3f bias:%.3f corrected:%.3f lpf:%.3f ekf_acc:%.3f\r\n",
               sample->acc_raw_cm_s2,
               sample->acc_bias_cm_s2,
               sample->acc_corrected_cm_s2,
               sample->acc_lpf_cm_s2,
               sample->acc_ekf_cm_s2);
        printf("[OBS] tof_used:%u innov:%.3f tof_vel:%.3f vel_dt_ms:%.3f vel_raw:%.3f vel_f:%.3f vel_innov:%.3f vel_used:%u\r\n",
               sample->tof_used,
               sample->tof_innov_cm,
               sample->tof_diff_vel,
               sample->height_tof_vel_dt_ms,
               sample->height_tof_vel_raw,
               sample->height_tof_vel_filtered,
               sample->height_vel_innov,
               sample->vel_used);*/
        printf("[2. LOCATION LOOP]\r\n");
        printf("Pos X (cm)  : Cur: %.3f | Tgt: %.3f | Err: %.3f\r\n",
               sample->loc_fb_pos_x_cm,
               sample->loc_exp_pos_x_cm,
               sample->loc_pos_err_x_cm);
        printf("Vel X (cm/s): Cur: %.3f | Tgt: %.3f | Err: %.3f\r\n",
               sample->loc_fb_vel_x_cm_s,
               sample->loc_exp_vel_x_cm_s,
               sample->loc_vel_err_x_cm_s);
        printf("Pos Y (cm)  : Cur: %.3f | Tgt: %.3f | Err: %.3f\r\n",
               sample->loc_fb_pos_y_cm,
               sample->loc_exp_pos_y_cm,
               sample->loc_pos_err_y_cm);
        printf("Vel Y (cm/s): Cur: %.3f | Tgt: %.3f | Err: %.3f\r\n",
               sample->loc_fb_vel_y_cm_s,
               sample->loc_exp_vel_y_cm_s,
               sample->loc_vel_err_y_cm_s);
        printf("[LOCDBG] vel_err body X/Y:%.3f %.3f\r\n",
               sample->loc_vel_err_x_body_cm_s,
               sample->loc_vel_err_y_body_cm_s);
       printf("[LOCDBG] pos_pid X p/i/d:%.3f %.3f %.3f | Y p/i/d:%.3f %.3f %.3f\r\n",
               sample->loc_pos_p_x,
               sample->loc_pos_i_x,
               sample->loc_pos_d_x,
               sample->loc_pos_p_y,
               sample->loc_pos_i_y,
               sample->loc_pos_d_y);
        printf("[LOCDBG] vel_pid X p/i/d:%.3f %.3f %.3f | Y p/i/d:%.3f %.3f %.3f\r\n",
               sample->loc_vel_p_x,
               sample->loc_vel_i_x,
               sample->loc_vel_d_x,
               sample->loc_vel_p_y,
               sample->loc_vel_i_y,
               sample->loc_vel_d_y);
        printf("[LOCDBG] raw roll:%.3f pitch:%.3f | sp/ramped roll:%.3f pitch:%.3f\r\n",
               sample->loc_raw_target_roll_deg,
               sample->loc_raw_target_pitch_deg,
               sample->target_roll_deg,
               sample->target_pitch_deg);
        printf("[LOCDBG] ready:%u hold:%u weight:%.3f\r\n",
               sample->loc_ready,
               sample->loc_hold_ready,
               sample->loc_weight);
        printf("[LOCDBG] legacy loc_output roll:%.3f pitch:%.3f\r\n",
               sample->loc_roll_out_deg,
               sample->loc_pitch_out_deg);

        printf("[3. ATTITUDE LOOP]\r\n");
        printf("Roll (deg)  : Cur: %3.2f | Tgt: %3.2f -> PID Out: %.2f\r\n",
               sample->roll_deg,
               sample->target_roll_deg,
               sample->ct_roll);
        printf("Pitch(deg)  : Cur: %3.2f | Tgt: %3.2f -> PID Out: %.2f\r\n",
               sample->pitch_deg,
               sample->target_pitch_deg,
               sample->ct_pitch);
        printf("Yaw (deg)   : Cur: %3.2f | Gz: %3.2f dps\r\n",
               sample->yaw_deg,
               sample->gyro_z_deg_s);
        printf("YawR(deg/s) : Tgt: %3.2f -> PID Out: %.2f\r\n",
               sample->target_yaw_rate_deg_s,
               sample->ct_yaw);
        printf("Rate R exp/fb/out: %.2f %.2f %.2f | P exp/fb/out: %.2f %.2f %.2f\r\n",
               sample->rate_exp_r,
               sample->rate_fb_r,
               sample->ct_roll,
               sample->rate_exp_p,
               sample->rate_fb_p,
               sample->ct_pitch);
        printf("Y exp/fb/out: %.2f %.2f %.2f\r\n",
               sample->rate_exp_y,
               sample->rate_fb_y,
               sample->ct_yaw);
        printf("Rate P R/P/Y: %.3f %.3f %.3f\r\n",
               sample->rate_p_r,
               sample->rate_p_p,
               sample->rate_p_y);
        printf("Rate I R/P/Y: %.3f %.3f %.3f\r\n",
               sample->rate_i_r,
               sample->rate_i_p,
               sample->rate_i_y);
        printf("Rate D R/P/Y: %.3f %.3f %.3f\r\n",
               sample->rate_d_r,
               sample->rate_d_p,
               sample->rate_d_y);
        /* PID configuration is static during a flight; keep it out of the
         * high-rate diagnostic stream. */
        /*printf("[PIDCFG] ANG kp R/P/Y: %.3f %.3f %.3f | RATE kp R/P/Y: %.4f %.4f %.4f\r\n",
               att_ctrl.angle_pid[0].kp,
               att_ctrl.angle_pid[1].kp,
               att_ctrl.angle_pid[2].kp,
               att_ctrl.rate_pid[0].kp,
               att_ctrl.rate_pid[1].kp,
               att_ctrl.rate_pid[2].kp);
        printf("[PIDCFG] RATE ki R/P/Y: %.4f %.4f %.4f | kd R/P/Y: %.5f %.5f %.5f | lp R/P/Y: %.2f %.2f %.2f\r\n",
               att_ctrl.rate_pid[0].ki,
               att_ctrl.rate_pid[1].ki,
               att_ctrl.rate_pid[2].ki,
               att_ctrl.rate_pid[0].kd,
               att_ctrl.rate_pid[1].kd,
               att_ctrl.rate_pid[2].kd,
               att_ctrl.rate_pid[0].low_pass,
               att_ctrl.rate_pid[1].low_pass,
               att_ctrl.rate_pid[2].low_pass);*/
        printf("[4. MOTOR OUTPUT (0 ~ 10000)]\r\n");
        printf("M1 (LF_CW) : %5d  |  M2 (RF_CCW): %5d\r\n",
               sample->m1,
               sample->m2);
        printf("M3 (LB_CCW): %5d  |  M4 (RB_CW) : %5d\r\n",
               sample->m3,
               sample->m4);
        printf("=============================================\r\n");
        debug_state_dump_index++;
        lines++;
    }

    if(debug_state_dump_index >= debug_state_count)
    {
        printf("DEBUG_STATE_END\r\n");
        debug_state_ready = 0u;
        debug_state_count = 0u;
        debug_state_dump_index = 0u;
    }
}

/**
 * @brief 核心状态机：管理飞行模式切换、生成各模式下的期望值
 * @param dT_s 时间间隔(秒)
 * @note 这是飞控上层逻辑的核心，决定了飞机在不同模式下的行为。
 */
void param_update(float dT_s)
{
    // 1. 解析遥控器数据，获得干净的杆量输入 `manual_input`
    prase_remote_ctrl_data(dT_s);

    if(dT_s <= 0.0f || dT_s > 0.1f)
    {
        dT_s = REMOTE_SAMPLE_TIME;
    }

    if((vehicle_state.armed != 0u) && (tof_health_ok() == 0u))
    {
        vehicle_state.armed = 0u;
        auto_landing_request = 0u;
        auto_landing_active = 0u;
        return;
    }

    // 安全保护：未解锁时，所有期望值贴住当前状态，避免解锁瞬间跳变。
    if (vehicle_state.armed == 0)
    {
        auto_landing_active = 0;
        alt_target_vel_z_ramped = 0.0f;
        alt_target_height_ramped = vehicle_state.current_height;
        vehicle_setpoint.target_height = vehicle_state.current_height;
        vehicle_setpoint.target_pos_x  = vehicle_state.current_pos_x;
        vehicle_setpoint.target_pos_y  = vehicle_state.current_pos_y;
        vehicle_setpoint.target_vel_x  = 0.0f;
        vehicle_setpoint.target_vel_y  = 0.0f;
        vehicle_setpoint.target_vel_z  = 0.0f;
        vehicle_setpoint.target_throttle = 0.0f;
        vehicle_setpoint.target_roll = 0.0f;
        vehicle_setpoint.target_pitch = 0.0f;
        vehicle_setpoint.target_yaw_rate = 0.0f;
        return;
    }

    if(auto_landing_request != 0u)
    {
        auto_landing_active = 1u;
        vehicle_state.flight_mode = FLY_AUTOLANDING;
        vehicle_setpoint.target_height = AUTO_LAND_TARGET_HEIGHT_CM;
        vehicle_setpoint.target_pos_x  = vehicle_state.current_pos_x;
        vehicle_setpoint.target_pos_y  = vehicle_state.current_pos_y;
        vehicle_setpoint.target_vel_x  = 0.0f;
        vehicle_setpoint.target_vel_y  = 0.0f;
        vehicle_setpoint.target_roll = 0.0f;
        vehicle_setpoint.target_pitch = 0.0f;
        vehicle_setpoint.target_yaw_rate = 0.0f;

        if((vehicle_state.current_height <= AUTO_LAND_DISARM_HEIGHT_CM) &&
           (fabsf(vehicle_state.current_vel_z) <= AUTO_LAND_DISARM_VEL_CM_S))
        {
            vehicle_state.armed = 0u;
            auto_landing_request = 0u;
            auto_landing_active = 0u;
            return;
        }

        vehicle_setpoint.target_height = LIMIT(vehicle_setpoint.target_height, MIN_HEIGHT, MAX_HEIGHT);
        return;
    }

    // 2. 编译期测试模式：一次只建议打开一个 *_MODE_ENABLE。
#if TEST_MODE_ENABLE

#if ATT_ONLY_MODE_ENABLE
    // 只测姿态环：右手 roll/pitch/yaw 直接给姿态环，左油门杆直接给混控基础油门。
    vehicle_state.flight_mode = FLY_HEIGHT_HOLD;
    vehicle_setpoint.target_height = vehicle_state.current_height;
    vehicle_setpoint.target_pos_x  = vehicle_state.current_pos_x;
    vehicle_setpoint.target_pos_y  = vehicle_state.current_pos_y;
    vehicle_setpoint.target_vel_x  = 0.0f;
    vehicle_setpoint.target_vel_y  = 0.0f;
    vehicle_setpoint.target_vel_z  = 0.0f;
    vehicle_setpoint.target_roll = manual_input.roll;
    vehicle_setpoint.target_pitch = manual_input.pitch;
    vehicle_setpoint.target_yaw_rate = 0.0f;//manual_input.yaw_rate;

    float climb_rate_cmd = manual_input.climb_rate;
    if(fabsf(climb_rate_cmd) < ATT_THROTTLE_HOLD_DEADZONE_CM_S)
    {
        climb_rate_cmd = 0.0f;
    }

    float throttle_step = climb_rate_cmd / MAX_VEL_XYZ * MAX_CT_VAL / 10.0f;
    throttle_step = LIMIT(throttle_step, -0.2f, 0.2f);
    vehicle_setpoint.target_throttle += throttle_step;
    vehicle_setpoint.target_throttle = LIMIT(vehicle_setpoint.target_throttle, 0.0f, MAX_CT_VAL);
    

#elif LOC_ONLY_MODE_ENABLE
    // 只测位置环：右杆给水平速度目标，油门杆仍直接给基础油门，方便低油门带桨固定测试。
    vehicle_state.flight_mode = FLY_POS_HOLD;
    vehicle_setpoint.target_height =70.0f;
    vehicle_setpoint.target_yaw_rate = 0;//manual_input.yaw_rate;
    //vehicle_setpoint.target_throttle = LIMIT(manual_input.climb_rate / MAX_VEL_XYZ * MAX_CT_VAL, 0.0f, MAX_CT_VAL);

    if(vehicle_state.flow_valid)
    {
        //vehicle_setpoint.target_pos_x += vel_earth_x * dT_s;
        //vehicle_setpoint.target_pos_y += vel_earth_y * dT_s;
        if((alt_phase != ALT_PHASE_HOLD) ||
           (vehicle_state.current_height < LOC_ENABLE_HEIGHT_CM))
        {
            vehicle_setpoint.target_pos_x = vehicle_state.current_pos_x;
            vehicle_setpoint.target_pos_y = vehicle_state.current_pos_y;
        }

    }
    else
    {
        vehicle_setpoint.target_pos_x = vehicle_state.current_pos_x;
        vehicle_setpoint.target_pos_y = vehicle_state.current_pos_y;
        vehicle_setpoint.target_vel_x = 0.0f;
        vehicle_setpoint.target_vel_y = 0.0f;
    }
#elif ALT_ONLY_MODE_ENABLE
    // 只测高度环：左杆给爬，油门杆仍直接给基础油门，方便低油门带桨固定测试。


    vehicle_state.flight_mode = FLY_HEIGHT_HOLD;
    vehicle_setpoint.target_pos_x  = vehicle_state.current_pos_x;
    vehicle_setpoint.target_pos_y  = vehicle_state.current_pos_y;
    vehicle_setpoint.target_vel_x  = 0.0f;
    vehicle_setpoint.target_vel_y  = 0.0f;
    vehicle_setpoint.target_roll = 0.0f;
    vehicle_setpoint.target_pitch = 0.0f;
    vehicle_setpoint.target_yaw_rate =0.0f;

  /* float climb_cmd =manual_input.climb_rate / MAX_HEIGHT * 10.0f;

    if(fabsf(climb_cmd) < 0.5f)
    {
        climb_cmd = 0.0f;
    }

  */
    alt_target_height_ramped = 70.0f;
    vehicle_setpoint.target_height =
        LIMIT(alt_target_height_ramped, 6.5f, MAX_HEIGHT);


  /*  float target_vel_z =
    manual_input.climb_rate / MAX_VEL_XYZ * ALT_TARGET_VEL_MAX_CM_S;

    if(fabsf(target_vel_z) < ALT_TARGET_VEL_DEADZONE_CM_S)
    {
        target_vel_z = 0.0f;
    }

    alt_target_vel_z_ramped = slew_rate_limit(
        alt_target_vel_z_ramped,
        target_vel_z,
        ALT_TARGET_VEL_SLEW_CM_S2,
        dT_s);
    vehicle_setpoint.target_vel_z = alt_target_vel_z_ramped;*/
#elif FLY_MODE_ENABLE
    // 完整飞行模式：原定高/定点状态机。
    flight_mode_t desired_mode = vehicle_state.flight_mode;
    if (lora3a22_uart_transfer.switch_key[2] == 0) 
    {
        desired_mode = FLY_HEIGHT_HOLD;
    } 
    else
    {
        // 当 switch_key[2] == 1 时，进入定点模式
        desired_mode = FLY_POS_HOLD;
    }

    // 检查是否可以安全切换到目标模式
    if (desired_mode == FLY_POS_HOLD && !vehicle_state.flow_valid) 
    {
        // 如果光流无效，则不允许进入定点模式，自动降级为更安全的定高模式
        desired_mode = FLY_HEIGHT_HOLD;
    }

    // 如果当前就在定点模式，但中途光流失效，也应立即降级
    if (vehicle_state.flight_mode == FLY_POS_HOLD && !vehicle_state.flow_valid)
    {
        vehicle_state.flight_mode = FLY_HEIGHT_HOLD;
    }

    // 如果模式发生切换，执行“进入模式”的初始化操作
    if (desired_mode != vehicle_state.flight_mode) 
    {
        if (desired_mode == FLY_POS_HOLD) 
        {
            // 刚进入定点模式时，将目标点设置为飞机当前位置，防止飞机乱跑
            vehicle_setpoint.target_pos_x = vehicle_state.current_pos_x;
            vehicle_setpoint.target_pos_y = vehicle_state.current_pos_y;
        }
        vehicle_state.flight_mode = desired_mode;
    }

    switch(vehicle_state.flight_mode)
    {
        case FLY_HEIGHT_HOLD: // 定高模式：杆量控制姿态角和升降速度
            vehicle_setpoint.target_roll = manual_input.roll;
            vehicle_setpoint.target_pitch = manual_input.pitch;
            vehicle_setpoint.target_yaw_rate = manual_input.yaw_rate;
            vehicle_setpoint.target_height += manual_input.climb_rate * dT_s;
            break;

        case FLY_POS_HOLD: // 定点模式：杆量控制水平速度和升降速度
            vehicle_setpoint.target_height += manual_input.climb_rate * dT_s;
            vehicle_setpoint.target_yaw_rate = manual_input.yaw_rate;
            
            // 将杆量映射为“机体坐标系”下的期望速度
            float target_vel_x_body = manual_input.pitch / MAX_ROLL_PITCH * MAX_HORIZONTAL_SPEED;
            float target_vel_y_body = manual_input.roll / MAX_ROLL_PITCH * MAX_HORIZONTAL_SPEED;
            
            // 将机体速度期望，通过航向角旋转到“世界坐标系”，用于更新目标位置点
            // 必须进行旋转映射！否则在空中改变机头朝向时，打杆方向会彻底错乱
            float vel_earth_x = target_vel_x_body * vehicle_state.yaw_cos - target_vel_y_body * vehicle_state.yaw_sin;
            float vel_earth_y = target_vel_x_body * vehicle_state.yaw_sin + target_vel_y_body * vehicle_state.yaw_cos;
            
            vehicle_setpoint.target_pos_x += vel_earth_x * dT_s;
            vehicle_setpoint.target_pos_y += vel_earth_y * dT_s;
            // 注意：此模式下，target_roll 和 target_pitch 由位置控制器(loc_ctrl)计算，无需手动设置
            break;

        default:
            vehicle_setpoint.target_roll = 0.0f;
            vehicle_setpoint.target_pitch = 0.0f;
            vehicle_setpoint.target_yaw_rate = 0.0f;
            break;
    }
#else
    // TEST_MODE_ENABLE=1 但没有选择具体模式时，保持安全零输出。
    vehicle_state.flight_mode = FLY_HEIGHT_HOLD;
    vehicle_setpoint.target_height = vehicle_state.current_height;
    vehicle_setpoint.target_pos_x  = vehicle_state.current_pos_x;
    vehicle_setpoint.target_pos_y  = vehicle_state.current_pos_y;
    vehicle_setpoint.target_vel_x  = 0.0f;
    vehicle_setpoint.target_vel_y  = 0.0f;
    vehicle_setpoint.target_vel_z  = 0.0f;
    vehicle_setpoint.target_throttle = 0.0f;
    vehicle_setpoint.target_roll = 0.0f;
    vehicle_setpoint.target_pitch = 0.0f;
    vehicle_setpoint.target_yaw_rate = 0.0f;
#endif

#else
    // 正常飞行模式：原定高/定点状态机。
    flight_mode_t desired_mode = vehicle_state.flight_mode;
    if (lora3a22_uart_transfer.switch_key[2] == 0) 
    {
        desired_mode = FLY_HEIGHT_HOLD;
    } 
    else
    {
        desired_mode = FLY_POS_HOLD;
    }

    if (desired_mode == FLY_POS_HOLD && !vehicle_state.flow_valid) 
    {
        desired_mode = FLY_HEIGHT_HOLD;
    }

    if (vehicle_state.flight_mode == FLY_POS_HOLD && !vehicle_state.flow_valid)
    {
        vehicle_state.flight_mode = FLY_HEIGHT_HOLD;
    }

    if (desired_mode != vehicle_state.flight_mode) 
    {
        if (desired_mode == FLY_POS_HOLD) 
        {
            vehicle_setpoint.target_pos_x = vehicle_state.current_pos_x;
            vehicle_setpoint.target_pos_y = vehicle_state.current_pos_y;
        }
        vehicle_state.flight_mode = desired_mode;
    }

    switch(vehicle_state.flight_mode)
    {
        case FLY_HEIGHT_HOLD:
            vehicle_setpoint.target_roll = manual_input.roll;
            vehicle_setpoint.target_pitch = manual_input.pitch;
            vehicle_setpoint.target_yaw_rate = manual_input.yaw_rate;
            vehicle_setpoint.target_height += manual_input.climb_rate * dT_s;
            break;

        case FLY_POS_HOLD:
        {
            vehicle_setpoint.target_height += manual_input.climb_rate * dT_s;
            vehicle_setpoint.target_yaw_rate = manual_input.yaw_rate;

            float target_vel_x_body = manual_input.pitch / MAX_ROLL_PITCH * MAX_HORIZONTAL_SPEED;
            float target_vel_y_body = manual_input.roll / MAX_ROLL_PITCH * MAX_HORIZONTAL_SPEED;
            float vel_earth_x = target_vel_x_body * vehicle_state.yaw_cos - target_vel_y_body * vehicle_state.yaw_sin;
            float vel_earth_y = target_vel_x_body * vehicle_state.yaw_sin + target_vel_y_body * vehicle_state.yaw_cos;

            vehicle_setpoint.target_pos_x += vel_earth_x * dT_s;
            vehicle_setpoint.target_pos_y += vel_earth_y * dT_s;
            break;
        }

        default:
            vehicle_setpoint.target_roll = 0.0f;
            vehicle_setpoint.target_pitch = 0.0f;
            vehicle_setpoint.target_yaw_rate = 0.0f;
            break;
    }
#endif

    // 对所有模式通用的期望值进行限幅
    vehicle_setpoint.target_height = LIMIT(vehicle_setpoint.target_height, MIN_HEIGHT, MAX_HEIGHT);
    vehicle_setpoint.target_roll = LIMIT(vehicle_setpoint.target_roll, -MAX_ROLL_PITCH, MAX_ROLL_PITCH);
    vehicle_setpoint.target_pitch = LIMIT(vehicle_setpoint.target_pitch, -MAX_ROLL_PITCH, MAX_ROLL_PITCH);
    vehicle_setpoint.target_yaw_rate = LIMIT(vehicle_setpoint.target_yaw_rate, -MAX_YAW_RATE, MAX_YAW_RATE);    
    vehicle_setpoint.target_vel_x = LIMIT(vehicle_setpoint.target_vel_x, -MAX_HORIZONTAL_SPEED, MAX_HORIZONTAL_SPEED);
    vehicle_setpoint.target_vel_y = LIMIT(vehicle_setpoint.target_vel_y, -MAX_HORIZONTAL_SPEED, MAX_HORIZONTAL_SPEED);  
    
}
