#include "zf_common_headfile.h"


static float alt_target_vel_z_ramped = 0.0f;
static float alt_target_height_ramped=0.0f;

#define LOC_TEST_TARGET_HEIGHT_CM  140.0f
#define LOC_TEST_TARGET_POS_X_CM     0.0f
#define LOC_TEST_TARGET_POS_Y_CM     0.0f

uint8_t locked_flag = 0;
uint8_t auto_landing_request = 0;
uint8_t auto_landing_active = 0;
volatile uint8_t preflight_error_flags = 0u;
volatile uint8_t flight_sensor_failsafe_flags = 0u;
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
    .data = (uint8_t *)lut_mapX, // Strong cast drops const; image_remap8 only reads the LUT.
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

void fisheye_lut_apply(fisheye_lut_id_t id)
{
    const fisheye_lut_t *lut = fisheye_lut_get(id);

    mapx_img.data = (uint8_t *)lut->map_x;
    mapx_img.width = lut->width;
    mapx_img.height = lut->height;
    mapx_img.step = lut->width;

    mapy_img.data = (uint8_t *)lut->map_y;
    mapy_img.width = lut->width;
    mapy_img.height = lut->height;
    mapy_img.step = lut->width;
}

typedef struct
{
    uint8_t valid;
    uint8_t pending;
    uint8_t accepted;
    uint8_t error_flags;
    uint8_t flight_mode;
    uint8_t imu_valid;
    uint8_t tof_ok;
    uint8_t tof_raw_valid;
    uint8_t tof_stale;
    uint8_t tof_stuck;
    uint8_t tof_valid_frames;
    uint8_t flow_valid;
    uint8_t flow_raw_valid;
    uint8_t flow_obs_valid;
    uint8_t flow_ekf_used;
    uint8_t flow_quality;
    uint32_t attempt;
    uint32_t time_us;
    uint32_t lc302_bytes;
    uint32_t lc302_frames;
    uint32_t lc302_errors;
    uint32_t lc302_last_rx_us;
    float flow_ready_time_s;
    float voltage;
    float height_cm;
} preflight_snapshot_t;

typedef struct
{
    uint8_t valid;
    uint8_t pending;
    uint8_t reason_flags;
    uint8_t imu_valid;
    uint8_t imu_attitude_valid;
    uint8_t tof_ok;
    uint8_t flow_valid;
    uint8_t flow_quality;
    uint32_t time_us;
    uint32_t imu_bad_frame_count;
    uint32_t lc302_bytes;
    uint32_t lc302_frames;
    float voltage;
    float height_cm;
} flight_failsafe_snapshot_t;

static preflight_snapshot_t preflight_snapshot = {0};
static flight_failsafe_snapshot_t flight_failsafe_snapshot = {0};
static float preflight_flow_ready_time_s = 0.0f;
static float flight_imu_fault_time_s = 0.0f;
static float flight_tof_fault_time_s = 0.0f;
static float flight_flow_fault_time_s = 0.0f;
static uint8_t flight_flow_required_latched = 0u;

static uint8_t preflight_flow_is_healthy(void)
{
    return ((lc302_data.byte_count > 0u) &&
            (lc302_data.frame_count > 0u) &&
            (lc302_data.quality >= LC302_QUALITY_MIN) &&
            (vehicle_state.flow_valid != 0u)) ? 1u : 0u;
}

static void preflight_capture_snapshot(uint8_t errors,
                                       uint8_t imu_valid,
                                       uint8_t tof_ok)
{
    preflight_snapshot.attempt++;
    preflight_snapshot.valid = 1u;
    preflight_snapshot.pending = 1u;
    preflight_snapshot.accepted = (errors == 0u) ? 1u : 0u;
    preflight_snapshot.error_flags = errors;
    preflight_snapshot.flight_mode = (uint8_t)vehicle_state.flight_mode;
    preflight_snapshot.imu_valid = imu_valid;
    preflight_snapshot.tof_ok = tof_ok;
    preflight_snapshot.tof_raw_valid = tof_health.raw_valid;
    preflight_snapshot.tof_stale = tof_health.stale;
    preflight_snapshot.tof_stuck = tof_health.stuck;
    preflight_snapshot.tof_valid_frames = tof_health.valid_frame_count;
    preflight_snapshot.flow_valid = vehicle_state.flow_valid;
    preflight_snapshot.flow_raw_valid = flow_health.raw_valid;
    preflight_snapshot.flow_obs_valid = flow_health.obs_valid;
    preflight_snapshot.flow_ekf_used = flow_health.ekf_used;
    preflight_snapshot.flow_quality = lc302_data.quality;
    preflight_snapshot.time_us = system_time_us();
    preflight_snapshot.lc302_bytes = lc302_data.byte_count;
    preflight_snapshot.lc302_frames = lc302_data.frame_count;
    preflight_snapshot.lc302_errors = lc302_data.checksum_error_count;
    preflight_snapshot.lc302_last_rx_us = lc302_data.last_frame_rx_us;
    preflight_snapshot.flow_ready_time_s = preflight_flow_ready_time_s;
    preflight_snapshot.voltage = vehicle_state.battery_voltage_filtered;
    preflight_snapshot.height_cm = vehicle_state.current_height;
}

static void flight_capture_failsafe_snapshot(uint8_t reasons)
{
    if(flight_failsafe_snapshot.valid != 0u)
    {
        return;
    }

    flight_failsafe_snapshot.valid = 1u;
    flight_failsafe_snapshot.pending = 1u;
    flight_failsafe_snapshot.reason_flags = reasons;
    flight_failsafe_snapshot.imu_valid = imu_is_valid();
    flight_failsafe_snapshot.imu_attitude_valid = imu_attitude_is_valid();
    flight_failsafe_snapshot.tof_ok = tof_health_ok();
    flight_failsafe_snapshot.flow_valid = vehicle_state.flow_valid;
    flight_failsafe_snapshot.flow_quality = lc302_data.quality;
    flight_failsafe_snapshot.time_us = system_time_us();
    flight_failsafe_snapshot.imu_bad_frame_count = imu_get_bad_frame_count();
    flight_failsafe_snapshot.lc302_bytes = lc302_data.byte_count;
    flight_failsafe_snapshot.lc302_frames = lc302_data.frame_count;
    flight_failsafe_snapshot.voltage = vehicle_state.battery_voltage_filtered;
    flight_failsafe_snapshot.height_cm = vehicle_state.current_height;
}

static void sensor_safety_update(float dT_s)
{
    uint8_t reasons = 0u;
    uint8_t flow_mode_active;
    uint8_t flow_required;

    if(vehicle_state.armed == 0u)
    {
        flight_imu_fault_time_s = 0.0f;
        flight_tof_fault_time_s = 0.0f;
        flight_flow_fault_time_s = 0.0f;
        flight_flow_required_latched = 0u;

        if(preflight_flow_is_healthy() != 0u)
        {
            preflight_flow_ready_time_s += dT_s;
            if(preflight_flow_ready_time_s > PREFLIGHT_FLOW_READY_TIME_S)
            {
                preflight_flow_ready_time_s = PREFLIGHT_FLOW_READY_TIME_S;
            }
        }
        else
        {
            preflight_flow_ready_time_s = 0.0f;
        }
        return;
    }

    /* A new arm must earn a fresh preflight window after the next disarm. */
    preflight_flow_ready_time_s = 0.0f;

    if((auto_landing_request != 0u) || (auto_landing_active != 0u))
    {
        return;
    }

    /* In flight, acceleration quality may be temporarily poor while gyro
     * propagation and attitude remain usable.  Auto-land only on a stale or
     * invalid attitude solution; keep imu_is_valid() for strict preflight. */
    flight_imu_fault_time_s = (imu_attitude_is_valid() == 0u) ?
        (flight_imu_fault_time_s + dT_s) : 0.0f;
    flight_tof_fault_time_s = (tof_health_ok() == 0u) ?
        (flight_tof_fault_time_s + dT_s) : 0.0f;
    flow_mode_active = ((vehicle_state.flight_mode == FLY_POS_HOLD) ||
                        (vehicle_state.flight_mode == FLY_AUTOFLY) ||
                        (vehicle_state.flight_mode == FLY_AUTOTAKEOFF)) ? 1u : 0u;

    /* LOC intentionally has no authority close to the floor.  Motor spool-up
     * can temporarily destroy image quality there, so do not turn that
     * expected low-altitude interval into an immediate auto-land.  Once the
     * aircraft reaches the LOC authority zone, latch the Flow requirement for
     * the rest of this armed flight; a later loss still forces auto-land even
     * if the vehicle descends below the threshold. */
    if((flow_mode_active != 0u) &&
       ((vehicle_state.current_height >= LOC_ENABLE_HEIGHT_CM) ||
        (loc_1l_ct.loc_ready != 0u) ||
        (loc_1l_ct.loc_weight > 0.0f)))
    {
        flight_flow_required_latched = 1u;
    }
    flow_required = ((flow_mode_active != 0u) &&
                     (flight_flow_required_latched != 0u)) ? 1u : 0u;
    flight_flow_fault_time_s = ((flow_required != 0u) &&
                                (vehicle_state.flow_valid == 0u)) ?
        (flight_flow_fault_time_s + dT_s) : 0.0f;

    if(flight_imu_fault_time_s >= FLIGHT_FAILSAFE_IMU_CONFIRM_S)
    {
        reasons |= PREFLIGHT_ERR_IMU;
    }
    if(flight_tof_fault_time_s >= FLIGHT_FAILSAFE_TOF_CONFIRM_S)
    {
        reasons |= PREFLIGHT_ERR_TOF;
    }
    if(flight_flow_fault_time_s >= FLIGHT_FAILSAFE_FLOW_CONFIRM_S)
    {
        reasons |= PREFLIGHT_ERR_FLOW_NOT_READY;
    }

    if(reasons != 0u)
    {
        flight_sensor_failsafe_flags = reasons;
        flight_capture_failsafe_snapshot(reasons);
        auto_landing_request = 1u;
    }
}



uint8_t preflight_check(void)
{
    uint8_t errors = 0u;
    uint8_t imu_valid = imu_is_valid();
    uint8_t tof_ok = tof_health_ok();

    if(imu_valid == 0u)
    {
        errors |= PREFLIGHT_ERR_IMU;
    }

    if(tof_ok == 0u)
    {
        errors |= PREFLIGHT_ERR_TOF;
    }

#if PREFLIGHT_REQUIRE_FLOW
    if(lc302_data.byte_count == 0u)
    {
        errors |= PREFLIGHT_ERR_FLOW_NO_BYTES;
    }
    else if(lc302_data.frame_count == 0u)
    {
        errors |= PREFLIGHT_ERR_FLOW_NO_FRAME;
    }
    else if(lc302_data.quality < LC302_QUALITY_MIN)
    {
        errors |= PREFLIGHT_ERR_FLOW_QUALITY;
    }
    else if((vehicle_state.flow_valid == 0u) ||
            (preflight_flow_ready_time_s < PREFLIGHT_FLOW_READY_TIME_S))
    {
        errors |= PREFLIGHT_ERR_FLOW_NOT_READY;
    }
#endif

    preflight_error_flags = errors;
    preflight_capture_snapshot(errors, imu_valid, tof_ok);
    if(errors == 0u)
    {
        flight_sensor_failsafe_flags = 0u;
        memset(&flight_failsafe_snapshot, 0, sizeof(flight_failsafe_snapshot));
    }
    return (errors == 0u) ? 1u : 0u;
}


void param_init(void)
{

}

#define DEBUG_STATE_LOG_SAMPLE_COUNT      600u
#define DEBUG_STATE_LOG_SAMPLE_RATE_HZ    50u
#define DEBUG_STATE_LOG_START_HEIGHT_CM   45.0f
#define DEBUG_HISTORY_SAMPLE_COUNT        1000u

typedef struct {
    uint32_t seq;
    uint32_t time_us;
    float dt_ms;
    uint8_t armed;
    uint8_t phase;
    float voltage;
    float att_voltage_scale;
    uint8_t loc_ready;
    uint8_t loc_hold;
    float loc_weight;
    float yaw_deg;
    float pos_err_x_e;
    float pos_err_y_e;
    float goal_pos_x_e;
    float goal_pos_y_e;
    float profile_pos_x_e;
    float profile_pos_y_e;
    float profile_vel_x_e;
    float profile_vel_y_e;
    float vel_x_e;
    float vel_y_e;
    float vel_tgt_x_e;
    float vel_tgt_y_e;
    float recovery_closing_speed;
    float recovery_stop_speed;
    uint8_t recovery_brake_active;
    float vel_err_x_b;
    float vel_err_y_b;
    float loc_raw_roll;
    float loc_raw_pitch;
    float loc_bias_roll;
    float loc_bias_pitch;
    float loc_ramped_roll;
    float loc_ramped_pitch;
    float loc_vel_p_x;
    float loc_vel_i_x;
    float loc_vel_d_x;
    float loc_vel_p_y;
    float loc_vel_i_y;
    float loc_vel_d_y;
    float roll_tgt;
    float roll_cur;
    float pitch_tgt;
    float pitch_cur;
    float sp_rate_ff_roll;
    float sp_rate_ff_pitch;
    float rate_tgt_r;
    float rate_fb_r;
    float rate_out_r;
    float rate_p_r;
    float rate_i_r;
    float rate_d_r;
    float rate_tgt_p;
    float rate_fb_p;
    float rate_out_p;
    float imu_acc_body_y_m_s2;
    float flow_obs_vx_e;
    float flow_obs_vy_e;
    float ekf_vx_e;
    float ekf_vy_e;
    uint8_t flow_valid;
    uint8_t flow_obs_valid;
    uint8_t flow_ekf_used;
    uint8_t flow_gate_clipped;
    float throttle;
    float height_cm;
    float vel_z_cm_s;
    int16_t m1;
    int16_t m2;
    int16_t m3;
    int16_t m4;
} debug_state_sample_t;

/* A compact 50 Hz flight timeline.  It is intentionally independent of the
 * detailed state dump: the detailed buffer keeps PID context, while this
 * buffer retains a longer, always-on history without changing control flow. */
typedef struct {
    uint32_t time_us;
    int16_t pos_err_y_dcm;
    int16_t ekf_vy_dcms;
    int16_t raw_roll_cdeg;
    int16_t roll_cur_cdeg;
    int16_t loc_vel_i_y_mdeg;
    int16_t loc_bias_roll_cdeg;
    int16_t loc_bias_pitch_cdeg;
    int16_t height_dcm;
    uint16_t voltage_cV;
    int16_t flow_obs_vy_dcms;
    int16_t yaw_cdeg;
    int16_t yaw_err_cdeg;
    int16_t yaw_rate_cdps;
    int16_t yaw_rate_tgt_cdps;
    int16_t yaw_out_milli;
    int16_t yaw_rate_p_milli;
    int16_t yaw_rate_i_milli;
    int16_t yaw_rate_d_milli;
    uint16_t m1;
    uint16_t m2;
    uint16_t m3;
    uint16_t m4;
    uint8_t flow_obs_valid;
    uint8_t flow_ekf_used;
    uint8_t flags;
    uint8_t phase;
} debug_history_sample_t;

static debug_state_sample_t debug_state_buf[DEBUG_STATE_LOG_SAMPLE_COUNT];
static debug_history_sample_t debug_history_buf[DEBUG_HISTORY_SAMPLE_COUNT];
static uint16_t debug_state_count = 0u;
static uint16_t debug_state_dump_index = 0u;
static uint8_t debug_state_recording = 0u;
static uint8_t debug_state_ready = 0u;
static uint16_t debug_state_wr_idx = 0u;
static uint8_t debug_state_buffer_full = 0u;
static uint16_t debug_history_count = 0u;
static uint16_t debug_history_dump_index = 0u;
static uint16_t debug_history_wr_idx = 0u;
static uint8_t debug_history_buffer_full = 0u;
static uint8_t debug_history_header_printed = 0u;

static int16_t debug_pack_i16(float value, float scale)
{
    float packed = value * scale;
    if(packed > 32767.0f) return 32767;
    if(packed < -32768.0f) return -32768;
    return (int16_t)((packed >= 0.0f) ? (packed + 0.5f) : (packed - 0.5f));
}

/**
 * @brief Capture one coherent 20 ms control snapshot into RAM.
 * @note Called only at the end of the 20 ms control section. Never prints.
 */
void debug_capture_states_20ms(void)
{
    uint32_t now_us = system_time_us();
    uint8_t armed = vehicle_state.armed;
    static uint32_t last_time_us = 0;

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
        debug_history_count = 0u;
        debug_history_dump_index = 0u;
        debug_history_wr_idx = 0u;
        debug_history_buffer_full = 0u;
        debug_history_header_printed = 0u;
        debug_state_recording = 1u;
        last_time_us = now_us;
    }

    if(((armed == 0u) || (alt_phase == ALT_PHASE_LANDING)) &&
       (debug_state_recording != 0u))
    {
        debug_state_recording = 0u;
        debug_state_ready = (debug_state_count > 0u) ? 1u : 0u;
        debug_state_dump_index = 0u;
        debug_history_dump_index = 0u;
        debug_history_header_printed = 0u;
    }

    if(debug_state_recording != 0u)
    {
        debug_state_sample_t *sample = &debug_state_buf[debug_state_wr_idx];

        sample->seq = debug_state_wr_idx;
        sample->time_us = now_us;
        sample->dt_ms = (last_time_us == 0) ? 0.0f : (float)(now_us - last_time_us) / 1000.0f;
        last_time_us = now_us;

        sample->armed = armed;
        sample->phase = (uint8_t)alt_phase;
        sample->voltage = vehicle_state.battery_voltage_filtered;
        sample->att_voltage_scale = att_voltage_output_scale;
        
        sample->loc_ready = loc_1l_ct.loc_ready;
        sample->loc_hold = loc_1l_ct.loc_hold_ready;
        sample->loc_weight = loc_1l_ct.loc_weight;
        sample->yaw_deg = vehicle_state.current_yaw;
        
        /* Keep the log error referenced to the final mission goal.  The LOC
         * controller's exp_pos is now the moving profile reference. */
        sample->pos_err_x_e = vehicle_setpoint.target_pos_x - vehicle_state.current_pos_x;
        sample->pos_err_y_e = vehicle_setpoint.target_pos_y - vehicle_state.current_pos_y;
        sample->goal_pos_x_e = vehicle_setpoint.target_pos_x;
        sample->goal_pos_y_e = vehicle_setpoint.target_pos_y;
        sample->profile_pos_x_e = loc_2l_ct.exp_pos_x;
        sample->profile_pos_y_e = loc_2l_ct.exp_pos_y;
        sample->profile_vel_x_e = loc_2l_ct.profile_vel_x;
        sample->profile_vel_y_e = loc_2l_ct.profile_vel_y;
        
        sample->vel_x_e = loc_1l_ct.fb_vel_x;
        sample->vel_y_e = loc_1l_ct.fb_vel_y;
        sample->vel_tgt_x_e = loc_1l_ct.exp_vel_x;
        sample->vel_tgt_y_e = loc_1l_ct.exp_vel_y;
        sample->recovery_closing_speed = loc_1l_ct.recovery_closing_speed;
        sample->recovery_stop_speed = loc_1l_ct.recovery_stop_speed;
        sample->recovery_brake_active = loc_1l_ct.recovery_brake_active;
        
        sample->vel_err_x_b = loc_1l_ct.vel_err_x_body;
        sample->vel_err_y_b = loc_1l_ct.vel_err_y_body;
        
        sample->loc_raw_roll = loc_1l_ct.raw_target_roll;
        sample->loc_raw_pitch = loc_1l_ct.raw_target_pitch;
        sample->loc_bias_roll = loc_1l_ct.horizontal_bias_roll;
        sample->loc_bias_pitch = loc_1l_ct.horizontal_bias_pitch;
        sample->loc_ramped_roll = vehicle_setpoint.target_roll;
        sample->loc_ramped_pitch = vehicle_setpoint.target_pitch;
        
        sample->loc_vel_p_x = loc_ctrl.vel_pid[0].out_p;
        sample->loc_vel_i_x = loc_ctrl.vel_pid[0].out_i;
        sample->loc_vel_d_x = loc_ctrl.vel_pid[0].out_d;
        sample->loc_vel_p_y = loc_ctrl.vel_pid[1].out_p;
        sample->loc_vel_i_y = loc_ctrl.vel_pid[1].out_i;
        sample->loc_vel_d_y = loc_ctrl.vel_pid[1].out_d;
        
        sample->roll_tgt = vehicle_setpoint.target_roll;
        sample->roll_cur = vehicle_state.current_roll;
        sample->pitch_tgt = vehicle_setpoint.target_pitch;
        sample->pitch_cur = vehicle_state.current_pitch;
        
        sample->sp_rate_ff_roll = att_1l_ct.sp_rate_ff[0];
        sample->sp_rate_ff_pitch = att_1l_ct.sp_rate_ff[1];
        
        sample->rate_tgt_r = att_1l_ct.exp_ang_vel[0];
        sample->rate_fb_r = att_1l_ct.fb_ang_vel[0];
        sample->rate_out_r = ct_val.rol;
        sample->rate_p_r = att_ctrl.rate_pid[0].out_p;
        sample->rate_i_r = att_ctrl.rate_pid[0].out_i;
        sample->rate_d_r = att_ctrl.rate_pid[0].out_d;
        
        sample->rate_tgt_p = att_1l_ct.exp_ang_vel[1];
        sample->rate_fb_p = att_1l_ct.fb_ang_vel[1];
        sample->rate_out_p = ct_val.pit;
        
        {
            float imu_linear_acc_body[3];
            imu_get_linear_acceleration(imu_linear_acc_body);
            sample->imu_acc_body_y_m_s2 = imu_linear_acc_body[1];
        }
        
        sample->flow_obs_vx_e = flow_health.obs_vx_cm_s;
        sample->flow_obs_vy_e = flow_health.obs_vy_cm_s;
        sample->ekf_vx_e = vehicle_state.current_vel_x;
        sample->ekf_vy_e = vehicle_state.current_vel_y;
        
        sample->flow_valid = vehicle_state.flow_valid;
        sample->flow_obs_valid = flow_health.obs_valid;
        sample->flow_ekf_used = flow_health.ekf_used;
        sample->flow_gate_clipped = flow_health.gate_clipped;
        
        sample->throttle = vehicle_setpoint.target_throttle;
        sample->height_cm = vehicle_state.current_height;
        sample->vel_z_cm_s = vehicle_state.current_vel_z;
        
        sample->m1 = motor_out.m1;
        sample->m2 = motor_out.m2;
        sample->m3 = motor_out.m3;
        sample->m4 = motor_out.m4;

        {
            debug_history_sample_t *history = &debug_history_buf[debug_history_wr_idx];
            uint8_t flags = 0u;

            if(loc_1l_ct.loc_hold_ready != 0u) flags |= (1u << 0);
            if(loc_1l_ct.loc_ready != 0u)      flags |= (1u << 1);
            if(vehicle_state.flow_valid != 0u) flags |= (1u << 2);
            if(flow_health.obs_valid != 0u)    flags |= (1u << 3);
            if(flow_health.ekf_used != 0u)     flags |= (1u << 4);

            history->time_us = now_us;
            history->pos_err_y_dcm = debug_pack_i16(sample->pos_err_y_e, 10.0f);
            history->ekf_vy_dcms = debug_pack_i16(sample->ekf_vy_e, 10.0f);
            history->raw_roll_cdeg = debug_pack_i16(sample->loc_raw_roll, 100.0f);
            history->roll_cur_cdeg = debug_pack_i16(sample->roll_cur, 100.0f);
            history->loc_vel_i_y_mdeg = debug_pack_i16(sample->loc_vel_i_y, 1000.0f);
            history->loc_bias_roll_cdeg = debug_pack_i16(sample->loc_bias_roll, 100.0f);
            history->loc_bias_pitch_cdeg = debug_pack_i16(sample->loc_bias_pitch, 100.0f);
            history->height_dcm = debug_pack_i16(sample->height_cm, 10.0f);
            history->voltage_cV = (uint16_t)LIMIT(sample->voltage * 100.0f + 0.5f, 0.0f, 65535.0f);
            history->flow_obs_vy_dcms = debug_pack_i16(sample->flow_obs_vy_e, 10.0f);
            history->yaw_cdeg = debug_pack_i16(sample->yaw_deg, 100.0f);
            history->yaw_err_cdeg = debug_pack_i16(att_2l_ct.yaw_err, 100.0f);
            history->yaw_rate_cdps = debug_pack_i16(att_1l_ct.fb_ang_vel[2], 100.0f);
            history->yaw_rate_tgt_cdps = debug_pack_i16(att_1l_ct.exp_ang_vel[2], 100.0f);
            history->yaw_out_milli = debug_pack_i16(ct_val.yaw, 1000.0f);
            history->yaw_rate_p_milli = debug_pack_i16(att_ctrl.rate_pid[2].out_p, 1000.0f);
            history->yaw_rate_i_milli = debug_pack_i16(att_ctrl.rate_pid[2].out_i, 1000.0f);
            history->yaw_rate_d_milli = debug_pack_i16(att_ctrl.rate_pid[2].out_d, 1000.0f);
            history->m1 = (uint16_t)LIMIT((float)sample->m1, 0.0f, 65535.0f);
            history->m2 = (uint16_t)LIMIT((float)sample->m2, 0.0f, 65535.0f);
            history->m3 = (uint16_t)LIMIT((float)sample->m3, 0.0f, 65535.0f);
            history->m4 = (uint16_t)LIMIT((float)sample->m4, 0.0f, 65535.0f);
            history->flow_obs_valid = sample->flow_obs_valid;
            history->flow_ekf_used = sample->flow_ekf_used;
            history->flags = flags;
            history->phase = sample->phase;

            debug_history_wr_idx++;
            if(debug_history_wr_idx >= DEBUG_HISTORY_SAMPLE_COUNT)
            {
                debug_history_wr_idx = 0u;
                debug_history_buffer_full = 1u;
            }
            debug_history_count = debug_history_buffer_full ?
                                  DEBUG_HISTORY_SAMPLE_COUNT : debug_history_wr_idx;
        }

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

static void debug_print_safety_snapshots(void)
{
    if(preflight_snapshot.pending != 0u)
    {
        printf("[PREFLIGHT],attempt=%lu,result=%s,flags=0x%02X,time_us=%lu,mode=%u,voltage=%.2f,height_cm=%.2f,imu=%u,tof_ok=%u,tof_raw=%u,tof_stale=%u,tof_stuck=%u,tof_frames=%u,flow=%u,flow_raw=%u,flow_obs=%u,flow_ekf=%u,flow_quality=%u,flow_ready_ms=%.0f,lc302_bytes=%lu,lc302_frames=%lu,lc302_errors=%lu,lc302_last_rx_us=%lu,reason_imu=%u,reason_tof=%u,reason_flow_no_bytes=%u,reason_flow_no_frame=%u,reason_flow_quality=%u,reason_flow_not_ready=%u\r\n",
               (unsigned long)preflight_snapshot.attempt,
               (preflight_snapshot.accepted != 0u) ? "ACCEPTED" : "BLOCKED",
               preflight_snapshot.error_flags,
               (unsigned long)preflight_snapshot.time_us,
               preflight_snapshot.flight_mode,
               preflight_snapshot.voltage,
               preflight_snapshot.height_cm,
               preflight_snapshot.imu_valid,
               preflight_snapshot.tof_ok,
               preflight_snapshot.tof_raw_valid,
               preflight_snapshot.tof_stale,
               preflight_snapshot.tof_stuck,
               preflight_snapshot.tof_valid_frames,
               preflight_snapshot.flow_valid,
               preflight_snapshot.flow_raw_valid,
               preflight_snapshot.flow_obs_valid,
               preflight_snapshot.flow_ekf_used,
               preflight_snapshot.flow_quality,
               preflight_snapshot.flow_ready_time_s * 1000.0f,
               (unsigned long)preflight_snapshot.lc302_bytes,
               (unsigned long)preflight_snapshot.lc302_frames,
               (unsigned long)preflight_snapshot.lc302_errors,
               (unsigned long)preflight_snapshot.lc302_last_rx_us,
               (preflight_snapshot.error_flags & PREFLIGHT_ERR_IMU) ? 1u : 0u,
               (preflight_snapshot.error_flags & PREFLIGHT_ERR_TOF) ? 1u : 0u,
               (preflight_snapshot.error_flags & PREFLIGHT_ERR_FLOW_NO_BYTES) ? 1u : 0u,
               (preflight_snapshot.error_flags & PREFLIGHT_ERR_FLOW_NO_FRAME) ? 1u : 0u,
               (preflight_snapshot.error_flags & PREFLIGHT_ERR_FLOW_QUALITY) ? 1u : 0u,
               (preflight_snapshot.error_flags & PREFLIGHT_ERR_FLOW_NOT_READY) ? 1u : 0u);
        preflight_snapshot.pending = 0u;
    }

    if(flight_failsafe_snapshot.pending != 0u)
    {
        printf("[SENSOR_FAILSAFE],action=AUTO_LAND,flags=0x%02X,time_us=%lu,voltage=%.2f,height_cm=%.2f,imu_strict=%u,imu_attitude=%u,imu_bad_frames=%lu,tof_ok=%u,flow=%u,flow_quality=%u,lc302_bytes=%lu,lc302_frames=%lu,reason_imu=%u,reason_tof=%u,reason_flow=%u\r\n",
               flight_failsafe_snapshot.reason_flags,
               (unsigned long)flight_failsafe_snapshot.time_us,
               flight_failsafe_snapshot.voltage,
               flight_failsafe_snapshot.height_cm,
               flight_failsafe_snapshot.imu_valid,
               flight_failsafe_snapshot.imu_attitude_valid,
               (unsigned long)flight_failsafe_snapshot.imu_bad_frame_count,
               flight_failsafe_snapshot.tof_ok,
               flight_failsafe_snapshot.flow_valid,
               flight_failsafe_snapshot.flow_quality,
               (unsigned long)flight_failsafe_snapshot.lc302_bytes,
               (unsigned long)flight_failsafe_snapshot.lc302_frames,
               (flight_failsafe_snapshot.reason_flags & PREFLIGHT_ERR_IMU) ? 1u : 0u,
               (flight_failsafe_snapshot.reason_flags & PREFLIGHT_ERR_TOF) ? 1u : 0u,
               (flight_failsafe_snapshot.reason_flags & PREFLIGHT_ERR_FLOW_NOT_READY) ? 1u : 0u);
        flight_failsafe_snapshot.pending = 0u;
    }
}

/**
 * @brief Dump the captured RAM buffer after disarm.
 * @note Called from the main loop. Never captures live flight state.
 */
void debug_print_states(void)
{
#if 0 /* Superseded malformed CSV emitter; retained temporarily for diff context. */
    if((debug_state_ready == 0u) || (vehicle_state.armed != 0u))
    {
        return;
    }

    if(debug_state_dump_index == 0u)
    {
        printf("FLIGHTCFG,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.3f,%.3f\r\n",
               att_ctrl.angle_pid[0].kp, att_ctrl.angle_pid[1].kp, att_ctrl.angle_pid[2].kp,
               att_ctrl.rate_pid[0].kp, att_ctrl.rate_pid[0].ki, att_ctrl.rate_pid[0].kd,
               att_ctrl.rate_pid[1].kp, att_ctrl.rate_pid[1].ki, att_ctrl.rate_pid[1].kd,
               att_ctrl.rate_pid[2].kp, att_ctrl.rate_pid[2].ki, att_ctrl.rate_pid[2].kd,
               loc_ctrl.pos_pid[0].kp, loc_ctrl.pos_pid[0].ki, loc_ctrl.pos_pid[0].kd,
               loc_ctrl.pos_pid[1].kp, loc_ctrl.pos_pid[1].ki, loc_ctrl.pos_pid[1].kd,
               loc_ctrl.vel_pid[0].kp, loc_ctrl.vel_pid[0].ki, loc_ctrl.vel_pid[0].kd,
               loc_ctrl.vel_pid[1].kp, loc_ctrl.vel_pid[1].ki, loc_ctrl.vel_pid[1].kd,
               alt_ctrl.vel_pid.kp, alt_ctrl.vel_pid.ki, alt_ctrl.vel_pid.kd,
               loc_ctrl.angle_limit, loc_ctrl.vel_limit);
               
        printf("seq,time_us,dt_ms,armed,phase,voltage,loc_ready,loc_hold,loc_weight,yaw_deg,pos_err_x_e,pos_err_y_e,vel_x_e,vel_y_e,vel_tgt_x_e,vel_tgt_y_e,recovery_closing_speed,recovery_stop_speed,recovery_brake_active,vel_err_x_b,vel_err_y_b,loc_raw_roll,loc_raw_pitch,loc_ramped_roll,loc_ramped_pitch,loc_vel_p_x,loc_vel_i_x,loc_vel_d_x,loc_vel_p_y,loc_vel_i_y,loc_vel_d_y,roll_tgt,roll_cur,pitch_tgt,pitch_cur,sp_rate_ff_roll,sp_rate_ff_pitch,rate_tgt_r,rate_fb_r,rate_out_r,rate_p_r,rate_i_r,rate_d_r,rate_tgt_p,rate_fb_p,rate_out_p,imu_acc_body_y_m_s2,flow_obs_vx_e,flow_obs_vy_e,ekf_vx_e,ekf_vy_e,flow_valid,flow_obs_valid,flow_ekf_used,flow_gate_clipped,throttle,height_cm,vel_z_cm_s,m1,m2,m3,m4,att_voltage_scale,goal_pos_x_e,goal_pos_y_e,profile_pos_x_e,profile_pos_y_e,profile_vel_x_e,profile_vel_y_e\r\n");
    }

    uint8_t lines = 0u;
    while((debug_state_dump_index < debug_state_count) && (lines < 10u))
    {
        uint16_t start_idx = debug_state_buffer_full ? debug_state_wr_idx : 0u;
        uint16_t real_idx = (start_idx + debug_state_dump_index) % DEBUG_STATE_LOG_SAMPLE_COUNT;
        const debug_state_sample_t *sample = &debug_state_buf[real_idx];

        printf("%lu,%lu,%.2f,%u,%u,%.2f,%u,%u,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%u,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.4f,%.2f,%.2f,%.2f,%.2f,%u,%u,%u,%u,%.2f,%.2f,%.2f,%d,%d,%d,%d,%.3f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n",
               (unsigned long)sample->seq, (unsigned long)sample->time_us, sample->dt_ms, sample->armed, sample->phase, sample->voltage,
               sample->loc_ready, sample->loc_hold, sample->loc_weight, sample->yaw_deg,
               sample->pos_err_x_e, sample->pos_err_y_e, sample->vel_x_e, sample->vel_y_e, sample->vel_tgt_x_e, sample->vel_tgt_y_e,
               sample->recovery_closing_speed, sample->recovery_stop_speed, sample->recovery_brake_active,
               sample->vel_err_x_b, sample->vel_err_y_b, sample->loc_raw_roll, sample->loc_raw_pitch, sample->loc_ramped_roll, sample->loc_ramped_pitch,
               sample->loc_vel_p_x, sample->loc_vel_i_x, sample->loc_vel_d_x, sample->loc_vel_p_y, sample->loc_vel_i_y, sample->loc_vel_d_y,
               sample->roll_tgt, sample->roll_cur, sample->pitch_tgt, sample->pitch_cur, sample->sp_rate_ff_roll, sample->sp_rate_ff_pitch,
               sample->rate_tgt_r, sample->rate_fb_r, sample->rate_out_r, sample->rate_p_r, sample->rate_i_r, sample->rate_d_r,
               sample->rate_tgt_p, sample->rate_fb_p, sample->rate_out_p, sample->imu_acc_body_y_m_s2,
               sample->flow_obs_vx_e, sample->flow_obs_vy_e, sample->ekf_vx_e, sample->ekf_vy_e,
               sample->flow_valid, sample->flow_obs_valid, sample->flow_ekf_used, sample->flow_gate_clipped,
               sample->throttle, sample->height_cm, sample->vel_z_cm_s,
               sample->m1, sample->m2, sample->m3, sample->m4,
               sample->att_voltage_scale,
               sample->goal_pos_x_e, sample->goal_pos_y_e,
               sample->profile_pos_x_e, sample->profile_pos_y_e,
               sample->profile_vel_x_e, sample->profile_vel_y_e);

        debug_state_dump_index++;
        lines++;
    }

    if(debug_state_dump_index >= debug_state_count)
    {
        /* Runtime values are loaded from Flash.  Keep this after every CSV
         * sample so a truncated log still retains the configuration. */
        printf("FLIGHTCFG,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.3f,%.3f\r\n",
               att_ctrl.angle_pid[0].kp, att_ctrl.angle_pid[1].kp, att_ctrl.angle_pid[2].kp,
               att_ctrl.rate_pid[0].kp, att_ctrl.rate_pid[0].ki, att_ctrl.rate_pid[0].kd,
               att_ctrl.rate_pid[1].kp, att_ctrl.rate_pid[1].ki, att_ctrl.rate_pid[1].kd,
               att_ctrl.rate_pid[2].kp, att_ctrl.rate_pid[2].ki, att_ctrl.rate_pid[2].kd,
               loc_ctrl.pos_pid[0].kp, loc_ctrl.pos_pid[0].ki, loc_ctrl.pos_pid[0].kd,
               loc_ctrl.pos_pid[1].kp, loc_ctrl.pos_pid[1].ki, loc_ctrl.pos_pid[1].kd,
               loc_ctrl.vel_pid[0].kp, loc_ctrl.vel_pid[0].ki, loc_ctrl.vel_pid[0].kd,
               loc_ctrl.vel_pid[1].kp, loc_ctrl.vel_pid[1].ki, loc_ctrl.vel_pid[1].kd,
               alt_ctrl.vel_pid.kp, alt_ctrl.vel_pid.ki, alt_ctrl.vel_pid.kd,
               loc_ctrl.angle_limit, loc_ctrl.vel_limit);
        printf("DEBUG_STATE_END\r\n");
        debug_state_ready = 0u;
        debug_state_count = 0u;
        debug_state_dump_index = 0u;
    }
#endif

    if(vehicle_state.armed != 0u)
    {
        return;
    }

    if(debug_state_ready == 0u)
    {
        if((preflight_snapshot.pending != 0u) ||
           (flight_failsafe_snapshot.pending != 0u))
        {
            debug_print_safety_snapshots();
            printf("DEBUG_STATE_END\r\n");
        }
        return;
    }

    if(debug_state_dump_index == 0u)
    {
        printf("seq,time_us,dt_ms,armed,phase,voltage,loc_ready,loc_hold,loc_weight,yaw_deg,pos_err_x_e,pos_err_y_e,vel_x_e,vel_y_e,vel_tgt_x_e,vel_tgt_y_e,recovery_closing_speed,recovery_stop_speed,recovery_brake_active,vel_err_x_b,vel_err_y_b,loc_raw_roll,loc_raw_pitch,loc_bias_roll,loc_bias_pitch,loc_ramped_roll,loc_ramped_pitch,loc_vel_p_x,loc_vel_i_x,loc_vel_d_x,loc_vel_p_y,loc_vel_i_y,loc_vel_d_y,roll_tgt,roll_cur,pitch_tgt,pitch_cur,sp_rate_ff_roll,sp_rate_ff_pitch,rate_tgt_r,rate_fb_r,rate_out_r,rate_p_r,rate_i_r,rate_d_r,rate_tgt_p,rate_fb_p,rate_out_p,imu_acc_body_y_m_s2,flow_obs_vx_e,flow_obs_vy_e,ekf_vx_e,ekf_vy_e,flow_valid,flow_obs_valid,flow_ekf_used,flow_gate_clipped,throttle,height_cm,vel_z_cm_s,m1,m2,m3,m4,att_voltage_scale,goal_pos_x_e,goal_pos_y_e,profile_pos_x_e,profile_pos_y_e,profile_vel_x_e,profile_vel_y_e\r\n");
    }

    uint8_t lines = 0u;
    while((debug_state_dump_index < debug_state_count) && (lines < 10u))
    {
        uint16_t start_idx = debug_state_buffer_full ? debug_state_wr_idx : 0u;
        uint16_t real_idx = (start_idx + debug_state_dump_index) % DEBUG_STATE_LOG_SAMPLE_COUNT;
        const debug_state_sample_t *sample = &debug_state_buf[real_idx];

        printf("%lu,%lu,%.2f,%u,%u,%.2f,%u,%u,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%u,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.4f,%.2f,%.2f,%.2f,%.2f,%u,%u,%u,%u,%.2f,%.2f,%.2f,%d,%d,%d,%d,%.3f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n",
               (unsigned long)sample->seq, (unsigned long)sample->time_us, sample->dt_ms, sample->armed, sample->phase, sample->voltage,
               sample->loc_ready, sample->loc_hold, sample->loc_weight, sample->yaw_deg,
               sample->pos_err_x_e, sample->pos_err_y_e, sample->vel_x_e, sample->vel_y_e, sample->vel_tgt_x_e, sample->vel_tgt_y_e,
               sample->recovery_closing_speed, sample->recovery_stop_speed, sample->recovery_brake_active,
               sample->vel_err_x_b, sample->vel_err_y_b, sample->loc_raw_roll, sample->loc_raw_pitch, sample->loc_bias_roll, sample->loc_bias_pitch, sample->loc_ramped_roll, sample->loc_ramped_pitch,
               sample->loc_vel_p_x, sample->loc_vel_i_x, sample->loc_vel_d_x, sample->loc_vel_p_y, sample->loc_vel_i_y, sample->loc_vel_d_y,
               sample->roll_tgt, sample->roll_cur, sample->pitch_tgt, sample->pitch_cur, sample->sp_rate_ff_roll, sample->sp_rate_ff_pitch,
               sample->rate_tgt_r, sample->rate_fb_r, sample->rate_out_r, sample->rate_p_r, sample->rate_i_r, sample->rate_d_r,
               sample->rate_tgt_p, sample->rate_fb_p, sample->rate_out_p, sample->imu_acc_body_y_m_s2,
               sample->flow_obs_vx_e, sample->flow_obs_vy_e, sample->ekf_vx_e, sample->ekf_vy_e,
               sample->flow_valid, sample->flow_obs_valid, sample->flow_ekf_used, sample->flow_gate_clipped,
               sample->throttle, sample->height_cm, sample->vel_z_cm_s,
               sample->m1, sample->m2, sample->m3, sample->m4,
               sample->att_voltage_scale,
               sample->goal_pos_x_e, sample->goal_pos_y_e,
               sample->profile_pos_x_e, sample->profile_pos_y_e,
               sample->profile_vel_x_e, sample->profile_vel_y_e);

        debug_state_dump_index++;
        lines++;
    }

    if(debug_state_dump_index < debug_state_count)
    {
        return;
    }

    if(debug_history_header_printed == 0u)
    {
        /* Packed units: dcm=0.1 cm, dcms=0.1 cm/s, cdeg=0.01 deg,
         * cdps=0.01 deg/s, mdeg=0.001 deg, cV=0.01 V.  Flags: bit0 hold, bit1 ready,
         * bit2 flow_valid, bit3 flow_obs_valid, bit4 flow_ekf_used. */
        printf("HISTORY_STATE_BEGIN,count=%u,fs=%u\r\n",
               debug_history_count, DEBUG_STATE_LOG_SAMPLE_RATE_HZ);
        printf("hseq,time_us,pos_err_y_dcm,ekf_vy_dcms,raw_roll_cdeg,roll_cur_cdeg,loc_vel_i_y_mdeg,loc_bias_roll_cdeg,loc_bias_pitch_cdeg,height_dcm,voltage_cV,flow_obs_vy_dcms,yaw_cdeg,yaw_err_cdeg,yaw_rate_cdps,yaw_rate_tgt_cdps,yaw_out_milli,yaw_rate_p_milli,yaw_rate_i_milli,yaw_rate_d_milli,m1,m2,m3,m4,flow_obs_valid,flow_ekf_used,flags,phase\r\n");
        debug_history_header_printed = 1u;
    }

    lines = 0u;
    while((debug_history_dump_index < debug_history_count) && (lines < 20u))
    {
        uint16_t start_idx = debug_history_buffer_full ? debug_history_wr_idx : 0u;
        uint16_t real_idx = (start_idx + debug_history_dump_index) % DEBUG_HISTORY_SAMPLE_COUNT;
        const debug_history_sample_t *history = &debug_history_buf[real_idx];

        printf("%u,%lu,%d,%d,%d,%d,%d,%d,%d,%d,%u,%d,%d,%d,%d,%d,%d,%d,%d,%d,%u,%u,%u,%u,%u,%u,%u,%u\r\n",
               debug_history_dump_index, (unsigned long)history->time_us,
               history->pos_err_y_dcm, history->ekf_vy_dcms,
               history->raw_roll_cdeg, history->roll_cur_cdeg,
               history->loc_vel_i_y_mdeg, history->loc_bias_roll_cdeg,
               history->loc_bias_pitch_cdeg, history->height_dcm,
               history->voltage_cV, history->flow_obs_vy_dcms, history->yaw_cdeg,
               history->yaw_err_cdeg, history->yaw_rate_cdps, history->yaw_rate_tgt_cdps,
               history->yaw_out_milli, history->yaw_rate_p_milli,
               history->yaw_rate_i_milli, history->yaw_rate_d_milli,
               history->m1, history->m2, history->m3, history->m4,
               history->flow_obs_valid, history->flow_ekf_used,
               history->flags, history->phase);
        debug_history_dump_index++;
        lines++;
    }

    if(debug_history_dump_index >= debug_history_count)
    {
        /* Runtime values are loaded from Flash.  This is deliberately last. */
        printf("FLIGHTCFG,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.3f,%.3f\r\n",
               att_ctrl.angle_pid[0].kp, att_ctrl.angle_pid[1].kp, att_ctrl.angle_pid[2].kp,
               att_ctrl.rate_pid[0].kp, att_ctrl.rate_pid[0].ki, att_ctrl.rate_pid[0].kd,
               att_ctrl.rate_pid[1].kp, att_ctrl.rate_pid[1].ki, att_ctrl.rate_pid[1].kd,
               att_ctrl.rate_pid[2].kp, att_ctrl.rate_pid[2].ki, att_ctrl.rate_pid[2].kd,
               loc_ctrl.pos_pid[0].kp, loc_ctrl.pos_pid[0].ki, loc_ctrl.pos_pid[0].kd,
               loc_ctrl.pos_pid[1].kp, loc_ctrl.pos_pid[1].ki, loc_ctrl.pos_pid[1].kd,
               loc_ctrl.vel_pid[0].kp, loc_ctrl.vel_pid[0].ki, loc_ctrl.vel_pid[0].kd,
               loc_ctrl.vel_pid[1].kp, loc_ctrl.vel_pid[1].ki, loc_ctrl.vel_pid[1].kd,
               alt_ctrl.vel_pid.kp, alt_ctrl.vel_pid.ki, alt_ctrl.vel_pid.kd,
               LOC_MAX_OUTPUT_ANGLE_DEG, MAX_HORIZONTAL_SPEED);
        printf("LOCBIASCFG,enabled=0,steady_bias_source=VEL_I,i_max_cm_s2=20.0\r\n");
        printf("PROFILECFG,pos_correction_cm_s=%.1f,terminal_pos_correction_cm_s=%.1f,terminal_total_vel_cm_s=%.1f,terminal_radius_cm=%.1f,terminal_cruise_radius_cm=%.1f,profile_near_cm_s=%.1f,profile_far_cm_s=%.1f,total_vel_cm_s=%.1f,leash_start_cm=%.1f,leash_max_cm=%.1f,leash_min_speed_scale=%.2f,opposing_ff=blocked,recovery_brake=enabled,terminal_hold=direct_pd,terminal_damping_scale=%.2f,brake_margin_cm_s=%.1f,brake_slew_cm_s2=%.1f,brake_accel_cm_s2=%.1f\r\n",
               LOC_POS_CORRECTION_LIMIT_CM_S,
               LOC_TERMINAL_POS_CORRECTION_LIMIT_CM_S,
               LOC_TERMINAL_TOTAL_VEL_LIMIT_CM_S,
               LOC_TERMINAL_APPROACH_RADIUS_CM,
               LOC_TERMINAL_CRUISE_RADIUS_CM,
               LOC_PROFILE_NEAR_SPEED_CM_S,
               LOC_PROFILE_FAR_SPEED_CM_S,
               LOC_TOTAL_VEL_LIMIT_CM_S,
               LOC_PROFILE_LEASH_START_CM,
               LOC_PROFILE_LEASH_MAX_CM,
               LOC_PROFILE_LEASH_MIN_SPEED_SCALE,
               LOC_TERMINAL_VEL_DAMPING_SCALE,
               LOC_RECOVERY_BRAKE_MARGIN_CM_S,
               LOC_RECOVERY_BRAKE_VEL_SLEW_CM_S2,
               LOC_TRAJ_ACCEL_CM_S2);
        debug_print_safety_snapshots();
        printf("DEBUG_STATE_END\r\n");
        debug_state_ready = 0u;
        debug_state_count = 0u;
        debug_state_dump_index = 0u;
        debug_history_count = 0u;
        debug_history_dump_index = 0u;
        debug_history_header_printed = 0u;
    }
}

/**
 * @brief 核心状态机：管理飞行模式切换、生成各模式下的期望值
 * @param dT_s 时间间隔(秒)
 * @note 这是飞控上层逻辑的核心，决定了飞机在不同模式下的行为。
 */
void param_update(float dT_s)
{
    if(dT_s <= 0.0f || dT_s > 0.1f)
    {
        dT_s = REMOTE_SAMPLE_TIME;
    }

    /* Update health timers before accepting a new arm edge.  In flight this
     * can only request the controlled landing state; it never cuts motors. */
    sensor_safety_update(dT_s);

    // 1. 解析遥控器数据，获得干净的杆量输入 `manual_input`
    prase_remote_ctrl_data(dT_s);

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
        uint8_t auto_land_entering = (auto_landing_active == 0u) ? 1u : 0u;

        auto_landing_active = 1u;
        vehicle_state.flight_mode = FLY_AUTOLANDING;
        vehicle_setpoint.target_height = AUTO_LAND_TARGET_HEIGHT_CM;

        /* Capture the horizontal landing point once.  Rewriting it to the
         * current EKF position every cycle silently cancels optical-flow hold
         * during descent, precisely when a ToF-only fault still leaves the
         * horizontal estimator healthy. */
        if(auto_land_entering != 0u)
        {
            vehicle_setpoint.target_pos_x  = vehicle_state.current_pos_x;
            vehicle_setpoint.target_pos_y  = vehicle_state.current_pos_y;
            vehicle_setpoint.target_vel_x  = 0.0f;
            vehicle_setpoint.target_vel_y  = 0.0f;
            vehicle_setpoint.target_roll = 0.0f;
            vehicle_setpoint.target_pitch = 0.0f;
        }
        vehicle_setpoint.target_yaw_rate = 0.0f;

        /* Normal path uses ToF height/velocity.  If ToF itself caused the
         * failsafe, the altitude state may be stale; in that case the
         * throttle-based landed detector in alt_ctrl provides the fallback. */
        if((alt_phase == ALT_PHASE_LANDED) ||
           (vehicle_state.current_height <= AUTO_LAND_DISARM_HEIGHT_CM))
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
    vehicle_setpoint.target_height = LOC_TEST_TARGET_HEIGHT_CM;
    vehicle_setpoint.target_yaw_rate = 0;//manual_input.yaw_rate;
    //vehicle_setpoint.target_throttle = LIMIT(manual_input.climb_rate / MAX_VEL_XYZ * MAX_CT_VAL, 0.0f, MAX_CT_VAL);

    if(vehicle_state.flow_valid)
    {
        //vehicle_setpoint.target_pos_x += vel_earth_x * dT_s;
        //vehicle_setpoint.target_pos_y += vel_earth_y * dT_s;
        if((vehicle_state.current_height < LOC_ENABLE_HEIGHT_CM) ||
           (loc_1l_ct.loc_hold_ready == 0u))
        {
            vehicle_setpoint.target_pos_x = vehicle_state.current_pos_x;
            vehicle_setpoint.target_pos_y = vehicle_state.current_pos_y;
        }
        else
        {
            /* Position capture is intentionally independent of ALTTRAJ.  As
             * soon as the fixed low-altitude/low-speed LOC gate has completed,
             * start the profile from the captured point toward the local-EKF
             * origin while the altitude trajectory continues to climb. */
            vehicle_setpoint.target_pos_x = LOC_TEST_TARGET_POS_X_CM;
            vehicle_setpoint.target_pos_y = LOC_TEST_TARGET_POS_Y_CM;
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
