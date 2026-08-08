#include "zf_common_headfile.h"
#include "beacon.h"
#include "mcar_comm.h"
#include "mcar_guidance.h"
#include "vision_nav.h"


static float alt_target_vel_z_ramped = 0.0f;
static float alt_target_height_ramped=0.0f;


uint8_t locked_flag = 0;
uint8_t auto_landing_request = 0;
uint8_t auto_landing_active = 0;
volatile uint8_t preflight_error_flags = 0u;
volatile uint8_t flight_sensor_failsafe_flags = 0u;
uint8_t mission_height_recovery_active = 0u;
uint8_t mission_task_requested = 0u;
uint8_t mission_cruise_ready = 0u;
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
    uint32_t mcar_rx_bytes;
    uint32_t mcar_hw_fifo_polls;
    uint32_t mcar_hw_fifo_bytes;
    uint32_t mcar_rx_good_frames;
    uint32_t mcar_rx_bad_frames;
    uint32_t mcar_rx_irqs;
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
    preflight_snapshot.mcar_rx_bytes = mcar_comm_diag.rx_byte_count;
    preflight_snapshot.mcar_hw_fifo_polls = mcar_comm_diag.hw_fifo_poll_count;
    preflight_snapshot.mcar_hw_fifo_bytes = mcar_comm_diag.hw_fifo_byte_count;
    preflight_snapshot.mcar_rx_good_frames = mcar_comm_diag.rx_count;
    preflight_snapshot.mcar_rx_bad_frames = mcar_comm_diag.rx_bad_frame_count;
    preflight_snapshot.mcar_rx_irqs = mcar_comm_diag.rx_irq_count;
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
    else if(vehicle_state.flow_valid == 0u)
    {
        errors |= (lc302_data.quality < LC302_QUALITY_MIN) ?
            PREFLIGHT_ERR_FLOW_QUALITY :
            PREFLIGHT_ERR_FLOW_NOT_READY;
    }
    else if(preflight_flow_ready_time_s < PREFLIGHT_FLOW_READY_TIME_S)
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

#define DEBUG_STATE_LOG_SAMPLE_COUNT      240u
#define DEBUG_STATE_LOG_SAMPLE_RATE_HZ     25u
#define DEBUG_STATE_LOG_START_HEIGHT_CM    0.0f
#define DEBUG_HISTORY_SAMPLE_COUNT           1u
#define MISSION_DEBUG_LOG_ENABLE             1u
#define FLIGHT_RAM_TRACE_ENABLE               1u

typedef struct {
    uint32_t seq;
    uint32_t capture_generation;
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
    float rate_p_p;
    float rate_i_p;
    float rate_d_p;
    float yaw_tgt;
    float yaw_err;
    float rate_tgt_y;
    float rate_fb_y;
    float rate_out_y;
    float rate_p_y;
    float rate_i_y;
    float rate_d_y;
    uint8_t takeoff_yaw_rate_hold;
    float imu_acc_body_y_m_s2;
    float flow_obs_vx_e;
    float flow_obs_vy_e;
    float ekf_vx_e;
    float ekf_vy_e;
    uint8_t flow_valid;
    uint8_t flow_obs_valid;
    uint8_t flow_ekf_used;
    uint8_t flow_gate_clipped;
    uint8_t flow_quality;
    uint16_t flow_accum_count;
    uint32_t flow_integration_us;
    float flow_raw_dx_cm;
    float flow_raw_dy_cm;
    float flow_vel_x_cm_s;
    float flow_vel_y_cm_s;
    float throttle;
    float height_cm;
    float vel_z_cm_s;
    float target_height_cm;
    float target_vel_z_cm_s;
    float final_height_cm;
    float profile_height_cm;
    float profile_vz_cm_s;
    float height_loop_output_cm_s;
    float height_loop_i_cm_s;
    float vel_loop_output;
    float vel_loop_i;
    float throttle_pre_limit;
    float throttle_post_limit;
    float throttle_max;
    int16_t m1;
    int16_t m2;
    int16_t m3;
    int16_t m4;
    uint8_t flight_mode;
    uint8_t beacon_found;
    uint8_t ycar_valid;
    uint8_t search_active;
    uint8_t search_lost_frames;
    uint8_t search_found_frames;
    uint8_t search_spiral_active;
    uint8_t search_center_settled;
    float search_center_distance_cm;
    float search_center_speed_cm_s;
    uint8_t nav_state;
    uint8_t flight_ready;
    uint8_t beacon_track_state;
    uint8_t beacon_track_seen_frames;
    uint8_t target_seq;
    uint8_t car_follow_active;
    uint8_t beacon_loss_brake_active;
    uint8_t car_stop_brake_active;
    float car_distance_cm;
    float car_closing_speed_cm_s;
    float car_brake_vel_cm_s;
    float car_ff_vel_x_cm_s;
    float car_ff_vel_y_cm_s;
    float target_vel_x_cm_s;
    float target_vel_y_cm_s;
    uint32_t car_tx_period_us;
    uint32_t car_rx_period_us;
    uint32_t car_tx_count;
    uint32_t car_rx_count;
    int16_t car_tx_err_forward_px;
    int16_t car_tx_err_right_px;
    int16_t car_tx_yaw_earth_cdeg;
    uint8_t car_tx_flags;
    float car_yaw_body_deg;
    uint8_t car_guidance_valid;
    uint8_t tof_raw_valid;
    uint8_t tof_stale;
    uint8_t tof_stream;
    uint8_t failsafe_flags;
    uint32_t vision_frame_id;
    uint32_t vision_rx_age_ms;
    uint32_t vision_cam_dt_us;
    uint32_t vision_process_us;
    uint32_t vision_display_us;
    uint32_t vision_att_age_ms;
    uint32_t tof_recovery_count;
    float tof_raw_cm;
    float pos_x_cm;
    float pos_y_cm;
} debug_state_sample_t;

typedef char debug_state_sample_must_fit_reserved_trace_window[
    (sizeof(debug_state_sample_t) <= 0x1FCu) ? 1 : -1];

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

/* Keep the full mission trace out of the CM7_1 camera DMA window
 * 0x28026024..0x2802B843. With the attitude diagnostics below, the expected
 * 0x1FC-byte sample keeps the 240-sample ring at
 * 0x28030000..0x2804DC3F, inside CM7_0 RAM and below its heap/stack at
 * 0x2807E000. Verify the exact extent in the post-build map. */
#pragma location = 0x28030000
__no_init static debug_state_sample_t debug_state_buf[DEBUG_STATE_LOG_SAMPLE_COUNT];
static debug_history_sample_t debug_history_buf[DEBUG_HISTORY_SAMPLE_COUNT];
static uint16_t debug_state_count = 0u;
static uint16_t debug_state_dump_index = 0u;
static uint8_t debug_state_recording = 0u;
static uint8_t debug_state_ready = 0u;
static uint16_t debug_state_wr_idx = 0u;
static uint8_t debug_state_buffer_full = 0u;
static uint32_t debug_state_capture_generation = 0u;
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
#if !FLIGHT_RAM_TRACE_ENABLE
    return;
#else
    uint32_t now_us = system_time_us();
    uint8_t armed = vehicle_state.armed;
    debug_state_sample_t *sample;
    static uint32_t last_time_us = 0;
    static uint8_t sample_divider = 0u;

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
        debug_state_capture_generation++;
        if(debug_state_capture_generation == 0u)
        {
            debug_state_capture_generation = 1u;
        }
        debug_history_count = 0u;
        debug_history_dump_index = 0u;
        debug_history_wr_idx = 0u;
        debug_history_buffer_full = 0u;
        debug_history_header_printed = 0u;
        debug_state_recording = 1u;
        sample_divider = 0u;
        last_time_us = now_us;
    }

    /* Keep the pre-descent settling window in the trace.  Once the gate
     * releases, stop before the long blocking post-flight dump can grow. */
    if(((armed == 0u) ||
        ((alt_phase == ALT_PHASE_LANDING) &&
         (auto_landing_active >= AUTO_LAND_STATE_DESCENDING))) &&
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
        sample_divider++;
        if(sample_divider <
           (DEBUG_STATE_LOG_SAMPLE_RATE_HZ >= 50u ?
            1u : (uint8_t)(50u / DEBUG_STATE_LOG_SAMPLE_RATE_HZ)))
        {
            return;
        }
        sample_divider = 0u;

        sample = &debug_state_buf[debug_state_wr_idx];

        sample->seq = debug_state_wr_idx;
        sample->capture_generation = debug_state_capture_generation;
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
        sample->rate_p_p = att_ctrl.rate_pid[1].out_p;
        sample->rate_i_p = att_ctrl.rate_pid[1].out_i;
        sample->rate_d_p = att_ctrl.rate_pid[1].out_d;

        sample->yaw_tgt = att_2l_ct.exp_yaw;
        sample->yaw_err = att_2l_ct.yaw_err;
        sample->rate_tgt_y = att_1l_ct.exp_ang_vel[2];
        sample->rate_fb_y = att_1l_ct.fb_ang_vel[2];
        sample->rate_out_y = ct_val.yaw;
        sample->rate_p_y = att_ctrl.rate_pid[2].out_p;
        sample->rate_i_y = att_ctrl.rate_pid[2].out_i;
        sample->rate_d_y = att_ctrl.rate_pid[2].out_d;
        sample->takeoff_yaw_rate_hold =
            att_takeoff_yaw_rate_hold_active;
        
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
        sample->flow_quality = flow_health.quality;
        sample->flow_accum_count = flow_health.lc302_accum_count;
        sample->flow_integration_us = flow_health.lc302_integration_us;
        sample->flow_raw_dx_cm = flow_health.raw_dx_cm;
        sample->flow_raw_dy_cm = flow_health.raw_dy_cm;
        sample->flow_vel_x_cm_s = flow_health.flow_vel_x_cm_s;
        sample->flow_vel_y_cm_s = flow_health.flow_vel_y_cm_s;
        
        sample->throttle = vehicle_setpoint.target_throttle;
        sample->height_cm = vehicle_state.current_height;
        sample->vel_z_cm_s = vehicle_state.current_vel_z;
        sample->target_height_cm = vehicle_setpoint.target_height;
        sample->target_vel_z_cm_s = vehicle_setpoint.target_vel_z;
        sample->final_height_cm = alt_ctrl_debug.final_height;
        sample->profile_height_cm = alt_ctrl_debug.profile_height;
        sample->profile_vz_cm_s = alt_ctrl_debug.profile_velocity;
        sample->height_loop_output_cm_s = alt_ctrl_debug.position_correction;
        sample->height_loop_i_cm_s = alt_ctrl.height_pid.out_i;
        sample->vel_loop_output = alt_ctrl_debug.pid_raw;
        sample->vel_loop_i = alt_ctrl.vel_pid.out_i;
        sample->throttle_pre_limit = alt_ctrl_debug.throttle_pre_limit;
        sample->throttle_post_limit = alt_ctrl_debug.throttle_post_limit;
        sample->throttle_max = alt_ctrl_debug.max_throttle;
        
        sample->m1 = motor_out.m1;
        sample->m2 = motor_out.m2;
        sample->m3 = motor_out.m3;
        sample->m4 = motor_out.m4;
        sample->flight_mode = (uint8_t)vehicle_state.flight_mode;
        sample->beacon_found =
            (beacon.status == BEACON_FOUND) ? 1u : 0u;
        sample->ycar_valid = ycar_info.valid;
#if BEACON_APPROACH_TEST_ENABLE
        sample->search_active = vision_nav_obs.approach_active;
        sample->search_lost_frames = vision_nav_obs.approach_lost_frames;
        sample->search_found_frames = vision_nav_obs.approach_found_frames;
#else
        sample->search_active = vision_nav_obs.search_move_active;
        sample->search_lost_frames = vision_nav_obs.search_lost_frames;
        sample->search_found_frames = vision_nav_obs.search_found_frames;
#endif
        sample->search_spiral_active = vision_nav_obs.search_spiral_active;
        sample->search_center_settled = vision_nav_obs.search_center_settled;
        sample->search_center_distance_cm =
            vision_nav_obs.search_center_distance_cm;
        sample->search_center_speed_cm_s =
            vision_nav_obs.search_center_speed_cm_s;
        sample->nav_state = (uint8_t)vision_nav_obs.state;
        sample->flight_ready = vision_nav_obs.flight_ready;
        sample->beacon_track_state = vision_nav_obs.beacon_track_state;
        sample->beacon_track_seen_frames =
            vision_nav_obs.beacon_track_seen_frames;
        sample->target_seq = vision_nav_obs.target_seq;
        sample->car_follow_active = vision_nav_obs.car_follow_active;
        sample->beacon_loss_brake_active =
            vision_nav_obs.beacon_loss_brake_active;
        sample->car_stop_brake_active =
            vision_nav_obs.car_stop_brake_active;
        sample->car_distance_cm = vision_nav_obs.car_distance_cm;
        sample->car_closing_speed_cm_s =
            vision_nav_obs.car_closing_speed_cm_s;
        sample->car_brake_vel_cm_s = vision_nav_obs.car_brake_vel_cm_s;
        sample->car_ff_vel_x_cm_s = vision_nav_obs.car_ff_vel_x_cm_s;
        sample->car_ff_vel_y_cm_s = vision_nav_obs.car_ff_vel_y_cm_s;
        sample->target_vel_x_cm_s = vehicle_setpoint.target_vel_x;
        sample->target_vel_y_cm_s = vehicle_setpoint.target_vel_y;
        sample->car_tx_period_us = mcar_comm_diag.tx_period_us;
        sample->car_rx_period_us = mcar_comm_diag.rx_period_us;
        sample->car_tx_count = mcar_comm_diag.tx_count;
        sample->car_rx_count = mcar_comm_diag.rx_count;
        sample->car_tx_err_forward_px = mcar_comm_diag.last_tx_err_forward_px;
        sample->car_tx_err_right_px = mcar_comm_diag.last_tx_err_right_px;
        sample->car_tx_yaw_earth_cdeg = mcar_comm_diag.last_tx_yaw_earth_cdeg;
        sample->car_tx_flags = mcar_comm_diag.last_tx_flags;
        sample->car_yaw_body_deg = mcar_guidance_diag.mcar_yaw_body_deg;
        sample->car_guidance_valid = mcar_guidance_diag.valid;
        sample->tof_raw_valid = tof_health.raw_valid;
        sample->tof_stale = tof_health.stale;
        sample->tof_stream = tof_health.streamcount;
        sample->failsafe_flags = flight_sensor_failsafe_flags;
        sample->vision_frame_id = vision_detection_snapshot.frame_id;
        sample->vision_rx_age_ms = (vision_last_frame_rx_us == 0u) ?
            0xFFFFFFFFu : (now_us - vision_last_frame_rx_us) / 1000u;
        sample->vision_cam_dt_us = vision_detection_snapshot.camera_dt_us;
        sample->vision_process_us = vision_detection_snapshot.process_us;
        sample->vision_display_us = vision_detection_snapshot.display_us;
        sample->vision_att_age_ms =
            (vision_detection_snapshot.attitude_timestamp_us == 0u) ?
            0xFFFFFFFFu :
            (now_us - vision_detection_snapshot.attitude_timestamp_us) / 1000u;
        sample->tof_recovery_count = tof_health.recovery_count;
        sample->tof_raw_cm = tof_health.raw_dist_cm;
        sample->pos_x_cm = vehicle_state.current_pos_x;
        sample->pos_y_cm = vehicle_state.current_pos_y;

#if !MISSION_DEBUG_LOG_ENABLE
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
#endif

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
#endif
}

static void debug_print_safety_snapshots(void)
{
    extern volatile uint8_t diag_pidwrite_pending;
    extern volatile uint8_t diag_pidwrite_stage;
    extern volatile uint32_t diag_pidwrite_time_us;
    extern volatile uint32_t diag_pidwrite_before[3];
    extern volatile uint32_t diag_pidwrite_after[3];

    if(diag_pidwrite_pending != 0u)
    {
        printf("[PIDWRITE],time_us=%lu,stage=%u,before=%08lX/%08lX/%08lX,after=%08lX/%08lX/%08lX,mode=%u,alt_phase=%u,height_cm=%.2f\r\n",
               (unsigned long)diag_pidwrite_time_us,
               diag_pidwrite_stage,
               (unsigned long)diag_pidwrite_before[0],
               (unsigned long)diag_pidwrite_before[1],
               (unsigned long)diag_pidwrite_before[2],
               (unsigned long)diag_pidwrite_after[0],
               (unsigned long)diag_pidwrite_after[1],
               (unsigned long)diag_pidwrite_after[2],
               (unsigned int)vehicle_state.flight_mode,
               (unsigned int)alt_phase,
               vehicle_state.current_height);
        diag_pidwrite_pending = 0u;
    }

    if(preflight_snapshot.pending != 0u)
    {
        printf("[PREFLIGHT],attempt=%lu,result=%s,flags=0x%02X,time_us=%lu,mode=%u,voltage=%.2f,height_cm=%.2f,imu=%u,tof_ok=%u,tof_raw=%u,tof_stale=%u,tof_stuck=%u,tof_frames=%u,flow=%u,flow_raw=%u,flow_obs=%u,flow_ekf=%u,flow_quality=%u,flow_ready_ms=%.0f,lc302_bytes=%lu,lc302_frames=%lu,lc302_errors=%lu,lc302_last_rx_us=%lu,mcar_rx_irqs=%lu,mcar_fifo_polls=%lu,mcar_fifo_bytes=%lu,mcar_rx_bytes=%lu,mcar_rx_good=%lu,mcar_rx_bad=%lu,reason_imu=%u,reason_tof=%u,reason_flow_no_bytes=%u,reason_flow_no_frame=%u,reason_flow_quality=%u,reason_flow_not_ready=%u\r\n",
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
               (unsigned long)preflight_snapshot.mcar_rx_irqs,
               (unsigned long)preflight_snapshot.mcar_hw_fifo_polls,
               (unsigned long)preflight_snapshot.mcar_hw_fifo_bytes,
               (unsigned long)preflight_snapshot.mcar_rx_bytes,
               (unsigned long)preflight_snapshot.mcar_rx_good_frames,
               (unsigned long)preflight_snapshot.mcar_rx_bad_frames,
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
        if((flight_failsafe_snapshot.reason_flags &
            PREFLIGHT_ERR_FLOW_NOT_READY) != 0u)
        {
            lc302_debug_print_packet_history();
        }
        flight_failsafe_snapshot.pending = 0u;
    }
}

/**
 * @brief Dump the captured RAM buffer after disarm.
 * @note Called from the main loop. Never captures live flight state.
 */
void debug_print_states(void)
{
    /*
     * Safety snapshots and the tiny LC302 packet history do not depend on the
     * optional flight RAM trace.  Always service them after disarm, even when
     * debug_state_ready can never be set because the large trace is disabled.
     */
    if(vehicle_state.armed == 0u)
    {
        debug_print_safety_snapshots();
    }

#if MISSION_DEBUG_LOG_ENABLE
    uint8_t lines = 0u;

    if((vehicle_state.armed != 0u) || (debug_state_ready == 0u))
    {
        return;
    }

    if(debug_state_dump_index == 0u)
    {
        printf("MISSION_LOG_HEADER,time_us,armed,mode,alt_phase,height_cm,batt_v,"
               "roll_deg,pitch_deg,target_roll_deg,target_pitch_deg,yaw_deg,"
               "flow,loc_ready,loc_hold,"
               "pos_x_cm,pos_y_cm,goal_x_cm,goal_y_cm,vel_x_cm_s,vel_y_cm_s,"
               "beacon,ycar,frame_id,rx_age_ms,cam_dt_us,vision_process_us,"
               "display_us,att_age_ms,search_active,lost_frames,found_frames,"
               "flow_quality,flow_accum_count,flow_integration_us,"
               "flow_raw_dx_cm,flow_raw_dy_cm,"
               "flow_vel_x_cm_s,flow_vel_y_cm_s,flow_gate_clipped,"
               "search_scan_active,search_center_settled,"
               "search_center_distance_cm,search_center_speed_cm_s,"
               "nav_state,flight_ready,beacon_track_state,"
               "beacon_track_seen_frames,target_seq,"
               "car_follow_active,beacon_loss_brake_active,"
               "car_stop_brake_active,car_distance_cm,"
               "car_closing_speed_cm_s,car_brake_vel_cm_s,"
               "car_ff_vel_x_cm_s,car_ff_vel_y_cm_s,"
               "target_vel_x_cm_s,target_vel_y_cm_s,"
               "car_tx_dt_us,car_rx_dt_us,"
               "car_tx_count,car_rx_count,car_err_fwd_px,car_err_right_px,"
               "car_yaw_body_deg,car_yaw_earth_deg,car_tx_flags,car_guidance_valid,"
               "failsafe,tof_raw_cm,tof_raw_valid,"
               "tof_stale,tof_stream,tof_recover,throttle,"
               "target_height_cm,current_vel_z_cm_s,target_vel_z_cm_s,"
               "final_height_cm,profile_height_cm,profile_vz_cm_s,"
               "height_loop_output_cm_s,height_loop_i_cm_s,"
               "vel_loop_output,vel_loop_i,throttle_pre_limit,"
               "throttle_post_limit,throttle_max,m1,m2,m3,m4,"
               "att_voltage_scale,yaw_target_deg,yaw_err_deg,"
               "rate_tgt_roll_dps,rate_fb_roll_dps,rate_out_roll,"
               "rate_p_roll,rate_i_roll,rate_d_roll,"
               "rate_tgt_pitch_dps,rate_fb_pitch_dps,rate_out_pitch,"
               "rate_p_pitch,rate_i_pitch,rate_d_pitch,"
               "rate_tgt_yaw_dps,rate_fb_yaw_dps,rate_out_yaw,"
               "rate_p_yaw,rate_i_yaw,rate_d_yaw,"
               "takeoff_yaw_rate_hold\r\n");
    }

    while((debug_state_dump_index < debug_state_count) && (lines < 10u))
    {
        uint16_t start_idx =
            debug_state_buffer_full ? debug_state_wr_idx : 0u;
        uint16_t real_idx =
            (start_idx + debug_state_dump_index) %
            DEBUG_STATE_LOG_SAMPLE_COUNT;
        const debug_state_sample_t *sample = &debug_state_buf[real_idx];

        if(sample->capture_generation != debug_state_capture_generation)
        {
            debug_state_dump_index++;
            continue;
        }

        printf("MISSION_LOG,%lu,%u,%u,%u,%.1f,%.2f,"
               "%.2f,%.2f,%.2f,%.2f,%.2f,%u,%u,%u,"
               "%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,"
               "%u,%u,%lu,%lu,%lu,%lu,%lu,%lu,%u,%u,%u,"
               "%u,%u,%lu,%.3f,%.3f,%.1f,%.1f,%u,"
               "%u,%u,%.1f,%.1f,"
               "%u,%u,%u,%u,%u,%u,%u,%u,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%lu,%lu,%lu,%lu,%d,%d,%.2f,%.2f,0x%02X,%u,%u,%.1f,%u,%u,%u,%lu,"
               "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%d,%d,%d,"
               "%.3f,%.3f,%.3f,"
               "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,"
               "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,"
               "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%u\r\n",
               (unsigned long)sample->time_us,
               (unsigned int)sample->armed,
               (unsigned int)sample->flight_mode,
               (unsigned int)sample->phase,
               sample->height_cm,
               sample->voltage,
               sample->roll_cur,
               sample->pitch_cur,
               sample->roll_tgt,
               sample->pitch_tgt,
               sample->yaw_deg,
               (unsigned int)sample->flow_valid,
               (unsigned int)sample->loc_ready,
               (unsigned int)sample->loc_hold,
               sample->pos_x_cm,
               sample->pos_y_cm,
               sample->goal_pos_x_e,
               sample->goal_pos_y_e,
               sample->vel_x_e,
               sample->vel_y_e,
               (unsigned int)sample->beacon_found,
               (unsigned int)sample->ycar_valid,
               (unsigned long)sample->vision_frame_id,
               (unsigned long)sample->vision_rx_age_ms,
               (unsigned long)sample->vision_cam_dt_us,
               (unsigned long)sample->vision_process_us,
               (unsigned long)sample->vision_display_us,
               (unsigned long)sample->vision_att_age_ms,
               (unsigned int)sample->search_active,
               (unsigned int)sample->search_lost_frames,
               (unsigned int)sample->search_found_frames,
               (unsigned int)sample->flow_quality,
               (unsigned int)sample->flow_accum_count,
               (unsigned long)sample->flow_integration_us,
               sample->flow_raw_dx_cm,
               sample->flow_raw_dy_cm,
               sample->flow_vel_x_cm_s,
               sample->flow_vel_y_cm_s,
               (unsigned int)sample->flow_gate_clipped,
               (unsigned int)sample->search_spiral_active,
               (unsigned int)sample->search_center_settled,
               sample->search_center_distance_cm,
               sample->search_center_speed_cm_s,
               (unsigned int)sample->nav_state,
               (unsigned int)sample->flight_ready,
               (unsigned int)sample->beacon_track_state,
               (unsigned int)sample->beacon_track_seen_frames,
               (unsigned int)sample->target_seq,
               (unsigned int)sample->car_follow_active,
               (unsigned int)sample->beacon_loss_brake_active,
               (unsigned int)sample->car_stop_brake_active,
               sample->car_distance_cm,
               sample->car_closing_speed_cm_s,
               sample->car_brake_vel_cm_s,
               sample->car_ff_vel_x_cm_s,
               sample->car_ff_vel_y_cm_s,
               sample->target_vel_x_cm_s,
               sample->target_vel_y_cm_s,
               (unsigned long)sample->car_tx_period_us,
               (unsigned long)sample->car_rx_period_us,
               (unsigned long)sample->car_tx_count,
               (unsigned long)sample->car_rx_count,
               sample->car_tx_err_forward_px,
               sample->car_tx_err_right_px,
               sample->car_yaw_body_deg,
               (float)sample->car_tx_yaw_earth_cdeg * 0.01f,
               (unsigned int)sample->car_tx_flags,
               (unsigned int)sample->car_guidance_valid,
               (unsigned int)sample->failsafe_flags,
               sample->tof_raw_cm,
               (unsigned int)sample->tof_raw_valid,
               (unsigned int)sample->tof_stale,
               (unsigned int)sample->tof_stream,
               (unsigned long)sample->tof_recovery_count,
               sample->throttle,
               sample->target_height_cm,
               sample->vel_z_cm_s,
               sample->target_vel_z_cm_s,
               sample->final_height_cm,
               sample->profile_height_cm,
               sample->profile_vz_cm_s,
               sample->height_loop_output_cm_s,
               sample->height_loop_i_cm_s,
               sample->vel_loop_output,
               sample->vel_loop_i,
               sample->throttle_pre_limit,
               sample->throttle_post_limit,
               sample->throttle_max,
               sample->m1,
               sample->m2,
               sample->m3,
               sample->m4,
               sample->att_voltage_scale,
               sample->yaw_tgt,
               sample->yaw_err,
               sample->rate_tgt_r,
               sample->rate_fb_r,
               sample->rate_out_r,
               sample->rate_p_r,
               sample->rate_i_r,
               sample->rate_d_r,
               sample->rate_tgt_p,
               sample->rate_fb_p,
               sample->rate_out_p,
               sample->rate_p_p,
               sample->rate_i_p,
               sample->rate_d_p,
               sample->rate_tgt_y,
               sample->rate_fb_y,
               sample->rate_out_y,
               sample->rate_p_y,
               sample->rate_i_y,
               sample->rate_d_y,
               (unsigned int)sample->takeoff_yaw_rate_hold);

        debug_state_dump_index++;
        lines++;
    }

    if(debug_state_dump_index >= debug_state_count)
    {
        debug_print_safety_snapshots();
        /* Keep runtime identity inside the MISSION block.  Consumers often
         * capture through MISSION_LOG_END only, and this mission-specific
         * branch returns before the legacy HISTORY/config emitter below. */
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
        printf("FLOWCFG,scale_x=%.4f,scale_y=%.4f,quality_min=%u\r\n",
               LC302_FLOW_SCALE_X,
               LC302_FLOW_SCALE_Y,
               (unsigned int)LC302_QUALITY_MIN);
        printf("ATTCTRL_CFG,takeoff_yaw_mode=rate_hold_below_cm,takeoff_yaw_heading_enable_cm=%.1f,heading_handoff=capture_current,roll_pitch_pid_unchanged=1\r\n",
               ATT_TAKEOFF_YAW_HEADING_ENABLE_HEIGHT_CM);
        printf("IMUCFG,mount_yaw_deg=%.1f,mount_yaw_positive=clockwise_top_view,gyro_acc_xy_same_rotation=1\r\n",
               IMU_MOUNT_YAW_DEG);
        printf("PROFILECFG,pos_correction_cm_s=%.1f,terminal_pos_correction_cm_s=%.1f,terminal_total_vel_cm_s=%.1f,terminal_radius_cm=%.1f,terminal_cruise_radius_cm=%.1f,profile_near_cm_s=%.1f,profile_far_cm_s=%.1f,total_vel_cm_s=%.1f,target_vel_slew_cm_s2=%.1f,car_stop_brake_slew_cm_s2=%.1f,traj_accel_cm_s2=%.1f,max_accel_cm_s2=%.1f,max_angle_deg=%.1f,terminal_damping_scale=%.2f,height_recovery_relative_speed_cm_s=%.1f\r\n",
               LOC_POS_CORRECTION_LIMIT_CM_S,
               LOC_TERMINAL_POS_CORRECTION_LIMIT_CM_S,
               LOC_TERMINAL_TOTAL_VEL_LIMIT_CM_S,
               LOC_TERMINAL_APPROACH_RADIUS_CM,
               LOC_TERMINAL_CRUISE_RADIUS_CM,
               LOC_PROFILE_NEAR_SPEED_CM_S,
               LOC_PROFILE_FAR_SPEED_CM_S,
               LOC_TOTAL_VEL_LIMIT_CM_S,
               LOC_TARGET_VEL_SLEW_CM_S2,
               LOC_CAR_STOP_BRAKE_VEL_SLEW_CM_S2,
               LOC_TRAJ_ACCEL_CM_S2,
               LOC_MAX_HORIZONTAL_ACCEL_CM_S2,
               LOC_MAX_OUTPUT_ANGLE_DEG,
               LOC_TERMINAL_VEL_DAMPING_SCALE,
               MISSION_HEIGHT_RECOVERY_RELATIVE_SPEED_LIMIT_CM_S);
        printf("NAVCFG,search_pattern=simple_finish_v2,direct_car_follow=%u,direct_target_mode=latched_increment,local_serpentine=%u,after_center=%s,search_center_x_cm=%.1f,search_center_y_cm=%.1f,search_center_capture_cm=%.1f,search_center_max_speed_cm_s=%.1f,search_center_settle_ms=%lu,search_hard_half_x_cm=%.1f,search_hard_half_y_cm=%.1f,search_scan_half_x_cm=%.1f,search_scan_half_y_cm=%.1f,search_inset_cm=%.1f,search_waypoint_capture_cm=%.1f,search_speed_limit_cm_s=%.1f,direct_px_to_cm=%.2f,direct_lpf_alpha=%.2f,direct_max_jump_px=%.1f,direct_deadzone_px=%.1f,direct_target_step_cm=%.2f,direct_target_leash_cm=%.1f,car_follow_speed_limit_cm_s=%.1f,takeoff_horizontal_enable_cm=%.1f,takeoff_horizontal_release_cm=%.1f,takeoff_hold_capture=current,takeoff_initial_angle_deg=%.1f,takeoff_initial_angle_until_cm=%.1f,takeoff_fault_guard=brake_no_land,horiz_fault_speed_cm_s=%.1f,horiz_fault_critical_cm_s=%.1f,horiz_fault_bad_frames=%u,horiz_fault_clear_cm_s=%.1f,horiz_fault_land_ms=%lu,horiz_fault_angle_deg=%.1f\r\n",
               (unsigned int)VISION_NAV_SIMPLE_DIRECT_CAR_FOLLOW,
               (unsigned int)VISION_NAV_LOCAL_SERPENTINE_ENABLE,
               (VISION_NAV_LOCAL_SERPENTINE_ENABLE != 0) ?
                   "serpentine" : "hold",
               VISION_NAV_SEARCH_CENTER_X_CM,
               VISION_NAV_SEARCH_CENTER_Y_CM,
               VISION_NAV_SEARCH_CENTER_CAPTURE_CM,
               VISION_NAV_SEARCH_CENTER_MAX_SPEED_CM_S,
               (unsigned long)(VISION_NAV_SEARCH_CENTER_SETTLE_US / 1000u),
               VISION_NAV_SEARCH_HALF_WIDTH_X_CM,
               VISION_NAV_SEARCH_HALF_WIDTH_Y_CM,
               VISION_NAV_SEARCH_SCAN_HALF_WIDTH_X_CM,
               VISION_NAV_SEARCH_SCAN_HALF_WIDTH_Y_CM,
               VISION_NAV_SEARCH_GEOFENCE_INSET_CM,
               VISION_NAV_SEARCH_WAYPOINT_CAPTURE_CM,
               VISION_NAV_SEARCH_SPEED_LIMIT_CM_S,
               VISION_NAV_DIRECT_CAR_PX_TO_CM,
               VISION_NAV_DIRECT_CAR_LPF_ALPHA,
               VISION_NAV_DIRECT_CAR_MAX_JUMP_PX,
               VISION_NAV_DIRECT_CAR_DEADZONE_PX,
               VISION_NAV_DIRECT_CAR_TARGET_STEP_CM,
               VISION_NAV_DIRECT_CAR_TARGET_LEASH_CM,
               VISION_NAV_CAR_FOLLOW_SPEED_LIMIT_CM_S,
               LOC_AUTO_TAKEOFF_HORIZONTAL_ENABLE_HEIGHT_CM,
               LOC_AUTO_TAKEOFF_HORIZONTAL_RELEASE_HEIGHT_CM,
               LOC_AUTO_TAKEOFF_INITIAL_MAX_OUTPUT_ANGLE_DEG,
               LOC_AUTO_TAKEOFF_INITIAL_ANGLE_LIMIT_HEIGHT_CM,
               VISION_NAV_HORIZONTAL_FAULT_SPEED_CM_S,
               VISION_NAV_HORIZONTAL_FAULT_CRITICAL_SPEED_CM_S,
               (unsigned int)VISION_NAV_HORIZONTAL_FAULT_BAD_FRAMES,
               VISION_NAV_HORIZONTAL_FAULT_CLEAR_SPEED_CM_S,
               (unsigned long)(VISION_NAV_HORIZONTAL_FAULT_LAND_US / 1000u),
               LOC_HORIZONTAL_FAULT_MAX_ANGLE_DEG);
        printf("FAILSAFECFG,imu_confirm_ms=%lu,tof_confirm_ms=%lu,flow_confirm_ms=%lu,remote_land_switch=immediate,horiz_fault_land_ms=%lu,radio_loss_autoland=%u\r\n",
               (unsigned long)(FLIGHT_FAILSAFE_IMU_CONFIRM_S * 1000.0f),
               (unsigned long)(FLIGHT_FAILSAFE_TOF_CONFIRM_S * 1000.0f),
               (unsigned long)(FLIGHT_FAILSAFE_FLOW_CONFIRM_S * 1000.0f),
               (unsigned long)(VISION_NAV_HORIZONTAL_FAULT_LAND_US / 1000u),
               (unsigned int)RADIO_LOSS_AUTOLAND_ENABLE);
        printf("BEACONPLANCFG,center_policy=%u,center_radius_cm=%.1f,center_only_frames=%u,observation_backoff_cm=%.1f,return_center_after_target=1,locked_candidate_radius_px=%d\r\n",
               (unsigned int)VISION_NAV_CENTER_BEACON_POLICY_ENABLE,
               VISION_NAV_CENTER_BEACON_RADIUS_CM,
               (unsigned int)VISION_NAV_CENTER_ONLY_CONFIRM_FRAMES,
               VISION_NAV_CENTER_OBSERVATION_BACKOFF_CM,
               VISION_NAV_LOCKED_CANDIDATE_RADIUS_PX);
        printf("YAWPOSCFG,pos_gain_scale=%.2f,pos_correction_limit_cm_s=%.1f,total_vel_limit_cm_s=%.1f,angle_limit_deg=%.1f,spin_mode=segmented_3x120,segment_deg=%.1f,segment_pause_ms=%lu,finish_tolerance_deg=%.1f,settle_ms=%lu\r\n",
               LOC_YAW_SPIN_POS_GAIN_SCALE,
               LOC_YAW_SPIN_POS_CORRECTION_LIMIT_CM_S,
               LOC_YAW_SPIN_TOTAL_VEL_LIMIT_CM_S,
               VISION_NAV_YAW_SPIN_LOC_ANGLE_DEG,
               VISION_NAV_YAW_SPIN_SEGMENT_DEG,
               (unsigned long)(VISION_NAV_YAW_SPIN_SEGMENT_PAUSE_US / 1000u),
               VISION_NAV_YAW_SPIN_FINISH_TOLERANCE_DEG,
               (unsigned long)(VISION_NAV_YAW_SPIN_SETTLE_US / 1000u));
        printf("YAWLOADCFG,enabled=%u,target_rate_dps=%.1f,max_rate_dps=%.1f,ct_limit=%.1f,recovery_mode=pause_resume,recovery_trigger_height_cm=%.1f,recovery_trigger_vz_cm_s=%.1f,recovery_resume_height_cm=%.1f,recovery_resume_vz_cm_s=%.1f\r\n",
               (unsigned int)VISION_NAV_YAW_SPIN_ENABLE,
               VISION_NAV_YAW_SPIN_RATE_DPS,
               VISION_NAV_YAW_SPIN_MAX_RATE_DPS,
               VISION_NAV_YAW_SPIN_CT_LIMIT,
               VISION_NAV_YAW_SPIN_RECOVERY_TRIGGER_HEIGHT_CM,
               VISION_NAV_YAW_SPIN_RECOVERY_TRIGGER_VZ_CM_S,
               VISION_NAV_YAW_SPIN_RECOVERY_RESUME_HEIGHT_CM,
               VISION_NAV_YAW_SPIN_RECOVERY_RESUME_VZ_CM_S);
        printf("TAKEOFFPOSCFG,damping_enable_cm=%.1f,hold_enable_cm=%.1f,release_cm=%.1f,preserve_launch_target=1,initial_angle_limit_deg=%.1f\r\n",
               LOC_AUTO_TAKEOFF_HORIZONTAL_ENABLE_HEIGHT_CM,
               LOC_AUTO_LOW_HOLD_HEIGHT_CM,
               LOC_AUTO_TAKEOFF_HORIZONTAL_RELEASE_HEIGHT_CM,
               LOC_AUTO_TAKEOFF_INITIAL_MAX_OUTPUT_ANGLE_DEG);
        printf("CARCMDCFG,aircraft_tx_shaper=%u,command_px=%d,slew_px_per_frame=%d,preserve_vector_ratio=1,force_recovery_slow=%u,handheld_follow_test=%u,handheld_beacon_follow_test=%u,che_pos_to_speed=8,che_arrive_threshold_px=5,expected_dominant_axis_cm_s=40\r\n",
               (unsigned int)MCAR_COMM_TX_SHAPER_ENABLE,
               MCAR_COMM_TX_SHAPER_COMMAND_PX,
               MCAR_COMM_TX_SHAPER_SLEW_PX_PER_FRAME,
               (unsigned int)MCAR_COMM_TX_SHAPER_FORCE_SLOW_FLAG,
               (unsigned int)MCAR_COMM_HANDHELD_FOLLOW_TEST_ENABLE,
               (unsigned int)MCAR_COMM_HANDHELD_BEACON_FOLLOW_TEST_ENABLE);
        printf("LANDCFG,pre_descent_settle=1,horiz_speed_cm_s=%.1f,pos_err_cm=%.1f,vz_cm_s=%.1f,stable_ms=%lu,timeout_ms=%lu,fault_bypass=1,takeoff_gate_separate=1\r\n",
               AUTO_LAND_SETTLE_HORIZ_SPEED_CM_S,
               AUTO_LAND_SETTLE_POS_ERR_CM,
               AUTO_LAND_SETTLE_VZ_CM_S,
               (unsigned long)(AUTO_LAND_SETTLE_STABLE_TIME_S * 1000.0f),
               (unsigned long)(AUTO_LAND_SETTLE_TIMEOUT_S * 1000.0f));
        printf("MISSION_LOG_END\r\n");
        printf("DEBUG_STATE_END\r\n");
        debug_state_ready = 0u;
        debug_state_count = 0u;
        debug_state_dump_index = 0u;
        debug_history_count = 0u;
        debug_history_dump_index = 0u;
        debug_history_header_printed = 0u;
    }
    return;
#else
#if 0 /* Live MISSION CSV blocks the main-loop ToF update; keep flight silent. */
    static uint8_t mission_print_divider = 0u;
    static uint8_t mission_header_printed = 0u;
    uint32_t now_us;
    uint32_t frame_age_ms;
    uint32_t attitude_age_ms;

    mission_print_divider++;
    if(mission_print_divider < 10u)
    {
        return;
    }
    mission_print_divider = 0u;

    now_us = system_time_us();
    frame_age_ms = (vision_last_frame_rx_us == 0u) ?
        0xFFFFFFFFu :
        (now_us - vision_last_frame_rx_us) / 1000u;
    attitude_age_ms = (vision_detection_snapshot.attitude_timestamp_us == 0u) ?
        0xFFFFFFFFu :
        (now_us - vision_detection_snapshot.attitude_timestamp_us) / 1000u;

    if(mission_header_printed == 0u)
    {
        printf("MISSION_HEADER,time_us,armed,rc_ok,arm_sw,preflight_err,"
               "flow_quality,flow_ready_ms,"
               "failsafe,tof_raw_cm,tof_raw_valid,tof_stale,tof_stream,tof_recover,"
               "mode,alt_phase,height_cm,batt_v,"
               "roll_deg,pitch_deg,yaw_deg,flow,loc_ready,loc_hold,"
               "pos_x_cm,pos_y_cm,goal_x_cm,goal_y_cm,vel_x_cm_s,vel_y_cm_s,"
               "beacon,ycar,frame_id,rx_age_ms,cam_dt_us,vision_process_us,"
               "display_us,att_age_ms,search_active,lost_frames,"
               "found_frames,nav_state,target_seq,car_fb,car_status,"
               "car_speed_fwd_cm_s,car_speed_right_cm_s\r\n");
        mission_header_printed = 1u;
    }

    printf("MISSION,%lu,%u,%u,%u,%u,%u,%lu,%u,%.1f,%u,%u,%u,%lu,"
           "%u,%u,%.1f,%.2f,"
           "%.2f,%.2f,%.2f,%u,%u,%u,"
           "%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,"
           "%u,%u,%lu,%lu,%lu,%lu,%lu,%lu,%u,%u,%u,%u,%u,%u,%u,%d,%d\r\n",
           (unsigned long)now_us,
           (unsigned int)vehicle_state.armed,
           (unsigned int)lora3a22_state_flag,
           (unsigned int)lora3a22_uart_transfer.switch_key[1],
           (unsigned int)preflight_error_flags,
           (unsigned int)lc302_data.quality,
           (unsigned long)(preflight_flow_ready_time_s * 1000.0f),
           (unsigned int)flight_sensor_failsafe_flags,
           tof_health.raw_dist_cm,
           (unsigned int)tof_health.raw_valid,
           (unsigned int)tof_health.stale,
           (unsigned int)tof_health.streamcount,
           (unsigned long)tof_health.recovery_count,
           (unsigned int)vehicle_state.flight_mode,
           (unsigned int)alt_phase,
           vehicle_state.current_height,
           vehicle_state.battery_voltage_filtered,
           vehicle_state.current_roll,
           vehicle_state.current_pitch,
           vehicle_state.current_yaw,
           (unsigned int)vehicle_state.flow_valid,
           (unsigned int)loc_1l_ct.loc_ready,
           (unsigned int)loc_1l_ct.loc_hold_ready,
           vehicle_state.current_pos_x,
           vehicle_state.current_pos_y,
           vehicle_setpoint.target_pos_x,
           vehicle_setpoint.target_pos_y,
           loc_1l_ct.fb_vel_x,
           loc_1l_ct.fb_vel_y,
           (unsigned int)((beacon.status == BEACON_FOUND) ? 1u : 0u),
           (unsigned int)ycar_info.valid,
           (unsigned long)vision_detection_snapshot.frame_id,
           (unsigned long)frame_age_ms,
           (unsigned long)vision_detection_snapshot.camera_dt_us,
           (unsigned long)vision_detection_snapshot.process_us,
           (unsigned long)vision_detection_snapshot.display_us,
           (unsigned long)attitude_age_ms,
#if BEACON_APPROACH_TEST_ENABLE
           (unsigned int)vision_nav_obs.approach_active,
           (unsigned int)vision_nav_obs.approach_lost_frames,
           (unsigned int)vision_nav_obs.approach_found_frames,
#else
           (unsigned int)vision_nav_obs.search_move_active,
           (unsigned int)vision_nav_obs.search_lost_frames,
           (unsigned int)vision_nav_obs.search_found_frames,
#endif
           (unsigned int)vision_nav_obs.state,
           (unsigned int)vision_nav_obs.target_seq,
           (unsigned int)mcar_comm_feedback.valid,
           (unsigned int)mcar_comm_feedback.status,
           (int)mcar_comm_feedback.speed_forward_cm_s,
           (int)mcar_comm_feedback.speed_right_cm_s);
    return;
#endif

#if 1 /* Capture in RAM while armed, then dump in bounded chunks after lock. */
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
        printf("PROFILECFG,pos_correction_cm_s=%.1f,terminal_pos_correction_cm_s=%.1f,terminal_total_vel_cm_s=%.1f,terminal_radius_cm=%.1f,terminal_cruise_radius_cm=%.1f,profile_near_cm_s=%.1f,profile_far_cm_s=%.1f,total_vel_cm_s=%.1f,leash_start_cm=%.1f,leash_max_cm=%.1f,leash_min_speed_scale=%.2f,opposing_ff=blocked,recovery_brake=enabled,terminal_hold=direct_pd,terminal_damping_scale=%.2f,brake_margin_cm_s=%.1f,brake_slew_cm_s2=%.1f,car_stop_brake_slew_cm_s2=%.1f,brake_accel_cm_s2=%.1f\r\n",
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
               LOC_CAR_STOP_BRAKE_VEL_SLEW_CM_S2,
               LOC_TRAJ_ACCEL_CM_S2);
        printf("NAVCFG,search_pattern=simple_finish_v2,direct_car_follow=%u,direct_target_mode=latched_increment,local_serpentine=%u,after_center=%s,search_center_capture_cm=%.1f,search_center_max_speed_cm_s=%.1f,search_center_settle_ms=%lu,search_half_x_cm=%.1f,search_half_y_cm=%.1f,search_waypoint_capture_cm=%.1f,search_speed_limit_cm_s=%.1f,direct_px_to_cm=%.2f,direct_lpf_alpha=%.2f,direct_max_jump_px=%.1f,direct_deadzone_px=%.1f,direct_target_step_cm=%.2f,direct_target_leash_cm=%.1f,car_follow_speed_limit_cm_s=%.1f,alt_vel_ff_gain=%.3f,takeoff_horizontal_enable_cm=%.1f,takeoff_horizontal_release_cm=%.1f,takeoff_hold_capture=current,takeoff_initial_angle_deg=%.1f,takeoff_initial_angle_until_cm=%.1f,takeoff_fault_guard=brake_no_land,horiz_fault_speed_cm_s=%.1f,horiz_fault_critical_cm_s=%.1f,horiz_fault_bad_frames=%u,horiz_fault_clear_cm_s=%.1f,horiz_fault_land_ms=%lu,horiz_fault_angle_deg=%.1f\r\n",
               (unsigned int)VISION_NAV_SIMPLE_DIRECT_CAR_FOLLOW,
               (unsigned int)VISION_NAV_LOCAL_SERPENTINE_ENABLE,
               (VISION_NAV_LOCAL_SERPENTINE_ENABLE != 0) ?
                   "serpentine" : "hold",
               VISION_NAV_SEARCH_CENTER_CAPTURE_CM,
               VISION_NAV_SEARCH_CENTER_MAX_SPEED_CM_S,
               (unsigned long)(VISION_NAV_SEARCH_CENTER_SETTLE_US / 1000u),
               VISION_NAV_SEARCH_HALF_WIDTH_X_CM,
               VISION_NAV_SEARCH_HALF_WIDTH_Y_CM,
               VISION_NAV_SEARCH_WAYPOINT_CAPTURE_CM,
               VISION_NAV_SEARCH_SPEED_LIMIT_CM_S,
               VISION_NAV_DIRECT_CAR_PX_TO_CM,
               VISION_NAV_DIRECT_CAR_LPF_ALPHA,
               VISION_NAV_DIRECT_CAR_MAX_JUMP_PX,
               VISION_NAV_DIRECT_CAR_DEADZONE_PX,
               VISION_NAV_DIRECT_CAR_TARGET_STEP_CM,
               VISION_NAV_DIRECT_CAR_TARGET_LEASH_CM,
               VISION_NAV_CAR_FOLLOW_SPEED_LIMIT_CM_S,
               ALT_VEL_FEEDFORWARD_GAIN,
               LOC_AUTO_TAKEOFF_HORIZONTAL_ENABLE_HEIGHT_CM,
               LOC_AUTO_TAKEOFF_HORIZONTAL_RELEASE_HEIGHT_CM,
               LOC_AUTO_TAKEOFF_INITIAL_MAX_OUTPUT_ANGLE_DEG,
               LOC_AUTO_TAKEOFF_INITIAL_ANGLE_LIMIT_HEIGHT_CM,
               VISION_NAV_HORIZONTAL_FAULT_SPEED_CM_S,
               VISION_NAV_HORIZONTAL_FAULT_CRITICAL_SPEED_CM_S,
               (unsigned int)VISION_NAV_HORIZONTAL_FAULT_BAD_FRAMES,
               VISION_NAV_HORIZONTAL_FAULT_CLEAR_SPEED_CM_S,
               (unsigned long)(VISION_NAV_HORIZONTAL_FAULT_LAND_US / 1000u),
               LOC_HORIZONTAL_FAULT_MAX_ANGLE_DEG);
        printf("FAILSAFECFG,imu_confirm_ms=%lu,tof_confirm_ms=%lu,flow_confirm_ms=%lu,remote_land_switch=immediate,horiz_fault_land_ms=%lu,radio_loss_autoland=%u\r\n",
               (unsigned long)(FLIGHT_FAILSAFE_IMU_CONFIRM_S * 1000.0f),
               (unsigned long)(FLIGHT_FAILSAFE_TOF_CONFIRM_S * 1000.0f),
               (unsigned long)(FLIGHT_FAILSAFE_FLOW_CONFIRM_S * 1000.0f),
               (unsigned long)(VISION_NAV_HORIZONTAL_FAULT_LAND_US / 1000u),
               (unsigned int)RADIO_LOSS_AUTOLAND_ENABLE);
        printf("BEACONPLANCFG,center_policy=%u,center_radius_cm=%.1f,center_only_frames=%u,observation_backoff_cm=%.1f,return_center_after_target=1,locked_candidate_radius_px=%d\r\n",
               (unsigned int)VISION_NAV_CENTER_BEACON_POLICY_ENABLE,
               VISION_NAV_CENTER_BEACON_RADIUS_CM,
               (unsigned int)VISION_NAV_CENTER_ONLY_CONFIRM_FRAMES,
               VISION_NAV_CENTER_OBSERVATION_BACKOFF_CM,
               VISION_NAV_LOCKED_CANDIDATE_RADIUS_PX);
        printf("YAWPOSCFG,pos_gain_scale=%.2f,pos_correction_limit_cm_s=%.1f,total_vel_limit_cm_s=%.1f,angle_limit_deg=%.1f,spin_mode=segmented_3x120,segment_deg=%.1f,segment_pause_ms=%lu,finish_tolerance_deg=%.1f,settle_ms=%lu\r\n",
               LOC_YAW_SPIN_POS_GAIN_SCALE,
               LOC_YAW_SPIN_POS_CORRECTION_LIMIT_CM_S,
               LOC_YAW_SPIN_TOTAL_VEL_LIMIT_CM_S,
               VISION_NAV_YAW_SPIN_LOC_ANGLE_DEG,
               VISION_NAV_YAW_SPIN_SEGMENT_DEG,
               (unsigned long)(VISION_NAV_YAW_SPIN_SEGMENT_PAUSE_US / 1000u),
               VISION_NAV_YAW_SPIN_FINISH_TOLERANCE_DEG,
               (unsigned long)(VISION_NAV_YAW_SPIN_SETTLE_US / 1000u));
        printf("YAWLOADCFG,enabled=%u,target_rate_dps=%.1f,max_rate_dps=%.1f,ct_limit=%.1f,recovery_mode=pause_resume,recovery_trigger_height_cm=%.1f,recovery_trigger_vz_cm_s=%.1f,recovery_resume_height_cm=%.1f,recovery_resume_vz_cm_s=%.1f\r\n",
               (unsigned int)VISION_NAV_YAW_SPIN_ENABLE,
               VISION_NAV_YAW_SPIN_RATE_DPS,
               VISION_NAV_YAW_SPIN_MAX_RATE_DPS,
               VISION_NAV_YAW_SPIN_CT_LIMIT,
               VISION_NAV_YAW_SPIN_RECOVERY_TRIGGER_HEIGHT_CM,
               VISION_NAV_YAW_SPIN_RECOVERY_TRIGGER_VZ_CM_S,
               VISION_NAV_YAW_SPIN_RECOVERY_RESUME_HEIGHT_CM,
               VISION_NAV_YAW_SPIN_RECOVERY_RESUME_VZ_CM_S);
        printf("TAKEOFFPOSCFG,damping_enable_cm=%.1f,hold_enable_cm=%.1f,release_cm=%.1f,preserve_launch_target=1,initial_angle_limit_deg=%.1f\r\n",
               LOC_AUTO_TAKEOFF_HORIZONTAL_ENABLE_HEIGHT_CM,
               LOC_AUTO_LOW_HOLD_HEIGHT_CM,
               LOC_AUTO_TAKEOFF_HORIZONTAL_RELEASE_HEIGHT_CM,
               LOC_AUTO_TAKEOFF_INITIAL_MAX_OUTPUT_ANGLE_DEG);
        printf("CARCMDCFG,aircraft_tx_shaper=%u,command_px=%d,slew_px_per_frame=%d,preserve_vector_ratio=1,force_recovery_slow=%u,handheld_follow_test=%u,handheld_beacon_follow_test=%u,che_pos_to_speed=8,che_arrive_threshold_px=5,expected_dominant_axis_cm_s=40\r\n",
               (unsigned int)MCAR_COMM_TX_SHAPER_ENABLE,
               MCAR_COMM_TX_SHAPER_COMMAND_PX,
               MCAR_COMM_TX_SHAPER_SLEW_PX_PER_FRAME,
               (unsigned int)MCAR_COMM_TX_SHAPER_FORCE_SLOW_FLAG,
               (unsigned int)MCAR_COMM_HANDHELD_FOLLOW_TEST_ENABLE,
               (unsigned int)MCAR_COMM_HANDHELD_BEACON_FOLLOW_TEST_ENABLE);
        debug_print_safety_snapshots();
        printf("DEBUG_STATE_END\r\n");
        debug_state_ready = 0u;
        debug_state_count = 0u;
        debug_state_dump_index = 0u;
        debug_history_count = 0u;
        debug_history_dump_index = 0u;
        debug_history_header_printed = 0u;
    }
#endif
#endif
}

/**
 * @brief 核心状态机：管理飞行模式切换、生成各模式下的期望值
 * @param dT_s 时间间隔(秒)
 * @note 这是飞控上层逻辑的核心，决定了飞机在不同模式下的行为。
 */
void param_update(float dT_s)
{
    static float mission_cruise_stable_time_s = 0.0f;
    static float mission_height_recovery_stable_time_s = 0.0f;
    static float auto_land_settle_time_s = 0.0f;
    static float auto_land_wait_time_s = 0.0f;
    static float auto_land_target_x_cm = 0.0f;
    static float auto_land_target_y_cm = 0.0f;
    static float auto_land_hold_height_cm = MIN_HEIGHT;
    static uint8_t mission_mode_active_last = 0u;
    static uint8_t mission_start_ready_latched = 0u;
    if(dT_s <= 0.0f || dT_s > 0.1f)
    {
        dT_s = REMOTE_SAMPLE_TIME;
    }

    /* Update health timers before accepting a new arm edge.  In flight this
     * can only request the controlled landing state; it never cuts motors. */
    sensor_safety_update(dT_s);

    // 1. 解析遥控器数据，获得干净的杆量输入 `manual_input`
    prase_remote_ctrl_data(dT_s);

    /* Camera code only publishes vision_detection_snapshot.  Horizontal
     * setpoints must be owned by this 20 ms control cycle: calling
     * vision_nav_update() from the asynchronous frame consumer let a new
     * frame overwrite a target between two LOC updates, producing one-frame
     * waypoint jumps during pre-mission Y-car alignment. */
    vision_nav_update();

    // 安全保护：未解锁时，所有期望值贴住当前状态，避免解锁瞬间跳变。
    if (vehicle_state.armed == 0)
    {
        mission_cruise_stable_time_s = 0.0f;
        mission_height_recovery_stable_time_s = 0.0f;
        mission_cruise_ready = 0u;
        mission_mode_active_last = 0u;
        mission_start_ready_latched = 0u;
        mission_task_requested = 0u;
        mission_height_recovery_active = 0u;
        auto_landing_active = AUTO_LAND_STATE_INACTIVE;
        auto_land_settle_time_s = 0.0f;
        auto_land_wait_time_s = 0.0f;
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
        uint8_t auto_land_entering =
            (auto_landing_active == AUTO_LAND_STATE_INACTIVE) ? 1u : 0u;
        float pos_err_x_cm;
        float pos_err_y_cm;
        float pos_err_cm;
        float horiz_speed_cm_s;
        uint8_t settle_inputs_available;

        if(auto_land_entering != 0u)
        {
            auto_landing_active = AUTO_LAND_STATE_SETTLING;
            auto_land_settle_time_s = 0.0f;
            auto_land_wait_time_s = 0.0f;
            auto_land_target_x_cm = vehicle_state.current_pos_x;
            auto_land_target_y_cm = vehicle_state.current_pos_y;
            auto_land_hold_height_cm = LIMIT(vehicle_state.current_height,
                                             MIN_HEIGHT,
                                             MAX_HEIGHT);
        }

        vehicle_state.flight_mode = FLY_AUTOLANDING;

        /* Keep one fixed landing point through both braking and descent.
         * Following current_pos every cycle would silently cancel hold. */
        vehicle_setpoint.target_pos_x = auto_land_target_x_cm;
        vehicle_setpoint.target_pos_y = auto_land_target_y_cm;
        vehicle_setpoint.target_vel_x = 0.0f;
        vehicle_setpoint.target_vel_y = 0.0f;
        vehicle_setpoint.target_roll = 0.0f;
        vehicle_setpoint.target_pitch = 0.0f;
        vehicle_setpoint.target_yaw_rate = 0.0f;

        /* Only the altitude state machine may disarm after an auto-land.  It
         * requires near-ground height, low vertical speed, low throttle and
         * a debounce interval.  A raw height sample must never bypass those
         * guards, especially on the same cycle as a radio-loss request. */
        if(alt_phase == ALT_PHASE_LANDED)
        {
            vehicle_state.armed = 0u;
            auto_landing_request = 0u;
            auto_landing_active = AUTO_LAND_STATE_INACTIVE;
            auto_land_settle_time_s = 0.0f;
            auto_land_wait_time_s = 0.0f;
            return;
        }

        if(auto_landing_active == AUTO_LAND_STATE_SETTLING)
        {
            pos_err_x_cm = auto_land_target_x_cm - vehicle_state.current_pos_x;
            pos_err_y_cm = auto_land_target_y_cm - vehicle_state.current_pos_y;
            pos_err_cm = sqrtf(pos_err_x_cm * pos_err_x_cm +
                               pos_err_y_cm * pos_err_y_cm);
            horiz_speed_cm_s =
                sqrtf(vehicle_state.current_vel_x * vehicle_state.current_vel_x +
                      vehicle_state.current_vel_y * vehicle_state.current_vel_y);
            auto_land_wait_time_s += dT_s;

            settle_inputs_available =
                ((flight_sensor_failsafe_flags == 0u) &&
                 (vehicle_state.flow_valid != 0u) &&
                 (vehicle_state.current_height > AUTO_LAND_TRIGGER_HEIGHT_CM)) ?
                1u : 0u;

            if((settle_inputs_available != 0u) &&
               (loc_1l_ct.loc_hold_ready != 0u) &&
               (horiz_speed_cm_s <= AUTO_LAND_SETTLE_HORIZ_SPEED_CM_S) &&
               (pos_err_cm <= AUTO_LAND_SETTLE_POS_ERR_CM) &&
               (fabsf(vehicle_state.current_vel_z) <= AUTO_LAND_SETTLE_VZ_CM_S))
            {
                auto_land_settle_time_s += dT_s;
            }
            else
            {
                auto_land_settle_time_s = 0.0f;
            }

            /* Fault landings and low-altitude requests must not hover while
             * waiting for an estimator that cannot become ready. */
            if((settle_inputs_available == 0u) ||
               (auto_land_settle_time_s >= AUTO_LAND_SETTLE_STABLE_TIME_S) ||
               (auto_land_wait_time_s >= AUTO_LAND_SETTLE_TIMEOUT_S))
            {
                auto_landing_active = AUTO_LAND_STATE_DESCENDING;
            }
        }

        if(auto_landing_active == AUTO_LAND_STATE_SETTLING)
        {
            vehicle_setpoint.target_height = auto_land_hold_height_cm;
            vehicle_setpoint.target_vel_z = 0.0f;
        }
        else
        {
            vehicle_setpoint.target_height = AUTO_LAND_TARGET_HEIGHT_CM;
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
#elif VISION_MISSION_MODE_ENABLE || BEACON_APPROACH_TEST_ENABLE
    /*
     * Competition mission test:
     * - climb to the validated vision height;
     * - hold the takeoff position until horizontal control is ready;
     * - then leave horizontal waypoint ownership to vision_nav.
     *
     * vision_nav owns the horizontal waypoint after LOC becomes ready.  The
     * selected compile-time vision mode decides whether that means field
     * search or the isolated beacon-approach test.
     */
    /*
     * Once the mission has started, a cable-loaded aircraft may not be able
     * to climb if horizontal motion is stopped at a fixed point.  Keep
     * AUTOFLY and the current mission target, but apply a dedicated horizontal
     * vector-speed cap until height recovers.
     */
    if((mission_mode_active_last != 0u) &&
       (vehicle_state.current_height <
        MISSION_CRUISE_DROP_PAUSE_HEIGHT_CM))
    {
        mission_height_recovery_active = 1u;
        mission_height_recovery_stable_time_s = 0.0f;
    }

    if(vision_nav_obs.horizontal_fault_active != 0u)
    {
        mission_cruise_ready = 0u;
        mission_cruise_stable_time_s = 0.0f;
    }

    if(mission_height_recovery_active != 0u)
    {
        if((vehicle_state.current_height >=
            (LOC_TEST_TARGET_HEIGHT_CM - MISSION_CRUISE_HEIGHT_ERR_CM)) &&
           (fabsf(vehicle_state.current_vel_z) <=
            MISSION_CRUISE_VZ_MAX_CM_S) &&
           (loc_1l_ct.loc_hold_ready != 0u))
        {
            mission_height_recovery_stable_time_s += dT_s;
            if(mission_height_recovery_stable_time_s >=
               MISSION_CRUISE_STABLE_TIME_S)
            {
                mission_height_recovery_active = 0u;
                mission_height_recovery_stable_time_s = 0.0f;
            }
        }
        else
        {
            mission_height_recovery_stable_time_s = 0.0f;
        }
    }

    if((vehicle_state.current_height >=
        (LOC_TEST_TARGET_HEIGHT_CM - MISSION_CRUISE_HEIGHT_ERR_CM)) &&
       (fabsf(vehicle_state.current_vel_z) <= MISSION_CRUISE_VZ_MAX_CM_S) &&
       (loc_1l_ct.loc_hold_ready != 0u) &&
       (vision_nav_obs.horizontal_fault_active == 0u))
    {
        mission_cruise_stable_time_s += dT_s;
        if(mission_cruise_stable_time_s >= MISSION_CRUISE_STABLE_TIME_S)
        {
            mission_cruise_ready = 1u;
        }
    }
    else if(mission_cruise_ready == 0u)
    {
        mission_cruise_stable_time_s = 0.0f;
    }

    {
        uint8_t mission_mode_active;

        /* Start directly once cruise is stable and the Y car is visible.
         * The external competition switch is intentionally not a task gate.
         * Once entered, a short car-vision dropout is handled by navigation
         * hold/STOP logic rather than returning to AUTOTAKEOFF. */
        if((mission_cruise_ready != 0u) &&
           (vision_nav_yaw_spin_is_done() != 0u) &&
           (vision_nav_obs.horizontal_fault_active == 0u) &&
           (ycar_info.valid != 0u))
        {
            mission_task_requested = 1u;
            mission_start_ready_latched = 1u;
        }

        mission_mode_active = (mission_start_ready_latched != 0u) ? 1u : 0u;

        /* Capture a neutral hold point on the sole entry edge so neither an
         * old takeoff goal nor a stale mission goal is carried into AUTOFLY. */
        if(mission_mode_active != mission_mode_active_last)
        {
            vehicle_setpoint.target_pos_x = vehicle_state.current_pos_x;
            vehicle_setpoint.target_pos_y = vehicle_state.current_pos_y;
            vehicle_setpoint.target_vel_x = 0.0f;
            vehicle_setpoint.target_vel_y = 0.0f;
            mission_height_recovery_active = 0u;
            mission_height_recovery_stable_time_s = 0.0f;
            if(mission_mode_active != 0u)
            {
                att_ctrl_set_yaw_target(VISION_NAV_MISSION_YAW_DEG);
            }
        }

        vehicle_state.flight_mode = (mission_mode_active != 0u) ?
            FLY_AUTOFLY : FLY_AUTOTAKEOFF;
        mission_mode_active_last = mission_mode_active;
    }
    vehicle_setpoint.target_height =
        (vision_nav_yaw_spin_is_done() != 0u) ?
        LOC_TEST_TARGET_HEIGHT_CM : VISION_NAV_YAW_SPIN_HEIGHT_CM;
    if(vision_nav_yaw_spin_is_active() == 0u)
    {
        vehicle_setpoint.target_yaw_rate = 0.0f;
    }

    if((vehicle_state.flow_valid == 0u) ||
       (vehicle_state.current_height < LOC_ENABLE_HEIGHT_CM) ||
       (loc_1l_ct.loc_hold_ready == 0u))
    {
        /*
         * FLY_AUTOTAKEOFF captures its horizontal target on mode entry, and
         * FLY_AUTOLANDING captures it when landing is requested.  Preserve
         * that point while low-altitude damping/hold gains blend in; following
         * the drifting EKF position here cancels the very error LOC needs.
         */
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
    {
        float yaw_rate_limit =
            (vision_nav_yaw_spin_is_active() != 0u) ?
            VISION_NAV_YAW_SPIN_MAX_RATE_DPS : MAX_YAW_RATE;
        vehicle_setpoint.target_yaw_rate =
            LIMIT(vehicle_setpoint.target_yaw_rate,
                  -yaw_rate_limit,
                   yaw_rate_limit);
    }
    vehicle_setpoint.target_vel_x = LIMIT(vehicle_setpoint.target_vel_x, -MAX_HORIZONTAL_SPEED, MAX_HORIZONTAL_SPEED);
    vehicle_setpoint.target_vel_y = LIMIT(vehicle_setpoint.target_vel_y, -MAX_HORIZONTAL_SPEED, MAX_HORIZONTAL_SPEED);  
    
}
