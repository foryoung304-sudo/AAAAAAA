#include "zf_common_headfile.h"

flow_health_t flow_health = {0};



static float flow_lite_absf(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float flow_gyro_int_x = 0.0f;
static float flow_gyro_int_y = 0.0f;
static float flow_gyro_int_z = 0.0f;

typedef struct
{
    float tilt_gate;
    float yaw_gate;

    float flow_dx_before_gate;
    float flow_dy_before_gate;
    float gyro_x_before_gate;
    float gyro_y_before_gate;
    float offset_disp_x;
    float offset_disp_y;
    float yaw_vision_disp_x;
    float yaw_vision_disp_y;
    float flow_dx_raw;
    float flow_dy_raw;
    float gyro_disp_x;
    float gyro_disp_y;
    float d_theta_x;
    float d_theta_y;
    float d_theta_z;

    float lc302_delta_x;
    float lc302_delta_y;
    uint8_t flow_valid;
    uint8_t flow_quality;
    float height_actual;

    float true_pos_x;
    float true_pos_y;
    float true_dx_raw;
    float true_dy_raw;

    float flow_dx_earth;
    float flow_dy_earth;
    float flow_pos_x;
    float flow_pos_y;
    float flow_vel_x;
    float flow_vel_y;
} flow_debug_t;

static flow_debug_t flow_debug = {0};



void flow_gyro_integrate(float dT_s)
{
    if(dT_s <= 0.0f || dT_s > 0.01f)
    {
        return;
    }

    flow_gyro_int_x += imu_data.gyro_actual[0] * (3.1415926f / 180.0f) * dT_s;
    flow_gyro_int_y += imu_data.gyro_actual[1] * (3.1415926f / 180.0f) * dT_s;
    flow_gyro_int_z += imu_data.gyro_actual[2] * (3.1415926f / 180.0f) * dT_s;
}

// 更新光流位置和速度
void update_position_from_flow(float dT_s)
{
    static float flow_pos_x=0.0f,flow_pos_y=0.0f;
    static float flow_vel_x=0.0f,flow_vel_y=0.0f;
    static float flow_speed=0.0f;
    static float last_delta_x=0.0f, last_delta_y=0.0f;
    static uint8_t flow_lost_count = 0u;
    static uint8_t flow_recover_count = 0u;


    memset(&flow_health, 0, sizeof(flow_health));

    uint8_t flow_state_fault = 0u;
    if(flow_speed > 500.0f)
    {
        // flow_pos_* is a legitimate accumulated displacement and must not
        // be used as a sensor-health limit.  A true velocity-state fault is
        // recoverable: clear every derived state so one stale value cannot
        // keep flow_valid latched low forever.
        flow_state_fault = 1u;
        flow_pos_x = 0.0f;
        flow_pos_y = 0.0f;
        flow_vel_x = 0.0f;
        flow_vel_y = 0.0f;
        flow_speed = 0.0f;
        last_delta_x = 0.0f;
        last_delta_y = 0.0f;
        flow_gyro_int_x = 0.0f;
        flow_gyro_int_y = 0.0f;
        flow_gyro_int_z = 0.0f;
    }

#if (FLOW_SENSOR_TYPE == FLOW_SENSOR_LC302)
  
    // 对像素位移进行简单低通滤波，并更新历史值（改用浮点数，防止亚像素精度丢失）
    float lc302_delta_x = 0.0f;
    float lc302_delta_y = 0.0f;
    uint8_t flow_valid = 0;
    uint8_t flow_quality = 0;
    uint16_t lc302_accum_count = 0u;
    uint32_t lc302_integration_us = 0u;
    uint32_t lc302_frame_count = 0u;
    
    lc302_get_motion(&lc302_delta_x, &lc302_delta_y, &flow_valid, &flow_quality,
                     &lc302_accum_count, &lc302_integration_us,
                     &lc302_frame_count);
    flow_health.raw_valid = (flow_valid != 0u) ? 1u : 0u;
    flow_health.quality = flow_quality;
    flow_health.lc302_accum_count = lc302_accum_count;
    flow_health.lc302_integration_us = lc302_integration_us;
    flow_health.lc302_frame_count = lc302_frame_count;

    // The LC302 runs independently from the 50 Hz control loop.  A zero
    // accumulation count means that no new sensor frame has arrived since
    // the previous read; it is not a bad optical-flow frame.  Keep the
    // 2 ms EKF prediction running, but skip this 50 Hz measurement update.
    // In particular, do not clear last_delta_* or the gyro integration
    // window: both must span until the next real LC302 frame.
    if(lc302_accum_count == 0u)
    {
        flow_health.quality_valid =
            (flow_quality >= LC302_QUALITY_MIN) ? 1u : 0u;

        if(flow_state_fault != 0u)
        {
            vehicle_state.flow_valid = 0u;
            flow_lost_count = FLOW_VALID_LOST_FRAMES;
            flow_recover_count = 0u;
        }

        return;
    }

#if LC302_SWAP_XY
    float lc302_swap_temp = lc302_delta_x;
    lc302_delta_x = lc302_delta_y;
    lc302_delta_y = lc302_swap_temp;
#endif

    uint8_t flow_frame_valid = ((flow_valid != 0u) && (flow_quality >= LC302_QUALITY_MIN)) ? 1u : 0u;
    flow_health.quality_valid = (flow_quality >= LC302_QUALITY_MIN) ? 1u : 0u;

    if(flow_frame_valid == 0u) // quality 0-255，低于阈值基本等于无效
    {
        lc302_delta_x = 0.0f;
        lc302_delta_y = 0.0f;
        last_delta_x = 0.0f;
        last_delta_y = 0.0f;
    }

    float delta_x = lc302_delta_x * 0.85f + last_delta_x * 0.15f;
    float delta_y = lc302_delta_y * 0.85f + last_delta_y * 0.15f;
#else
    pmw3901_get_motion();
    uint8_t flow_valid = 1;
    uint8_t flow_frame_valid = 1u;
    float delta_x = (float)pmw3901_delta_x * 0.85f + last_delta_x * 0.15f;
    float delta_y = (float)pmw3901_delta_y * 0.85f + last_delta_y * 0.15f;
    flow_health.raw_valid = 1u;
    flow_health.quality_valid = 1u;
    flow_health.quality = 255u;
#endif

    if(flow_state_fault != 0u)
    {
        vehicle_state.flow_valid = 0u;
        flow_lost_count = FLOW_VALID_LOST_FRAMES;
        flow_recover_count = 0u;
    }
    else if(flow_frame_valid != 0u)
    {
        flow_lost_count = 0u;
        if(flow_recover_count < FLOW_VALID_RECOVER_FRAMES)
        {
            flow_recover_count++;
        }
        if(flow_recover_count >= FLOW_VALID_RECOVER_FRAMES)
        {
            vehicle_state.flow_valid = 1u;
        }
    }
    else
    {
        flow_recover_count = 0u;
        if(flow_lost_count < FLOW_VALID_LOST_FRAMES)
        {
            flow_lost_count++;
        }
        if(flow_lost_count >= FLOW_VALID_LOST_FRAMES)
        {
            vehicle_state.flow_valid = 0u;
        }
    }

    last_delta_x = delta_x;//前-后+
    last_delta_y = delta_y;//左-右+
    
    float height_actual = vehicle_state.current_height;
    flow_health.height_cm = height_actual;
    flow_health.height_valid = ((vehicle_state.current_height >= 5.0f) && (vehicle_state.current_height <= 155.0f)) ? 1u : 0u;
    flow_health.tilt_valid = 1u;
    flow_health.yaw_valid = 1u;
    // 光流原始位移（转为cm）
#if (FLOW_SENSOR_TYPE == FLOW_SENSOR_LC302)
    float flow_dx_raw = delta_x * height_actual*LC302_FLOW_SCALE_X;
    float flow_dy_raw = -delta_y * height_actual*LC302_FLOW_SCALE_Y;

    if(LC302_INSTALL_YAW_DEG != 0.0f)
    {
        float install_yaw = LC302_INSTALL_YAW_DEG * 3.1415926f / 180.0f;
        float install_c = cosf(install_yaw);
        float install_s = sinf(install_yaw);
        float flow_dx_corr = flow_dx_raw * install_c - flow_dy_raw * install_s;
        float flow_dy_corr = flow_dx_raw * install_s + flow_dy_raw * install_c;
        flow_dx_raw = flow_dx_corr;
        flow_dy_raw = flow_dy_corr;
    }
#else
    float flow_scale = height_actual * 0.00213f;
    float flow_dx_raw = -delta_x * flow_scale;
    float flow_dy_raw = delta_y * flow_scale;
#endif
    flow_health.raw_dx_cm = flow_dx_raw;
    flow_health.raw_dy_cm = flow_dy_raw;
    
    // ====================== 光流双重解耦补偿 ======================
    //
    float d_theta_x = flow_gyro_int_x; // Roll 旋转
    float d_theta_y = flow_gyro_int_y; // Pitch 旋转
    float d_theta_z = flow_gyro_int_z; // Yaw 旋转
#if FLOW_RP_USE_ATTITUDE_DELTA
    static uint8_t rp_delta_inited = 0u;
    static float last_roll_deg = 0.0f;
    static float last_pitch_deg = 0.0f;
    float roll_delta_deg = 0.0f;
    float pitch_delta_deg = 0.0f;

    if(rp_delta_inited)
    {
        roll_delta_deg = imu_data.roll - last_roll_deg;
        pitch_delta_deg = imu_data.pitch - last_pitch_deg;
        d_theta_x = roll_delta_deg * (3.1415926f / 180.0f);
        d_theta_y = pitch_delta_deg * (3.1415926f / 180.0f);
    }
    else
    {
        rp_delta_inited = 1u;
        d_theta_x = 0.0f;
        d_theta_y = 0.0f;
    }
    last_roll_deg = imu_data.roll;
    last_pitch_deg = imu_data.pitch;
#endif
#if FLOW_YAW_USE_ATTITUDE_DELTA
    static uint8_t yaw_delta_inited = 0u;
    static float last_yaw_deg = 0.0f;
    float yaw_delta_deg = 0.0f;

    if(yaw_delta_inited)
    {
        yaw_delta_deg = imu_data.yaw - last_yaw_deg;
        while(yaw_delta_deg > 180.0f)
        {
            yaw_delta_deg -= 360.0f;
        }
        while(yaw_delta_deg < -180.0f)
        {
            yaw_delta_deg += 360.0f;
        }
        d_theta_z = yaw_delta_deg * (3.1415926f / 180.0f);
    }
    else
    {
        yaw_delta_inited = 1u;
        d_theta_z = 0.0f;
    }
    last_yaw_deg = imu_data.yaw;
#endif
    flow_gyro_int_x = 0.0f;
    flow_gyro_int_y = 0.0f;
    flow_gyro_int_z = 0.0f;

    // 2. 偏心杠杆臂物理补偿 (光流探头距离重心的坐标，单位: cm)
    // Body X 前正，Body Y 左正。只启用 yaw 偏心补偿，roll/pitch 的 Z 偏心暂时保持关闭。
#if FLOW_YAW_OFFSET_ENABLE
    float offset_disp_x =  d_theta_z * FLOW_OFFSET_Y_CM;
    float offset_disp_y = -d_theta_z * FLOW_OFFSET_X_CM;
#else
    float offset_disp_x = 0.0f;
    float offset_disp_y = 0.0f;
#endif
    // LC302 yaw 视场旋转经验补偿，单位 cm。K 需要通过原地单方向 yaw 实测拟合。
#if FLOW_YAW_VISION_ENABLE
    float yaw_vision_disp_x = FLOW_YAW_VISION_GAIN_X * height_actual * d_theta_z;
    float yaw_vision_disp_y = FLOW_YAW_VISION_GAIN_Y * height_actual * d_theta_z;
#else
    float yaw_vision_disp_x = 0.0f;
    float yaw_vision_disp_y = 0.0f;
#endif
    // 3. 陀螺仪视觉补偿 
    float gyro_disp_x =  d_theta_y * height_actual * FLOW_GYRO_COMP_GAIN_X; // Pitch 旋转引起的X轴假位移
    float gyro_disp_y = -d_theta_x * height_actual * FLOW_GYRO_COMP_GAIN_Y; // Roll 旋转引起的Y轴假位移

    float flow_dx_before_gate = flow_dx_raw;
    float flow_dy_before_gate = flow_dy_raw;
    float gyro_x_before_gate = gyro_disp_x;
    float gyro_y_before_gate = gyro_disp_y;

#if FLOW_CALIB_RAW_ONLY
    gyro_disp_x = 0.0f;
    gyro_disp_y = 0.0f;
    offset_disp_x = 0.0f;
    offset_disp_y = 0.0f;
    yaw_vision_disp_x = 0.0f;
    yaw_vision_disp_y = 0.0f;

#if FLOW_CALIB_USE_RAW_DEADZONE
    if(fabsf(flow_dx_raw) < FLOW_RAW_DEADZONE_CM)
    {
        flow_dx_raw = 0.0f;
    }
    if(fabsf(flow_dy_raw) < FLOW_RAW_DEADZONE_CM)
    {
        flow_dy_raw = 0.0f;
    }
#endif

    float true_dx_raw = flow_dx_raw;
    float true_dy_raw = flow_dy_raw;
    float tilt_gate = 1.0f;
    float yaw_gate = 1.0f;

#if FLOW_CALIB_USE_GATE
    float tilt_abs = MAX(fabsf(imu_data.roll), fabsf(imu_data.pitch));
    flow_health.tilt_valid = (tilt_abs <= 30.0f) ? 1u : 0u;

    if(tilt_abs > 30.0f)
    {
        tilt_gate = 0.0f;
    }
    else if(tilt_abs > 20.0f)
    {
        tilt_gate = (30.0f - tilt_abs) / 10.0f * 0.5f;
    }
    else if(tilt_abs > 15.0f)
    {
        tilt_gate = 1.0f - (tilt_abs - 15.0f) / 5.0f * 0.3f;
    }

    float yaw_rate_abs = fabsf(imu_data.gyro_actual[2]);
    yaw_gate = (yaw_rate_abs > FLOW_YAW_REJECT_DPS) ? 0.0f : 1.0f;

    if(yaw_gate <= 0.0f)
    {
        true_dx_raw = 0.0f;
        true_dy_raw = 0.0f;
    }

    true_dx_raw *= tilt_gate;
    true_dy_raw *= tilt_gate;
#endif
#else
    // raw 很小时，说明本帧没有观测到足够图像运动。
    // 此时禁止 gyro 补偿项单独生成位移，避免静止/小抖动时积分漂移。
    if(fabsf(flow_dx_raw) < FLOW_RAW_DEADZONE_CM)
    {
        flow_dx_raw = 0.0f;
        gyro_disp_x = 0.0f;
    }
    if(fabsf(flow_dy_raw) < FLOW_RAW_DEADZONE_CM)
    {
        flow_dy_raw = 0.0f;
        gyro_disp_y = 0.0f;
    }
    if((flow_dx_raw == 0.0f) && (flow_dy_raw == 0.0f))
    {
        offset_disp_x = 0.0f;
        offset_disp_y = 0.0f;
        yaw_vision_disp_x = 0.0f;
        yaw_vision_disp_y = 0.0f;
    }

    // 4. 剔除所有假位移，获得重心的真实物理平移速度
    float true_dx_raw = flow_dx_raw - gyro_disp_x - offset_disp_x - yaw_vision_disp_x;
    float true_dy_raw = flow_dy_raw - gyro_disp_y - offset_disp_y - yaw_vision_disp_y;

    float tilt_abs = MAX(fabsf(imu_data.roll), fabsf(imu_data.pitch));

        float tilt_gate = 1.0f;

        if(tilt_abs > 30.0f)
        {
            tilt_gate = 0.0f;
        }
        else if(tilt_abs > 20.0f)
        {
            tilt_gate = (30.0f - tilt_abs) / 10.0f * 0.5f;  // 20°时0.5，30°时0
        }
        else if(tilt_abs > 15.0f)
        {
            tilt_gate = 1.0f - (tilt_abs - 15.0f) / 5.0f * 0.3f;  // 15°时1，20°时0.7
        }

    // yaw 门控
    float yaw_rate_abs = fabsf(imu_data.gyro_actual[2]);
    float yaw_gate = (yaw_rate_abs > FLOW_YAW_REJECT_DPS) ? 0.0f : 1.0f;
    flow_health.yaw_valid = (yaw_gate > 0.0f) ? 1u : 0u;

    if(yaw_gate <= 0.0f)
    {
        true_dx_raw = 0.0f;
        true_dy_raw = 0.0f;
    }

    true_dx_raw *= tilt_gate;
    true_dy_raw *= tilt_gate;
#endif

    // 50Hz 光流更新时，10cm/帧约等于 500cm/s。
    if (fabsf(true_dx_raw) > 10.0f || fabsf(true_dy_raw) > 10.0f) 
    {
        true_dx_raw = 0.0f;
        true_dy_raw = 0.0f;
    }

    if(fabsf(true_dy_raw) < FLOW_GYRO_DEADZONE_CM)
    {
        true_dy_raw = 0.0f;
    }  
    if(fabsf(true_dx_raw) < FLOW_GYRO_DEADZONE_CM)
    {
        true_dx_raw = 0.0f;
    }

#if (FLOW_MOTION_CONFIRM_FRAMES > 1)
    static uint8_t motion_confirm_x = 0u;
    static uint8_t motion_confirm_y = 0u;
    static int8_t last_motion_sign_x = 0;
    static int8_t last_motion_sign_y = 0;
    int8_t motion_sign_x = (true_dx_raw > 0.0f) ? 1 : ((true_dx_raw < 0.0f) ? -1 : 0);
    int8_t motion_sign_y = (true_dy_raw > 0.0f) ? 1 : ((true_dy_raw < 0.0f) ? -1 : 0);

    if(motion_sign_x == 0)
    {
        motion_confirm_x = 0u;
        last_motion_sign_x = 0;
    }
    else
    {
        motion_confirm_x = (motion_sign_x == last_motion_sign_x) ? (uint8_t)(motion_confirm_x + 1u) : 1u;
        last_motion_sign_x = motion_sign_x;
        if(motion_confirm_x < FLOW_MOTION_CONFIRM_FRAMES)
        {
            true_dx_raw = 0.0f;
        }
    }

    if(motion_sign_y == 0)
    {
        motion_confirm_y = 0u;
        last_motion_sign_y = 0;
    }
    else
    {
        motion_confirm_y = (motion_sign_y == last_motion_sign_y) ? (uint8_t)(motion_confirm_y + 1u) : 1u;
        last_motion_sign_y = motion_sign_y;
        if(motion_confirm_y < FLOW_MOTION_CONFIRM_FRAMES)
        {
            true_dy_raw = 0.0f;
        }
    }
#endif

    flow_health.true_dx_body_cm = true_dx_raw;
    flow_health.true_dy_body_cm = true_dy_raw;
    // ==============================================================

    float flow_dx_earth = true_dx_raw * vehicle_state.yaw_cos - true_dy_raw * vehicle_state.yaw_sin;
    float flow_dy_earth = true_dx_raw * vehicle_state.yaw_sin + true_dy_raw * vehicle_state.yaw_cos;
    flow_health.earth_dx_cm = flow_dx_earth;
    flow_health.earth_dy_cm = flow_dy_earth;
    


    
    // 积分更新光流位
    
    flow_pos_x += flow_dx_earth;
    flow_pos_y += flow_dy_earth;
    
    // LC302 displacement belongs to its own integration window, not to the
    // 50 Hz task period.  Fall back to the documented nominal frame rate if
    // a packet reports a nonsensical integration span.
    float obs_dt_s = (float)lc302_integration_us * 0.000001f;
    float obs_dt_min = (float)lc302_accum_count * FLOW_LC302_MIN_FRAME_DT_S;
    float obs_dt_max = (float)lc302_accum_count * FLOW_LC302_MAX_FRAME_DT_S;
    if((obs_dt_s < obs_dt_min) || (obs_dt_s > obs_dt_max))
    {
        obs_dt_s = (float)lc302_accum_count * FLOW_LC302_NOMINAL_FRAME_DT_S;
    }
    flow_health.obs_dt_s = obs_dt_s;

    // 计算光流速度（cm/s）
    if(obs_dt_s > 0.001f)
    {
        float new_vel_x = flow_dx_earth / obs_dt_s;
        float new_vel_y = flow_dy_earth / obs_dt_s;
        flow_health.obs_vx_cm_s = new_vel_x;
        flow_health.obs_vy_cm_s = new_vel_y;
        flow_health.obs_valid = ((flow_health.raw_valid != 0u) &&
                                 (vehicle_state.flow_valid != 0u) &&
                                 (flow_health.height_valid != 0u) &&
                                 (flow_health.tilt_valid != 0u) &&
                                 (flow_health.yaw_valid != 0u)) ? 1u : 0u;
        
        // 低通滤波（滤除高频噪声）
        flow_vel_x = 0.7f * flow_vel_x + 0.3f * new_vel_x;
        flow_vel_y = 0.7f * flow_vel_y + 0.3f * new_vel_y;
    }
    
    flow_speed = sqrtf(flow_vel_x * flow_vel_x + 
                              flow_vel_y * flow_vel_y);
    flow_health.flow_pos_x_cm = flow_pos_x;
    flow_health.flow_pos_y_cm = flow_pos_y;
    flow_health.flow_vel_x_cm_s = flow_vel_x;
    flow_health.flow_vel_y_cm_s = flow_vel_y;
    
    
    static float raw_pos_y = 0.0f;
    static float true_pos_y = 0.0f;
    static float gyro_pos_y = 0.0f;
    static float offset_pos_y = 0.0f;
    static float true_pos_x = 0.0f;

    raw_pos_y    += flow_dy_raw;
    gyro_pos_y   += gyro_disp_y;
    offset_pos_y += offset_disp_y;
    true_pos_y   += true_dy_raw;
    true_pos_x   += true_dx_raw;

    flow_debug.tilt_gate = tilt_gate;
    flow_debug.yaw_gate = yaw_gate;
    flow_debug.flow_dx_before_gate = flow_dx_before_gate;
    flow_debug.flow_dy_before_gate = flow_dy_before_gate;
    flow_debug.gyro_x_before_gate = gyro_x_before_gate;
    flow_debug.gyro_y_before_gate = gyro_y_before_gate;
    flow_debug.offset_disp_x = offset_disp_x;
    flow_debug.offset_disp_y = offset_disp_y;
    flow_debug.yaw_vision_disp_x = yaw_vision_disp_x;
    flow_debug.yaw_vision_disp_y = yaw_vision_disp_y;
    flow_debug.flow_dx_raw = flow_dx_raw;
    flow_debug.flow_dy_raw = flow_dy_raw;
    flow_debug.gyro_disp_x = gyro_disp_x;
    flow_debug.gyro_disp_y = gyro_disp_y;
    flow_debug.d_theta_x = d_theta_x;
    flow_debug.d_theta_y = d_theta_y;
    flow_debug.d_theta_z = d_theta_z;
#if (FLOW_SENSOR_TYPE == FLOW_SENSOR_LC302)
    flow_debug.lc302_delta_x = lc302_delta_x;
    flow_debug.lc302_delta_y = lc302_delta_y;
    flow_debug.flow_quality = flow_quality;
#else
    flow_debug.lc302_delta_x = delta_x;
    flow_debug.lc302_delta_y = delta_y;
    flow_debug.flow_quality = 255u;
#endif
    flow_debug.flow_valid = flow_valid;
    flow_debug.height_actual = height_actual;
    flow_debug.true_pos_x = true_pos_x;
    flow_debug.true_pos_y = true_pos_y;
    flow_debug.true_dx_raw = true_dx_raw;
    flow_debug.true_dy_raw = true_dy_raw;
    flow_debug.flow_dx_earth = flow_dx_earth;
    flow_debug.flow_dy_earth = flow_dy_earth;
    flow_debug.flow_pos_x = flow_pos_x;
    flow_debug.flow_pos_y = flow_pos_y;
    flow_debug.flow_vel_x = flow_vel_x;
    flow_debug.flow_vel_y = flow_vel_y;

   /* vehicle_state.current_pos_x=flow_pos_x;
    vehicle_state.current_pos_y=flow_pos_y;
    vehicle_state.current_vel_x=flow_vel_x;
    vehicle_state.current_vel_y=flow_vel_y;*/

}

void flow_debug_print(void)
{
#if FLOW_DEBUG_ENABLE
    static uint32_t debug_count = 0;

    if((debug_count++ % FLOW_DEBUG_DIV) != 0u)
    {
        return;
    }

#if FLOW_DEBUG_GATE
    printf("tilt_gate=%.2f,yaw_gate=%.2f\n", flow_debug.tilt_gate, flow_debug.yaw_gate);
#endif
#if FLOW_DEBUG_ATTITUDE
    printf("roll_cmp:dy_raw=%f,gyro_y=%f,true_y=%f,offset_y=%f,dy_gate=%f,gyro_y_gate=%f,roll=%f,gx=%f,dtheta_x=%f\n",
            flow_debug.flow_dy_before_gate,
            flow_debug.gyro_y_before_gate,
            flow_debug.flow_dy_before_gate - flow_debug.gyro_y_before_gate - flow_debug.offset_disp_y,
            flow_debug.offset_disp_y,
            flow_debug.flow_dy_raw,
            flow_debug.gyro_disp_y,
            imu_data.roll,
            imu_data.gyro_actual[0],
            flow_debug.d_theta_x);
    printf("pitch_cmp:dx_raw=%f,gyro_x=%f,true_x=%f,offset_x=%f,dx_gate=%f,gyro_x_gate=%f,pitch=%f,gy=%f,dtheta_y=%f\n",
            flow_debug.flow_dx_before_gate,
            flow_debug.gyro_x_before_gate,
            flow_debug.flow_dx_before_gate - flow_debug.gyro_x_before_gate - flow_debug.offset_disp_x,
            flow_debug.offset_disp_x,
            flow_debug.flow_dx_raw,
            flow_debug.gyro_disp_x,
            imu_data.pitch,
            imu_data.gyro_actual[1],
            flow_debug.d_theta_y);
    printf("yaw_cmp: dx_raw=%f, dy_raw=%f, off_x=%f, off_y=%f, yaw_vis_x=%f, yaw_vis_y=%f, dtheta_z=%f, yaw=%f\n\n",
            flow_debug.flow_dx_before_gate,
            flow_debug.flow_dy_before_gate,
            flow_debug.offset_disp_x,
            flow_debug.offset_disp_y,
            flow_debug.yaw_vision_disp_x,
            flow_debug.yaw_vision_disp_y,
            flow_debug.d_theta_z,
            imu_data.yaw);
    printf("current_height=%.2f", vehicle_state.current_height);
#endif
#if (FLOW_SENSOR_TYPE == FLOW_SENSOR_LC302) && FLOW_DEBUG_LC302
    printf("[LC302] dx=%f,dy=%f,valid=%d,quality=%d,scale_x=%f,scale_y=%f,h=%f,flow_valid=%d\n",
            flow_debug.lc302_delta_x,
            flow_debug.lc302_delta_y,
            flow_debug.flow_valid,
            flow_debug.flow_quality,
            LC302_FLOW_SCALE_X,
            LC302_FLOW_SCALE_Y,
            flow_debug.height_actual,
            vehicle_state.flow_valid);
#endif

#if FLOW_DEBUG_BODY
    printf("body_state:true_pos_x=%f,true_pos_y=%f,true_dx=%f,true_dy=%f,height_actual=%f\n",
            flow_debug.true_pos_x,
            flow_debug.true_pos_y,
            flow_debug.true_dx_raw,
            flow_debug.true_dy_raw,
            vehicle_state.current_height);
#endif
#if FLOW_DEBUG_EARTH
    printf("earth_state:earth_dx=%f,earth_dy=%f,pos_x=%f,pos_y=%f,vel_x=%f,vel_y=%f\n\n",
            flow_debug.flow_dx_earth,
            flow_debug.flow_dy_earth,
            flow_debug.flow_pos_x,
            flow_debug.flow_pos_y,
            flow_debug.flow_vel_x,
            flow_debug.flow_vel_y);
#endif
#endif
}

// EKF 预测步：XY水平位置与速度 (简单的匀速运动模型)
void ekf_lite_predict_xy(float dT_s)
{
    if(dT_s <= 0.0f || dT_s > FLOW_LITE_MAX_DT_S)
    {
        dT_s = 0.02f;
    }

    // 位置 = 速度 * dt
    ekf_lite_state.x += ekf_lite_state.vx * dT_s;
    ekf_lite_state.y += ekf_lite_state.vy * dT_s;

    // 更新位置和速度的协方差，并加入过程噪声 Q
    ekf_lite_p_xy[0] += ekf_lite_p_xy[2] * dT_s * dT_s + 5.0f;
    ekf_lite_p_xy[1] += ekf_lite_p_xy[3] * dT_s * dT_s + 5.0f;
    ekf_lite_p_xy[2] += FLOW_LITE_Q_VEL * dT_s;
    ekf_lite_p_xy[3] += FLOW_LITE_Q_VEL * dT_s;

    ekf_lite_state.x = LIMIT(ekf_lite_state.x, -FLOW_LITE_POS_LIMIT_CM, FLOW_LITE_POS_LIMIT_CM);
    ekf_lite_state.y = LIMIT(ekf_lite_state.y, -FLOW_LITE_POS_LIMIT_CM, FLOW_LITE_POS_LIMIT_CM);

    vehicle_state.current_pos_x = ekf_lite_state.x;
    vehicle_state.current_pos_y = ekf_lite_state.y;
    vehicle_state.current_vel_x = ekf_lite_state.vx;
    vehicle_state.current_vel_y = ekf_lite_state.vy;
}

// EKF 更新步：XY水平速度 (利用光流传感器数据校正)
void ekf_lite_update_xy(float dT_s)
{
    const float r_flow = FLOW_LITE_R_VEL; // 光流测量的噪声协方差 R
    float innov_x = flow_health.obs_vx_cm_s - ekf_lite_state.vx; // X轴新息
    float innov_y = flow_health.obs_vy_cm_s - ekf_lite_state.vy; // Y轴新息
    float kx;
    float ky;
    float obs_dt_s = flow_health.obs_dt_s;
    float gate_limit;
    (void)dT_s;

    if((obs_dt_s <= 0.0f) || (obs_dt_s > FLOW_LITE_MAX_DT_S))
    {
        obs_dt_s = 0.02f;
    }
    gate_limit = FLOW_LITE_VEL_GATE_BASE_CMS +
                 FLOW_LITE_MAX_ACCEL_CMS2 * obs_dt_s;
    gate_limit = LIMIT(gate_limit, FLOW_LITE_VEL_GATE_BASE_CMS,
                       FLOW_LITE_VEL_GATE_MAX_CMS);
    flow_health.gate_limit_cm_s = gate_limit;

    ekf_lite_health.flow_innov_x = innov_x;
    ekf_lite_health.flow_innov_y = innov_y;
    ekf_lite_health.flow_used = 0u;
    flow_health.ekf_used = 0u;
    flow_health.gate_clipped = 0u;
    flow_health.innov_vx_cm_s = innov_x;
    flow_health.innov_vy_cm_s = innov_y;

    // 检查光流数据是否有效
    if(flow_health.obs_valid == 0u)
    {
        return;
    }

    // 如果飞行器倾角过大(>25度)，光流容易失真或丢失，拒绝更新
    if(flow_lite_absf(imu_data.roll) > 25.0f || flow_lite_absf(imu_data.pitch) > 25.0f)
    {
        return;
    }

    // 剧烈偏航(Yaw)旋转时也会导致光流估算偏差大，拒绝更新
    if(flow_lite_absf(imu_data.gyro_actual[2]) > 120.0f)
    {
        return;
    }

    // Physical innovation gate: independently limit the innovation
    // to prevent a single bad observation from injecting a massive
    // acceleration, while still allowing the filter to catch up robustly.
    if(flow_lite_absf(innov_x) > gate_limit ||
       flow_lite_absf(innov_y) > gate_limit)
    {
        flow_health.gate_clipped = 1u;
    }

    innov_x = LIMIT(innov_x, -gate_limit, gate_limit);
    innov_y = LIMIT(innov_y, -gate_limit, gate_limit);

    // 计算卡尔曼增益 K (这里仅简化为标量计算，因为 X Y 是解耦的)
    kx = ekf_lite_p_xy[2] / (ekf_lite_p_xy[2] + r_flow);
    ky = ekf_lite_p_xy[3] / (ekf_lite_p_xy[3] + r_flow);

    // 状态更新与协方差更新
    ekf_lite_state.vx += kx * innov_x;
    ekf_lite_state.vy += ky * innov_y;
    ekf_lite_p_xy[2] *= (1.0f - kx);
    ekf_lite_p_xy[3] *= (1.0f - ky);

    ekf_lite_state.vx = LIMIT(ekf_lite_state.vx, -FLOW_LITE_VEL_LIMIT_CMS, FLOW_LITE_VEL_LIMIT_CMS);
    ekf_lite_state.vy = LIMIT(ekf_lite_state.vy, -FLOW_LITE_VEL_LIMIT_CMS, FLOW_LITE_VEL_LIMIT_CMS);
    ekf_lite_health.flow_used = 1u;
    flow_health.ekf_used = 1u;

    vehicle_state.current_pos_x = ekf_lite_state.x;
    vehicle_state.current_pos_y = ekf_lite_state.y;
    vehicle_state.current_vel_x = ekf_lite_state.vx;
    vehicle_state.current_vel_y = ekf_lite_state.vy;
}
