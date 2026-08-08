#include "zf_common_headfile.h"

float tof_dist_cm=0.0f;
float height_cm=0.0f;
tof_health_t tof_health = {0};

float acc_z_cm_s2;
float acc_z_world_raw_cm_s2;
float acc_z_world_bias_cm_s2 = 0.0f;
float acc_z_world_corrected_cm_s2 = 0.0f;
float acc_z_world_lpf_cm_s2 = 0.0f;
float tof_diff_vel_debug = 0.0f;
float height_tof_vel_raw_debug = 0.0f;
float height_tof_vel_filtered_debug = 0.0f;
float height_vel_innov_debug = 0.0f;
float height_tof_vel_dt_ms_debug = 0.0f;
uint8_t height_vel_update_used_debug = 0u;

static uint16_t tof_last_raw_mm = 0u;
static uint32_t tof_last_sample_time_us = 0u;
static uint8_t tof_last_raw_valid = 0u;
static uint8_t tof_last_streamcount = 0u;
static uint8_t tof_last_stream_valid = 0u;
static uint32_t tof_ground_fault_since_us = 0u;
static uint32_t tof_last_recovery_attempt_us = 0u;

static void tof_health_reset_tracking(void)
{
    tof_last_raw_mm = 0u;
    tof_last_sample_time_us = 0u;
    tof_last_raw_valid = 0u;
    tof_last_streamcount = 0u;
    tof_last_stream_valid = 0u;
    tof_ground_fault_since_us = 0u;
    tof_health.raw_valid = 0u;
    tof_health.stale = 1u;
    tof_health.stuck = 0u;
    tof_health.valid_frame_count = 0u;
    tof_health.last_fresh_time_us = 0u;
    tof_health.unchanged_time_us = 0u;
}

static void tof_health_update(uint8_t fresh_valid, uint16_t raw_mm)
{
    uint32_t now_us = system_time_us();

    if(fresh_valid != 0u)
    {
        if(tof_last_raw_valid != 0u)
        {
            uint32_t dt_us = now_us - tof_last_sample_time_us;

            if(raw_mm == tof_last_raw_mm)
            {
                tof_health.unchanged_time_us += dt_us;
            }
            else
            {
                tof_health.unchanged_time_us = 0u;
            }
        }
        else
        {
            tof_health.unchanged_time_us = 0u;
        }

        tof_last_raw_mm = raw_mm;
        tof_last_raw_valid = 1u;
        tof_last_sample_time_us = now_us;
        tof_health.last_fresh_time_us = now_us;
        if(tof_health.valid_frame_count < TOF_PREFLIGHT_VALID_FRAMES)
        {
            tof_health.valid_frame_count++;
        }
    }

    tof_health.stale =
        ((now_us - tof_health.last_fresh_time_us) > TOF_STALE_TIMEOUT_US) ?
        1u : 0u;
    /* A stationary aircraft can legitimately return the exact same range for
     * many frames. Frame freshness is checked with the VL53L8 streamcount in
     * tof_get_distance(); a frozen stream therefore becomes stale instead of
     * being inferred from an unchanged measurement value. */
    tof_health.stuck = 0u;
}

uint8_t tof_health_ok(void)
{
    if((tof_health.last_fresh_time_us == 0u) ||
       (tof_health.stale != 0u) ||
       (tof_health.stuck != 0u) ||
       (tof_health.valid_frame_count < TOF_PREFLIGHT_VALID_FRAMES) ||
       (tof_health.height_cm < HEIGHT_NEAR_GROUND_RECOVER_CM) ||
       (tof_health.height_cm > MAX_HEIGHT))
    {
        return 0u;
    }

    if((vehicle_state.armed == 0u) &&
       (tof_health.height_cm > TOF_PREFLIGHT_MAX_HEIGHT_CM))
    {
        return 0u;
    }

    return 1u;
}

static uint8_t height_ekf_is_landed(void)
{
    if(vehicle_state.armed == 0u)
    {
        return 1u;
    }

    return (throttle_ramped_debug < (system_get_hover_throttle_base() - HEIGHT_EKF_LANDED_THR_MARGIN));
}

void tof_init(void)
{
    uint8_t status;

    tof_last_recovery_attempt_us = system_time_us();
    tof_health_reset_tracking();
#if (TOF_SENSOR_TYPE == TOF_SENSOR_VL53L8)
    status = vl53l8_init();
#else
    dl1b_init();
    status = 0u;
#endif
    tof_health.recovery_last_status = status;
}

static void tof_recovery_task(void)
{
    uint32_t now_us;
    uint8_t ground_height_invalid;
    uint8_t recovery_required = 0u;

#if ((MCAR_COMM_FIXED_TARGET_TEST_ENABLE != 0u) || \
     (MCAR_COMM_HANDHELD_FOLLOW_TEST_ENABLE != 0u) || \
     (MCAR_COMM_HANDHELD_BEACON_FOLLOW_TEST_ENABLE != 0u))
    /*
     * These disarmed communication/vision tests do not use ToF as a mission
     * gate.  When the downward VL53L8 sees the car roof (or is hand-held
     * above the preflight range), the normal disarmed recovery path would
     * synchronously reload the sensor firmware.  That blocks the CM7_0 main
     * loop for about 3.52 s and starves the independent 20 ms car heartbeat.
     * Keep normal ranging active, but do not auto-reinitialize the sensor in
     * a bench-test build.  Startup initialization and all formal-task builds
     * are unchanged.
     */
    tof_ground_fault_since_us = 0u;
    return;
#endif

    if(vehicle_state.armed != 0u)
    {
        tof_ground_fault_since_us = 0u;
        return;
    }

    now_us = system_time_us();
    ground_height_invalid =
        ((tof_health.raw_valid != 0u) &&
         ((tof_health.height_cm < HEIGHT_NEAR_GROUND_RECOVER_CM) ||
          (tof_health.height_cm > TOF_PREFLIGHT_MAX_HEIGHT_CM))) ? 1u : 0u;

    if(ground_height_invalid != 0u)
    {
        if(tof_ground_fault_since_us == 0u)
        {
            tof_ground_fault_since_us = now_us;
        }
        else if((now_us - tof_ground_fault_since_us) >= TOF_GROUND_FAULT_CONFIRM_US)
        {
            recovery_required = 1u;
        }
    }
    else
    {
        tof_ground_fault_since_us = 0u;
    }

    if((tof_health.stale != 0u) || (tof_health.stuck != 0u))
    {
        recovery_required = 1u;
    }

    if((recovery_required != 0u) &&
       ((tof_last_recovery_attempt_us == 0u) ||
        ((now_us - tof_last_recovery_attempt_us) >= TOF_RECOVERY_RETRY_US)))
    {
        tof_last_recovery_attempt_us = now_us;
        tof_health.recovery_active = 1u;
        tof_health.recovery_count++;
        tof_init();
        tof_health.recovery_active = 0u;
    }
}

void tof_get_distance(void)
{
#if (TOF_SENSOR_TYPE == TOF_SENSOR_VL53L8)
    uint16_t vl53l8_distance_mm = 0u;
    uint8_t fresh_valid = 0u;

    vl53l8_update();
    tof_health.raw_valid = 0u;
    if((vl53l8_data.data_ready != 0u) &&
       (vl53l8_get_height_mm(&vl53l8_distance_mm) != 0u))
    {
        uint8_t streamcount = vl53l8_data.streamcount;

        if((tof_last_stream_valid == 0u) ||
           (streamcount != tof_last_streamcount))
        {
            tof_dist_cm = (float)vl53l8_distance_mm / 10.0f;
            tof_health.raw_valid = 1u;
            fresh_valid = 1u;
            tof_last_streamcount = streamcount;
            tof_last_stream_valid = 1u;
            tof_health.streamcount = streamcount;
        }
    }
    tof_health_update(fresh_valid, vl53l8_distance_mm);
#else
    uint8_t fresh_valid = 0u;
    uint16_t dl1b_distance_snapshot_mm = 0u;

    dl1b_get_distance();
    tof_health.raw_valid = 0u;
    if(dl1b_finsh_flag)
    {
        tof_dist_cm = dl1b_distance_mm / 10.0f;  // mm转cm
        tof_health.raw_valid = 1u;
        fresh_valid = 1u;
        dl1b_distance_snapshot_mm = dl1b_distance_mm;
    }
    tof_health_update(fresh_valid, dl1b_distance_snapshot_mm);
#endif
    tof_health.raw_dist_cm = tof_dist_cm;
    tof_recovery_task();
}

// 更新当前高度与垂直速度
void update_current_height(float dT_s)
{
    tof_get_distance();
    
    // 数学上绝对正确的空间几何投影解耦
    float cos_r = vehicle_state.roll_cos;
    float cos_p = vehicle_state.pitch_cos;
    


    // TODO: 根据实际测试结果决定是否启用此行
    // if(cos_r < 0.965f) cos_r = 0.965f; 
    // if(cos_p < 0.965f) cos_p = 0.965f;
    
    // 1. 基础投影解耦（斜边拉直）
    height_cm = tof_dist_cm * cos_r * cos_p;
    
 
    
    // TOF 偏心安装物理补偿 
    float TOF_OFFSET_X = 8.5f;  // 1. TOF距离底板重心的前后偏移量 
    float TOF_OFFSET_Y = -0.5f; // Y轴向左，所以右侧是负数
    /* The lens is below the center of gravity.  Add its earth-vertical
     * component so current_height remains a center-of-gravity height. */
    float tof_to_cg_vertical_cm = TOF_TO_CG_VERTICAL_OFFSET_CM * cos_r * cos_p;
    
    // 计算因为飞机倾斜，导致 TOF 模块自身发生的真实物理升降高度

    float lever_arm_z = TOF_OFFSET_X * vehicle_state.pitch_sin 
                      + TOF_OFFSET_Y * vehicle_state.roll_sin;
    
    // 剔除偏心造成的假升降，还原出飞机真实重心的垂直高度！
    height_cm += tof_to_cg_vertical_cm - lever_arm_z;
    tof_health.height_cm = height_cm;
    
    static float last_height=0.0f;
    static float filtered_vel_z=0.0f;
    static float filtered_height=0.0f;
    // 简单的低通滤波
    if(filtered_height == 0.0f) 
    {
        filtered_height = height_cm; 
    }
    else 
    {
        filtered_height += 0.2f * (height_cm - filtered_height);
    }
    
    if(dT_s > 0 && dT_s < 0.5f)
    {
        float new_vel_z = (filtered_height - last_height) / dT_s;
        tof_diff_vel_debug = new_vel_z;
        
        // 获取真实的世界坐标系Z轴线性加速度 (去除了地球重力，单位 m/s²)
        float linear_acc_body[3], linear_acc_world[3];
        imu_get_linear_acceleration(linear_acc_body); 
        imu_transform_vector_body_to_world(linear_acc_body, linear_acc_world); 
        
        float acc_z_world_cm_s2 = linear_acc_world[2] * 100.0f; // m/s² 转换为 cm/s²
        
        // 低通滤波
        filtered_vel_z = 0.98f * (filtered_vel_z + acc_z_world_cm_s2*dT_s)+ 0.02f * new_vel_z;
        last_height = filtered_height;
    }

       // ===== 调试输出：观察投影前的原始值 =====
    /*static uint32_t debug_count = 0;
    if(debug_count++ % 10 == 0)  // 每10次输出一次，避免串口刷屏
    {
        printf("DEBUG: tof_dist_cm=%.2f, cos_r=%.3f, cos_p=%.3f, current_height=%.2f, pitch=%.2f, roll=%.2f\n", 
               tof_dist_cm, cos_r, cos_p, filtered_height, imu_data.pitch, imu_data.roll);
    }*/

    //vehicle_state.current_height = filtered_height;
    //vehicle_state.current_vel_z = filtered_vel_z;
}
void ekf_lite_predict_height(float dT_s)
{
    float linear_acc_body[3];
    float linear_acc_world[3];
    static uint8_t acc_z_world_lpf_valid = 0u;


    imu_get_linear_acceleration(linear_acc_body);
    imu_transform_vector_body_to_world(linear_acc_body, linear_acc_world);

    acc_z_world_raw_cm_s2 = linear_acc_world[2] * 100.0f;

    // 未解锁且机体静止时学习“世界 Z 线性加速度”残差。
    // 解锁后冻结，避免把真实飞行加速度学成零偏。
    if((vehicle_state.armed == 0u) &&
       (ABS(imu_data.gyro_actual[0]) < HEIGHT_ACC_Z_BIAS_GYRO_LIMIT) &&
       (ABS(imu_data.gyro_actual[1]) < HEIGHT_ACC_Z_BIAS_GYRO_LIMIT) &&
       (ABS(imu_data.gyro_actual[2]) < HEIGHT_ACC_Z_BIAS_GYRO_LIMIT) &&
       (ABS(acc_z_world_raw_cm_s2 - acc_z_world_bias_cm_s2) <
        HEIGHT_ACC_Z_BIAS_LEARN_LIMIT))
    {
        acc_z_world_bias_cm_s2 +=
            HEIGHT_ACC_Z_BIAS_ALPHA *
            (acc_z_world_raw_cm_s2 - acc_z_world_bias_cm_s2);
    }

    acc_z_world_corrected_cm_s2 =
        acc_z_world_raw_cm_s2 - acc_z_world_bias_cm_s2;

    if(acc_z_world_lpf_valid == 0u)
    {
        acc_z_world_lpf_cm_s2 = acc_z_world_corrected_cm_s2;
        acc_z_world_lpf_valid = 1u;
    }
    else
    {
        acc_z_world_lpf_cm_s2 +=
            HEIGHT_ACC_Z_LPF_ALPHA *
            (acc_z_world_corrected_cm_s2 - acc_z_world_lpf_cm_s2);
    }

    acc_z_cm_s2 = LIMIT(acc_z_world_lpf_cm_s2,
                        -HEIGHT_ACC_Z_LIMIT_CM_S2,
                        HEIGHT_ACC_Z_LIMIT_CM_S2) * HEIGHT_ACC_Z_EKF_GAIN;

    if(height_ekf_is_landed() != 0u)
    {
        acc_z_cm_s2 = 0.0f;
        acc_z_world_lpf_cm_s2 = 0.0f;
        acc_z_world_lpf_valid = 0u;
        ekf_lite_state.vz=0;
        ekf_lite_p_z[0][1] = 0.0f;
        ekf_lite_p_z[1][0] = 0.0f;
        return;
    }

    ekf_lite_state.z += ekf_lite_state.vz * dT_s + 0.5f * acc_z_cm_s2 * dT_s * dT_s;
    ekf_lite_state.vz += acc_z_cm_s2 * dT_s;

    ekf_lite_p_z[0][0] += ekf_lite_p_z[1][1] * dT_s * dT_s + HEIGHT_EKF_Q_Z * dT_s;
    ekf_lite_p_z[0][1] += ekf_lite_p_z[1][1] * dT_s;
    ekf_lite_p_z[1][0] = ekf_lite_p_z[0][1];
    ekf_lite_p_z[1][1] += HEIGHT_EKF_Q_VZ * dT_s;
}
void ekf_lite_update_height(void)
{
    static uint8_t height_ekf_z_inited = 0u;
    static uint8_t tof_gate_recover_count = 0u;
    static float tof_gate_recover_height_cm = 0.0f;

    if(vl53l8_data.data_ready==0)
    {
        return;
    }
    const float r_tof = HEIGHT_TOF_R_CM2;
    const float r_tof_vel = HEIGHT_TOF_VEL_R_CM2_S2;
    static float last_z_meas = 0.0f;
    static uint8_t last_z_meas_valid = 0u;
    static float tof_vel_filtered = 0.0f;
    static uint32_t last_update_time_us = 0u;
    uint32_t update_time_us = system_time_us();
    float z_meas = height_cm;
    float innov = z_meas - ekf_lite_state.z;
    float z_delta = 0.0f;
    float tof_vel_dt_s = 0.0f;
    float s;
    float k0;
    float k1;
    float p00;
    float p01;
    float p10;
    float p11;

    ekf_lite_health.tof_innov = innov;
    ekf_lite_health.tof_used = 0u;
    tof_health.ekf_used = 0u;
    tof_health.near_ground_recover = 0u;
    tof_health.gate_clipped = 0u;
    tof_health.range_reject = 0u;
    tof_health.innov_cm = innov;
    height_vel_update_used_debug = 0u;

    // 近地低于控制最小高度时，允许估计器恢复到地面状态，避免旧速度继续预测发散。
    if(height_cm >= HEIGHT_NEAR_GROUND_RECOVER_CM && height_cm < MIN_HEIGHT)
    {
        ekf_lite_state.z = MIN_HEIGHT;
        ekf_lite_state.vz = 0.0f;
        ekf_lite_p_z[0][0] = 25.0f;
        ekf_lite_p_z[0][1] = 0.0f;
        ekf_lite_p_z[1][0] = 0.0f;
        ekf_lite_p_z[1][1] = 25.0f;
        ekf_lite_health.tof_used = 0u;
        vehicle_state.current_height = ekf_lite_state.z;
        vehicle_state.current_vel_z = ekf_lite_state.vz;
        tof_health.near_ground_recover = 1u;
        tof_health.height_est_cm = ekf_lite_state.z;
        tof_health.vel_z_cm_s = ekf_lite_state.vz;
        return;
    }

    // 如果高度处于不合理范围，拒绝使用本次TOF数据更新
    if(height_cm > MAX_HEIGHT || height_cm < HEIGHT_NEAR_GROUND_RECOVER_CM)
    {
        tof_health.range_reject = 1u;
        return;
    }

    if((height_ekf_z_inited == 0u) || (vehicle_state.armed == 0u))
    {
        ekf_lite_state.z = LIMIT(height_cm, MIN_HEIGHT, MAX_HEIGHT);
        ekf_lite_state.vz = 0.0f;
        ekf_lite_p_z[0][0] = 25.0f;
        ekf_lite_p_z[0][1] = 0.0f;
        ekf_lite_p_z[1][0] = 0.0f;
        ekf_lite_p_z[1][1] = 25.0f;
        height_ekf_z_inited = 1u;
        tof_gate_recover_count = 0u;
        vehicle_state.current_height = ekf_lite_state.z;
        vehicle_state.current_vel_z = ekf_lite_state.vz;
        tof_health.height_est_cm = ekf_lite_state.z;
        tof_health.vel_z_cm_s = ekf_lite_state.vz;
        return;
    }

    // 新息门控 (Gate)：如果误差过大(>35cm)，说明可能是地形突变或干扰，拒绝更新
    if(fabs(innov) > EKF_LITE_TOF_GATE_CM)
    {
        /*
         * A valid, persistent ToF step must not leave the height estimator
         * permanently stranded behind the innovation gate.  Require five
         * mutually consistent measurements before re-seeding; isolated
         * range spikes are still rejected.
         */
        if((tof_gate_recover_count == 0u) ||
           (fabsf(height_cm - tof_gate_recover_height_cm) <= 10.0f))
        {
            tof_gate_recover_count++;
        }
        else
        {
            tof_gate_recover_count = 1u;
        }
        tof_gate_recover_height_cm = height_cm;

        if(tof_gate_recover_count >= 5u)
        {
            ekf_lite_state.z = LIMIT(height_cm, MIN_HEIGHT, MAX_HEIGHT);
            ekf_lite_state.vz = 0.0f;
            ekf_lite_p_z[0][0] = 25.0f;
            ekf_lite_p_z[0][1] = 0.0f;
            ekf_lite_p_z[1][0] = 0.0f;
            ekf_lite_p_z[1][1] = 25.0f;
            tof_gate_recover_count = 0u;
            vehicle_state.current_height = ekf_lite_state.z;
            vehicle_state.current_vel_z = ekf_lite_state.vz;
            tof_health.height_est_cm = ekf_lite_state.z;
            tof_health.vel_z_cm_s = ekf_lite_state.vz;
            tof_health.ekf_used = 1u;
            return;
        }

        ekf_lite_state.vz = 0.0f;
        vehicle_state.current_height = ekf_lite_state.z;
        vehicle_state.current_vel_z = ekf_lite_state.vz;
        tof_health.gate_clipped = 1u;
        tof_health.height_est_cm = ekf_lite_state.z;
        tof_health.vel_z_cm_s = ekf_lite_state.vz;
        return;
    }

    tof_gate_recover_count = 0u;

    // 计算新息协方差 S = H * P * H^T + R
    s = ekf_lite_p_z[0][0] + r_tof;
    if(s < 0.001f)
    {
        return;
    }

    p00 = ekf_lite_p_z[0][0];
    p01 = ekf_lite_p_z[0][1];
    p10 = ekf_lite_p_z[1][0];
    p11 = ekf_lite_p_z[1][1];

    // 计算卡尔曼增益 K = P * H^T * S^-1
    k0 = p00 / s;
    k1 = p10 / s;

    // 状态更新：X = X + K * innov
    ekf_lite_state.z += k0 * innov;
    ekf_lite_state.vz += k1 * innov;

    // 协方差更新：P = (I - K * H) * P
    ekf_lite_p_z[0][0] = p00 - k0 * p00;
    ekf_lite_p_z[0][1] = p01 - k0 * p01;
    ekf_lite_p_z[1][0] = p10 - k1 * p00;
    ekf_lite_p_z[1][1] = p11 - k1 * p01;

    // 对最终状态进行限幅，防止滤波发散
    ekf_lite_state.z = LIMIT(ekf_lite_state.z, MIN_HEIGHT, MAX_HEIGHT);
    ekf_lite_state.vz = LIMIT(ekf_lite_state.vz, -MAX_VEL_XYZ, MAX_VEL_XYZ);

    if(last_z_meas_valid != 0u)
    {
        z_delta = z_meas - last_z_meas;
#if HEIGHT_TOF_VEL_OBS_ENABLE
        tof_vel_dt_s = (update_time_us - last_update_time_us) / 1000000.0f;
        height_tof_vel_dt_ms_debug = tof_vel_dt_s * 1000.0f;
        if(tof_vel_dt_s > 0.005f && tof_vel_dt_s < 0.2f)
        {
            float tof_vel_raw_unclamped = z_delta / tof_vel_dt_s;
            float tof_vel_raw = LIMIT(tof_vel_raw_unclamped,
                                      -HEIGHT_TOF_VEL_LIMIT_CM_S,
                                      HEIGHT_TOF_VEL_LIMIT_CM_S);
            float vel_innov;
            float vel_s;
            float kv0;
            float kv1;

            height_tof_vel_raw_debug = tof_vel_raw;
            if(ABS(tof_vel_raw_unclamped) <= HEIGHT_TOF_VEL_REJECT_CM_S)
            {
                tof_vel_filtered += HEIGHT_TOF_VEL_LPF_ALPHA * (tof_vel_raw - tof_vel_filtered);
                height_tof_vel_filtered_debug = tof_vel_filtered;
                vel_innov = tof_vel_filtered - ekf_lite_state.vz;
                height_vel_innov_debug = vel_innov;
                vel_s = ekf_lite_p_z[1][1] + r_tof_vel;

                if(vel_s > 0.001f)
                {
                    p00 = ekf_lite_p_z[0][0];
                    p01 = ekf_lite_p_z[0][1];
                    p10 = ekf_lite_p_z[1][0];
                    p11 = ekf_lite_p_z[1][1];

                    kv0 = p01 / vel_s;
                    kv1 = p11 / vel_s;

                    ekf_lite_state.z += kv0 * vel_innov;
                    ekf_lite_state.vz += kv1 * vel_innov;
                    height_vel_update_used_debug = 1u;

                    ekf_lite_p_z[0][0] = p00 - kv0 * p10;
                    ekf_lite_p_z[0][1] = p01 - kv0 * p11;
                    ekf_lite_p_z[1][0] = p10 - kv1 * p10;
                    ekf_lite_p_z[1][1] = p11 - kv1 * p11;
                }
            }
            else
            {
                height_tof_vel_filtered_debug = tof_vel_filtered;
                height_vel_innov_debug = 0.0f;
            }
        }
#endif
        if(vehicle_state.armed == 0u &&
           ABS(innov) < HEIGHT_ZUPT_INNOV_CM &&
           ABS(z_delta) < HEIGHT_ZUPT_DELTA_CM)
        {
            ekf_lite_state.vz *= HEIGHT_ZUPT_VEL_DECAY;
        }
    }
    last_z_meas = z_meas;
    last_z_meas_valid = 1u;
    last_update_time_us = update_time_us;

    ekf_lite_state.z = LIMIT(ekf_lite_state.z, MIN_HEIGHT, MAX_HEIGHT);
    ekf_lite_state.vz = LIMIT(ekf_lite_state.vz, -MAX_VEL_XYZ, MAX_VEL_XYZ);

    ekf_lite_health.tof_used = 1u;
    tof_health.ekf_used = 1u;
    tof_health.height_est_cm = ekf_lite_state.z;
    tof_health.vel_z_cm_s = ekf_lite_state.vz;

    vehicle_state.current_height = ekf_lite_state.z;
    vehicle_state.current_vel_z = ekf_lite_state.vz;
}
