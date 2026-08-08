#include "zf_common_headfile.h"
#include "loc_ctrl.h"
#include "vision_nav.h"



attitude_control_t att_ctrl = {0};

ct_val_t ct_val = {0};
att_2l_ct_t att_2l_ct = {0};
att_1l_ct_t att_1l_ct = {0};
float att_voltage_output_scale = 1.0f;
uint8_t att_takeoff_yaw_rate_hold_active = 0u;

#define ATT_I_FREEZE_DUTY      3800
#define ATT_I_FREEZE_THROTTLE  (((float)(ATT_I_FREEZE_DUTY - MOTOR_DUTY_MIN_START) / (float)(MOTOR_DUTY_MAX - MOTOR_DUTY_MIN_START)) * MAX_CT_VAL + MOTOR_OUTPUT_DEADZONE)
#define ATT_SP_RATE_FF_GAIN       0.35f
#define ATT_SP_RATE_FF_LIMIT_DPS  8.0f
#define ATT_SP_RATE_FF_ALPHA      0.35f
#define ATT_LARGE_ERR_START_DEG       0.50f
#define ATT_LARGE_ERR_BLEND_DEG       0.50f
#define ATT_LARGE_ERR_RATE_GAIN       1.75f
#define ATT_LARGE_ERR_RATE_LIMIT_DPS  3.00f
#define ATT_VOLTAGE_REF_V              10.20f
#define ATT_VOLTAGE_SCALE_MIN           0.90f
#define ATT_VOLTAGE_SCALE_MAX           1.08f

static float att_prev_target_roll = 0.0f;
static float att_prev_target_pitch = 0.0f;
static float att_roll_sp_rate_ff = 0.0f;
static float att_pitch_sp_rate_ff = 0.0f;
static uint8_t att_sp_rate_ff_ready = 0u;

static float att_update_sp_rate_ff(float target,
                                   float *prev_target,
                                   float *filtered_rate,
                                   float dT_s)
{
    float raw_rate;

    if((att_sp_rate_ff_ready == 0u) || (dT_s <= 0.0f) || (dT_s > 0.1f))
    {
        *prev_target = target;
        *filtered_rate = 0.0f;
        return 0.0f;
    }

    raw_rate = (target - *prev_target) / dT_s;
    raw_rate = LIMIT(raw_rate, -MAX_ROLLING_SPEED, MAX_ROLLING_SPEED);
    *filtered_rate += ATT_SP_RATE_FF_ALPHA * (raw_rate - *filtered_rate);
    *filtered_rate = LIMIT(*filtered_rate,
                           -ATT_SP_RATE_FF_LIMIT_DPS,
                            ATT_SP_RATE_FF_LIMIT_DPS);
    *prev_target = target;

    return *filtered_rate;
}

/* Increase attitude recovery only after a meaningful tracking error exists.
 * The zero-boost region preserves the current small-error/high-frequency
 * behaviour, while the independent limit keeps a large LOC request bounded. */
static float att_large_error_rate_boost(float angle_error_deg)
{
    float boost = 0.0f;
    float excess = 0.0f;
    float blend = 1.0f;
    float t = 0.0f;

    if(angle_error_deg > ATT_LARGE_ERR_START_DEG)
    {
        excess = angle_error_deg - ATT_LARGE_ERR_START_DEG;
        if(excess < ATT_LARGE_ERR_BLEND_DEG)
        {
            t = excess / ATT_LARGE_ERR_BLEND_DEG;
            blend = t * t * (3.0f - 2.0f * t);
        }
        boost = excess * ATT_LARGE_ERR_RATE_GAIN * blend;
    }
    else if(angle_error_deg < -ATT_LARGE_ERR_START_DEG)
    {
        excess = -angle_error_deg - ATT_LARGE_ERR_START_DEG;
        if(excess < ATT_LARGE_ERR_BLEND_DEG)
        {
            t = excess / ATT_LARGE_ERR_BLEND_DEG;
            blend = t * t * (3.0f - 2.0f * t);
        }
        boost = -excess * ATT_LARGE_ERR_RATE_GAIN * blend;
    }

    return LIMIT(boost,
                 -ATT_LARGE_ERR_RATE_LIMIT_DPS,
                  ATT_LARGE_ERR_RATE_LIMIT_DPS);
}

static float att_get_voltage_output_scale(void)
{
    float voltage = vehicle_state.battery_voltage_filtered;
    float voltage_ratio;

    if(voltage <= 1.0f)
    {
        return 1.0f;
    }

    voltage_ratio = ATT_VOLTAGE_REF_V / voltage;

    /* A 1.5-power model is the controlled midpoint between the proven-safe
     * linear compensation and the overly aggressive square model.  It trims
     * full-battery rate-loop authority without giving up as much large-error
     * recovery authority as ratio^2. */
    return LIMIT(voltage_ratio * sqrtf(voltage_ratio),
                 ATT_VOLTAGE_SCALE_MIN,
                 ATT_VOLTAGE_SCALE_MAX);
}

static uint8_t att_rate_i_enabled(void)
{
    return (vehicle_state.armed != 0 && vehicle_setpoint.target_throttle >= ATT_I_FREEZE_THROTTLE);
}

static float att_rate_pid_solve(pid_param_t *pid,
                                float error,
                                float dT_s,
                                uint8_t enable_i,
                                float output_limit)
{
    float old_i = pid->out_i;
    float output = stan_pid_solve(pid, error, dT_s, 0);

    if(enable_i == 0)
    {
        // 低油门时逐渐释放旧积分，避免再次起飞突然输出
        pid->out_i *= 0.995f;
        output = pid->out_p + pid->out_i + pid->out_d;
    }
    else if((output > output_limit && error > 0.0f) ||
            (output < -output_limit && error < 0.0f))
    {
        // 输出饱和且误差仍推动积分加深：撤销本周期积分
        pid->out_i = old_i;
        output = pid->out_p + pid->out_i + pid->out_d;
    }

    return LIMIT(output, -output_limit, output_limit);
}


// 姿态控制初始化
void att_ctrl_init(void)
{
    // 直接初始化PID参数（可以根据实际调试调整）
    
    // 角度环PID参数（外环）- 用于自稳模式
    att_ctrl.angle_pid[0] = (pid_param_t){
        .kp = 3.190f,    // Roll Kp
        .ki = 0.00f,     // Roll Ki
        .kd = 0.0f,      // Roll Kd
        .i_max = 10.0f,
        .p_max = 50.0f,
        .d_max = 10.0f,
        .low_pass = 0.2f
    };
    
    att_ctrl.angle_pid[1] = (pid_param_t){
        .kp = 3.184f,    // Pitch Kp
        .ki = 0.00f,     // Pitch Ki
        .kd = 0.0f,      // Pitch Kd
        .i_max = 10.0f,
        .p_max = 50.0f,
        .d_max = 10.0f,
        .low_pass = 0.2f
    };
    
    att_ctrl.angle_pid[2] = (pid_param_t){
        .kp = 0.872f,    // Yaw Kp
        .ki = 0.00f,     // Yaw Ki
        .kd = 0.0f,     // Yaw Kd
        .i_max = 10.0f,
        .p_max = 50.0f,
        .d_max = 10.0f,
        .low_pass = 0.2f
    };
    
    // 角速率环PID参数（内环）- 用于精确控制
    att_ctrl.rate_pid[0] = (pid_param_t)
    {
        .kp = 0.1228f,    // Roll Rate Kp
        .ki = 0.026f,     // Roll Rate Ki
        .kd = 0.0024f,    // Roll Rate Kd
        .i_max = 50.0f,
        .p_max = 500.0f,
        .d_max = 100.0f,
        .low_pass = 0.20f
    };
    
    att_ctrl.rate_pid[1] = (pid_param_t){
        .kp = 0.1353f,    // Pitch Rate Kp
        .ki = 0.030f,     // Pitch Rate Ki
        .kd = 0.0020f,    // Pitch Rate Kd
        .i_max = 50.0f,
        .p_max = 500.0f,
        .d_max = 100.0f,
        .low_pass = 0.20f
    };
    
    att_ctrl.rate_pid[2] = (pid_param_t){
        .kp = 0.354f,     // Yaw Rate Kp
        .ki = 0.025f,      // Yaw Rate Ki
        .kd = 0.0003f,    // Yaw Rate Kd
        .i_max = 50.0f,
        .p_max = 500.0f,
        .d_max = 100.0f,
        .low_pass = 0.2f
    };
    
}

void att_ctrl_set_yaw_target(float yaw_deg)
{
    while(yaw_deg < -180.0f) yaw_deg += 360.0f;
    while(yaw_deg > 180.0f) yaw_deg -= 360.0f;

    /* target_yaw_rate=0 only preserves the previously integrated heading.
     * Mission entry needs an explicit absolute heading, matching the
     * reference completion code's fixed FlyControl_yaw=0 behavior. */
    att_takeoff_yaw_rate_hold_active = 0u;
    stan_pid_reset(&att_ctrl.angle_pid[2]);
    att_2l_ct.exp_yaw = yaw_deg;
    att_1l_ct.set_yaw_speed = 0.0f;
}




// 角度环控制（外环）
void att_2level_ctrl( float dT_s)
{
    float yaw_speed_limit =
        (vision_nav_yaw_spin_is_active() != 0u) ?
        VISION_NAV_YAW_SPIN_MAX_RATE_DPS : MAX_YAW_SPEED;

     // 未起飞或锁定时复位
    if(vehicle_state.armed==0)
    {
        att_2l_ct.exp_rol = 0;
        att_2l_ct.exp_pit = 0;
        att_1l_ct.set_yaw_speed = 0;
        att_2l_ct.exp_yaw = vehicle_state.current_yaw; // 复位期望偏航角为当前偏航角，防止解锁瞬间大误差导致的剧烈旋转
        att_prev_target_roll = 0.0f;
        att_prev_target_pitch = 0.0f;
        att_roll_sp_rate_ff = 0.0f;
        att_pitch_sp_rate_ff = 0.0f;
        att_sp_rate_ff_ready = 0u;
        att_takeoff_yaw_rate_hold_active = 0u;

        
        // 复位PID积分项，防止地面积分饱和
        for(int i = 0; i < 3; i++) 
        {
            stan_pid_reset(&att_ctrl.angle_pid[i]);
            stan_pid_reset(&att_ctrl.rate_pid[i]);
        }
        return;
    }

    // 获取当前姿态（从IMU）
  
    att_2l_ct.fb_rol = vehicle_state.current_roll;
    att_2l_ct.fb_pit = vehicle_state.current_pitch;
    att_2l_ct.fb_yaw = vehicle_state.current_yaw;
    
    
    // 计算期望姿态角
    att_2l_ct.exp_rol = vehicle_setpoint.target_roll;

    att_2l_ct.exp_pit = vehicle_setpoint.target_pitch;
    
    // 限幅
    att_2l_ct.exp_rol = LIMIT(att_2l_ct.exp_rol, -MAX_ANGLE, MAX_ANGLE);
    att_2l_ct.exp_pit = LIMIT(att_2l_ct.exp_pit, -MAX_ANGLE, MAX_ANGLE);

    float roll_sp_rate_ff = 0.0f;
    float pitch_sp_rate_ff = 0.0f;

    /* Location control produces a slew-limited attitude trajectory.  Give
     * that trajectory a modest lead without changing manual attitude feel. */
    if(loc_1l_ct.loc_ready != 0u)
    {
        roll_sp_rate_ff = att_update_sp_rate_ff(att_2l_ct.exp_rol,
                                                  &att_prev_target_roll,
                                                  &att_roll_sp_rate_ff,
                                                  dT_s);
        pitch_sp_rate_ff = att_update_sp_rate_ff(att_2l_ct.exp_pit,
                                                   &att_prev_target_pitch,
                                                   &att_pitch_sp_rate_ff,
                                                   dT_s);
        if(att_sp_rate_ff_ready == 0u)
        {
            att_sp_rate_ff_ready = 1u;
        }
    }
    else
    {
        att_prev_target_roll = att_2l_ct.exp_rol;
        att_prev_target_pitch = att_2l_ct.exp_pit;
        att_roll_sp_rate_ff = 0.0f;
        att_pitch_sp_rate_ff = 0.0f;
        att_sp_rate_ff_ready = 0u;
    }
    att_1l_ct.sp_rate_ff[0] = ATT_SP_RATE_FF_GAIN * roll_sp_rate_ff;
    att_1l_ct.sp_rate_ff[1] = ATT_SP_RATE_FF_GAIN * pitch_sp_rate_ff;
    
    /*
     * A six-axis IMU has no absolute yaw reference. During the first few
     * centimetres of takeoff, vibration-induced gyro bias can therefore
     * accumulate into a false heading error. Chasing that error creates a
     * large diagonal motor differential exactly when thrust matching is
     * least repeatable. Keep only zero-rate yaw damping below the horizontal
     * takeoff gate, then capture the current heading for a bumpless handoff.
     */
    if((vehicle_state.flight_mode == FLY_AUTOTAKEOFF) &&
       (vehicle_state.current_height <
        ATT_TAKEOFF_YAW_HEADING_ENABLE_HEIGHT_CM))
    {
        if(att_takeoff_yaw_rate_hold_active == 0u)
        {
            stan_pid_reset(&att_ctrl.angle_pid[2]);
            stan_pid_reset(&att_ctrl.rate_pid[2]);
        }
        att_takeoff_yaw_rate_hold_active = 1u;
        att_1l_ct.set_yaw_speed = 0.0f;
        att_2l_ct.exp_yaw = att_2l_ct.fb_yaw;
        att_2l_ct.yaw_err = 0.0f;
    }
    else
    {
        float set_yaw_av_tmp = vehicle_setpoint.target_yaw_rate;

        if(att_takeoff_yaw_rate_hold_active != 0u)
        {
            att_2l_ct.exp_yaw = att_2l_ct.fb_yaw;
            att_1l_ct.set_yaw_speed = 0.0f;
            stan_pid_reset(&att_ctrl.angle_pid[2]);
        }
        att_takeoff_yaw_rate_hold_active = 0u;

        set_yaw_av_tmp =
            LIMIT(set_yaw_av_tmp, -yaw_speed_limit, yaw_speed_limit);
        att_1l_ct.set_yaw_speed +=
            LIMIT((set_yaw_av_tmp - att_1l_ct.set_yaw_speed),
                  -30.0f,
                   30.0f);
        att_2l_ct.exp_yaw += att_1l_ct.set_yaw_speed * dT_s;

        if(att_2l_ct.exp_yaw < -180.0f) att_2l_ct.exp_yaw += 360.0f;
        else if(att_2l_ct.exp_yaw > 180.0f) att_2l_ct.exp_yaw -= 360.0f;

        att_2l_ct.yaw_err = att_2l_ct.exp_yaw - att_2l_ct.fb_yaw;
        if(att_2l_ct.yaw_err < -180.0f) att_2l_ct.yaw_err += 360.0f;
        else if(att_2l_ct.yaw_err > 180.0f) att_2l_ct.yaw_err -= 360.0f;
    }
    


    
    // 外环PID计算
    float roll_angle_error = att_2l_ct.exp_rol - att_2l_ct.fb_rol;
    float pitch_angle_error = att_2l_ct.exp_pit - att_2l_ct.fb_pit;
    float roll_large_error_boost = 0.0f;
    float pitch_large_error_boost = 0.0f;

    /* Only LOC gets the large-error recovery boost.  Manual attitude feel and
     * the Flash-loaded PID parameters remain unchanged. */
    if(loc_1l_ct.loc_ready != 0u)
    {
        roll_large_error_boost = att_large_error_rate_boost(roll_angle_error);
        pitch_large_error_boost = att_large_error_rate_boost(pitch_angle_error);
    }

    att_1l_ct.exp_ang_vel[0] =
        stan_pid_solve(&att_ctrl.angle_pid[0],
                       roll_angle_error,
                       dT_s,
                       0) + att_1l_ct.sp_rate_ff[0] + roll_large_error_boost;
    att_1l_ct.exp_ang_vel[1] =
        stan_pid_solve(&att_ctrl.angle_pid[1],
                       pitch_angle_error,
                       dT_s,
                       0) + att_1l_ct.sp_rate_ff[1] + pitch_large_error_boost;
    if(att_takeoff_yaw_rate_hold_active != 0u)
    {
        att_1l_ct.exp_ang_vel[2] = 0.0f;
    }
    else
    {
        att_1l_ct.exp_ang_vel[2] =
            stan_pid_solve(&att_ctrl.angle_pid[2],
                           att_2l_ct.yaw_err,
                           dT_s,
                           0);
    }
    
    // 限幅
    att_1l_ct.exp_ang_vel[0] = LIMIT(att_1l_ct.exp_ang_vel[0], -MAX_ROLLING_SPEED, MAX_ROLLING_SPEED);
    att_1l_ct.exp_ang_vel[1] = LIMIT(att_1l_ct.exp_ang_vel[1], -MAX_ROLLING_SPEED, MAX_ROLLING_SPEED);
    att_1l_ct.exp_ang_vel[2] = LIMIT(att_1l_ct.exp_ang_vel[2],
                                    -yaw_speed_limit,
                                     yaw_speed_limit);
}

// 角速率环控制（内环）
void att_1level_ctrl(float dT_s)
{
    float yaw_control_limit =
        (vision_nav_yaw_spin_is_active() != 0u) ?
        VISION_NAV_YAW_SPIN_CT_LIMIT : MAX_YAW_CT_VAL;
    if(vehicle_state.armed==0)
    {
        ct_val.rol = 0;
        ct_val.pit = 0;
        ct_val.yaw = 0;
        att_voltage_output_scale = 1.0f;
        att_1l_ct.fb_ang_vel[0] = imu_data.gyro_actual[0];
        att_1l_ct.fb_ang_vel[1] = imu_data.gyro_actual[1];
        att_1l_ct.fb_ang_vel[2] = imu_data.gyro_actual[2];
        for(int i = 0; i < 3; i++)
        {
            stan_pid_reset(&att_ctrl.rate_pid[i]);
        }
        return;
    }

    // gyro_actual 已在 IMU 层经过 25 Hz 二阶低通；内环重复低通会增加相位滞后并诱发振荡。
    att_1l_ct.fb_ang_vel[0] = imu_data.gyro_actual[0];
    att_1l_ct.fb_ang_vel[1] = imu_data.gyro_actual[1];
    att_1l_ct.fb_ang_vel[2] = imu_data.gyro_actual[2];

    // 内环PID计算
    uint8_t enable_rate_i = att_rate_i_enabled();
    ct_val.rol = att_rate_pid_solve(&att_ctrl.rate_pid[0],att_1l_ct.exp_ang_vel[0] - att_1l_ct.fb_ang_vel[0],  dT_s, enable_rate_i, MAX_ATT1_VAL);

    ct_val.pit = att_rate_pid_solve(&att_ctrl.rate_pid[1],att_1l_ct.exp_ang_vel[1] - att_1l_ct.fb_ang_vel[1],dT_s, enable_rate_i, MAX_ATT1_VAL);

    ct_val.yaw = att_rate_pid_solve(&att_ctrl.rate_pid[2],
                                    att_1l_ct.exp_ang_vel[2] -
                                        att_1l_ct.fb_ang_vel[2],
                                    dT_s,
                                    enable_rate_i,
                                    yaw_control_limit);

    /* Normalize actuator authority to the voltage at which the current Flash
     * PID set was validated.  Hover throttle already adapts the base thrust,
     * so the local differential-control gain is compensated linearly, not
     * quadratically. */
    att_voltage_output_scale = att_get_voltage_output_scale();
    ct_val.rol *= att_voltage_output_scale;
    ct_val.pit *= att_voltage_output_scale;
    ct_val.yaw *= att_voltage_output_scale;
    // 输出限幅
    ct_val.rol =LIMIT(ct_val.rol, -MAX_ATT1_VAL, MAX_ATT1_VAL);
    ct_val.pit =  LIMIT(ct_val.pit, -MAX_ATT1_VAL, MAX_ATT1_VAL);
    ct_val.yaw = LIMIT(ct_val.yaw,
                       -yaw_control_limit,
                        yaw_control_limit);
}

// 姿态控制主函数
/*void att_ctrl_update(float exp_rol, float exp_pit, float exp_yaw)
{
    static uint32_t last_time = 0;
    uint32_t current_time = system_time_ms();
    float dT_s = (current_time - last_time) / 1000.0f;
    
    if(dT_s <= 0 || dT_s > 0.1f) 
    {
        dT_s = 0.01f;
    }
    last_time = current_time;
    
    // 检查是否解锁
    if(!att_ctrl.armed || !att_ctrl.calibrated) 
    {
        ct_val.rol = 0;
        ct_val.pit = 0;
        ct_val.yaw = 0;
        motor_set_duty(MOTOR_RF, 0);
        motor_set_duty(MOTOR_RB, 0);
        motor_set_duty(MOTOR_LF, 0);
        motor_set_duty(MOTOR_LB, 0);
        return;
    }
    
    // 外环控制
    att_2level_ctrl(exp_rol, exp_pit, exp_yaw, dT_s);
    
    // 内环控制
    att_1level_ctrl(dT_s);
    

}*/
