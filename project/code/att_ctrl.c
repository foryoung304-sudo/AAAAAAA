#include "zf_common_headfile.h"



attitude_control_t att_ctrl = {0};

ct_val_t ct_val = {0};
att_2l_ct_t att_2l_ct = {0};
att_1l_ct_t att_1l_ct = {0};

#define ATT_I_FREEZE_DUTY      3800
#define ATT_I_FREEZE_THROTTLE  (((float)(ATT_I_FREEZE_DUTY - MOTOR_DUTY_MIN_START) / (float)(MOTOR_DUTY_MAX - MOTOR_DUTY_MIN_START)) * MAX_CT_VAL + MOTOR_OUTPUT_DEADZONE)
#define ATT_SP_RATE_FF_GAIN       0.00f
#define ATT_SP_RATE_FF_LIMIT_DPS  8.0f
#define ATT_SP_RATE_FF_ALPHA      0.35f

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
        .kp =2.5f,      // Roll Kp
        .ki = 0.00f,     // Roll Ki
        .kd = 0.0f,      // Roll Kd
        .i_max = 10.0f,
        .p_max = 50.0f,
        .d_max = 10.0f,
        .low_pass = 0.2f
    };
    
    att_ctrl.angle_pid[1] = (pid_param_t){
        .kp = 2.5f,      // Pitch Kp
        .ki = 0.00f,     // Pitch Ki
        .kd = 0.0f,      // Pitch Kd
        .i_max = 10.0f,
        .p_max = 50.0f,
        .d_max = 10.0f,
        .low_pass = 0.2f
    };
    
    att_ctrl.angle_pid[2] = (pid_param_t){
        .kp = 0.9f,      // Yaw Kp
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
        .kp = 0.165f,     // Roll Rate Kp
        .ki = 0.03f,      // Roll Rate Ki
        .kd = 0.0005f,    // Roll Rate Kd
        .i_max = 50.0f,
        .p_max = 500.0f,
        .d_max = 100.0f,
        .low_pass = 0.20f
    };
    
    att_ctrl.rate_pid[1] = (pid_param_t){
        .kp = 0.165f,     // Pitch Rate Kp
        .ki = 0.04f,      // Pitch Rate Ki
        .kd = 0.0005f,    // Pitch Rate Kd
        .i_max = 50.0f,
        .p_max = 500.0f,
        .d_max = 100.0f,
        .low_pass = 0.20f
    };
    
    att_ctrl.rate_pid[2] = (pid_param_t){
        .kp = 0.35f,      // Yaw Rate Kp
        .ki = 0.025f,      // Yaw Rate Ki
        .kd = 0.000f,      // Yaw Rate Kd (暂关, 防噪声放大)
        .i_max = 50.0f,
        .p_max = 500.0f,
        .d_max = 100.0f,
        .low_pass = 0.2f
    };
    
}




// 角度环控制（外环）
void att_2level_ctrl( float dT_s)
{

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

    float roll_sp_rate_ff = att_update_sp_rate_ff(att_2l_ct.exp_rol,
                                                  &att_prev_target_roll,
                                                  &att_roll_sp_rate_ff,
                                                  dT_s);
    float pitch_sp_rate_ff = att_update_sp_rate_ff(att_2l_ct.exp_pit,
                                                   &att_prev_target_pitch,
                                                   &att_pitch_sp_rate_ff,
                                                   dT_s);
    if(att_sp_rate_ff_ready == 0u)
    {
        att_sp_rate_ff_ready = 1u;
    }
    
    // YAW处理
    float set_yaw_av_tmp = vehicle_setpoint.target_yaw_rate;
    set_yaw_av_tmp = LIMIT(set_yaw_av_tmp, -MAX_YAW_SPEED, MAX_YAW_SPEED);   
     // 平滑处理
    att_1l_ct.set_yaw_speed += LIMIT((set_yaw_av_tmp - att_1l_ct.set_yaw_speed), -30.0f, 30.0f);
    att_2l_ct.exp_yaw += att_1l_ct.set_yaw_speed * dT_s;
    
    // 限制±180度
    if(att_2l_ct.exp_yaw < -180.0f) att_2l_ct.exp_yaw += 360.0f;
    else if(att_2l_ct.exp_yaw > 180.0f) att_2l_ct.exp_yaw -= 360.0f;
    
        // 计算YAW误差
    att_2l_ct.yaw_err = att_2l_ct.exp_yaw - att_2l_ct.fb_yaw;
    if(att_2l_ct.yaw_err < -180.0f) att_2l_ct.yaw_err += 360.0f;
    else if(att_2l_ct.yaw_err > 180.0f) att_2l_ct.yaw_err -= 360.0f;
   
    // 角度误差处理
    if(att_2l_ct.yaw_err > 90.0f) 
    {
        if(set_yaw_av_tmp > 0) set_yaw_av_tmp = 0;
    } 
    else if(att_2l_ct.yaw_err < -90.0f) 
    {
        if(set_yaw_av_tmp < 0) set_yaw_av_tmp = 0;
    }
    


    
    // 外环PID计算

    
    att_1l_ct.exp_ang_vel[0] =
        stan_pid_solve(&att_ctrl.angle_pid[0],
                       att_2l_ct.exp_rol - att_2l_ct.fb_rol,
                       dT_s,
                       0) + ATT_SP_RATE_FF_GAIN * roll_sp_rate_ff;
    att_1l_ct.exp_ang_vel[1] =
        stan_pid_solve(&att_ctrl.angle_pid[1],
                       att_2l_ct.exp_pit - att_2l_ct.fb_pit,
                       dT_s,
                       0) + ATT_SP_RATE_FF_GAIN * pitch_sp_rate_ff;
    att_1l_ct.exp_ang_vel[2] = stan_pid_solve(&att_ctrl.angle_pid[2], att_2l_ct.yaw_err, dT_s, 0);
    
    // 限幅
    att_1l_ct.exp_ang_vel[0] = LIMIT(att_1l_ct.exp_ang_vel[0], -MAX_ROLLING_SPEED, MAX_ROLLING_SPEED);
    att_1l_ct.exp_ang_vel[1] = LIMIT(att_1l_ct.exp_ang_vel[1], -MAX_ROLLING_SPEED, MAX_ROLLING_SPEED);
    att_1l_ct.exp_ang_vel[2] = LIMIT(att_1l_ct.exp_ang_vel[2], -MAX_YAW_SPEED, MAX_YAW_SPEED);
}

// 角速率环控制（内环）
void att_1level_ctrl(float dT_s)
{
    if(vehicle_state.armed==0)
    {
        ct_val.rol = 0;
        ct_val.pit = 0;
        ct_val.yaw = 0;
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

    ct_val.yaw = att_rate_pid_solve(&att_ctrl.rate_pid[2],att_1l_ct.exp_ang_vel[2] - att_1l_ct.fb_ang_vel[2],dT_s, enable_rate_i, MAX_YAW_CT_VAL);
    // 输出限幅
    ct_val.rol =LIMIT(ct_val.rol, -MAX_ATT1_VAL, MAX_ATT1_VAL);
    ct_val.pit =  LIMIT(ct_val.pit, -MAX_ATT1_VAL, MAX_ATT1_VAL);
    ct_val.yaw = LIMIT(ct_val.yaw, -MAX_YAW_CT_VAL, MAX_YAW_CT_VAL);
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
