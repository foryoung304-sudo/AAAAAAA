#include "zf_common_headfile.h"

// 控制限制参数



motor_out_t motor_out = {0};
static float throttle_ramped = 0.0f;

float throttle_ramped_debug=0.0f;
float motor_mix_debug_throttle = 0.0f;
float motor_mix_debug_roll = 0.0f;
float motor_mix_debug_pitch = 0.0f;
float motor_mix_debug_yaw = 0.0f;
float motor_mix_debug_raw_avg = 0.0f;
float motor_mix_debug_shift_up = 0.0f;
float motor_mix_debug_shift_down = 0.0f;
float motor_mix_debug_final_avg = 0.0f;


static void throttle_ramp_reset(void)
{
    throttle_ramped = 0.0f;
}

static float throttle_ramp_update(float target_throttle)
{
    float target = LIMIT(target_throttle, 0.0f, MAX_CT_VAL);
    float diff = target - throttle_ramped;
    float step ;
    if(diff<0)
    {
        step=THROTTLE_RAMP_DOWN_STEP;
    }
    else
    {
        step=(throttle_ramped>=THROTTLE_SPOOL_DONE)?THROTTLE_RAMP_FLIGHT_STEP:THROTTLE_RAMP_SPOOL_STEP;
    }
    
    diff=LIMIT(diff,-step,step);

    throttle_ramped += diff;
    return throttle_ramped;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     核心四轴电机混控与电调输出
// 备注信息     将姿态环/高度环结果按 X 型四旋翼物理模型融合，并量程映射后下发给电调
//-------------------------------------------------------------------------------------------------------------------
void motor_mixing_output(void)
{
    motor_mix_cnt++;
    // 1. 终极安全防线：未解锁状态，强制切断动力，发送 0x03 急停！
    if(vehicle_state.armed == 0)
    {
         small_driver_set_duty(0, 0, 0, 0);
        throttle_ramp_reset();
        motor_out.m1 = 0;
        motor_out.m2 = 0;   
        motor_out.m3 = 0;
        motor_out.m4 = 0;
         return;
    
    }

    //  获取各个控制环的输出值
    float throttle = throttle_ramp_update(vehicle_setpoint.target_throttle); // 仅缓变基础油门，姿态修正保持实时
        throttle = LIMIT(throttle, 0.0f, THROTTLE_MAX);  
        throttle_ramped_debug=throttle; 
    float roll     = ct_val.rol * ROLL_MIX_DIR;
    float pitch    = ct_val.pit * PITCH_MIX_DIR;
    float yaw      = ct_val.yaw * YAW_MIX_DIR;

    float att_scale=1.0f;
    if(throttle<THROTTLE_SPOOL_DONE)
    {
        att_scale=throttle / THROTTLE_SPOOL_DONE;
    }
    else if(vehicle_state.current_height < ATT_TAKEOFF_SCALE_FULL_CM)
    {
        float height_ratio =
            (vehicle_state.current_height - ATT_TAKEOFF_SCALE_START_CM) /
            (ATT_TAKEOFF_SCALE_FULL_CM - ATT_TAKEOFF_SCALE_START_CM);
        height_ratio = LIMIT(height_ratio, 0.0f, 1.0f);
        att_scale = ATT_TAKEOFF_SCALE_MIN +
            (1.0f - ATT_TAKEOFF_SCALE_MIN) * height_ratio;
    }

    roll *= att_scale;
    pitch *= att_scale;
    yaw *= att_scale*att_scale;
    motor_mix_debug_throttle = throttle;
    motor_mix_debug_roll = roll;
    motor_mix_debug_pitch = pitch;
    motor_mix_debug_yaw = yaw;
    motor_mix_debug_raw_avg = throttle;
    motor_mix_debug_shift_up = 0.0f;
    motor_mix_debug_shift_down = 0.0f;
    motor_mix_debug_final_avg = throttle;

    if(fabsf(throttle) < MOTOR_OUTPUT_DEADZONE)
    {
        motor_out.m1 = 0;
        motor_out.m2 = 0;   
        motor_out.m3 = 0;
        motor_out.m4 = 0;
    }
    else
    {// 四轴混控（X型）
        if(vision_nav_yaw_spin_is_active() != 0u)
        {
            float base1 = throttle + pitch + roll;
            float base2 = throttle + pitch - roll;
            float base3 = throttle - pitch + roll;
            float base4 = throttle - pitch - roll;
            float base_min = MIN(MIN(base1, base2), MIN(base3, base4));
            float base_max = MAX(MAX(base1, base2), MAX(base3, base4));
            float yaw_headroom = MIN(base_min, MAX_CT_VAL - base_max);
            yaw_headroom = MAX(yaw_headroom, 0.0f);
            yaw = LIMIT(yaw, -yaw_headroom, yaw_headroom);
            motor_mix_debug_yaw = yaw;
        }

        // M1: 左前 CW, M2: 右前 CCW, M3: 左后 CCW, M4: 右后 CW
        float m1 = throttle +pitch + roll - yaw;
        float m2 = throttle + pitch - roll + yaw;
        float m3 = throttle - pitch + roll + yaw;
        float m4 = throttle - pitch - roll - yaw;
        motor_mix_debug_raw_avg = (m1 + m2 + m3 + m4) * 0.25f;

        // 整体平移限幅：优先保留电机之间的姿态差动，避免单电机限幅吃掉回正力矩。
        float mix_min = m1;
        float mix_max = m1;
        mix_min = MIN(mix_min, m2);
        mix_min = MIN(mix_min, m3);
        mix_min = MIN(mix_min, m4);
        mix_max = MAX(mix_max, m2);
        mix_max = MAX(mix_max, m3);
        mix_max = MAX(mix_max, m4);

        if(mix_min < 0.0f)
        {
            float shift = -mix_min;
            motor_mix_debug_shift_up = shift;
            m1 += shift;
            m2 += shift;
            m3 += shift;
            m4 += shift;
            mix_max += shift;
        }

        if(mix_max > MAX_CT_VAL)
        {
            float shift = mix_max - MAX_CT_VAL;
            motor_mix_debug_shift_down = shift;
            m1 -= shift;
            m2 -= shift;
            m3 -= shift;
            m4 -= shift;
        }

        m1 = LIMIT(m1, 0, MAX_CT_VAL);
        m2 = LIMIT(m2, 0, MAX_CT_VAL);
        m3 = LIMIT(m3, 0, MAX_CT_VAL);
        m4 = LIMIT(m4, 0, MAX_CT_VAL);
        motor_mix_debug_final_avg = (m1 + m2 + m3 + m4) * 0.25f;


        float scale = (MOTOR_DUTY_MAX - MOTOR_DUTY_MIN_START) / MAX_CT_VAL;
        
        // 比例缩放：把控制量映射到电调占空比范围。
        motor_out.m1 =(int16_t)LIMIT(((m1-MOTOR_OUTPUT_DEADZONE) *  scale+ MOTOR_DUTY_MIN_START),MOTOR_DUTY_MIN_START, MOTOR_DUTY_MAX); 
        motor_out.m2 =(int16_t)LIMIT(((m2-MOTOR_OUTPUT_DEADZONE) *  scale+ MOTOR_DUTY_MIN_START),MOTOR_DUTY_MIN_START, MOTOR_DUTY_MAX);
        motor_out.m3 =(int16_t)LIMIT(((m3-MOTOR_OUTPUT_DEADZONE) *  scale+ MOTOR_DUTY_MIN_START),MOTOR_DUTY_MIN_START, MOTOR_DUTY_MAX);
        motor_out.m4 =(int16_t)LIMIT(((m4-MOTOR_OUTPUT_DEADZONE) *  scale+ MOTOR_DUTY_MIN_START),MOTOR_DUTY_MIN_START, MOTOR_DUTY_MAX);
    
    }
  
    small_driver_set_duty(motor_out.m1, motor_out.m2, motor_out.m3, motor_out.m4);
    
}

