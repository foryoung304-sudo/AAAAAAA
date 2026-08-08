#ifndef __FLY_CTRL_H__
#define __FLY_CTRL_H__

#include "zf_common_headfile.h"



// 控制限制参数
#define MAX_CT_VAL    50.0f       // 最大控制输出

#define MOTOR_DUTY_MIN_START  900
#define MOTOR_DUTY_MAX        7500
#define MOTOR_OUTPUT_DEADZONE 0.1f
#define THROTTLE_MAX          45.0f
#define ROLL_MIX_DIR          (1.0f)
#define PITCH_MIX_DIR         (1.0f)
#define YAW_MIX_DIR           (1.0f)
#define THROTTLE_RAMP_SPOOL_STEP   0.05f
#define THROTTLE_RAMP_FLIGHT_STEP  0.12f
#define THROTTLE_RAMP_DOWN_STEP    0.33f
#define THROTTLE_SPOOL_DONE        20.0f
#define ATT_TAKEOFF_SCALE_START_CM 8.0f
#define ATT_TAKEOFF_SCALE_FULL_CM  13.0f
#define ATT_TAKEOFF_SCALE_MIN      0.65f

extern float throttle_ramped_debug;
extern float motor_mix_debug_throttle;
extern float motor_mix_debug_roll;
extern float motor_mix_debug_pitch;
extern float motor_mix_debug_yaw;
extern float motor_mix_debug_raw_avg;
extern float motor_mix_debug_shift_up;
extern float motor_mix_debug_shift_down;
extern float motor_mix_debug_final_avg;

typedef struct 
{
    int16_t m1;
    int16_t m2;
    int16_t m3;
    int16_t m4;
} motor_out_t;

extern motor_out_t motor_out;

void motor_mixing_output(void);



#endif /* __FLY_CTRL_H__ */
