#ifndef __ATT_CTRL_H__
#define __ATT_CTRL_H__   

#include  "zf_common_headfile.h"

// 控制限制参数

#define MAX_ROLLING_SPEED 50.0f
#define MAX_ANGLE     30.0f        // 最大角度限制
#define MAX_YAW_SPEED 5.0f       // 最大偏航速度
#define MAX_ATT1_VAL   10.0f        // 角速率环最大输出 (°/s)
#define MAX_YAW_CT_VAL 7.0f        // 偏航控制输出最大
  
// 姿态控制结构体
typedef struct {
    // 角度环PID控制器（外环）
    pid_param_t angle_pid[3];  // Roll, Pitch, Yaw
    
    // 角速率环PID控制器（内环）
    pid_param_t rate_pid[3];   // Roll Rate, Pitch Rate, Yaw Rate
    
} attitude_control_t;

extern attitude_control_t att_ctrl;// 角度环数据结构
typedef struct 
{
    float exp_rol;
    float exp_pit;
    float exp_yaw;
    float fb_rol;
    float fb_pit;
    float fb_yaw;
    float yaw_err;
    float exp_rol_adj;
    float exp_pit_adj;
    
} att_2l_ct_t;

// 角速率环数据结构
typedef struct 
{
    float exp_ang_vel[3];
    float fb_ang_vel[3];
    float set_yaw_speed;
} att_1l_ct_t;

extern att_2l_ct_t att_2l_ct;
extern att_1l_ct_t att_1l_ct;

// 控制输出
typedef struct 
{
    float rol;
    float pit;
    float yaw;
} ct_val_t;

extern ct_val_t ct_val;
// 函数声明

// 初始化
void att_ctrl_init(void);


// 角度环控制（外环）
void att_2level_ctrl(float dT_s);

// 角速率环控制（内环）
void att_1level_ctrl(float dT_s);



#endif
