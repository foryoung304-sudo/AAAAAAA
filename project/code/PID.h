/*
 * PID.h
 *
 *  Created on: 2025��10��4��
 *      Author: lenovo
 */

#ifndef _PID_H_
#define _PID_H_

#include "zf_common_headfile.h"

#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#define CLAMP(input, low, upper) MIN(MAX(input, low), upper)

typedef struct
{
        float kp;
        float ki;
        float kd;
        float i_max;
        float p_max;
        float d_max;
        float low_pass;
        float out_p;
        float out_i;
        float out_d;
        float error;
        float pre_error;
        float pp_error;


}pid_param_t;

#define PID_CREATE(_kp,_ki,_kd,_low_pass,max_p,max_i,max_d) \
        {                                    \
            .kp = _kp,                       \
            .ki = _ki,                       \
            .kd = _kd,                       \
            .low_pass = _low_pass,           \
            .out_p = 0,                      \
            .out_i = 0,                      \
            .out_d = 0,                      \
            .p_max = max_p,                  \
            .i_max = max_i,                  \
            .d_max = max_d,                  \
            .error = 0,                      \
            .pre_error = 0,                  \
            .pp_error = 0                    \
        }

//-------------�����ģ��pid����----------------------------//
#define NORM_STANDARD                   100.0f              //
#define FUZZY_DOMAIN_MIN                -3.0f
#define FUZZY_DOMAIN_MAX                3.0f
#define FUZZY_SET_NUM                   7

typedef enum
{
    NB=0,
    NM,
    NS,
    ZO,
    PS,
    PM,
    PB
}FuzzySet;

extern const FuzzySet Kp_Rule[FUZZY_SET_NUM][FUZZY_SET_NUM];
extern const FuzzySet Kd_Rule[FUZZY_SET_NUM][FUZZY_SET_NUM];

float stan_pid_solve(pid_param_t *pid, float error, float dt, uint8_t is_saturated);
float pid_solve_limited_i(pid_param_t *pid,
                          float error,
                          float dt,
                          uint8_t enable_i,
                          float output_limit,
                          float i_decay);
float inc_pid_solve(pid_param_t *pid,float error);
float changable_pid_solve(pid_param_t *pid,float error);
void bangbang_pid_solve(pid_param_t *pid,float error);
float i_separation_pid_solve(pid_param_t *pid,float error);
float anti_windup_pid_solve(pid_param_t *pid, float error,float dt,uint8_t is_saturated);


void stan_pid_reset(pid_param_t *pid);
float derivative_first_pid_solve(pid_param_t *pid, float setpoint, float feedback);
float dead_zone_pid_solve(pid_param_t *pid, float error) ;
float adaptive_pid_solve(pid_param_t *pid, float error);
float expert_pid_solve(pid_param_t *pid, float error);
float fuzzy_membership(float norm_val,FuzzySet f_set);
FuzzySet norm_val_to_fuzzy(float norm_val);
float defuzzy(FuzzySet f_set);
float multi_fuzzy_pid_solve(pid_param_t *pid,float e_delta,
                            float ec_delte,float e_v,float ec_v,
                            float k,float a);



#endif /* CODE_PID_H_ */
