/*
 * PID.c
 *
 *  Created on: 2025年10月4日
 *      Author: lenovo
 */

#include "zf_common_headfile.h"


/*常规pid
 *
 */

float stan_pid_solve(pid_param_t *pid, float error, float dt, uint8_t is_saturated)
{
    if (dt <= 0.0f) dt = 0.001f;

    pid->out_p = pid->kp * error;

    // 动态抗积分饱和
    if (!(is_saturated && (error * pid->out_i > 0.0f))) {
        pid->out_i += pid->ki * error * dt;
        pid->out_i = CLAMP(pid->out_i, -pid->i_max, pid->i_max);
    }

    // 真实的微分
    float derivative = (error - pid->pre_error) / dt;
    float d_raw = pid->kd * derivative;
    pid->out_d = pid->out_d * (1.0f - pid->low_pass) + d_raw * pid->low_pass;
    pid->out_d = CLAMP(pid->out_d, -pid->d_max, pid->d_max);

    pid->pre_error = error;

    float output = pid->out_p + pid->out_i + pid->out_d;
    return CLAMP(output, -pid->p_max, pid->p_max);
}

float pid_solve_limited_i(pid_param_t *pid,
                          float error,
                          float dt,
                          uint8_t enable_i,
                          float output_limit,
                          float i_decay)
{
    float old_i = pid->out_i;
    float output = stan_pid_solve(pid, error, dt, 0);

    if(output_limit < 0.0f)
    {
        output_limit = -output_limit;
    }

    if(enable_i == 0)
    {
        pid->out_i = old_i * CLAMP(i_decay, 0.0f, 1.0f);
        output = pid->out_p + pid->out_i + pid->out_d;
    }
    else if((output > output_limit && error > 0.0f) ||
            (output < -output_limit && error < 0.0f))
    {
        pid->out_i = old_i;
        output = pid->out_p + pid->out_i + pid->out_d;
    }

    return CLAMP(output, -output_limit, output_limit);
}

// PID积分清空函数
void stan_pid_reset(pid_param_t *pid)
{
    pid->out_i = 0;
    pid->out_d = 0;
    pid->error = 0;
    pid->pre_error = 0;
    pid->pp_error = 0;
}


/*增量式
 *
 */

float inc_pid_solve(pid_param_t *pid,float error)
{
    pid->out_d=CLAMP(pid->kd*(error-2*pid->pre_error+pid->pp_error),-pid->d_max,pid->d_max);
    pid->out_p=CLAMP(pid->kp*(error-pid->pre_error),-pid->p_max,pid->p_max);
    pid->out_i=CLAMP(pid->ki*error,-pid->i_max,pid->i_max);

    pid->pp_error=pid->pre_error;
    pid->pre_error=error;

    return pid->out_p+pid->out_i+pid->out_d;

}


/*变积分
 *
 */

float change_kib=4;

float changable_pid_solve(pid_param_t *pid,float error)
{
    // d²e/dt² ≈ (e[k] - 2*e[k-1] + e[k-2])/T²，这里假设T=1
    pid->out_d=pid->kd*(error-2*pid->pre_error+pid->pp_error);
    pid->out_p=pid->kp*(error-pid->pre_error);

    float ki_index=pid->ki;

    if(error+pid->pre_error>0)
    {
        //动态调整ki，error越大，ki越小
        ki_index=(pid->ki)-(pid->ki)/(1+exp(change_kib-0.2*fabs(error)));
    }

    pid->out_i=ki_index*error;

    pid->pp_error=pid->pre_error;
    pid->pre_error=error;

    return CLAMP(pid->out_p, -pid->p_max, pid->p_max)
            + CLAMP(pid->out_i, -pid->i_max, pid->i_max)
            + CLAMP(pid->out_d, -pid->d_max, pid->d_max);

}



/*Bang-Bang与PID结合
 *
 */
float bangbang_out=0;

void bangbang_pid_solve(pid_param_t *pid,float error)
{
    float BangBang_output=15000,BangBang_error=8;
    pid->error=error;

    if(error>BangBang_error || error<-BangBang_error)
    {
        bangbang_out=(error>0)?BangBang_output:(-BangBang_output);
    }
    else
    {
        pid->out_d=pid->kd*(error-2*pid->pre_error+pid->pp_error);
        pid->out_p=pid->kp*(error-pid->pre_error);
        pid->out_i=pid->ki*error;

        bangbang_out=CLAMP(pid->out_p, -pid->p_max, pid->p_max)
                    + CLAMP(pid->out_i, -pid->i_max, pid->i_max)
                    + CLAMP(pid->out_d, -pid->d_max, pid->d_max);
    }

    pid->pp_error=pid->pre_error;
    pid->pre_error=error;
}


/*
 * 积分分离
 */
float i_separation_pid_solve(pid_param_t *pid, float error)
{
    // 死区：小误差时不控制，避免抖动
    if (fabs(error) < 3.0f) {
        return 0.0f;
    }


    pid->out_p = pid->kp * error;


    if (fabs(error) < 20.0f) {
        pid->out_i += pid->ki * error;
        pid->out_i = CLAMP(pid->out_i, -pid->i_max, pid->i_max);
    }
    else
    {
        pid->out_i *= 0.95f;  // 大误差时衰减积分
    }

    // 微分项低通滤波
    float derivative = error - pid->pre_error;
    pid->out_d = pid->out_d * 0.6f + derivative * pid->kd * 0.4f;
    pid->out_d = CLAMP(pid->out_d, -pid->d_max, pid->d_max);

    pid->pre_error = error;

    float output = pid->out_p + pid->out_i + pid->out_d;
    output = CLAMP(output, -pid->p_max, pid->p_max);

    return output;
}

/**
 * 抗积分饱和PID控制器
 * 通过积分限幅和积分分离防止积分饱和
 */
float anti_windup_pid_solve(pid_param_t *pid, float error,float dt,uint8_t is_saturated)
{
    float output;

    pid->out_p = pid->kp * error;

    if(!is_saturated && error * pid->out_i>0.0f)
    {
        pid->out_i += pid->ki * error;

        pid->out_i=CLAMP(pid->out_i,-pid->i_max,pid->i_max);
    }
    

    pid->out_d = pid->kd * (error - pid->pre_error)/dt;

    output = pid->out_p + pid->out_i + pid->out_d;
    output=CLAMP(output,-pid->p_max,pid->p_max);

    pid->pre_error = error;

    return output;
}


/**
 * 微分先行PID控制器
 * 只对被控量进行微分，不对设定值进行微分
 */
float derivative_first_pid_solve(pid_param_t *pid, float setpoint, float feedback)
{
    float error = setpoint - feedback;
    float output;

    pid->out_p = pid->kp * error;

    pid->out_i += pid->ki * error;
    pid->out_i=CLAMP(pid->out_i,-pid->i_max,pid->i_max);

    pid->out_d = pid->kd * (pid->pre_error - feedback);

    output = pid->out_p + pid->out_i + pid->out_d;

    output=CLAMP(output,-pid->p_max,pid->p_max);

    pid->pre_error = feedback;

    return output;
}




/**
 * 带死区的PID控制器
 * 在误差较小时不进行调节，减少系统抖动
 */
float dead_zone_pid_solve(pid_param_t *pid, float error)
{
    float dead_zone = 5.0f;  // 死区阈值
    float output;

    if (fabs(error) < dead_zone) {

        return pid->out_p + pid->out_i + pid->out_d;
    }

    pid->out_p = pid->kp * error;
    pid->out_i += pid->ki * error;

    pid->out_i=CLAMP(pid->out_i,-pid->i_max,pid->i_max);

    pid->out_d = pid->kd * (error - pid->pre_error);

    output = pid->out_p + pid->out_i + pid->out_d;
    output=CLAMP(output,-pid->p_max,pid->p_max);

    pid->pre_error = error;

    return output;
}




/**
 * 自适应PID控制器
 * 根据误差大小自动调整PID参数
 */
float adaptive_pid_solve(pid_param_t *pid, float error)
{
    float abs_error = fabs(error);


    float original_kp = pid->kp;
    float original_ki = pid->ki;
    float original_kd = pid->kd;

    if (abs_error > 100.0f)
    {

        pid->kp *= 1.3f;
        pid->kd *= 0.7f;
    }
    else if(abs_error < 20.0f)
    {

        pid->ki *= 1.5f;
    }

    pid->out_d = pid->kd * (error - 2 * pid->pre_error + pid->pp_error);
    pid->out_p = pid->kp * (error - pid->pre_error);
    pid->out_i = pid->ki * error;

    float output = pid->out_p + pid->out_i + pid->out_d;
    output = CLAMP(output, -pid->p_max, pid->p_max);

    pid->kp = original_kp;
    pid->ki = original_ki;
    pid->kd = original_kd;

    pid->pp_error = pid->pre_error;
    pid->pre_error = error;

    return output;
}


/**
 * 专家PID控制器
 * 根据误差和误差变化率自动调整PID参数
 */
float expert_pid_solve(pid_param_t *pid, float error)
{
    float ec = error - pid->pre_error;  // 误差变化率
    float abs_error = fabs(error);
    float abs_ec = fabs(ec);

    float original_kp = pid->kp;
    float original_ki = pid->ki;
    float original_kd = pid->kd;

    if (abs_error > 80.0f)
    {
        // 大误差情况
        if (abs_ec > 20.0f)
        {
            // 误差和误差变化率都大，减小比例和积分，增大微分
            pid->kp *= 0.8f;
            pid->ki *= 0.5f;
            pid->kd *= 1.5f;
        }
        else
        {
            // 误差大但变化率小，增大比例，减小微分
            pid->kp *= 1.2f;
            pid->kd *= 0.8f;
        }
    }
    else if (abs_error < 30.0f)
    {
        // 小误差情况
        if (abs_ec > 10.0f) {
            // 误差小但变化率大，减小比例和积分
            pid->kp *= 0.9f;
            pid->ki *= 0.8f;
        }
        else
        {
            // 误差和变化率都小，增大积分，减小微分
            pid->ki *= 1.2f;
            pid->kd *= 0.7f;
        }
    }

    // 增量式PID计算
    pid->out_d = pid->kd * (error - 2 * pid->pre_error + pid->pp_error);
    pid->out_p = pid->kp * (error - pid->pre_error);
    pid->out_i = pid->ki * error;

    float output = pid->out_p + pid->out_i + pid->out_d;
    output = CLAMP(output, -pid->p_max, pid->p_max);

    // 恢复原始参数
    pid->kp = original_kp;
    pid->ki = original_ki;
    pid->kd = original_kd;

    // 更新历史值
    pid->pp_error = pid->pre_error;
    pid->pre_error = error;

    return output;
}


//--------------------------------多变量模糊pid----------------------------------//

const FuzzySet Kp_Rule[FUZZY_SET_NUM][FUZZY_SET_NUM]=
{
        {NB, NB, NM, NM, NM, ZO, ZO},  // e_delta=NB
        {NB, NM, NM, NM, NS, ZO, PS},  // e_delta=NM
        {NM, NM, NS, NS, ZO, PS, PS},  // e_delta=NS
        {NM, NM, NS, ZO, PS, PM, PM},  // e_delta=ZO
        {NS, NS, ZO, PS, PM, PM, PM},  // e_delta=PS
        {NS, ZO, PS, PS, PM, PB, PB},  // e_delta=PM
        {ZO, ZO, PS, PS, PM, PB, PB}   // e_delta=PB
};


const FuzzySet Kd_Rule[FUZZY_SET_NUM][FUZZY_SET_NUM]=
{
    {NB, NB, NM, NM, NM, ZO, ZO},  // e_delta=NB
    {NB, NM, NM, NM, NS, ZO, PS},  // e_delta=NM
    {NM, NM, NS, NS, ZO, PS, PS},  // e_delta=NS
    {NM, NM, NS, ZO, PS, PM, PM},  // e_delta=ZO
    {NS, NS, ZO, PS, PM, PM, PM},  // e_delta=PS
    {NS, ZO, PS, PS, PM, PB, PB},  // e_delta=PM
    {ZO, ZO, PS, PS, PM, PB, PB}   // e_delta=PB
};

/*
 * 三角形隶属度函数
 *
 * 隶属度
  1 |      /\
    |     /  \
    |    /    \
  0 |---/------\---
    -1  0   1   2   3
         论域值
 */
float fuzzy_membership(float norm_val,FuzzySet f_set)
{
    float center=(float)f_set-3.0f;
    float membership=0.0f;

    if(norm_val>=(center-1.0f) && norm_val<=center)
    {
        membership=norm_val-(center-1.0f);
    }
    else if(norm_val>center && norm_val<=(center+1.0f))
    {
        membership=(center+1.0f)-norm_val;
    }

    return CLAMP(membership,0.0f,1.0f);
}


/*
 * 模糊集合映射
 *
 * 隶属度
  1 |      /\           /\
    |     /  \         /  \
    |    /    \       /    \
  0.7|---|----*\-----/------\--  ← 输入值0.7
    |   |     | \   /        \
  0.3|   |     |  \/          \
    |   |     |  /\           |\
  0 |___|_____|_/_\__________|_\____
   -3  -2   -1   0   1   2   3
              ↑
            最佳匹配(ZO)
 */

FuzzySet norm_val_to_fuzzy(float norm_val)
{
    float max_mem=0.0f;
    FuzzySet best_set=ZO;

    for(int i=NB;i<=PB;i++)
    {
        float mem=fuzzy_membership(norm_val,(FuzzySet)i);
        if(mem>max_mem)
        {
            max_mem=mem;
            best_set=(FuzzySet)i;
        }
    }
    return best_set;
}

/*
 * 去模糊
 */
float defuzzy(FuzzySet f_set)
{
    float domain_vals[FUZZY_SET_NUM]={-3.0f, -2.0f, -1.0f, 0.0f, 1.0f, 2.0f, 3.0f};
    return domain_vals[f_set];
}

/*多变量协同模糊pid
 * 输入参数：
//   pid:        PID参数结构体指针（你的定义）
//   e_delta:    舵机摆角误差
//   ec_delta:   舵机摆角误差变化
//   e_v:        电机速度误差
//   ec_v:       电机速度误差变化
//   k:          线性调整比例系数
//   a:          线性调整偏移系数
 */

float multi_fuzzy_pid_solve(pid_param_t *pid,float e_delta,
                            float ec_delte,float e_v,float ec_v,
                            float k,float a)
{
    if(pid==NULL)return 0;

    //误差归一化
    float e_v_norm=CLAMP(e_v/NORM_STANDARD,FUZZY_DOMAIN_MIN,FUZZY_DOMAIN_MAX);
    float e_delta_norm=CLAMP(e_delta,FUZZY_DOMAIN_MIN,FUZZY_DOMAIN_MAX);

    //模糊化
    FuzzySet f_e_delta =norm_val_to_fuzzy(e_delta_norm);
    FuzzySet f_e_v=norm_val_to_fuzzy(e_v_norm);

    //模糊推理
    FuzzySet f_kp=Kp_Rule[f_e_delta][f_e_v];
    FuzzySet f_kd=Kd_Rule[f_e_delta][f_e_v];
    pid->ki=0.1f;

    //去模糊
    pid->kp=defuzzy(f_kp)*0.5f;
    pid->kd=defuzzy(f_kd)*0.2f;

    //pid计算
    float ec=pid->error-pid->pre_error;

    pid->out_p=pid->kp*pid->error;
    pid->out_p=CLAMP(pid->out_p,-pid->p_max,pid->p_max);

    pid->out_i+=pid->ki*pid->error;
    pid->out_i=CLAMP(pid->out_i,-pid->i_max,pid->i_max);

    pid->out_d=(1-pid->low_pass)*pid->kd*ec+pid->low_pass*pid->out_d;
    pid->out_d=CLAMP(pid->out_d,-pid->d_max,pid->d_max);

    //输出参数线性调整
    //out_adjust=k*pid_out+a   k,a根据硬件特性更改
    float out_p_adjust=k*pid->out_p+a;
    float out_i_adjust=k*pid->out_i+a;
    float out_d_adjust=k*pid->out_d+a;

    float pid_out=out_p_adjust+out_i_adjust+out_d_adjust;

    pid->pp_error=pid->pre_error;
    pid->pre_error=pid->error;

    return pid_out;

}


//-------------------------------多变量协同模糊pid-------------------------------------------------//
