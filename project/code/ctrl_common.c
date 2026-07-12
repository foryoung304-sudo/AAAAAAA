#include "zf_common_headfile.h"

float ctrl_slew_limit(float current,float target,float rate ,float dT_s)
{
    float max_step=rate*dT_s;
    float delta=target-current;
    delta=LIMIT(delta,-max_step,max_step);
    return current+delta;
}

float ctrl_brake_speed(float accel,float distance)
{
    if(accel<=0.0f)
    {
        return 0.0f;
    }
    return sqrtf(2.0f*accel*fabs(distance));
}

float ctrl_smoothstep01(float x)
{
    x=LIMIT(x,0.0f,1.0f);
    return x*x*(3.0f-2.0f*x);

}

float ctrl_ramp_weight(float value, float start ,float end, float min_weight)
{
    float ratio;
    if((end-start)==0.0f)
    {
        return 1.0f;
    }
    ratio=ctrl_smoothstep01((value-start)/(end-start));

    return min_weight+(1.0f-min_weight)*ratio;

}

float ctrl_terminal_clamp(float final_err,float cmd)
{
        if((final_err < 0.0f) && (cmd > 0.0f))
    {
        return 0.0f;
    }

    if((final_err > 0.0f) && (cmd < 0.0f))
    {
        return 0.0f;
    }

    return cmd;
}