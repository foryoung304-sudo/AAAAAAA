#include "zf_common_headfile.h"
float ctrl_slew_limit(float current,float target,float rate ,float dT_s);
float ctrl_brake_speed(float accel, float distance);
float ctrl_smoothstep01(float x);
float ctrl_ramp_weight(float value, float start, float end, float min_weight);
float ctrl_terminal_clamp(float final_err,float cmd);