#include "mcar_guidance.h"

static int16 mcar_guidance_round_to_i16(float value)
{
    if(value > 32767.0f) return 32767;
    if(value < -32768.0f) return -32768;
    return (int16)(value >= 0.0f ? value + 0.5f : value - 0.5f);
}

static float mcar_guidance_wrap_180(float angle_deg)
{
    while(angle_deg > 180.0f) angle_deg -= 360.0f;
    while(angle_deg < -180.0f) angle_deg += 360.0f;
    return angle_deg;
}

mcar_guidance_t mcar_guidance_calculate(float beacon_body_x,
                                        float beacon_body_y,
                                        float mcar_body_x,
                                        float mcar_body_y,
                                        float mcar_head_body_x,
                                        float mcar_head_body_y,
                                        float drone_yaw_earth_deg)
{
    mcar_guidance_t result = {0};
    float head_norm = sqrtf(mcar_head_body_x * mcar_head_body_x
                          + mcar_head_body_y * mcar_head_body_y);

    if(head_norm < 0.001f)
    {
        return result;
    }

    float head_x = mcar_head_body_x / head_norm;
    float head_y = mcar_head_body_y / head_norm;
    float rel_x = beacon_body_x - mcar_body_x;
    float rel_y = beacon_body_y - mcar_body_y;
    float err_forward = rel_x * head_x + rel_y * head_y;
    float err_right = -rel_x * head_y + rel_y * head_x;
    float mcar_yaw_body_deg = atan2f(head_y, head_x) * 57.2957795f;
    float mcar_yaw_earth_deg = mcar_guidance_wrap_180(drone_yaw_earth_deg
                                                    + mcar_yaw_body_deg);

    result.err_forward_px = mcar_guidance_round_to_i16(err_forward);
    result.err_right_px = mcar_guidance_round_to_i16(err_right);
    result.mcar_yaw_earth_cdeg = mcar_guidance_round_to_i16(mcar_yaw_earth_deg * 100.0f);
    result.valid = 1u;
    return result;
}
