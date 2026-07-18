#ifndef MCAR_GUIDANCE_H_
#define MCAR_GUIDANCE_H_

#include "zf_common_headfile.h"

typedef struct
{
    int16 err_forward_px;
    int16 err_right_px;
    int16 mcar_yaw_earth_cdeg;
    uint8 valid;
} mcar_guidance_t;

mcar_guidance_t mcar_guidance_calculate(float beacon_body_x,
                                        float beacon_body_y,
                                        float mcar_body_x,
                                        float mcar_body_y,
                                        float mcar_head_body_x,
                                        float mcar_head_body_y,
                                        float drone_yaw_earth_deg);

#endif
