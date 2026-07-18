#ifndef __VISION_NAV_H__
#define __VISION_NAV_H__

#include "zf_common_headfile.h"

#define VISION_NAV_ENABLE_EKF_UPDATE 0
#define VISION_NAV_BEACON_COUNT 3
#define VISION_NAV_MIN_HEIGHT_CM 30.0f
#define VISION_NAV_MAX_HEIGHT_CM 180.0f
#define VISION_NAV_MAX_ATT_DEG 10.0f
#define VISION_NAV_MAX_RESIDUAL_CM 120.0f
#define VISION_NAV_MIN_STABLE_FRAMES 3
#define VISION_NAV_PRINT_INTERVAL_US 100000u

typedef struct {
    float x_cm;
    float y_cm;
} vision_nav_point_t;

typedef struct {
    uint8_t valid;
    uint8_t used_for_ekf;
    uint8_t stable_frames;
    uint8_t beacon_id;
    float drone_x_cm;
    float drone_y_cm;
    float residual_x_cm;
    float residual_y_cm;
    float confidence;
} vision_nav_obs_t;

extern vision_nav_obs_t vision_nav_obs;
extern uint8_t vision_nav_current_beacon_id;

void vision_nav_init(void);
void vision_nav_reset(void);
void vision_nav_set_current_beacon(uint8_t beacon_id);
void vision_nav_next_beacon(void);
void vision_nav_update(void);

#endif
