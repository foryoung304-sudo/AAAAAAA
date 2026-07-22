#ifndef __VISION_NAV_H__
#define __VISION_NAV_H__

#include "zf_common_headfile.h"

#define VISION_NAV_ENABLE_EKF_UPDATE          0
#define VISION_NAV_MIN_HEIGHT_CM              30.0f
#define VISION_NAV_MAX_HEIGHT_CM              180.0f
#define VISION_NAV_MAX_ATT_DEG                10.0f
#define VISION_NAV_MAX_RESIDUAL_CM            80.0f
#define VISION_NAV_MAX_EKF_CORRECTION_CM      1.0f
#define VISION_NAV_MIN_STABLE_FRAMES          3u
#define VISION_NAV_ANCHOR_STABLE_CM           20.0f
#define VISION_NAV_TARGET_SWITCH_CM           60.0f
#define VISION_NAV_FUSION_INTERVAL_US         100000u
#define VISION_NAV_BEACON_OFF_CONFIRM_US      400000u
#define VISION_NAV_TARGET_LOST_RESET_US       1000000u
#define VISION_NAV_MCAR_FEEDBACK_TIMEOUT_US   500000u
#define VISION_NAV_MCAR_ARRIVE_ERR_PX         12.0f
#define VISION_NAV_PRINT_INTERVAL_US          100000u
#define VISION_NAV_FAST_SEARCH_ENABLE                1
#define VISION_NAV_SEARCH_CENTER_X_CM            100.0f
#define VISION_NAV_SEARCH_CENTER_Y_CM            150.0f
#define VISION_NAV_SEARCH_SPEED_LIMIT_CM_S         30.0f
#define VISION_NAV_SEARCH_MIN_HEIGHT_CM           65.0f
#define VISION_NAV_SEARCH_LOST_FRAMES               3u
#define VISION_NAV_SEARCH_REACQUIRE_FRAMES          2u
#define VISION_NAV_APPROACH_SPEED_LIMIT_CM_S        30.0f
#define VISION_NAV_APPROACH_MIN_HEIGHT_CM           65.0f
#define VISION_NAV_APPROACH_ACQUIRE_FRAMES            3u
#define VISION_NAV_APPROACH_LOST_FRAMES               3u
#define VISION_NAV_APPROACH_STOP_RADIUS_CM          15.0f
#define VISION_NAV_APPROACH_TARGET_STEP_CM           8.0f
#define VISION_NAV_CAR_FOLLOW_START_CM               70.0f
#define VISION_NAV_CAR_FOLLOW_STOP_CM                50.0f
#define VISION_NAV_CAR_FOLLOW_TARGET_STEP_CM          8.0f
#define VISION_NAV_CAR_FOLLOW_SPEED_LIMIT_CM_S       22.0f

typedef enum
{
    VISION_NAV_SEARCH = 0,
    VISION_NAV_TRACKING,
    VISION_NAV_CAR_ARRIVED,
    VISION_NAV_DONE
} vision_nav_state_t;

typedef struct
{
    uint8_t valid;
    uint8_t used_for_ekf;
    uint8_t stable_frames;
    uint8_t target_seq;
    uint8_t state;
    uint8_t anchor_valid;
    uint8_t car_arrived;
    uint8_t target_done;
    uint8_t geometry_ok;
    uint8_t feedback_fresh;
    uint8_t search_move_active;
    uint8_t search_lost_frames;
    uint8_t search_found_frames;
    uint8_t approach_active;
    uint8_t approach_arrived;
    uint8_t approach_lost_frames;
    uint8_t approach_found_frames;
    uint8_t car_follow_active;
    float car_distance_cm;
    float drone_x_cm;
    float drone_y_cm;
    float rel_earth_x_cm;
    float rel_earth_y_cm;
    float observed_beacon_x_cm;
    float observed_beacon_y_cm;
    float residual_x_cm;
    float residual_y_cm;
    float confidence;
    float ekf_correction_x_cm;
    float ekf_correction_y_cm;
    float anchor_x_cm;
    float anchor_y_cm;
    float pending_anchor_x_cm;
    float pending_anchor_y_cm;
    uint32_t last_fusion_us;
    uint32_t lost_since_us;
} vision_nav_obs_t;

extern vision_nav_obs_t vision_nav_obs;

void vision_nav_init(void);
void vision_nav_reset(void);
void vision_nav_update(void);
uint8_t vision_nav_target_active(void);
uint8_t vision_nav_target_seq(void);

#endif
