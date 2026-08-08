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
#define VISION_NAV_TARGET_LOST_RESET_US       1200000u
#define VISION_NAV_MCAR_FEEDBACK_TIMEOUT_US   500000u
#define VISION_NAV_MCAR_ARRIVE_ERR_PX         12.0f
#define VISION_NAV_PRINT_INTERVAL_US          100000u
#define VISION_NAV_FAST_SEARCH_ENABLE                1
#define VISION_NAV_SIMPLE_FINISH_MODE                1
#define VISION_NAV_SIMPLE_DIRECT_CAR_FOLLOW          1
#define VISION_NAV_CENTER_BEACON_POLICY_ENABLE       1
#define VISION_NAV_CENTER_BEACON_RADIUS_CM         75.0f
#define VISION_NAV_CENTER_ONLY_CONFIRM_FRAMES         8u
#define VISION_NAV_LOCKED_CANDIDATE_RADIUS_PX        30
#define VISION_NAV_CENTER_OBSERVATION_BACKOFF_CM    50.0f
/* Search trajectory is intentionally deferred.  Keep the fixed first leg,
 * then hold the search centre instead of running the local six-point path. */
#define VISION_NAV_LOCAL_SERPENTINE_ENABLE           0
#define VISION_NAV_SEARCH_CENTER_X_CM            175.0f
#define VISION_NAV_SEARCH_CENTER_Y_CM            0.0f
#define VISION_NAV_COMPETITION_SWITCH_INDEX          2u
#define VISION_NAV_MISSION_YAW_DEG                   0.0f
#define VISION_NAV_MISSION_YAW_START_ERR_DEG        10.0f
#define VISION_NAV_COMPETITION_ADVANCE_CM          325.0f
#define VISION_NAV_SEARCH_SPEED_LIMIT_CM_S         25.0f
#define VISION_NAV_SEARCH_MIN_HEIGHT_CM           65.0f
#define VISION_NAV_SEARCH_LOST_FRAMES               3u
#define VISION_NAV_SEARCH_REACQUIRE_FRAMES          2u
#define VISION_NAV_SEARCH_BEACON_BRAKE_HOLD_US 1000000u
#define VISION_NAV_SEARCH_HINT_TIMEOUT_US       3000000u
#define VISION_NAV_SEARCH_HINT_DEADZONE_PX          10.0f
#define VISION_NAV_SEARCH_HINT_VOTE_MAX                6
#define VISION_NAV_SEARCH_RESTART_MAX_SPEED_CM_S    15.0f
#define VISION_NAV_SEARCH_CENTER_CAPTURE_CM         30.0f
#define VISION_NAV_SEARCH_CENTER_MAX_SPEED_CM_S     18.0f
#define VISION_NAV_SEARCH_CENTER_SETTLE_US               0u
#define VISION_NAV_SEARCH_HALF_WIDTH_X_CM             110.0f
#define VISION_NAV_SEARCH_HALF_WIDTH_Y_CM             110.0f
#define VISION_NAV_SEARCH_GEOFENCE_INSET_CM             30.0f
#define VISION_NAV_SEARCH_SCAN_HALF_WIDTH_X_CM           90.0f
#define VISION_NAV_SEARCH_SCAN_HALF_WIDTH_Y_CM           60.0f
#define VISION_NAV_SEARCH_BOUNDARY_BRAKE_START_CM        60.0f
#define VISION_NAV_SEARCH_BOUNDARY_NEAR_CM               30.0f
#define VISION_NAV_SEARCH_BOUNDARY_NEAR_SPEED_CM_S       20.0f
#define VISION_NAV_SEARCH_WAYPOINT_CAPTURE_CM          30.0f
#define VISION_NAV_SEARCH_WAYPOINT_COUNT                 6u
#define VISION_NAV_APPROACH_SPEED_LIMIT_CM_S        30.0f
#define VISION_NAV_APPROACH_MIN_HEIGHT_CM           65.0f
#define VISION_NAV_APPROACH_ACQUIRE_FRAMES            3u
#define VISION_NAV_APPROACH_LOST_FRAMES               3u
#define VISION_NAV_APPROACH_STOP_RADIUS_CM          15.0f
#define VISION_NAV_APPROACH_TARGET_STEP_CM           8.0f
#define VISION_NAV_CAR_FOLLOW_START_CM               70.0f
#define VISION_NAV_CAR_FOLLOW_STOP_CM                50.0f
#define VISION_NAV_CAR_FOLLOW_MOVING_START_CM_S       8.0f
#define VISION_NAV_CAR_FOLLOW_MOVING_STOP_CM_S        4.0f
#define VISION_NAV_CAR_FOLLOW_TARGET_STEP_CM          0.7f
#define VISION_NAV_CAR_FOLLOW_SPEED_LIMIT_CM_S       35.0f
#define VISION_NAV_DIRECT_CAR_PX_TO_CM                 0.50f
#define VISION_NAV_DIRECT_CAR_LPF_ALPHA                0.30f
#define VISION_NAV_DIRECT_CAR_MAX_JUMP_PX             40.0f
#define VISION_NAV_DIRECT_CAR_DEADZONE_PX              5.0f
#define VISION_NAV_DIRECT_CAR_TARGET_STEP_CM            0.70f
#define VISION_NAV_DIRECT_CAR_TARGET_LEASH_CM          25.0f
#define VISION_NAV_CAR_CMD_DEADZONE_PX                  5.0f
#define VISION_NAV_CAR_CMD_SPEED_PER_ERR_CM_S           3.0f
#define VISION_NAV_CAR_CMD_MAX_SPEED_CM_S              55.0f
#define VISION_NAV_CAR_RECOVERY_CMD_MAX_SPEED_CM_S     12.0f
#define VISION_NAV_CAR_MEASURED_CORRECTION_GAIN          0.35f
#define VISION_NAV_CAR_FF_ENTRY_MAX_SPEED_CM_S        20.0f
#define VISION_NAV_CAR_FF_MAX_SPEED_CM_S               35.0f
#define VISION_NAV_CAR_FF_ENTRY_TIME_US             1000000u
#define VISION_NAV_CAR_FF_SLEW_CM_S2                   60.0f
#define VISION_NAV_BEACON_LOSS_BRAKE_DELAY_US        800000u
#define VISION_NAV_CAR_REENTRY_MAX_SPEED_CM_S           10.0f
#define VISION_NAV_CAR_FF_REJECT_SPEED_CM_S          180.0f
#define VISION_NAV_CAR_FF_LPF_ALPHA                    0.25f
#define VISION_NAV_CAR_FEEDBACK_BLEND_ALPHA             0.15f
#define VISION_NAV_CAR_BRAKE_ACCEL_CM_S2              80.0f
#define VISION_NAV_CAR_BRAKE_BUFFER_CM                 25.0f
#define VISION_NAV_CAR_BRAKE_MARGIN_CM_S                8.0f
#define VISION_NAV_CAR_BRAKE_MAX_CM_S                 120.0f
#define VISION_NAV_CAR_PREDICT_HOLD_US              120000u
#define VISION_NAV_CAR_RECOVERY_ENTER_HEIGHT_CM       45.0f
#define VISION_NAV_CAR_RECOVERY_EXIT_HEIGHT_CM        65.0f
#define VISION_NAV_CAR_RECOVERY_EXIT_FRAMES            3u
#define VISION_NAV_CAR_RECOVERY_POS_LEAD_S              0.20f
#define VISION_NAV_CAR_RECOVERY_MAX_ANGLE_DEG            8.0f
#define VISION_NAV_FLIGHT_READY_MIN_HEIGHT_CM           100.0f
#define VISION_NAV_FLIGHT_READY_MAX_ATT_DEG               15.0f
#define VISION_NAV_FLIGHT_READY_GOOD_FRAMES               3u
#define VISION_NAV_FLIGHT_READY_BAD_FRAMES                2u
/* 1: stable takeoff -> yaw 360 -> capture local field origin -> mission.
 * 0: bypass the complete pre-mission spin feature and use normal startup. */
#define VISION_NAV_YAW_SPIN_ENABLE                         1
#define VISION_NAV_YAW_SPIN_RATE_DPS                   -24.0f
#define VISION_NAV_YAW_SPIN_MAX_RATE_DPS                26.0f
#define VISION_NAV_YAW_SPIN_CT_LIMIT                     5.0f
#define VISION_NAV_YAW_SPIN_TOTAL_DEG                  360.0f
#define VISION_NAV_YAW_SPIN_FINISH_TOLERANCE_DEG        10.0f
#define VISION_NAV_YAW_SPIN_SEGMENT_DEG                120.0f
#define VISION_NAV_YAW_SPIN_SEGMENT_PAUSE_US          250000u
#define VISION_NAV_YAW_SPIN_SETTLE_US                100000u
#define VISION_NAV_YAW_SPIN_LOC_ANGLE_DEG                3.5f
#define VISION_NAV_YAW_SPIN_HEIGHT_CM                   80.0f
#define VISION_NAV_YAW_SPIN_HEIGHT_ERR_CM               10.0f
#define VISION_NAV_YAW_SPIN_READY_VZ_CM_S                8.0f
#define VISION_NAV_YAW_SPIN_READY_US                 500000u
#define VISION_NAV_YAW_SPIN_RECOVERY_TRIGGER_HEIGHT_CM 60.0f
#define VISION_NAV_YAW_SPIN_RECOVERY_TRIGGER_VZ_CM_S  -25.0f
#define VISION_NAV_YAW_SPIN_RECOVERY_RESUME_HEIGHT_CM  70.0f
#define VISION_NAV_YAW_SPIN_RECOVERY_RESUME_VZ_CM_S    -5.0f
#define VISION_NAV_FIELD_CENTER_FORWARD_CM             325.0f
#define VISION_NAV_HORIZONTAL_FAULT_SPEED_CM_S            75.0f
#define VISION_NAV_HORIZONTAL_FAULT_CRITICAL_SPEED_CM_S  120.0f
#define VISION_NAV_HORIZONTAL_FAULT_BAD_FRAMES             10u
#define VISION_NAV_HORIZONTAL_FAULT_CLEAR_SPEED_CM_S      30.0f
#define VISION_NAV_HORIZONTAL_FAULT_CLEAR_FRAMES          15u
#define VISION_NAV_HORIZONTAL_FAULT_LAND_US          6000000u
#define VISION_NAV_TRACK_CONFIRM_FRAMES                   3u
#define VISION_NAV_TRACK_CONFIRM_WINDOW_FRAMES            5u
#define VISION_NAV_TRACK_COAST_US                     200000u
#define VISION_NAV_PREMISSION_ALIGN_START_HEIGHT_CM      100.0f
#define VISION_NAV_PREMISSION_ALIGN_ERR_CM                15.0f
#define VISION_NAV_PREMISSION_ALIGN_SETTLE_US          500000u
#define VISION_NAV_PREMISSION_ALIGN_STEP_CM               35.0f
#define VISION_NAV_PREMISSION_ALIGN_OBS_GATE_CM           25.0f
#define VISION_NAV_PREMISSION_ALIGN_CONFIRM_FRAMES           3u
#define VISION_NAV_PREMISSION_ALIGN_TARGET_SPEED_CM_S     25.0f

typedef enum
{
    VISION_NAV_SEARCH = 0,
    VISION_NAV_TRACKING,
    VISION_NAV_CAR_ARRIVED,
    VISION_NAV_DONE
} vision_nav_state_t;

typedef enum
{
    VISION_BEACON_TRACK_LOST = 0,
    VISION_BEACON_TRACK_TENTATIVE,
    VISION_BEACON_TRACK_CONFIRMED,
    VISION_BEACON_TRACK_COASTING
} vision_beacon_track_state_t;

typedef enum
{
    VISION_PREMISSION_WAIT_CRUISE = 0,
    VISION_PREMISSION_ALIGN_CAR,
    VISION_PREMISSION_ALIGNED
} vision_premission_state_t;

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
    /* Legacy field name: nonzero means the continuous search scan is active. */
    uint8_t search_spiral_active;
    uint8_t search_lost_frames;
    uint8_t search_found_frames;
    uint8_t approach_active;
    uint8_t approach_arrived;
    uint8_t approach_lost_frames;
    uint8_t approach_found_frames;
    uint8_t car_follow_active;
    uint8_t car_prediction_active;
    uint8_t car_stop_brake_active;
    uint8_t beacon_loss_brake_active;
    uint8_t car_height_recovery_active;
    uint8_t flight_ready;
    uint8_t flight_ready_good_frames;
    uint8_t flight_ready_bad_frames;
    uint8_t horizontal_fault_active;
    uint8_t horizontal_fault_bad_frames;
    uint8_t horizontal_fault_good_frames;
    uint8_t beacon_track_state;
    uint8_t beacon_track_seen_frames;
    uint8_t premission_state;
    uint8_t premission_align_ready;
    uint8_t search_center_settled;
    uint8_t search_center_from_map;
    float search_ff_vel_x_cm_s;
    float search_ff_vel_y_cm_s;
    float search_center_distance_cm;
    float search_center_speed_cm_s;
    float car_distance_cm;
    float car_ff_vel_x_cm_s;
    float car_ff_vel_y_cm_s;
    float car_closing_speed_cm_s;
    float car_brake_vel_cm_s;
    float car_cmd_ff_vel_x_cm_s;
    float car_cmd_ff_vel_y_cm_s;
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
    float beacon_track_x_cm;
    float beacon_track_y_cm;
    uint32_t last_fusion_us;
    uint32_t lost_since_us;
    uint32_t beacon_track_last_seen_us;
} vision_nav_obs_t;

extern vision_nav_obs_t vision_nav_obs;

void vision_nav_init(void);
void vision_nav_reset(void);
void vision_nav_update(void);
uint8_t vision_nav_target_active(void);
uint8_t vision_nav_target_seq(void);
uint8_t vision_nav_beacon_command_valid(void);
uint8_t vision_nav_premission_align_ready(void);
uint8_t vision_nav_beacon_command_held(void);
uint8_t vision_nav_flight_ready(void);
uint8_t vision_nav_yaw_spin_is_active(void);
uint8_t vision_nav_yaw_spin_is_done(void);
void vision_nav_get_search_center(float *x_cm, float *y_cm);
uint8_t vision_nav_center_beacon_policy_active(void);
uint8_t vision_nav_beacon_is_center_reference(float body_x_cm,
                                               float body_y_cm);
uint8_t vision_nav_locked_target_is_center(void);
uint8_t vision_nav_beacon_selection_blocked(void);

#endif
