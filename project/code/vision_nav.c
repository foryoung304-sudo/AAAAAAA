#include "vision_nav.h"
#include "beacon.h"
#include "ekf_lite.h"
#include "flow.h"
#include "mcar_comm.h"
#include "param.h"

vision_nav_obs_t vision_nav_obs = {0};
static uint8_t vision_nav_hold_valid = 0u;
static float vision_nav_hold_x = 0.0f;
static float vision_nav_hold_y = 0.0f;
static float vision_nav_car_last_x = 0.0f;
static float vision_nav_car_last_y = 0.0f;
static float vision_nav_car_vel_x = 0.0f;
static float vision_nav_car_vel_y = 0.0f;
static uint32_t vision_nav_car_last_obs_us = 0u;
static uint32_t vision_nav_car_last_frame_id = 0u;
static uint32_t vision_nav_car_last_feedback_us = 0u;
static uint8_t vision_nav_car_velocity_valid = 0u;
static float vision_nav_car_ff_applied_x = 0.0f;
static float vision_nav_car_ff_applied_y = 0.0f;
static uint32_t vision_nav_car_ff_last_us = 0u;
static uint32_t vision_nav_car_follow_start_us = 0u;
static uint8_t vision_nav_car_recovery_active = 0u;
static uint8_t vision_nav_car_recovery_good_frames = 0u;
typedef enum
{
    VISION_NAV_SEARCH_RETURN_CENTER = 0,
    VISION_NAV_SEARCH_SCAN_LANES
} vision_nav_search_phase_t;

static vision_nav_search_phase_t vision_nav_search_phase =
    VISION_NAV_SEARCH_RETURN_CENTER;
static uint8_t vision_nav_search_waypoint_index = 0u;
static int8_t vision_nav_search_first_side = 1;
static uint8_t vision_nav_search_geofence_recovery = 0u;
static uint32_t vision_nav_search_beacon_last_seen_us = 0u;
static uint8_t vision_nav_competition_latched = 0u;
static uint8_t vision_nav_competition_switch_active = 0u;
static float vision_nav_field_center_x_cm = VISION_NAV_SEARCH_CENTER_X_CM;
static float vision_nav_field_center_y_cm = VISION_NAV_SEARCH_CENTER_Y_CM;
static float vision_nav_search_center_x_cm = VISION_NAV_SEARCH_CENTER_X_CM;
static float vision_nav_search_center_y_cm = VISION_NAV_SEARCH_CENTER_Y_CM;
static float vision_nav_search_right_x = 0.0f;
static float vision_nav_search_right_y = 1.0f;
static int8_t vision_nav_search_hint_vote = 0;
static uint8_t vision_nav_search_hint_valid = 0u;
static uint32_t vision_nav_search_hint_last_us = 0u;
static uint8_t vision_nav_cover_loss_latched = 0u;
static uint8_t vision_nav_track_window_frames = 0u;
static uint8_t vision_nav_simple_first_target_seen = 0u;
static uint8_t vision_nav_center_return_required = 0u;
static uint32_t vision_nav_beacon_last_frame_id = 0xFFFFFFFFu;
static uint8_t vision_nav_direct_car_filter_valid = 0u;
static uint32_t vision_nav_direct_car_last_frame_id = 0xFFFFFFFFu;
static float vision_nav_direct_car_last_forward_px = 0.0f;
static float vision_nav_direct_car_last_right_px = 0.0f;
static float vision_nav_direct_car_forward_px = 0.0f;
static float vision_nav_direct_car_right_px = 0.0f;
static uint8_t vision_nav_direct_car_target_valid = 0u;
static float vision_nav_direct_car_target_x_cm = 0.0f;
static float vision_nav_direct_car_target_y_cm = 0.0f;
static uint32_t vision_nav_horizontal_fault_since_us = 0u;
static uint8_t vision_nav_yaw_spin_active = 0u;
static uint8_t vision_nav_yaw_spin_done = 0u;
static uint8_t vision_nav_yaw_spin_segment_paused = 0u;
static float vision_nav_yaw_spin_last_deg = 0.0f;
static float vision_nav_yaw_spin_accumulated_deg = 0.0f;
static float vision_nav_yaw_spin_segment_target_deg =
    VISION_NAV_YAW_SPIN_SEGMENT_DEG;
static uint32_t vision_nav_yaw_spin_segment_pause_since_us = 0u;
static uint32_t vision_nav_yaw_spin_ready_since_us = 0u;
static uint32_t vision_nav_yaw_spin_settle_since_us = 0u;
static uint8_t vision_nav_yaw_spin_height_recovery_active = 0u;
static uint8_t vision_nav_yaw_spin_origin_valid = 0u;
static float vision_nav_yaw_spin_origin_x_cm = 0.0f;
static float vision_nav_yaw_spin_origin_y_cm = 0.0f;

static void vision_nav_search_reset_path(void);
static float vision_nav_absf(float value);

static void vision_nav_set_center_observation_point(void)
{
    float forward_x = vision_nav_search_right_y;
    float forward_y = -vision_nav_search_right_x;

    vision_nav_search_center_x_cm = vision_nav_field_center_x_cm -
        VISION_NAV_CENTER_OBSERVATION_BACKOFF_CM * forward_x;
    vision_nav_search_center_y_cm = vision_nav_field_center_y_cm -
        VISION_NAV_CENTER_OBSERVATION_BACKOFF_CM * forward_y;
}

static void vision_nav_direct_car_reset(void)
{
    vision_nav_direct_car_filter_valid = 0u;
    vision_nav_direct_car_last_frame_id = 0xFFFFFFFFu;
    vision_nav_direct_car_last_forward_px = 0.0f;
    vision_nav_direct_car_last_right_px = 0.0f;
    vision_nav_direct_car_forward_px = 0.0f;
    vision_nav_direct_car_right_px = 0.0f;
    vision_nav_direct_car_target_valid = 0u;
    vision_nav_direct_car_target_x_cm = 0.0f;
    vision_nav_direct_car_target_y_cm = 0.0f;
}

static void vision_nav_update_horizontal_fault(uint32_t now_us)
{
    float horizontal_speed = sqrtf(
        vehicle_state.current_vel_x * vehicle_state.current_vel_x +
        vehicle_state.current_vel_y * vehicle_state.current_vel_y);
    uint8_t bad_sample =
        ((flow_health.gate_clipped != 0u) ||
         (horizontal_speed >=
          VISION_NAV_HORIZONTAL_FAULT_SPEED_CM_S)) ? 1u : 0u;
    uint8_t critical_sample =
        (horizontal_speed >=
         VISION_NAV_HORIZONTAL_FAULT_CRITICAL_SPEED_CM_S) ? 1u : 0u;

#if VISION_NAV_YAW_SPIN_ENABLE
    /* Body yaw can look like a large translation to optical flow.  During the
     * deliberate pre-mission turn, keep weak position hold but do not turn
     * that expected observation error into a horizontal-flight fault. */
    if(vision_nav_yaw_spin_active != 0u)
    {
        vision_nav_obs.horizontal_fault_active = 0u;
        vision_nav_obs.horizontal_fault_bad_frames = 0u;
        vision_nav_obs.horizontal_fault_good_frames = 0u;
        vision_nav_horizontal_fault_since_us = 0u;
        return;
    }
#endif

    if((vehicle_state.armed == 0u) ||
       ((vehicle_state.flight_mode != FLY_AUTOFLY) &&
        (vehicle_state.flight_mode != FLY_AUTOTAKEOFF)))
    {
        vision_nav_obs.horizontal_fault_active = 0u;
        vision_nav_obs.horizontal_fault_bad_frames = 0u;
        vision_nav_obs.horizontal_fault_good_frames = 0u;
        vision_nav_horizontal_fault_since_us = 0u;
        return;
    }

    if(vision_nav_obs.horizontal_fault_active != 0u)
    {
        if((vehicle_state.flow_valid != 0u) &&
           (flow_health.gate_clipped == 0u) &&
           (horizontal_speed <=
            VISION_NAV_HORIZONTAL_FAULT_CLEAR_SPEED_CM_S))
        {
            if(vision_nav_obs.horizontal_fault_good_frames < 255u)
                vision_nav_obs.horizontal_fault_good_frames++;
            if(vision_nav_obs.horizontal_fault_good_frames >=
               VISION_NAV_HORIZONTAL_FAULT_CLEAR_FRAMES)
            {
                vision_nav_obs.horizontal_fault_active = 0u;
                vision_nav_obs.horizontal_fault_bad_frames = 0u;
                vision_nav_obs.horizontal_fault_good_frames = 0u;
                vision_nav_horizontal_fault_since_us = 0u;
            }
        }
        else
        {
            vision_nav_obs.horizontal_fault_good_frames = 0u;
        }

        /* During automatic takeoff, bad-but-valid Flow must brake and block
         * mission admission, but it must not introduce a new landing trigger.
         * The existing sustained-fault landing remains an AUTOFLY safeguard. */
        if((vehicle_state.flight_mode == FLY_AUTOFLY) &&
           (vision_nav_horizontal_fault_since_us != 0u) &&
           ((now_us - vision_nav_horizontal_fault_since_us) >=
            VISION_NAV_HORIZONTAL_FAULT_LAND_US))
        {
            auto_landing_request = 1u;
        }
    }
    else
    {
        if(bad_sample != 0u)
        {
            if(vision_nav_obs.horizontal_fault_bad_frames < 255u)
                vision_nav_obs.horizontal_fault_bad_frames++;
        }
        else
        {
            vision_nav_obs.horizontal_fault_bad_frames = 0u;
        }

        if((critical_sample != 0u) ||
           (vision_nav_obs.horizontal_fault_bad_frames >=
            VISION_NAV_HORIZONTAL_FAULT_BAD_FRAMES))
        {
            vision_nav_obs.horizontal_fault_active = 1u;
            vision_nav_obs.horizontal_fault_good_frames = 0u;
            vision_nav_horizontal_fault_since_us = now_us;
        }
    }

    if(vision_nav_obs.horizontal_fault_active != 0u)
    {
        vision_nav_obs.flight_ready = 0u;
        vision_nav_obs.flight_ready_good_frames = 0u;
        vision_nav_obs.car_follow_active = 0u;
        vision_nav_obs.car_prediction_active = 0u;
        vision_nav_obs.search_move_active = 0u;
        vision_nav_obs.search_spiral_active = 0u;
        vision_nav_obs.car_stop_brake_active = 1u;
        vision_nav_obs.car_ff_vel_x_cm_s = 0.0f;
        vision_nav_obs.car_ff_vel_y_cm_s = 0.0f;
        vision_nav_obs.search_ff_vel_x_cm_s = 0.0f;
        vision_nav_obs.search_ff_vel_y_cm_s = 0.0f;
        vision_nav_direct_car_reset();
    }
}

static void vision_nav_update_flight_readiness(uint8_t competition_active)
{
    uint8_t mission_yaw_ready = 1u;
#if VISION_NAV_YAW_SPIN_ENABLE
    mission_yaw_ready =
        (vision_nav_absf(vehicle_state.current_yaw -
                         VISION_NAV_MISSION_YAW_DEG) <=
         VISION_NAV_MISSION_YAW_START_ERR_DEG) ? 1u : 0u;
#endif

    uint8_t raw_ready =
        ((competition_active != 0u) &&
         (vehicle_state.armed != 0u) &&
         (vehicle_state.flight_mode == FLY_AUTOFLY) &&
         (vehicle_state.flow_valid != 0u) &&
         (loc_1l_ct.loc_hold_ready != 0u) &&
         (vision_nav_obs.horizontal_fault_active == 0u) &&
         (mission_height_recovery_active == 0u) &&
         (mission_yaw_ready != 0u) &&
         (vehicle_state.current_height >=
          VISION_NAV_FLIGHT_READY_MIN_HEIGHT_CM) &&
         (vehicle_state.current_height <= VISION_NAV_MAX_HEIGHT_CM) &&
         (vision_nav_absf(vehicle_state.current_roll) <=
          VISION_NAV_FLIGHT_READY_MAX_ATT_DEG) &&
         (vision_nav_absf(vehicle_state.current_pitch) <=
          VISION_NAV_FLIGHT_READY_MAX_ATT_DEG)) ? 1u : 0u;

    if(raw_ready != 0u)
    {
        vision_nav_obs.flight_ready_bad_frames = 0u;
        if(vision_nav_obs.flight_ready_good_frames < 255u)
        {
            vision_nav_obs.flight_ready_good_frames++;
        }
        if(vision_nav_obs.flight_ready_good_frames >=
           VISION_NAV_FLIGHT_READY_GOOD_FRAMES)
        {
            vision_nav_obs.flight_ready = 1u;
        }
    }
    else
    {
        vision_nav_obs.flight_ready_good_frames = 0u;
        if(vision_nav_obs.flight_ready_bad_frames < 255u)
        {
            vision_nav_obs.flight_ready_bad_frames++;
        }
        if(vision_nav_obs.flight_ready_bad_frames >=
           VISION_NAV_FLIGHT_READY_BAD_FRAMES)
        {
            vision_nav_obs.flight_ready = 0u;
        }
    }
}

static void vision_nav_update_beacon_track(uint8_t observation_valid,
                                           float observed_x,
                                           float observed_y,
                                           uint32_t now_us)
{
    float jump_x;
    float jump_y;

    if(observation_valid == 0u)
    {
        if(vision_nav_obs.beacon_track_state ==
           VISION_BEACON_TRACK_TENTATIVE)
        {
            if(vision_nav_track_window_frames < 255u)
            {
                vision_nav_track_window_frames++;
            }
            if(vision_nav_track_window_frames >=
               VISION_NAV_TRACK_CONFIRM_WINDOW_FRAMES)
            {
                vision_nav_obs.beacon_track_state = VISION_BEACON_TRACK_LOST;
                vision_nav_obs.beacon_track_seen_frames = 0u;
                vision_nav_track_window_frames = 0u;
            }
        }
        else if(vision_nav_obs.beacon_track_state ==
                VISION_BEACON_TRACK_CONFIRMED)
        {
            vision_nav_obs.beacon_track_state = VISION_BEACON_TRACK_COASTING;
            vision_nav_track_window_frames = 0u;
        }
        else if((vision_nav_obs.beacon_track_state ==
                 VISION_BEACON_TRACK_COASTING) &&
                (vision_nav_obs.beacon_track_last_seen_us != 0u) &&
                ((now_us - vision_nav_obs.beacon_track_last_seen_us) >=
                 VISION_NAV_TRACK_COAST_US))
        {
            vision_nav_obs.beacon_track_state = VISION_BEACON_TRACK_LOST;
            vision_nav_obs.beacon_track_seen_frames = 0u;
            vision_nav_track_window_frames = 0u;
        }
        return;
    }

    if(vision_nav_obs.beacon_track_state == VISION_BEACON_TRACK_LOST)
    {
        vision_nav_obs.beacon_track_state = VISION_BEACON_TRACK_TENTATIVE;
        vision_nav_obs.beacon_track_x_cm = observed_x;
        vision_nav_obs.beacon_track_y_cm = observed_y;
        vision_nav_obs.beacon_track_seen_frames = 1u;
        vision_nav_track_window_frames = 1u;
    }
    else if(vision_nav_obs.beacon_track_state ==
            VISION_BEACON_TRACK_COASTING)
    {
        jump_x = observed_x - vision_nav_obs.beacon_track_x_cm;
        jump_y = observed_y - vision_nav_obs.beacon_track_y_cm;
        if((vision_nav_absf(jump_x) <= VISION_NAV_ANCHOR_STABLE_CM) &&
           (vision_nav_absf(jump_y) <= VISION_NAV_ANCHOR_STABLE_CM))
        {
            vision_nav_obs.beacon_track_x_cm =
                0.7f * vision_nav_obs.beacon_track_x_cm + 0.3f * observed_x;
            vision_nav_obs.beacon_track_y_cm =
                0.7f * vision_nav_obs.beacon_track_y_cm + 0.3f * observed_y;
            vision_nav_obs.beacon_track_state =
                VISION_BEACON_TRACK_CONFIRMED;
            if(vision_nav_obs.beacon_track_seen_frames <
               VISION_NAV_TRACK_CONFIRM_FRAMES)
            {
                vision_nav_obs.beacon_track_seen_frames =
                    VISION_NAV_TRACK_CONFIRM_FRAMES;
            }
            vision_nav_track_window_frames = 0u;
        }
        else
        {
            vision_nav_obs.beacon_track_state =
                VISION_BEACON_TRACK_TENTATIVE;
            vision_nav_obs.beacon_track_x_cm = observed_x;
            vision_nav_obs.beacon_track_y_cm = observed_y;
            vision_nav_obs.beacon_track_seen_frames = 1u;
            vision_nav_track_window_frames = 1u;
        }
    }
    else
    {
        jump_x = observed_x - vision_nav_obs.beacon_track_x_cm;
        jump_y = observed_y - vision_nav_obs.beacon_track_y_cm;
        if((vision_nav_absf(jump_x) <= VISION_NAV_ANCHOR_STABLE_CM) &&
           (vision_nav_absf(jump_y) <= VISION_NAV_ANCHOR_STABLE_CM))
        {
            vision_nav_obs.beacon_track_x_cm =
                0.7f * vision_nav_obs.beacon_track_x_cm + 0.3f * observed_x;
            vision_nav_obs.beacon_track_y_cm =
                0.7f * vision_nav_obs.beacon_track_y_cm + 0.3f * observed_y;
            if(vision_nav_obs.beacon_track_seen_frames < 255u)
            {
                vision_nav_obs.beacon_track_seen_frames++;
            }
            if(vision_nav_obs.beacon_track_seen_frames >=
               VISION_NAV_TRACK_CONFIRM_FRAMES)
            {
                vision_nav_obs.beacon_track_state =
                    VISION_BEACON_TRACK_CONFIRMED;
                vision_nav_track_window_frames = 0u;
            }
            else if(vision_nav_obs.beacon_track_state ==
                    VISION_BEACON_TRACK_TENTATIVE)
            {
                if(vision_nav_track_window_frames < 255u)
                {
                    vision_nav_track_window_frames++;
                }
                if(vision_nav_track_window_frames >=
                   VISION_NAV_TRACK_CONFIRM_WINDOW_FRAMES)
                {
                    /* The current hit seeds the next bounded window. */
                    vision_nav_obs.beacon_track_seen_frames = 1u;
                    vision_nav_track_window_frames = 1u;
                    vision_nav_obs.beacon_track_x_cm = observed_x;
                    vision_nav_obs.beacon_track_y_cm = observed_y;
                }
            }
        }
        else
        {
            vision_nav_obs.beacon_track_state =
                VISION_BEACON_TRACK_TENTATIVE;
            vision_nav_obs.beacon_track_x_cm = observed_x;
            vision_nav_obs.beacon_track_y_cm = observed_y;
            vision_nav_obs.beacon_track_seen_frames = 1u;
            vision_nav_track_window_frames = 1u;
        }
    }
    vision_nav_obs.beacon_track_last_seen_us = now_us;
}

static uint8_t vision_nav_competition_update(void)
{
    uint8_t task_active =
        ((mission_task_requested != 0u) &&
         (vehicle_state.flight_mode == FLY_AUTOFLY)) ? 1u : 0u;

    if(vehicle_state.armed == 0u)
    {
        vision_nav_competition_latched = 0u;
        vision_nav_competition_switch_active = 0u;
        vision_nav_field_center_x_cm = VISION_NAV_SEARCH_CENTER_X_CM;
        vision_nav_field_center_y_cm = VISION_NAV_SEARCH_CENTER_Y_CM;
        vision_nav_search_center_x_cm = VISION_NAV_SEARCH_CENTER_X_CM;
        vision_nav_search_center_y_cm = VISION_NAV_SEARCH_CENTER_Y_CM;
        vision_nav_search_right_x = 0.0f;
        vision_nav_search_right_y = 1.0f;
        vision_nav_search_hint_vote = 0;
        vision_nav_search_hint_valid = 0u;
        vision_nav_search_hint_last_us = 0u;
        vision_nav_simple_first_target_seen = 0u;
        vision_nav_center_return_required = 0u;
        vision_nav_obs.search_center_from_map = 0u;
        return 0u;
    }

    if(task_active == 0u)
    {
        vision_nav_competition_latched = 0u;
        vision_nav_competition_switch_active = 0u;
        vision_nav_simple_first_target_seen = 0u;
        vision_nav_center_return_required = 0u;
        vision_nav_search_beacon_last_seen_us = 0u;
        return 0u;
    }

    if(vision_nav_competition_latched == 0u)
    {
#if VISION_NAV_YAW_SPIN_ENABLE
        /*
         * Treat the post-spin capture point as this run's local field origin.
         * The physical launch area is approximately field (0, 3.25 m), so
         * the square centre is one half-field length straight ahead.
         */
        if(vision_nav_yaw_spin_origin_valid != 0u)
        {
            vision_nav_field_center_x_cm =
                vision_nav_yaw_spin_origin_x_cm +
                VISION_NAV_FIELD_CENTER_FORWARD_CM;
            vision_nav_field_center_y_cm =
                vision_nav_yaw_spin_origin_y_cm;
            vision_nav_search_right_x = 0.0f;
            vision_nav_search_right_y = 1.0f;
        }
        else
        {
            vision_nav_field_center_x_cm =
                vehicle_state.current_pos_x +
                VISION_NAV_FIELD_CENTER_FORWARD_CM;
            vision_nav_field_center_y_cm =
                vehicle_state.current_pos_y;
            vision_nav_search_right_x = 0.0f;
            vision_nav_search_right_y = 1.0f;
        }
        vision_nav_set_center_observation_point();
        vision_nav_obs.search_center_from_map = 0u;
#else
        if(field_map_get_search_center_earth(
               &vision_nav_field_center_x_cm,
               &vision_nav_field_center_y_cm) != 0u)
        {
            vision_nav_obs.search_center_from_map = 1u;
        }
        else
        {
            vision_nav_field_center_x_cm = vehicle_state.current_pos_x +
                VISION_NAV_COMPETITION_ADVANCE_CM;
            vision_nav_field_center_y_cm = vehicle_state.current_pos_y;
            vision_nav_obs.search_center_from_map = 0u;
        }
        vision_nav_search_right_x = 0.0f;
        vision_nav_search_right_y = 1.0f;
        vision_nav_set_center_observation_point();
#endif
        vision_nav_search_hint_vote = 0;
        vision_nav_search_hint_valid = 0u;
        vision_nav_search_hint_last_us = 0u;
        vision_nav_center_return_required = 0u;
        vision_nav_competition_latched = 1u;
        vision_nav_search_reset_path();
    }

    vision_nav_competition_switch_active =
        (vision_nav_competition_latched != 0u) ? 1u : 0u;
    return vision_nav_competition_switch_active;
}

static float vision_nav_absf(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float vision_nav_clampf(float value, float min_value, float max_value)
{
    if(value < min_value) return min_value;
    if(value > max_value) return max_value;
    return value;
}

static void vision_nav_limit_vector(float *x, float *y, float limit)
{
    float magnitude = sqrtf((*x) * (*x) + (*y) * (*y));
    if((magnitude > limit) && (magnitude > 0.001f))
    {
        float scale = limit / magnitude;
        *x *= scale;
        *y *= scale;
    }
}

static float vision_nav_car_command_axis(float error_px)
{
    float magnitude = vision_nav_absf(error_px);
    if(magnitude <= VISION_NAV_CAR_CMD_DEADZONE_PX)
    {
        return 0.0f;
    }
    magnitude = (magnitude - VISION_NAV_CAR_CMD_DEADZONE_PX) *
                VISION_NAV_CAR_CMD_SPEED_PER_ERR_CM_S;
    return (error_px < 0.0f) ? -magnitude : magnitude;
}

static void vision_nav_car_ff_reset(uint32_t now_us)
{
    vision_nav_car_ff_applied_x = 0.0f;
    vision_nav_car_ff_applied_y = 0.0f;
    vision_nav_car_ff_last_us = now_us;
}

static void vision_nav_car_ff_smooth(float *target_x,
                                     float *target_y,
                                     uint32_t now_us)
{
    float dt = (vision_nav_car_ff_last_us == 0u) ? 0.02f :
        (float)(now_us - vision_nav_car_ff_last_us) * 1.0e-6f;
    float max_speed = VISION_NAV_CAR_FF_MAX_SPEED_CM_S;
    float dx;
    float dy;
    float max_step;

    dt = vision_nav_clampf(dt, 0.0f, 0.10f);
    if((vision_nav_car_follow_start_us != 0u) &&
       ((now_us - vision_nav_car_follow_start_us) <
        VISION_NAV_CAR_FF_ENTRY_TIME_US))
    {
        max_speed = VISION_NAV_CAR_FF_ENTRY_MAX_SPEED_CM_S;
    }
    vision_nav_limit_vector(target_x, target_y, max_speed);

    dx = *target_x - vision_nav_car_ff_applied_x;
    dy = *target_y - vision_nav_car_ff_applied_y;
    max_step = VISION_NAV_CAR_FF_SLEW_CM_S2 * dt;
    vision_nav_limit_vector(&dx, &dy, max_step);
    vision_nav_car_ff_applied_x += dx;
    vision_nav_car_ff_applied_y += dy;
    vision_nav_car_ff_last_us = now_us;
    *target_x = vision_nav_car_ff_applied_x;
    *target_y = vision_nav_car_ff_applied_y;
}

static void vision_nav_release_hold_position(void)
{
    vision_nav_hold_valid = 0u;
}

static void vision_nav_hold_current_position(void)
{
    if(vision_nav_hold_valid == 0u)
    {
        vision_nav_hold_x = vehicle_state.current_pos_x;
        vision_nav_hold_y = vehicle_state.current_pos_y;
        vision_nav_hold_valid = 1u;
    }
    vehicle_setpoint.target_pos_x = vision_nav_hold_x;
    vehicle_setpoint.target_pos_y = vision_nav_hold_y;
    vehicle_setpoint.target_vel_x = 0.0f;
    vehicle_setpoint.target_vel_y = 0.0f;
}

static uint8_t vision_nav_yaw_spin_update(uint32_t now_us)
{
#if VISION_NAV_YAW_SPIN_ENABLE
    float delta_deg;
    const float finish_deg = VISION_NAV_YAW_SPIN_TOTAL_DEG -
        VISION_NAV_YAW_SPIN_FINISH_TOLERANCE_DEG;

    if(vehicle_state.armed == 0u)
    {
        vision_nav_yaw_spin_active = 0u;
        vision_nav_yaw_spin_done = 0u;
        vision_nav_yaw_spin_segment_paused = 0u;
        vision_nav_yaw_spin_accumulated_deg = 0.0f;
        vision_nav_yaw_spin_segment_target_deg =
            VISION_NAV_YAW_SPIN_SEGMENT_DEG;
        vision_nav_yaw_spin_segment_pause_since_us = 0u;
        vision_nav_yaw_spin_ready_since_us = 0u;
        vision_nav_yaw_spin_settle_since_us = 0u;
        vision_nav_yaw_spin_height_recovery_active = 0u;
        vision_nav_yaw_spin_origin_valid = 0u;
        vehicle_setpoint.target_yaw_rate = 0.0f;
        return 0u;
    }

    if((vehicle_state.flight_mode != FLY_AUTOTAKEOFF) &&
       (vision_nav_yaw_spin_active != 0u))
    {
        vision_nav_yaw_spin_active = 0u;
        vision_nav_yaw_spin_height_recovery_active = 0u;
        vision_nav_yaw_spin_settle_since_us = 0u;
        vehicle_setpoint.target_yaw_rate = 0.0f;
        return 0u;
    }

    if(vision_nav_obs.horizontal_fault_active != 0u)
    {
        vision_nav_yaw_spin_active = 0u;
        vision_nav_yaw_spin_done = 0u;
        vision_nav_yaw_spin_origin_valid = 0u;
        vision_nav_yaw_spin_height_recovery_active = 0u;
        vision_nav_yaw_spin_settle_since_us = 0u;
        vehicle_setpoint.target_yaw_rate = 0.0f;
        return 0u;
    }

    if((vision_nav_yaw_spin_active == 0u) &&
       (vision_nav_yaw_spin_done == 0u) &&
       (vehicle_state.flight_mode == FLY_AUTOTAKEOFF))
    {
        if((vehicle_state.current_height >=
            (VISION_NAV_YAW_SPIN_HEIGHT_CM -
             VISION_NAV_YAW_SPIN_HEIGHT_ERR_CM)) &&
           (vision_nav_absf(vehicle_state.current_vel_z) <=
            VISION_NAV_YAW_SPIN_READY_VZ_CM_S) &&
           (loc_1l_ct.loc_hold_ready != 0u) &&
           (vision_nav_obs.horizontal_fault_active == 0u))
        {
            if(vision_nav_yaw_spin_ready_since_us == 0u)
            {
                vision_nav_yaw_spin_ready_since_us = now_us;
            }
            if((now_us - vision_nav_yaw_spin_ready_since_us) >=
               VISION_NAV_YAW_SPIN_READY_US)
            {
                vision_nav_yaw_spin_active = 1u;
                vision_nav_yaw_spin_last_deg = vehicle_state.current_yaw;
                vision_nav_yaw_spin_accumulated_deg = 0.0f;
                vision_nav_yaw_spin_segment_target_deg =
                    VISION_NAV_YAW_SPIN_SEGMENT_DEG;
                vision_nav_yaw_spin_segment_paused = 0u;
                vision_nav_yaw_spin_segment_pause_since_us = 0u;
                vision_nav_yaw_spin_ready_since_us = 0u;
                vision_nav_yaw_spin_settle_since_us = 0u;
                vision_nav_yaw_spin_height_recovery_active = 0u;
                vision_nav_hold_current_position();
            }
        }
        else
        {
            vision_nav_yaw_spin_ready_since_us = 0u;
        }
    }

    if(vision_nav_yaw_spin_active == 0u)
    {
        return 0u;
    }

    if((vision_nav_yaw_spin_height_recovery_active == 0u) &&
       ((vehicle_state.current_height <
         VISION_NAV_YAW_SPIN_RECOVERY_TRIGGER_HEIGHT_CM) ||
        (vehicle_state.current_vel_z <
         VISION_NAV_YAW_SPIN_RECOVERY_TRIGGER_VZ_CM_S)))
    {
        vision_nav_yaw_spin_height_recovery_active = 1u;
    }

    if(vision_nav_yaw_spin_height_recovery_active != 0u)
    {
        vision_nav_hold_current_position();
        vehicle_setpoint.target_yaw_rate = 0.0f;
        vision_nav_yaw_spin_last_deg = vehicle_state.current_yaw;
        if((vehicle_state.current_height >=
            VISION_NAV_YAW_SPIN_RECOVERY_RESUME_HEIGHT_CM) &&
           (vehicle_state.current_vel_z >=
            VISION_NAV_YAW_SPIN_RECOVERY_RESUME_VZ_CM_S))
        {
            vision_nav_yaw_spin_height_recovery_active = 0u;
        }
        return 1u;
    }

    /* Integrate the wrapped yaw delta so crossing +/-180 degrees is safe. */
    delta_deg = vehicle_state.current_yaw - vision_nav_yaw_spin_last_deg;
    while(delta_deg > 180.0f) delta_deg -= 360.0f;
    while(delta_deg < -180.0f) delta_deg += 360.0f;
    if(VISION_NAV_YAW_SPIN_RATE_DPS >= 0.0f)
    {
        vision_nav_yaw_spin_accumulated_deg += delta_deg;
    }
    else
    {
        vision_nav_yaw_spin_accumulated_deg -= delta_deg;
    }
    if(vision_nav_yaw_spin_accumulated_deg < 0.0f)
    {
        vision_nav_yaw_spin_accumulated_deg = 0.0f;
    }
    vision_nav_yaw_spin_last_deg = vehicle_state.current_yaw;

    vision_nav_hold_current_position();
    vision_nav_obs.search_move_active = 0u;
    vision_nav_obs.search_spiral_active = 0u;
    vision_nav_obs.car_follow_active = 0u;
    vision_nav_obs.car_prediction_active = 0u;
    vision_nav_obs.car_stop_brake_active = 1u;
    vision_nav_obs.search_ff_vel_x_cm_s = 0.0f;
    vision_nav_obs.search_ff_vel_y_cm_s = 0.0f;
    vision_nav_obs.car_ff_vel_x_cm_s = 0.0f;
    vision_nav_obs.car_ff_vel_y_cm_s = 0.0f;
    vision_nav_direct_car_reset();

    if(vision_nav_yaw_spin_segment_paused != 0u)
    {
        vehicle_setpoint.target_yaw_rate = 0.0f;
        if((now_us - vision_nav_yaw_spin_segment_pause_since_us) >=
           VISION_NAV_YAW_SPIN_SEGMENT_PAUSE_US)
        {
            vision_nav_yaw_spin_segment_paused = 0u;
            vision_nav_yaw_spin_segment_pause_since_us = 0u;
            vision_nav_yaw_spin_segment_target_deg +=
                VISION_NAV_YAW_SPIN_SEGMENT_DEG;
        }
        return 1u;
    }

    if((vision_nav_yaw_spin_accumulated_deg >=
        vision_nav_yaw_spin_segment_target_deg) &&
       (vision_nav_yaw_spin_accumulated_deg <
        VISION_NAV_YAW_SPIN_TOTAL_DEG))
    {
        vision_nav_yaw_spin_segment_paused = 1u;
        vision_nav_yaw_spin_segment_pause_since_us = now_us;
        vehicle_setpoint.target_yaw_rate = 0.0f;
        return 1u;
    }

    if(vision_nav_yaw_spin_accumulated_deg < finish_deg)
    {
        vehicle_setpoint.target_yaw_rate = VISION_NAV_YAW_SPIN_RATE_DPS;
        return 1u;
    }

    vehicle_setpoint.target_yaw_rate = 0.0f;
    if(vision_nav_yaw_spin_settle_since_us == 0u)
    {
        vision_nav_yaw_spin_settle_since_us = now_us;
    }
    if((now_us - vision_nav_yaw_spin_settle_since_us) >=
       VISION_NAV_YAW_SPIN_SETTLE_US)
    {
        vision_nav_yaw_spin_active = 0u;
        vision_nav_yaw_spin_done = 1u;
        vision_nav_yaw_spin_height_recovery_active = 0u;
        vision_nav_yaw_spin_origin_valid = 1u;
        vision_nav_yaw_spin_origin_x_cm = vehicle_state.current_pos_x;
        vision_nav_yaw_spin_origin_y_cm = vehicle_state.current_pos_y;
        vision_nav_hold_valid = 0u;
        vision_nav_hold_current_position();
    }
    return 1u;
#else
    (void)now_us;
    return 0u;
#endif
}

static void vision_nav_beacon_approach_update(uint8_t geometry_ok,
                                              float observed_beacon_x,
                                              float observed_beacon_y,
                                              float rel_x_e,
                                              float rel_y_e)
{
#if BEACON_APPROACH_TEST_ENABLE
    uint8_t eligible = ((vehicle_state.armed != 0u) &&
                        (vehicle_state.flight_mode == FLY_AUTOFLY) &&
                        (vehicle_state.flow_valid != 0u) &&
                        (vehicle_state.current_height >=
                         VISION_NAV_APPROACH_MIN_HEIGHT_CM)) ? 1u : 0u;

    vision_nav_obs.search_move_active = 0u;
    vision_nav_obs.search_lost_frames = 0u;
    vision_nav_obs.search_found_frames = 0u;

    if(eligible == 0u)
    {
        vision_nav_obs.approach_active = 0u;
        vision_nav_obs.approach_arrived = 0u;
        vision_nav_obs.approach_lost_frames = 0u;
        vision_nav_obs.approach_found_frames = 0u;
        vision_nav_release_hold_position();
        return;
    }

    if(geometry_ok == 0u)
    {
        vision_nav_obs.approach_found_frames = 0u;
        if(vision_nav_obs.approach_arrived != 0u)
        {
            vision_nav_hold_current_position();
            return;
        }
        if(vision_nav_obs.approach_lost_frames < 255u)
        {
            vision_nav_obs.approach_lost_frames++;
        }
        if(vision_nav_obs.approach_active == 0u ||
           vision_nav_obs.approach_lost_frames >=
           VISION_NAV_APPROACH_LOST_FRAMES)
        {
            vision_nav_obs.approach_active = 0u;
            vision_nav_release_hold_position();
            vision_nav_hold_current_position();
        }
        return;
    }

    vision_nav_obs.approach_lost_frames = 0u;
    if(vision_nav_obs.approach_found_frames < 255u)
    {
        vision_nav_obs.approach_found_frames++;
    }

    if(vision_nav_obs.approach_arrived != 0u)
    {
        vision_nav_hold_current_position();
        return;
    }

    if(vision_nav_obs.approach_active == 0u)
    {
        if(vision_nav_obs.approach_found_frames <
           VISION_NAV_APPROACH_ACQUIRE_FRAMES)
        {
            vision_nav_hold_current_position();
            return;
        }
        vision_nav_release_hold_position();
        vision_nav_obs.approach_active = 1u;
        vision_nav_obs.approach_arrived = 0u;
        vehicle_setpoint.target_pos_x = observed_beacon_x;
        vehicle_setpoint.target_pos_y = observed_beacon_y;
    }
    else
    {
        float dx = observed_beacon_x - vehicle_setpoint.target_pos_x;
        float dy = observed_beacon_y - vehicle_setpoint.target_pos_y;
        float distance = sqrtf(dx * dx + dy * dy);
        if(distance > VISION_NAV_APPROACH_TARGET_STEP_CM)
        {
            float scale = VISION_NAV_APPROACH_TARGET_STEP_CM / distance;
            dx *= scale;
            dy *= scale;
        }
        vehicle_setpoint.target_pos_x += dx;
        vehicle_setpoint.target_pos_y += dy;
    }

    if(sqrtf(rel_x_e * rel_x_e + rel_y_e * rel_y_e) <=
       VISION_NAV_APPROACH_STOP_RADIUS_CM)
    {
        vision_nav_obs.approach_active = 0u;
        vision_nav_obs.approach_arrived = 1u;
        vision_nav_release_hold_position();
        vision_nav_hold_current_position();
    }
    vehicle_setpoint.target_vel_x = 0.0f;
    vehicle_setpoint.target_vel_y = 0.0f;
#else
    (void)geometry_ok;
    (void)observed_beacon_x;
    (void)observed_beacon_y;
    (void)rel_x_e;
    (void)rel_y_e;
#endif
}

static void vision_nav_search_reset_path(void)
{
    vision_nav_search_phase = VISION_NAV_SEARCH_RETURN_CENTER;
    vision_nav_search_waypoint_index = 0u;
    vision_nav_obs.search_spiral_active = 0u;
    vision_nav_obs.search_center_settled = 0u;
    vision_nav_obs.search_center_distance_cm = 0.0f;
    vision_nav_obs.search_center_speed_cm_s = 0.0f;
    vision_nav_obs.search_ff_vel_x_cm_s = 0.0f;
    vision_nav_obs.search_ff_vel_y_cm_s = 0.0f;
}

#if VISION_NAV_LOCAL_SERPENTINE_ENABLE
static void vision_nav_search_waypoint(uint8_t index,
                                       int8_t first_side,
                                       float *target_x,
                                       float *target_y)
{
    float center_x = vision_nav_search_center_x_cm;
    float center_y = vision_nav_search_center_y_cm;

    /* A small three-row serpentine around the last covered lamp.  The
     * forward/right axes are latched at task start, so yaw never needs to
     * change and the tethered car can follow the aircraft continuously. */
    {
        uint8_t row = (uint8_t)(index / 2u);
        float side = (((index + row) & 1u) == 0u) ?
            (float)first_side : (float)-first_side;
        float forward = ((float)row + 1.0f) *
            (VISION_NAV_SEARCH_SCAN_HALF_WIDTH_X_CM / 3.0f);
        float forward_x = vision_nav_search_right_y;
        float forward_y = -vision_nav_search_right_x;
        *target_x = center_x + forward * forward_x +
            side * VISION_NAV_SEARCH_SCAN_HALF_WIDTH_Y_CM *
            vision_nav_search_right_x;
        *target_y = center_y + forward * forward_y +
            side * VISION_NAV_SEARCH_SCAN_HALF_WIDTH_Y_CM *
            vision_nav_search_right_y;
    }
}
#endif

static void vision_nav_fast_search_update(uint8_t beacon_camera_seen,
                                          uint32_t now_us)
{
#if VISION_NAV_FAST_SEARCH_ENABLE && !BEACON_APPROACH_TEST_ENABLE
    float center_dx;
    float center_dy;
    float center_distance;
    float center_speed;
    float current_speed;
    uint8_t eligible = ((vision_nav_obs.flight_ready != 0u) &&
                        (vision_nav_obs.car_arrived == 0u)) ? 1u : 0u;

    if(eligible == 0u)
    {
        vision_nav_obs.search_move_active = 0u;
        vision_nav_search_geofence_recovery = 0u;
        vision_nav_search_reset_path();
        vision_nav_obs.search_lost_frames = 0u;
        vision_nav_obs.search_found_frames = 0u;
        return;
    }

    current_speed = sqrtf(vehicle_state.current_vel_x *
                          vehicle_state.current_vel_x +
                          vehicle_state.current_vel_y *
                          vehicle_state.current_vel_y);

    {
        float min_x = vision_nav_search_center_x_cm -
                      VISION_NAV_SEARCH_HALF_WIDTH_X_CM;
        float max_x = vision_nav_search_center_x_cm +
                      VISION_NAV_SEARCH_HALF_WIDTH_X_CM;
        float min_y = vision_nav_search_center_y_cm -
                      VISION_NAV_SEARCH_HALF_WIDTH_Y_CM;
        float max_y = vision_nav_search_center_y_cm +
                      VISION_NAV_SEARCH_HALF_WIDTH_Y_CM;
        float recover_min_x = min_x + VISION_NAV_SEARCH_GEOFENCE_INSET_CM;
        float recover_max_x = max_x - VISION_NAV_SEARCH_GEOFENCE_INSET_CM;
        float recover_min_y = min_y + VISION_NAV_SEARCH_GEOFENCE_INSET_CM;
        float recover_max_y = max_y - VISION_NAV_SEARCH_GEOFENCE_INSET_CM;

        if((vision_nav_obs.search_center_settled != 0u) &&
           ((vehicle_state.current_pos_x < min_x) ||
            (vehicle_state.current_pos_x > max_x) ||
            (vehicle_state.current_pos_y < min_y) ||
            (vehicle_state.current_pos_y > max_y)))
        {
            vision_nav_search_geofence_recovery = 1u;
        }

        if(vision_nav_search_geofence_recovery != 0u)
        {
            if((vehicle_state.current_pos_x >= recover_min_x) &&
               (vehicle_state.current_pos_x <= recover_max_x) &&
               (vehicle_state.current_pos_y >= recover_min_y) &&
               (vehicle_state.current_pos_y <= recover_max_y))
            {
                vision_nav_search_geofence_recovery = 0u;
                vision_nav_search_reset_path();
            }
            else
            {
                if(beacon_camera_seen != 0u)
                {
                    /* Raw beacon evidence outranks boundary recovery motion:
                     * brake first, then let the normal 3-in-5 tracker decide
                     * whether cover guidance or recovery should resume. */
                    vision_nav_search_beacon_last_seen_us = now_us;
                    vision_nav_obs.search_move_active = 0u;
                    vision_nav_obs.search_spiral_active = 0u;
                    vision_nav_obs.search_ff_vel_x_cm_s = 0.0f;
                    vision_nav_obs.search_ff_vel_y_cm_s = 0.0f;
                    vision_nav_release_hold_position();
                    vision_nav_hold_current_position();
                    return;
                }
                /*
                 * Boundary recovery owns the mission target.  Keep the car
                 * following the aircraft and ignore flickering beacon
                 * reacquisition until the aircraft is safely back inside.
                 */
                vision_nav_obs.search_move_active = 1u;
                vision_nav_obs.search_found_frames = 0u;
                vehicle_setpoint.target_pos_x =
                    vision_nav_clampf(vehicle_state.current_pos_x,
                                      recover_min_x, recover_max_x);
                vehicle_setpoint.target_pos_y =
                    vision_nav_clampf(vehicle_state.current_pos_y,
                                      recover_min_y, recover_max_y);
                vehicle_setpoint.target_vel_x = 0.0f;
                vehicle_setpoint.target_vel_y = 0.0f;
                vision_nav_obs.search_spiral_active = 0u;
                vision_nav_obs.search_ff_vel_x_cm_s = 0.0f;
                vision_nav_obs.search_ff_vel_y_cm_s = 0.0f;
                return;
            }
        }
    }

    /* Search admission is a camera-semantic decision, not a projection-
     * geometry decision.  A visible beacon must block the move even when
     * roll/pitch temporarily makes its geometry unusable. */
    if(beacon_camera_seen != 0u)
    {
        float image_error_x =
            (float)beacon.centerX - IMAGE_CENTER_X;

        /* A beacon flickering at the image boundary is still useful evidence.
         * Remember its side, but keep the normal consecutive-frame rule for
         * declaring a real target. */
        if(image_error_x > VISION_NAV_SEARCH_HINT_DEADZONE_PX)
        {
            if(vision_nav_search_hint_vote < VISION_NAV_SEARCH_HINT_VOTE_MAX)
            {
                vision_nav_search_hint_vote++;
            }
            vision_nav_search_hint_valid = 1u;
            vision_nav_search_hint_last_us = now_us;
        }
        else if(image_error_x < -VISION_NAV_SEARCH_HINT_DEADZONE_PX)
        {
            if(vision_nav_search_hint_vote > -VISION_NAV_SEARCH_HINT_VOTE_MAX)
            {
                vision_nav_search_hint_vote--;
            }
            vision_nav_search_hint_valid = 1u;
            vision_nav_search_hint_last_us = now_us;
        }

        if((vision_nav_search_hint_valid != 0u) &&
           (vision_nav_search_hint_vote != 0))
        {
            vision_nav_search_first_side =
                (vision_nav_search_hint_vote > 0) ? 1 : -1;
        }

        vision_nav_obs.search_lost_frames = 0u;
        if(vision_nav_obs.search_found_frames < 255u)
        {
            vision_nav_obs.search_found_frames++;
        }

        /* The first raw candidate brakes the leader immediately.  The car
         * therefore receives STOP while the existing 3-in-5 tracker decides
         * whether this is a real lamp; a false candidate may resume only
         * after the bounded one-second brake hold and low-speed check. */
        vision_nav_search_beacon_last_seen_us = now_us;
        if(vision_nav_obs.search_move_active != 0u)
        {
            vision_nav_obs.search_move_active = 0u;
            vision_nav_obs.search_spiral_active = 0u;
            vision_nav_obs.search_ff_vel_x_cm_s = 0.0f;
            vision_nav_obs.search_ff_vel_y_cm_s = 0.0f;
            vision_nav_release_hold_position();
            vision_nav_hold_current_position();
        }

        /* Once geometry-consistent tracking is confirmed, begin_target()
         * has already switched the mission to TRACKING. */
        if((vision_nav_obs.state != VISION_NAV_SEARCH) &&
           (vision_nav_obs.beacon_track_state ==
            VISION_BEACON_TRACK_CONFIRMED))
        {
            vision_nav_search_reset_path();
            /* Search motion contaminates image-differenced absolute car
             * velocity. Re-seed from post-brake observations before allowing
             * measured car velocity into aircraft feed-forward. */
            vision_nav_car_vel_x = 0.0f;
            vision_nav_car_vel_y = 0.0f;
            vision_nav_car_velocity_valid = 0u;
            vision_nav_car_last_obs_us = 0u;
            vision_nav_car_last_frame_id =
                vision_detection_snapshot.frame_id;
            if(vision_nav_obs.car_follow_active == 0u)
            {
                vision_nav_hold_current_position();
            }
        }
    }
    else
    {
        if((vision_nav_search_hint_valid != 0u) &&
           ((now_us - vision_nav_search_hint_last_us) >
            VISION_NAV_SEARCH_HINT_TIMEOUT_US))
        {
            vision_nav_search_hint_vote = 0;
            vision_nav_search_hint_valid = 0u;
            vision_nav_search_hint_last_us = 0u;
        }

        vision_nav_obs.search_found_frames = 0u;

#if !VISION_NAV_LOCAL_SERPENTINE_ENABLE
        /* Once the fixed leg has settled, do not re-arm search motion merely
         * because no beacon is visible.  Candidate processing above remains
         * live while LOC gets normal stationary-waypoint braking. */
        if(vision_nav_obs.search_center_settled != 0u)
        {
            vision_nav_obs.search_move_active = 0u;
            vision_nav_obs.search_lost_frames = 0u;
            vehicle_setpoint.target_pos_x = vision_nav_search_center_x_cm;
            vehicle_setpoint.target_pos_y = vision_nav_search_center_y_cm;
            vehicle_setpoint.target_vel_x = 0.0f;
            vehicle_setpoint.target_vel_y = 0.0f;
            vision_nav_obs.search_spiral_active = 0u;
            vision_nav_obs.search_ff_vel_x_cm_s = 0.0f;
            vision_nav_obs.search_ff_vel_y_cm_s = 0.0f;
            return;
        }
#endif

        /* The school task starts the centre run only after the Y car is
         * actually visible while no beacon is visible.  Require the pair of
         * conditions for consecutive frames before starting; once admitted,
         * continue the centre run through brief Y-car observation gaps. */
        if(vision_nav_obs.search_move_active == 0u)
        {
            uint8_t beacon_brake_hold_active =
                ((vision_nav_search_beacon_last_seen_us != 0u) &&
                 ((now_us - vision_nav_search_beacon_last_seen_us) <
                  VISION_NAV_SEARCH_BEACON_BRAKE_HOLD_US)) ? 1u : 0u;

            if((beacon_brake_hold_active == 0u) &&
               (current_speed <= VISION_NAV_SEARCH_RESTART_MAX_SPEED_CM_S) &&
               (ycar_info.valid != 0u))
            {
                if(vision_nav_obs.search_lost_frames < 255u)
                {
                    vision_nav_obs.search_lost_frames++;
                }
            }
            else
            {
                vision_nav_obs.search_lost_frames = 0u;
            }
        }
        if(vision_nav_obs.search_lost_frames >= VISION_NAV_SEARCH_LOST_FRAMES)
        {
            if(vision_nav_obs.search_move_active == 0u)
            {
                float lateral_from_center;

                vision_nav_search_reset_path();
                if((vision_nav_search_hint_valid != 0u) &&
                   (vision_nav_search_hint_vote != 0))
                {
                    vision_nav_search_first_side =
                        (vision_nav_search_hint_vote > 0) ? 1 : -1;
                }
                else
                {
                    lateral_from_center =
                        (vehicle_state.current_pos_x -
                         vision_nav_search_center_x_cm) *
                            vision_nav_search_right_x +
                        (vehicle_state.current_pos_y -
                         vision_nav_search_center_y_cm) *
                            vision_nav_search_right_y;
                    vision_nav_search_first_side =
                        (lateral_from_center >= 0.0f) ? 1 : -1;
                }
            }
            vision_nav_obs.search_move_active = 1u;
        }
    }

    if((vehicle_state.flight_mode == FLY_AUTOFLY) &&
       (vision_nav_obs.search_move_active != 0u))
    {
        center_dx = vehicle_state.current_pos_x - vision_nav_search_center_x_cm;
        center_dy = vehicle_state.current_pos_y - vision_nav_search_center_y_cm;
        center_distance = sqrtf(center_dx * center_dx + center_dy * center_dy);
        center_speed = sqrtf(vehicle_state.current_vel_x *
                             vehicle_state.current_vel_x +
                             vehicle_state.current_vel_y *
                             vehicle_state.current_vel_y);
        vision_nav_obs.search_center_distance_cm = center_distance;
        vision_nav_obs.search_center_speed_cm_s = center_speed;

        if(vision_nav_search_phase == VISION_NAV_SEARCH_RETURN_CENTER)
        {
            if((center_distance <= VISION_NAV_SEARCH_CENTER_CAPTURE_CM) &&
               (center_speed <= VISION_NAV_SEARCH_CENTER_MAX_SPEED_CM_S))
            {
                vision_nav_search_phase = VISION_NAV_SEARCH_SCAN_LANES;
                vision_nav_search_waypoint_index = 0u;
                vision_nav_obs.search_center_settled = 1u;
                vision_nav_center_return_required = 0u;
            }

            if(vision_nav_search_phase == VISION_NAV_SEARCH_RETURN_CENTER)
            {
                vehicle_setpoint.target_pos_x =
                    vision_nav_search_center_x_cm;
                vehicle_setpoint.target_pos_y =
                    vision_nav_search_center_y_cm;
                vehicle_setpoint.target_vel_x = 0.0f;
                vehicle_setpoint.target_vel_y = 0.0f;
                vision_nav_obs.search_spiral_active = 0u;
                vision_nav_obs.search_ff_vel_x_cm_s = 0.0f;
                vision_nav_obs.search_ff_vel_y_cm_s = 0.0f;
                return;
            }
        }

#if !VISION_NAV_LOCAL_SERPENTINE_ENABLE
        /* The fixed first leg and the post-cover one-second wait remain
         * active, but no local search trajectory owns the aircraft after it
         * reaches the centre.  End search motion so LOC uses its full static
         * waypoint braking and the Y car receives STOP with the aircraft. */
        vision_nav_obs.search_move_active = 0u;
        vehicle_setpoint.target_pos_x = vision_nav_search_center_x_cm;
        vehicle_setpoint.target_pos_y = vision_nav_search_center_y_cm;
        vehicle_setpoint.target_vel_x = 0.0f;
        vehicle_setpoint.target_vel_y = 0.0f;
        vision_nav_obs.search_spiral_active = 0u;
        vision_nav_obs.search_ff_vel_x_cm_s = 0.0f;
        vision_nav_obs.search_ff_vel_y_cm_s = 0.0f;
        return;
#else
        {
            float target_x;
            float target_y;
            float waypoint_dx;
            float waypoint_dy;
            float waypoint_distance;

            vision_nav_search_waypoint(vision_nav_search_waypoint_index,
                                       vision_nav_search_first_side,
                                       &target_x,
                                       &target_y);
            waypoint_dx = target_x - vehicle_state.current_pos_x;
            waypoint_dy = target_y - vehicle_state.current_pos_y;
            waypoint_distance = sqrtf(waypoint_dx * waypoint_dx +
                                      waypoint_dy * waypoint_dy);
            if(waypoint_distance <= VISION_NAV_SEARCH_WAYPOINT_CAPTURE_CM)
            {
                vision_nav_search_waypoint_index =
                    (uint8_t)((vision_nav_search_waypoint_index + 1u) %
                              VISION_NAV_SEARCH_WAYPOINT_COUNT);
                vision_nav_search_waypoint(vision_nav_search_waypoint_index,
                                           vision_nav_search_first_side,
                                           &target_x,
                                           &target_y);
            }

            vehicle_setpoint.target_pos_x = target_x;
            vehicle_setpoint.target_pos_y = target_y;
            vehicle_setpoint.target_vel_x = 0.0f;
            vehicle_setpoint.target_vel_y = 0.0f;
            vision_nav_obs.search_ff_vel_x_cm_s = 0.0f;
            vision_nav_obs.search_ff_vel_y_cm_s = 0.0f;
            vision_nav_obs.search_spiral_active = 1u;
        }
#endif
    }
#else
    (void)beacon_camera_seen;
    (void)now_us;
#endif
}

static void vision_nav_direct_car_follow_update(uint32_t now_us)
{
    uint8_t eligible;
    float forward_px;
    float right_px;
    float forward_cm;
    float right_cm;
    float earth_x_cm;
    float earth_y_cm;
    uint8_t new_direct_frame = 0u;

    vision_nav_obs.car_prediction_active = 0u;
    vision_nav_obs.car_ff_vel_x_cm_s = 0.0f;
    vision_nav_obs.car_ff_vel_y_cm_s = 0.0f;
    vision_nav_obs.car_cmd_ff_vel_x_cm_s = 0.0f;
    vision_nav_obs.car_cmd_ff_vel_y_cm_s = 0.0f;
    vision_nav_obs.car_closing_speed_cm_s = 0.0f;
    vision_nav_obs.car_brake_vel_cm_s = 0.0f;
    vision_nav_obs.car_height_recovery_active = 0u;

    /* During search the waypoint generator owns the position setpoint. */
    if((vehicle_state.flight_mode == FLY_AUTOFLY) &&
       (vision_nav_obs.search_move_active != 0u))
    {
        vision_nav_obs.car_follow_active = 0u;
        vision_nav_obs.car_stop_brake_active = 0u;
        vision_nav_obs.car_distance_cm = 0.0f;
        vision_nav_direct_car_reset();
        vision_nav_release_hold_position();
        return;
    }

    eligible = ((vehicle_state.armed != 0u) &&
                (vehicle_state.flight_mode == FLY_AUTOFLY) &&
                (vehicle_state.flow_valid != 0u) &&
                (loc_1l_ct.loc_hold_ready != 0u) &&
                (vision_nav_obs.flight_ready != 0u) &&
                (vision_nav_obs.state == VISION_NAV_TRACKING) &&
                (vision_nav_obs.beacon_loss_brake_active == 0u) &&
                (ycar_info.valid != 0u)) ? 1u : 0u;

    if(eligible == 0u)
    {
        vision_nav_obs.car_follow_active = 0u;
        vision_nav_obs.car_stop_brake_active = 1u;
        vision_nav_obs.car_distance_cm = 0.0f;
        vision_nav_direct_car_reset();
        if((vehicle_state.flight_mode == FLY_AUTOFLY) ||
           (vehicle_state.flight_mode == FLY_AUTOTAKEOFF))
        {
            vision_nav_hold_current_position();
        }
        else
        {
            vision_nav_release_hold_position();
        }
        return;
    }

    /* ycar_body_x/y are already the current project's camera error in body
     * forward/right axes. Filter once per camera frame, reject large jumps,
     * then advance a latched world-frame target through the existing LOC,
     * optical-flow and attitude cascade. Keeping the target latched inside
     * the camera deadzone prevents aircraft drift from rewriting the hold
     * point to the current moving position. */
    if(vision_detection_snapshot.frame_id !=
       vision_nav_direct_car_last_frame_id)
    {
        float raw_forward_px = ycar_body_x;
        float raw_right_px = ycar_body_y;

        vision_nav_direct_car_last_frame_id =
            vision_detection_snapshot.frame_id;
        new_direct_frame = 1u;
        if(vision_nav_direct_car_filter_valid == 0u)
        {
            vision_nav_direct_car_last_forward_px = raw_forward_px;
            vision_nav_direct_car_last_right_px = raw_right_px;
            vision_nav_direct_car_forward_px =
                VISION_NAV_DIRECT_CAR_LPF_ALPHA * raw_forward_px;
            vision_nav_direct_car_right_px =
                VISION_NAV_DIRECT_CAR_LPF_ALPHA * raw_right_px;
            vision_nav_direct_car_filter_valid = 1u;
        }
        else
        {
            if(vision_nav_absf(raw_forward_px -
                               vision_nav_direct_car_last_forward_px) <=
               VISION_NAV_DIRECT_CAR_MAX_JUMP_PX)
            {
                vision_nav_direct_car_last_forward_px = raw_forward_px;
            }
            if(vision_nav_absf(raw_right_px -
                               vision_nav_direct_car_last_right_px) <=
               VISION_NAV_DIRECT_CAR_MAX_JUMP_PX)
            {
                vision_nav_direct_car_last_right_px = raw_right_px;
            }
            vision_nav_direct_car_forward_px +=
                VISION_NAV_DIRECT_CAR_LPF_ALPHA *
                (vision_nav_direct_car_last_forward_px -
                 vision_nav_direct_car_forward_px);
            vision_nav_direct_car_right_px +=
                VISION_NAV_DIRECT_CAR_LPF_ALPHA *
                (vision_nav_direct_car_last_right_px -
                 vision_nav_direct_car_right_px);
        }
    }

    forward_px = vision_nav_direct_car_forward_px;
    right_px = vision_nav_direct_car_right_px;
    if(vision_nav_absf(forward_px) <=
       VISION_NAV_DIRECT_CAR_DEADZONE_PX)
    {
        forward_px = 0.0f;
    }
    if(vision_nav_absf(right_px) <=
       VISION_NAV_DIRECT_CAR_DEADZONE_PX)
    {
        right_px = 0.0f;
    }

    forward_cm = forward_px * VISION_NAV_DIRECT_CAR_PX_TO_CM;
    right_cm = right_px * VISION_NAV_DIRECT_CAR_PX_TO_CM;
    earth_x_cm = forward_cm * vehicle_state.yaw_cos -
                 right_cm * vehicle_state.yaw_sin;
    earth_y_cm = forward_cm * vehicle_state.yaw_sin +
                 right_cm * vehicle_state.yaw_cos;

    if(vision_nav_direct_car_target_valid == 0u)
    {
        vision_nav_direct_car_target_x_cm =
            vehicle_setpoint.target_pos_x;
        vision_nav_direct_car_target_y_cm =
            vehicle_setpoint.target_pos_y;
        vision_nav_direct_car_target_valid = 1u;
    }

    if(new_direct_frame != 0u)
    {
        float step_x = earth_x_cm;
        float step_y = earth_y_cm;
        float step = sqrtf(step_x * step_x + step_y * step_y);
        if((step > VISION_NAV_DIRECT_CAR_TARGET_STEP_CM) &&
           (step > 0.001f))
        {
            float scale = VISION_NAV_DIRECT_CAR_TARGET_STEP_CM / step;
            step_x *= scale;
            step_y *= scale;
        }
        vision_nav_direct_car_target_x_cm += step_x;
        vision_nav_direct_car_target_y_cm += step_y;
    }

    {
        float leash_x = vision_nav_direct_car_target_x_cm -
                        vehicle_state.current_pos_x;
        float leash_y = vision_nav_direct_car_target_y_cm -
                        vehicle_state.current_pos_y;
        float leash = sqrtf(leash_x * leash_x + leash_y * leash_y);
        if((leash > VISION_NAV_DIRECT_CAR_TARGET_LEASH_CM) &&
           (leash > 0.001f))
        {
            float scale = VISION_NAV_DIRECT_CAR_TARGET_LEASH_CM / leash;
            vision_nav_direct_car_target_x_cm =
                vehicle_state.current_pos_x + leash_x * scale;
            vision_nav_direct_car_target_y_cm =
                vehicle_state.current_pos_y + leash_y * scale;
        }
    }

    vision_nav_release_hold_position();
    vehicle_setpoint.target_pos_x = vision_nav_direct_car_target_x_cm;
    vehicle_setpoint.target_pos_y = vision_nav_direct_car_target_y_cm;
    vehicle_setpoint.target_vel_x = 0.0f;
    vehicle_setpoint.target_vel_y = 0.0f;
    vision_nav_obs.car_distance_cm =
        sqrtf(forward_cm * forward_cm + right_cm * right_cm);
    vision_nav_obs.car_follow_active = 1u;
    vision_nav_obs.car_stop_brake_active = 0u;
}

static void vision_nav_follow_car_update(uint8_t beacon_geometry_ok,
                                         uint32_t now_us)
{
#if VISION_MISSION_MODE_ENABLE
#if VISION_NAV_SIMPLE_DIRECT_CAR_FOLLOW
    (void)beacon_geometry_ok;
    vision_nav_direct_car_follow_update(now_us);
#else
    (void)beacon_geometry_ok;
    float car_rel_body_x;
    float car_rel_body_y;
    float car_rel_earth_x;
    float car_rel_earth_y;
    float distance;
    float car_speed;
    float car_command_speed = 0.0f;
    float command_ff_x = 0.0f;
    float command_ff_y = 0.0f;
    float car_x;
    float car_y;
    float ff_x = 0.0f;
    float ff_y = 0.0f;
    float car_closing_speed = 0.0f;
    float car_brake_vel = 0.0f;
    uint8_t new_car_observation = 0u;
    uint8_t car_geometry_ok;
    uint8_t eligible;
    uint8_t feedback_speed_valid;
    uint8_t feedback_stopping;

    car_rel_body_x = ycar_body_x * vehicle_state.current_height *
                     BEACON_SCALE_X + CAMERA_OFFSET_BODY_X_CM;
    car_rel_body_y = ycar_body_y * vehicle_state.current_height *
                     BEACON_SCALE_Y + CAMERA_OFFSET_BODY_Y_CM;
    car_rel_earth_x = car_rel_body_x * vehicle_state.yaw_cos -
                      car_rel_body_y * vehicle_state.yaw_sin;
    car_rel_earth_y = car_rel_body_x * vehicle_state.yaw_sin +
                      car_rel_body_y * vehicle_state.yaw_cos;
    distance = sqrtf(car_rel_earth_x * car_rel_earth_x +
                     car_rel_earth_y * car_rel_earth_y);
    car_speed = sqrtf(vision_nav_car_vel_x * vision_nav_car_vel_x +
                      vision_nav_car_vel_y * vision_nav_car_vel_y);
    feedback_speed_valid = (mcar_comm_feedback_is_fresh(
        vision_nav_obs.target_seq, VISION_NAV_MCAR_FEEDBACK_TIMEOUT_US) != 0u &&
        (mcar_comm_feedback.status & MCAR_COMM_STATUS_SPEED_VALID) != 0u &&
        (mcar_comm_feedback.status & MCAR_COMM_STATUS_COMM_LOST) == 0u) ? 1u : 0u;
    feedback_stopping = (feedback_speed_valid != 0u &&
        (mcar_comm_feedback.status & MCAR_COMM_STATUS_STOPPING) != 0u &&
        (mcar_comm_feedback.status & MCAR_COMM_STATUS_STOPPED) == 0u) ? 1u : 0u;
    vision_nav_obs.car_distance_cm = distance;
    vision_nav_obs.car_prediction_active = 0u;
    vision_nav_obs.car_stop_brake_active = 0u;
    vision_nav_obs.car_cmd_ff_vel_x_cm_s = 0.0f;
    vision_nav_obs.car_cmd_ff_vel_y_cm_s = 0.0f;
    vision_nav_obs.car_closing_speed_cm_s = 0.0f;
    vision_nav_obs.car_brake_vel_cm_s = 0.0f;
    car_geometry_ok = ((ycar_info.valid != 0u) &&
                       (vehicle_state.current_height >= VISION_NAV_MIN_HEIGHT_CM) &&
                       (vehicle_state.current_height <= VISION_NAV_MAX_HEIGHT_CM) &&
                       (vision_nav_absf(vehicle_state.current_roll) <=
                        VISION_NAV_MAX_ATT_DEG) &&
                       (vision_nav_absf(vehicle_state.current_pitch) <=
                        VISION_NAV_MAX_ATT_DEG)) ? 1u : 0u;

    car_x = vehicle_state.current_pos_x + car_rel_earth_x;
    car_y = vehicle_state.current_pos_y + car_rel_earth_y;

    /* Mirror the car firmware's target_vel = transmitted_error * 8 command,
     * then rotate car forward/right into the local earth frame. */
    if(((mcar_comm_diag.last_tx_flags & MCAR_COMM_FLAG_TARGET_ACTIVE) != 0u) &&
       ((mcar_comm_diag.last_tx_flags & MCAR_COMM_FLAG_YAW_VALID) != 0u) &&
       ((mcar_comm_diag.last_tx_flags & MCAR_COMM_FLAG_STOP) == 0u) &&
       (mcar_guidance_diag.valid != 0u))
    {
        float yaw_rad = mcar_guidance_diag.mcar_yaw_earth_deg * 0.0174532925f;
        float yaw_cos = cosf(yaw_rad);
        float yaw_sin = sinf(yaw_rad);
        float command_forward = vision_nav_car_command_axis(
            (float)mcar_comm_diag.last_tx_err_forward_px);
        float command_right = vision_nav_car_command_axis(
            (float)mcar_comm_diag.last_tx_err_right_px);

        command_ff_x = command_forward * yaw_cos - command_right * yaw_sin;
        command_ff_y = command_forward * yaw_sin + command_right * yaw_cos;
        vision_nav_limit_vector(&command_ff_x, &command_ff_y,
            ((mcar_comm_diag.last_tx_flags &
              MCAR_COMM_FLAG_RECOVERY_SLOW) != 0u) ?
            VISION_NAV_CAR_RECOVERY_CMD_MAX_SPEED_CM_S :
            VISION_NAV_CAR_CMD_MAX_SPEED_CM_S);
        car_command_speed = sqrtf(command_ff_x * command_ff_x +
                                  command_ff_y * command_ff_y);
        vision_nav_obs.car_cmd_ff_vel_x_cm_s = command_ff_x;
        vision_nav_obs.car_cmd_ff_vel_y_cm_s = command_ff_y;
    }

    /* The car returns encoder-derived body forward/right speed while
     * SPEED_VALID is set.  A steering/direction encoder is useful for motion
     * direction and STOPPING, but is not ground truth: blend it lightly into
     * the visual estimate instead of replacing that estimate. */
    if((feedback_speed_valid != 0u) &&
       (mcar_guidance_diag.valid != 0u) &&
       (mcar_comm_feedback.timestamp_us != vision_nav_car_last_feedback_us))
    {
        float yaw_rad = mcar_guidance_diag.mcar_yaw_earth_deg * 0.0174532925f;
        float yaw_cos = cosf(yaw_rad);
        float yaw_sin = sinf(yaw_rad);
        float feedback_forward =
            (float)mcar_comm_feedback.speed_forward_cm_s;
        float feedback_right =
            (float)mcar_comm_feedback.speed_right_cm_s;
        float feedback_vel_x = feedback_forward * yaw_cos -
                               feedback_right * yaw_sin;
        float feedback_vel_y = feedback_forward * yaw_sin +
                               feedback_right * yaw_cos;
        float feedback_speed = sqrtf(feedback_vel_x * feedback_vel_x +
                                     feedback_vel_y * feedback_vel_y);

        vision_nav_car_last_feedback_us = mcar_comm_feedback.timestamp_us;

        if(feedback_speed <= VISION_NAV_CAR_FF_REJECT_SPEED_CM_S)
        {
            if(vision_nav_car_velocity_valid == 0u)
            {
                /* Startup fallback until two usable visual car samples exist. */
                vision_nav_car_vel_x = feedback_vel_x;
                vision_nav_car_vel_y = feedback_vel_y;
                vision_nav_car_velocity_valid = 1u;
            }
            else
            {
                vision_nav_car_vel_x += VISION_NAV_CAR_FEEDBACK_BLEND_ALPHA *
                    (feedback_vel_x - vision_nav_car_vel_x);
                vision_nav_car_vel_y += VISION_NAV_CAR_FEEDBACK_BLEND_ALPHA *
                    (feedback_vel_y - vision_nav_car_vel_y);
            }
            vision_nav_limit_vector(&vision_nav_car_vel_x,
                                    &vision_nav_car_vel_y,
                                    VISION_NAV_CAR_FF_MAX_SPEED_CM_S);
        }
    }
    if((car_geometry_ok != 0u) &&
       (vision_detection_snapshot.frame_id != vision_nav_car_last_frame_id))
    {
        new_car_observation = 1u;
        if(vision_nav_car_last_obs_us != 0u)
        {
            float obs_dt = (float)(now_us - vision_nav_car_last_obs_us) * 1.0e-6f;
            if((obs_dt >= 0.010f) && (obs_dt <= 0.250f))
            {
                float raw_vx = (car_x - vision_nav_car_last_x) / obs_dt;
                float raw_vy = (car_y - vision_nav_car_last_y) / obs_dt;
                float raw_speed = sqrtf(raw_vx * raw_vx + raw_vy * raw_vy);
                if(raw_speed <= VISION_NAV_CAR_FF_REJECT_SPEED_CM_S)
                {
                    if(vision_nav_car_velocity_valid == 0u)
                    {
                        vision_nav_car_vel_x = raw_vx;
                        vision_nav_car_vel_y = raw_vy;
                        vision_nav_car_velocity_valid = 1u;
                    }
                    else
                    {
                        vision_nav_car_vel_x += VISION_NAV_CAR_FF_LPF_ALPHA *
                            (raw_vx - vision_nav_car_vel_x);
                        vision_nav_car_vel_y += VISION_NAV_CAR_FF_LPF_ALPHA *
                            (raw_vy - vision_nav_car_vel_y);
                    }
                    vision_nav_limit_vector(&vision_nav_car_vel_x,
                                            &vision_nav_car_vel_y,
                                            VISION_NAV_CAR_FF_MAX_SPEED_CM_S);
                }
            }
        }
        vision_nav_car_last_x = car_x;
        vision_nav_car_last_y = car_y;
        vision_nav_car_last_obs_us = now_us;
        vision_nav_car_last_frame_id = vision_detection_snapshot.frame_id;
    }

    /* All following start/stop and braking decisions use the final fused
     * velocity, including a visual update received in this same frame. */
    car_speed = sqrtf(vision_nav_car_vel_x * vision_nav_car_vel_x +
                      vision_nav_car_vel_y * vision_nav_car_vel_y);

    if(vehicle_state.current_height <= VISION_NAV_CAR_RECOVERY_ENTER_HEIGHT_CM)
    {
        vision_nav_car_recovery_active = 1u;
        vision_nav_car_recovery_good_frames = 0u;
    }
    else if(vision_nav_car_recovery_active != 0u)
    {
        /* Altitude recovery authority must not remain locked at 8 degrees
         * merely because the Y-car is outside the camera view.  Once height
         * itself is healthy for several consecutive vision updates, release
         * the low-altitude angle protection; car observation eligibility is
         * handled independently by the normal follow state machine. */
        if(vehicle_state.current_height >=
           VISION_NAV_CAR_RECOVERY_EXIT_HEIGHT_CM)
        {
            if(vision_nav_car_recovery_good_frames < 255u)
                vision_nav_car_recovery_good_frames++;
            if(vision_nav_car_recovery_good_frames >=
               VISION_NAV_CAR_RECOVERY_EXIT_FRAMES)
            {
                vision_nav_car_recovery_active = 0u;
                vision_nav_car_recovery_good_frames = 0u;
            }
        }
        else
        {
            vision_nav_car_recovery_good_frames = 0u;
        }
    }
    vision_nav_obs.car_height_recovery_active =
        vision_nav_car_recovery_active;

    /*
     * Search owns the waypoint and deliberately commands the car to follow
     * the aircraft, so do not replace its target with a hold point.
     */
    if((vehicle_state.flight_mode == FLY_AUTOFLY) &&
       (vision_nav_obs.search_move_active != 0u))
    {
        vision_nav_obs.car_follow_active = 0u;
        vision_nav_release_hold_position();
        return;
    }

    /*
     * Falling below the mission-height floor changes the mode from AUTOFLY
     * back to AUTOTAKEOFF.  The communication layer then correctly sends
     * STOP to the car, but the aircraft must also stop following the stale
     * horizontal trajectory; otherwise it keeps moving while the car is
     * stationary and excites the physical cable.
     */
    if(vehicle_state.flight_mode != FLY_AUTOFLY)
    {
        vision_nav_obs.car_follow_active = 0u;
        vision_nav_obs.car_prediction_active = 0u;
        vision_nav_obs.car_stop_brake_active = 1u;
        vision_nav_obs.car_ff_vel_x_cm_s = 0.0f;
        vision_nav_obs.car_ff_vel_y_cm_s = 0.0f;
        vision_nav_obs.car_cmd_ff_vel_x_cm_s = 0.0f;
        vision_nav_obs.car_cmd_ff_vel_y_cm_s = 0.0f;
        vision_nav_car_vel_x = 0.0f;
        vision_nav_car_vel_y = 0.0f;
        vision_nav_car_velocity_valid = 0u;
        if(vehicle_state.flight_mode == FLY_AUTOTAKEOFF)
        {
            vision_nav_hold_current_position();
        }
        else
        {
            vision_nav_release_hold_position();
        }
        return;
    }

    if(vision_nav_obs.flight_ready == 0u)
    {
        vision_nav_obs.car_follow_active = 0u;
        vision_nav_obs.car_prediction_active = 0u;
        vision_nav_obs.car_stop_brake_active = 1u;
        vision_nav_obs.car_ff_vel_x_cm_s = 0.0f;
        vision_nav_obs.car_ff_vel_y_cm_s = 0.0f;
        vision_nav_obs.car_cmd_ff_vel_x_cm_s = 0.0f;
        vision_nav_obs.car_cmd_ff_vel_y_cm_s = 0.0f;
        vision_nav_car_vel_x = 0.0f;
        vision_nav_car_vel_y = 0.0f;
        vision_nav_car_velocity_valid = 0u;
        vision_nav_car_follow_start_us = 0u;
        vision_nav_car_ff_reset(now_us);
        vision_nav_hold_current_position();
        return;
    }

    /* A likely car-on-beacon loss brakes the tethered aircraft.  Ordinary
     * camera flicker remains in bounded track coasting instead of repeatedly
     * switching the aircraft between follow and position hold. */
    if(vision_nav_obs.beacon_loss_brake_active != 0u)
    {
        vision_nav_obs.car_follow_active = 0u;
        vision_nav_obs.car_prediction_active = 0u;
        vision_nav_obs.car_stop_brake_active = 1u;
        vision_nav_obs.car_ff_vel_x_cm_s = 0.0f;
        vision_nav_obs.car_ff_vel_y_cm_s = 0.0f;
        vision_nav_obs.car_cmd_ff_vel_x_cm_s = 0.0f;
        vision_nav_obs.car_cmd_ff_vel_y_cm_s = 0.0f;
        vision_nav_car_vel_x = 0.0f;
        vision_nav_car_vel_y = 0.0f;
        vision_nav_car_velocity_valid = 0u;
        vision_nav_car_follow_start_us = 0u;
        vision_nav_car_ff_reset(now_us);
        vision_nav_hold_current_position();
        return;
    }

    if((vision_nav_obs.car_arrived != 0u) ||
       (vision_nav_obs.target_done != 0u))
    {
        vision_nav_obs.car_follow_active = 0u;
        vision_nav_obs.car_prediction_active = 0u;
        vision_nav_obs.car_stop_brake_active = 1u;
        vision_nav_obs.car_ff_vel_x_cm_s = 0.0f;
        vision_nav_obs.car_ff_vel_y_cm_s = 0.0f;
        vision_nav_obs.car_cmd_ff_vel_x_cm_s = 0.0f;
        vision_nav_obs.car_cmd_ff_vel_y_cm_s = 0.0f;
        vision_nav_car_vel_x = 0.0f;
        vision_nav_car_vel_y = 0.0f;
        vision_nav_car_velocity_valid = 0u;
        vision_nav_car_follow_start_us = 0u;
        vision_nav_car_ff_reset(now_us);
        vision_nav_hold_current_position();
        return;
    }

    /*
     * A transmitted STOP is authoritative.  Do not let image-differenced
     * car velocity re-arm following while the car is intentionally stopped:
     * aircraft motion and projection noise can otherwise make a stationary
     * car appear to move, causing the aircraft to chase its own disturbance
     * and repeatedly tension the physical cable.
     *
     * Hold capture is latched, so repeated STOP frames do not rewrite the
     * target to the moving aircraft position every camera frame.
     */
    if(((mcar_comm_diag.last_tx_flags & MCAR_COMM_FLAG_STOP) != 0u) &&
       (feedback_stopping == 0u))
    {
        vision_nav_obs.car_follow_active = 0u;
        vision_nav_obs.car_prediction_active = 0u;
        vision_nav_obs.car_stop_brake_active = 1u;
        vision_nav_obs.car_ff_vel_x_cm_s = 0.0f;
        vision_nav_obs.car_ff_vel_y_cm_s = 0.0f;
        vision_nav_obs.car_cmd_ff_vel_x_cm_s = 0.0f;
        vision_nav_obs.car_cmd_ff_vel_y_cm_s = 0.0f;
        vision_nav_car_vel_x = 0.0f;
        vision_nav_car_vel_y = 0.0f;
        vision_nav_car_velocity_valid = 0u;
        vision_nav_car_follow_start_us = 0u;
        vision_nav_car_ff_reset(now_us);
        vision_nav_hold_current_position();
        return;
    }

    eligible = ((vehicle_state.armed != 0u) &&
                (vehicle_state.flow_valid != 0u) &&
                (loc_1l_ct.loc_hold_ready != 0u) &&
                (car_geometry_ok != 0u) &&
                (vision_nav_obs.state == VISION_NAV_TRACKING)) ? 1u : 0u;

    if(eligible == 0u)
    {
        uint32_t observation_age_us = (vision_nav_car_last_obs_us == 0u) ?
            0xFFFFFFFFu : (now_us - vision_nav_car_last_obs_us);
        if((vision_nav_obs.car_follow_active != 0u) &&
           (vehicle_state.armed != 0u) &&
           (vehicle_state.flight_mode == FLY_AUTOFLY) &&
           (vehicle_state.flow_valid != 0u) &&
           (loc_1l_ct.loc_hold_ready != 0u) &&
           (vision_nav_car_velocity_valid != 0u) &&
           (observation_age_us <= VISION_NAV_CAR_PREDICT_HOLD_US))
        {
            float decay = 1.0f -
                (float)observation_age_us /
                (float)VISION_NAV_CAR_PREDICT_HOLD_US;
            float predict_dt = (vision_detection_snapshot.camera_dt_us > 0u) ?
                (float)vision_detection_snapshot.camera_dt_us * 1.0e-6f : 0.02f;
            decay = vision_nav_clampf(decay, 0.0f, 1.0f);
            predict_dt = vision_nav_clampf(predict_dt, 0.01f, 0.10f);
            ff_x = (command_ff_x + vision_nav_car_vel_x *
                    VISION_NAV_CAR_MEASURED_CORRECTION_GAIN) * decay;
            ff_y = (command_ff_y + vision_nav_car_vel_y *
                    VISION_NAV_CAR_MEASURED_CORRECTION_GAIN) * decay;
            vision_nav_limit_vector(&ff_x, &ff_y,
                                    VISION_NAV_CAR_FF_MAX_SPEED_CM_S);
            vision_nav_car_ff_smooth(&ff_x, &ff_y, now_us);
            vehicle_setpoint.target_pos_x += ff_x * predict_dt;
            vehicle_setpoint.target_pos_y += ff_y * predict_dt;
            vehicle_setpoint.target_vel_x = ff_x;
            vehicle_setpoint.target_vel_y = ff_y;
            vision_nav_obs.car_prediction_active = 1u;
            vision_nav_obs.car_ff_vel_x_cm_s = ff_x;
            vision_nav_obs.car_ff_vel_y_cm_s = ff_y;
            return;
        }
        if(vision_nav_obs.car_follow_active != 0u)
        {
            vision_nav_obs.car_follow_active = 0u;
            vision_nav_car_follow_start_us = 0u;
            vision_nav_car_ff_reset(now_us);
            vision_nav_release_hold_position();
            vision_nav_hold_current_position();
        }
        return;
    }

    if(vision_nav_obs.car_follow_active == 0u)
    {
        float first_step;
        float scale;
        float aircraft_speed = sqrtf(
            vehicle_state.current_vel_x * vehicle_state.current_vel_x +
            vehicle_state.current_vel_y * vehicle_state.current_vel_y);

        if((distance <= VISION_NAV_CAR_FOLLOW_START_CM) &&
           (car_speed < VISION_NAV_CAR_FOLLOW_MOVING_START_CM_S) &&
           (car_command_speed < VISION_NAV_CAR_FOLLOW_MOVING_START_CM_S))
        {
            return;
        }
        if(aircraft_speed > VISION_NAV_CAR_REENTRY_MAX_SPEED_CM_S)
        {
            return;
        }

        vision_nav_release_hold_position();
        vision_nav_obs.car_follow_active = 1u;
        vision_nav_car_follow_start_us = now_us;
        vision_nav_car_ff_reset(now_us);
        first_step = distance - VISION_NAV_CAR_FOLLOW_STOP_CM;
        if(first_step < 0.0f)
        {
            first_step = 0.0f;
        }
        if(first_step > VISION_NAV_CAR_FOLLOW_TARGET_STEP_CM)
        {
            first_step = VISION_NAV_CAR_FOLLOW_TARGET_STEP_CM;
        }
        scale = first_step / distance;
        vehicle_setpoint.target_pos_x =
            vehicle_state.current_pos_x + car_rel_earth_x * scale;
        vehicle_setpoint.target_pos_y =
            vehicle_state.current_pos_y + car_rel_earth_y * scale;
    }
    else if((distance <= VISION_NAV_CAR_FOLLOW_STOP_CM) &&
            (car_speed <= VISION_NAV_CAR_FOLLOW_MOVING_STOP_CM_S) &&
            (car_command_speed <= VISION_NAV_CAR_FOLLOW_MOVING_STOP_CM_S))
    {
        vision_nav_obs.car_follow_active = 0u;
        vision_nav_car_follow_start_us = 0u;
        vision_nav_car_ff_reset(now_us);
        vision_nav_release_hold_position();
        vision_nav_hold_current_position();
    }
    else
    {
        float follow_distance = distance - VISION_NAV_CAR_FOLLOW_STOP_CM;
        float follow_scale;
        if(follow_distance < 0.0f) follow_distance = 0.0f;
        follow_scale = (distance > 0.001f) ? follow_distance / distance : 0.0f;
        float desired_x = vehicle_state.current_pos_x +
            car_rel_earth_x * follow_scale;
        float desired_y = vehicle_state.current_pos_y +
            car_rel_earth_y * follow_scale;
        float dx = desired_x - vehicle_setpoint.target_pos_x;
        float dy = desired_y - vehicle_setpoint.target_pos_y;
        float target_step = sqrtf(dx * dx + dy * dy);

        if(target_step > VISION_NAV_CAR_FOLLOW_TARGET_STEP_CM)
        {
            float scale = VISION_NAV_CAR_FOLLOW_TARGET_STEP_CM / target_step;
            dx *= scale;
            dy *= scale;
        }
        vehicle_setpoint.target_pos_x += dx;
        vehicle_setpoint.target_pos_y += dy;
    }

    ff_x = command_ff_x;
    ff_y = command_ff_y;
    if(vision_nav_car_velocity_valid != 0u)
    {
        ff_x += vision_nav_car_vel_x *
                VISION_NAV_CAR_MEASURED_CORRECTION_GAIN;
        ff_y += vision_nav_car_vel_y *
                VISION_NAV_CAR_MEASURED_CORRECTION_GAIN;
    }
    vision_nav_limit_vector(&ff_x, &ff_y,
                            VISION_NAV_CAR_FF_MAX_SPEED_CM_S);
    if(distance > 0.001f)
    {
        float unit_x = car_rel_earth_x / distance;
        float unit_y = car_rel_earth_y / distance;
        float car_ref_vx = (vision_nav_car_velocity_valid != 0u) ?
            vision_nav_car_vel_x : command_ff_x;
        float car_ref_vy = (vision_nav_car_velocity_valid != 0u) ?
            vision_nav_car_vel_y : command_ff_y;
        float spacing_remaining = distance -
            VISION_NAV_CAR_FOLLOW_STOP_CM -
            VISION_NAV_CAR_BRAKE_BUFFER_CM;
        float allowed_closing_speed;

        if(spacing_remaining < 0.0f)
        {
            spacing_remaining = 0.0f;
        }
        car_closing_speed =
            (vehicle_state.current_vel_x - car_ref_vx) * unit_x +
            (vehicle_state.current_vel_y - car_ref_vy) * unit_y;
        allowed_closing_speed = sqrtf(2.0f *
            VISION_NAV_CAR_BRAKE_ACCEL_CM_S2 * spacing_remaining);
        allowed_closing_speed -= VISION_NAV_CAR_BRAKE_MARGIN_CM_S;
        if(allowed_closing_speed < 0.0f)
        {
            allowed_closing_speed = 0.0f;
        }
        car_brake_vel = car_closing_speed - allowed_closing_speed;
        car_brake_vel = vision_nav_clampf(car_brake_vel,
                                          0.0f,
                                          VISION_NAV_CAR_BRAKE_MAX_CM_S);
        ff_x -= unit_x * car_brake_vel;
        ff_y -= unit_y * car_brake_vel;
        vision_nav_limit_vector(&ff_x, &ff_y,
                                VISION_NAV_CAR_FF_MAX_SPEED_CM_S);
    }
    vision_nav_obs.car_closing_speed_cm_s = car_closing_speed;
    vision_nav_obs.car_brake_vel_cm_s = car_brake_vel;
    vision_nav_car_ff_smooth(&ff_x, &ff_y, now_us);
    if(vision_nav_car_recovery_active != 0u)
    {
        vehicle_setpoint.target_pos_x = vehicle_state.current_pos_x +
            ff_x * VISION_NAV_CAR_RECOVERY_POS_LEAD_S;
        vehicle_setpoint.target_pos_y = vehicle_state.current_pos_y +
            ff_y * VISION_NAV_CAR_RECOVERY_POS_LEAD_S;
    }
    vehicle_setpoint.target_vel_x = ff_x;
    vehicle_setpoint.target_vel_y = ff_y;
    vision_nav_obs.car_ff_vel_x_cm_s = ff_x;
    vision_nav_obs.car_ff_vel_y_cm_s = ff_y;
#endif
#else
    (void)beacon_geometry_ok;
    (void)now_us;
    vision_nav_obs.car_follow_active = 0u;
    vision_nav_obs.car_distance_cm = 0.0f;
#endif
}

static uint8_t vision_nav_geometry_ok(void)
{
    if(beacon.status != BEACON_FOUND) return 0u;
    if(vehicle_state.current_height < VISION_NAV_MIN_HEIGHT_CM) return 0u;
    if(vehicle_state.current_height > VISION_NAV_MAX_HEIGHT_CM) return 0u;
    if(vision_nav_absf(vehicle_state.current_roll) > VISION_NAV_MAX_ATT_DEG) return 0u;
    if(vision_nav_absf(vehicle_state.current_pitch) > VISION_NAV_MAX_ATT_DEG) return 0u;
    return 1u;
}

static void vision_nav_clear_measurement(void)
{
    vision_nav_obs.valid = 0u;
    vision_nav_obs.used_for_ekf = 0u;
    vision_nav_obs.confidence = 0.0f;
    vision_nav_obs.drone_x_cm = 0.0f;
    vision_nav_obs.drone_y_cm = 0.0f;
    vision_nav_obs.rel_earth_x_cm = 0.0f;
    vision_nav_obs.rel_earth_y_cm = 0.0f;
    vision_nav_obs.observed_beacon_x_cm = 0.0f;
    vision_nav_obs.observed_beacon_y_cm = 0.0f;
    vision_nav_obs.residual_x_cm = 0.0f;
    vision_nav_obs.residual_y_cm = 0.0f;
    vision_nav_obs.ekf_correction_x_cm = 0.0f;
    vision_nav_obs.ekf_correction_y_cm = 0.0f;
}

static void vision_nav_start_search(void)
{
    uint32_t now_us = system_time_us();

    vision_nav_obs.state = VISION_NAV_SEARCH;
    vision_nav_obs.anchor_valid = 0u;
    vision_nav_obs.car_arrived = 0u;
    vision_nav_obs.target_done = 0u;
    vision_nav_obs.stable_frames = 0u;
    vision_nav_obs.lost_since_us = 0u;
    vision_nav_cover_loss_latched = 0u;
    vision_nav_obs.search_move_active = 0u;
    vision_nav_search_reset_path();

#if VISION_NAV_SIMPLE_FINISH_MODE
    if(vision_nav_simple_first_target_seen != 0u)
    {
        /* Hold briefly after a covered/lost lamp, then return to the fixed
         * centre observation point before selecting another lamp.  The
         * aircraft leads and the tethered Y car follows through search mode. */
        vision_nav_center_return_required = 1u;
        vision_nav_search_beacon_last_seen_us = now_us;
        vision_nav_release_hold_position();
        vision_nav_hold_current_position();
    }
#else
    (void)now_us;
#endif
}

static void vision_nav_begin_target(float beacon_x, float beacon_y)
{
    vision_nav_obs.target_seq++;
    if(vision_nav_obs.target_seq == 0u) vision_nav_obs.target_seq = 1u;
    vision_nav_obs.anchor_x_cm = beacon_x;
    vision_nav_obs.anchor_y_cm = beacon_y;
    vision_nav_obs.anchor_valid = 1u;
    vision_nav_obs.car_arrived = 0u;
    vision_nav_obs.target_done = 0u;
    vision_nav_obs.state = VISION_NAV_TRACKING;
    vision_nav_obs.lost_since_us = 0u;
    vision_nav_cover_loss_latched = 0u;
#if VISION_NAV_SIMPLE_FINISH_MODE
    vision_nav_simple_first_target_seen = 1u;
#endif
}

static void vision_nav_apply_ekf(float residual_x, float residual_y,
                                 float confidence)
{
#if VISION_NAV_ENABLE_EKF_UPDATE
    float r = 1600.0f / vision_nav_clampf(confidence, 0.25f, 1.0f);
    float kx = ekf_lite_p_xy[0] / (ekf_lite_p_xy[0] + r);
    float ky = ekf_lite_p_xy[1] / (ekf_lite_p_xy[1] + r);
    float correction_x = kx * residual_x;
    float correction_y = ky * residual_y;

    correction_x = vision_nav_clampf(correction_x,
        -VISION_NAV_MAX_EKF_CORRECTION_CM, VISION_NAV_MAX_EKF_CORRECTION_CM);
    correction_y = vision_nav_clampf(correction_y,
        -VISION_NAV_MAX_EKF_CORRECTION_CM, VISION_NAV_MAX_EKF_CORRECTION_CM);

    ekf_lite_state.x += correction_x;
    ekf_lite_state.y += correction_y;
    if(vision_nav_absf(residual_x) > 0.001f) kx = vision_nav_absf(correction_x / residual_x);
    if(vision_nav_absf(residual_y) > 0.001f) ky = vision_nav_absf(correction_y / residual_y);
    ekf_lite_p_xy[0] *= (1.0f - vision_nav_clampf(kx, 0.0f, 1.0f));
    ekf_lite_p_xy[1] *= (1.0f - vision_nav_clampf(ky, 0.0f, 1.0f));
    vehicle_state.current_pos_x = ekf_lite_state.x;
    vehicle_state.current_pos_y = ekf_lite_state.y;
    vision_nav_obs.ekf_correction_x_cm = correction_x;
    vision_nav_obs.ekf_correction_y_cm = correction_y;
    vision_nav_obs.used_for_ekf = 1u;
#else
    (void)residual_x;
    (void)residual_y;
    (void)confidence;
#endif
}

void vision_nav_init(void)
{
    vision_nav_reset();
}

void vision_nav_reset(void)
{
    memset(&vision_nav_obs, 0, sizeof(vision_nav_obs));
    vision_nav_release_hold_position();
    vision_nav_car_last_x = 0.0f;
    vision_nav_car_last_y = 0.0f;
    vision_nav_car_vel_x = 0.0f;
    vision_nav_car_vel_y = 0.0f;
    vision_nav_car_last_obs_us = 0u;
    vision_nav_car_last_frame_id = 0u;
    vision_nav_car_last_feedback_us = 0u;
    vision_nav_yaw_spin_active = 0u;
    vision_nav_yaw_spin_done = 0u;
    vision_nav_yaw_spin_segment_paused = 0u;
    vision_nav_yaw_spin_last_deg = 0.0f;
    vision_nav_yaw_spin_accumulated_deg = 0.0f;
    vision_nav_yaw_spin_segment_target_deg =
        VISION_NAV_YAW_SPIN_SEGMENT_DEG;
    vision_nav_yaw_spin_segment_pause_since_us = 0u;
    vision_nav_yaw_spin_ready_since_us = 0u;
    vision_nav_yaw_spin_settle_since_us = 0u;
    vision_nav_yaw_spin_height_recovery_active = 0u;
    vision_nav_yaw_spin_origin_valid = 0u;
    vision_nav_yaw_spin_origin_x_cm = 0.0f;
    vision_nav_yaw_spin_origin_y_cm = 0.0f;
    vision_nav_car_velocity_valid = 0u;
    vision_nav_car_ff_applied_x = 0.0f;
    vision_nav_car_ff_applied_y = 0.0f;
    vision_nav_car_ff_last_us = 0u;
    vision_nav_car_follow_start_us = 0u;
    vision_nav_car_recovery_active = 0u;
    vision_nav_car_recovery_good_frames = 0u;
    vision_nav_search_beacon_last_seen_us = 0u;
    vision_nav_competition_latched = 0u;
    vision_nav_competition_switch_active = 0u;
    vision_nav_field_center_x_cm = VISION_NAV_SEARCH_CENTER_X_CM;
    vision_nav_field_center_y_cm = VISION_NAV_SEARCH_CENTER_Y_CM;
    vision_nav_search_center_x_cm = VISION_NAV_SEARCH_CENTER_X_CM;
    vision_nav_search_center_y_cm = VISION_NAV_SEARCH_CENTER_Y_CM;
    vision_nav_search_right_x = 0.0f;
    vision_nav_search_right_y = 1.0f;
    vision_nav_search_hint_vote = 0;
    vision_nav_search_hint_valid = 0u;
    vision_nav_search_hint_last_us = 0u;
    vision_nav_horizontal_fault_since_us = 0u;
    vision_nav_obs.search_center_from_map = 0u;
    vision_nav_cover_loss_latched = 0u;
    vision_nav_track_window_frames = 0u;
    vision_nav_simple_first_target_seen = 0u;
    vision_nav_center_return_required = 0u;
    vision_nav_beacon_last_frame_id = 0xFFFFFFFFu;
    vision_nav_direct_car_reset();
    vision_nav_search_reset_path();
    vision_nav_start_search();
}

uint8_t vision_nav_target_active(void)
{
    return (vision_nav_obs.state == VISION_NAV_TRACKING ||
            vision_nav_obs.state == VISION_NAV_CAR_ARRIVED) ? 1u : 0u;
}

uint8_t vision_nav_target_seq(void)
{
    return vision_nav_obs.target_seq;
}

void vision_nav_get_search_center(float *x_cm, float *y_cm)
{
    if(x_cm != NULL) *x_cm = vision_nav_search_center_x_cm;
    if(y_cm != NULL) *y_cm = vision_nav_search_center_y_cm;
}

uint8_t vision_nav_center_beacon_policy_active(void)
{
#if VISION_NAV_CENTER_BEACON_POLICY_ENABLE && VISION_NAV_SIMPLE_FINISH_MODE
    return vision_nav_competition_switch_active;
#else
    return 0u;
#endif
}

uint8_t vision_nav_beacon_is_center_reference(float body_x_cm,
                                               float body_y_cm)
{
#if VISION_NAV_CENTER_BEACON_POLICY_ENABLE && VISION_NAV_SIMPLE_FINISH_MODE
    float rel_x_e;
    float rel_y_e;
    float dx;
    float dy;

    if(vision_nav_competition_switch_active == 0u) return 0u;

    rel_x_e = body_x_cm * vehicle_state.yaw_cos -
              body_y_cm * vehicle_state.yaw_sin;
    rel_y_e = body_x_cm * vehicle_state.yaw_sin +
              body_y_cm * vehicle_state.yaw_cos;
    dx = ekf_lite_state.x + rel_x_e - vision_nav_field_center_x_cm;
    dy = ekf_lite_state.y + rel_y_e - vision_nav_field_center_y_cm;
    return ((dx * dx + dy * dy) <=
            (VISION_NAV_CENTER_BEACON_RADIUS_CM *
             VISION_NAV_CENTER_BEACON_RADIUS_CM)) ? 1u : 0u;
#else
    (void)body_x_cm;
    (void)body_y_cm;
    return 0u;
#endif
}

uint8_t vision_nav_locked_target_is_center(void)
{
#if VISION_NAV_CENTER_BEACON_POLICY_ENABLE && VISION_NAV_SIMPLE_FINISH_MODE
    float dx;
    float dy;

    if((vision_nav_competition_switch_active == 0u) ||
       (vision_nav_obs.anchor_valid == 0u))
    {
        return 0u;
    }

    dx = vision_nav_obs.anchor_x_cm - vision_nav_field_center_x_cm;
    dy = vision_nav_obs.anchor_y_cm - vision_nav_field_center_y_cm;
    return ((dx * dx + dy * dy) <=
            (VISION_NAV_CENTER_BEACON_RADIUS_CM *
             VISION_NAV_CENTER_BEACON_RADIUS_CM)) ? 1u : 0u;
#else
    return 0u;
#endif
}

uint8_t vision_nav_beacon_selection_blocked(void)
{
#if VISION_NAV_CENTER_BEACON_POLICY_ENABLE && VISION_NAV_SIMPLE_FINISH_MODE
    return ((vision_nav_competition_switch_active != 0u) &&
            (vision_nav_center_return_required != 0u) &&
            (vision_nav_obs.search_center_settled == 0u)) ? 1u : 0u;
#else
    return 0u;
#endif
}

static uint8_t vision_nav_cover_loss_likely(uint32_t now_us)
{
    uint8_t car_reports_stopping;
    uint8_t recent_near_target_command;

    if(vision_nav_cover_loss_latched != 0u)
    {
        return 1u;
    }

    if((vision_nav_obs.car_arrived != 0u) ||
       (vision_nav_obs.state == VISION_NAV_CAR_ARRIVED))
    {
        vision_nav_cover_loss_latched = 1u;
        return 1u;
    }

    car_reports_stopping =
        (mcar_comm_feedback_is_fresh(vision_nav_obs.target_seq,
                                     VISION_NAV_MCAR_FEEDBACK_TIMEOUT_US) &&
         ((mcar_comm_feedback.status &
           MCAR_COMM_STATUS_COMM_LOST) == 0u) &&
         ((mcar_comm_feedback.status &
           (MCAR_COMM_STATUS_STOPPING |
            MCAR_COMM_STATUS_ARRIVED |
            MCAR_COMM_STATUS_STOPPED)) != 0u)) ? 1u : 0u;

    recent_near_target_command =
        ((mcar_comm_diag.last_tx_us != 0u) &&
         ((now_us - mcar_comm_diag.last_tx_us) <=
          VISION_NAV_MCAR_FEEDBACK_TIMEOUT_US) &&
         ((mcar_comm_diag.last_tx_flags &
           MCAR_COMM_FLAG_TARGET_ACTIVE) != 0u) &&
         ((mcar_comm_diag.last_tx_flags & MCAR_COMM_FLAG_STOP) == 0u) &&
         (vision_nav_absf((float)mcar_comm_diag.last_tx_err_forward_px) <=
          VISION_NAV_MCAR_ARRIVE_ERR_PX) &&
         (vision_nav_absf((float)mcar_comm_diag.last_tx_err_right_px) <=
          VISION_NAV_MCAR_ARRIVE_ERR_PX)) ? 1u : 0u;

    if((car_reports_stopping != 0u) ||
       (recent_near_target_command != 0u))
    {
        vision_nav_cover_loss_latched = 1u;
        return 1u;
    }

    return 0u;
}

uint8_t vision_nav_beacon_command_held(void)
{
    uint32_t now_us;
    uint32_t lost_us;

    if(vision_nav_competition_switch_active == 0u ||
       beacon.status == BEACON_FOUND ||
       vision_nav_obs.anchor_valid == 0u ||
       vision_nav_obs.state != VISION_NAV_TRACKING ||
       vision_nav_obs.lost_since_us == 0u)
    {
        return 0u;
    }

    now_us = system_time_us();
    lost_us = now_us - vision_nav_obs.lost_since_us;

    /* Near the beacon, a disappearance is probably the car covering it: keep
     * the old command only through the short braking grace period.  Far from
     * the beacon, follow the tracker's bounded COASTING interval so a brief
     * edge/attitude dropout does not alternate TARGET and STOP commands. */
    if(vision_nav_cover_loss_likely(now_us) != 0u)
    {
        return (lost_us < VISION_NAV_BEACON_LOSS_BRAKE_DELAY_US) ? 1u : 0u;
    }

    return ((vision_nav_obs.beacon_track_state ==
             VISION_BEACON_TRACK_COASTING) &&
            (lost_us < VISION_NAV_TRACK_COAST_US)) ? 1u : 0u;
}

uint8_t vision_nav_beacon_command_valid(void)
{
    if(vision_nav_competition_switch_active == 0u)
    {
        return 0u;
    }

    /* A raw detector hit is not automatically the active lamp.  Navigation
     * deliberately keeps the confirmed anchor when a new bright blob jumps
     * farther than the target-switch gate; apply the same gate at the final
     * car-command seam so the Y car cannot reverse on that unconfirmed frame.
     * The bounded held-command path below remains available for a true short
     * dropout, while a visible but inconsistent blob produces an immediate
     * STOP until tracking is reacquired. */
    if((beacon.status == BEACON_FOUND) &&
       (vision_nav_obs.state == VISION_NAV_TRACKING) &&
       (vision_nav_obs.anchor_valid != 0u) &&
       ((vision_nav_absf(vision_nav_obs.observed_beacon_x_cm -
                         vision_nav_obs.anchor_x_cm) >
         VISION_NAV_TARGET_SWITCH_CM) ||
        (vision_nav_absf(vision_nav_obs.observed_beacon_y_cm -
                         vision_nav_obs.anchor_y_cm) >
         VISION_NAV_TARGET_SWITCH_CM)))
    {
        return 0u;
    }

    return (beacon.status == BEACON_FOUND ||
            vision_nav_beacon_command_held() != 0u) ? 1u : 0u;
}

uint8_t vision_nav_flight_ready(void)
{
    return vision_nav_obs.flight_ready;
}

uint8_t vision_nav_yaw_spin_is_active(void)
{
#if VISION_NAV_YAW_SPIN_ENABLE
    return vision_nav_yaw_spin_active;
#else
    return 0u;
#endif
}

uint8_t vision_nav_yaw_spin_is_done(void)
{
#if VISION_NAV_YAW_SPIN_ENABLE
    return vision_nav_yaw_spin_done;
#else
    return 1u;
#endif
}

uint8_t vision_nav_premission_align_ready(void)
{
    return vision_nav_obs.premission_align_ready;
}

/* Keep the car stopped while the aircraft moves above it before task start. */
static void vision_nav_premission_align_update(uint32_t now_us)
{
    static uint32_t aligned_since_us = 0u;
    static uint32_t last_update_us = 0u;
    static uint8_t candidate_frames = 0u;
    static uint8_t anchor_valid = 0u;
    static uint8_t target_valid = 0u;
    static float candidate_x = 0.0f;
    static float candidate_y = 0.0f;
    static float anchor_x = 0.0f;
    static float anchor_y = 0.0f;
    static float target_x = 0.0f;
    static float target_y = 0.0f;
    float rel_body_x;
    float rel_body_y;
    float rel_earth_x;
    float rel_earth_y;
    float observed_x;
    float observed_y;
    float distance;
    float target_distance;
    float dt_s;
    float max_step;
    uint8_t eligible;

#if VISION_NAV_SIMPLE_FINISH_MODE
    /* Last-day completion profile: take off vertically and start the fixed-
     * yaw forward leg from the current point.  Pre-positioning above the car
     * is deliberately removed because it is not needed by the task and was
     * an additional horizontal closed loop before mission start. */
    (void)now_us;
    vision_nav_obs.premission_state = VISION_PREMISSION_WAIT_CRUISE;
    vision_nav_obs.premission_align_ready = 1u;
    return;
#endif

    eligible = ((vehicle_state.armed != 0u) &&
                (vehicle_state.flight_mode == FLY_AUTOTAKEOFF) &&
                (vehicle_state.current_height >=
                 VISION_NAV_PREMISSION_ALIGN_START_HEIGHT_CM) &&
                (vehicle_state.flow_valid != 0u) &&
                (loc_1l_ct.loc_hold_ready != 0u) &&
                (ycar_info.valid != 0u)) ? 1u : 0u;

    vision_nav_obs.premission_align_ready = 0u;
    if(eligible == 0u)
    {
        aligned_since_us = 0u;
        last_update_us = 0u;
        candidate_frames = 0u;
        anchor_valid = 0u;
        target_valid = 0u;
        vision_nav_obs.premission_state =
            (vehicle_state.current_height >=
             VISION_NAV_PREMISSION_ALIGN_START_HEIGHT_CM) ?
            VISION_PREMISSION_ALIGN_CAR : VISION_PREMISSION_WAIT_CRUISE;
        return;
    }

    rel_body_x = ycar_body_x * vehicle_state.current_height *
                 BEACON_SCALE_X + CAMERA_OFFSET_BODY_X_CM;
    rel_body_y = ycar_body_y * vehicle_state.current_height *
                 BEACON_SCALE_Y + CAMERA_OFFSET_BODY_Y_CM;
    rel_earth_x = rel_body_x * vehicle_state.yaw_cos -
                      rel_body_y * vehicle_state.yaw_sin;
    rel_earth_y = rel_body_x * vehicle_state.yaw_sin +
                      rel_body_y * vehicle_state.yaw_cos;
    observed_x = vehicle_state.current_pos_x + rel_earth_x;
    observed_y = vehicle_state.current_pos_y + rel_earth_y;

    if(anchor_valid == 0u)
    {
        if((candidate_frames == 0u) ||
           (sqrtf((observed_x - candidate_x) * (observed_x - candidate_x) +
                  (observed_y - candidate_y) * (observed_y - candidate_y)) <=
            VISION_NAV_PREMISSION_ALIGN_OBS_GATE_CM))
        {
            if(candidate_frames == 0u)
            {
                candidate_x = observed_x;
                candidate_y = observed_y;
            }
            else
            {
                candidate_x = 0.7f * candidate_x + 0.3f * observed_x;
                candidate_y = 0.7f * candidate_y + 0.3f * observed_y;
            }
            if(candidate_frames < 255u) candidate_frames++;
        }
        else
        {
            candidate_x = observed_x;
            candidate_y = observed_y;
            candidate_frames = 1u;
        }

        if(candidate_frames >= VISION_NAV_PREMISSION_ALIGN_CONFIRM_FRAMES)
        {
            anchor_x = candidate_x;
            anchor_y = candidate_y;
            anchor_valid = 1u;
            target_x = vehicle_state.current_pos_x;
            target_y = vehicle_state.current_pos_y;
            target_valid = 1u;
        }
        else
        {
            vehicle_setpoint.target_pos_x = vehicle_state.current_pos_x;
            vehicle_setpoint.target_pos_y = vehicle_state.current_pos_y;
            vehicle_setpoint.target_vel_x = 0.0f;
            vehicle_setpoint.target_vel_y = 0.0f;
            vision_nav_obs.premission_state = VISION_PREMISSION_ALIGN_CAR;
            return;
        }
    }
    else if(sqrtf((observed_x - anchor_x) * (observed_x - anchor_x) +
                  (observed_y - anchor_y) * (observed_y - anchor_y)) <=
            VISION_NAV_PREMISSION_ALIGN_OBS_GATE_CM)
    {
        /* A stopped car's ground position should be stable.  Smooth only
         * consistent observations; one-frame projection jumps are ignored. */
        anchor_x = 0.85f * anchor_x + 0.15f * observed_x;
        anchor_y = 0.85f * anchor_y + 0.15f * observed_y;
    }

    distance = sqrtf((anchor_x - vehicle_state.current_pos_x) *
                     (anchor_x - vehicle_state.current_pos_x) +
                     (anchor_y - vehicle_state.current_pos_y) *
                     (anchor_y - vehicle_state.current_pos_y));

    if(distance <= VISION_NAV_PREMISSION_ALIGN_ERR_CM)
    {
        if(aligned_since_us == 0u) aligned_since_us = now_us;
        if((now_us - aligned_since_us) >=
           VISION_NAV_PREMISSION_ALIGN_SETTLE_US)
        {
            vision_nav_obs.premission_state = VISION_PREMISSION_ALIGNED;
            vision_nav_obs.premission_align_ready = 1u;
            return;
        }
    }
    else
    {
        aligned_since_us = 0u;
    }

    if(last_update_us == 0u)
    {
        last_update_us = now_us;
    }
    dt_s = (float)(now_us - last_update_us) / 1000000.0f;
    last_update_us = now_us;
    dt_s = vision_nav_clampf(dt_s, 0.0f, 0.10f);
    max_step = VISION_NAV_PREMISSION_ALIGN_TARGET_SPEED_CM_S * dt_s;
    target_distance = sqrtf((anchor_x - target_x) * (anchor_x - target_x) +
                            (anchor_y - target_y) * (anchor_y - target_y));
    if((target_valid == 0u) || (target_distance <= max_step))
    {
        target_x = anchor_x;
        target_y = anchor_y;
        target_valid = 1u;
    }
    else if(target_distance > 0.001f)
    {
        target_x += (anchor_x - target_x) * max_step / target_distance;
        target_y += (anchor_y - target_y) * max_step / target_distance;
    }
    vehicle_setpoint.target_pos_x = target_x;
    vehicle_setpoint.target_pos_y = target_y;
    vehicle_setpoint.target_vel_x = 0.0f;
    vehicle_setpoint.target_vel_y = 0.0f;
    vision_nav_obs.premission_state = VISION_PREMISSION_ALIGN_CAR;
}

void vision_nav_update(void)
{
    float rel_x_e = 0.0f;
    float rel_y_e = 0.0f;
    float observed_beacon_x = 0.0f;
    float observed_beacon_y = 0.0f;
    float jump_x;
    float jump_y;
    uint32_t now_us = system_time_us();
    uint8_t new_vision_frame =
        (vision_detection_snapshot.frame_id !=
         vision_nav_beacon_last_frame_id) ? 1u : 0u;
    uint8_t competition_active = vision_nav_competition_update();
    uint8_t geometry_ok = (competition_active != 0u) ?
        vision_nav_geometry_ok() : 0u;
    uint8_t beacon_camera_seen =
        ((competition_active != 0u) &&
         (beacon.status == BEACON_FOUND)) ? 1u : 0u;
    uint8_t feedback_fresh = mcar_comm_feedback_is_fresh(
        vision_nav_obs.target_seq, VISION_NAV_MCAR_FEEDBACK_TIMEOUT_US);

    if(new_vision_frame != 0u)
    {
        vision_nav_beacon_last_frame_id =
            vision_detection_snapshot.frame_id;
    }

    vision_nav_clear_measurement();
    vision_nav_update_horizontal_fault(now_us);
    vision_nav_update_flight_readiness(competition_active);
    vision_nav_obs.geometry_ok = geometry_ok;
    vision_nav_obs.feedback_fresh = feedback_fresh;

    /*
     * The blind-box opening spin owns navigation until one full yaw turn has
     * completed.  This keeps the cable, aircraft position and Y-car command
     * path quiet while the attitude controller performs the rotation.
     */
    if(vision_nav_yaw_spin_update(now_us) != 0u)
    {
        return;
    }

#if !VISION_NAV_SIMPLE_DIRECT_CAR_FOLLOW
    if((vision_nav_obs.state == VISION_NAV_TRACKING ||
        vision_nav_obs.state == VISION_NAV_CAR_ARRIVED) &&
       feedback_fresh &&
       (mcar_comm_feedback.status & MCAR_COMM_STATUS_ARRIVED) != 0u &&
       (mcar_comm_feedback.status & MCAR_COMM_STATUS_STOPPED) != 0u &&
       (mcar_comm_feedback.status & MCAR_COMM_STATUS_SPEED_VALID) != 0u &&
       (mcar_comm_feedback.status & MCAR_COMM_STATUS_COMM_LOST) == 0u)
    {
        vision_nav_obs.car_arrived = 1u;
        if(vision_nav_obs.state == VISION_NAV_TRACKING)
        {
            vision_nav_obs.state = VISION_NAV_CAR_ARRIVED;
        }
    }
#endif

    if(geometry_ok)
    {
        rel_x_e = beacon_drone_x_cm * vehicle_state.yaw_cos -
                  beacon_drone_y_cm * vehicle_state.yaw_sin;
        rel_y_e = beacon_drone_x_cm * vehicle_state.yaw_sin +
                  beacon_drone_y_cm * vehicle_state.yaw_cos;
        observed_beacon_x = ekf_lite_state.x + rel_x_e;
        observed_beacon_y = ekf_lite_state.y + rel_y_e;
        vision_nav_obs.rel_earth_x_cm = rel_x_e;
        vision_nav_obs.rel_earth_y_cm = rel_y_e;
        vision_nav_obs.observed_beacon_x_cm = observed_beacon_x;
        vision_nav_obs.observed_beacon_y_cm = observed_beacon_y;
        if(new_vision_frame != 0u)
        {
            vision_nav_update_beacon_track(1u, observed_beacon_x,
                                           observed_beacon_y, now_us);
        }

        if(vision_nav_obs.state == VISION_NAV_DONE)
        {
            vision_nav_start_search();
        }

        if(vision_nav_obs.state == VISION_NAV_SEARCH)
        {
            vision_nav_obs.stable_frames =
                vision_nav_obs.beacon_track_seen_frames;
            vision_nav_obs.pending_anchor_x_cm =
                vision_nav_obs.beacon_track_x_cm;
            vision_nav_obs.pending_anchor_y_cm =
                vision_nav_obs.beacon_track_y_cm;
            if((vision_nav_obs.flight_ready != 0u) &&
               (vision_nav_obs.beacon_track_state ==
                VISION_BEACON_TRACK_CONFIRMED))
            {
                vision_nav_begin_target(vision_nav_obs.beacon_track_x_cm,
                                        vision_nav_obs.beacon_track_y_cm);
            }
        }
        else if(vision_nav_obs.anchor_valid)
        {
            jump_x = observed_beacon_x - vision_nav_obs.anchor_x_cm;
            jump_y = observed_beacon_y - vision_nav_obs.anchor_y_cm;

            if(vision_nav_absf(jump_x) > VISION_NAV_TARGET_SWITCH_CM ||
               vision_nav_absf(jump_y) > VISION_NAV_TARGET_SWITCH_CM)
            {
                if(vision_nav_obs.car_arrived)
                {
                    vision_nav_obs.target_done = 1u;
                    vision_nav_obs.state = VISION_NAV_DONE;
                }
                else
                {
                    /* A one-frame geometry jump is an invalid observation,
                     * not a new target. Keep the confirmed anchor/sequence
                     * until the existing lost-target timeout expires. */
                    if(vision_nav_obs.lost_since_us == 0u)
                    {
                        vision_nav_obs.lost_since_us = now_us;
                    }
                    else if(now_us - vision_nav_obs.lost_since_us >=
                            VISION_NAV_TARGET_LOST_RESET_US)
                    {
                        vision_nav_start_search();
                    }
                }
            }
            else
            {
                float drone_x = vision_nav_obs.anchor_x_cm - rel_x_e;
                float drone_y = vision_nav_obs.anchor_y_cm - rel_y_e;
                float residual_x = drone_x - ekf_lite_state.x;
                float residual_y = drone_y - ekf_lite_state.y;

                vision_nav_obs.lost_since_us = 0u;
                vision_nav_obs.drone_x_cm = drone_x;
                vision_nav_obs.drone_y_cm = drone_y;
                vision_nav_obs.residual_x_cm = residual_x;
                vision_nav_obs.residual_y_cm = residual_y;
                vision_nav_obs.confidence = 1.0f;
                vision_nav_obs.valid = (vision_nav_absf(residual_x) <= VISION_NAV_MAX_RESIDUAL_CM &&
                                        vision_nav_absf(residual_y) <= VISION_NAV_MAX_RESIDUAL_CM) ? 1u : 0u;

                if(vision_nav_obs.valid &&
                   now_us - vision_nav_obs.last_fusion_us >= VISION_NAV_FUSION_INTERVAL_US)
                {
                    vision_nav_obs.last_fusion_us = now_us;
                    vision_nav_apply_ekf(residual_x, residual_y, 1.0f);
                }
            }
        }
    }

    else if(vision_nav_obs.anchor_valid)
    {
        if(new_vision_frame != 0u)
        {
            vision_nav_update_beacon_track(0u, 0.0f, 0.0f, now_us);
        }
        if(vision_nav_obs.lost_since_us == 0u) vision_nav_obs.lost_since_us = now_us;

        if(vision_nav_obs.car_arrived &&
           now_us - vision_nav_obs.lost_since_us >= VISION_NAV_BEACON_OFF_CONFIRM_US)
        {
            vision_nav_obs.target_done = 1u;
            vision_nav_obs.state = VISION_NAV_DONE;
            vision_nav_obs.anchor_valid = 0u;
        }
        else if(!vision_nav_obs.car_arrived &&
                now_us - vision_nav_obs.lost_since_us >= VISION_NAV_TARGET_LOST_RESET_US)
        {
            vision_nav_start_search();
        }
    }
    else
    {
        if(new_vision_frame != 0u)
        {
            vision_nav_update_beacon_track(0u, 0.0f, 0.0f, now_us);
        }
    }

    /* Enter WAIT_NEXT/SEARCH as soon as lamp-off confirmation completes.
     * Simple-finish mode first holds for one second; the optional local path
     * is currently disabled, so the aircraft then remains at that centre. */
    if(vision_nav_obs.state == VISION_NAV_DONE)
    {
        vision_nav_start_search();
    }

    if((beacon_camera_seen != 0u) ||
       (vision_nav_obs.anchor_valid == 0u) ||
       (vision_nav_obs.state != VISION_NAV_TRACKING) ||
       (vision_nav_obs.lost_since_us == 0u))
    {
        vision_nav_cover_loss_latched = 0u;
    }

    vision_nav_obs.beacon_loss_brake_active =
        ((vision_nav_obs.anchor_valid != 0u) &&
         (vision_nav_obs.state == VISION_NAV_TRACKING) &&
         (beacon_camera_seen == 0u) &&
         (vision_nav_cover_loss_likely(now_us) != 0u) &&
         (vision_nav_obs.lost_since_us != 0u) &&
         ((now_us - vision_nav_obs.lost_since_us) >=
          VISION_NAV_BEACON_LOSS_BRAKE_DELAY_US)) ? 1u : 0u;

    vision_nav_beacon_approach_update(geometry_ok,
                                      observed_beacon_x,
                                      observed_beacon_y,
                                      rel_x_e,
                                      rel_y_e);
    vision_nav_fast_search_update(beacon_camera_seen, now_us);
    vision_nav_follow_car_update(geometry_ok, now_us);
    vision_nav_premission_align_update(now_us);

    {
        static uint32_t last_print_us = 0u;
        if(now_us - last_print_us >= VISION_NAV_PRINT_INTERVAL_US)
        {
            last_print_us = now_us;
            uint32_t lost_us = (vision_nav_obs.lost_since_us == 0u) ? 0u :
                               now_us - vision_nav_obs.lost_since_us;
            /*printf("VNAV:t=%lu,state=%u,seq=%u,geom=%u,anchor=%u,car_arr=%u,done=%u,"
                   "h=%.1f,rpy=(%.2f,%.2f,%.2f),px_raw=(%d,%d),px_corr=(%.1f,%.1f),"
                   "rel_body=(%.1f,%.1f),rel_earth=(%.1f,%.1f),anchor_xy=(%.1f,%.1f),"
                   "beacon_obs=(%.1f,%.1f),drone_obs=(%.1f,%.1f),ekf=(%.1f,%.1f),"
                   "res=(%.1f,%.1f),valid=%u,used=%u,corr=(%.2f,%.2f),lost_us=%lu,"
                   "search=(%u,%u,%u),goal=(%.1f,%.1f),"
                   "fb=(%u,%u,%u,0x%02X,%d,%d)\r\n",
                   (unsigned long)now_us,
                   vision_nav_obs.state, vision_nav_obs.target_seq,
                   vision_nav_obs.geometry_ok, vision_nav_obs.anchor_valid,
                   vision_nav_obs.car_arrived,
                   vision_nav_obs.target_done,
                   vehicle_state.current_height,
                   vehicle_state.current_roll,
                   vehicle_state.current_pitch,
                   imu_data.yaw,
                   beacon.centerX, beacon.centerY,
                   beacon_corr_x, beacon_corr_y,
                   beacon_drone_x_cm, beacon_drone_y_cm,
                   vision_nav_obs.rel_earth_x_cm,
                   vision_nav_obs.rel_earth_y_cm,
                   vision_nav_obs.anchor_x_cm,
                   vision_nav_obs.anchor_y_cm,
                   vision_nav_obs.observed_beacon_x_cm,
                   vision_nav_obs.observed_beacon_y_cm,
                   vision_nav_obs.drone_x_cm, vision_nav_obs.drone_y_cm,
                   ekf_lite_state.x, ekf_lite_state.y,
                   vision_nav_obs.residual_x_cm, vision_nav_obs.residual_y_cm,
                   vision_nav_obs.valid, vision_nav_obs.used_for_ekf,
                   vision_nav_obs.ekf_correction_x_cm,
                   vision_nav_obs.ekf_correction_y_cm,
                   (unsigned long)lost_us,
                   vision_nav_obs.search_move_active,
                   vision_nav_obs.search_lost_frames,
                   vision_nav_obs.search_found_frames,
                   vehicle_setpoint.target_pos_x,
                   vehicle_setpoint.target_pos_y,
                   mcar_comm_feedback.valid,
                   vision_nav_obs.feedback_fresh,
                   mcar_comm_feedback.target_seq,
                   mcar_comm_feedback.status,
                   mcar_comm_feedback.speed_forward_cm_s,
                   mcar_comm_feedback.speed_right_cm_s);*/
        }
    }
}
