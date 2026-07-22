#include "vision_nav.h"
#include "beacon.h"
#include "ekf_lite.h"
#include "mcar_comm.h"
#include "param.h"

vision_nav_obs_t vision_nav_obs = {0};
static uint8_t vision_nav_hold_valid = 0u;
static float vision_nav_hold_x = 0.0f;
static float vision_nav_hold_y = 0.0f;

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

static void vision_nav_fast_search_update(uint8_t geometry_ok)
{
#if VISION_NAV_FAST_SEARCH_ENABLE && !BEACON_APPROACH_TEST_ENABLE
    uint8_t eligible = ((vehicle_state.armed != 0u) &&
                        (vehicle_state.flight_mode == FLY_AUTOFLY) &&
                        (vehicle_state.flow_valid != 0u) &&
                        (vehicle_state.current_height >=
                         VISION_NAV_SEARCH_MIN_HEIGHT_CM) &&
                        (vision_nav_obs.car_arrived == 0u)) ? 1u : 0u;

    if(eligible == 0u)
    {
        vision_nav_obs.search_move_active = 0u;
        vision_nav_obs.search_lost_frames = 0u;
        vision_nav_obs.search_found_frames = 0u;
        return;
    }

    if(geometry_ok != 0u)
    {
        vision_nav_obs.search_lost_frames = 0u;
        if(vision_nav_obs.search_found_frames < 255u)
        {
            vision_nav_obs.search_found_frames++;
        }

        if((vision_nav_obs.search_move_active != 0u) &&
           (vision_nav_obs.search_found_frames >=
            VISION_NAV_SEARCH_REACQUIRE_FRAMES))
        {
            vision_nav_obs.search_move_active = 0u;
            vehicle_setpoint.target_pos_x = vehicle_state.current_pos_x;
            vehicle_setpoint.target_pos_y = vehicle_state.current_pos_y;
            vehicle_setpoint.target_vel_x = 0.0f;
            vehicle_setpoint.target_vel_y = 0.0f;
        }
    }
    else
    {
        vision_nav_obs.search_found_frames = 0u;
        if(vision_nav_obs.search_lost_frames < 255u)
        {
            vision_nav_obs.search_lost_frames++;
        }
        if(vision_nav_obs.search_lost_frames >= VISION_NAV_SEARCH_LOST_FRAMES)
        {
            vision_nav_obs.search_move_active = 1u;
        }
    }

    if(vision_nav_obs.search_move_active != 0u)
    {
        vehicle_setpoint.target_pos_x = VISION_NAV_SEARCH_CENTER_X_CM;
        vehicle_setpoint.target_pos_y = VISION_NAV_SEARCH_CENTER_Y_CM;
        vehicle_setpoint.target_vel_x = 0.0f;
        vehicle_setpoint.target_vel_y = 0.0f;
    }
#else
    (void)geometry_ok;
#endif
}

static void vision_nav_follow_car_update(uint8_t beacon_geometry_ok)
{
#if VISION_MISSION_MODE_ENABLE
    float car_rel_body_x;
    float car_rel_body_y;
    float car_rel_earth_x;
    float car_rel_earth_y;
    float distance;
    uint8_t eligible;

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
    vision_nav_obs.car_distance_cm = distance;

    /*
     * Search, takeoff, landing and completed-target states own the waypoint.
     * Drop the leash without writing a hold target so those states keep it.
     */
    if((vision_nav_obs.search_move_active != 0u) ||
       (vehicle_state.flight_mode != FLY_AUTOFLY) ||
       (vision_nav_obs.car_arrived != 0u) ||
       (vision_nav_obs.target_done != 0u))
    {
        vision_nav_obs.car_follow_active = 0u;
        vision_nav_release_hold_position();
        return;
    }

    eligible = ((vehicle_state.armed != 0u) &&
                (vehicle_state.flow_valid != 0u) &&
                (loc_1l_ct.loc_hold_ready != 0u) &&
                (beacon_geometry_ok != 0u) &&
                (ycar_info.valid != 0u) &&
                (vision_nav_obs.state == VISION_NAV_TRACKING)) ? 1u : 0u;

    if(eligible == 0u)
    {
        if(vision_nav_obs.car_follow_active != 0u)
        {
            vision_nav_obs.car_follow_active = 0u;
            vision_nav_release_hold_position();
            vision_nav_hold_current_position();
        }
        return;
    }

    if(vision_nav_obs.car_follow_active == 0u)
    {
        float first_step;
        float scale;

        if(distance <= VISION_NAV_CAR_FOLLOW_START_CM)
        {
            return;
        }

        vision_nav_release_hold_position();
        vision_nav_obs.car_follow_active = 1u;
        first_step = distance - VISION_NAV_CAR_FOLLOW_STOP_CM;
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
    else if(distance <= VISION_NAV_CAR_FOLLOW_STOP_CM)
    {
        vision_nav_obs.car_follow_active = 0u;
        vision_nav_release_hold_position();
        vision_nav_hold_current_position();
    }
    else
    {
        float desired_x = vehicle_state.current_pos_x +
            car_rel_earth_x *
            ((distance - VISION_NAV_CAR_FOLLOW_STOP_CM) / distance);
        float desired_y = vehicle_state.current_pos_y +
            car_rel_earth_y *
            ((distance - VISION_NAV_CAR_FOLLOW_STOP_CM) / distance);
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

    vehicle_setpoint.target_vel_x = 0.0f;
    vehicle_setpoint.target_vel_y = 0.0f;
#else
    (void)beacon_geometry_ok;
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
    vision_nav_obs.state = VISION_NAV_SEARCH;
    vision_nav_obs.anchor_valid = 0u;
    vision_nav_obs.car_arrived = 0u;
    vision_nav_obs.target_done = 0u;
    vision_nav_obs.stable_frames = 0u;
    vision_nav_obs.lost_since_us = 0u;
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

void vision_nav_update(void)
{
    float rel_x_e = 0.0f;
    float rel_y_e = 0.0f;
    float observed_beacon_x = 0.0f;
    float observed_beacon_y = 0.0f;
    float jump_x;
    float jump_y;
    uint32_t now_us = system_time_us();
    uint8_t geometry_ok = vision_nav_geometry_ok();
    uint8_t feedback_fresh = mcar_comm_feedback_is_fresh(
        vision_nav_obs.target_seq, VISION_NAV_MCAR_FEEDBACK_TIMEOUT_US);

    vision_nav_clear_measurement();
    vision_nav_obs.geometry_ok = geometry_ok;
    vision_nav_obs.feedback_fresh = feedback_fresh;

    if(feedback_fresh &&
       (mcar_comm_feedback.status & MCAR_COMM_STATUS_ARRIVED) != 0u &&
       vision_nav_absf((float)mcar_comm_feedback.err_forward_px) <= VISION_NAV_MCAR_ARRIVE_ERR_PX &&
       vision_nav_absf((float)mcar_comm_feedback.err_right_px) <= VISION_NAV_MCAR_ARRIVE_ERR_PX)
    {
        vision_nav_obs.car_arrived = 1u;
        if(vision_nav_obs.state == VISION_NAV_TRACKING)
        {
            vision_nav_obs.state = VISION_NAV_CAR_ARRIVED;
        }
    }

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

        if(vision_nav_obs.state == VISION_NAV_DONE)
        {
            vision_nav_start_search();
        }

        if(vision_nav_obs.state == VISION_NAV_SEARCH)
        {
            if(vision_nav_obs.stable_frames == 0u)
            {
                vision_nav_obs.pending_anchor_x_cm = observed_beacon_x;
                vision_nav_obs.pending_anchor_y_cm = observed_beacon_y;
                vision_nav_obs.stable_frames = 1u;
            }
            else
            {
                jump_x = observed_beacon_x - vision_nav_obs.pending_anchor_x_cm;
                jump_y = observed_beacon_y - vision_nav_obs.pending_anchor_y_cm;
                if(vision_nav_absf(jump_x) <= VISION_NAV_ANCHOR_STABLE_CM &&
                   vision_nav_absf(jump_y) <= VISION_NAV_ANCHOR_STABLE_CM)
                {
                    vision_nav_obs.pending_anchor_x_cm =
                        0.7f * vision_nav_obs.pending_anchor_x_cm + 0.3f * observed_beacon_x;
                    vision_nav_obs.pending_anchor_y_cm =
                        0.7f * vision_nav_obs.pending_anchor_y_cm + 0.3f * observed_beacon_y;
                    if(vision_nav_obs.stable_frames < 255u) vision_nav_obs.stable_frames++;
                    if(vision_nav_obs.stable_frames >= VISION_NAV_MIN_STABLE_FRAMES)
                    {
                        vision_nav_begin_target(vision_nav_obs.pending_anchor_x_cm,
                                                vision_nav_obs.pending_anchor_y_cm);
                    }
                }
                else
                {
                    vision_nav_obs.pending_anchor_x_cm = observed_beacon_x;
                    vision_nav_obs.pending_anchor_y_cm = observed_beacon_y;
                    vision_nav_obs.stable_frames = 1u;
                }
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
                    vision_nav_start_search();
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

    vision_nav_beacon_approach_update(geometry_ok,
                                      observed_beacon_x,
                                      observed_beacon_y,
                                      rel_x_e,
                                      rel_y_e);
    vision_nav_fast_search_update(geometry_ok);
    vision_nav_follow_car_update(geometry_ok);

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
                   mcar_comm_feedback.err_forward_px,
                   mcar_comm_feedback.err_right_px);*/
        }
    }
}
