#include "zf_common_headfile.h"
#include "vision_nav.h"


// ============================================================================
// Runtime state
// ============================================================================

location_control_t loc_ctrl = {0};

loc_2l_ct_t loc_2l_ct = {0};
loc_1l_ct_t loc_1l_ct = {0};
loc_output_t loc_output = {0};

typedef struct
{
    flight_mode_t last_flight_mode;
    float target_vel_x_ramped;
    float target_vel_y_ramped;
    float profile_pos_x;
    float profile_pos_y;
    float profile_vel_x;
    float profile_vel_y;
    uint8_t profile_valid;
    float target_roll_ramped;
    float target_pitch_ramped;
    float vel_err_x_body_filtered;
    float vel_err_y_body_filtered;
    float loc_weight;
    float loc_ready_time_s;
    float brake_weight;
    uint8_t loc_ready_last;
    uint8_t loc_hold_active;
    float loc_hold_stable_time_s;
    float loc_hold_time_s;
    float vel_i_yaw_sin;
    float vel_i_yaw_cos;
    uint8_t vel_i_yaw_valid;
    uint8_t vel_pid_saturated_x;
    uint8_t vel_pid_saturated_y;
} loc_runtime_t;

static loc_runtime_t loc_rt =
{
    .last_flight_mode = FLY_POS_HOLD
};
static uint8_t loc_horizontal_fault_last = 0u;


// ============================================================================
// Mode helpers
// ============================================================================

static uint8_t loc_control_enabled(void)
{
    return ((vehicle_state.armed != 0u) && (vehicle_state.flow_valid != 0u));
}

static uint8_t loc_mode_enabled(void)
{
    switch(vehicle_state.flight_mode)
    {
        case FLY_POS_HOLD:
        case FLY_AUTOFLY:
        case FLY_AUTOLANDING:
        case FLY_AUTOTAKEOFF:
            return 1u;

        case FLY_HEIGHT_HOLD:
        default:
            return 0u;
    }
}

static uint8_t loc_output_base_ready(void)
{
    return ((loc_control_enabled() != 0u) &&
            (loc_mode_enabled() != 0u));
}

static uint8_t loc_auto_low_height_mode(void)
{
    return ((vehicle_state.flight_mode == FLY_AUTOTAKEOFF) ||
            (vehicle_state.flight_mode == FLY_AUTOLANDING)) ? 1u : 0u;
}

static uint8_t loc_auto_takeoff_mode(void)
{
    return (vehicle_state.flight_mode == FLY_AUTOTAKEOFF) ? 1u : 0u;
}

static float loc_damping_enable_height(void)
{
    if(loc_auto_takeoff_mode() != 0u)
    {
        if((loc_rt.loc_ready_last != 0u) ||
           (loc_rt.loc_hold_active != 0u))
        {
            return LOC_AUTO_TAKEOFF_HORIZONTAL_RELEASE_HEIGHT_CM;
        }
        return LOC_AUTO_TAKEOFF_HORIZONTAL_ENABLE_HEIGHT_CM;
    }

    return (loc_auto_low_height_mode() != 0u) ?
           LOC_AUTO_LOW_ENABLE_HEIGHT_CM : LOC_ENABLE_HEIGHT_CM;
}

static uint8_t loc_damping_ready(void)
{
    return ((loc_output_base_ready() != 0u) &&
            (vehicle_state.current_height >= loc_damping_enable_height()));
}

static float loc_hold_enable_height(void)
{
    /* Horizontal authority must not move upward with a higher altitude goal.
     * Start damping at LOC_ENABLE_HEIGHT_CM, then allow position capture at a
     * fixed, proven height while the altitude profile continues climbing. */
    if(loc_auto_takeoff_mode() != 0u)
    {
        return LOC_AUTO_LOW_HOLD_HEIGHT_CM;
    }

    return (loc_auto_low_height_mode() != 0u) ?
           LOC_AUTO_LOW_LANDING_HOLD_HEIGHT_CM : LOC_HOLD_ENABLE_HEIGHT_CM;
}

static uint8_t loc_hold_entry_ready(void)
{
    float enable_height = loc_hold_enable_height();
    float horiz_spd = sqrtf(vehicle_state.current_vel_x * vehicle_state.current_vel_x +
                            vehicle_state.current_vel_y * vehicle_state.current_vel_y);

    float vz_limit = (loc_auto_low_height_mode() != 0u) ?
                     LOC_AUTO_LOW_ENABLE_VZ_MAX_CM_S : LOC_ENABLE_VZ_MAX_CM_S;

    if((loc_damping_ready() == 0u) ||
       (vehicle_state.current_height < enable_height) ||
       (fabsf(vehicle_state.current_vel_z) > vz_limit) ||
       (fabsf(vehicle_state.current_roll) > LOC_ENABLE_ATT_MAX_DEG) ||
       (fabsf(vehicle_state.current_pitch) > LOC_ENABLE_ATT_MAX_DEG))
    {
        return 0u;
    }

    /*
     * Automatic modes capture the current point when hold starts.  Once the
     * height, vertical-speed and attitude gates are valid, waiting for low
     * horizontal speed can deadlock takeoff after an early disturbance.
     */
    if(loc_auto_low_height_mode() != 0u)
    {
        return 1u;
    }

    return (horiz_spd <= LOC_ENABLE_HORIZ_VEL_MAX_CM_S) ? 1u : 0u;
}

static uint8_t loc_hold_should_release(void)
{
    float release_height =
        loc_hold_enable_height() - LOC_HOLD_RELEASE_MARGIN_CM;

    if(release_height < loc_damping_enable_height())
    {
        release_height = loc_damping_enable_height();
    }

    return ((loc_damping_ready() == 0u) ||
            (vehicle_state.current_height < release_height));
}

static uint8_t loc_hold_ready(void)
{
    return loc_rt.loc_hold_active;
}

static uint8_t loc_output_ready(void)
{
    return loc_damping_ready();
}

static void loc_reset_pos_pids(void)
{
    stan_pid_reset(&loc_ctrl.pos_pid[0]);
    stan_pid_reset(&loc_ctrl.pos_pid[1]);
}

static void loc_reset_vel_pids(void)
{
    stan_pid_reset(&loc_ctrl.vel_pid[0]);
    stan_pid_reset(&loc_ctrl.vel_pid[1]);
    loc_rt.vel_i_yaw_valid = 0u;
    loc_rt.vel_pid_saturated_x = 0u;
    loc_rt.vel_pid_saturated_y = 0u;
}

/*
 * Damping and position hold are different control regimes.  The velocity
 * integrator accumulated while arresting the takeoff drift must not bias the
 * newly captured position target.  Keep the derivative history intact here:
 * clearing it would create an artificial derivative transient at hold entry.
 */
static void loc_clear_vel_pid_integrators(void)
{
    loc_ctrl.vel_pid[0].out_i = 0.0f;
    loc_ctrl.vel_pid[1].out_i = 0.0f;
    loc_rt.vel_pid_saturated_x = 0u;
    loc_rt.vel_pid_saturated_y = 0u;
}

static void loc_reset_pids(void)
{
    loc_reset_pos_pids();
    loc_reset_vel_pids();
}

static void loc_reset_runtime_output(void)
{
    loc_rt.target_vel_x_ramped = 0.0f;
    loc_rt.target_vel_y_ramped = 0.0f;
    loc_rt.target_roll_ramped = 0.0f;
    loc_rt.target_pitch_ramped = 0.0f;
    loc_rt.vel_err_x_body_filtered = 0.0f;
    loc_rt.vel_err_y_body_filtered = 0.0f;
    loc_rt.brake_weight = 0.0f;
}

static void loc_reset_position_profile(float pos_x, float pos_y)
{
    loc_rt.profile_pos_x = pos_x;
    loc_rt.profile_pos_y = pos_y;
    loc_rt.profile_vel_x = 0.0f;
    loc_rt.profile_vel_y = 0.0f;
    loc_rt.profile_valid = 1u;
    loc_2l_ct.profile_vel_x = 0.0f;
    loc_2l_ct.profile_vel_y = 0.0f;
}

/* Vector trapezoidal profile for a dynamically changing earth-frame goal.
 * The leash keeps the reference reachable after a disturbance. */
static void loc_update_position_profile(float goal_x,
                                        float goal_y,
                                        float dT_s)
{
    float tracking_x;
    float tracking_y;
    float tracking_distance;
    float goal_x_from_vehicle;
    float goal_y_from_vehicle;
    float goal_distance_from_vehicle;
    float dx;
    float dy;
    float distance;
    float speed_blend;
    float profile_speed_limit;
    float desired_speed;
    float tracking_scale;
    float desired_vx;
    float desired_vy;
    float dv_x;
    float dv_y;
    float dv;
    float max_dv;
    float next_x;
    float next_y;

    if(loc_rt.profile_valid == 0u)
    {
        loc_reset_position_profile(vehicle_state.current_pos_x,
                                   vehicle_state.current_pos_y);
    }

    tracking_x = loc_rt.profile_pos_x - vehicle_state.current_pos_x;
    tracking_y = loc_rt.profile_pos_y - vehicle_state.current_pos_y;
    tracking_distance = sqrtf(tracking_x * tracking_x + tracking_y * tracking_y);
    goal_x_from_vehicle = goal_x - vehicle_state.current_pos_x;
    goal_y_from_vehicle = goal_y - vehicle_state.current_pos_y;
    goal_distance_from_vehicle = sqrtf(goal_x_from_vehicle * goal_x_from_vehicle +
                                       goal_y_from_vehicle * goal_y_from_vehicle);

    if((tracking_distance > LOC_PROFILE_LEASH_START_CM) &&
       (goal_distance_from_vehicle > 0.05f))
    {
        float reference_distance = LIMIT(tracking_distance,
                                         0.0f,
                                         LOC_PROFILE_LEASH_MAX_CM);
        float goal_unit_x = goal_x_from_vehicle / goal_distance_from_vehicle;
        float goal_unit_y = goal_y_from_vehicle / goal_distance_from_vehicle;
        float along_goal_velocity;

        if(reference_distance > goal_distance_from_vehicle)
        {
            reference_distance = goal_distance_from_vehicle;
        }

        loc_rt.profile_pos_x = vehicle_state.current_pos_x +
                               goal_unit_x * reference_distance;
        loc_rt.profile_pos_y = vehicle_state.current_pos_y +
                               goal_unit_y * reference_distance;
        along_goal_velocity = loc_rt.profile_vel_x * goal_unit_x +
                              loc_rt.profile_vel_y * goal_unit_y;
        if(along_goal_velocity < 0.0f)
        {
            along_goal_velocity = 0.0f;
        }
        loc_rt.profile_vel_x = along_goal_velocity * goal_unit_x;
        loc_rt.profile_vel_y = along_goal_velocity * goal_unit_y;
        tracking_distance = reference_distance;
    }
    else if(goal_distance_from_vehicle <= 0.05f)
    {
        loc_rt.profile_pos_x = goal_x;
        loc_rt.profile_pos_y = goal_y;
        loc_rt.profile_vel_x = 0.0f;
        loc_rt.profile_vel_y = 0.0f;
        tracking_distance = 0.0f;
    }

    dx = goal_x - loc_rt.profile_pos_x;
    dy = goal_y - loc_rt.profile_pos_y;
    distance = sqrtf(dx * dx + dy * dy);
    if(distance <= 0.05f)
    {
        loc_rt.profile_pos_x = goal_x;
        loc_rt.profile_pos_y = goal_y;
        loc_rt.profile_vel_x = 0.0f;
        loc_rt.profile_vel_y = 0.0f;
        loc_2l_ct.profile_vel_x = 0.0f;
        loc_2l_ct.profile_vel_y = 0.0f;
        return;
    }

    speed_blend = ctrl_smoothstep01(
        (distance - LOC_PROFILE_BLEND_START_CM) /
        (LOC_PROFILE_BLEND_END_CM - LOC_PROFILE_BLEND_START_CM));
    profile_speed_limit = LOC_PROFILE_NEAR_SPEED_CM_S +
        (LOC_PROFILE_FAR_SPEED_CM_S - LOC_PROFILE_NEAR_SPEED_CM_S) * speed_blend;
    if((vehicle_state.flight_mode == FLY_AUTOFLY) &&
       (vision_nav_obs.search_move_active != 0u) &&
       (profile_speed_limit > VISION_NAV_SEARCH_SPEED_LIMIT_CM_S))
    {
        profile_speed_limit = VISION_NAV_SEARCH_SPEED_LIMIT_CM_S;
    }
    if((vehicle_state.flight_mode == FLY_AUTOFLY) &&
       (vision_nav_obs.approach_active != 0u) &&
       (profile_speed_limit > VISION_NAV_APPROACH_SPEED_LIMIT_CM_S))
    {
        profile_speed_limit = VISION_NAV_APPROACH_SPEED_LIMIT_CM_S;
    }
    if((vehicle_state.flight_mode == FLY_AUTOFLY) &&
       (vision_nav_obs.car_follow_active != 0u) &&
       (profile_speed_limit > VISION_NAV_CAR_FOLLOW_SPEED_LIMIT_CM_S))
    {
        profile_speed_limit = VISION_NAV_CAR_FOLLOW_SPEED_LIMIT_CM_S;
    }
    desired_speed = LIMIT(sqrtf(2.0f * LOC_TRAJ_ACCEL_CM_S2 * distance),
                          0.0f,
                          profile_speed_limit);
    tracking_scale = 1.0f - ctrl_smoothstep01(
        (tracking_distance - LOC_PROFILE_LEASH_START_CM) /
        (LOC_PROFILE_LEASH_MAX_CM - LOC_PROFILE_LEASH_START_CM));
    tracking_scale = LOC_PROFILE_LEASH_MIN_SPEED_SCALE +
                     (1.0f - LOC_PROFILE_LEASH_MIN_SPEED_SCALE) *
                     tracking_scale;
    desired_speed *= tracking_scale;
    desired_vx = desired_speed * dx / distance;
    desired_vy = desired_speed * dy / distance;

    if((vehicle_state.flight_mode == FLY_AUTOFLY) &&
       (vision_nav_obs.search_move_active != 0u))
    {
        float search_center_x;
        float search_center_y;
        float min_x;
        float max_x;
        float min_y;
        float max_y;
        float boundary_distance;
        float outward_speed_limit;

        vision_nav_get_search_center(&search_center_x, &search_center_y);
        min_x = search_center_x - VISION_NAV_SEARCH_HALF_WIDTH_X_CM;
        max_x = search_center_x + VISION_NAV_SEARCH_HALF_WIDTH_X_CM;
        min_y = search_center_y - VISION_NAV_SEARCH_HALF_WIDTH_Y_CM;
        max_y = search_center_y + VISION_NAV_SEARCH_HALF_WIDTH_Y_CM;

        /*
         * Keep full cruise speed in the field interior.  Within 60 cm of an
         * outer search edge, reduce only the velocity component pointing out
         * of the field: 100 -> 20 cm/s between 60 and 30 cm, then 20 -> 0
         * cm/s at the edge.  Tangential lane speed is not reduced.
         */
        if(desired_vx < 0.0f)
        {
            boundary_distance = vehicle_state.current_pos_x - min_x;
            if(boundary_distance < VISION_NAV_SEARCH_BOUNDARY_BRAKE_START_CM)
            {
                if(boundary_distance <= 0.0f)
                {
                    outward_speed_limit = 0.0f;
                }
                else if(boundary_distance <=
                        VISION_NAV_SEARCH_BOUNDARY_NEAR_CM)
                {
                    outward_speed_limit =
                        VISION_NAV_SEARCH_BOUNDARY_NEAR_SPEED_CM_S *
                        boundary_distance /
                        VISION_NAV_SEARCH_BOUNDARY_NEAR_CM;
                }
                else
                {
                    outward_speed_limit =
                        VISION_NAV_SEARCH_BOUNDARY_NEAR_SPEED_CM_S +
                        (VISION_NAV_SEARCH_SPEED_LIMIT_CM_S -
                         VISION_NAV_SEARCH_BOUNDARY_NEAR_SPEED_CM_S) *
                        (boundary_distance -
                         VISION_NAV_SEARCH_BOUNDARY_NEAR_CM) /
                        (VISION_NAV_SEARCH_BOUNDARY_BRAKE_START_CM -
                         VISION_NAV_SEARCH_BOUNDARY_NEAR_CM);
                }
                if(desired_vx < -outward_speed_limit)
                    desired_vx = -outward_speed_limit;
            }
        }
        else if(desired_vx > 0.0f)
        {
            boundary_distance = max_x - vehicle_state.current_pos_x;
            if(boundary_distance < VISION_NAV_SEARCH_BOUNDARY_BRAKE_START_CM)
            {
                if(boundary_distance <= 0.0f)
                {
                    outward_speed_limit = 0.0f;
                }
                else if(boundary_distance <=
                        VISION_NAV_SEARCH_BOUNDARY_NEAR_CM)
                {
                    outward_speed_limit =
                        VISION_NAV_SEARCH_BOUNDARY_NEAR_SPEED_CM_S *
                        boundary_distance /
                        VISION_NAV_SEARCH_BOUNDARY_NEAR_CM;
                }
                else
                {
                    outward_speed_limit =
                        VISION_NAV_SEARCH_BOUNDARY_NEAR_SPEED_CM_S +
                        (VISION_NAV_SEARCH_SPEED_LIMIT_CM_S -
                         VISION_NAV_SEARCH_BOUNDARY_NEAR_SPEED_CM_S) *
                        (boundary_distance -
                         VISION_NAV_SEARCH_BOUNDARY_NEAR_CM) /
                        (VISION_NAV_SEARCH_BOUNDARY_BRAKE_START_CM -
                         VISION_NAV_SEARCH_BOUNDARY_NEAR_CM);
                }
                if(desired_vx > outward_speed_limit)
                    desired_vx = outward_speed_limit;
            }
        }

        if(desired_vy < 0.0f)
        {
            boundary_distance = vehicle_state.current_pos_y - min_y;
            if(boundary_distance < VISION_NAV_SEARCH_BOUNDARY_BRAKE_START_CM)
            {
                if(boundary_distance <= 0.0f)
                {
                    outward_speed_limit = 0.0f;
                }
                else if(boundary_distance <=
                        VISION_NAV_SEARCH_BOUNDARY_NEAR_CM)
                {
                    outward_speed_limit =
                        VISION_NAV_SEARCH_BOUNDARY_NEAR_SPEED_CM_S *
                        boundary_distance /
                        VISION_NAV_SEARCH_BOUNDARY_NEAR_CM;
                }
                else
                {
                    outward_speed_limit =
                        VISION_NAV_SEARCH_BOUNDARY_NEAR_SPEED_CM_S +
                        (VISION_NAV_SEARCH_SPEED_LIMIT_CM_S -
                         VISION_NAV_SEARCH_BOUNDARY_NEAR_SPEED_CM_S) *
                        (boundary_distance -
                         VISION_NAV_SEARCH_BOUNDARY_NEAR_CM) /
                        (VISION_NAV_SEARCH_BOUNDARY_BRAKE_START_CM -
                         VISION_NAV_SEARCH_BOUNDARY_NEAR_CM);
                }
                if(desired_vy < -outward_speed_limit)
                    desired_vy = -outward_speed_limit;
            }
        }
        else if(desired_vy > 0.0f)
        {
            boundary_distance = max_y - vehicle_state.current_pos_y;
            if(boundary_distance < VISION_NAV_SEARCH_BOUNDARY_BRAKE_START_CM)
            {
                if(boundary_distance <= 0.0f)
                {
                    outward_speed_limit = 0.0f;
                }
                else if(boundary_distance <=
                        VISION_NAV_SEARCH_BOUNDARY_NEAR_CM)
                {
                    outward_speed_limit =
                        VISION_NAV_SEARCH_BOUNDARY_NEAR_SPEED_CM_S *
                        boundary_distance /
                        VISION_NAV_SEARCH_BOUNDARY_NEAR_CM;
                }
                else
                {
                    outward_speed_limit =
                        VISION_NAV_SEARCH_BOUNDARY_NEAR_SPEED_CM_S +
                        (VISION_NAV_SEARCH_SPEED_LIMIT_CM_S -
                         VISION_NAV_SEARCH_BOUNDARY_NEAR_SPEED_CM_S) *
                        (boundary_distance -
                         VISION_NAV_SEARCH_BOUNDARY_NEAR_CM) /
                        (VISION_NAV_SEARCH_BOUNDARY_BRAKE_START_CM -
                         VISION_NAV_SEARCH_BOUNDARY_NEAR_CM);
                }
                if(desired_vy > outward_speed_limit)
                    desired_vy = outward_speed_limit;
            }
        }
    }

    dv_x = desired_vx - loc_rt.profile_vel_x;
    dv_y = desired_vy - loc_rt.profile_vel_y;
    dv = sqrtf(dv_x * dv_x + dv_y * dv_y);
    max_dv = LOC_TRAJ_ACCEL_CM_S2 * dT_s;
    if((dv > max_dv) && (dv > 0.0001f))
    {
        dv_x *= max_dv / dv;
        dv_y *= max_dv / dv;
    }

    loc_rt.profile_vel_x += dv_x;
    loc_rt.profile_vel_y += dv_y;
    next_x = loc_rt.profile_pos_x + loc_rt.profile_vel_x * dT_s;
    next_y = loc_rt.profile_pos_y + loc_rt.profile_vel_y * dT_s;
    if((dx * (goal_x - next_x) + dy * (goal_y - next_y)) <= 0.0f)
    {
        loc_rt.profile_pos_x = goal_x;
        loc_rt.profile_pos_y = goal_y;
        loc_rt.profile_vel_x = 0.0f;
        loc_rt.profile_vel_y = 0.0f;
    }
    else
    {
        loc_rt.profile_pos_x = next_x;
        loc_rt.profile_pos_y = next_y;
    }

    loc_2l_ct.profile_vel_x = loc_rt.profile_vel_x;
    loc_2l_ct.profile_vel_y = loc_rt.profile_vel_y;
}

static void loc_reset_vel_debug(void)
{
    loc_1l_ct.vel_err_x_body = 0.0f;
    loc_1l_ct.vel_err_y_body = 0.0f;
    loc_1l_ct.raw_target_roll = 0.0f;
    loc_1l_ct.raw_target_pitch = 0.0f;
    loc_1l_ct.dynamic_target_roll = 0.0f;
    loc_1l_ct.dynamic_target_pitch = 0.0f;
    loc_1l_ct.target_accel_x_body = 0.0f;
    loc_1l_ct.target_accel_y_body = 0.0f;
    loc_1l_ct.horizontal_bias_roll = 0.0f;
    loc_1l_ct.horizontal_bias_pitch = 0.0f;
    loc_1l_ct.loc_weight = loc_rt.loc_weight;
    loc_1l_ct.recovery_closing_speed = 0.0f;
    loc_1l_ct.recovery_stop_speed = 0.0f;
    loc_1l_ct.recovery_brake_active = 0u;
    loc_1l_ct.loc_ready = 0u;
    loc_1l_ct.loc_hold_ready = 0u;
}

static void loc_hold_current_without_output(uint8_t clear_attitude_setpoint)
{
    if(loc_auto_low_height_mode() == 0u)
    {
        vehicle_setpoint.target_pos_x = vehicle_state.current_pos_x;
        vehicle_setpoint.target_pos_y = vehicle_state.current_pos_y;
    }
    vehicle_setpoint.target_vel_x = 0.0f;
    vehicle_setpoint.target_vel_y = 0.0f;

    if(clear_attitude_setpoint != 0u)
    {
        vehicle_setpoint.target_roll = 0.0f;
        vehicle_setpoint.target_pitch = 0.0f;
    }

    loc_2l_ct.exp_pos_x = vehicle_state.current_pos_x;
    loc_2l_ct.exp_pos_y = vehicle_state.current_pos_y;
    loc_2l_ct.fb_pos_x = vehicle_state.current_pos_x;
    loc_2l_ct.fb_pos_y = vehicle_state.current_pos_y;
    loc_2l_ct.exp_vel_x = 0.0f;
    loc_2l_ct.exp_vel_y = 0.0f;

    loc_1l_ct.exp_vel_x = 0.0f;
    loc_1l_ct.exp_vel_y = 0.0f;
    loc_1l_ct.fb_vel_x = vehicle_state.current_vel_x;
    loc_1l_ct.fb_vel_y = vehicle_state.current_vel_y;

    loc_rt.loc_weight = 0.0f;
    loc_rt.loc_ready_time_s = 0.0f;
    loc_rt.brake_weight = 0.0f;
    loc_rt.loc_hold_active = 0u;
    loc_rt.loc_hold_stable_time_s = 0.0f;
    loc_rt.loc_hold_time_s = 0.0f;
    loc_reset_runtime_output();
    loc_reset_pids();
    loc_reset_vel_debug();
}

static uint8_t loc_brake_phase_active(void)
{
    return ((loc_rt.loc_ready_last != 0u) &&
            (loc_rt.loc_ready_time_s < LOC_BRAKE_TIME_S));
}

static float loc_soft_deadband(float value, float deadband)
{
    if(value > deadband)
    {
        return value - deadband;
    }

    if(value < -deadband)
    {
        return value + deadband;
    }

    return 0.0f;
}

static float loc_hold_error_ratio(void)
{
    float err_x = loc_2l_ct.exp_pos_x - loc_2l_ct.fb_pos_x;
    float err_y = loc_2l_ct.exp_pos_y - loc_2l_ct.fb_pos_y;
    float err_abs = MAX(fabsf(err_x), fabsf(err_y));

    if((LOC_HOLD_ERR_ACTIVE_CM - LOC_HOLD_ERR_RELAX_CM) <= 0.001f)
    {
        return 1.0f;
    }

    return ctrl_smoothstep01((err_abs - LOC_HOLD_ERR_RELAX_CM) /
                             (LOC_HOLD_ERR_ACTIVE_CM - LOC_HOLD_ERR_RELAX_CM));
}

static float loc_damping_angle_limit(void)
{
    float enable_height = loc_hold_enable_height();
    float damping_height = loc_damping_enable_height();
    float height_span = enable_height - damping_height;
    float height_ratio = 1.0f;
    float vz_abs = fabsf(vehicle_state.current_vel_z);
    float reduce_ratio =
        ctrl_smoothstep01((vz_abs - LOC_DAMPING_VZ_FULL_CM_S) /
                          (LOC_DAMPING_VZ_REDUCE_CM_S - LOC_DAMPING_VZ_FULL_CM_S));
    float height_angle_limit;

    if(height_span > 0.001f)
    {
        height_ratio =
            ctrl_smoothstep01((vehicle_state.current_height - damping_height) /
                              height_span);
    }

    height_angle_limit =
        LOC_DAMPING_MIN_OUTPUT_ANGLE_DEG +
        (LOC_DAMPING_MAX_OUTPUT_ANGLE_DEG -
         LOC_DAMPING_MIN_OUTPUT_ANGLE_DEG) * height_ratio;

    return height_angle_limit -
           (height_angle_limit - LOC_DAMPING_MIN_OUTPUT_ANGLE_DEG) * reduce_ratio;
}

static void loc_earth_vel_err_to_body(float err_x_earth,
                                      float err_y_earth,
                                      float *err_x_body,
                                      float *err_y_body)
{
    *err_x_body =  err_x_earth * vehicle_state.yaw_cos + err_y_earth * vehicle_state.yaw_sin;
    *err_y_body = -err_x_earth * vehicle_state.yaw_sin + err_y_earth * vehicle_state.yaw_cos;
}

static float loc_accel_to_angle_deg(float accel_cm_s2)
{
    return atan2f(accel_cm_s2, LOC_GRAVITY_CM_S2) * 57.2957795f;
}


static void loc_rotate_vel_integrators_with_yaw(void)
{
    const float yaw_sin = vehicle_state.yaw_sin;
    const float yaw_cos = vehicle_state.yaw_cos;

    if(loc_rt.vel_i_yaw_valid == 0u)
    {
        loc_rt.vel_i_yaw_sin = yaw_sin;
        loc_rt.vel_i_yaw_cos = yaw_cos;
        loc_rt.vel_i_yaw_valid = 1u;
        return;
    }

    /* delta = yaw_now - yaw_previous; this also handles +/-180 deg wrap. */
    const float delta_cos =
        yaw_cos * loc_rt.vel_i_yaw_cos + yaw_sin * loc_rt.vel_i_yaw_sin;
    const float delta_sin =
        yaw_sin * loc_rt.vel_i_yaw_cos - yaw_cos * loc_rt.vel_i_yaw_sin;
    const float old_i_x = loc_ctrl.vel_pid[0].out_i;
    const float old_i_y = loc_ctrl.vel_pid[1].out_i;

    loc_ctrl.vel_pid[0].out_i = delta_cos * old_i_x + delta_sin * old_i_y;
    loc_ctrl.vel_pid[1].out_i = -delta_sin * old_i_x + delta_cos * old_i_y;

    loc_rt.vel_i_yaw_sin = yaw_sin;
    loc_rt.vel_i_yaw_cos = yaw_cos;
}


// 位置控制初始化
void loc_ctrl_init(void)
{
    // 位置环PID参数（外环）
    loc_ctrl.pos_pid[0] = (pid_param_t){     // X方向
        .kp = 0.351f,
        .ki = 0.0f,
        .kd = 0.0f,
        .i_max = 0.0f,
        .p_max = LOC_POS_CORRECTION_LIMIT_CM_S,
        .d_max = 0.0f,
        .low_pass = 0.1f
    };
    
    loc_ctrl.pos_pid[1] = (pid_param_t){     // Y方向
        .kp = 0.311f,
        .ki = 0.0f,
        .kd = 0.0f,
        .i_max = 0.0f,
        .p_max = LOC_POS_CORRECTION_LIMIT_CM_S,
        .d_max = 0.0f,
        .low_pass = 0.1f
    };
    
    /* Velocity PID output is horizontal acceleration (cm/s^2), not angle.
     * The acceleration command is converted to lean angle explicitly below. */
    loc_ctrl.vel_pid[0] = (pid_param_t){     // Vx方向
        .kp = 1.112f,
        .ki = 0.2435f,
        .kd = 0.051f,
        .i_max = 20.0f,
        .p_max = LOC_MAX_HORIZONTAL_ACCEL_CM_S2,
        .d_max = 30.0f,
        .low_pass = 0.2f
    };
    
    loc_ctrl.vel_pid[1] = (pid_param_t){     // Vy方向
        .kp = 1.2081f,
        .ki = 0.230f,
        .kd = 0.0605f,
        .i_max = 20.0f,
        .p_max = LOC_MAX_HORIZONTAL_ACCEL_CM_S2,
        .d_max = 30.0f,
        .low_pass = 0.2f
    };

}


// 位置环控制（外环）
void loc_2level_ctrl(float dT_s)
{
    loc_1l_ct.recovery_closing_speed = 0.0f;
    loc_1l_ct.recovery_stop_speed = 0.0f;
    loc_1l_ct.recovery_brake_active = 0u;

    if(loc_control_enabled() == 0u)
    {
        // 未起飞时不进行位置控制
        loc_hold_current_without_output(loc_mode_enabled());
        return;

    }

    // 获取当前位置
    loc_2l_ct.fb_pos_x = vehicle_state.current_pos_x;
    loc_2l_ct.fb_pos_y = vehicle_state.current_pos_y;
    
    // 根据模式设置期望位置
    if(loc_mode_enabled() != 0u)
    {
        if(loc_output_ready() == 0u)
        {
            loc_hold_current_without_output(1u);
            return;
        }

        loc_2l_ct.fb_pos_x = vehicle_state.current_pos_x;
        loc_2l_ct.fb_pos_y = vehicle_state.current_pos_y;

        if(loc_hold_ready() == 0u)
        {
            /* Between the low takeoff damping gate and the position-hold
             * gate, keep the launch target fixed. Replacing it with the
             * current estimate accepts drift before the later Yaw hold. */
            vehicle_setpoint.target_vel_x = 0.0f;
            vehicle_setpoint.target_vel_y = 0.0f;

            loc_2l_ct.exp_pos_x = vehicle_state.current_pos_x;
            loc_2l_ct.exp_pos_y = vehicle_state.current_pos_y;
            loc_2l_ct.exp_vel_x = 0.0f;
            loc_2l_ct.exp_vel_y = 0.0f;
            loc_rt.target_vel_x_ramped = 0.0f;
            loc_rt.target_vel_y_ramped = 0.0f;
            loc_reset_position_profile(vehicle_state.current_pos_x,
                                       vehicle_state.current_pos_y);
            loc_reset_pos_pids();
            return;
        }

        if(loc_brake_phase_active() != 0u)
        {
            loc_2l_ct.exp_vel_x = 0.0f;
            loc_2l_ct.exp_vel_y = 0.0f;
            loc_rt.target_vel_x_ramped = 0.0f;
            loc_rt.target_vel_y_ramped = 0.0f;
            vehicle_setpoint.target_vel_x = 0.0f;
            vehicle_setpoint.target_vel_y = 0.0f;
            return;
        }

        float goal_err_x = vehicle_setpoint.target_pos_x - loc_2l_ct.fb_pos_x;
        float goal_err_y = vehicle_setpoint.target_pos_y - loc_2l_ct.fb_pos_y;
        float goal_distance = sqrtf(goal_err_x * goal_err_x +
                                    goal_err_y * goal_err_y);
        float terminal_blend = ctrl_smoothstep01(
            (goal_distance - LOC_TERMINAL_APPROACH_RADIUS_CM) /
            (LOC_TERMINAL_CRUISE_RADIUS_CM -
             LOC_TERMINAL_APPROACH_RADIUS_CM));
        float pos_correction_limit =
            LOC_TERMINAL_POS_CORRECTION_LIMIT_CM_S +
            (LOC_POS_CORRECTION_LIMIT_CM_S -
             LOC_TERMINAL_POS_CORRECTION_LIMIT_CM_S) * terminal_blend;
        float total_vel_limit =
            LOC_TERMINAL_TOTAL_VEL_LIMIT_CM_S +
            (LOC_TOTAL_VEL_LIMIT_CM_S - LOC_TERMINAL_TOTAL_VEL_LIMIT_CM_S) *
            terminal_blend;
        float position_gain_scale = 1.0f;
        uint8_t mission_motion_authorized =
            (vehicle_state.flight_mode == FLY_AUTOFLY) ? 1u : 0u;
        uint8_t car_motion_active =
            ((mission_motion_authorized != 0u) &&
             ((vision_nav_obs.car_follow_active != 0u) ||
              (vision_nav_obs.car_prediction_active != 0u))) ? 1u : 0u;
        uint8_t search_motion_active =
            ((mission_motion_authorized != 0u) &&
             (vision_nav_obs.search_move_active != 0u)) ? 1u : 0u;
        uint8_t moving_target_active =
            ((car_motion_active != 0u) ||
             (search_motion_active != 0u)) ? 1u : 0u;
        if(vision_nav_yaw_spin_is_active() != 0u)
        {
            /* Yaw rotation is a stationary hold, but the normal terminal
             * profile is intentionally soft.  Stiffen only this phase so
             * optical-flow drift creates an earlier return command without
             * changing the mission/car-follow tuning. */
            position_gain_scale = LOC_YAW_SPIN_POS_GAIN_SCALE;
            pos_correction_limit =
                LOC_YAW_SPIN_POS_CORRECTION_LIMIT_CM_S;
            total_vel_limit = LOC_YAW_SPIN_TOTAL_VEL_LIMIT_CM_S;
        }
        if(moving_target_active != 0u)
        {
#if VISION_NAV_SIMPLE_DIRECT_CAR_FOLLOW
            /* The simple completion profile has no moving-car feed-forward.
             * Its direct visual correction and every search leg therefore
             * get a true vector-speed cap at the final LOC output. */
            total_vel_limit = (car_motion_active != 0u) ?
                VISION_NAV_CAR_FOLLOW_SPEED_LIMIT_CM_S :
                VISION_NAV_SEARCH_SPEED_LIMIT_CM_S;
#else
            /* A moving car is not a terminal stationary waypoint.  Keep the
             * full vector-speed budget so its velocity feed-forward survives
             * even when the spacing error is small. */
            total_vel_limit = LOC_TOTAL_VEL_LIMIT_CM_S;
#endif
        }
        loc_update_position_profile(vehicle_setpoint.target_pos_x,
                                    vehicle_setpoint.target_pos_y,
                                    dT_s);
        loc_2l_ct.exp_pos_x = vehicle_setpoint.target_pos_x;
        loc_2l_ct.exp_pos_y = vehicle_setpoint.target_pos_y;

        /* The profile is feed-forward only.  Position feedback must always
         * close on the final hold point; otherwise a leashed profile can move
         * with a drifting aircraft and hide the error that should bring it
         * home. */
        /* A trajectory feed-forward may briefly point away from the final
         * goal after its internal reference overshoots.  It must never cancel
         * the position feedback that is trying to return to the competition
         * target.  Keep aligned feed-forward, reject only the opposing part. */
        /* Profile velocity is a normal feed-forward term, not a second goal.
         * Keep only the component that points to the final competition target;
         * the final position feedback, terminal speed envelope and velocity
         * PID remain authoritative for capture and braking. */
        float profile_ff_x = ctrl_terminal_clamp(goal_err_x,
                                                  loc_rt.profile_vel_x);
        float profile_ff_y = ctrl_terminal_clamp(goal_err_y,
                                                  loc_rt.profile_vel_y);

        loc_2l_ct.exp_vel_x = profile_ff_x + LIMIT(
            position_gain_scale *
            stan_pid_solve(&loc_ctrl.pos_pid[0], goal_err_x, dT_s, 0),
            -pos_correction_limit, pos_correction_limit);
        loc_2l_ct.exp_vel_y = profile_ff_y + LIMIT(
            position_gain_scale *
            stan_pid_solve(&loc_ctrl.pos_pid[1], goal_err_y, dT_s, 0),
            -pos_correction_limit, pos_correction_limit);

        loc_2l_ct.exp_vel_x=ctrl_terminal_clamp(goal_err_x,loc_2l_ct.exp_vel_x);
        loc_2l_ct.exp_vel_y=ctrl_terminal_clamp(goal_err_y,loc_2l_ct.exp_vel_y);

        float brake_speed_x = ctrl_brake_speed(LOC_TRAJ_ACCEL_CM_S2, goal_err_x);
        float brake_speed_y = ctrl_brake_speed(LOC_TRAJ_ACCEL_CM_S2, goal_err_y);

            loc_2l_ct.exp_vel_x = LIMIT(loc_2l_ct.exp_vel_x,
                                        -brake_speed_x,
                                        brake_speed_x);

            loc_2l_ct.exp_vel_y = LIMIT(loc_2l_ct.exp_vel_y,
                                        -brake_speed_y,
                                        brake_speed_y);

        if((mission_height_recovery_active != 0u) &&
           (car_motion_active != 0u))
        {
            /* During altitude recovery this vector is the aircraft's motion
             * relative to the moving car: profile feed-forward plus spacing
             * correction.  Limit only that relative motion.  The measured
             * car feed-forward is added below and must survive, otherwise a
             * 25 cm/s total cap makes the tether pull the aircraft down when
             * the car is moving faster. */
            float relative_speed = sqrtf(
                loc_2l_ct.exp_vel_x * loc_2l_ct.exp_vel_x +
                loc_2l_ct.exp_vel_y * loc_2l_ct.exp_vel_y);
            if(relative_speed >
               MISSION_HEIGHT_RECOVERY_RELATIVE_SPEED_LIMIT_CM_S)
            {
                float relative_scale =
                    MISSION_HEIGHT_RECOVERY_RELATIVE_SPEED_LIMIT_CM_S /
                    relative_speed;
                loc_2l_ct.exp_vel_x *= relative_scale;
                loc_2l_ct.exp_vel_y *= relative_scale;
            }
        }
        else if((mission_height_recovery_active != 0u) &&
                (total_vel_limit >
                 MISSION_HEIGHT_RECOVERY_RELATIVE_SPEED_LIMIT_CM_S))
        {
            /* With no valid moving-car feed-forward, retain the conservative
             * absolute recovery limit. */
            total_vel_limit =
                MISSION_HEIGHT_RECOVERY_RELATIVE_SPEED_LIMIT_CM_S;
        }

        if(car_motion_active != 0u)
        {
            /* Position/profile terms close the spacing error; the measured
             * car velocity moves that spacing target with the car.  Add this
             * after the stationary-waypoint braking envelope so feed-forward
             * is not clipped merely because the spacing error is small. */
            loc_2l_ct.exp_vel_x += vision_nav_obs.car_ff_vel_x_cm_s;
            loc_2l_ct.exp_vel_y += vision_nav_obs.car_ff_vel_y_cm_s;
        }
        if(search_motion_active != 0u)
        {
            loc_2l_ct.exp_vel_x += vision_nav_obs.search_ff_vel_x_cm_s;
            loc_2l_ct.exp_vel_y += vision_nav_obs.search_ff_vel_y_cm_s;
        }

        float target_vel_x = loc_2l_ct.exp_vel_x;
        float target_vel_y = loc_2l_ct.exp_vel_y;
        float target_speed = sqrtf(target_vel_x * target_vel_x +
                                   target_vel_y * target_vel_y);
        float target_vel_slew_cm_s2 = LOC_TARGET_VEL_SLEW_CM_S2;
        uint8_t car_stop_decel_active = 0u;
        uint8_t recovery_brake_active = 0u;
        float recovery_closing_speed = 0.0f;
        float recovery_stop_speed = 0.0f;

        /* The competition limit is a horizontal vector speed, not a per-axis
         * limit; preserve direction when the profile and correction sum is
         * larger than the available cruise speed. */
        if(target_speed > total_vel_limit)
        {
            target_vel_x *= total_vel_limit / target_speed;
            target_vel_y *= total_vel_limit / target_speed;
        }

        /* Braking must be based on measured closing speed as well as
         * remaining distance.  A position-only target keeps asking for a
         * small toward-goal velocity even when the aircraft is already
         * crossing the target far too fast to stop in time. */
        if((goal_distance > 0.5f) && (moving_target_active == 0u))
        {
            float goal_unit_x = goal_err_x / goal_distance;
            float goal_unit_y = goal_err_y / goal_distance;
            float closing_speed =
                vehicle_state.current_vel_x * goal_unit_x +
                vehicle_state.current_vel_y * goal_unit_y;
            float stop_speed = sqrtf(2.0f * LOC_TRAJ_ACCEL_CM_S2 *
                                      goal_distance);
            recovery_closing_speed = closing_speed;
            recovery_stop_speed = stop_speed;

            if(closing_speed > (stop_speed + LOC_RECOVERY_BRAKE_MARGIN_CM_S))
            {
                target_vel_x = -goal_unit_x * total_vel_limit;
                target_vel_y = -goal_unit_y * total_vel_limit;
                recovery_brake_active = 1u;
            }
        }

            loc_1l_ct.recovery_closing_speed = recovery_closing_speed;
            loc_1l_ct.recovery_stop_speed = recovery_stop_speed;
            loc_1l_ct.recovery_brake_active = recovery_brake_active;

            if(recovery_brake_active != 0u)
            {
                target_vel_slew_cm_s2 =
                    LOC_RECOVERY_BRAKE_VEL_SLEW_CM_S2;
            }
            if((mission_motion_authorized != 0u) &&
               (vision_nav_obs.car_stop_brake_active != 0u))
            {
                float ramped_speed = sqrtf(
                    loc_rt.target_vel_x_ramped * loc_rt.target_vel_x_ramped +
                    loc_rt.target_vel_y_ramped * loc_rt.target_vel_y_ramped);
                if(target_speed < ramped_speed)
                {
                    target_vel_slew_cm_s2 =
                        LOC_CAR_STOP_BRAKE_VEL_SLEW_CM_S2;
                    car_stop_decel_active = 1u;
                }
            }

            if(car_stop_decel_active != 0u)
            {
                float dv_x = target_vel_x - loc_rt.target_vel_x_ramped;
                float dv_y = target_vel_y - loc_rt.target_vel_y_ramped;
                float dv = sqrtf(dv_x * dv_x + dv_y * dv_y);
                float max_dv = target_vel_slew_cm_s2 * dT_s;

                if((dv > max_dv) && (dv > 0.001f))
                {
                    float scale = max_dv / dv;
                    dv_x *= scale;
                    dv_y *= scale;
                }
                loc_rt.target_vel_x_ramped += dv_x;
                loc_rt.target_vel_y_ramped += dv_y;
            }
            else
            {
                loc_rt.target_vel_x_ramped =
                    ctrl_slew_limit(loc_rt.target_vel_x_ramped,
                                    target_vel_x,
                                    target_vel_slew_cm_s2,
                                    dT_s);

                loc_rt.target_vel_y_ramped =
                    ctrl_slew_limit(loc_rt.target_vel_y_ramped,
                                    target_vel_y,
                                    target_vel_slew_cm_s2,
                                    dT_s);
            }

        vehicle_setpoint.target_vel_x = loc_rt.target_vel_x_ramped;
        vehicle_setpoint.target_vel_y = loc_rt.target_vel_y_ramped;


    }
    else
    {
        loc_hold_current_without_output(0u);
    }


}

// 速度环控制（内环）
void loc_1level_ctrl(float dT_s)
{
    if(loc_control_enabled() == 0u)
    {
        if(loc_mode_enabled() != 0u)
        {
            vehicle_setpoint.target_roll = 0.0f;
            vehicle_setpoint.target_pitch = 0.0f;
        }
        loc_reset_vel_debug();
        loc_rt.vel_i_yaw_valid = 0u;
        return; // 未解锁不输出
    }

    if(loc_mode_enabled() != 0u)
    {
        if(loc_output_ready() == 0u)
        {
            vehicle_setpoint.target_roll = 0.0f;
            vehicle_setpoint.target_pitch = 0.0f;
            loc_reset_vel_debug();
            loc_reset_vel_pids();
            return;
        }

        loc_1l_ct.exp_vel_x = vehicle_setpoint.target_vel_x;
        loc_1l_ct.exp_vel_y = vehicle_setpoint.target_vel_y;
        loc_1l_ct.fb_vel_x = vehicle_state.current_vel_x;
        loc_1l_ct.fb_vel_y = vehicle_state.current_vel_y;


        float vel_err_x_earth = loc_1l_ct.exp_vel_x - loc_1l_ct.fb_vel_x;
        float vel_err_y_earth = loc_1l_ct.exp_vel_y - loc_1l_ct.fb_vel_y;
        float terminal_direct_weight = 0.0f;
        float current_vel_x_body = 0.0f;
        float current_vel_y_body = 0.0f;

        float vel_err_x_body;
        float vel_err_y_body;

        if((loc_hold_ready() != 0u) &&
           ((vehicle_state.flight_mode != FLY_AUTOFLY) ||
            ((vision_nav_obs.car_follow_active == 0u) &&
             (vision_nav_obs.car_prediction_active == 0u) &&
             (vision_nav_obs.search_spiral_active == 0u))))
        {
            float goal_err_x = vehicle_setpoint.target_pos_x -
                               vehicle_state.current_pos_x;
            float goal_err_y = vehicle_setpoint.target_pos_y -
                               vehicle_state.current_pos_y;
            float goal_distance = sqrtf(goal_err_x * goal_err_x +
                                        goal_err_y * goal_err_y);
            float direct_vel_x = LIMIT(loc_ctrl.pos_pid[0].kp * goal_err_x,
                                       -LOC_TERMINAL_POS_CORRECTION_LIMIT_CM_S,
                                        LOC_TERMINAL_POS_CORRECTION_LIMIT_CM_S);
            float direct_vel_y = LIMIT(loc_ctrl.pos_pid[1].kp * goal_err_y,
                                       -LOC_TERMINAL_POS_CORRECTION_LIMIT_CM_S,
                                        LOC_TERMINAL_POS_CORRECTION_LIMIT_CM_S);
            float direct_speed = sqrtf(direct_vel_x * direct_vel_x +
                                       direct_vel_y * direct_vel_y);

            if(direct_speed > LOC_TERMINAL_TOTAL_VEL_LIMIT_CM_S)
            {
                direct_vel_x *= LOC_TERMINAL_TOTAL_VEL_LIMIT_CM_S /
                                direct_speed;
                direct_vel_y *= LOC_TERMINAL_TOTAL_VEL_LIMIT_CM_S /
                                direct_speed;
            }

            terminal_direct_weight = 1.0f - ctrl_smoothstep01(
                (goal_distance - LOC_TERMINAL_APPROACH_RADIUS_CM) /
                (LOC_TERMINAL_CRUISE_RADIUS_CM -
                 LOC_TERMINAL_APPROACH_RADIUS_CM));

            /* In terminal hold, close directly on the final position and
             * measured velocity.  Profile, target-velocity slew and recovery
             * logic remain authoritative outside the terminal blend. */
            vel_err_x_earth += terminal_direct_weight *
                ((direct_vel_x - loc_1l_ct.fb_vel_x) - vel_err_x_earth);
            vel_err_y_earth += terminal_direct_weight *
                ((direct_vel_y - loc_1l_ct.fb_vel_y) - vel_err_y_earth);
        }

        loc_rotate_vel_integrators_with_yaw();
        loc_earth_vel_err_to_body(loc_1l_ct.fb_vel_x,
                                  loc_1l_ct.fb_vel_y,
                                  &current_vel_x_body,
                                  &current_vel_y_body);
        loc_earth_vel_err_to_body(vel_err_x_earth,
                                  vel_err_y_earth,
                                  &vel_err_x_body,
                                  &vel_err_y_body);

        if((loc_brake_phase_active() != 0u) ||
           (loc_hold_ready() == 0u))
        {
            /*
             * Takeoff drift arrest and position hold are different plants.
             * During arrest the aircraft still carries the previous attitude
             * and lateral acceleration through each velocity zero crossing.
             * Applying the full hold-loop gain here reverses the attitude
             * command faster than the airframe can settle and creates a
             * growing pendulum.  Schedule down the complete velocity error so
             * P, I and D retain their Flash-configured ratios; hold mode still
             * uses the unscaled Flash PID parameters.
             */
            vel_err_x_body *= LOC_DAMPING_VEL_ERR_SCALE;
            vel_err_y_body *= LOC_DAMPING_VEL_ERR_SCALE;
            loc_rt.vel_err_x_body_filtered = vel_err_x_body;
            loc_rt.vel_err_y_body_filtered = vel_err_y_body;
        }
        else
        {
            float hold_error_ratio = loc_hold_error_ratio();
            float lpf_alpha =
                LOC_HOLD_VEL_ERR_LPF_ALPHA_MIN +
                (LOC_HOLD_VEL_ERR_LPF_ALPHA_MAX -
                 LOC_HOLD_VEL_ERR_LPF_ALPHA_MIN) * hold_error_ratio;
            float deadband =
                LOC_HOLD_VEL_ERR_DEADBAND_MAX_CM_S -
                (LOC_HOLD_VEL_ERR_DEADBAND_MAX_CM_S -
                 LOC_HOLD_VEL_ERR_DEADBAND_MIN_CM_S) * hold_error_ratio;

            loc_rt.vel_err_x_body_filtered +=
                (vel_err_x_body - loc_rt.vel_err_x_body_filtered) *
                lpf_alpha;
            loc_rt.vel_err_y_body_filtered +=
                (vel_err_y_body - loc_rt.vel_err_y_body_filtered) *
                lpf_alpha;

            /* The direct terminal controller uses the current measured
             * velocity as damping.  Blending the old filtered path out avoids
             * carrying its phase lag into the final hold region. */
            vel_err_x_body = terminal_direct_weight * vel_err_x_body +
                (1.0f - terminal_direct_weight) *
                loc_soft_deadband(loc_rt.vel_err_x_body_filtered, deadband);
            vel_err_y_body = terminal_direct_weight * vel_err_y_body +
                (1.0f - terminal_direct_weight) *
                loc_soft_deadband(loc_rt.vel_err_y_body_filtered, deadband);
        }

        loc_1l_ct.vel_err_x_body = vel_err_x_body;
        loc_1l_ct.vel_err_y_body = vel_err_y_body;

        /* The velocity I term is a steady-bias compensator, not a braking
         * command.  During a return or overshoot its old direction can oppose
         * the now-reversed velocity error and delay recovery through the
         * target.  Release only that conflicting memory while a valid hold is
         * active; keep the remaining I term for CG/voltage bias compensation.
         * At 50 Hz, 0.96 gives an approximately 0.5 s release time constant. */
        if(loc_hold_ready() != 0u)
        {
            if((loc_ctrl.vel_pid[0].out_i * vel_err_x_body) < 0.0f)
            {
                loc_ctrl.vel_pid[0].out_i *= LOC_VEL_I_OPPOSE_DECAY;
            }
            if((loc_ctrl.vel_pid[1].out_i * vel_err_y_body) < 0.0f)
            {
                loc_ctrl.vel_pid[1].out_i *= LOC_VEL_I_OPPOSE_DECAY;
            }
        }

     
        {
            float pid_accel_x = stan_pid_solve(&loc_ctrl.vel_pid[0],
                                               vel_err_x_body,
                                               dT_s,
                                               loc_rt.vel_pid_saturated_x);
            float pid_accel_y = stan_pid_solve(&loc_ctrl.vel_pid[1],
                                               vel_err_y_body,
                                               dT_s,
                                               loc_rt.vel_pid_saturated_y);
            float direct_accel_x = loc_ctrl.vel_pid[0].kp * vel_err_x_body +
                                   loc_ctrl.vel_pid[0].out_i -
                (LOC_TERMINAL_VEL_DAMPING_SCALE - 1.0f) *
                loc_ctrl.vel_pid[0].kp * current_vel_x_body;
            float direct_accel_y = loc_ctrl.vel_pid[1].kp * vel_err_y_body +
                                   loc_ctrl.vel_pid[1].out_i -
                (LOC_TERMINAL_VEL_DAMPING_SCALE - 1.0f) *
                loc_ctrl.vel_pid[1].kp * current_vel_y_body;

            /* At the final hold point this is the explicit second-order law:
             * accel = (pos_kp * vel_kp) * position_error
             *       - damping_scale * vel_kp * measured_velocity
             *       + bounded bias integral.
             * The P/I gains remain Flash-sourced.  The explicit terminal
             * damping scale is logged in PROFILECFG; D is blended out because
             * measured velocity already supplies physical damping. */
            loc_1l_ct.target_accel_x_body = pid_accel_x +
                terminal_direct_weight * (direct_accel_x - pid_accel_x);
            loc_1l_ct.target_accel_y_body = pid_accel_y +
                terminal_direct_weight * (direct_accel_y - pid_accel_y);
        }

        /* +body-X acceleration requires negative pitch; +body-Y requires
         * positive roll for this airframe's established attitude signs. */
        loc_1l_ct.dynamic_target_pitch =
            -loc_accel_to_angle_deg(loc_1l_ct.target_accel_x_body);
        loc_1l_ct.dynamic_target_roll =
             loc_accel_to_angle_deg(loc_1l_ct.target_accel_y_body);

        loc_1l_ct.horizontal_bias_roll = 0.0f;
        loc_1l_ct.horizontal_bias_pitch = 0.0f;

        vehicle_setpoint.target_pitch = loc_1l_ct.dynamic_target_pitch;
        vehicle_setpoint.target_roll = loc_1l_ct.dynamic_target_roll;
        float angle_limit = LOC_MAX_OUTPUT_ANGLE_DEG;
        if(loc_hold_ready() == 0u)
        {
            angle_limit = loc_damping_angle_limit();
        }
        if((loc_auto_low_height_mode() != 0u) &&
           (vehicle_state.current_height < LOC_ENABLE_HEIGHT_CM))
        {
            angle_limit = fminf(angle_limit,
                                LOC_AUTO_LOW_MAX_OUTPUT_ANGLE_DEG);
        }
        if((loc_auto_takeoff_mode() != 0u) &&
           (vehicle_state.current_height <
            LOC_AUTO_TAKEOFF_INITIAL_ANGLE_LIMIT_HEIGHT_CM))
        {
            angle_limit = fminf(
                angle_limit,
                LOC_AUTO_TAKEOFF_INITIAL_MAX_OUTPUT_ANGLE_DEG);
        }
        if(vision_nav_yaw_spin_is_active() != 0u)
        {
            angle_limit = fminf(angle_limit,
                                VISION_NAV_YAW_SPIN_LOC_ANGLE_DEG);
        }
        if((vehicle_state.flight_mode == FLY_AUTOFLY) &&
           (vision_nav_obs.car_height_recovery_active != 0u))
        {
            /* Keep matching the car's velocity, but reserve thrust for the
             * 140 cm altitude recovery instead of spending it on a large
             * spacing-error bank angle. */
            angle_limit = fminf(angle_limit,
                                VISION_NAV_CAR_RECOVERY_MAX_ANGLE_DEG);
        }
        if(vision_nav_obs.horizontal_fault_active != 0u)
        {
            angle_limit = fminf(angle_limit,
                                LOC_HORIZONTAL_FAULT_MAX_ANGLE_DEG);
        }

        /* Feed the real downstream authority limit back to the velocity PID.
         * stan_pid_solve() then freezes only integration that would push
         * farther into saturation, while opposite-sign error can unwind I. */
        loc_rt.vel_pid_saturated_x =
            ((fabsf(vehicle_setpoint.target_pitch) >= (angle_limit - 0.01f)) ||
             (fabsf(loc_1l_ct.target_accel_x_body) >=
              (LOC_MAX_HORIZONTAL_ACCEL_CM_S2 - 0.5f))) ? 1u : 0u;
        loc_rt.vel_pid_saturated_y =
            ((fabsf(vehicle_setpoint.target_roll) >= (angle_limit - 0.01f)) ||
             (fabsf(loc_1l_ct.target_accel_y_body) >=
              (LOC_MAX_HORIZONTAL_ACCEL_CM_S2 - 0.5f))) ? 1u : 0u;

        vehicle_setpoint.target_roll = LIMIT(vehicle_setpoint.target_roll,
                                             -angle_limit,
                                             angle_limit);
        vehicle_setpoint.target_pitch = LIMIT(vehicle_setpoint.target_pitch,
                                              -angle_limit,
                                              angle_limit);
        loc_1l_ct.raw_target_roll = vehicle_setpoint.target_roll;
        loc_1l_ct.raw_target_pitch = vehicle_setpoint.target_pitch;
        float angle_slew = LOC_HOLD_TARGET_ANGEL_SLEW_DEG_S2;
        if(loc_brake_phase_active() != 0u)
        {
            angle_slew = LOC_TARGET_ANGEL_SLEW_DEG_S2;
        }

        loc_rt.target_roll_ramped = ctrl_slew_limit(loc_rt.target_roll_ramped, vehicle_setpoint.target_roll, angle_slew, dT_s);
        loc_rt.target_pitch_ramped = ctrl_slew_limit(loc_rt.target_pitch_ramped, vehicle_setpoint.target_pitch, angle_slew, dT_s);

        float output_weight = loc_rt.loc_weight;
        if(loc_brake_phase_active() != 0u)
        {
            output_weight = loc_rt.brake_weight;
        }
        loc_1l_ct.loc_weight = output_weight;

        vehicle_setpoint.target_roll = loc_rt.target_roll_ramped * output_weight;
        vehicle_setpoint.target_pitch = loc_rt.target_pitch_ramped * output_weight;
    }
    else
    {
        loc_reset_runtime_output();
        loc_reset_vel_debug();

        loc_reset_vel_pids();
    }
   
}

// 位置控制主函数
void loc_ctrl_update(float dT_s)
{
    uint8_t ready;

    if((dT_s <= 0.0f) || (dT_s > 0.1f))
    {
        dT_s = 0.02f;
    }

    // 检查状态
    if((vehicle_state.flight_mode != loc_rt.last_flight_mode) &&
       ((vehicle_state.flight_mode == FLY_POS_HOLD) ||
        (vehicle_state.flight_mode == FLY_AUTOFLY) ||
        (vehicle_state.flight_mode == FLY_AUTOTAKEOFF) ||
        (vehicle_state.flight_mode == FLY_AUTOLANDING)))
    {
        vehicle_setpoint.target_pos_x = vehicle_state.current_pos_x;
        vehicle_setpoint.target_pos_y = vehicle_state.current_pos_y;

        /* AUTOFLY and AUTOLANDING both take ownership from a moving target.
         * Do not carry the old profile velocity, velocity-I bias or attitude
         * ramp into either transition.  Landing must brake around its newly
         * captured point before vertical descent begins. */
        if((vehicle_state.flight_mode == FLY_AUTOFLY) ||
           (vehicle_state.flight_mode == FLY_AUTOLANDING))
        {
            vehicle_setpoint.target_vel_x = 0.0f;
            vehicle_setpoint.target_vel_y = 0.0f;
            loc_reset_position_profile(vehicle_state.current_pos_x,
                                       vehicle_state.current_pos_y);
            loc_reset_runtime_output();
            loc_reset_pids();
            loc_reset_vel_debug();
        }
    }

    loc_rt.last_flight_mode = vehicle_state.flight_mode;

    if(vision_nav_obs.horizontal_fault_active != 0u)
    {
        vehicle_setpoint.target_pos_x = vehicle_state.current_pos_x;
        vehicle_setpoint.target_pos_y = vehicle_state.current_pos_y;
        vehicle_setpoint.target_vel_x = 0.0f;
        vehicle_setpoint.target_vel_y = 0.0f;
        loc_reset_position_profile(vehicle_state.current_pos_x,
                                   vehicle_state.current_pos_y);
        if(loc_horizontal_fault_last == 0u)
        {
            loc_reset_runtime_output();
            loc_reset_pids();
            loc_reset_vel_debug();
        }
    }
    loc_horizontal_fault_last =
        vision_nav_obs.horizontal_fault_active;

    ready = loc_output_ready();
    if(ready == 0u)
    {
        loc_rt.loc_ready_last = 0u;
        loc_rt.loc_weight = 0.0f;
        loc_rt.loc_ready_time_s = 0.0f;
        loc_rt.brake_weight = 0.0f;
        loc_rt.loc_hold_active = 0u;
        loc_rt.loc_hold_time_s = 0.0f;
    }
    else
    {
        if(loc_rt.loc_ready_last == 0u)
        {
            if(loc_auto_low_height_mode() == 0u)
            {
                vehicle_setpoint.target_pos_x = vehicle_state.current_pos_x;
                vehicle_setpoint.target_pos_y = vehicle_state.current_pos_y;
            }
            loc_reset_runtime_output();
            loc_reset_pids();
            loc_reset_vel_debug();
            loc_rt.loc_weight = 0.0f;
            loc_rt.loc_ready_time_s = 0.0f;
            loc_rt.brake_weight = 0.0f;
            loc_rt.loc_hold_active = 0u;
            loc_rt.loc_hold_stable_time_s = 0.0f;
            loc_rt.loc_hold_time_s = 0.0f;
        }

        if(loc_rt.loc_hold_active != 0u)
        {
            if(loc_hold_should_release() != 0u)
            {
                loc_rt.loc_hold_active = 0u;
                loc_rt.loc_hold_stable_time_s = 0.0f;
                loc_rt.loc_hold_time_s = 0.0f;
            }
            else
            {
                loc_rt.loc_hold_time_s += dT_s;
            }
        }
        else
        {
            if(loc_hold_entry_ready() != 0u)
            {
                loc_rt.loc_hold_stable_time_s += dT_s;
                if((loc_auto_low_height_mode() != 0u) ||
                   (loc_rt.loc_hold_stable_time_s >= 0.3f))
                {
                    /* Automatic takeoff preserves the target captured on
                     * mode entry, so hold corrects early drift instead of
                     * declaring the displaced point to be the new origin. */
                    loc_2l_ct.exp_pos_x = vehicle_state.current_pos_x;
                    loc_2l_ct.exp_pos_y = vehicle_state.current_pos_y;
                    loc_reset_position_profile(vehicle_state.current_pos_x,
                                               vehicle_state.current_pos_y);
                    loc_reset_pos_pids();
                    // Remove braking I bias without erasing derivative
                    // history; a full reset would create a D transient at
                    // the damping-to-hold transition.
                    loc_clear_vel_pid_integrators();
                    loc_rt.loc_hold_active = 1u;
                    loc_rt.loc_hold_stable_time_s = 0.0f;
                    loc_rt.loc_hold_time_s = 0.0f;
                }
            }
            else
            {
                loc_rt.loc_hold_stable_time_s = 0.0f;
            }
        }

        float blend_time_s = (loc_auto_low_height_mode() != 0u) ?
            LOC_AUTO_LOW_BLEND_TIME_S : LOC_BLEND_TIME_S;

        loc_rt.loc_ready_time_s += dT_s;
        loc_rt.loc_weight += dT_s / blend_time_s;
        loc_rt.loc_weight = LIMIT(loc_rt.loc_weight, 0.0f, 1.0f);
        loc_rt.brake_weight += dT_s / LOC_BRAKE_BLEND_TIME_S;
        loc_rt.brake_weight = LIMIT(loc_rt.brake_weight, 0.0f, 1.0f);
        loc_rt.loc_ready_last = 1u;
    }
    loc_1l_ct.loc_weight = loc_rt.loc_weight;
    loc_1l_ct.loc_ready = ready;
    loc_1l_ct.loc_hold_ready = loc_hold_ready();
    
    // 外环：位置控制
    loc_2level_ctrl(dT_s);
    
    // 内环：速度控制
    loc_1level_ctrl(dT_s);
}
