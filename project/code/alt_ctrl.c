#include "zf_common_headfile.h"

altitude_control_t alt_ctrl = {0};

//--------runtime_state----------------------------------//
typedef struct
{
    float throttle_val;
    float takeoff_headroom_ramped;
    float target_vel_z_ramped;
    float target_throttle_ramped;
    float landed_low_time_s;
    uint8_t landed_state;
    float alt_vel_test_time_s;
    float alt_vel_test_ready_time_s;
    uint8_t alt_vel_test_started;
    float alt_profile_height_cm;
    float alt_profile_vel_cm_s;
    float alt_profile_acc_cm_s2;
    uint8_t alt_profile_valid;
    float alt_profile_ready_time_s;
    uint8_t alt_profile_started;
    float alt_armed_voltage;
} alt_runtime_t;

static alt_runtime_t alt_rt =
{
    .landed_state = 1u,
    .alt_armed_voltage = 12.0f
};
//------------------static_ctrl_func--------------------------------------------//

static uint8_t alt_landed_raw(void)
{
    if(vehicle_state.armed == 0u)
    {
        return 1u;
    }

    /* Low throttle is normal during a commanded descent.  It is evidence of
     * landing only when the aircraft is also close to the floor and no longer
     * moving vertically.  Without these gates ALT_PHASE_LANDED can be reached
     * in the air and param_update() will correctly-but-dangerously disarm. */
    if(vehicle_state.current_height > AUTO_LAND_DISARM_HEIGHT_CM)
    {
        return 0u;
    }

    if(fabsf(vehicle_state.current_vel_z) > AUTO_LAND_DISARM_VEL_CM_S)
    {
        return 0u;
    }

    /* During commanded auto-landing the height loop may still hold roughly
     * hover throttle after touchdown.  Height and vertical-speed debounce are
     * sufficient here; requiring low throttle can prevent disarming forever. */
    if(vehicle_state.flight_mode == FLY_AUTOLANDING)
    {
        return 1u;
    }

    return (throttle_ramped_debug <
            (system_get_hover_throttle_base() - ALT_LANDED_THR_MARGIN)) ? 1u : 0u;
}

static void alt_landed_update(float dT_s)
{
    if(vehicle_state.armed == 0u)
    {
        alt_rt.landed_low_time_s = 0.0f;
        alt_rt.landed_state = 1u;
        return;
    }

    if(alt_landed_raw() != 0u)
    {
        alt_rt.landed_low_time_s += dT_s;
        if(alt_rt.landed_low_time_s >= ALT_LANDED_DEBOUNCE_S)
        {
            alt_rt.landed_state = 1u;
        }
    }
    else
    {
        alt_rt.landed_low_time_s = 0.0f;
        alt_rt.landed_state = 0u;
    }
}

static uint8_t alt_is_landed(void)
{
    return alt_rt.landed_state;
}

static uint8_t alt_i_enable(void)
{
    return (alt_is_landed() == 0u);
}

static float alt_authority_weight(void)
{
    float ratio;
    float height = vehicle_state.current_height;

    if((ALT_TRANSITION_HEIGHT_CM - ALT_TAKEOFF_HEIGHT_CM) == 0.0f)
    {
        return 1.0f;
    }



    ratio = (height - ALT_TAKEOFF_HEIGHT_CM) /
            (ALT_TRANSITION_HEIGHT_CM - ALT_TAKEOFF_HEIGHT_CM);

    ratio = ctrl_smoothstep01(ratio);
    return ALT_AUTHORITY_MIN + (1.0f - ALT_AUTHORITY_MIN) * ratio;
}

static float alt_vel_feedback_weight(void)
{
    float ratio;
    float height = vehicle_state.current_height;

    if((ALT_VEL_FB_FULL_HEIGHT_CM - ALT_VEL_FB_START_HEIGHT_CM) == 0.0f)
    {
        return 1.0f;
    }

    ratio = (height - ALT_VEL_FB_START_HEIGHT_CM) /
            (ALT_VEL_FB_FULL_HEIGHT_CM - ALT_VEL_FB_START_HEIGHT_CM);
    ratio = ctrl_smoothstep01(ratio);
    return ALT_VEL_FB_MIN_WEIGHT + (1.0f - ALT_VEL_FB_MIN_WEIGHT) * ratio;
}

static void alt_throttle_limit(float authority, float trim, float *min_throttle, float *max_throttle)
{
    float hover_base = system_get_hover_throttle_base() + trim;
    float range = ALT_THROTTLE_AUTHORITY_RANGE * authority;

    *min_throttle = hover_base - range;
    *max_throttle = hover_base + ALT_MAX_THROTTLE_HEADROOM;
}

volatile alt_2l_ct_t alt_2l_ct = {0};
volatile alt_1l_ct_t alt_1l_ct = {0};
volatile alt_phase_t alt_phase = ALT_PHASE_LANDED;
volatile alt_ctrl_debug_t alt_ctrl_debug = {0};
//------------------------alt_phase_state--------------------------------------//
static void alt_phase_reset(void)
{
    alt_phase = ALT_PHASE_LANDED;
    alt_rt.landed_low_time_s = 0.0f;
    alt_rt.landed_state = 1u;
}

static void alt_phase_update(float dT_s)
{
    static float spooling_time_s = 0.0f;
    uint8_t is_landed = alt_is_landed();

    if((dT_s <= 0.0f) || (dT_s > 0.1f))
    {
        dT_s = 0.02f;
    }

    alt_landed_update(dT_s);
    is_landed = alt_is_landed();

    if(vehicle_state.armed == 0u)
    {
        alt_phase_reset();
        spooling_time_s = 0.0f;
        return;
    }

    if((vehicle_state.flight_mode == FLY_AUTOLANDING) &&
       (alt_phase != ALT_PHASE_LANDING))
    {
        alt_phase = ALT_PHASE_LANDING;
    }

    switch(alt_phase)
    {
        case ALT_PHASE_LANDED:
            alt_phase = ALT_PHASE_SPOOLING;
            spooling_time_s = 0.0f;
            break;

        case ALT_PHASE_SPOOLING:
            spooling_time_s += dT_s;
            if((is_landed == 0u) ||
               (spooling_time_s >= ALT_SPOOLING_TIMEOUT_S))
            {
                alt_phase = ALT_PHASE_TAKEOFF;
                spooling_time_s = 0.0f;
            }
            break;

        case ALT_PHASE_TAKEOFF:
            spooling_time_s = 0.0f;
            if((vehicle_state.current_height >= ALT_TAKEOFF_COMPLETE_HEIGHT_CM) ||
               ((vehicle_setpoint.target_height - vehicle_state.current_height) <=
                ALT_TAKEOFF_COMPLETE_ERR_CM))
            {
                alt_phase = ALT_PHASE_HOLD;
            }
            break;

        case ALT_PHASE_HOLD:
            spooling_time_s = 0.0f;
            break;

        case ALT_PHASE_LANDING:
        default:
            if(is_landed != 0u)
            {
                alt_phase = ALT_PHASE_LANDED;
                spooling_time_s = 0.0f;
            }
            else if(vehicle_state.flight_mode != FLY_AUTOLANDING)
            {
                alt_phase = ALT_PHASE_HOLD;
                spooling_time_s = 0.0f;
            }
            break;
    }
}
//--------------------alt_ctrl-------------------------------------//
void alt_ctrl_init(void)
{
    alt_ctrl.height_pid = (pid_param_t)
    {
        .kp = 0.06f,
        .ki = 0.0f,
        .kd = 0.0f,
        .i_max = 50.0f,
        .p_max = 100.0f,
        .d_max = 20.0f,
        .low_pass = 0.1f
    };
    
    alt_ctrl.vel_pid = (pid_param_t)
    {
        .kp = 0.135f,
        .ki = 0.100f,
        .kd = 0.0f,
        .i_max = 3.0f,
        .p_max = 100.0f,
        .d_max = 20.0f,
        .low_pass = 0.2f
    };
}

void alt_2level_ctrl(float dT_s)
{
    float final_height;
    float remaining_before;
    float remaining_after;
    float brake_speed;
    float desired_profile_vel;
    float profile_error;
    float position_correction;
    float velocity_command;
    float profile_climb_limit;
    float profile_vel_before;
    float profile_height_before_constraints;
    float final_err;

    if((dT_s <= 0.0f) || (dT_s > 0.1f))
    {
        dT_s = 0.02f;
    }

    if(vehicle_state.armed == 0u)
    {
        stan_pid_reset(&alt_ctrl.height_pid);
        alt_rt.target_vel_z_ramped = 0.0f;
        alt_rt.alt_profile_height_cm = vehicle_state.current_height;
        alt_rt.alt_profile_vel_cm_s = 0.0f;
        alt_rt.alt_profile_acc_cm_s2 = 0.0f;
        alt_rt.alt_profile_valid = 0u;
        alt_rt.alt_profile_ready_time_s = 0.0f;
        alt_rt.alt_profile_started = 0u;
        alt_rt.alt_armed_voltage = vehicle_state.battery_voltage_filtered;
        alt_rt.alt_vel_test_time_s = 0.0f;
        alt_rt.alt_vel_test_ready_time_s = 0.0f;
        alt_rt.alt_vel_test_started = 0u;
        alt_phase_reset();
        return;
    }

    switch(vehicle_state.flight_mode)
    {
        case FLY_POS_HOLD:
        case FLY_HEIGHT_HOLD:
        case FLY_AUTOFLY:
        case FLY_AUTOTAKEOFF:
        case FLY_AUTOLANDING:
        {
            final_height = LIMIT(vehicle_setpoint.target_height,
                                 MIN_HEIGHT,
                                 MAX_HEIGHT);
            final_err = final_height - vehicle_state.current_height;

            if(alt_rt.alt_profile_valid == 0u)
            {
                alt_rt.alt_profile_height_cm = vehicle_state.current_height;
                alt_rt.alt_profile_vel_cm_s = 0.0f;
                alt_rt.alt_profile_acc_cm_s2 = 0.0f;
                alt_rt.alt_profile_valid = 1u;
                alt_rt.alt_profile_ready_time_s = 0.0f;
                alt_rt.alt_profile_started = 0u;
                stan_pid_reset(&alt_ctrl.height_pid);
            }

            if(alt_rt.alt_profile_started == 0u)
            {
                /* TAKEOFF must start its trajectory immediately.  Waiting
                 * until 10 cm while pinning the profile to current height
                 * makes the height error zero and prevents liftoff boost. */
                if((alt_phase == ALT_PHASE_TAKEOFF) ||
                   ((alt_phase == ALT_PHASE_HOLD) &&
                    (vehicle_state.current_height >=
                     ALT_TAKEOFF_COMPLETE_HEIGHT_CM)))
                {
                    alt_rt.alt_profile_started = 1u;
                    alt_rt.alt_profile_height_cm = vehicle_state.current_height;
                    alt_rt.alt_profile_vel_cm_s =
                        LIMIT(vehicle_state.current_vel_z,
                              -ALT_TAKEOFF_VEL_CMD_LIMIT_CM_S,
                               ALT_TAKEOFF_VEL_CMD_LIMIT_CM_S);
                    alt_rt.alt_profile_acc_cm_s2 = 0.0f;
                    stan_pid_reset(&alt_ctrl.height_pid);
                }

                if(alt_rt.alt_profile_started == 0u)
                {
                    alt_rt.alt_profile_height_cm = vehicle_state.current_height;
                    alt_rt.alt_profile_vel_cm_s = 0.0f;
                    alt_rt.alt_profile_acc_cm_s2 = 0.0f;
                }
            }
            float descend_limit =
                (vehicle_state.flight_mode == FLY_AUTOLANDING) ?
                AUTO_LAND_DESCEND_SPEED_CM_S :
                ALT_TRAJ_MAX_DESCEND_CM_S;

            if(((flight_sensor_failsafe_flags & PREFLIGHT_ERR_TOF) != 0u) &&
               (vehicle_state.flight_mode == FLY_AUTOLANDING))
            {
                descend_limit = AUTO_LAND_TOF_FAILSAFE_DESCEND_SPEED_CM_S;
            }

            
            remaining_before = final_height - alt_rt.alt_profile_height_cm;
            brake_speed = ctrl_brake_speed(ALT_TRAJ_ACCEL_CM_S2, remaining_before);
            profile_climb_limit =
                (vehicle_state.current_height <
                 ALT_TAKEOFF_VEL_LIMIT_HEIGHT_CM) ?
                ALT_TAKEOFF_VEL_CMD_LIMIT_CM_S :
                ALT_TRAJ_MAX_CLIMB_CM_S;

            if(remaining_before > 0.0f)
            {
                desired_profile_vel = LIMIT(brake_speed,
                                            0.0f,
                                            profile_climb_limit);
            }
            else if(remaining_before < 0.0f)
            {
                desired_profile_vel = -LIMIT(brake_speed,
                                             0.0f,
                                             descend_limit);
            }
            else
            {
                desired_profile_vel = 0.0f;
            }

            if(ALT_TERMINAL_DECAY_DIST_CM > 0.001f)
            {
                float terminal_dist =
                    fabsf(final_height - vehicle_state.current_height);
                float terminal_decay =
                    LIMIT(terminal_dist / ALT_TERMINAL_DECAY_DIST_CM,
                          0.0f,
                          1.0f);

                desired_profile_vel *= terminal_decay;
            }

            profile_height_before_constraints = alt_rt.alt_profile_height_cm;
            profile_vel_before = alt_rt.alt_profile_vel_cm_s;
            if(alt_rt.alt_profile_started != 0u)
            {
                alt_rt.alt_profile_vel_cm_s =
                    ctrl_slew_limit(alt_rt.alt_profile_vel_cm_s,
                                   desired_profile_vel,
                                   ALT_TRAJ_ACCEL_CM_S2,
                                   dT_s);
                alt_rt.alt_profile_height_cm += alt_rt.alt_profile_vel_cm_s * dT_s;
            }

            if(ALT_PROFILE_MAX_LEAD_CM > 0.001f)
            {
                float max_profile_height =
                    vehicle_state.current_height + ALT_PROFILE_MAX_LEAD_CM;
                float min_profile_height =
                    vehicle_state.current_height - ALT_PROFILE_MAX_LEAD_CM;

                if((final_height > vehicle_state.current_height) &&
                   (alt_rt.alt_profile_height_cm > max_profile_height) &&
                   (max_profile_height < final_height))
                {
                    alt_rt.alt_profile_height_cm = max_profile_height;
                }
                else if((final_height < vehicle_state.current_height) &&
                        (alt_rt.alt_profile_height_cm < min_profile_height) &&
                        (min_profile_height > final_height))
                {
                    alt_rt.alt_profile_height_cm = min_profile_height;
                }
            }

            remaining_after = final_height - alt_rt.alt_profile_height_cm;
            if((remaining_before != 0.0f) &&
               (remaining_before * remaining_after <= 0.0f))
            {
                alt_rt.alt_profile_height_cm = final_height;
            }
            if(((vehicle_state.current_height - final_height) *
                (alt_rt.alt_profile_height_cm - final_height) < 0.0f) &&
                (fabsf(vehicle_state.current_height - final_height) > 0.2f))
            {
                alt_rt.alt_profile_height_cm = final_height;
            }

            if(alt_rt.alt_profile_started != 0u)
            {
                alt_rt.alt_profile_vel_cm_s =
                    (alt_rt.alt_profile_height_cm - profile_height_before_constraints) /
                    dT_s;
                alt_rt.alt_profile_vel_cm_s = LIMIT(alt_rt.alt_profile_vel_cm_s,
                                             -descend_limit,
                                             ALT_TRAJ_MAX_CLIMB_CM_S);
                if((final_err > 0.0f) && (alt_rt.alt_profile_vel_cm_s < 0.0f))
                {
                    alt_rt.alt_profile_vel_cm_s = 0.0f;
                }
                else if((final_err < 0.0f) && (alt_rt.alt_profile_vel_cm_s > 0.0f))
                {
                    alt_rt.alt_profile_vel_cm_s = 0.0f;
                }
                alt_rt.alt_profile_acc_cm_s2 =
                    (alt_rt.alt_profile_vel_cm_s - profile_vel_before) / dT_s;
                alt_rt.alt_profile_acc_cm_s2 = LIMIT(alt_rt.alt_profile_acc_cm_s2,
                                              -ALT_TRAJ_ACCEL_CM_S2,
                                              ALT_TRAJ_ACCEL_CM_S2);
            }


            alt_2l_ct.exp_height = alt_rt.alt_profile_height_cm;
            alt_2l_ct.fb_height = vehicle_state.current_height;
            alt_2l_ct.height_err = alt_2l_ct.exp_height - alt_2l_ct.fb_height;
            alt_phase_update(dT_s);

            profile_error = alt_2l_ct.height_err;
            position_correction =
                stan_pid_solve(&alt_ctrl.height_pid,
                               profile_error,
                               dT_s,
                               0);
            position_correction = LIMIT(position_correction,
                                        -ALT_POS_CORR_LIMIT_CM_S,
                                        ALT_POS_CORR_LIMIT_CM_S);

            velocity_command = alt_rt.alt_profile_vel_cm_s + position_correction;
            velocity_command = LIMIT(velocity_command,
                                     -descend_limit,
                                     ALT_TRAJ_MAX_CLIMB_CM_S);
            if((alt_phase == ALT_PHASE_TAKEOFF) ||
               ((vehicle_state.flight_mode != FLY_AUTOLANDING) &&
                (vehicle_state.current_height < ALT_TAKEOFF_VEL_LIMIT_HEIGHT_CM)))
            {
                velocity_command = LIMIT(velocity_command,
                                         -ALT_TAKEOFF_VEL_CMD_LIMIT_CM_S,
                                         ALT_TAKEOFF_VEL_CMD_LIMIT_CM_S);
            }

            if((final_err < 0.0f) && (velocity_command > 0.0f))
            {
                velocity_command = 0.0f;
            }
            else if((final_err > 0.0f) && (velocity_command < 0.0f))
            {
                velocity_command = 0.0f;
            }

            if((vehicle_state.flight_mode != FLY_AUTOLANDING) &&
               (final_err > ALT_SINK_ARREST_ERR_CM) &&
               (vehicle_state.current_vel_z < -ALT_SINK_ARREST_VEL_CM_S))
            {
                float sink_extra =
                    -(vehicle_state.current_vel_z + ALT_SINK_ARREST_VEL_CM_S);
                float sink_recover_cmd =
                    ALT_SINK_ARREST_MIN_CMD_CM_S + sink_extra;

                sink_recover_cmd = LIMIT(sink_recover_cmd,
                                         ALT_SINK_ARREST_MIN_CMD_CM_S,
                                         ALT_SINK_ARREST_MAX_CMD_CM_S);
                velocity_command = MAX(velocity_command, sink_recover_cmd);
            }

            velocity_command = LIMIT(velocity_command,
                                     -descend_limit,
                                     ALT_TRAJ_MAX_CLIMB_CM_S);

            if((vehicle_state.flight_mode == FLY_AUTOLANDING) &&
               (auto_landing_active >= AUTO_LAND_STATE_DESCENDING) &&
               (vehicle_state.current_height > AUTO_LAND_MIN_DESCEND_HEIGHT_CM))
            {
                velocity_command = MIN(velocity_command,
                                       -AUTO_LAND_MIN_DESCEND_CMD_CM_S);
            }

            // All sources (trajectory, position correction and sink arrest)
            // share one final command path.  In particular, a sink-arrest
            // request must not step the velocity target from a small value
            // straight to several cm/s, otherwise it creates a climb/brake
            // limit cycle around the hold height.
            alt_rt.target_vel_z_ramped =
                ctrl_slew_limit(alt_rt.target_vel_z_ramped,
                                velocity_command,
                                ALT_VEL_CMD_SLEW_CM_S2,
                                dT_s);
            alt_2l_ct.exp_vel = alt_rt.target_vel_z_ramped;
            vehicle_setpoint.target_vel_z = alt_rt.target_vel_z_ramped;

            alt_ctrl_debug.final_height = final_height;
            alt_ctrl_debug.profile_height = alt_rt.alt_profile_height_cm;
            alt_ctrl_debug.profile_velocity = alt_rt.alt_profile_vel_cm_s;
            alt_ctrl_debug.profile_acceleration = alt_rt.alt_profile_acc_cm_s2;
            alt_ctrl_debug.profile_error = profile_error;
            alt_ctrl_debug.position_correction = position_correction;
            alt_ctrl_debug.velocity_command = alt_rt.target_vel_z_ramped;

            // 临时速度环短程辨识：HOLD、离地且油门到位连续 0.3 s 后计时。
            // 2 cm/s 保持 2.5 s，随后 4 cm/s 保持 1.5 s，最后归零。
           /* if(alt_rt.alt_vel_test_started == 0u)
            {
                if((alt_phase == ALT_PHASE_HOLD) &&
                   (throttle_ramped_debug >=
                    (system_get_hover_throttle_base() - 0.5f)) &&
                   (vehicle_state.current_height >= ALT_TAKEOFF_COMPLETE_HEIGHT_CM))
                {
                    alt_rt.alt_vel_test_ready_time_s += dT_s;
                    if(alt_rt.alt_vel_test_ready_time_s >= 0.3f)
                    {
                        alt_rt.alt_vel_test_started = 1u;
                        alt_rt.alt_vel_test_time_s = 0.0f;
                    }
                }
                else
                {
                    alt_rt.alt_vel_test_ready_time_s = 0.0f;
                }

                alt_rt.alt_vel_test_time_s = 0.0f;
                // 辨识准备阶段保持小幅上升，避免 6 cm 高度目标产生负速度，
                // 并确保能够达到测试所需的离地高度。

            }
            else
            {
                alt_rt.alt_vel_test_time_s += dT_s;
                if(alt_rt.alt_vel_test_time_s < 2.0f)
                {
                    vehicle_setpoint.target_vel_z = 2.0f;
                }
                else if(alt_rt.alt_vel_test_time_s < 3.5f)
                {
                    vehicle_setpoint.target_vel_z = 4.0f;
                }
                else
                {
                    vehicle_setpoint.target_vel_z = 0.0f;
                }
            }*/
            break;
        }

        default:
            stan_pid_reset(&alt_ctrl.height_pid);
            alt_rt.target_vel_z_ramped = 0.0f;
            alt_rt.alt_profile_valid = 0u;
            alt_rt.alt_profile_vel_cm_s = 0.0f;
            alt_rt.alt_profile_acc_cm_s2 = 0.0f;
            alt_rt.alt_profile_ready_time_s = 0.0f;
            alt_rt.alt_profile_started = 0u;
            break;
    }
}

void alt_1level_ctrl(float dT_s)
{
    static uint8_t was_landed = 1u;
    static float hover_trim = 0.0f;
    uint8_t is_landed;
    float min_throttle;
    float max_throttle;
    float authority;
    float pid_out;
    float vel_feedforward;
    float acc_feedforward;
    float vel_feedback_weight;
    float near_ground_brake;
    float dyn_gain_scale;
    float pid_weighted;
    float vel_damping;
    float dynamic_throttle;

    if(vehicle_state.armed == 0u)
    {
        vehicle_setpoint.target_throttle = 0.0f;
        alt_1l_ct.fb_climb_rate = 0.0f;
        stan_pid_reset(&alt_ctrl.vel_pid);
        alt_rt.takeoff_headroom_ramped = 0.0f;
        alt_rt.target_throttle_ramped = 0.0f;
        hover_trim = 0.0f;
        alt_rt.alt_profile_acc_cm_s2 = 0.0f;
        alt_phase_reset();
        was_landed = 1u;
        return;
    }

    is_landed = alt_is_landed();
    if(was_landed != 0u && is_landed == 0u)
    {
        stan_pid_reset(&alt_ctrl.vel_pid);
    }
    was_landed = is_landed;

    alt_1l_ct.fb_climb_rate = vehicle_state.current_vel_z;
    alt_1l_ct.exp_climb_rate = vehicle_setpoint.target_vel_z;
    alt_1l_ct.vel_err = alt_1l_ct.exp_climb_rate - alt_1l_ct.fb_climb_rate;

    if((alt_phase == ALT_PHASE_HOLD) &&
       ((alt_ctrl.vel_pid.out_i * alt_1l_ct.vel_err) < 0.0f))
    {
        alt_ctrl.vel_pid.out_i *= ALT_VEL_I_OPPOSE_DECAY;
    }

    static float i_delay_timer = 0.0f;
    uint8_t integrate_enable = 0;

    if (alt_phase == ALT_PHASE_HOLD &&
        fabsf(alt_1l_ct.vel_err) < 10.0f)
    {
        i_delay_timer += dT_s;
        if (i_delay_timer >= 0.15f)
        {
            integrate_enable = 1;
        }
    }
    else
    {
        i_delay_timer = 0.0f;
    }

    authority = alt_authority_weight();
    vel_feedback_weight = alt_vel_feedback_weight();
    pid_out = pid_solve_limited_i(&alt_ctrl.vel_pid,
                                  alt_1l_ct.vel_err,
                                  dT_s,
                                  (alt_i_enable() != 0u && vel_feedback_weight > 0.8f && integrate_enable),
                                  MAX_THROTTLE_OUTPUT,
                                  ALT_VEL_I_DISABLED_DECAY);
    vel_feedforward = LIMIT(ALT_VEL_FEEDFORWARD_GAIN * vehicle_setpoint.target_vel_z,
                            -ALT_VEL_FEEDFORWARD_LIMIT,
                            ALT_VEL_FEEDFORWARD_LIMIT);
    acc_feedforward = LIMIT(ALT_ACC_FEEDFORWARD_GAIN * alt_rt.alt_profile_acc_cm_s2,
                            -ALT_ACC_FEEDFORWARD_LIMIT,
                            ALT_ACC_FEEDFORWARD_LIMIT);

    float hover_trim_target = 0.0f;
    if (alt_phase == ALT_PHASE_HOLD && vehicle_state.current_height >= ALT_TAKEOFF_COMPLETE_HEIGHT_CM)
    {
        hover_trim_target = ALT_HOVER_TRIM_VOLT_GAIN * (alt_rt.alt_armed_voltage - vehicle_state.battery_voltage_filtered);
        hover_trim_target = LIMIT(hover_trim_target, 0.0f, ALT_HOVER_TRIM_MAX);
    }
    hover_trim = ctrl_slew_limit(hover_trim, hover_trim_target, ALT_HOVER_TRIM_SLEW, dT_s);

    if(vehicle_state.battery_voltage_filtered > 1.0f)
    {
        dyn_gain_scale =
            ALT_DYN_GAIN_REF_VOLTAGE / vehicle_state.battery_voltage_filtered;
        dyn_gain_scale *= dyn_gain_scale;
        dyn_gain_scale = LIMIT(dyn_gain_scale,
                               ALT_DYN_GAIN_SCALE_MIN,
                               ALT_DYN_GAIN_SCALE_MAX);
    }
    else
    {
        dyn_gain_scale = 1.0f;
    }

    pid_weighted = authority * vel_feedback_weight * pid_out;
    vel_damping = ALT_VEL_DAMPING_GAIN * vehicle_state.current_vel_z;
    dynamic_throttle =
        vel_feedforward +
        acc_feedforward +
        pid_weighted -
        vel_damping;

    alt_rt.throttle_val = system_get_hover_throttle_base()
                   + hover_trim
                   + dyn_gain_scale * dynamic_throttle;

    alt_ctrl_debug.authority = authority;
    alt_ctrl_debug.vel_feedback_weight = vel_feedback_weight;
    alt_ctrl_debug.pid_raw = pid_out;
    alt_ctrl_debug.pid_weighted = dyn_gain_scale * pid_weighted;
    alt_ctrl_debug.vel_feedforward =
        dyn_gain_scale * (vel_feedforward + acc_feedforward);
    alt_ctrl_debug.dyn_gain_scale = dyn_gain_scale;
    alt_ctrl_debug.hover_trim = hover_trim;
    alt_ctrl_debug.hover_base_with_trim =
        system_get_hover_throttle_base() + hover_trim;
    alt_ctrl_debug.vel_damping = dyn_gain_scale * vel_damping;

    near_ground_brake = 0.0f;
    if((alt_phase != ALT_PHASE_TAKEOFF) &&
       (vehicle_state.current_height < ALT_NEAR_GROUND_BRAKE_HEIGHT_CM) &&
       (vehicle_state.current_vel_z > 0.0f))
    {
        near_ground_brake =
            LIMIT(ALT_NEAR_GROUND_BRAKE_GAIN * vehicle_state.current_vel_z,
                  0.0f,
                  ALT_NEAR_GROUND_BRAKE_LIMIT);
        alt_rt.throttle_val -= near_ground_brake;
    }

    alt_throttle_limit(authority, hover_trim, &min_throttle, &max_throttle);
    alt_ctrl_debug.near_ground_brake = near_ground_brake;
    alt_ctrl_debug.throttle_pre_limit = alt_rt.throttle_val;
    alt_ctrl_debug.min_throttle = min_throttle;
    alt_ctrl_debug.max_throttle = max_throttle;

    vehicle_setpoint.target_throttle =
        LIMIT(alt_rt.throttle_val,
              min_throttle,
              max_throttle);
    alt_ctrl_debug.throttle_post_limit =
        vehicle_setpoint.target_throttle;

    if((alt_phase == ALT_PHASE_TAKEOFF) &&
       (vehicle_state.current_height < ALT_TAKEOFF_BOOST_HEIGHT_CM) &&
       ((vehicle_setpoint.target_height - vehicle_state.current_height) >
        ALT_TAKEOFF_BOOST_MIN_ERR_CM))
    {
        float boost_weight = 1.0f;
        float takeoff_min_throttle;
        float target_headroom;
        float headroom_step;

        if(vehicle_state.current_vel_z > 0.0f)
        {
            boost_weight = 1.0f -
                (vehicle_state.current_vel_z / ALT_TAKEOFF_BOOST_FADE_VEL_CM_S);
            boost_weight = LIMIT(boost_weight, 0.0f, 1.0f);
        }

        if(boost_weight > 0.0f)
        {
            target_headroom = ALT_TAKEOFF_MIN_HEADROOM * boost_weight;
            headroom_step = ALT_TAKEOFF_HEADROOM_SLEW * dT_s;
            alt_rt.takeoff_headroom_ramped +=
                LIMIT(target_headroom - alt_rt.takeoff_headroom_ramped,
                      -headroom_step,
                      headroom_step);

            takeoff_min_throttle = system_get_hover_throttle_base() +
                hover_trim + alt_rt.takeoff_headroom_ramped;

            vehicle_setpoint.target_throttle =
                LIMIT(vehicle_setpoint.target_throttle,
                      takeoff_min_throttle,
                      max_throttle);
        }
        else
        {
            alt_rt.takeoff_headroom_ramped = 0.0f;
        }
    }
    else
    {
        alt_rt.takeoff_headroom_ramped = 0.0f;
    }

    alt_rt.target_throttle_ramped = ctrl_slew_limit(alt_rt.target_throttle_ramped,
                                            vehicle_setpoint.target_throttle,
                                            ALT_THROTTLE_SLEW,
                                            dT_s);
    vehicle_setpoint.target_throttle = alt_rt.target_throttle_ramped;
    alt_ctrl_debug.throttle_final = alt_rt.target_throttle_ramped;
}

void alt_ctrl_update(float dT_s)
{
    alt_2level_ctrl(dT_s);
    alt_1level_ctrl(dT_s);
}
