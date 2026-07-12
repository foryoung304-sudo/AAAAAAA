#include "zf_common_headfile.h"


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
} loc_runtime_t;

static loc_runtime_t loc_rt =
{
    .last_flight_mode = FLY_POS_HOLD
};


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

static uint8_t loc_damping_ready(void)
{
    return ((loc_output_base_ready() != 0u) &&
            (vehicle_state.current_height >= LOC_ENABLE_HEIGHT_CM));
}

static float loc_hold_enable_height(void)
{
    float enable_height = vehicle_setpoint.target_height - LOC_ENABLE_TARGET_MARGIN_CM;

    if(enable_height < LOC_ENABLE_HEIGHT_CM)
    {
        enable_height = LOC_ENABLE_HEIGHT_CM;
    }

    return enable_height;
}

static uint8_t loc_hold_entry_ready(void)
{
    float enable_height = loc_hold_enable_height();
    float horiz_spd = sqrtf(vehicle_state.current_vel_x * vehicle_state.current_vel_x +
                            vehicle_state.current_vel_y * vehicle_state.current_vel_y);

    return ((loc_damping_ready() != 0u) &&
            (alt_phase == ALT_PHASE_HOLD) &&
            (vehicle_state.current_height >= enable_height) &&
            (fabsf(vehicle_state.current_vel_z) <= LOC_ENABLE_VZ_MAX_CM_S) &&
            (horiz_spd <= LOC_ENABLE_HORIZ_VEL_MAX_CM_S));
}

static uint8_t loc_hold_should_release(void)
{
    float release_height =
        loc_hold_enable_height() - LOC_HOLD_RELEASE_MARGIN_CM;

    if(release_height < LOC_ENABLE_HEIGHT_CM)
    {
        release_height = LOC_ENABLE_HEIGHT_CM;
    }

    return ((loc_damping_ready() == 0u) ||
            (alt_phase != ALT_PHASE_HOLD) ||
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

static void loc_reset_vel_debug(void)
{
    loc_1l_ct.vel_err_x_body = 0.0f;
    loc_1l_ct.vel_err_y_body = 0.0f;
    loc_1l_ct.raw_target_roll = 0.0f;
    loc_1l_ct.raw_target_pitch = 0.0f;
    loc_1l_ct.loc_weight = loc_rt.loc_weight;
    loc_1l_ct.loc_ready = 0u;
    loc_1l_ct.loc_hold_ready = 0u;
}

static void loc_hold_current_without_output(uint8_t clear_attitude_setpoint)
{
    vehicle_setpoint.target_pos_x = vehicle_state.current_pos_x;
    vehicle_setpoint.target_pos_y = vehicle_state.current_pos_y;
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
    float height_span = enable_height - LOC_ENABLE_HEIGHT_CM;
    float height_ratio = 1.0f;
    float vz_abs = fabsf(vehicle_state.current_vel_z);
    float reduce_ratio =
        ctrl_smoothstep01((vz_abs - LOC_DAMPING_VZ_FULL_CM_S) /
                          (LOC_DAMPING_VZ_REDUCE_CM_S - LOC_DAMPING_VZ_FULL_CM_S));
    float height_angle_limit;

    if(height_span > 0.001f)
    {
        height_ratio =
            ctrl_smoothstep01((vehicle_state.current_height - LOC_ENABLE_HEIGHT_CM) /
                              height_span);
    }

    height_angle_limit =
        LOC_DAMPING_MIN_OUTPUT_ANGLE_DEG +
        (LOC_DAMPING_MAX_OUTPUT_ANGLE_DEG -
         LOC_DAMPING_MIN_OUTPUT_ANGLE_DEG) * height_ratio;

    return height_angle_limit -
           (height_angle_limit - LOC_DAMPING_MIN_OUTPUT_ANGLE_DEG) * reduce_ratio;
}


// ============================================================================
// Coordinate transform helpers
// ============================================================================
static void loc_earth_vel_err_to_body(float err_x_earth,
                                      float err_y_earth,
                                      float *err_x_body,
                                      float *err_y_body)
{
    *err_x_body =  err_x_earth * vehicle_state.yaw_cos + err_y_earth * vehicle_state.yaw_sin;
    *err_y_body = -err_x_earth * vehicle_state.yaw_sin + err_y_earth * vehicle_state.yaw_cos;
}
// ============================================================================
// Public interface
// ============================================================================

// 位置控制初始化
void loc_ctrl_init(void)
{
    // 位置环PID参数（外环）
    loc_ctrl.pos_pid[0] = (pid_param_t){     // X方向
        .kp = 1.0f,
        .ki = 0.0f,
        .kd = 0.0f,
        .i_max = 0.0f,
        .p_max = 5.0f,
        .d_max = 0.0f,
        .low_pass = 0.1f
    };
    
    loc_ctrl.pos_pid[1] = (pid_param_t){     // Y方向
        .kp = 1.0f,
        .ki = 0.0f,
        .kd = 0.0f,
        .i_max = 0.0f,
        .p_max = 5.0f,
        .d_max = 0.0f,
        .low_pass = 0.1f
    };
    
    // 速度环PID参数（内环）
    loc_ctrl.vel_pid[0] = (pid_param_t){     // Vx方向
        .kp = 2.0f,
        .ki = 0.5f,
        .kd = 0.5f,
        .i_max = 20.0f,
        .p_max = MAX_VEL_CT_VAL,
        .d_max = 10.0f,
        .low_pass = 0.2f
    };
    
    loc_ctrl.vel_pid[1] = (pid_param_t){     // Vy方向
        .kp = 2.0f,
        .ki = 0.5f,
        .kd = 0.5f,
        .i_max = 20.0f,
        .p_max = MAX_VEL_CT_VAL,
        .d_max = 10.0f,
        .low_pass = 0.2f
    };

}


// 位置环控制（外环）
void loc_2level_ctrl(float dT_s)
{

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

        loc_2l_ct.exp_pos_x = vehicle_setpoint.target_pos_x;
        loc_2l_ct.exp_pos_y = vehicle_setpoint.target_pos_y;
        loc_2l_ct.fb_pos_x = vehicle_state.current_pos_x;
        loc_2l_ct.fb_pos_y = vehicle_state.current_pos_y;

        if(loc_hold_ready() == 0u)
        {
            vehicle_setpoint.target_pos_x = vehicle_state.current_pos_x;
            vehicle_setpoint.target_pos_y = vehicle_state.current_pos_y;
            vehicle_setpoint.target_vel_x = 0.0f;
            vehicle_setpoint.target_vel_y = 0.0f;

            loc_2l_ct.exp_pos_x = vehicle_state.current_pos_x;
            loc_2l_ct.exp_pos_y = vehicle_state.current_pos_y;
            loc_2l_ct.exp_vel_x = 0.0f;
            loc_2l_ct.exp_vel_y = 0.0f;
            loc_rt.target_vel_x_ramped = 0.0f;
            loc_rt.target_vel_y_ramped = 0.0f;
            loc_reset_pos_pids();
            return;
        }

        float final_err_x=loc_2l_ct.exp_pos_x-loc_2l_ct.fb_pos_x;
        float final_err_y=loc_2l_ct.exp_pos_y-loc_2l_ct.fb_pos_y;

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

        loc_2l_ct.exp_vel_x = stan_pid_solve(&loc_ctrl.pos_pid[0], final_err_x, dT_s, 0);
        loc_2l_ct.exp_vel_y = stan_pid_solve(&loc_ctrl.pos_pid[1], final_err_y, dT_s, 0);

        loc_2l_ct.exp_vel_x=ctrl_terminal_clamp(final_err_x,loc_2l_ct.exp_vel_x);
        loc_2l_ct.exp_vel_y=ctrl_terminal_clamp(final_err_y,loc_2l_ct.exp_vel_y);

        float brake_speed_x = ctrl_brake_speed(LOC_TRAJ_ACCEL_CM_S2, final_err_x);
        float brake_speed_y = ctrl_brake_speed(LOC_TRAJ_ACCEL_CM_S2, final_err_y);

            loc_2l_ct.exp_vel_x = LIMIT(loc_2l_ct.exp_vel_x,
                                        -brake_speed_x,
                                        brake_speed_x);

            loc_2l_ct.exp_vel_y = LIMIT(loc_2l_ct.exp_vel_y,
                                        -brake_speed_y,
                                        brake_speed_y);

        float target_vel_x = LIMIT(loc_2l_ct.exp_vel_x, -MAX_HORIZONTAL_SPEED, MAX_HORIZONTAL_SPEED);
        float target_vel_y = LIMIT(loc_2l_ct.exp_vel_y, -MAX_HORIZONTAL_SPEED, MAX_HORIZONTAL_SPEED);

            loc_rt.target_vel_x_ramped =
                ctrl_slew_limit(loc_rt.target_vel_x_ramped,
                                target_vel_x,
                                LOC_TARGET_VEL_SLEW_CM_S2,
                                dT_s);

            loc_rt.target_vel_y_ramped =
                ctrl_slew_limit(loc_rt.target_vel_y_ramped,
                                target_vel_y,
                                LOC_TARGET_VEL_SLEW_CM_S2,
                                dT_s);

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

        float vel_err_x_body;
        float vel_err_y_body;


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

            vel_err_x_body = loc_soft_deadband(loc_rt.vel_err_x_body_filtered,
                                               deadband);
            vel_err_y_body = loc_soft_deadband(loc_rt.vel_err_y_body_filtered,
                                               deadband);
        }

        loc_1l_ct.vel_err_x_body = vel_err_x_body;
        loc_1l_ct.vel_err_y_body = vel_err_y_body;

     
        vehicle_setpoint.target_pitch = -stan_pid_solve(&loc_ctrl.vel_pid[0], vel_err_x_body, dT_s, 0);
        vehicle_setpoint.target_roll  =  stan_pid_solve(&loc_ctrl.vel_pid[1], vel_err_y_body, dT_s, 0);
        float angle_limit = LOC_MAX_OUTPUT_ANGLE_DEG;
        if(loc_hold_ready() == 0u)
        {
            angle_limit = loc_damping_angle_limit();
        }

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
    if(vehicle_state.flight_mode==FLY_POS_HOLD && loc_rt.last_flight_mode!=FLY_POS_HOLD)
    {
        vehicle_setpoint.target_pos_x = vehicle_state.current_pos_x;
        vehicle_setpoint.target_pos_y = vehicle_state.current_pos_y;
    }

    loc_rt.last_flight_mode = vehicle_state.flight_mode;

    ready = loc_output_ready();
    if(ready == 0u)
    {
        loc_rt.loc_ready_last = 0u;
        loc_rt.loc_weight = 0.0f;
        loc_rt.loc_ready_time_s = 0.0f;
        loc_rt.brake_weight = 0.0f;
        loc_rt.loc_hold_active = 0u;
    }
    else
    {
        if(loc_rt.loc_ready_last == 0u)
        {
            vehicle_setpoint.target_pos_x = vehicle_state.current_pos_x;
            vehicle_setpoint.target_pos_y = vehicle_state.current_pos_y;
            loc_reset_runtime_output();
            loc_reset_pids();
            loc_reset_vel_debug();
            loc_rt.loc_weight = 0.0f;
            loc_rt.loc_ready_time_s = 0.0f;
            loc_rt.brake_weight = 0.0f;
            loc_rt.loc_hold_active = 0u;
            loc_rt.loc_hold_stable_time_s = 0.0f;
        }

        if(loc_rt.loc_hold_active != 0u)
        {
            if(loc_hold_should_release() != 0u)
            {
                loc_rt.loc_hold_active = 0u;
                loc_rt.loc_hold_stable_time_s = 0.0f;
            }
        }
        else
        {
            if(loc_hold_entry_ready() != 0u)
            {
                loc_rt.loc_hold_stable_time_s += dT_s;
                if(loc_rt.loc_hold_stable_time_s >= 0.3f)
                {
                    /* Start the newly captured hold target */
                    vehicle_setpoint.target_pos_x = vehicle_state.current_pos_x;
                    vehicle_setpoint.target_pos_y = vehicle_state.current_pos_y;
                    loc_2l_ct.exp_pos_x = vehicle_state.current_pos_x;
                    loc_2l_ct.exp_pos_y = vehicle_state.current_pos_y;
                    loc_reset_pos_pids();
                    // Remove braking I bias without erasing derivative
                    // history; a full reset would create a D transient at
                    // the damping-to-hold transition.
                    loc_clear_vel_pid_integrators();
                    loc_rt.loc_hold_active = 1u;
                    loc_rt.loc_hold_stable_time_s = 0.0f;
                }
            }
            else
            {
                loc_rt.loc_hold_stable_time_s = 0.0f;
            }
        }

        loc_rt.loc_ready_time_s += dT_s;
        loc_rt.loc_weight += dT_s / LOC_BLEND_TIME_S;
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
