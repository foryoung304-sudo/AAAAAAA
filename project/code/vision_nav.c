#include "vision_nav.h"
#include "beacon.h"
#include "ekf_lite.h"
#include "param.h"

vision_nav_obs_t vision_nav_obs = {0};
uint8_t vision_nav_current_beacon_id = 0;

static const vision_nav_point_t vision_nav_beacon_map[VISION_NAV_BEACON_COUNT] = {
    {0.0f, 0.0f},
    {100.0f, 0.0f},
    {200.0f, 0.0f},
};

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

void vision_nav_init(void)
{
    vision_nav_reset();
}

void vision_nav_reset(void)
{
    memset(&vision_nav_obs, 0, sizeof(vision_nav_obs));
    vision_nav_current_beacon_id = 0;
}

void vision_nav_set_current_beacon(uint8_t beacon_id)
{
    if(beacon_id < VISION_NAV_BEACON_COUNT)
    {
        vision_nav_current_beacon_id = beacon_id;
        vision_nav_obs.stable_frames = 0;
    }
}

void vision_nav_next_beacon(void)
{
    if((vision_nav_current_beacon_id + 1u) < VISION_NAV_BEACON_COUNT)
    {
        vision_nav_current_beacon_id++;
        vision_nav_obs.stable_frames = 0;
    }
}

static uint8_t vision_nav_gate_ok(float residual_x, float residual_y)
{
    if(beacon.status != BEACON_FOUND) return 0u;
    if(vehicle_state.current_height < VISION_NAV_MIN_HEIGHT_CM) return 0u;
    if(vehicle_state.current_height > VISION_NAV_MAX_HEIGHT_CM) return 0u;
    if(vision_nav_absf(vehicle_state.current_roll) > VISION_NAV_MAX_ATT_DEG) return 0u;
    if(vision_nav_absf(vehicle_state.current_pitch) > VISION_NAV_MAX_ATT_DEG) return 0u;
    if(vision_nav_absf(residual_x) > VISION_NAV_MAX_RESIDUAL_CM) return 0u;
    if(vision_nav_absf(residual_y) > VISION_NAV_MAX_RESIDUAL_CM) return 0u;
    return 1u;
}

static void vision_nav_update_ekf_xy(float meas_x, float meas_y, float confidence)
{
#if VISION_NAV_ENABLE_EKF_UPDATE
    float r = 900.0f / vision_nav_clampf(confidence, 0.25f, 1.0f);
    float innov_x = meas_x - ekf_lite_state.x;
    float innov_y = meas_y - ekf_lite_state.y;
    float kx = ekf_lite_p_xy[0] / (ekf_lite_p_xy[0] + r);
    float ky = ekf_lite_p_xy[1] / (ekf_lite_p_xy[1] + r);

    ekf_lite_state.x += kx * innov_x;
    ekf_lite_state.y += ky * innov_y;
    ekf_lite_p_xy[0] *= (1.0f - kx);
    ekf_lite_p_xy[1] *= (1.0f - ky);
    vehicle_state.current_pos_x = ekf_lite_state.x;
    vehicle_state.current_pos_y = ekf_lite_state.y;
#else
    (void)meas_x;
    (void)meas_y;
    (void)confidence;
#endif
}

void vision_nav_update(void)
{
    const vision_nav_point_t *target = &vision_nav_beacon_map[vision_nav_current_beacon_id];
    float rel_x_e;
    float rel_y_e;
    float drone_x;
    float drone_y;
    float residual_x;
    float residual_y;
    float confidence;
    uint8_t gate_ok;

    vision_nav_obs.valid = 0u;
    vision_nav_obs.used_for_ekf = 0u;
    vision_nav_obs.beacon_id = vision_nav_current_beacon_id;

    if(vision_nav_current_beacon_id >= VISION_NAV_BEACON_COUNT)
    {
        vision_nav_obs.stable_frames = 0u;
        return;
    }

    rel_x_e = beacon_body_x * vehicle_state.yaw_cos -
              beacon_body_y * vehicle_state.yaw_sin;
    rel_y_e = beacon_body_x * vehicle_state.yaw_sin +
              beacon_body_y * vehicle_state.yaw_cos;

    drone_x = target->x_cm - rel_x_e;
    drone_y = target->y_cm - rel_y_e;
    residual_x = drone_x - ekf_lite_state.x;
    residual_y = drone_y - ekf_lite_state.y;

    gate_ok = vision_nav_gate_ok(residual_x, residual_y);
    if(gate_ok)
    {
        if(vision_nav_obs.stable_frames < 255u)
        {
            vision_nav_obs.stable_frames++;
        }
    }
    else
    {
        vision_nav_obs.stable_frames = 0u;
    }

    confidence = (float)vision_nav_obs.stable_frames /
                 (float)VISION_NAV_MIN_STABLE_FRAMES;
    confidence = vision_nav_clampf(confidence, 0.0f, 1.0f);

    vision_nav_obs.drone_x_cm = drone_x;
    vision_nav_obs.drone_y_cm = drone_y;
    vision_nav_obs.residual_x_cm = residual_x;
    vision_nav_obs.residual_y_cm = residual_y;
    vision_nav_obs.confidence = confidence;
    vision_nav_obs.valid = (gate_ok &&
        vision_nav_obs.stable_frames >= VISION_NAV_MIN_STABLE_FRAMES) ? 1u : 0u;

    if(vision_nav_obs.valid)
    {
        vision_nav_update_ekf_xy(drone_x, drone_y, confidence);
        vision_nav_obs.used_for_ekf = (VISION_NAV_ENABLE_EKF_UPDATE != 0) ? 1u : 0u;
    }

    {
        static uint32_t last_print_us = 0u;
        if(system_time_us() - last_print_us > VISION_NAV_PRINT_INTERVAL_US)
        {
            last_print_us = system_time_us();
            printf("VNAV: id=%u obs(%.1f,%.1f) ekf(%.1f,%.1f) res(%.1f,%.1f) valid=%u stable=%u\n",
                   vision_nav_obs.beacon_id,
                   vision_nav_obs.drone_x_cm,
                   vision_nav_obs.drone_y_cm,
                   ekf_lite_state.x,
                   ekf_lite_state.y,
                   vision_nav_obs.residual_x_cm,
                   vision_nav_obs.residual_y_cm,
                   vision_nav_obs.valid,
                   vision_nav_obs.stable_frames);
        }
    }
}
