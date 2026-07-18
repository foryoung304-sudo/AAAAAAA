#ifndef __FLOW_H__
#define __FLOW_H__

#include "zf_common_headfile.h"

#define LC302_LOW_LIGHT_MODE 0

#define FLOW_RAW_DEADZONE_DAY_CM 0.12f
#define FLOW_GYRO_DEADZONE_DAY_CM 0.03f
#define FLOW_RAW_DEADZONE_NIGHT_CM 1.50f
#define FLOW_GYRO_DEADZONE_NIGHT_CM 0.30f

#if LC302_LOW_LIGHT_MODE
#define FLOW_RAW_DEADZONE_CM FLOW_RAW_DEADZONE_NIGHT_CM
#define FLOW_GYRO_DEADZONE_CM FLOW_GYRO_DEADZONE_NIGHT_CM
#define FLOW_MOTION_CONFIRM_FRAMES 2
#else
#define FLOW_RAW_DEADZONE_CM FLOW_RAW_DEADZONE_DAY_CM
#define FLOW_GYRO_DEADZONE_CM FLOW_GYRO_DEADZONE_DAY_CM
#define FLOW_MOTION_CONFIRM_FRAMES 1
#endif
#define FLOW_VALID_LOST_FRAMES 5
#define FLOW_VALID_RECOVER_FRAMES 2
#define FLOW_GYRO_COMP_GAIN_X 1.85f
#define FLOW_GYRO_COMP_GAIN_Y 1.20f
/* LC302 rotation compensation must use the gyro angle accumulated over the
 * actual sensor-frame interval.  Filtered attitude deltas attenuate/lag fast
 * roll and pitch motion and leave height-scaled false optical-flow velocity. */
#define FLOW_RP_USE_ATTITUDE_DELTA 0
#define FLOW_YAW_REJECT_DPS 150.0f
#define FLOW_YAW_USE_ATTITUDE_DELTA 1
#define FLOW_CALIB_RAW_ONLY 0
#define FLOW_CALIB_USE_RAW_DEADZONE 0
#define FLOW_CALIB_USE_GATE 0

#define FLOW_SENSOR_LC302 1
#define FLOW_SENSOR_PMW3901 0
#define FLOW_SENSOR_TYPE FLOW_SENSOR_LC302

#define LC302_FLOW_SCALE_X_BASE 1.34f
#define LC302_FLOW_SCALE_Y_BASE 1.60f
#define LC302_LOW_LIGHT_SCALE_GAIN 15.0f
#if LC302_LOW_LIGHT_MODE
#define LC302_FLOW_SCALE_X                                                     \
  (LC302_FLOW_SCALE_X_BASE * LC302_LOW_LIGHT_SCALE_GAIN)
#define LC302_FLOW_SCALE_Y                                                     \
  (LC302_FLOW_SCALE_Y_BASE * LC302_LOW_LIGHT_SCALE_GAIN)
#else
#define LC302_FLOW_SCALE_X LC302_FLOW_SCALE_X_BASE
#define LC302_FLOW_SCALE_Y LC302_FLOW_SCALE_Y_BASE
#endif
#define LC302_QUALITY_MIN 100
#define LC302_SWAP_XY 1
#define LC302_INSTALL_YAW_DEG 0.0f
#define FLOW_OFFSET_X_CM 9.3f
#define FLOW_OFFSET_Y_CM 1.4f
#define FLOW_YAW_OFFSET_ENABLE 1
#define FLOW_YAW_VISION_ENABLE 1
#define FLOW_YAW_VISION_GAIN_X -0.26f
#define FLOW_YAW_VISION_GAIN_Y 0.45f

#define FLOW_DEBUG_ENABLE 0
#define FLOW_DEBUG_DIV 1
#define FLOW_DEBUG_GATE 0
#define FLOW_DEBUG_ATTITUDE 0
#define FLOW_DEBUG_BODY 0
#define FLOW_DEBUG_EARTH 0
#define FLOW_DEBUG_LC302 0

#define FLOW_LITE_VEL_GATE_BASE_CMS 25.0f
#define FLOW_LITE_MAX_ACCEL_CMS2 300.0f
#define FLOW_LITE_VEL_GATE_MAX_CMS 60.0f
#define FLOW_LITE_MAX_DT_S 0.10f
#define FLOW_LITE_R_VEL 600.0f
#define FLOW_LITE_Q_VEL 200.0f
/* The EKF must not consume a single LC302 frame directly.  A frame-scale
 * displacement outlier otherwise becomes an immediate velocity command to
 * the horizontal controller. */
#define FLOW_EKF_VEL_LPF_ALPHA 0.30f
#define FLOW_LC302_NOMINAL_FRAME_DT_S (1.0f / 48.0f)
#define FLOW_LC302_MIN_FRAME_DT_S 0.005f
#define FLOW_LC302_MAX_FRAME_DT_S 0.050f
/* 64 x 2 ms keeps enough gyro history for an accumulated LC302 frame plus
 * normal UART/control scheduling jitter. */
#define FLOW_GYRO_HISTORY_LEN 64u
#define FLOW_GYRO_SHADOW_COUNT 6u
#define FLOW_LITE_POS_LIMIT_CM 500.0f
#define FLOW_LITE_VEL_LIMIT_CMS 350.0f

extern uint8_t flow_cnt;

typedef struct {
  uint8_t raw_valid;
  uint8_t quality_valid;
  uint8_t height_valid;
  uint8_t tilt_valid;
  uint8_t yaw_valid;
  uint8_t obs_valid;
  uint8_t ekf_used;
  uint8_t gate_clipped;
  uint8_t quality;
  uint16_t lc302_accum_count;
  uint32_t lc302_integration_us;
  uint32_t lc302_frame_count;
  uint32_t lc302_frame_start_rx_us;
  uint32_t lc302_frame_rx_us;
  uint32_t gyro_history_hit_count;
  uint32_t gyro_history_fallback_count;
  uint8_t gyro_history_selected_hit;
  uint8_t gyro_shadow_hit_mask;
  float height_cm;
  float raw_dx_cm;
  float raw_dy_cm;
  float true_dx_body_cm;
  float true_dy_body_cm;
  float earth_dx_cm;
  float earth_dy_cm;
  float flow_pos_x_cm;
  float flow_pos_y_cm;
  float flow_vel_x_cm_s;
  float flow_vel_y_cm_s;
  float obs_vx_cm_s;
  float obs_vy_cm_s;
  float innov_vx_cm_s;
  float innov_vy_cm_s;
  float obs_dt_s;
  float gate_limit_cm_s;
  float gyro_comp_y_shadow_cm[FLOW_GYRO_SHADOW_COUNT];
  float true_dy_shadow_cm[FLOW_GYRO_SHADOW_COUNT];
} flow_health_t;

extern flow_health_t flow_health;

void flow_gyro_integrate(float dT_s);
void update_position_from_flow(float dT_s);
void flow_debug_print(void);
void ekf_lite_predict_xy(float dT_s);
void ekf_lite_update_xy(float dT_s);

#endif
