#ifndef __TOF_H__
#define __TOF_H__

#include "zf_common_headfile.h"

#define TOF_SENSOR_DL1B                  0
#define TOF_SENSOR_VL53L8                1

#ifndef TOF_SENSOR_TYPE
#define TOF_SENSOR_TYPE                  TOF_SENSOR_VL53L8
#endif



#define HEIGHT_EKF_LANDED_THR_MARGIN    3.0f
#define HEIGHT_ZUPT_INNOV_CM            0.2f
#define HEIGHT_ZUPT_DELTA_CM            0.3f
#define HEIGHT_ZUPT_VEL_DECAY           0.98f
#define HEIGHT_NEAR_GROUND_RECOVER_CM   11.0f
#define HEIGHT_ACC_Z_EKF_GAIN           0.20f
#define HEIGHT_ACC_Z_LIMIT_CM_S2        300.0f
#define HEIGHT_ACC_Z_LPF_ALPHA          0.18f
#define HEIGHT_ACC_Z_BIAS_ALPHA         0.01f
#define HEIGHT_ACC_Z_BIAS_LEARN_LIMIT   50.0f
#define HEIGHT_ACC_Z_BIAS_GYRO_LIMIT    5.0f
#define HEIGHT_EKF_Q_Z                  25.0f
#define HEIGHT_EKF_Q_VZ                 400.0f
#define HEIGHT_TOF_R_CM2                25.0f
#define HEIGHT_TOF_VEL_OBS_ENABLE       1
#define HEIGHT_TOF_VEL_LIMIT_CM_S       80.0f
#define HEIGHT_TOF_VEL_REJECT_CM_S      45.0f
#define HEIGHT_TOF_VEL_LPF_ALPHA        0.55f
#define HEIGHT_TOF_VEL_R_CM2_S2         200.0f
#define TOF_STALE_TIMEOUT_US            120000u
#define TOF_STUCK_TIMEOUT_US            500000u
#define TOF_PREFLIGHT_MAX_HEIGHT_CM     30.0f
#define TOF_PREFLIGHT_VALID_FRAMES      5u
#define TOF_RECOVERY_RETRY_US           2000000u
#define TOF_GROUND_FAULT_CONFIRM_US     500000u
/* Downward ToF lens is 10 cm below the aircraft center of gravity. */
#define TOF_TO_CG_VERTICAL_OFFSET_CM    10.0f

extern float tof_dist_cm;
extern float acc_z_cm_s2;
extern float acc_z_world_raw_cm_s2;
extern float acc_z_world_bias_cm_s2;
extern float acc_z_world_corrected_cm_s2;
extern float acc_z_world_lpf_cm_s2;
extern float tof_diff_vel_debug;
extern float height_tof_vel_raw_debug;
extern float height_tof_vel_filtered_debug;
extern float height_vel_innov_debug;
extern float height_tof_vel_dt_ms_debug;
extern uint8_t height_vel_update_used_debug;
extern float height_cm;
typedef struct
{
    uint8_t raw_valid;
    uint8_t ekf_used;
    uint8_t near_ground_recover;
    uint8_t gate_clipped;
    uint8_t range_reject;
    uint8_t stale;
    uint8_t stuck;
    uint8_t recovery_active;
    uint8_t recovery_last_status;
    uint8_t valid_frame_count;
    uint8_t streamcount;
    float raw_dist_cm;
    float height_cm;
    float height_est_cm;
    float vel_z_cm_s;
    float innov_cm;
    float last_height_cm;
    uint32_t last_fresh_time_us;
    uint32_t unchanged_time_us;
    uint32_t recovery_count;
} tof_health_t;

extern tof_health_t tof_health;

void tof_init(void);
void tof_get_distance(void);
uint8_t tof_health_ok(void);
void update_current_height(float dT_s);
void ekf_lite_predict_height(float dT_s);
void ekf_lite_update_height(void);

#endif
