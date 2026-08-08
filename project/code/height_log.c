#include "zf_common_headfile.h"

typedef struct
{
    uint32_t time_us;
    int16_t tof_mm;
    int16_t height_centi_cm;
    int16_t ekf_z_centi_cm;
    int16_t ekf_vz_centi_cm_s;
    int16_t acc_raw_deci_cm_s2;
    int16_t acc_corrected_deci_cm_s2;
    int16_t acc_lpf_deci_cm_s2;
    int16_t acc_ekf_deci_cm_s2;
    int16_t tof_diff_centi_cm_s;
    int16_t innov_centi_cm;
    int16_t throttle_centi;
    int16_t target_height_centi_cm;
    int16_t current_vz_centi_cm_s;
    int16_t target_vz_centi_cm_s;
    int16_t final_height_centi_cm;
    int16_t profile_height_centi_cm;
    int16_t profile_vz_centi_cm_s;
    int16_t profile_accel_deci_cm_s2;
    int16_t position_correction_centi_cm_s;
    int16_t height_loop_i_centi_cm_s;
    int16_t vel_loop_output_centi;
    int16_t vel_loop_i_centi;
    uint8_t tof_used;
    uint8_t armed;
} height_log_sample_t;

static height_log_sample_t height_log_buf[HEIGHT_LOG_SAMPLE_COUNT];
static volatile uint16_t height_log_count = 0u;
static volatile uint8_t height_log_recording = 0u;
static volatile uint8_t height_log_ready = 0u;
static uint16_t height_log_dump_index = 0u;

static int16_t height_log_i16(float value)
{
    if(value > 32767.0f)
    {
        return 32767;
    }
    if(value < -32768.0f)
    {
        return -32768;
    }
    return (int16_t)value;
}

void height_log_update_50hz(void)
{
    uint8_t armed = vehicle_state.armed;

    if((armed != 0u) && (height_log_recording == 0u) &&
       (height_log_ready == 0u))
    {
        height_log_count = 0u;
        height_log_recording = 1u;
    }

    if((armed == 0u) && (height_log_recording != 0u))
    {
        height_log_recording = 0u;
        height_log_ready = (height_log_count > 0u) ? 1u : 0u;
        height_log_dump_index = 0u;
        return;
    }

    if((height_log_recording != 0u) &&
       (height_log_count < HEIGHT_LOG_SAMPLE_COUNT))
    {
        uint16_t index = height_log_count;
        height_log_sample_t *sample = &height_log_buf[index];

        sample->time_us = system_time_us();
        sample->tof_mm = height_log_i16(tof_dist_cm * 10.0f);
        sample->height_centi_cm = height_log_i16(height_cm * 100.0f);
        sample->ekf_z_centi_cm = height_log_i16(ekf_lite_state.z * 100.0f);
        sample->ekf_vz_centi_cm_s = height_log_i16(ekf_lite_state.vz * 100.0f);
        sample->acc_raw_deci_cm_s2 =
            height_log_i16(acc_z_world_raw_cm_s2 * 10.0f);
        sample->acc_corrected_deci_cm_s2 =
            height_log_i16(acc_z_world_corrected_cm_s2 * 10.0f);
        sample->acc_lpf_deci_cm_s2 =
            height_log_i16(acc_z_world_lpf_cm_s2 * 10.0f);
        sample->acc_ekf_deci_cm_s2 = height_log_i16(acc_z_cm_s2 * 10.0f);
        sample->tof_diff_centi_cm_s =
            height_log_i16(tof_diff_vel_debug * 100.0f);
        sample->innov_centi_cm =
            height_log_i16(ekf_lite_health.tof_innov * 100.0f);
        sample->throttle_centi =
            height_log_i16(vehicle_setpoint.target_throttle * 100.0f);
        sample->target_height_centi_cm =
            height_log_i16(vehicle_setpoint.target_height * 100.0f);
        sample->current_vz_centi_cm_s =
            height_log_i16(vehicle_state.current_vel_z * 100.0f);
        sample->target_vz_centi_cm_s =
            height_log_i16(vehicle_setpoint.target_vel_z * 100.0f);
        sample->final_height_centi_cm =
            height_log_i16(alt_ctrl_debug.final_height * 100.0f);
        sample->profile_height_centi_cm =
            height_log_i16(alt_ctrl_debug.profile_height * 100.0f);
        sample->profile_vz_centi_cm_s =
            height_log_i16(alt_ctrl_debug.profile_velocity * 100.0f);
        sample->profile_accel_deci_cm_s2 =
            height_log_i16(alt_ctrl_debug.profile_acceleration * 10.0f);
        sample->position_correction_centi_cm_s =
            height_log_i16(alt_ctrl_debug.position_correction * 100.0f);
        sample->height_loop_i_centi_cm_s =
            height_log_i16(alt_ctrl.height_pid.out_i * 100.0f);
        sample->vel_loop_output_centi =
            height_log_i16(alt_ctrl_debug.pid_raw * 100.0f);
        sample->vel_loop_i_centi =
            height_log_i16(alt_ctrl.vel_pid.out_i * 100.0f);
        sample->tof_used = ekf_lite_health.tof_used;
        sample->armed = armed;
        height_log_count = index + 1u;

        if(height_log_count >= HEIGHT_LOG_SAMPLE_COUNT)
        {
            height_log_recording = 0u;
            height_log_ready = 1u;
            height_log_dump_index = 0u;
        }
    }
}

void height_log_dump_task(void)
{
    uint8_t lines = 0u;

    if((height_log_ready == 0u) || (vehicle_state.armed != 0u))
    {
        return;
    }

    if(height_log_dump_index == 0u)
    {
        printf("HEIGHT_LOG_BEGIN,count=%u,fs=50\r\n", height_log_count);
        printf("seq,time_us,tof_cm,height_cm,ekf_z_cm,ekf_vz_cm_s,acc_raw_cm_s2,acc_corrected_cm_s2,acc_lpf_cm_s2,acc_ekf_cm_s2,tof_diff_cm_s,innov_cm,throttle,target_height_cm,current_vel_z_cm_s,target_vel_z_cm_s,tof_used,armed,final_height_cm,profile_height_cm,height_loop_output_cm_s,height_loop_i_cm_s,profile_vz_cm_s,vel_loop_output,vel_loop_i,profile_accel_cm_s2\r\n");
    }

    while((height_log_dump_index < height_log_count) && (lines < 4u))
    {
        const height_log_sample_t *sample =
            &height_log_buf[height_log_dump_index];

        printf("%u,%lu,%.1f,%.2f,%.2f,%.2f,%.1f,%.1f,%.1f,%.1f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%u,%u,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.1f\r\n",
               height_log_dump_index,
               (unsigned long)sample->time_us,
               sample->tof_mm * 0.1f,
               sample->height_centi_cm * 0.01f,
               sample->ekf_z_centi_cm * 0.01f,
               sample->ekf_vz_centi_cm_s * 0.01f,
               sample->acc_raw_deci_cm_s2 * 0.1f,
               sample->acc_corrected_deci_cm_s2 * 0.1f,
               sample->acc_lpf_deci_cm_s2 * 0.1f,
               sample->acc_ekf_deci_cm_s2 * 0.1f,
               sample->tof_diff_centi_cm_s * 0.01f,
               sample->innov_centi_cm * 0.01f,
               sample->throttle_centi * 0.01f,
               sample->target_height_centi_cm * 0.01f,
               sample->current_vz_centi_cm_s * 0.01f,
               sample->target_vz_centi_cm_s * 0.01f,
               sample->tof_used,
               sample->armed,
               sample->final_height_centi_cm * 0.01f,
               sample->profile_height_centi_cm * 0.01f,
               sample->position_correction_centi_cm_s * 0.01f,
               sample->height_loop_i_centi_cm_s * 0.01f,
               sample->profile_vz_centi_cm_s * 0.01f,
               sample->vel_loop_output_centi * 0.01f,
               sample->vel_loop_i_centi * 0.01f,
               sample->profile_accel_deci_cm_s2 * 0.1f);
        height_log_dump_index++;
        lines++;
    }

    if(height_log_dump_index >= height_log_count)
    {
        printf("HEIGHT_LOG_END\r\n");
        height_log_ready = 0u;
        height_log_count = 0u;
        height_log_dump_index = 0u;
    }
}

uint8_t height_log_is_dumping(void)
{
    return height_log_ready;
}
