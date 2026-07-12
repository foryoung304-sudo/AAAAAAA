#include "zf_common_headfile.h"

typedef struct
{
    uint32_t time_us;
    int16_t raw_dx_centi_cm;
    int16_t raw_dy_centi_cm;
    int16_t true_dx_centi_cm;
    int16_t true_dy_centi_cm;
    int16_t earth_dx_centi_cm;
    int16_t earth_dy_centi_cm;
    int16_t flow_x_centi_cm;
    int16_t flow_y_centi_cm;
    int16_t flow_vx_deci_cm_s;
    int16_t flow_vy_deci_cm_s;
    int16_t obs_vx_deci_cm_s;
    int16_t obs_vy_deci_cm_s;
    int16_t ekf_x_centi_cm;
    int16_t ekf_y_centi_cm;
    int16_t ekf_vx_deci_cm_s;
    int16_t ekf_vy_deci_cm_s;
    int16_t innov_vx_centi_cm_s;
    int16_t innov_vy_centi_cm_s;
    int16_t roll_centi_deg;
    int16_t pitch_centi_deg;
    int16_t yaw_centi_deg;
    int16_t height_centi_cm;
    uint8_t raw_valid;
    uint8_t obs_valid;
    uint8_t ekf_used;
    uint8_t gate_clipped;
    uint8_t quality;
    uint8_t armed;
    uint16_t lc302_accum_count;
    uint32_t lc302_frame_count;
} loc_log_sample_t;

static loc_log_sample_t loc_log_buf[LOC_LOG_SAMPLE_COUNT];
static volatile uint16_t loc_log_count = 0u;
static volatile uint8_t loc_log_recording = 0u;
static volatile uint8_t loc_log_ready = 0u;
static uint16_t loc_log_dump_index = 0u;

static int16_t loc_log_i16(float value)
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

void loc_log_update_50hz(void)
{
    uint8_t armed = vehicle_state.armed;
    uint8_t start_ready =
        ((armed != 0u) &&
         (vehicle_state.current_height >= LOC_LOG_START_HEIGHT_CM)) ? 1u : 0u;

    if((start_ready != 0u) && (loc_log_recording == 0u) &&
       (loc_log_ready == 0u))
    {
        loc_log_count = 0u;
        loc_log_recording = 1u;
    }

    if((armed == 0u) && (loc_log_recording != 0u))
    {
        loc_log_recording = 0u;
        loc_log_ready = (loc_log_count > 0u) ? 1u : 0u;
        loc_log_dump_index = 0u;
        return;
    }

    if((loc_log_recording != 0u) &&
       (loc_log_count < LOC_LOG_SAMPLE_COUNT))
    {
        uint16_t index = loc_log_count;
        loc_log_sample_t *sample = &loc_log_buf[index];

        sample->time_us = system_time_us();
        sample->raw_dx_centi_cm = loc_log_i16(flow_health.raw_dx_cm * 100.0f);
        sample->raw_dy_centi_cm = loc_log_i16(flow_health.raw_dy_cm * 100.0f);
        sample->true_dx_centi_cm = loc_log_i16(flow_health.true_dx_body_cm * 100.0f);
        sample->true_dy_centi_cm = loc_log_i16(flow_health.true_dy_body_cm * 100.0f);
        sample->earth_dx_centi_cm = loc_log_i16(flow_health.earth_dx_cm * 100.0f);
        sample->earth_dy_centi_cm = loc_log_i16(flow_health.earth_dy_cm * 100.0f);
        sample->flow_x_centi_cm = loc_log_i16(flow_health.flow_pos_x_cm * 100.0f);
        sample->flow_y_centi_cm = loc_log_i16(flow_health.flow_pos_y_cm * 100.0f);
        sample->flow_vx_deci_cm_s = loc_log_i16(flow_health.flow_vel_x_cm_s * 10.0f);
        sample->flow_vy_deci_cm_s = loc_log_i16(flow_health.flow_vel_y_cm_s * 10.0f);
        sample->obs_vx_deci_cm_s = loc_log_i16(flow_health.obs_vx_cm_s * 10.0f);
        sample->obs_vy_deci_cm_s = loc_log_i16(flow_health.obs_vy_cm_s * 10.0f);
        sample->ekf_x_centi_cm = loc_log_i16(ekf_lite_state.x * 100.0f);
        sample->ekf_y_centi_cm = loc_log_i16(ekf_lite_state.y * 100.0f);
        sample->ekf_vx_deci_cm_s = loc_log_i16(ekf_lite_state.vx * 10.0f);
        sample->ekf_vy_deci_cm_s = loc_log_i16(ekf_lite_state.vy * 10.0f);
        sample->innov_vx_centi_cm_s = loc_log_i16(flow_health.innov_vx_cm_s * 100.0f);
        sample->innov_vy_centi_cm_s = loc_log_i16(flow_health.innov_vy_cm_s * 100.0f);
        sample->roll_centi_deg = loc_log_i16(imu_data.roll * 100.0f);
        sample->pitch_centi_deg = loc_log_i16(imu_data.pitch * 100.0f);
        sample->yaw_centi_deg = loc_log_i16(imu_data.yaw * 100.0f);
        sample->height_centi_cm = loc_log_i16(vehicle_state.current_height * 100.0f);
        sample->raw_valid = flow_health.raw_valid;
        sample->obs_valid = flow_health.obs_valid;
        sample->ekf_used = flow_health.ekf_used;
        sample->gate_clipped = flow_health.gate_clipped;
        sample->quality = flow_health.quality;
        sample->armed = armed;
        sample->lc302_accum_count = flow_health.lc302_accum_count;
        sample->lc302_frame_count = flow_health.lc302_frame_count;
        loc_log_count = index + 1u;

        if(loc_log_count >= LOC_LOG_SAMPLE_COUNT)
        {
            loc_log_recording = 0u;
            loc_log_ready = 1u;
            loc_log_dump_index = 0u;
        }
    }
}

void loc_log_dump_task(void)
{
    uint8_t lines = 0u;

    if((loc_log_ready == 0u) || (vehicle_state.armed != 0u))
    {
        return;
    }

    if(loc_log_dump_index == 0u)
    {
        printf("LOC_LOG_BEGIN,count=%u,fs=50\r\n", loc_log_count);
        printf("seq,time_us,raw_dx_cm,raw_dy_cm,true_dx_body_cm,true_dy_body_cm,earth_dx_cm,earth_dy_cm,flow_x_cm,flow_y_cm,flow_vx_cm_s,flow_vy_cm_s,obs_vx_cm_s,obs_vy_cm_s,ekf_x_cm,ekf_y_cm,ekf_vx_cm_s,ekf_vy_cm_s,innov_vx_cm_s,innov_vy_cm_s,roll_deg,pitch_deg,yaw_deg,height_cm,raw_valid,obs_valid,ekf_used,gate_clipped,quality,armed,lc302_accum_count,lc302_frame_count\r\n");
    }

    while((loc_log_dump_index < loc_log_count) && (lines < 4u))
    {
        const loc_log_sample_t *sample = &loc_log_buf[loc_log_dump_index];

        printf("%u,%lu,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%u,%u,%u,%u,%u,%u,%u,%lu\r\n",
               loc_log_dump_index,
               (unsigned long)sample->time_us,
               sample->raw_dx_centi_cm * 0.01f,
               sample->raw_dy_centi_cm * 0.01f,
               sample->true_dx_centi_cm * 0.01f,
               sample->true_dy_centi_cm * 0.01f,
               sample->earth_dx_centi_cm * 0.01f,
               sample->earth_dy_centi_cm * 0.01f,
               sample->flow_x_centi_cm * 0.01f,
               sample->flow_y_centi_cm * 0.01f,
               sample->flow_vx_deci_cm_s * 0.1f,
               sample->flow_vy_deci_cm_s * 0.1f,
               sample->obs_vx_deci_cm_s * 0.1f,
               sample->obs_vy_deci_cm_s * 0.1f,
               sample->ekf_x_centi_cm * 0.01f,
               sample->ekf_y_centi_cm * 0.01f,
               sample->ekf_vx_deci_cm_s * 0.1f,
               sample->ekf_vy_deci_cm_s * 0.1f,
               sample->innov_vx_centi_cm_s * 0.01f,
               sample->innov_vy_centi_cm_s * 0.01f,
               sample->roll_centi_deg * 0.01f,
               sample->pitch_centi_deg * 0.01f,
               sample->yaw_centi_deg * 0.01f,
               sample->height_centi_cm * 0.01f,
               sample->raw_valid,
               sample->obs_valid,
               sample->ekf_used,
               sample->gate_clipped,
               sample->quality,
               sample->armed,
               sample->lc302_accum_count,
               (unsigned long)sample->lc302_frame_count);
        loc_log_dump_index++;
        lines++;
    }

    if(loc_log_dump_index >= loc_log_count)
    {
        printf("LOC_LOG_END\r\n");
        loc_log_ready = 0u;
        loc_log_count = 0u;
        loc_log_dump_index = 0u;
    }
}

uint8_t loc_log_is_dumping(void)
{
    return loc_log_ready;
}
