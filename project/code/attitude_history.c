#include "zf_common_headfile.h"
#include "attitude_history.h"

#define ATTITUDE_HISTORY_SIZE 32u

static AttitudeHistorySample_t attitude_history[ATTITUDE_HISTORY_SIZE];
static volatile uint8_t attitude_history_head = 0u;
static volatile uint8_t attitude_history_count = 0u;

void attitude_history_reset(void)
{
    memset(attitude_history, 0, sizeof(attitude_history));
    attitude_history_head = 0u;
    attitude_history_count = 0u;
}

void attitude_history_record(uint32_t timestamp_us,
                             float roll_deg,
                             float pitch_deg,
                             float yaw_deg)
{
    uint8_t next = (uint8_t)((attitude_history_head + 1u) % ATTITUDE_HISTORY_SIZE);

    attitude_history[next].timestamp_us = timestamp_us;
    attitude_history[next].roll_deg = roll_deg;
    attitude_history[next].pitch_deg = pitch_deg;
    attitude_history[next].yaw_deg = yaw_deg;
    __DMB();
    attitude_history_head = next;
    if(attitude_history_count < ATTITUDE_HISTORY_SIZE)
    {
        attitude_history_count++;
    }
}

uint8_t attitude_history_find_nearest(uint32_t timestamp_us,
                                      AttitudeHistorySample_t *sample,
                                      uint32_t *time_error_us)
{
    uint8_t count;
    uint8_t head;
    uint8_t found = 0u;
    uint32_t best_error = 0xFFFFFFFFu;
    AttitudeHistorySample_t best_sample = {0u, 0.0f, 0.0f, 0.0f};

    if(sample == NULL)
    {
        return 0u;
    }

    __DMB();
    count = attitude_history_count;
    head = attitude_history_head;

    for(uint8_t i = 0u; i < count; i++)
    {
        uint8_t index = (uint8_t)((head + ATTITUDE_HISTORY_SIZE - i) %
                                  ATTITUDE_HISTORY_SIZE);
        AttitudeHistorySample_t candidate = attitude_history[index];
        uint32_t error;

        if(candidate.timestamp_us == 0u)
        {
            continue;
        }

        {
            int32_t delta = (int32_t)(timestamp_us - candidate.timestamp_us);
            error = (delta >= 0) ? (uint32_t)delta : (uint32_t)(-delta);
        }
        if(error < best_error)
        {
            best_error = error;
            best_sample = candidate;
            found = 1u;
        }
    }

    if(found)
    {
        *sample = best_sample;
        if(time_error_us != NULL)
        {
            *time_error_us = best_error;
        }
    }
    return found;
}
