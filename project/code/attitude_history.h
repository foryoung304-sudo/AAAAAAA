#ifndef ATTITUDE_HISTORY_H
#define ATTITUDE_HISTORY_H

#include "zf_common_typedef.h"

typedef struct {
    uint32_t timestamp_us;
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
} AttitudeHistorySample_t;

void attitude_history_reset(void);
void attitude_history_record(uint32_t timestamp_us,
                             float roll_deg,
                             float pitch_deg,
                             float yaw_deg);
uint8_t attitude_history_find_nearest(uint32_t timestamp_us,
                                      AttitudeHistorySample_t *sample,
                                      uint32_t *time_error_us);

#endif
