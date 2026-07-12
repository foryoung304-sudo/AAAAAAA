#ifndef __IMU_LOG_H__
#define __IMU_LOG_H__

#include "zf_common_typedef.h"

#define IMU_LOG_SAMPLE_COUNT    (2000)
#define IMU_LOG_GYRO_SCALE      (100.0f)
#define IMU_LOG_ACC_SCALE       (10000.0f)

typedef struct {
    int16 gx;
    int16 gy;
    int16 gz;
    int16 ax;
    int16 ay;
    int16 az;
    uint16 bad;
} imu_log_sample_t;

void imu_log_start(void);
void imu_log_sample_isr(void);
void imu_log_dump_task(void);
uint8 imu_log_is_full(void);

#endif
