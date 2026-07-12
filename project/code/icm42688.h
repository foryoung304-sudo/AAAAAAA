#ifndef __ICM42688_H__
#define __ICM42688_H__

#include "zf_common_typedef.h"
#include "zf_device_imu660rc.h"

// ICM42688 reuses the IMU660RC hardware SPI pins so wiring can stay unchanged.
#define ICM42688_SPI_SPEED              (10 * 1000 * 1000)
#define ICM42688_SPI                    (IMU660RC_SPI)
#define ICM42688_SPC_PIN                (IMU660RC_SPC_PIN)
#define ICM42688_SDI_PIN                (IMU660RC_SDI_PIN)
#define ICM42688_SDO_PIN                (IMU660RC_SDO_PIN)
#define ICM42688_CS_PIN                 (IMU660RC_CS_PIN)
#define ICM42688_CS(x)                  ((x) ? (gpio_high(ICM42688_CS_PIN)) : (gpio_low(ICM42688_CS_PIN)))

#define ICM42688_SPI_W                  (0x00)
#define ICM42688_SPI_R                  (0x80)
#define ICM42688_WHO_AM_I_VALUE         (0x47)
#define ICM42688_TIMEOUT_COUNT          (0x00FF)

typedef enum
{
    ICM42688_ACC_SAMPLE_SGN_16G = 0,
    ICM42688_ACC_SAMPLE_SGN_8G,
    ICM42688_ACC_SAMPLE_SGN_4G,
    ICM42688_ACC_SAMPLE_SGN_2G,
}icm42688_acc_sample_config;

typedef enum
{
    ICM42688_GYRO_SAMPLE_SGN_2000DPS = 0,
    ICM42688_GYRO_SAMPLE_SGN_1000DPS,
    ICM42688_GYRO_SAMPLE_SGN_500DPS,
    ICM42688_GYRO_SAMPLE_SGN_250DPS,
    ICM42688_GYRO_SAMPLE_SGN_125DPS,
    ICM42688_GYRO_SAMPLE_SGN_62_5DPS,
    ICM42688_GYRO_SAMPLE_SGN_31_25DPS,
    ICM42688_GYRO_SAMPLE_SGN_15_625DPS,
}icm42688_gyro_sample_config;

typedef enum
{
    ICM42688_ODR_32000HZ = 0,
    ICM42688_ODR_16000HZ,
    ICM42688_ODR_8000HZ,
    ICM42688_ODR_4000HZ,
    ICM42688_ODR_2000HZ,
    ICM42688_ODR_1000HZ,
    ICM42688_ODR_200HZ,
    ICM42688_ODR_100HZ,
    ICM42688_ODR_50HZ,
    ICM42688_ODR_25HZ,
    ICM42688_ODR_12_5HZ,
    ICM42688_ODR_6_25HZ,
    ICM42688_ODR_3_125HZ,
    ICM42688_ODR_1_5625HZ,
    ICM42688_ODR_500HZ,
}icm42688_odr_config;

#define ICM42688_ACC_SAMPLE_DEFAULT     (ICM42688_ACC_SAMPLE_SGN_16G)
#define ICM42688_GYRO_SAMPLE_DEFAULT    (ICM42688_GYRO_SAMPLE_SGN_2000DPS)
#define ICM42688_ACC_ODR_DEFAULT        (ICM42688_ODR_1000HZ)
#define ICM42688_GYRO_ODR_DEFAULT       (ICM42688_ODR_1000HZ)

#define ICM42688_DEVICE_CONFIG          (0x11)
#define ICM42688_ACCEL_DATA_X1          (0x1F)
#define ICM42688_GYRO_DATA_X1           (0x25)
#define ICM42688_INTF_CONFIG0           (0x4C)
#define ICM42688_PWR_MGMT0              (0x4E)
#define ICM42688_GYRO_CONFIG0           (0x4F)
#define ICM42688_ACCEL_CONFIG0          (0x50)
#define ICM42688_WHO_AM_I               (0x75)
#define ICM42688_REG_BANK_SEL           (0x76)

extern int16 icm42688_gyro_x, icm42688_gyro_y, icm42688_gyro_z;
extern int16 icm42688_acc_x,  icm42688_acc_y,  icm42688_acc_z;
extern float icm42688_transition_factor[2];

void    icm42688_get_acc            (void);
void    icm42688_get_gyro           (void);
uint8   icm42688_get_chip_id        (void);
uint8   icm42688_init               (void);
void    icm42688_set_config         (icm42688_acc_sample_config acc_sample, icm42688_odr_config acc_odr,
                                     icm42688_gyro_sample_config gyro_sample, icm42688_odr_config gyro_odr);

#define icm42688_acc_transition(acc_value)      ((float)(acc_value) / icm42688_transition_factor[0])
#define icm42688_gyro_transition(gyro_value)    ((float)(gyro_value) / icm42688_transition_factor[1])

#endif
