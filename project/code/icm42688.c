#include "zf_common_debug.h"
#include "zf_driver_delay.h"
#include "zf_driver_gpio.h"
#include "zf_driver_spi.h"
#include "icm42688.h"

int16 icm42688_gyro_x = 0, icm42688_gyro_y = 0, icm42688_gyro_z = 0;
int16 icm42688_acc_x  = 0, icm42688_acc_y  = 0, icm42688_acc_z  = 0;
float icm42688_transition_factor[2] = {2048.0f, 16.384f};

static void icm42688_write_register(uint8 reg, uint8 data)
{
    ICM42688_CS(0);
    spi_write_8bit_register(ICM42688_SPI, reg | ICM42688_SPI_W, data);
    ICM42688_CS(1);
}

static uint8 icm42688_read_register(uint8 reg)
{
    uint8 data = 0;

    ICM42688_CS(0);
    data = spi_read_8bit_register(ICM42688_SPI, reg | ICM42688_SPI_R);
    ICM42688_CS(1);

    return data;
}

static void icm42688_read_registers(uint8 reg, uint8 *data, uint32 len)
{
    ICM42688_CS(0);
    spi_read_8bit_registers(ICM42688_SPI, reg | ICM42688_SPI_R, data, len);
    ICM42688_CS(1);
}

static uint8 icm42688_self_check(void)
{
    uint8 dat = 0;
    uint8 return_state = 0;
    uint16 timeout_count = 0;

    while(ICM42688_WHO_AM_I_VALUE != dat)
    {
        if(ICM42688_TIMEOUT_COUNT < timeout_count++)
        {
            return_state = 1;
            break;
        }
        dat = icm42688_read_register(ICM42688_WHO_AM_I);
        system_delay_ms(10);
    }

    return return_state;
}

void icm42688_get_acc(void)
{
    uint8 dat[6];

    icm42688_read_registers(ICM42688_ACCEL_DATA_X1, dat, 6);
    icm42688_acc_x = (int16)(((uint16)dat[0] << 8) | dat[1]);
    icm42688_acc_y = (int16)(((uint16)dat[2] << 8) | dat[3]);
    icm42688_acc_z = (int16)(((uint16)dat[4] << 8) | dat[5]);
}

void icm42688_get_gyro(void)
{
    uint8 dat[6];

    icm42688_read_registers(ICM42688_GYRO_DATA_X1, dat, 6);
    icm42688_gyro_x = (int16)(((uint16)dat[0] << 8) | dat[1]);
    icm42688_gyro_y = (int16)(((uint16)dat[2] << 8) | dat[3]);
    icm42688_gyro_z = (int16)(((uint16)dat[4] << 8) | dat[5]);
}

uint8 icm42688_get_chip_id(void)
{
    return icm42688_read_register(ICM42688_WHO_AM_I);
}

void icm42688_set_config(icm42688_acc_sample_config acc_sample, icm42688_odr_config acc_odr,
                         icm42688_gyro_sample_config gyro_sample, icm42688_odr_config gyro_odr)
{
    icm42688_write_register(ICM42688_ACCEL_CONFIG0, (uint8)((acc_sample << 5) | (acc_odr + 1)));
    icm42688_write_register(ICM42688_GYRO_CONFIG0,  (uint8)((gyro_sample << 5) | (gyro_odr + 1)));

    switch(acc_sample)
    {
        case ICM42688_ACC_SAMPLE_SGN_2G:   icm42688_transition_factor[0] = 16384.0f; break;
        case ICM42688_ACC_SAMPLE_SGN_4G:   icm42688_transition_factor[0] = 8192.0f;  break;
        case ICM42688_ACC_SAMPLE_SGN_8G:   icm42688_transition_factor[0] = 4096.0f;  break;
        case ICM42688_ACC_SAMPLE_SGN_16G:  icm42688_transition_factor[0] = 2048.0f;  break;
        default:                           icm42688_transition_factor[0] = 4096.0f;  break;
    }

    switch(gyro_sample)
    {
        case ICM42688_GYRO_SAMPLE_SGN_15_625DPS: icm42688_transition_factor[1] = 2097.152f; break;
        case ICM42688_GYRO_SAMPLE_SGN_31_25DPS:  icm42688_transition_factor[1] = 1048.576f; break;
        case ICM42688_GYRO_SAMPLE_SGN_62_5DPS:   icm42688_transition_factor[1] = 524.288f;  break;
        case ICM42688_GYRO_SAMPLE_SGN_125DPS:    icm42688_transition_factor[1] = 262.144f;  break;
        case ICM42688_GYRO_SAMPLE_SGN_250DPS:    icm42688_transition_factor[1] = 131.072f;  break;
        case ICM42688_GYRO_SAMPLE_SGN_500DPS:    icm42688_transition_factor[1] = 65.536f;   break;
        case ICM42688_GYRO_SAMPLE_SGN_1000DPS:   icm42688_transition_factor[1] = 32.768f;   break;
        case ICM42688_GYRO_SAMPLE_SGN_2000DPS:   icm42688_transition_factor[1] = 16.384f;   break;
        default:                                 icm42688_transition_factor[1] = 16.384f;   break;
    }
}

uint8 icm42688_init(void)
{
    uint8 return_state = 0;

    system_delay_ms(10);

    spi_init(ICM42688_SPI, SPI_MODE0, ICM42688_SPI_SPEED, ICM42688_SPC_PIN, ICM42688_SDI_PIN, ICM42688_SDO_PIN, SPI_CS_NULL);
    gpio_init(ICM42688_CS_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    system_delay_ms(10);

    do
    {
        if(icm42688_self_check())
        {
            zf_log(0, "icm42688 self check error.");
            return_state = 1;
            break;
        }

        icm42688_write_register(ICM42688_PWR_MGMT0, 0x00);
        system_delay_ms(10);

        icm42688_set_config(ICM42688_ACC_SAMPLE_DEFAULT, ICM42688_ACC_ODR_DEFAULT,
                            ICM42688_GYRO_SAMPLE_DEFAULT, ICM42688_GYRO_ODR_DEFAULT);

        icm42688_write_register(ICM42688_PWR_MGMT0, 0x0F);
        system_delay_ms(10);
    }while(0);

    return return_state;
}
