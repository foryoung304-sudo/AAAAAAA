#ifndef VL53L8_IF_H_
#define VL53L8_IF_H_

#include <stdint.h>
#include "vl53lmz_api.h"

#ifndef VL53L8_IIC_SCL_PIN
#define VL53L8_IIC_SCL_PIN                 (P19_1)
#endif

#ifndef VL53L8_IIC_SDA_PIN
#define VL53L8_IIC_SDA_PIN                 (P19_0)
#endif

#ifndef VL53L8_IIC_DELAY
#define VL53L8_IIC_DELAY                   (100u)
#endif

#ifndef VL53L8_IIC_ADDRESS
#define VL53L8_IIC_ADDRESS                 (0x52u)
#endif

#ifndef VL53L8_XS_PIN
#define VL53L8_XS_PIN                      (P21_6)
#endif

#ifndef VL53L8_USE_XS_RESET
#define VL53L8_USE_XS_RESET                (1u)
#endif

#ifndef VL53L8_RESOLUTION
#define VL53L8_RESOLUTION                  (VL53LMZ_RESOLUTION_4X4)
#endif

#ifndef VL53L8_USE_CENTER_8X8
#define VL53L8_USE_CENTER_8X8              (0u)
#endif

#ifndef VL53L8_RANGING_FREQ_HZ
#define VL53L8_RANGING_FREQ_HZ             (50u)
#endif

#define VL53L8_ZONE_COUNT                  (64u)

typedef struct
{
    uint8_t inited;
    uint8_t ranging;
    uint8_t data_ready;
    uint8_t valid;
    uint8_t resolution;
    uint8_t streamcount;
    uint8_t center_zone_count;
    uint8_t center_valid_count;
    uint8_t center_trimmed;
    uint16_t center_raw_avg_mm;
    uint16_t center_distance_mm;
    int16_t distance_mm[VL53L8_ZONE_COUNT];
    uint8_t target_status[VL53L8_ZONE_COUNT];
}vl53l8_data_t;

extern vl53l8_data_t vl53l8_data;

uint8_t vl53l8_init(void);
uint8_t vl53l8_update(void);
uint8_t vl53l8_get_height_mm(uint16_t *distance_mm);
const vl53l8_data_t *vl53l8_get_data(void);

#endif
