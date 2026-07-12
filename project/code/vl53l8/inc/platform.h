#ifndef VL53L8_PLATFORM_H_
#define VL53L8_PLATFORM_H_

#include <stdint.h>
#include <string.h>

typedef struct
{
    uint16_t address;
}VL53LMZ_Platform;

#define VL53LMZ_NB_TARGET_PER_ZONE        1U

#define VL53LMZ_DISABLE_AMBIENT_PER_SPAD
#define VL53LMZ_DISABLE_NB_SPADS_ENABLED
#define VL53LMZ_DISABLE_NB_TARGET_DETECTED
#define VL53LMZ_DISABLE_SIGNAL_PER_SPAD
#define VL53LMZ_DISABLE_RANGE_SIGMA_MM
#define VL53LMZ_DISABLE_REFLECTANCE_PERCENT
#define VL53LMZ_DISABLE_MOTION_INDICATOR

uint8_t RdByte(VL53LMZ_Platform *p_platform, uint16_t RegisterAdress, uint8_t *p_value);
uint8_t WrByte(VL53LMZ_Platform *p_platform, uint16_t RegisterAdress, uint8_t value);
uint8_t RdMulti(VL53LMZ_Platform *p_platform, uint16_t RegisterAdress, uint8_t *p_values, uint32_t size);
uint8_t WrMulti(VL53LMZ_Platform *p_platform, uint16_t RegisterAdress, uint8_t *p_values, uint32_t size);
void SwapBuffer(uint8_t *buffer, uint16_t size);
uint8_t WaitMs(VL53LMZ_Platform *p_platform, uint32_t TimeMs);

#endif
