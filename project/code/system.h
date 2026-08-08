#ifndef __SYSTEM_HELPERS_H__
#define __SYSTEM_HELPERS_H__

#include "zf_common_typedef.h"

#define BATTERY_ADC_CHANNEL              ADC0_CH21_P07_5
#define BATTERY_ADC_RESOLUTION           ADC_12BIT
#define BATTERY_ADC_AVG_COUNT            (8u)
#define BATTERY_ADC_FULL_SCALE           (4095.0f)
#define BATTERY_ADC_REF_VOLTAGE          (3.3f)
#define BATTERY_DIVIDER_SCALE            (11.0f)
#define BATTERY_VOLTAGE_FILTER_ALPHA     (0.08f)

#define HOVER_THR_MAP_VOLTAGE_HIGH       (12.4f)
#define HOVER_THR_MAP_VOLTAGE_MID_HIGH   (12.1f)
#define HOVER_THR_MAP_VOLTAGE_MID_LOW    (11.7f)
#define HOVER_THR_MAP_VOLTAGE_LOW        (11.4f)

#define HOVER_THR_MAP_VALUE_HIGH         (31.5f)
#define HOVER_THR_MAP_VALUE_MID_HIGH     (32.5f)
#define HOVER_THR_MAP_VALUE_MID_LOW      (33.5f)
#define HOVER_THR_MAP_VALUE_LOW          (34.7f)

void system_power_init(void);
void system_power_update(float dT_s);

float system_get_battery_voltage(void);
float system_get_battery_voltage_filtered(void);
float system_get_hover_throttle_base(void);
uint8_t system_is_hover_throttle_locked(void);

#endif
