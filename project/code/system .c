#include "zf_common_headfile.h"


static uint8_t system_power_inited = 0u;
static uint8_t system_power_last_armed = 0u;
static float system_power_armed_hover_base = HOVER_THR_MAP_VALUE_MID_HIGH;

static float clampf_local(float value, float min_value, float max_value)
{
    if(value < min_value)
    {
        return min_value;
    }
    if(value > max_value)
    {
        return max_value;
    }
    return value;
}

static float lerpf_local(float x0, float y0, float x1, float y1, float x)
{
    float ratio;

    if((x1 - x0) == 0.0f)
    {
        return y0;
    }

    ratio = (x - x0) / (x1 - x0);
    return y0 + ratio * (y1 - y0);
}

static float battery_adc_to_voltage(uint16_t adc_raw)
{
    float pin_voltage = ((float)adc_raw / BATTERY_ADC_FULL_SCALE) * BATTERY_ADC_REF_VOLTAGE;
    return pin_voltage * BATTERY_DIVIDER_SCALE;
}

static float map_hover_throttle_from_voltage(float voltage)
{
    if(voltage >= HOVER_THR_MAP_VOLTAGE_HIGH)
    {
        return HOVER_THR_MAP_VALUE_HIGH;
    }
    if(voltage >= HOVER_THR_MAP_VOLTAGE_MID_HIGH)
    {
        return lerpf_local(
            HOVER_THR_MAP_VOLTAGE_MID_HIGH, HOVER_THR_MAP_VALUE_MID_HIGH,
            HOVER_THR_MAP_VOLTAGE_HIGH, HOVER_THR_MAP_VALUE_HIGH,
            voltage);
    }
    if(voltage >= HOVER_THR_MAP_VOLTAGE_MID_LOW)
    {
        return lerpf_local(
            HOVER_THR_MAP_VOLTAGE_MID_LOW, HOVER_THR_MAP_VALUE_MID_LOW,
            HOVER_THR_MAP_VOLTAGE_MID_HIGH, HOVER_THR_MAP_VALUE_MID_HIGH,
            voltage);
    }
    if(voltage >= HOVER_THR_MAP_VOLTAGE_LOW)
    {
        return lerpf_local(
            HOVER_THR_MAP_VOLTAGE_LOW, HOVER_THR_MAP_VALUE_LOW,
            HOVER_THR_MAP_VOLTAGE_MID_LOW, HOVER_THR_MAP_VALUE_MID_LOW,
            voltage);
    }

    return HOVER_THR_MAP_VALUE_LOW;
}

void system_power_init(void)
{
    adc_init(BATTERY_ADC_CHANNEL, BATTERY_ADC_RESOLUTION);

    vehicle_state.battery_voltage = 0.0f;
    vehicle_state.battery_voltage_filtered = 0.0f;
    vehicle_state.hover_throttle_base = HOVER_THR_MAP_VALUE_MID_HIGH;
    system_power_armed_hover_base = vehicle_state.hover_throttle_base;
    system_power_last_armed = vehicle_state.armed;
    system_power_inited = 1u;
}

void system_power_update(float dT_s)
{
    uint16_t adc_raw;
    float voltage_raw;

    (void)dT_s;

    if(system_power_inited == 0u)
    {
        system_power_init();
    }

    adc_raw = adc_mean_filter_convert(BATTERY_ADC_CHANNEL, BATTERY_ADC_AVG_COUNT);
    voltage_raw = battery_adc_to_voltage(adc_raw);

    vehicle_state.battery_voltage = voltage_raw;
    if(vehicle_state.battery_voltage_filtered <= 0.01f)
    {
        vehicle_state.battery_voltage_filtered = voltage_raw;
    }
    else
    {
        vehicle_state.battery_voltage_filtered +=
            BATTERY_VOLTAGE_FILTER_ALPHA * (voltage_raw - vehicle_state.battery_voltage_filtered);
    }

    vehicle_state.battery_voltage_filtered =
        clampf_local(vehicle_state.battery_voltage_filtered, 9.0f, 13.0f);

    if(vehicle_state.armed == 0u)
    {
        vehicle_state.hover_throttle_base =
            map_hover_throttle_from_voltage(vehicle_state.battery_voltage_filtered);
        system_power_armed_hover_base = vehicle_state.hover_throttle_base;
    }
    else if(system_power_last_armed == 0u)
    {
        vehicle_state.hover_throttle_base =
            map_hover_throttle_from_voltage(vehicle_state.battery_voltage_filtered);
        system_power_armed_hover_base = vehicle_state.hover_throttle_base;
    }
    else
    {
        vehicle_state.hover_throttle_base = system_power_armed_hover_base;
    }

    system_power_last_armed = vehicle_state.armed;
}

float system_get_battery_voltage(void)
{
    return vehicle_state.battery_voltage;
}

float system_get_battery_voltage_filtered(void)
{
    return vehicle_state.battery_voltage_filtered;
}

float system_get_hover_throttle_base(void)
{
    return vehicle_state.hover_throttle_base;
}

uint8_t system_is_hover_throttle_locked(void)
{
    return (vehicle_state.armed != 0u) ? 1u : 0u;
}
