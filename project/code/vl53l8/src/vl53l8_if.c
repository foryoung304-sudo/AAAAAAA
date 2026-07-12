#include "zf_common_headfile.h"

vl53l8_data_t vl53l8_data;

static VL53LMZ_Configuration vl53l8_config;
static VL53LMZ_ResultsData vl53l8_results;

static void vl53l8_xs_reset(void)
{
    gpio_init(VL53L8_XS_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    system_delay_ms(50);
#if VL53L8_USE_XS_RESET
    gpio_low(VL53L8_XS_PIN);
    system_delay_ms(10);
    gpio_high(VL53L8_XS_PIN);
#endif
    system_delay_ms(100);
}

static uint8_t vl53l8_target_is_valid(uint8_t status)
{
    return ((status == 5u) || (status == 6u) || (status == 9u)) ? 1u : 0u;
}

static void vl53l8_decode_results(void)
{
    uint8_t i;
    uint32_t sum = 0u;
    uint8_t count = 0u;
    uint16_t valid_distance[16];
#if VL53L8_USE_CENTER_8X8
    static const uint8_t center_zone[] = 
    {
        18u, 19u, 20u, 21u,
        26u, 27u, 28u, 29u,
        34u, 35u, 36u, 37u,
        42u, 43u, 44u, 45u
    };
#else
    static const uint8_t center_zone[] = 
    {
        0u, 1u, 2u, 3u,
        4u, 5u, 6u, 7u,
        8u, 9u, 10u, 11u,
        12u, 13u, 14u, 15u
    };
#endif
    const uint8_t center_zone_count = (uint8_t)(sizeof(center_zone) / sizeof(center_zone[0]));

    memset(&vl53l8_data.distance_mm[0], 0, sizeof(vl53l8_data.distance_mm));
    memset(&vl53l8_data.target_status[0], 0, sizeof(vl53l8_data.target_status));

    vl53l8_data.streamcount = vl53l8_config.streamcount;
    vl53l8_data.resolution = VL53L8_RESOLUTION;
    vl53l8_data.data_ready = 1u;
    vl53l8_data.center_zone_count = center_zone_count;
    vl53l8_data.center_valid_count = 0u;
    vl53l8_data.center_trimmed = 0u;
    vl53l8_data.center_raw_avg_mm = 0u;

    for(i = 0u; i < VL53L8_RESOLUTION; i++)
    {
        uint8_t target_index = (uint8_t)(VL53LMZ_NB_TARGET_PER_ZONE * i);
        vl53l8_data.distance_mm[i] = vl53l8_results.distance_mm[target_index];
        vl53l8_data.target_status[i] = vl53l8_results.target_status[target_index];
    }

    for(i = 0u; i < center_zone_count; i++)
    {
        uint8_t zone = center_zone[i];

        if((zone < VL53L8_RESOLUTION) && vl53l8_target_is_valid(vl53l8_data.target_status[zone]))
        {
            int16_t distance = vl53l8_data.distance_mm[zone];

            if(distance > 0)
            {
                valid_distance[count] = (uint16_t)distance;
                count++;
            }
        }
    }

    if(count > 0u)
    {
        uint16_t min_distance = valid_distance[0];
        uint16_t max_distance = valid_distance[0];
        uint8_t min_index = 0u;
        uint8_t max_index = 0u;
        uint8_t raw_count = count;
        uint32_t raw_sum = 0u;

        for(i = 1u; i < count; i++)
        {
            if(valid_distance[i] < min_distance)
            {
                min_distance = valid_distance[i];
                min_index = i;
            }
            if(valid_distance[i] > max_distance)
            {
                max_distance = valid_distance[i];
                max_index = i;
            }
        }

        for(i = 0u; i < count; i++)
        {
            raw_sum += valid_distance[i];
        }
        vl53l8_data.center_valid_count = raw_count;
        vl53l8_data.center_raw_avg_mm = (uint16_t)(raw_sum / raw_count);

        for(i = 0u; i < count; i++)
        {
            if((count >= 6u) && ((i == min_index) || (i == max_index)))
            {
                continue;
            }
            sum += valid_distance[i];
        }

        if(count >= 6u)
        {
            count -= 2u;
            vl53l8_data.center_trimmed = 1u;
        }

        vl53l8_data.center_distance_mm = (uint16_t)(sum / count);
        vl53l8_data.valid = 1u;
    }
    else
    {
        vl53l8_data.valid = 0u;
    }
}

uint8_t vl53l8_init(void)
{
    uint8_t status = 0u;
    uint8_t is_alive = 0u;

    memset(&vl53l8_data, 0, sizeof(vl53l8_data));
    memset(&vl53l8_config, 0, sizeof(vl53l8_config));
    memset(&vl53l8_results, 0, sizeof(vl53l8_results));

    vl53l8_xs_reset();

    vl53l8_config.platform.address = VL53L8_IIC_ADDRESS;

    status = vl53lmz_is_alive(&vl53l8_config, &is_alive);
    if((status != VL53LMZ_STATUS_OK) || (is_alive == 0u))
    {
        return 1u;
    }

    status = vl53lmz_init(&vl53l8_config);
    if(status != VL53LMZ_STATUS_OK)
    {
        return status;
    }

    status = vl53lmz_set_resolution(&vl53l8_config, VL53L8_RESOLUTION);
    if(status != VL53LMZ_STATUS_OK)
    {
        return status;
    }

    status = vl53lmz_set_ranging_frequency_hz(&vl53l8_config, VL53L8_RANGING_FREQ_HZ);
    if(status != VL53LMZ_STATUS_OK)
    {
        return status;
    }

    status = vl53lmz_start_ranging(&vl53l8_config);
    if(status != VL53LMZ_STATUS_OK)
    {
        return status;
    }

    vl53l8_data.inited = 1u;
    vl53l8_data.ranging = 1u;
    vl53l8_data.resolution = VL53L8_RESOLUTION;

    return VL53LMZ_STATUS_OK;
}

uint8_t vl53l8_update(void)
{
    uint8_t status = 0u;
    uint8_t is_ready = 0u;

    if(vl53l8_data.ranging == 0u)
    {
        return 1u;
    }

    vl53l8_data.data_ready = 0u;

    status = vl53lmz_check_data_ready(&vl53l8_config, &is_ready);
    if(status != VL53LMZ_STATUS_OK)
    {
        return status;
    }

    if(is_ready != 0u)
    {
        status = vl53lmz_get_ranging_data(&vl53l8_config, &vl53l8_results);
        if(status == VL53LMZ_STATUS_OK)
        {
            vl53l8_decode_results();
        }
    }

    return status;
}

uint8_t vl53l8_get_height_mm(uint16_t *distance_mm)
{
    if(distance_mm != 0)
    {
        *distance_mm = vl53l8_data.center_distance_mm;
    }

    return vl53l8_data.valid;
}

const vl53l8_data_t *vl53l8_get_data(void)
{
    return &vl53l8_data;
}
