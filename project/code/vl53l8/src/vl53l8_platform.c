#include "zf_common_headfile.h"
#include "platform.h"

#ifndef VL53L8_IIC_CHUNK_SIZE
#define VL53L8_IIC_CHUNK_SIZE              (128u)
#endif

static soft_iic_info_struct vl53l8_iic;
static uint8_t vl53l8_iic_inited = 0u;
static uint8_t vl53l8_tx_buffer[VL53L8_IIC_CHUNK_SIZE + 2u];

static void vl53l8_platform_prepare(VL53LMZ_Platform *p_platform)
{
    uint8_t addr_7bit = (uint8_t)((p_platform->address >> 1) & 0x7Fu);

    if(vl53l8_iic_inited == 0u)
    {
        soft_iic_init(&vl53l8_iic, addr_7bit, VL53L8_IIC_DELAY, VL53L8_IIC_SCL_PIN, VL53L8_IIC_SDA_PIN);
        vl53l8_iic_inited = 1u;
    }
    else
    {
        vl53l8_iic.addr = addr_7bit;
    }
}

uint8_t RdByte(VL53LMZ_Platform *p_platform, uint16_t RegisterAdress, uint8_t *p_value)
{
    return RdMulti(p_platform, RegisterAdress, p_value, 1u);
}

uint8_t WrByte(VL53LMZ_Platform *p_platform, uint16_t RegisterAdress, uint8_t value)
{
    return WrMulti(p_platform, RegisterAdress, &value, 1u);
}

uint8_t WrMulti(VL53LMZ_Platform *p_platform, uint16_t RegisterAdress, uint8_t *p_values, uint32_t size)
{
    uint32_t offset = 0u;

    if((p_platform == 0) || ((p_values == 0) && (size != 0u)))
    {
        return 1u;
    }

    vl53l8_platform_prepare(p_platform);

    while(offset < size)
    {
        uint32_t chunk = size - offset;
        uint16_t reg = (uint16_t)(RegisterAdress + offset);

        if(chunk > VL53L8_IIC_CHUNK_SIZE)
        {
            chunk = VL53L8_IIC_CHUNK_SIZE;
        }

        vl53l8_tx_buffer[0] = (uint8_t)((reg >> 8) & 0xFFu);
        vl53l8_tx_buffer[1] = (uint8_t)(reg & 0xFFu);
        memcpy(&vl53l8_tx_buffer[2], &p_values[offset], chunk);
        soft_iic_write_8bit_array(&vl53l8_iic, vl53l8_tx_buffer, chunk + 2u);

        offset += chunk;
    }

    return 0u;
}

uint8_t RdMulti(VL53LMZ_Platform *p_platform, uint16_t RegisterAdress, uint8_t *p_values, uint32_t size)
{
    uint32_t offset = 0u;

    if((p_platform == 0) || ((p_values == 0) && (size != 0u)))
    {
        return 1u;
    }

    vl53l8_platform_prepare(p_platform);

    while(offset < size)
    {
        uint32_t chunk = size - offset;
        uint16_t reg = (uint16_t)(RegisterAdress + offset);
        uint8_t reg_buffer[2];

        if(chunk > VL53L8_IIC_CHUNK_SIZE)
        {
            chunk = VL53L8_IIC_CHUNK_SIZE;
        }

        reg_buffer[0] = (uint8_t)((reg >> 8) & 0xFFu);
        reg_buffer[1] = (uint8_t)(reg & 0xFFu);
        soft_iic_transfer_8bit_array(&vl53l8_iic, reg_buffer, 2u, &p_values[offset], chunk);

        offset += chunk;
    }

    return 0u;
}

void SwapBuffer(uint8_t *buffer, uint16_t size)
{
    uint32_t i;

    for(i = 0u; i < size; i += 4u)
    {
        uint8_t tmp0 = buffer[i];
        uint8_t tmp1 = buffer[i + 1u];

        buffer[i] = buffer[i + 3u];
        buffer[i + 1u] = buffer[i + 2u];
        buffer[i + 2u] = tmp1;
        buffer[i + 3u] = tmp0;
    }
}

uint8_t WaitMs(VL53LMZ_Platform *p_platform, uint32_t TimeMs)
{
    (void)p_platform;
    system_delay_ms(TimeMs);
    return 0u;
}
