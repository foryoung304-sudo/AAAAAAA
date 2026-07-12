#include "lc302.h"

lc302_data_t lc302_data;

static uint8_t lc302_payload[LC302_FRAME_LEN];
static uint8_t lc302_payload_index = 0;
static uint8_t lc302_parse_state = 0;
static uint8_t lc302_xor_calc = 0;
static uint8_t lc302_xor_recv = 0;

static int16_t lc302_i16_le(uint8_t low, uint8_t high)
{
    return (int16_t)((uint16_t)low | ((uint16_t)high << 8));
}

static uint16_t lc302_u16_le(uint8_t low, uint8_t high)
{
    return (uint16_t)((uint16_t)low | ((uint16_t)high << 8));
}

static void lc302_decode_payload(void)
{
    lc302_data.raw_x = lc302_i16_le(lc302_payload[0], lc302_payload[1]);
    lc302_data.raw_y = lc302_i16_le(lc302_payload[2], lc302_payload[3]);
    lc302_data.integration_timespan = lc302_u16_le(lc302_payload[4], lc302_payload[5]);
    lc302_data.height = lc302_u16_le(lc302_payload[6], lc302_payload[7]);
    lc302_data.quality = lc302_payload[8];
    lc302_data.version = lc302_payload[9];

    lc302_data.valid = (lc302_data.quality > 0u) ? 1u : 0u;
    lc302_data.confidence = lc302_data.quality;
    // Pure optical-flow LC302/UP-FLOW-30X-3C may report a reserved height
    // value such as 999. Keep it only for logging; use external TOF height.
    lc302_data.height_cm = (float)lc302_data.height;

    // LC302-3C demo sends optical-flow integral scaled by 10000.
    // Do not multiply by packet height here; flow.c uses vehicle_state.current_height.
    lc302_data.flow_x = (float)lc302_data.raw_x / 10000.0f;
    lc302_data.flow_y = (float)lc302_data.raw_y / 10000.0f;
    lc302_data.accum_flow_x += lc302_data.flow_x;
    lc302_data.accum_flow_y += lc302_data.flow_y;
    lc302_data.accum_count++;
    lc302_data.accum_integration_us += lc302_data.integration_timespan;
    lc302_data.update = 1u;
    lc302_data.frame_count++;
}

static void lc302_parse_byte(uint8_t ch)
{
    lc302_data.byte_count++;

    switch(lc302_parse_state)
    {
        case 0:
            if(ch == LC302_FRAME_HEAD)
            {
                lc302_parse_state = 1;
            }
            break;

        case 1:
            if(ch == LC302_FRAME_LEN)
            {
                lc302_parse_state = 2;
                lc302_payload_index = 0;
                lc302_xor_calc = 0;
            }
            else
            {
                lc302_parse_state = 0;
            }
            break;

        case 2:
            lc302_payload[lc302_payload_index++] = ch;
            lc302_xor_calc ^= ch;
            if(lc302_payload_index >= LC302_FRAME_LEN)
            {
                lc302_parse_state = 3;
            }
            break;

        case 3:
            lc302_xor_recv = ch;
            lc302_parse_state = 4;
            break;

        case 4:
            if((ch == LC302_FRAME_END) && (lc302_xor_recv == lc302_xor_calc))
            {
                lc302_decode_payload();
            }
            else
            {
                lc302_data.checksum_error_count++;
            }
            lc302_parse_state = 0;
            break;

        default:
            lc302_parse_state = 0;
            break;
    }
}

void lc302_init(void)
{
    memset(&lc302_data, 0, sizeof(lc302_data));
    memset(lc302_payload, 0, sizeof(lc302_payload));
    lc302_payload_index = 0;
    lc302_parse_state = 0;
    lc302_xor_calc = 0;
    lc302_xor_recv = 0;

    uart_init(LC302_UART, LC302_BAUDRATE, LC302_RX_PIN, LC302_TX_PIN);
    uart_rx_interrupt(LC302_UART, 1);
}

void lc302_uart_callback(void)
{
    uint8_t receive_data;

    while(uart_query_byte(LC302_UART, &receive_data))
    {
        lc302_parse_byte(receive_data);
    }
}

void lc302_update(void)
{
    lc302_uart_callback();
}

void lc302_get_motion(float *dx, float *dy, uint8_t *valid, uint8_t *quality,
                      uint16_t *count, uint32_t *integration_us,
                      uint32_t *frame_count)
{
    float snapshot_dx;
    float snapshot_dy;
    uint8_t snapshot_valid;
    uint8_t snapshot_quality;
    uint16_t snapshot_count;
    uint32_t snapshot_integration_us;
    uint32_t snapshot_frame_count;
    uint32_t primask = interrupt_global_disable();

    snapshot_dx = lc302_data.accum_flow_x;
    snapshot_dy = lc302_data.accum_flow_y;
    snapshot_count = lc302_data.accum_count;
    snapshot_integration_us = lc302_data.accum_integration_us;
    snapshot_frame_count = lc302_data.frame_count;
    snapshot_valid = (snapshot_count > 0u) ? lc302_data.valid : 0u;
    snapshot_quality = lc302_data.quality;

    lc302_data.accum_flow_x = 0.0f;
    lc302_data.accum_flow_y = 0.0f;
    lc302_data.accum_count = 0u;
    lc302_data.accum_integration_us = 0u;
    lc302_data.update = 0u;
    interrupt_global_enable(primask);

    if(dx != 0)
    {
        *dx = snapshot_dx;
    }

    if(dy != 0)
    {
        *dy = snapshot_dy;
    }

    if(valid != 0)
    {
        *valid = snapshot_valid;
    }

    if(quality != 0)
    {
        *quality = snapshot_quality;
    }
    if(count != 0) *count = snapshot_count;
    if(integration_us != 0) *integration_us = snapshot_integration_us;
    if(frame_count != 0) *frame_count = snapshot_frame_count;
}

void lc302_debug_print(void)
{
    printf("[LC302_TEST] bytes=%lu frames=%lu err=%lu raw_x=%d raw_y=%d flow_x=%f flow_y=%f accum_x=%f accum_y=%f accum_count=%u valid=%d quality=%d height=%d dt=%d ver=%d update=%d\n",
            lc302_data.byte_count,
            lc302_data.frame_count,
            lc302_data.checksum_error_count,
            lc302_data.raw_x,
            lc302_data.raw_y,
            lc302_data.flow_x,
            lc302_data.flow_y,
            lc302_data.accum_flow_x,
            lc302_data.accum_flow_y,
            lc302_data.accum_count,
            lc302_data.valid,
            lc302_data.quality,
            lc302_data.height,
            lc302_data.integration_timespan,
            lc302_data.version,
            lc302_data.update);
}
