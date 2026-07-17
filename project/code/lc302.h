#ifndef LC302_H_
#define LC302_H_

#include "zf_common_headfile.h"

#ifndef LC302_UART
#define LC302_UART                         (UART_3)
#endif

#ifndef LC302_BAUDRATE
#define LC302_BAUDRATE                     (19200)
#endif

#ifndef LC302_RX_PIN
#define LC302_RX_PIN                       (UART3_RX_P13_0)
#endif

#ifndef LC302_TX_PIN
#define LC302_TX_PIN                       (UART3_TX_P13_1)
#endif

#define LC302_FRAME_HEAD                   (0xFEu)
#define LC302_FRAME_LEN                    (0x0Au)
#define LC302_FRAME_END                    (0x55u)

typedef struct
{
    int16_t raw_x;
    int16_t raw_y;
    uint16_t integration_timespan;
    uint16_t height;
    uint8_t quality;
    uint8_t valid;
    uint8_t version;
    float flow_x;
    float flow_y;
    float accum_flow_x;
    float accum_flow_y;
    uint16_t accum_count;
    uint32_t accum_integration_us;
    float height_cm;
    uint8_t confidence;
    uint8_t update;
    uint32_t byte_count;
    uint32_t frame_count;
    uint32_t last_frame_start_rx_us;
    uint32_t last_frame_rx_us;
    uint32_t checksum_error_count;
}lc302_data_t;

extern lc302_data_t lc302_data;

void lc302_init(void);
void lc302_uart_callback(void);
void lc302_update(void);
void lc302_get_motion(float *dx, float *dy, uint8_t *valid, uint8_t *quality,
                      uint16_t *count, uint32_t *integration_us,
                      uint32_t *frame_count,
                      uint32_t *last_frame_start_rx_us,
                      uint32_t *last_frame_rx_us);
void lc302_debug_print(void);

#endif
