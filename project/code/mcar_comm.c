#include "mcar_comm.h"

static uint8 mcar_comm_sequence;

static uint8 mcar_comm_crc8(const uint8 *data, uint8 length)
{
    uint8 crc = 0x00u;

    for(uint8 i = 0u; i < length; i++)
    {
        crc ^= data[i];
        for(uint8 bit = 0u; bit < 8u; bit++)
        {
            crc = (crc & 0x80u) ? (uint8)((crc << 1u) ^ 0x07u)
                                : (uint8)(crc << 1u);
        }
    }

    return crc;
}

void mcar_comm_init(void)
{
    mcar_comm_sequence = 0u;
    uart_init(MCAR_COMM_UART, MCAR_COMM_BAUDRATE,
              MCAR_COMM_TX_PIN, MCAR_COMM_RX_PIN);
}

void mcar_comm_send_target(int16 err_forward_px, int16 err_right_px,
                           int16 mcar_yaw_earth_cdeg, uint8 flags)
{
    uint8 frame[MCAR_COMM_FRAME_SIZE];

    frame[0] = MCAR_COMM_FRAME_HEAD;
    frame[1] = MCAR_COMM_CMD_TARGET;
    frame[2] = mcar_comm_sequence++;
    frame[3] = flags;
    frame[4] = (uint8)((uint16)err_forward_px & 0x00FFu);
    frame[5] = (uint8)(((uint16)err_forward_px >> 8u) & 0x00FFu);
    frame[6] = (uint8)((uint16)err_right_px & 0x00FFu);
    frame[7] = (uint8)(((uint16)err_right_px >> 8u) & 0x00FFu);
    frame[8] = (uint8)((uint16)mcar_yaw_earth_cdeg & 0x00FFu);
    frame[9] = (uint8)(((uint16)mcar_yaw_earth_cdeg >> 8u) & 0x00FFu);
    frame[10] = mcar_comm_crc8(frame, 10u);
    frame[11] = MCAR_COMM_FRAME_TAIL;

    uart_write_buffer(MCAR_COMM_UART, frame, MCAR_COMM_FRAME_SIZE);
}

void mcar_comm_send_stop(void)
{
    mcar_comm_send_target(0, 0, 0, MCAR_COMM_FLAG_STOP);
}
