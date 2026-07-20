#include "mcar_comm.h"

mcar_comm_feedback_t mcar_comm_feedback = {0};

static uint8 mcar_comm_rx_frame[MCAR_COMM_FEEDBACK_SIZE];
static uint8 mcar_comm_rx_count;

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
    memset(&mcar_comm_feedback, 0, sizeof(mcar_comm_feedback));
    mcar_comm_rx_count = 0u;
    uart_init(MCAR_COMM_UART, MCAR_COMM_BAUDRATE,
              MCAR_COMM_TX_PIN, MCAR_COMM_RX_PIN);
}

void mcar_comm_poll(void)
{
    uint8 data;

    while(uart_query_byte(MCAR_COMM_UART, &data))
    {
        if(mcar_comm_rx_count == 0u)
        {
            if(data != MCAR_COMM_FRAME_HEAD) continue;
        }

        mcar_comm_rx_frame[mcar_comm_rx_count++] = data;
        if(mcar_comm_rx_count < MCAR_COMM_FEEDBACK_SIZE) continue;

        if(mcar_comm_rx_frame[1] == MCAR_COMM_CMD_FEEDBACK &&
           mcar_comm_rx_frame[9] == MCAR_COMM_FRAME_TAIL &&
           mcar_comm_rx_frame[8] == mcar_comm_crc8(mcar_comm_rx_frame, 8u))
        {
            mcar_comm_feedback.target_seq = mcar_comm_rx_frame[2];
            mcar_comm_feedback.status = mcar_comm_rx_frame[3];
            mcar_comm_feedback.err_forward_px = (int16)(
                (uint16)mcar_comm_rx_frame[4] |
                ((uint16)mcar_comm_rx_frame[5] << 8u));
            mcar_comm_feedback.err_right_px = (int16)(
                (uint16)mcar_comm_rx_frame[6] |
                ((uint16)mcar_comm_rx_frame[7] << 8u));
            mcar_comm_feedback.timestamp_us = system_time_us();
            mcar_comm_feedback.valid = 1u;
        }

        mcar_comm_rx_count = 0u;
    }
}

uint8 mcar_comm_feedback_is_fresh(uint8 target_seq, uint32 max_age_us)
{
    return (mcar_comm_feedback.valid != 0u &&
            mcar_comm_feedback.target_seq == target_seq &&
            system_time_us() - mcar_comm_feedback.timestamp_us <= max_age_us) ? 1u : 0u;
}

void mcar_comm_send_target(uint8 target_seq,
                           int16 err_forward_px, int16 err_right_px,
                           int16 mcar_yaw_earth_cdeg, uint8 flags)
{
    uint8 frame[MCAR_COMM_FRAME_SIZE];

    frame[0] = MCAR_COMM_FRAME_HEAD;
    frame[1] = MCAR_COMM_CMD_TARGET;
    frame[2] = target_seq;
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

void mcar_comm_send_stop(uint8 target_seq)
{
    mcar_comm_send_target(target_seq, 0, 0, 0, MCAR_COMM_FLAG_STOP);
}
