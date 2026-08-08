#include "mcar_comm.h"

mcar_comm_feedback_t mcar_comm_feedback = {0};
mcar_comm_diag_t mcar_comm_diag = {0};

static uint8 mcar_comm_rx_frame[MCAR_COMM_FEEDBACK_SIZE];
static uint8 mcar_comm_rx_count;
static volatile uint8 mcar_comm_polling;
static int16 mcar_comm_tx_shaped_forward_px;
static int16 mcar_comm_tx_shaped_right_px;
static uint8 mcar_comm_tx_shaper_axis;
static uint8 mcar_comm_tx_shaper_last_seq;
static uint8 mcar_comm_tx_shaper_seq_valid;

static int32 mcar_comm_abs_i16(int16 value)
{
    return (value < 0) ? -(int32)value : (int32)value;
}

static int16 mcar_comm_slew_i16(int16 current, int16 target, int16 step)
{
    if(current < target)
    {
        int32 next = (int32)current + step;
        return (next > target) ? target : (int16)next;
    }
    if(current > target)
    {
        int32 next = (int32)current - step;
        return (next < target) ? target : (int16)next;
    }
    return current;
}

static int16 mcar_comm_scale_axis_i16(int16 raw_axis, int32 max_abs)
{
    int32 scaled;

    if(max_abs <= 0) return 0;
    scaled = (int32)raw_axis * MCAR_COMM_TX_SHAPER_COMMAND_PX;
    scaled += (scaled >= 0) ? (max_abs / 2) : -(max_abs / 2);
    return (int16)(scaled / max_abs);
}

static void mcar_comm_tx_shaper_reset(void)
{
    mcar_comm_tx_shaped_forward_px = 0;
    mcar_comm_tx_shaped_right_px = 0;
    mcar_comm_tx_shaper_axis = 0u;
    mcar_comm_tx_shaper_seq_valid = 0u;
    mcar_comm_diag.tx_shaper_active = 0u;
    mcar_comm_diag.tx_shaper_axis = 0u;
}

static void mcar_comm_shape_target(uint8 target_seq,
                                   int16 *err_forward_px,
                                   int16 *err_right_px,
                                   uint8 *flags)
{
#if MCAR_COMM_TX_SHAPER_ENABLE
    int16 raw_forward = *err_forward_px;
    int16 raw_right = *err_right_px;
    int16 desired_forward = 0;
    int16 desired_right = 0;
    int32 abs_forward;
    int32 abs_right;

    if(((*flags & MCAR_COMM_FLAG_STOP) != 0u) ||
       ((*flags & MCAR_COMM_FLAG_TARGET_ACTIVE) == 0u))
    {
        mcar_comm_tx_shaper_reset();
        *err_forward_px = 0;
        *err_right_px = 0;
        return;
    }

    if((mcar_comm_tx_shaper_seq_valid == 0u) ||
       (target_seq != mcar_comm_tx_shaper_last_seq))
    {
        mcar_comm_tx_shaped_forward_px = 0;
        mcar_comm_tx_shaped_right_px = 0;
        mcar_comm_tx_shaper_axis = 0u;
        mcar_comm_tx_shaper_last_seq = target_seq;
        mcar_comm_tx_shaper_seq_valid = 1u;
    }

    abs_forward = mcar_comm_abs_i16(raw_forward);
    abs_right = mcar_comm_abs_i16(raw_right);

    /* Preserve the original two-axis direction while limiting its dominant
     * component to the smallest useful command understood by che.  This
     * avoids both 80/80 diagonal saturation and Manhattan-style tether lag. */
    if((abs_forward >= MCAR_COMM_TX_SHAPER_COMMAND_PX) ||
       (abs_right >= MCAR_COMM_TX_SHAPER_COMMAND_PX))
    {
        int32 max_abs = (abs_forward >= abs_right) ?
            abs_forward : abs_right;
        desired_forward = mcar_comm_scale_axis_i16(raw_forward, max_abs);
        desired_right = mcar_comm_scale_axis_i16(raw_right, max_abs);
        mcar_comm_tx_shaper_axis =
            (abs_forward >= abs_right) ? 1u : 2u;
    }
    else
    {
        mcar_comm_tx_shaper_axis = 0u;
    }

    mcar_comm_tx_shaped_forward_px = mcar_comm_slew_i16(
        mcar_comm_tx_shaped_forward_px, desired_forward,
        MCAR_COMM_TX_SHAPER_SLEW_PX_PER_FRAME);
    mcar_comm_tx_shaped_right_px = mcar_comm_slew_i16(
        mcar_comm_tx_shaped_right_px, desired_right,
        MCAR_COMM_TX_SHAPER_SLEW_PX_PER_FRAME);

    *err_forward_px = mcar_comm_tx_shaped_forward_px;
    *err_right_px = mcar_comm_tx_shaped_right_px;
#if MCAR_COMM_TX_SHAPER_FORCE_SLOW_FLAG
    *flags |= MCAR_COMM_FLAG_RECOVERY_SLOW;
#endif
    mcar_comm_diag.tx_shaper_active = 1u;
    mcar_comm_diag.tx_shaper_axis = mcar_comm_tx_shaper_axis;
#else
    (void)target_seq;
    (void)err_forward_px;
    (void)err_right_px;
    (void)flags;
#endif
}

static void mcar_comm_resync_after_bad_frame(void)
{
    uint8 i;

    for(i = 1u; i < MCAR_COMM_FEEDBACK_SIZE; i++)
    {
        if(mcar_comm_rx_frame[i] == MCAR_COMM_FRAME_HEAD)
        {
            uint8 remain = (uint8)(MCAR_COMM_FEEDBACK_SIZE - i);
            uint8 j;
            for(j = 0u; j < remain; j++)
            {
                mcar_comm_rx_frame[j] = mcar_comm_rx_frame[i + j];
            }
            mcar_comm_rx_count = remain;
            return;
        }
    }

    mcar_comm_rx_count = 0u;
}

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
    memset(&mcar_comm_diag, 0, sizeof(mcar_comm_diag));
    mcar_comm_rx_count = 0u;
    mcar_comm_polling = 0u;
    mcar_comm_tx_shaper_reset();
    uart_init(MCAR_COMM_UART, MCAR_COMM_BAUDRATE,
              MCAR_COMM_TX_PIN, MCAR_COMM_RX_PIN);
    /* Receive only through the 2 ms direct-FIFO poll.  Enabling the generic
     * 16-byte IRQ buffer as a second consumer makes the same SCB FIFO subject
     * to two independent receive paths and has already corrupted adjacent
     * flight-control RAM.  A 10-byte feedback frame fits in the 16-byte FIFO;
     * at the currently configured 115200 baud it takes about 0.87 ms. */
    uart_rx_interrupt(MCAR_COMM_UART, 0u);
}

static void mcar_comm_consume_byte(uint8 data)
{
    mcar_comm_diag.rx_byte_count++;
    if(mcar_comm_rx_count == 0u)
    {
        if(data != MCAR_COMM_FRAME_HEAD) return;
    }

    mcar_comm_rx_frame[mcar_comm_rx_count++] = data;
    if(mcar_comm_rx_count < MCAR_COMM_FEEDBACK_SIZE) return;

    if(mcar_comm_rx_frame[1] == MCAR_COMM_CMD_FEEDBACK &&
       mcar_comm_rx_frame[9] == MCAR_COMM_FRAME_TAIL &&
       mcar_comm_rx_frame[8] == mcar_comm_crc8(mcar_comm_rx_frame, 8u))
    {
        uint32 now_us = system_time_us();

        mcar_comm_feedback.target_seq = mcar_comm_rx_frame[2];
        mcar_comm_feedback.status = mcar_comm_rx_frame[3];
        mcar_comm_feedback.speed_forward_cm_s = (int16)(
            (uint16)mcar_comm_rx_frame[4] |
            ((uint16)mcar_comm_rx_frame[5] << 8u));
        mcar_comm_feedback.speed_right_cm_s = (int16)(
            (uint16)mcar_comm_rx_frame[6] |
            ((uint16)mcar_comm_rx_frame[7] << 8u));
        mcar_comm_feedback.timestamp_us = now_us;
        mcar_comm_feedback.valid = 1u;
        mcar_comm_diag.rx_period_us = (mcar_comm_diag.last_rx_us == 0u) ?
            0u : now_us - mcar_comm_diag.last_rx_us;
        mcar_comm_diag.last_rx_us = now_us;
        mcar_comm_diag.rx_count++;
        mcar_comm_rx_count = 0u;
    }
    else
    {
        mcar_comm_diag.rx_bad_frame_count++;
        mcar_comm_resync_after_bad_frame();
    }
}

void mcar_comm_poll(void)
{
    volatile stc_SCB_t *uart_base = get_scb_module(MCAR_COMM_UART);

    /* The previous UART5/SCB7 interrupt did not reach CM7_0 on preflight.
     * Drain the hardware FIFO directly so reception is independent of that
     * route.  The 2 ms PIT caller keeps a 10-byte reply well below FIFO size. */
    if(mcar_comm_polling != 0u) return;
    mcar_comm_polling = 1u;
    mcar_comm_diag.hw_fifo_poll_count++;

    while(Cy_SCB_UART_GetNumInRxFifo(uart_base) != 0u)
    {
        mcar_comm_diag.hw_fifo_byte_count++;
        mcar_comm_consume_byte((uint8)Cy_SCB_UART_Get(uart_base));
    }

    mcar_comm_polling = 0u;
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
    uint32 now_us = system_time_us();

    mcar_comm_shape_target(target_seq, &err_forward_px,
                           &err_right_px, &flags);

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

    mcar_comm_diag.tx_period_us = (mcar_comm_diag.last_tx_us == 0u) ?
        0u : now_us - mcar_comm_diag.last_tx_us;
    mcar_comm_diag.last_tx_us = now_us;
    mcar_comm_diag.tx_count++;
    mcar_comm_diag.last_tx_err_forward_px = err_forward_px;
    mcar_comm_diag.last_tx_err_right_px = err_right_px;
    mcar_comm_diag.last_tx_yaw_earth_cdeg = mcar_yaw_earth_cdeg;
    mcar_comm_diag.last_tx_flags = flags;
    uart_write_buffer(MCAR_COMM_UART, frame, MCAR_COMM_FRAME_SIZE);
}

void mcar_comm_send_stop(uint8 target_seq)
{
    mcar_comm_send_target(target_seq, 0, 0, 0, MCAR_COMM_FLAG_STOP);
}
