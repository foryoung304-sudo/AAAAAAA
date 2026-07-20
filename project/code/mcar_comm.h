#ifndef MCAR_COMM_H_
#define MCAR_COMM_H_

#include "zf_common_headfile.h"

#define MCAR_COMM_UART                 (UART_5)
#define MCAR_COMM_BAUDRATE             (115200u)
#define MCAR_COMM_TX_PIN               (UART5_TX_P02_1)
#define MCAR_COMM_RX_PIN               (UART5_RX_P02_0)

#define MCAR_COMM_FRAME_HEAD           (0xA5u)
#define MCAR_COMM_FRAME_TAIL           (0x5Au)
#define MCAR_COMM_CMD_TARGET           (0x01u)
#define MCAR_COMM_CMD_FEEDBACK         (0x81u)
#define MCAR_COMM_FRAME_SIZE           (12u)
#define MCAR_COMM_FEEDBACK_SIZE        (10u)

#define MCAR_COMM_FLAG_BEACON_VALID    (1u << 0)
#define MCAR_COMM_FLAG_MCAR_VALID      (1u << 1)
#define MCAR_COMM_FLAG_YAW_VALID       (1u << 2)
#define MCAR_COMM_FLAG_OBS_HELD        (1u << 3)
#define MCAR_COMM_FLAG_TARGET_ACTIVE   (1u << 4)
#define MCAR_COMM_FLAG_STOP            (1u << 7)

#define MCAR_COMM_STATUS_TRACKING      (1u << 0)
#define MCAR_COMM_STATUS_ARRIVED       (1u << 1)
#define MCAR_COMM_STATUS_FAILED        (1u << 2)

typedef struct
{
    uint8 valid;
    uint8 target_seq;
    uint8 status;
    int16 err_forward_px;
    int16 err_right_px;
    uint32 timestamp_us;
} mcar_comm_feedback_t;

/*
 * A5 CMD SEQ FLAGS FWD_L FWD_H RIGHT_L RIGHT_H YAW_L YAW_H CRC8 5A
 * Position errors are signed image pixels in the M-car frame. Yaw is the
 * signed visual M-car heading in the local earth frame, in centidegrees.
 *
 * Car feedback:
 * A5 81 TARGET_SEQ STATUS FWD_L FWD_H RIGHT_L RIGHT_H CRC8 5A
 * ARRIVED is accepted only when TARGET_SEQ matches and both returned errors
 * are inside the arrival threshold.
 */
void mcar_comm_init(void);
void mcar_comm_poll(void);
void mcar_comm_send_target(uint8 target_seq,
                           int16 err_forward_px, int16 err_right_px,
                           int16 mcar_yaw_earth_cdeg, uint8 flags);
void mcar_comm_send_stop(uint8 target_seq);
uint8 mcar_comm_feedback_is_fresh(uint8 target_seq, uint32 max_age_us);

extern mcar_comm_feedback_t mcar_comm_feedback;

#endif
