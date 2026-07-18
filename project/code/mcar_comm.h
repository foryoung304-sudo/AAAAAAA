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
#define MCAR_COMM_FRAME_SIZE           (12u)

#define MCAR_COMM_FLAG_BEACON_VALID    (1u << 0)
#define MCAR_COMM_FLAG_MCAR_VALID      (1u << 1)
#define MCAR_COMM_FLAG_YAW_VALID       (1u << 2)
#define MCAR_COMM_FLAG_OBS_HELD        (1u << 3)
#define MCAR_COMM_FLAG_STOP            (1u << 7)

/*
 * A5 CMD SEQ FLAGS FWD_L FWD_H RIGHT_L RIGHT_H YAW_L YAW_H CRC8 5A
 * Position errors are signed image pixels in the M-car frame. Yaw is the
 * signed visual M-car heading in the local earth frame, in centidegrees.
 */
void mcar_comm_init(void);
void mcar_comm_send_target(int16 err_forward_px, int16 err_right_px,
                           int16 mcar_yaw_earth_cdeg, uint8 flags);
void mcar_comm_send_stop(void);

#endif
