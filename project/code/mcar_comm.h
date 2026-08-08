#ifndef MCAR_COMM_H_
#define MCAR_COMM_H_

#include "zf_common_headfile.h"

#define MCAR_COMM_UART                 (UART_6)
#define MCAR_COMM_BAUDRATE             (115200u)
#define MCAR_COMM_TX_PIN               (UART6_TX_P03_1)
#define MCAR_COMM_RX_PIN               (UART6_RX_P03_0)

/* Bench test only: repeatedly command the car forward from each vision frame. */
#define MCAR_COMM_FIXED_TARGET_TEST_ENABLE (0u)
/* Handheld bench test: with motors disarmed, make the Y car follow the
 * aircraft/camera center as soon as a fresh Y-car observation is available. */
#define MCAR_COMM_HANDHELD_FOLLOW_TEST_ENABLE (0u)
/* Handheld vision/communication test: with motors disarmed and mission gates
 * bypassed, command the Y car from the fresh beacon-to-car visual error. */
#define MCAR_COMM_HANDHELD_BEACON_FOLLOW_TEST_ENABLE (0u)

/* A beacon that touches the Y-car tail becomes the same saturated binary
 * shape.  In the handheld test, mirror the formal mission's bounded cover
 * grace period instead of turning that expected near-target occlusion into
 * an immediate STOP. */
#define MCAR_COMM_HANDHELD_BEACON_OVERLAP_RADIUS_PX  (18)
#define MCAR_COMM_HANDHELD_BEACON_OVERLAP_HOLD_US    (200000u)

#if (MCAR_COMM_FIXED_TARGET_TEST_ENABLE + \
     MCAR_COMM_HANDHELD_FOLLOW_TEST_ENABLE + \
     MCAR_COMM_HANDHELD_BEACON_FOLLOW_TEST_ENABLE) > 1u
#error "Enable only one MCAR communication test mode"
#endif

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
#define MCAR_COMM_FLAG_RECOVERY_SLOW   (1u << 5)
#define MCAR_COMM_FLAG_STOP            (1u << 7)

#define MCAR_COMM_STATUS_TRACKING      (1u << 0)
#define MCAR_COMM_STATUS_ARRIVED       (1u << 1)
#define MCAR_COMM_STATUS_FAILED        (1u << 2)
#define MCAR_COMM_STATUS_STOPPING      (1u << 3)
#define MCAR_COMM_STATUS_STOPPED       (1u << 4)
#define MCAR_COMM_STATUS_COMM_LOST     (1u << 5)
#define MCAR_COMM_STATUS_SPEED_VALID   (1u << 6)

/* Last-day aircraft-side command shaper for the current che firmware.
 * che uses receive_error * 8 and treats both axes below 5 px as arrived, so
 * 5 px is the smallest useful nonzero command without changing car code. */
#define MCAR_COMM_TX_SHAPER_ENABLE             1
#define MCAR_COMM_TX_SHAPER_COMMAND_PX         5
#define MCAR_COMM_TX_SHAPER_SLEW_PX_PER_FRAME  10
#define MCAR_COMM_TX_SHAPER_FORCE_SLOW_FLAG    1

typedef struct
{
    uint8 valid;
    uint8 target_seq;
    uint8 status;
    int16 speed_forward_cm_s;
    int16 speed_right_cm_s;
    uint32 timestamp_us;
} mcar_comm_feedback_t;

typedef struct
{
    uint32 tx_count;
    uint32 rx_count;
    uint32 rx_irq_count;
    uint32 hw_fifo_poll_count;
    uint32 hw_fifo_byte_count;
    uint32 rx_byte_count;
    uint32 rx_bad_frame_count;
    uint32 tx_period_us;
    uint32 rx_period_us;
    uint32 last_tx_us;
    uint32 last_rx_us;
    int16 last_tx_err_forward_px;
    int16 last_tx_err_right_px;
    int16 last_tx_yaw_earth_cdeg;
    uint8 last_tx_flags;
    uint8 tx_shaper_active;
    uint8 tx_shaper_axis;
} mcar_comm_diag_t;

/*
 * A5 CMD SEQ FLAGS FWD_L FWD_H RIGHT_L RIGHT_H YAW_L YAW_H CRC8 5A
 * Position errors are signed image pixels in the M-car frame. Yaw is the
 * signed visual M-car heading in the local earth frame, in centidegrees.
 *
 * Car feedback:
 * A5 81 TARGET_SEQ STATUS FWD_L FWD_H RIGHT_L RIGHT_H CRC8 5A
 * With SPEED_VALID, the two signed fields contain measured car forward/right
 * speeds in cm/s. ARRIVED is accepted only after STOPPED.
 */
void mcar_comm_init(void);
void mcar_comm_poll(void);
void mcar_comm_send_target(uint8 target_seq,
                           int16 err_forward_px, int16 err_right_px,
                           int16 mcar_yaw_earth_cdeg, uint8 flags);
void mcar_comm_send_stop(uint8 target_seq);
uint8 mcar_comm_feedback_is_fresh(uint8 target_seq, uint32 max_age_us);

extern mcar_comm_feedback_t mcar_comm_feedback;
extern mcar_comm_diag_t mcar_comm_diag;

#endif
