#ifndef VISION_WORKER_H
#define VISION_WORKER_H

#include "zf_common_typedef.h"
#define IR_BINARY_THRESHOLD_CENTER 220u
#define IR_BINARY_THRESHOLD_EDGE   170u
#define WORKER_EDGE_COMP_START_PERMILLE 600u

#define WORKER_THRESHOLD          120u
#define WORKER_BEACON_THRESHOLD_CENTER 200u
#define WORKER_BEACON_THRESHOLD_EDGE   170u
#define WORKER_MIN_AREA          4u
#define WORKER_MAX_AREA          300u
#define WORKER_MAX_LABELS        128u
#define WORKER_YCAR_MIN_PART_AREA  8u
#define WORKER_YCAR_MERGE_GAP      6
#define WORKER_YCAR_MIN_PIX        30u
#define WORKER_YCAR_MAX_PIX      1000u
#define WORKER_YCAR_VALID_MIN_PIX  40u
#define WORKER_YCAR_ASPECT_MIN    1.0f
#define WORKER_YCAR_ASPECT_MAX    4.0f
#define WORKER_YCAR_FILL_MAX       0.70f
#define WORKER_YCAR_HOLE_TEST_SIZE 48
#define WORKER_YCAR_HOLE_PIX_MAX    8u
#define WORKER_YCAR_SHAPE_EXCLUDE_RADIUS_PX 6
#define WORKER_YCAR_EXCLUDE_RADIUS_PX 4
#define WORKER_YCAR_EXCLUDE_HOLD_FRAMES 8u
#define WORKER_YCAR_BORDER_MARGIN_PX     2

/* A round beacon can be strongly stretched throughout the outer LUT band,
 * before its component literally touches the image boundary.  Keep the
 * strict interior gate, but allow dense outer-band components to elongate. */
#define WORKER_BEACON_ASPECT_MAX         1.7f
#define WORKER_BEACON_BORDER_ASPECT_MAX  6.0f
#define WORKER_BEACON_FILL_MIN           0.35f
#define WORKER_BEACON_BORDER_FILL_MIN    0.45f
#define WORKER_BEACON_BORDER_MARGIN_PX  12
#define WORKER_BEACON_YCAR_SPLIT_TRACK_RADIUS_PX 16
#define WORKER_BEACON_YCAR_SPLIT_MAX_AREA        150u
#define WORKER_BEACON_YCAR_SPLIT_FILL_MIN        0.50f
#define WORKER_BEACON_FUSED_TRACK_RADIUS_PX      18
#define WORKER_BEACON_FUSED_WINDOW_RADIUS_PX      6
#define WORKER_BEACON_FUSED_MIN_PIX              3u

#define VISION_IPS114_ENABLE             1
#define VISION_IPS114_FRAME_DIVIDER     5u
#define VISION_IPS114_DIR               IPS114_PORTAIT
#define VISION_IPS114_WIDTH             240
#define VISION_IPS114_HEIGHT            135
#define VISION_IPS114_IMAGE_X           ((VISION_IPS114_WIDTH - MT9V03X_W) / 2)
#define VISION_IPS114_IMAGE_Y           ((VISION_IPS114_HEIGHT - MT9V03X_H) / 2)

#define EXPOSURE_TIME                   400

// Camera streaming is a debug mode. UART2 text logging must be disabled while it is enabled.
#define VISION_ASSISTANT_STREAM_ENABLE          0
#define VISION_ASSISTANT_STREAM_FRAME_DIVIDER  100u


void vision_worker_init(void);
uint8_t vision_worker_process(void);
void ips114_draw_circle(uint16 x, uint16 y, uint16 radius, const uint16 color);
void ips114_draw_filled_circle(uint16 x, uint16 y, uint16 radius, const uint16 color);

#endif
