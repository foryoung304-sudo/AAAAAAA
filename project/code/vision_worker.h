#ifndef VISION_WORKER_H
#define VISION_WORKER_H

#include "zf_common_typedef.h"
#define IR_BINARY_THRESHOLD 220u

#define WORKER_THRESHOLD          120u
#define WORKER_BEACON_THRESHOLD  200u
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

#define VISION_IPS114_ENABLE             0
#define VISION_IPS114_FRAME_DIVIDER     5u
#define VISION_IPS114_DIR               IPS114_PORTAIT
#define VISION_IPS114_WIDTH             240
#define VISION_IPS114_HEIGHT            135
#define VISION_IPS114_IMAGE_X           ((VISION_IPS114_WIDTH - MT9V03X_W) / 2)
#define VISION_IPS114_IMAGE_Y           ((VISION_IPS114_HEIGHT - MT9V03X_H) / 2)

// Camera streaming is a debug mode. UART2 text logging must be disabled while it is enabled.
#define VISION_ASSISTANT_STREAM_ENABLE          0
#define VISION_ASSISTANT_STREAM_FRAME_DIVIDER  100u


void vision_worker_init(void);
uint8_t vision_worker_process(void);
void ips114_draw_circle(uint16 x, uint16 y, uint16 radius, const uint16 color);
void ips114_draw_filled_circle(uint16 x, uint16 y, uint16 radius, const uint16 color);

#endif
