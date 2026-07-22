#ifndef __BEACON_H__
#define __BEACON_H__

#include "vision_shared.h"

#include "zf_common_headfile.h"
typedef enum {
    BEACON_NOT_FOUND = 0,
    BEACON_FOUND,
    BEACON_CENTERED
} BeaconStatus;

// 信标信息结构体
typedef struct {
    BeaconStatus status;
    int centerX;
    int centerY;
    int length;
    int length_last;
    uint16_t area;
    int last_centerX;
    int last_centerY;
    int sum_last;
    int sum;
    int quanzhong;
    int last_quanzhong;
} BeaconInfo;

// 中心点结构体
typedef struct {
    int x;
    int y;
    int distance;
    float quanzhong;
} CenterPoint;


extern BeaconInfo beacon;
#define IR_THRESHOLD 200
#define MAX_CENTERS 6
#define MAX_BLOB 10
#define BEACON_AREA_MIN 4
#define BEACON_AREA_MAX 300
#define BEACON_EMA_ALPHA 0.9f
#define BEACON_LOST_HOLD 3
#define BEACON_LOST_SEARCH 5
#define CAR_IR_THRESHOLD 120
#define CAR_BLOB_MIN_AREA 8
#define CAR_BLOB_MERGE_GAP 6
#define CAR_MIN_PIX 30
#define CAR_MAX_PIX 500
#define YCAR_LOST_HOLD 5
#define CAR_ASPECT_MIN 1.2f
#define CAR_ASPECT_MAX 3.5f
#define CAR_W_RATIO_MIN 1.3f
#define CAR_DIR_EMA_ALPHA 0.7f
#define MAX_IR_BLOBS 32
#define YCAR_DETECTION_ENABLE 1
#define YCAR_GUIDANCE_ENABLE  1

#define BEACON_CAL_DEBUG_ENABLE       1
#define BEACON_CAL_DEBUG_INTERVAL_US  500000u

typedef enum {
    IR_BLOB_UNKNOWN = 0,
    IR_BLOB_BEACON_CANDIDATE,
    IR_BLOB_YCAR_PART
} IrBlobType_t;

typedef struct {
    int16_t  cx;
    int16_t  cy;
    uint16_t area;
    uint16_t brightness;
    uint8_t  valid;
} BeaconBlob_t;

typedef struct {
    int16_t cx;
    int16_t cy;
    float hx;
    float hy;
    uint16_t score;
    uint8_t valid;
} YCarInfo_t;

typedef struct {
    int16_t cx;
    int16_t cy;
    int16_t min_x;
    int16_t max_x;
    int16_t min_y;
    int16_t max_y;
    uint16_t area;
    uint16_t brightness;
    float aspect;
    float fill;
    IrBlobType_t type;
    uint8_t label;
    uint8_t valid;
} IrBlob_t;

extern uint8_t debug_blob_cnt;
extern float debug_beacon_score;
extern float debug_beacon_second_score;
extern int16_t debug_beacon_second_x;
extern int16_t debug_beacon_second_y;
extern uint8_t debug_beacon_lost;
extern float beacon_corr_x;
extern float beacon_corr_y;
extern float beacon_body_x;
extern float beacon_body_y;
extern float beacon_camera_x_cm;
extern float beacon_camera_y_cm;
extern float beacon_drone_x_cm;
extern float beacon_drone_y_cm;
extern YCarInfo_t ycar_info;
extern float ycar_body_x;
extern float ycar_body_y;
extern float ycar_head_body_x;
extern float ycar_head_body_y;
extern float debug_ycar_angle;
extern uint8_t debug_ycar_lost;
extern VisionDetectionSnapshot_t vision_detection_snapshot;
extern uint32_t vision_last_frame_rx_us;
extern uint32_t vision_profile_camera_dt_us;
extern uint32_t vision_profile_process_us;
extern uint32_t vision_profile_remap_us;
extern uint32_t vision_profile_blob_us;
extern uint32_t vision_profile_detect_us;
extern uint32_t vision_profile_display_us;

uint16_t find_centers(const image_t *img, CenterPoint centers[MAX_CENTERS]);
void detect_beacon(image_t *img, BeaconInfo *info);
uint8_t beacon_find_blobs_gray(const image_t *img, BeaconBlob_t blobs[], uint8_t max_blobs);
uint8_t ir_find_blobs_gray(const image_t *img, IrBlob_t blobs[], uint8_t max_blobs);
void classify_ir_blobs(IrBlob_t blobs[], uint8_t blob_cnt);
void detect_beacon_from_ir_blobs(const IrBlob_t blobs[], uint8_t blob_cnt,
                                 float roll_deg, float pitch_deg,
                                 BeaconInfo *info);
void ycar_detect_from_ir_blobs(const image_t *img, const IrBlob_t blobs[], uint8_t blob_cnt, YCarInfo_t *info);
void ycar_detect_gray(const image_t *img, YCarInfo_t *info);
void vision_attitude_compensation(float u_raw, float v_raw, float *u_corr, float *v_corr);
void vision_attitude_compensation_with_attitude(float u_raw, float v_raw,
                                                 float roll_deg, float pitch_deg,
                                                 float *u_corr, float *v_corr);
uint8_t image_process();
uint8_t vision_consumer_update(void);

#endif
