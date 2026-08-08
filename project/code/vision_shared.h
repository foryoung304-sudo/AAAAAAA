#ifndef VISION_SHARED_H
#define VISION_SHARED_H

#include "zf_common_typedef.h"

#define VISION_SHARED_RAM_ADDRESS 0x28001000u
#define VISION_MAX_BEACON_CANDIDATES 4u
#define VISION_FIELD_MAP_MAX_POINTS 8u

typedef enum {
    FIELD_MAP_STATUS_EMPTY = 0,
    FIELD_MAP_STATUS_LOADED,
    FIELD_MAP_STATUS_NEED_START,
    FIELD_MAP_STATUS_MAP_BAD,
    FIELD_MAP_STATUS_ARMED_LOCK,
    FIELD_MAP_STATUS_FLOW_BAD,
    FIELD_MAP_STATUS_START_SAVED,
    FIELD_MAP_STATUS_FLASH_FAIL,
    FIELD_MAP_STATUS_FULL,
    FIELD_MAP_STATUS_COORD_BAD,
    FIELD_MAP_STATUS_POINT_SAVED,
    FIELD_MAP_STATUS_POINT_UNDONE,
    FIELD_MAP_STATUS_NEED_POINTS,
    FIELD_MAP_STATUS_READY
} field_map_status_t;

typedef struct {
    uint8_t display_active;
    uint8_t map_valid;
    uint8_t flash_valid;
    uint8_t session_active;
    uint8_t point_count;
    uint8_t status;
    uint8_t reserved[2];
    int16_t current_forward_cm;
    int16_t current_right_cm;
    int16_t center_forward_cm;
    int16_t center_right_cm;
    int16_t point_forward_cm[VISION_FIELD_MAP_MAX_POINTS];
    int16_t point_right_cm[VISION_FIELD_MAP_MAX_POINTS];
} VisionFieldMapDisplay_t;

typedef struct {
    int16_t x;
    int16_t y;
    uint16_t area;
    uint16_t brightness;
    float score;
} VisionBeaconCandidate_t;

typedef struct {
    uint32_t frame_id;
    uint32_t timestamp_us;
    uint32_t camera_dt_us;
    uint32_t process_us;
    uint32_t display_us;
    uint8_t blob_count;
    uint8_t beacon_valid;
    uint8_t ycar_valid;
    uint8_t beacon_candidate_count;
    float beacon_score;
    float beacon_second_score;
    uint16_t ycar_score;
    int16_t beacon_x;
    int16_t beacon_y;
    int16_t beacon_second_x;
    int16_t beacon_second_y;
    int16_t ycar_x;
    int16_t ycar_y;
    float ycar_head_x;
    float ycar_head_y;
    uint32_t attitude_generation;
    uint32_t attitude_timestamp_us;
    float frame_roll_deg;
    float frame_pitch_deg;
    float frame_yaw_deg;
    float frame_height_cm;
    VisionBeaconCandidate_t beacon_candidates[VISION_MAX_BEACON_CANDIDATES];
} VisionDetectionSnapshot_t;

typedef struct {
    uint32_t generation;
    uint32_t timestamp_us;
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    float height_cm;
    int16_t car_tx_err_forward_px;
    int16_t car_tx_err_right_px;
    uint8_t flight_mode;
    uint8_t aircraft_armed;
    uint8_t reserved[2];
    VisionFieldMapDisplay_t field_map;
} VisionAttitudeSample_t;

void vision_shared_writer_init(void);
void vision_shared_publish(const VisionDetectionSnapshot_t *snapshot);
uint8_t vision_shared_read(VisionDetectionSnapshot_t *snapshot);
__vfp void vision_attitude_shared_publish(uint32_t timestamp_us,
                                    float roll_deg, float pitch_deg,
                                    float yaw_deg, float height_cm,
                                    int16_t car_tx_err_forward_px,
                                    int16_t car_tx_err_right_px,
                                    uint8_t flight_mode,
                                    uint8_t aircraft_armed,
                                    const VisionFieldMapDisplay_t *field_map);
uint8_t vision_attitude_shared_read(VisionAttitudeSample_t *sample);

#endif
