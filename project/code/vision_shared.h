#ifndef VISION_SHARED_H
#define VISION_SHARED_H

#include "zf_common_typedef.h"

#define VISION_SHARED_RAM_ADDRESS 0x28001000u
#define VISION_MAX_BEACON_CANDIDATES 4u

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
} VisionAttitudeSample_t;

void vision_shared_writer_init(void);
void vision_shared_publish(const VisionDetectionSnapshot_t *snapshot);
uint8_t vision_shared_read(VisionDetectionSnapshot_t *snapshot);
void vision_attitude_shared_publish(uint32_t timestamp_us,
                                    float roll_deg, float pitch_deg,
                                    float yaw_deg, float height_cm);
uint8_t vision_attitude_shared_read(VisionAttitudeSample_t *sample);

#endif
