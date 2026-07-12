#ifndef __BEACON_H__
#define __BEACON_H__

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

uint16_t find_centers(const image_t *img, CenterPoint centers[MAX_CENTERS]);
void detect_beacon(image_t *img, BeaconInfo *info);
void vision_attitude_compensation(float u_raw, float v_raw, float *u_corr, float *v_corr);
void image_attitude_compensation(const image_t *img_src, image_t *img_dst);
uint8_t image_process();

#endif