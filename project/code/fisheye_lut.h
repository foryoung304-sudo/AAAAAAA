/* Fisheye Undistortion LUT (188x120) */
#ifndef __FISHEYE_LUT_H__
#define __FISHEYE_LUT_H__

#include <stdint.h>

#define LUT_WIDTH 188
#define LUT_HEIGHT 120
#define LUT_SIZE (LUT_WIDTH * LUT_HEIGHT)

typedef enum
{
    FISHEYE_LUT_170_DEG = 0,
    FISHEYE_LUT_140_DEG = 1,
    FISHEYE_LUT_DEFAULT = FISHEYE_LUT_170_DEG
} fisheye_lut_id_t;

typedef struct
{
    const uint8_t *map_x;
    const uint8_t *map_y;
    uint16_t width;
    uint16_t height;
    const char *name;
} fisheye_lut_t;

extern const uint8_t lut_mapX[LUT_SIZE];
extern const uint8_t lut_mapY[LUT_SIZE];
extern const uint8_t lut_mapX_140[LUT_SIZE];
extern const uint8_t lut_mapY_140[LUT_SIZE];

extern const fisheye_lut_t fisheye_lut_170deg;
extern const fisheye_lut_t fisheye_lut_140deg;

const fisheye_lut_t *fisheye_lut_get(fisheye_lut_id_t id);

#endif
