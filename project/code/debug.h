#ifndef __DEBUG_H__
#define __DEBUG_H__

#include "zf_common_headfile.h"

void ips200_draw_filled_circle(uint16 x, uint16 y, uint16 radius, const uint16 color);
void ips200_draw_circle(uint16 x, uint16 y, uint16 radius, const uint16 color);
void ips200_draw_thick_circle(uint16 x, uint16 y, uint16 radius, const uint16 color);
void debug_pit_diag_task(uint32_t *main_height_max_us);


#endif
