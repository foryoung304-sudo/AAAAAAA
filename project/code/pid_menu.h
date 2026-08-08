#ifndef __PID_MENU_H__
#define __PID_MENU_H__

#include "vision_shared.h"

/*
 * The old PID menu is no longer part of the flight configuration path.  Its
 * four idle keys are reused by the disarmed field-map calibration service.
 * PID Work Flash page 95 is deliberately not touched by this module.
 */
#define FIELD_MAP_CALIBRATION_ENABLE       0u
#define FIELD_MAP_FLASH_PAGE              94u
#define FIELD_MAP_PID_RESERVED_PAGE       95u
#define FIELD_MAP_MIN_BEACON_POINTS        3u
#define FIELD_MAP_COORD_LIMIT_CM        1200.0f
#define FIELD_MAP_KEY_SCAN_PERIOD_MS       20u

void field_map_init(void);
void field_map_task(void);
uint8_t field_map_get_display(VisionFieldMapDisplay_t *display);
uint8_t field_map_get_search_center_earth(float *x_cm, float *y_cm);
uint8_t field_map_get_beacon_point_earth(uint8_t index,
                                         float *x_cm, float *y_cm);
uint8_t field_map_get_beacon_point_count(void);

/* Compatibility aliases: any stale caller enters the map service, never the
 * retired PID Flash loader/menu. */
void pid_menu_init(void);
void pid_menu_task(void);

#endif
