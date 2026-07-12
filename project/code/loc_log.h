#ifndef __LOC_LOG_H__
#define __LOC_LOG_H__

#include "zf_common_headfile.h"

#define LOC_LOG_SAMPLE_COUNT 400u
#define LOC_LOG_START_HEIGHT_CM 45.0f

void loc_log_update_50hz(void);
void loc_log_dump_task(void);
uint8_t loc_log_is_dumping(void);

#endif
