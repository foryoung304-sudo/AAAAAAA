#ifndef __HEIGHT_LOG_H__
#define __HEIGHT_LOG_H__

#include "zf_common_headfile.h"

#define HEIGHT_LOG_SAMPLE_COUNT 400u

void height_log_update_50hz(void);
void height_log_dump_task(void);
uint8_t height_log_is_dumping(void);

#endif
