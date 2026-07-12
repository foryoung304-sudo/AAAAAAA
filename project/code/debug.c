#include "zf_common_headfile.h"

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     IPS200 画填充圆
// 参数说明     x               圆心x坐标 [0, ips200_width_max-1]
// 参数说明     y               圆心y坐标 [0, ips200_height_max-1]
// 参数说明     radius          圆的半径（像素）
// 参数说明     color           颜色格式 RGB565
// 返回参数     void
// 使用示例     ips200_draw_filled_circle(160, 120, 8, RGB565_RED);
// 备注信息     为了显眼，半径建议 >= 5
//-------------------------------------------------------------------------------------------------------------------
void ips200_draw_filled_circle(uint16 x, uint16 y, uint16 radius, const uint16 color)
{
    if (radius == 0) return;

    // 计算边界范围
    int16 x_start = (int16)x - (int16)radius;
    int16 x_end = (int16)x + (int16)radius;
    int16 y_start = (int16)y - (int16)radius;
    int16 y_end = (int16)y + (int16)radius;
    int16 height_max=MT9V03X_H-radius;
    int16 width_max=MT9V03X_W-radius;
    // 限制在屏幕范围内
    if (x_start < 0) x_start = 0;
    if (x_end >= width_max) x_end = width_max - 1;
    if (y_start < 0) y_start = 0;
    if (y_end >= height_max) y_end = height_max - 1;

    int16 radius_sq = (int16)radius * (int16)radius;

    // 填充圆：遍历边界矩形内的每个点，判断是否在圆内
    for (int16 cy = y_start; cy <= y_end; cy++) {
        for (int16 cx = x_start; cx <= x_end; cx++) {
            int16 dx = cx - (int16)x;
            int16 dy = cy - (int16)y;
            if (dx * dx + dy * dy <= radius_sq) {
                ips200_draw_point((uint16)cx, (uint16)cy, color);
            }
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     IPS200 画圆环（轮廓圆）
// 参数说明     x               圆心x坐标 [0, ips200_width_max-1]
// 参数说明     y               圆心y坐标 [0, ips200_height_max-1]
// 参数说明     radius          圆的半径（像素）
// 参数说明     color           颜色格式 RGB565
// 返回参数     void
// 使用示例     ips200_draw_circle(160, 120, 10, RGB565_RED);
// 备注信息     使用中点圆算法，效率较高
//-------------------------------------------------------------------------------------------------------------------
void ips200_draw_circle(uint16 x, uint16 y, uint16 radius, const uint16 color)
{
    if (radius == 0) return;

    int16 d = 1 - (int16)radius;
    int16 x_offset = 0;
    int16 y_offset = (int16)radius;
    int16 height_max=MT9V03X_H-radius;
    int16 width_max=MT9V03X_W-radius;

    // 八分圆法画圆
    while (x_offset <= y_offset) {
        // 画8个对称点
        if ((int16)x + x_offset >= radius && (int16)x + x_offset < width_max &&
            (int16)y + y_offset >= radius && (int16)y + y_offset < height_max) {
            ips200_draw_point(x + x_offset, y + y_offset, color);
        }
        if ((int16)x + y_offset >= radius && (int16)x + y_offset < width_max &&
            (int16)y + x_offset >= radius && (int16)y + x_offset < height_max) {
            ips200_draw_point(x + y_offset, y + x_offset, color);
        }
        if ((int16)x - x_offset >= radius && (int16)x - x_offset < width_max &&
            (int16)y + y_offset >= radius && (int16)y + y_offset < height_max) {
            ips200_draw_point(x - x_offset, y + y_offset, color);
        }
        if ((int16)x - y_offset >= radius && (int16)x - y_offset < width_max &&
            (int16)y + x_offset >= radius && (int16)y + x_offset < height_max) {
            ips200_draw_point(x - y_offset, y + x_offset, color);
        }
        if ((int16)x + x_offset >= radius && (int16)x + x_offset < width_max &&
            (int16)y - y_offset >= radius && (int16)y - y_offset < height_max) {
            ips200_draw_point(x + x_offset, y - y_offset, color);
        }
        if ((int16)x + y_offset >= radius && (int16)x + y_offset < width_max &&
            (int16)y - x_offset >= radius && (int16)y - x_offset < height_max) {
            ips200_draw_point(x + y_offset, y - x_offset, color);
        }
        if ((int16)x - x_offset >= radius && (int16)x - x_offset < width_max &&
            (int16)y - y_offset >= radius && (int16)y - y_offset < height_max) {
            ips200_draw_point(x - x_offset, y - y_offset, color);
        }
        if ((int16)x - y_offset >= radius && (int16)x - y_offset < width_max &&
            (int16)y - x_offset >= radius && (int16)y - x_offset < height_max) {
            ips200_draw_point(x - y_offset, y - x_offset, color);
        }

        if (d < 0) {
            d += 2 * x_offset + 3;
        } else {
            d += 2 * (x_offset - y_offset) + 5;
            y_offset--;
        }
        x_offset++;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     IPS200 画粗圆（显眼的圆环，画多层）
// 参数说明     x               圆心x坐标 [0, ips200_width_max-1]
// 参数说明     y               圆心y坐标 [0, ips200_height_max-1]
// 参数说明     radius          圆的半径（像素）
// 参数说明     color           颜色格式 RGB565
// 返回参数     void
// 使用示例     ips200_draw_thick_circle(160, 120, 8, RGB565_RED);
// 备注信息     为了更显眼，画3层不同半径的圆
//-------------------------------------------------------------------------------------------------------------------
void ips200_draw_thick_circle(uint16 x, uint16 y, uint16 radius, const uint16 color)
{
    if (radius == 0) return;

    // 画3层圆，让圆更显眼
    if (radius >= 2) {
        ips200_draw_circle(x, y, radius, color);
        ips200_draw_circle(x, y, radius - 1, color);
        if (radius >= 3) {
            ips200_draw_circle(x, y, radius - 2, color);
        }
    } else {
        // 半径太小，直接画填充圆
        ips200_draw_filled_circle(x, y, radius, color);
    }
}

extern float real_dt_2ms;
extern float real_dt_20ms;
extern volatile uint8_t diag_print_request;
extern volatile uint32_t diag_pit_count;
extern volatile uint32_t diag_dt2_max_us;
extern volatile uint32_t diag_dt20_max_us;
extern volatile uint32_t diag_isr_cost_max_us;
extern volatile uint32_t diag_dt2_over_3ms;
extern volatile uint32_t diag_dt2_over_5ms;
extern volatile uint32_t diag_dt2_over_8ms;
extern volatile uint32_t diag_dt20_over_30ms;
extern volatile uint32_t diag_power_max_us;
extern volatile uint32_t diag_height_max_us;
extern volatile uint32_t diag_ekf_h_max_us;
extern volatile uint32_t diag_flow_max_us;
extern volatile uint32_t diag_ekf_xy_max_us;
extern volatile uint32_t diag_param_max_us;
extern volatile uint32_t diag_att2_max_us;
extern volatile uint32_t diag_alt2_max_us;
extern volatile uint32_t diag_alt1_max_us;
extern volatile uint32_t height_update_drop_count;

void debug_pit_diag_task(uint32_t *main_height_max_us)
{
    if(!diag_print_request) return;

    diag_print_request = 0;
    uint32_t pit_count = diag_pit_count;
    uint32_t dt2_max_us = diag_dt2_max_us;
    uint32_t dt20_max_us = diag_dt20_max_us;
    uint32_t isr_cost_max_us = diag_isr_cost_max_us;
    uint32_t dt2_over_3ms = diag_dt2_over_3ms;
    uint32_t dt2_over_5ms = diag_dt2_over_5ms;
    uint32_t dt2_over_8ms = diag_dt2_over_8ms;
    uint32_t dt20_over_30ms = diag_dt20_over_30ms;
    uint32_t power_max_us = diag_power_max_us;
    uint32_t height_max_us = diag_height_max_us;
    uint32_t ekf_h_max_us = diag_ekf_h_max_us;
    uint32_t flow_max_us = diag_flow_max_us;
    uint32_t ekf_xy_max_us = diag_ekf_xy_max_us;
    uint32_t param_max_us = diag_param_max_us;
    uint32_t att2_max_us = diag_att2_max_us;
    uint32_t alt2_max_us = diag_alt2_max_us;
    uint32_t alt1_max_us = diag_alt1_max_us;
    uint32_t main_height_us = main_height_max_us ? *main_height_max_us : 0;
    uint32_t height_drop_count = height_update_drop_count;

    diag_pit_count = 0;
    diag_dt2_max_us = 0;
    diag_dt20_max_us = 0;
    diag_isr_cost_max_us = 0;
    diag_dt2_over_3ms = 0;
    diag_dt2_over_5ms = 0;
    diag_dt2_over_8ms = 0;
    diag_dt20_over_30ms = 0;
    diag_power_max_us = 0;
    diag_height_max_us = 0;
    diag_ekf_h_max_us = 0;
    diag_flow_max_us = 0;
    diag_ekf_xy_max_us = 0;
    diag_param_max_us = 0;
    diag_att2_max_us = 0;
    diag_alt2_max_us = 0;
    diag_alt1_max_us = 0;
    if(main_height_max_us) *main_height_max_us = 0;
    height_update_drop_count = 0;

    printf("pit_diag: cnt=%lu dt2_max=%luus over3=%lu over5=%lu over8=%lu dt20_max=%luus dt20_over30=%lu isr_max=%luus now_dt2=%.6f now_dt20=%.6f\r\n",
           (unsigned long)pit_count,
           (unsigned long)dt2_max_us,
           (unsigned long)dt2_over_3ms,
           (unsigned long)dt2_over_5ms,
           (unsigned long)dt2_over_8ms,
           (unsigned long)dt20_max_us,
           (unsigned long)dt20_over_30ms,
           (unsigned long)isr_cost_max_us,
           real_dt_2ms,
           real_dt_20ms);
    printf("pit_seg: power=%luus height=%luus ekf_h=%luus flow=%luus ekf_xy=%luus param=%luus att2=%luus alt2=%luus alt1=%luus\r\n",
           (unsigned long)power_max_us,
           (unsigned long)height_max_us,
           (unsigned long)ekf_h_max_us,
           (unsigned long)flow_max_us,
           (unsigned long)ekf_xy_max_us,
           (unsigned long)param_max_us,
           (unsigned long)att2_max_us,
           (unsigned long)alt2_max_us,
           (unsigned long)alt1_max_us);
    printf("main_seg: height=%luus height_drop=%lu\r\n",
           (unsigned long)main_height_us,
           (unsigned long)height_drop_count);
}
