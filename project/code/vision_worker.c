#include "zf_common_headfile.h"
#include "vision_worker.h"
#include "vision_shared.h"
#include "fisheye_lut.h"

static uint8_t worker_binary[MT9V03X_H][MT9V03X_W];
static uint8_t worker_base[MT9V03X_H][MT9V03X_W];
static uint8_t worker_undistorted[MT9V03X_H][MT9V03X_W];
static uint8_t worker_stream_image[MT9V03X_H][MT9V03X_W];
static uint8_t worker_labels[MT9V03X_H][MT9V03X_W];
static uint8_t worker_ycar_hole_map[WORKER_YCAR_HOLE_TEST_SIZE]
                                    [WORKER_YCAR_HOLE_TEST_SIZE];
static uint16_t worker_ycar_hole_queue[
    WORKER_YCAR_HOLE_TEST_SIZE * WORKER_YCAR_HOLE_TEST_SIZE];

static image_t worker_undistorted_img =
{
    .data = &worker_undistorted[0][0],
    .width = MT9V03X_W,
    .height = MT9V03X_H,
    .step = MT9V03X_W
};

static image_t worker_binary_img =
{
    .data = &worker_binary[0][0],
    .width = MT9V03X_W,
    .height = MT9V03X_H,
    .step = MT9V03X_W
};
static int16_t worker_prev_x = -1;
static int16_t worker_prev_y = -1;
static int16_t worker_filter_x = -1;
static int16_t worker_filter_y = -1;
static uint32_t worker_frame_id = 0u;
static uint32_t worker_last_camera_timestamp_us = 0u;
static uint32_t worker_last_display_us = 0u;
static float worker_ycar_hx = 0.0f;
static float worker_ycar_hy = 0.0f;
static uint8_t worker_ycar_dir_valid = 0u;
static float worker_ycar_pending_hx = 0.0f;
static float worker_ycar_pending_hy = 0.0f;
static uint8_t worker_ycar_pending_frames = 0u;
static uint8_t worker_ycar_shape_valid = 0u;
static int16_t worker_ycar_shape_x = 0;
static int16_t worker_ycar_shape_y = 0;

static uint32_t vision_worker_time_us(void)
{
    return timer_get(TC_TIME2_CH1);
}
static int16_t worker_last_ycar_x = 0;
static int16_t worker_last_ycar_y = 0;
static uint8_t worker_last_ycar_age = 255u;

static void ips114_draw_point_clipped(int x, int y, uint16 color)
{
    if(x >= 0 && x < VISION_IPS114_WIDTH &&
       y >= 0 && y < VISION_IPS114_HEIGHT)
    {
        ips114_draw_point((uint16)x, (uint16)y, color);
    }
}

void ips114_draw_circle(uint16 x, uint16 y, uint16 radius, const uint16 color)
{
    int cx = x;
    int cy = y;
    int px = radius;
    int py = 0;
    int error = 1 - px;

    while(px >= py)
    {
        ips114_draw_point_clipped(cx + px, cy + py, color);
        ips114_draw_point_clipped(cx + py, cy + px, color);
        ips114_draw_point_clipped(cx - py, cy + px, color);
        ips114_draw_point_clipped(cx - px, cy + py, color);
        ips114_draw_point_clipped(cx - px, cy - py, color);
        ips114_draw_point_clipped(cx - py, cy - px, color);
        ips114_draw_point_clipped(cx + py, cy - px, color);
        ips114_draw_point_clipped(cx + px, cy - py, color);
        py++;
        if(error < 0)
        {
            error += 2 * py + 1;
        }
        else
        {
            px--;
            error += 2 * (py - px) + 1;
        }
    }
}

void ips114_draw_filled_circle(uint16 x, uint16 y, uint16 radius,
                               const uint16 color)
{
    int cx = x;
    int cy = y;
    int r = radius;
    for(int dy = -r; dy <= r; dy++)
    {
        int dx = (int)sqrtf((float)(r * r - dy * dy));
        int x0 = cx - dx;
        int x1 = cx + dx;
        int yy = cy + dy;
        if(yy < 0 || yy >= VISION_IPS114_HEIGHT) continue;
        if(x0 < 0) x0 = 0;
        if(x1 >= VISION_IPS114_WIDTH) x1 = VISION_IPS114_WIDTH - 1;
        ips114_draw_line((uint16)x0, (uint16)yy,
                         (uint16)x1, (uint16)yy, color);
    }
}

static int vision_display_clamp_x(int x)
{
    if(x < 0) return 0;
    if(x >= VISION_IPS114_WIDTH) return VISION_IPS114_WIDTH - 1;
    return x;
}

static int vision_display_clamp_y(int y)
{
    if(y < 0) return 0;
    if(y >= VISION_IPS114_HEIGHT) return VISION_IPS114_HEIGHT - 1;
    return y;
}

static void worker_stream_draw_pixel(int x, int y)
{
    if(x >= 0 && x < MT9V03X_W && y >= 0 && y < MT9V03X_H)
    {
        worker_stream_image[y][x] = 128u;
    }
}

static void worker_stream_draw_circle(int cx, int cy, int radius)
{
    int x = radius;
    int y = 0;
    int error = 1 - radius;

    while(x >= y)
    {
        worker_stream_draw_pixel(cx + x, cy + y);
        worker_stream_draw_pixel(cx + y, cy + x);
        worker_stream_draw_pixel(cx - y, cy + x);
        worker_stream_draw_pixel(cx - x, cy + y);
        worker_stream_draw_pixel(cx - x, cy - y);
        worker_stream_draw_pixel(cx - y, cy - x);
        worker_stream_draw_pixel(cx + y, cy - x);
        worker_stream_draw_pixel(cx + x, cy - y);
        y++;
        if(error < 0) error += 2 * y + 1;
        else
        {
            x--;
            error += 2 * (y - x) + 1;
        }
    }
}

static void worker_stream_draw_line(int x0, int y0, int x1, int y1)
{
    int dx = abs(x1 - x0);
    int sx = (x0 < x1) ? 1 : -1;
    int dy = -abs(y1 - y0);
    int sy = (y0 < y1) ? 1 : -1;
    int error = dx + dy;

    for(;;)
    {
        int error2;
        worker_stream_draw_pixel(x0, y0);
        if(x0 == x1 && y0 == y1) break;
        error2 = 2 * error;
        if(error2 >= dy) { error += dy; x0 += sx; }
        if(error2 <= dx) { error += dx; y0 += sy; }
    }
}

static void worker_stream_prepare(const VisionDetectionSnapshot_t *snapshot)
{
    memcpy(worker_stream_image, worker_binary, MT9V03X_IMAGE_SIZE);

    if(snapshot->beacon_valid != 0u)
    {
        worker_stream_draw_circle(snapshot->beacon_x, snapshot->beacon_y, 5);
    }
    if(snapshot->ycar_valid != 0u)
    {
        int x = snapshot->ycar_x;
        int y = snapshot->ycar_y;
        int hx = x + (int)(snapshot->ycar_head_x * 22.0f);
        int hy = y + (int)(snapshot->ycar_head_y * 22.0f);

        worker_stream_draw_circle(x, y, 6);
        worker_stream_draw_line(x - 5, y, x + 5, y);
        worker_stream_draw_line(x, y - 5, x, y + 5);
        worker_stream_draw_line(x, y, hx, hy);
        worker_stream_draw_circle(hx, hy, 2);
    }
}

static void vision_worker_display(const VisionDetectionSnapshot_t *snapshot)
{
#if VISION_IPS114_ENABLE
    ips114_show_gray_image(VISION_IPS114_IMAGE_X, VISION_IPS114_IMAGE_Y,
                           &worker_binary[0][0],
                           MT9V03X_W, MT9V03X_H,
                           MT9V03X_W, MT9V03X_H, 0u);

    if(snapshot->beacon_valid != 0u)
    {
        int x = VISION_IPS114_IMAGE_X + snapshot->beacon_x;
        int y = VISION_IPS114_IMAGE_Y + snapshot->beacon_y;
        ips114_draw_circle((uint16)vision_display_clamp_x(x),
                           (uint16)vision_display_clamp_y(y),
                           5u, RGB565_RED);
    }

    if(snapshot->ycar_valid != 0u)
    {
        int x = VISION_IPS114_IMAGE_X + snapshot->ycar_x;
        int y = VISION_IPS114_IMAGE_Y + snapshot->ycar_y;
        int hx = vision_display_clamp_x(
            x + (int)(snapshot->ycar_head_x * 22.0f));
        int hy = vision_display_clamp_y(
            y + (int)(snapshot->ycar_head_y * 22.0f));
        int cx = vision_display_clamp_x(x);
        int cy = vision_display_clamp_y(y);

        ips114_draw_circle((uint16)cx, (uint16)cy, 6u, RGB565_GREEN);
        ips114_draw_line((uint16)vision_display_clamp_x(cx - 5), (uint16)cy,
                         (uint16)vision_display_clamp_x(cx + 5), (uint16)cy,
                         RGB565_CYAN);
        ips114_draw_line((uint16)cx, (uint16)vision_display_clamp_y(cy - 5),
                         (uint16)cx, (uint16)vision_display_clamp_y(cy + 5),
                         RGB565_CYAN);
        ips114_draw_line((uint16)cx, (uint16)cy,
                         (uint16)hx, (uint16)hy, RGB565_YELLOW);
        ips114_draw_filled_circle((uint16)hx, (uint16)hy, 2u, RGB565_YELLOW);
    }
#else
    (void)snapshot;
#endif
}

static uint8_t worker_find(uint8_t parent[], uint8_t value)
{
    while(parent[value] != value)
    {
        parent[value] = parent[parent[value]];
        value = parent[value];
    }
    return value;
}

static void worker_union(uint8_t parent[], uint8_t a, uint8_t b)
{
    uint8_t ra = worker_find(parent, a);
    uint8_t rb = worker_find(parent, b);
    if(ra == rb) return;
    if(ra < rb) parent[rb] = ra;
    else parent[ra] = rb;
}

static void worker_remap(void)
{
    for(uint32_t i = 0u; i < MT9V03X_IMAGE_SIZE; i++)
    {
        uint8_t src_x = lut_mapX[i];
        uint8_t src_y = lut_mapY[i];
        ((uint8_t *)worker_undistorted)[i] = worker_base[src_y][src_x];
    }
}

static float worker_base_score(const VisionBeaconCandidate_t *candidate)
{
    float center_distance = (float)(MT9V03X_W / 2 - candidate->x);
    if(center_distance < 0.0f) center_distance = -center_distance;
    return (float)candidate->brightness + (float)candidate->area * 0.5f +
           ((float)MT9V03X_W / 2.0f - center_distance) * 0.3f;
}

static void worker_insert_candidate(VisionDetectionSnapshot_t *snapshot,
                                    VisionBeaconCandidate_t candidate)
{
    uint8_t at = snapshot->beacon_candidate_count;
    if(at >= VISION_MAX_BEACON_CANDIDATES)
    {
        at = VISION_MAX_BEACON_CANDIDATES - 1u;
        if(candidate.score <= snapshot->beacon_candidates[at].score) return;
    }
    else
    {
        snapshot->beacon_candidate_count++;
    }

    while(at > 0u && candidate.score > snapshot->beacon_candidates[at - 1u].score)
    {
        snapshot->beacon_candidates[at] = snapshot->beacon_candidates[at - 1u];
        at--;
    }
    snapshot->beacon_candidates[at] = candidate;
}

static uint8_t worker_components_near(const int16_t min_x[], const int16_t max_x[],
                                      const int16_t min_y[], const int16_t max_y[],
                                      uint8_t a, uint8_t b)
{
    int dx = (min_x[a] > max_x[b]) ? min_x[a] - max_x[b] :
             (min_x[b] > max_x[a]) ? min_x[b] - max_x[a] : 0;
    int dy = (min_y[a] > max_y[b]) ? min_y[a] - max_y[b] :
             (min_y[b] > max_y[a]) ? min_y[b] - max_y[a] : 0;
    return (dx <= WORKER_YCAR_MERGE_GAP &&
            dy <= WORKER_YCAR_MERGE_GAP) ? 1u : 0u;
}

static uint8_t worker_set_ycar_extrema_heading(
    VisionDetectionSnapshot_t *snapshot,
    const uint8_t keep[],
    uint16_t area)
{
    static const int16_t dir_x[16] =
        {1000, 924, 707, 383, 0, -383, -707, -924,
         -1000, -924, -707, -383, 0, 383, 707, 924};
    static const int16_t dir_y[16] =
        {0, 383, 707, 924, 1000, 924, 707, 383,
         0, -383, -707, -924, -1000, -924, -707, -383};
    int32_t extreme_score[16];
    int16_t extreme_x[16] = {0};
    int16_t extreme_y[16] = {0};
    int16_t candidate_x[16];
    int16_t candidate_y[16];
    float candidate_branch[16] = {0.0f};
    uint8_t candidate_count = 0u;
    uint8_t tip_index[3] = {0};
    float best_geometry_score = -1.0f;
    int32_t sum_x = 0;
    int32_t sum_y = 0;
    int32_t count = 0;
    uint8_t geometry_valid = 0u;
    int16_t shape_min_x = MT9V03X_W;
    int16_t shape_max_x = -1;
    int16_t shape_min_y = MT9V03X_H;
    int16_t shape_max_y = -1;

    for(uint8_t i = 0u; i < 16u; i++) extreme_score[i] = (-2147483647L - 1L);
    for(int y = 0; y < MT9V03X_H; y++)
    {
        for(int x = 0; x < MT9V03X_W; x++)
        {
            uint8_t id = worker_labels[y][x];
            if(id == 0u || keep[id] == 0u) continue;
            sum_x += x;
            sum_y += y;
            count++;
            if(x < shape_min_x) shape_min_x = (int16_t)x;
            if(x > shape_max_x) shape_max_x = (int16_t)x;
            if(y < shape_min_y) shape_min_y = (int16_t)y;
            if(y > shape_max_y) shape_max_y = (int16_t)y;
            for(uint8_t i = 0u; i < 16u; i++)
            {
                int32_t score = x * dir_x[i] + y * dir_y[i];
                if(score > extreme_score[i])
                {
                    extreme_score[i] = score;
                    extreme_x[i] = (int16_t)x;
                    extreme_y[i] = (int16_t)y;
                }
            }
        }
    }
    if(count < (int32_t)WORKER_YCAR_MIN_PIX) return 0u;

    {
        int shape_width = shape_max_x - shape_min_x + 3;
        int shape_height = shape_max_y - shape_min_y + 3;
        if(shape_width <= WORKER_YCAR_HOLE_TEST_SIZE &&
           shape_height <= WORKER_YCAR_HOLE_TEST_SIZE)
        {
            uint16_t queue_read = 0u;
            uint16_t queue_write = 0u;
            uint16_t enclosed = 0u;

            memset(worker_ycar_hole_map, 0, sizeof(worker_ycar_hole_map));
            for(int y = shape_min_y; y <= shape_max_y; y++)
            {
                for(int x = shape_min_x; x <= shape_max_x; x++)
                {
                    uint8_t id = worker_labels[y][x];
                    if(id != 0u && keep[id] != 0u)
                    {
                        worker_ycar_hole_map[y - shape_min_y + 1]
                                             [x - shape_min_x + 1] = 1u;
                    }
                }
            }

            worker_ycar_hole_map[0][0] = 2u;
            worker_ycar_hole_queue[queue_write++] = 0u;
            while(queue_read < queue_write)
            {
                uint16_t packed = worker_ycar_hole_queue[queue_read++];
                int local_x = packed % WORKER_YCAR_HOLE_TEST_SIZE;
                int local_y = packed / WORKER_YCAR_HOLE_TEST_SIZE;
                static const int8_t dx[4] = {1, -1, 0, 0};
                static const int8_t dy[4] = {0, 0, 1, -1};
                for(uint8_t direction = 0u; direction < 4u; direction++)
                {
                    int nx = local_x + dx[direction];
                    int ny = local_y + dy[direction];
                    if(nx < 0 || nx >= shape_width ||
                       ny < 0 || ny >= shape_height ||
                       worker_ycar_hole_map[ny][nx] != 0u)
                    {
                        continue;
                    }
                    worker_ycar_hole_map[ny][nx] = 2u;
                    worker_ycar_hole_queue[queue_write++] =
                        (uint16_t)(ny * WORKER_YCAR_HOLE_TEST_SIZE + nx);
                }
            }

            for(int y = 0; y < shape_height; y++)
            {
                for(int x = 0; x < shape_width; x++)
                {
                    if(worker_ycar_hole_map[y][x] == 0u) enclosed++;
                }
            }
            if(enclosed > WORKER_YCAR_HOLE_PIX_MAX) return 0u;
        }
    }

    for(uint8_t i = 0u; i < 16u; i++)
    {
        uint8_t duplicate = 0u;
        for(uint8_t j = 0u; j < candidate_count; j++)
        {
            if(candidate_x[j] == extreme_x[i] && candidate_y[j] == extreme_y[i])
            {
                duplicate = 1u;
                break;
            }
        }
        if(duplicate == 0u)
        {
            candidate_x[candidate_count] = extreme_x[i];
            candidate_y[candidate_count] = extreme_y[i];
            candidate_count++;
        }
    }

    for(uint8_t tip = 0u; tip < candidate_count; tip++)
    {
        int tx = candidate_x[tip];
        int ty = candidate_y[tip];
        int min_local_x = (tx > 14) ? tx - 14 : 0;
        int max_local_x = (tx + 14 < MT9V03X_W) ? tx + 14 : MT9V03X_W - 1;
        int min_local_y = (ty > 14) ? ty - 14 : 0;
        int max_local_y = (ty + 14 < MT9V03X_H) ? ty + 14 : MT9V03X_H - 1;
        float xx = 0.0f;
        float yy = 0.0f;
        float xy = 0.0f;
        uint16_t local_count = 0u;

        for(int y = min_local_y; y <= max_local_y; y++)
        {
            for(int x = min_local_x; x <= max_local_x; x++)
            {
                uint8_t id = worker_labels[y][x];
                int dx;
                int dy;
                int distance2;
                if(id == 0u || keep[id] == 0u) continue;
                dx = x - tx;
                dy = y - ty;
                distance2 = dx * dx + dy * dy;
                if(distance2 < 2 || distance2 > 196) continue;
                xx += (float)(dx * dx);
                yy += (float)(dy * dy);
                xy += (float)(dx * dy);
                local_count++;
            }
        }

        if(local_count >= 3u)
        {
            float trace = xx + yy;
            float disc = sqrtf((xx - yy) * (xx - yy) + 4.0f * xy * xy);
            candidate_branch[tip] =
                (trace - disc) / (trace + disc + 1e-6f);
        }
    }

    for(uint8_t i = 0u; i + 2u < candidate_count; i++)
    {
        for(uint8_t j = i + 1u; j + 1u < candidate_count; j++)
        {
            for(uint8_t k = j + 1u; k < candidate_count; k++)
            {
                int32_t ax = candidate_x[j] - candidate_x[i];
                int32_t ay = candidate_y[j] - candidate_y[i];
                int32_t bx = candidate_x[k] - candidate_x[i];
                int32_t by = candidate_y[k] - candidate_y[i];
                int64_t triangle_area = (int64_t)ax * by - (int64_t)ay * bx;
                uint8_t triangle_tip[3] = {i, j, k};
                uint8_t apex = 0u;
                uint8_t end_a;
                uint8_t end_b;
                float second_branch;
                float apex_margin;
                float arm_a;
                float arm_b;
                float symmetry;
                float geometry_score;
                if(triangle_area < 0) triangle_area = -triangle_area;
                if(triangle_area < 12) continue;

                if(candidate_branch[j] > candidate_branch[triangle_tip[apex]])
                {
                    apex = 1u;
                }
                if(candidate_branch[k] > candidate_branch[triangle_tip[apex]])
                {
                    apex = 2u;
                }
                end_a = (apex == 0u) ? 1u : 0u;
                end_b = (apex == 2u) ? 1u : 2u;
                second_branch =
                    fmaxf(candidate_branch[triangle_tip[end_a]],
                          candidate_branch[triangle_tip[end_b]]);
                apex_margin =
                    candidate_branch[triangle_tip[apex]] - second_branch;
                arm_a = sqrtf(
                    (float)((candidate_x[triangle_tip[end_a]] -
                             candidate_x[triangle_tip[apex]]) *
                            (candidate_x[triangle_tip[end_a]] -
                             candidate_x[triangle_tip[apex]]) +
                            (candidate_y[triangle_tip[end_a]] -
                             candidate_y[triangle_tip[apex]]) *
                            (candidate_y[triangle_tip[end_a]] -
                             candidate_y[triangle_tip[apex]])));
                arm_b = sqrtf(
                    (float)((candidate_x[triangle_tip[end_b]] -
                             candidate_x[triangle_tip[apex]]) *
                            (candidate_x[triangle_tip[end_b]] -
                             candidate_x[triangle_tip[apex]]) +
                            (candidate_y[triangle_tip[end_b]] -
                             candidate_y[triangle_tip[apex]]) *
                            (candidate_y[triangle_tip[end_b]] -
                             candidate_y[triangle_tip[apex]])));
                if(arm_a < 5.0f || arm_b < 5.0f) continue;
                symmetry = fminf(arm_a, arm_b) / fmaxf(arm_a, arm_b);
                geometry_score = (float)triangle_area *
                    (0.5f + 0.5f * symmetry) *
                    (1.0f + apex_margin);

                if(geometry_score > best_geometry_score)
                {
                    best_geometry_score = geometry_score;
                    tip_index[0] = triangle_tip[apex];
                    tip_index[1] = triangle_tip[end_a];
                    tip_index[2] = triangle_tip[end_b];
                }
            }
        }
    }

    if(best_geometry_score > 0.0f)
    {
        uint8_t apex = 0u;
        uint8_t end_a = 1u;
        uint8_t end_b = 2u;
        float hx;
        float hy;
        float norm;
        hx = 2.0f * candidate_x[tip_index[apex]] -
             candidate_x[tip_index[end_a]] - candidate_x[tip_index[end_b]];
        hy = 2.0f * candidate_y[tip_index[apex]] -
             candidate_y[tip_index[end_a]] - candidate_y[tip_index[end_b]];
        norm = sqrtf(hx * hx + hy * hy);
        if(norm > 1e-6f)
        {
            hx /= norm;
            hy /= norm;
            worker_ycar_shape_valid = 1u;
            worker_ycar_shape_x = (int16_t)((sum_x + count / 2) / count);
            worker_ycar_shape_y = (int16_t)((sum_y + count / 2) / count);
            if(worker_ycar_dir_valid == 0u)
            {
                float pending_dot;

                if(worker_ycar_pending_frames == 0u)
                {
                    worker_ycar_pending_hx = hx;
                    worker_ycar_pending_hy = hy;
                    worker_ycar_pending_frames = 1u;
                    return 0u;
                }

                pending_dot = worker_ycar_pending_hx * hx +
                              worker_ycar_pending_hy * hy;
                if(pending_dot < 0.8f)
                {
                    worker_ycar_pending_hx = hx;
                    worker_ycar_pending_hy = hy;
                    worker_ycar_pending_frames = 1u;
                    return 0u;
                }

                worker_ycar_hx = worker_ycar_pending_hx + hx;
                worker_ycar_hy = worker_ycar_pending_hy + hy;
                norm = sqrtf(worker_ycar_hx * worker_ycar_hx +
                             worker_ycar_hy * worker_ycar_hy);
                if(norm <= 1e-6f) return 0u;
                worker_ycar_hx /= norm;
                worker_ycar_hy /= norm;
                worker_ycar_dir_valid = 1u;
                worker_ycar_pending_frames = 0u;
            }
            else
            {
                if(worker_ycar_hx * hx + worker_ycar_hy * hy < 0.5f)
                {
                    return 0u;
                }
                worker_ycar_hx = 0.3f * worker_ycar_hx + 0.7f * hx;
                worker_ycar_hy = 0.3f * worker_ycar_hy + 0.7f * hy;
                norm = sqrtf(worker_ycar_hx * worker_ycar_hx +
                             worker_ycar_hy * worker_ycar_hy);
                if(norm > 1e-6f)
                {
                    worker_ycar_hx /= norm;
                    worker_ycar_hy /= norm;
                }
            }
            geometry_valid = 1u;
        }
    }

    if(geometry_valid == 0u) return 0u;

    snapshot->ycar_valid = 1u;
    snapshot->ycar_score = area;
    snapshot->ycar_x = (int16_t)((sum_x + count / 2) / count);
    snapshot->ycar_y = (int16_t)((sum_y + count / 2) / count);
    snapshot->ycar_head_x = worker_ycar_hx;
    snapshot->ycar_head_y = worker_ycar_hy;
    return 1u;
}

static void worker_detect_ycar(VisionDetectionSnapshot_t *snapshot,
                               uint8_t components, const uint16_t area[],
                               const int16_t min_x[], const int16_t max_x[],
                               const int16_t min_y[], const int16_t max_y[])
{
    uint8_t best_keep[WORKER_MAX_LABELS] = {0};
    uint16_t best_area = 0u;

    for(uint8_t seed = 1u; seed <= components; seed++)
    {
        uint8_t keep[WORKER_MAX_LABELS] = {0};
        uint8_t changed = 1u;
        uint16_t group_area = 0u;
        int group_min_x = MT9V03X_W;
        int group_max_x = 0;
        int group_min_y = MT9V03X_H;
        int group_max_y = 0;

        if(area[seed] < WORKER_YCAR_MIN_PART_AREA) continue;
        keep[seed] = 1u;
        while(changed != 0u)
        {
            changed = 0u;
            for(uint8_t i = 1u; i <= components; i++)
            {
                if(keep[i] != 0u || area[i] < WORKER_YCAR_MIN_PART_AREA) continue;
                for(uint8_t j = 1u; j <= components; j++)
                {
                    if(keep[j] == 0u) continue;
                    if(worker_components_near(min_x, max_x, min_y, max_y, i, j))
                    {
                        keep[i] = 1u;
                        changed = 1u;
                        break;
                    }
                }
            }
        }

        for(uint8_t i = 1u; i <= components; i++)
        {
            if(keep[i] == 0u) continue;
            group_area += area[i];
            if(min_x[i] < group_min_x) group_min_x = min_x[i];
            if(max_x[i] > group_max_x) group_max_x = max_x[i];
            if(min_y[i] < group_min_y) group_min_y = min_y[i];
            if(max_y[i] > group_max_y) group_max_y = max_y[i];
        }

        if(group_area >= WORKER_YCAR_MIN_PIX &&
           group_area <= WORKER_YCAR_MAX_PIX)
        {
            int width = group_max_x - group_min_x + 1;
            int height = group_max_y - group_min_y + 1;
            float aspect;
            float fill;
            if(width <= 0 || height <= 0) continue;
            aspect = (width > height) ? (float)width / height :
                                       (float)height / width;
            fill = (float)group_area / ((float)width * height);
            if(aspect < WORKER_YCAR_ASPECT_MIN ||
               aspect > WORKER_YCAR_ASPECT_MAX) continue;
            if(fill > WORKER_YCAR_FILL_MAX) continue;
            if(group_area > best_area)
            {
                best_area = group_area;
                memcpy(best_keep, keep, sizeof(best_keep));
            }
        }
    }

    if(best_area >= WORKER_YCAR_VALID_MIN_PIX)
    {
        (void)worker_set_ycar_extrema_heading(snapshot, best_keep, best_area);
    }
    return;

#if 0
    if(best_area >= WORKER_YCAR_VALID_MIN_PIX)
    {
        int32_t sum_x = 0;
        int32_t sum_y = 0;
        int32_t count = 0;
        float cxx = 0.0f;
        float cyy = 0.0f;
        float cxy = 0.0f;
        float mx;
        float my;
        float e1x;
        float e1y;
        float e2x;
        float e2y;
        float moment2_e1 = 0.0f;
        float moment2_e2 = 0.0f;
        float moment3_e1 = 0.0f;
        float moment3_e2 = 0.0f;

        for(int y = 0; y < MT9V03X_H; y++)
        {
            for(int x = 0; x < MT9V03X_W; x++)
            {
                uint8_t id = worker_labels[y][x];
                if(id == 0u || best_keep[id] == 0u) continue;
                sum_x += x;
                sum_y += y;
                count++;
            }
        }
        if(count < (int32_t)WORKER_YCAR_MIN_PIX) return;
        mx = (float)sum_x / count;
        my = (float)sum_y / count;

        for(int y = 0; y < MT9V03X_H; y++)
        {
            for(int x = 0; x < MT9V03X_W; x++)
            {
                uint8_t id = worker_labels[y][x];
                float dx;
                float dy;
                if(id == 0u || best_keep[id] == 0u) continue;
                dx = x - mx;
                dy = y - my;
                cxx += dx * dx;
                cyy += dy * dy;
                cxy += dx * dy;
            }
        }
        cxx /= count;
        cyy /= count;
        cxy /= count;
        if(fabsf(cxy) < 0.5f)
        {
            e1x = (cxx >= cyy) ? 1.0f : 0.0f;
            e1y = (cxx >= cyy) ? 0.0f : 1.0f;
        }
        else
        {
            float d = (cxx - cyy) * 0.5f;
            float r = sqrtf(d * d + cxy * cxy);
            float nx = cxy;
            float ny = r - d;
            float norm = sqrtf(nx * nx + ny * ny);
            if(norm < 1e-6f) return;
            e1x = nx / norm;
            e1y = ny / norm;
        }
        e2x = -e1y;
        e2y = e1x;

        for(int y = 0; y < MT9V03X_H; y++)
        {
            for(int x = 0; x < MT9V03X_W; x++)
            {
                uint8_t id = worker_labels[y][x];
                float dx;
                float dy;
                float p1;
                float p2;
                if(id == 0u || best_keep[id] == 0u) continue;
                dx = x - mx;
                dy = y - my;
                p1 = dx * e1x + dy * e1y;
                p2 = dx * e2x + dy * e2y;
                moment2_e1 += p1 * p1;
                moment2_e2 += p2 * p2;
                moment3_e1 += p1 * p1 * p1;
                moment3_e2 += p2 * p2 * p2;
            }
        }

        if(count > 2)
        {
            float rough_hx;
            float rough_hy;
            float skew_e1;
            float skew_e2;
            float apex_max = -1e9f;
            float apex_x = 0.0f;
            float apex_y = 0.0f;
            uint16_t apex_count = 0u;
            float left_x = 0.0f;
            float left_y = 0.0f;
            float right_x = 0.0f;
            float right_y = 0.0f;
            float left_dist2 = -1.0f;
            float right_dist2 = -1.0f;
            float hx = worker_ycar_hx;
            float hy = worker_ycar_hy;
            float norm;

            skew_e1 = fabsf(moment3_e1) /
                      (moment2_e1 * sqrtf(moment2_e1 / count) + 1e-6f);
            skew_e2 = fabsf(moment3_e2) /
                      (moment2_e2 * sqrtf(moment2_e2 / count) + 1e-6f);
            if(skew_e1 >= skew_e2)
            {
                rough_hx = (moment3_e1 >= 0.0f) ? -e1x : e1x;
                rough_hy = (moment3_e1 >= 0.0f) ? -e1y : e1y;
            }
            else
            {
                rough_hx = (moment3_e2 >= 0.0f) ? -e2x : e2x;
                rough_hy = (moment3_e2 >= 0.0f) ? -e2y : e2y;
            }
            if(worker_ycar_dir_valid != 0u &&
               worker_ycar_hx * rough_hx + worker_ycar_hy * rough_hy < 0.0f)
            {
                rough_hx = -rough_hx;
                rough_hy = -rough_hy;
            }

            // Find the V apex at the narrow end of the rough symmetry axis.
            for(int y = 0; y < MT9V03X_H; y++)
            {
                for(int x = 0; x < MT9V03X_W; x++)
                {
                    uint8_t id = worker_labels[y][x];
                    float projection;
                    if(id == 0u || best_keep[id] == 0u) continue;
                    projection = (x - mx) * rough_hx + (y - my) * rough_hy;
                    if(projection > apex_max) apex_max = projection;
                }
            }
            for(int y = 0; y < MT9V03X_H; y++)
            {
                for(int x = 0; x < MT9V03X_W; x++)
                {
                    uint8_t id = worker_labels[y][x];
                    float projection;
                    if(id == 0u || best_keep[id] == 0u) continue;
                    projection = (x - mx) * rough_hx + (y - my) * rough_hy;
                    if(projection >= apex_max - 2.0f)
                    {
                        apex_x += x;
                        apex_y += y;
                        apex_count++;
                    }
                }
            }
            if(apex_count != 0u)
            {
                apex_x /= apex_count;
                apex_y /= apex_count;
                for(int y = 0; y < MT9V03X_H; y++)
                {
                    for(int x = 0; x < MT9V03X_W; x++)
                    {
                        uint8_t id = worker_labels[y][x];
                        float vx;
                        float vy;
                        float forward;
                        float side;
                        float dist2;
                        if(id == 0u || best_keep[id] == 0u) continue;
                        vx = x - apex_x;
                        vy = y - apex_y;
                        forward = vx * rough_hx + vy * rough_hy;
                        if(forward >= -2.0f) continue;
                        side = rough_hx * vy - rough_hy * vx;
                        dist2 = vx * vx + vy * vy;
                        if(side >= 0.0f && dist2 > left_dist2)
                        {
                            left_dist2 = dist2;
                            left_x = x;
                            left_y = y;
                        }
                        else if(side < 0.0f && dist2 > right_dist2)
                        {
                            right_dist2 = dist2;
                            right_x = x;
                            right_y = y;
                        }
                    }
                }
            }

            if(left_dist2 > 4.0f && right_dist2 > 4.0f)
            {
                float left_vx = left_x - apex_x;
                float left_vy = left_y - apex_y;
                float right_vx = right_x - apex_x;
                float right_vy = right_y - apex_y;
                float left_norm = sqrtf(left_vx * left_vx + left_vy * left_vy);
                float right_norm = sqrtf(right_vx * right_vx + right_vy * right_vy);
                float open_x = left_vx / left_norm + right_vx / right_norm;
                float open_y = left_vy / left_norm + right_vy / right_norm;
                float open_norm = sqrtf(open_x * open_x + open_y * open_y);
                if(open_norm > 1e-6f)
                {
                    hx = -open_x / open_norm;
                    hy = -open_y / open_norm;
                    if(worker_ycar_dir_valid != 0u &&
                       worker_ycar_hx * hx + worker_ycar_hy * hy < 0.0f)
                    {
                        hx = -hx;
                        hy = -hy;
                    }
                    if(worker_ycar_dir_valid == 0u)
                    {
                        worker_ycar_hx = hx;
                        worker_ycar_hy = hy;
                        worker_ycar_dir_valid = 1u;
                    }
                    else
                    {
                        worker_ycar_hx = 0.3f * worker_ycar_hx + 0.7f * hx;
                        worker_ycar_hy = 0.3f * worker_ycar_hy + 0.7f * hy;
                    }
                    norm = sqrtf(worker_ycar_hx * worker_ycar_hx +
                                 worker_ycar_hy * worker_ycar_hy);
                    if(norm > 1e-6f)
                    {
                        worker_ycar_hx /= norm;
                        worker_ycar_hy /= norm;
                    }
                }
            }

            snapshot->ycar_valid = 1u;
            snapshot->ycar_score = best_area;
            snapshot->ycar_x = (int16_t)(mx + 0.5f);
            snapshot->ycar_y = (int16_t)(my + 0.5f);
            if(worker_ycar_dir_valid != 0u)
            {
                snapshot->ycar_head_x = worker_ycar_hx;
                snapshot->ycar_head_y = worker_ycar_hy;
            }
            else
            {
                snapshot->ycar_head_x = rough_hx;
                snapshot->ycar_head_y = rough_hy;
            }
        }
    }
#endif
}

static void worker_detect(VisionDetectionSnapshot_t *snapshot)
{
    uint8_t parent[WORKER_MAX_LABELS];
    uint8_t root_to_id[WORKER_MAX_LABELS] = {0};
    uint16_t area[WORKER_MAX_LABELS] = {0};
    uint32_t sum_x[WORKER_MAX_LABELS] = {0};
    uint32_t sum_y[WORKER_MAX_LABELS] = {0};
    uint32_t sum_v[WORKER_MAX_LABELS] = {0};
    int16_t min_x[WORKER_MAX_LABELS];
    int16_t max_x[WORKER_MAX_LABELS];
    int16_t min_y[WORKER_MAX_LABELS];
    int16_t max_y[WORKER_MAX_LABELS];
    uint8_t next_label = 1u;
    uint8_t components = 0u;
    float best_score = -100000.0f;
    float second_score = -100000.0f;
    int16_t best_x = 0;
    int16_t best_y = 0;

    memset(worker_labels, 0, sizeof(worker_labels));
    for(uint8_t i = 0u; i < WORKER_MAX_LABELS; i++)
    {
        parent[i] = i;
        min_x[i] = MT9V03X_W;
        max_x[i] = 0;
        min_y[i] = MT9V03X_H;
        max_y[i] = 0;
    }

    for(int y = 0; y < MT9V03X_H; y++)
    {
        for(int x = 0; x < MT9V03X_W; x++)
        {
            uint8_t best = 0u;
            uint8_t n;
            if(worker_binary[y][x] == 0u) continue;
            if(x > 0 && (n = worker_labels[y][x - 1]) != 0u) best = n;
            if(y > 0 && (n = worker_labels[y - 1][x]) != 0u && (best == 0u || n < best)) best = n;
            if(x > 0 && y > 0 && (n = worker_labels[y - 1][x - 1]) != 0u && (best == 0u || n < best)) best = n;
            if(x + 1 < MT9V03X_W && y > 0 && (n = worker_labels[y - 1][x + 1]) != 0u && (best == 0u || n < best)) best = n;
            if(best == 0u)
            {
                if(next_label >= WORKER_MAX_LABELS) continue;
                best = next_label++;
            }
            worker_labels[y][x] = best;
            if(x > 0 && worker_labels[y][x - 1] != 0u) worker_union(parent, best, worker_labels[y][x - 1]);
            if(y > 0 && worker_labels[y - 1][x] != 0u) worker_union(parent, best, worker_labels[y - 1][x]);
            if(x > 0 && y > 0 && worker_labels[y - 1][x - 1] != 0u) worker_union(parent, best, worker_labels[y - 1][x - 1]);
            if(x + 1 < MT9V03X_W && y > 0 && worker_labels[y - 1][x + 1] != 0u) worker_union(parent, best, worker_labels[y - 1][x + 1]);
        }
    }

    for(int y = 0; y < MT9V03X_H; y++)
    {
        for(int x = 0; x < MT9V03X_W; x++)
        {
            uint8_t label = worker_labels[y][x];
            uint8_t id;
            if(label == 0u) continue;
            label = worker_find(parent, label);
            if(root_to_id[label] == 0u) root_to_id[label] = ++components;
            id = root_to_id[label];
            worker_labels[y][x] = id;
            area[id]++;
            sum_x[id] += (uint32_t)x;
            sum_y[id] += (uint32_t)y;
            sum_v[id] += worker_undistorted[y][x];
            if(x < min_x[id]) min_x[id] = x;
            if(x > max_x[id]) max_x[id] = x;
            if(y < min_y[id]) min_y[id] = y;
            if(y > max_y[id]) max_y[id] = y;
        }
    }

    snapshot->blob_count = components;
    worker_ycar_shape_valid = 0u;
    worker_detect_ycar(snapshot, components, area,
                       min_x, max_x, min_y, max_y);
    if(snapshot->ycar_valid != 0u)
    {
        worker_last_ycar_x = snapshot->ycar_x;
        worker_last_ycar_y = snapshot->ycar_y;
        worker_last_ycar_age = 0u;
    }
    else if(worker_last_ycar_age < 255u)
    {
        worker_last_ycar_age++;
        if(worker_last_ycar_age > WORKER_YCAR_EXCLUDE_HOLD_FRAMES)
        {
            worker_ycar_dir_valid = 0u;
            worker_ycar_pending_frames = 0u;
        }
    }
    for(uint8_t id = 1u; id <= components; id++)
    {
        VisionBeaconCandidate_t candidate;
        int width;
        int height;
        float aspect;
        float fill;
        float score;
        if(area[id] < WORKER_MIN_AREA || area[id] > WORKER_MAX_AREA) continue;
        width = max_x[id] - min_x[id] + 1;
        height = max_y[id] - min_y[id] + 1;
        if(width <= 0 || height <= 0) continue;
        aspect = (width > height) ? (float)width / height : (float)height / width;
        fill = (float)area[id] / ((float)width * height);
        candidate.x = (int16_t)((sum_x[id] + area[id] / 2u) / area[id]);
        candidate.y = (int16_t)((sum_y[id] + area[id] / 2u) / area[id]);
        candidate.area = area[id];
        candidate.brightness = (uint16_t)(sum_v[id] / area[id]);
        candidate.score = worker_base_score(&candidate);
        if(candidate.brightness < WORKER_BEACON_THRESHOLD || aspect >= 1.7f || fill <= 0.35f) continue;
        if(worker_ycar_shape_valid != 0u)
        {
            int dx = candidate.x - worker_ycar_shape_x;
            int dy = candidate.y - worker_ycar_shape_y;
            if(dx * dx + dy * dy <=
               WORKER_YCAR_SHAPE_EXCLUDE_RADIUS_PX *
               WORKER_YCAR_SHAPE_EXCLUDE_RADIUS_PX)
            {
                continue;
            }
        }
        if(worker_last_ycar_age <= WORKER_YCAR_EXCLUDE_HOLD_FRAMES)
        {
            int dx = candidate.x - worker_last_ycar_x;
            int dy = candidate.y - worker_last_ycar_y;
            if(dx * dx + dy * dy <=
               WORKER_YCAR_EXCLUDE_RADIUS_PX * WORKER_YCAR_EXCLUDE_RADIUS_PX)
            {
                continue;
            }
        }
        worker_insert_candidate(snapshot, candidate);

        score = candidate.score;
        if(worker_prev_x >= 0)
        {
            float dx = candidate.x - worker_prev_x;
            float dy = candidate.y - worker_prev_y;
            score += 40.0f / (1.0f + (dx * dx + dy * dy) / 100.0f);
        }
        if(score > best_score)
        {
            second_score = best_score;
            snapshot->beacon_second_x = best_x;
            snapshot->beacon_second_y = best_y;
            best_score = score;
            best_x = candidate.x;
            best_y = candidate.y;
        }
        else if(score > second_score)
        {
            second_score = score;
            snapshot->beacon_second_x = candidate.x;
            snapshot->beacon_second_y = candidate.y;
        }
    }

    if(best_score > -99999.0f)
    {
        if(worker_filter_x < 0)
        {
            worker_filter_x = best_x;
            worker_filter_y = best_y;
        }
        else
        {
            worker_filter_x = (int16_t)(worker_filter_x * 0.1f + best_x * 0.9f + 0.5f);
            worker_filter_y = (int16_t)(worker_filter_y * 0.1f + best_y * 0.9f + 0.5f);
        }
        worker_prev_x = worker_filter_x;
        worker_prev_y = worker_filter_y;
        snapshot->beacon_valid = 1u;
        snapshot->beacon_x = worker_filter_x;
        snapshot->beacon_y = worker_filter_y;
        snapshot->beacon_score = best_score;
        snapshot->beacon_second_score = (second_score > -99999.0f) ? second_score : 0.0f;
    }
}

void vision_worker_init(void)
{
    worker_frame_id = 0u;
    worker_last_camera_timestamp_us = 0u;
    worker_last_display_us = 0u;
    worker_prev_x = -1;
    worker_prev_y = -1;
    worker_filter_x = -1;
    worker_filter_y = -1;
    worker_ycar_hx = 0.0f;
    worker_ycar_hy = 0.0f;
    worker_ycar_dir_valid = 0u;
    worker_ycar_pending_hx = 0.0f;
    worker_ycar_pending_hy = 0.0f;
    worker_ycar_pending_frames = 0u;
    worker_ycar_shape_valid = 0u;
    worker_ycar_shape_x = 0;
    worker_ycar_shape_y = 0;
    worker_last_ycar_x = 0;
    worker_last_ycar_y = 0;
    worker_last_ycar_age = 255u;
    vision_shared_writer_init();
#if VISION_ASSISTANT_STREAM_ENABLE
    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_DEBUG_UART);
    seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_GRAY,
                                                 &worker_stream_image[0][0],
                                                 MT9V03X_W,
                                                 MT9V03X_H);
#endif
#if VISION_IPS114_ENABLE
    ips114_set_dir(VISION_IPS114_DIR);
    ips114_init();
    ips114_full(RGB565_BLACK);
#endif
    timer_init(TC_TIME2_CH1, TIMER_US);
    timer_clear(TC_TIME2_CH1);
    timer_start(TC_TIME2_CH1);
    mt9v03x_set_timestamp_source(vision_worker_time_us);
    mt9v03x_init();
}

uint8_t vision_worker_process(void)
{
    VisionDetectionSnapshot_t snapshot;
    VisionAttitudeSample_t attitude;
    uint32_t process_start_us;
    uint32_t camera_timestamp_us;

    if(mt9v03x_finish_flag == 0u) return 0u;
    mt9v03x_finish_flag = 0u;
    process_start_us = vision_worker_time_us();
    camera_timestamp_us = mt9v03x_frame_timestamp_us;
    memset(&snapshot, 0, sizeof(snapshot));
    snapshot.timestamp_us = camera_timestamp_us;
    snapshot.camera_dt_us = (worker_last_camera_timestamp_us == 0u) ? 0u :
        camera_timestamp_us - worker_last_camera_timestamp_us;
    snapshot.display_us = worker_last_display_us;
    worker_last_camera_timestamp_us = camera_timestamp_us;

    if(vision_attitude_shared_read(&attitude))
    {
        snapshot.attitude_generation = attitude.generation;
        snapshot.attitude_timestamp_us = attitude.timestamp_us;
        snapshot.frame_roll_deg = attitude.roll_deg;
        snapshot.frame_pitch_deg = attitude.pitch_deg;
        snapshot.frame_yaw_deg = attitude.yaw_deg;
        snapshot.frame_height_cm = attitude.height_cm;
    }

    memcpy(worker_base, mt9v03x_image, MT9V03X_IMAGE_SIZE);
    worker_remap();
    threshold_fixed(&worker_undistorted_img, &worker_binary_img,
                    IR_BINARY_THRESHOLD, 0u, 255u);
    worker_detect(&snapshot);
    snapshot.frame_id = ++worker_frame_id;
    snapshot.process_us = vision_worker_time_us() - process_start_us;
    vision_shared_publish(&snapshot);
#if VISION_ASSISTANT_STREAM_ENABLE
    if((worker_frame_id % VISION_ASSISTANT_STREAM_FRAME_DIVIDER) == 0u)
    {
        worker_stream_prepare(&snapshot);
        seekfree_assistant_camera_send();
    }
#endif
#if VISION_IPS114_ENABLE
    if((worker_frame_id % VISION_IPS114_FRAME_DIVIDER) == 0u)
    {
        uint32_t display_start_us = vision_worker_time_us();
        vision_worker_display(&snapshot);
        worker_last_display_us = vision_worker_time_us() - display_start_us;
    }
#endif
    return 1u;
}
