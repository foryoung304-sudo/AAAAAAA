#include "zf_common_headfile.h"
#include "vision_worker.h"
#include "vision_shared.h"
#include "fisheye_lut.h"

static uint8_t worker_binary[MT9V03X_H][MT9V03X_W];
static uint8_t worker_base[MT9V03X_H][MT9V03X_W];
static uint8_t worker_undistorted[MT9V03X_H][MT9V03X_W];
static uint8_t worker_labels[MT9V03X_H][MT9V03X_W];

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
static float worker_ycar_hx = 0.0f;
static float worker_ycar_hy = 0.0f;
static uint8_t worker_ycar_dir_valid = 0u;
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
            if(width <= 0 || height <= 0) continue;
            aspect = (width > height) ? (float)width / height :
                                       (float)height / width;
            if(aspect < WORKER_YCAR_ASPECT_MIN ||
               aspect > WORKER_YCAR_ASPECT_MAX) continue;
            if(group_area > best_area)
            {
                best_area = group_area;
                memcpy(best_keep, keep, sizeof(best_keep));
            }
        }
    }

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
        float pos_min = 1e9f;
        float pos_max = -1e9f;
        float neg_min = 1e9f;
        float neg_max = -1e9f;
        uint16_t pos_count = 0u;
        uint16_t neg_count = 0u;

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
                if(p2 >= 0.0f)
                {
                    if(p1 < pos_min) pos_min = p1;
                    if(p1 > pos_max) pos_max = p1;
                    pos_count++;
                }
                else
                {
                    if(p1 < neg_min) neg_min = p1;
                    if(p1 > neg_max) neg_max = p1;
                    neg_count++;
                }
            }
        }

        if(pos_count > 1u && neg_count > 1u)
        {
            float pos_width = pos_max - pos_min;
            float neg_width = neg_max - neg_min;
            float small_width = (pos_width < neg_width) ? pos_width : neg_width;
            float large_width = (pos_width > neg_width) ? pos_width : neg_width;
            float hx;
            float hy;
            float norm;
            if(small_width <= 0.1f ||
               large_width / small_width < WORKER_YCAR_WIDTH_RATIO) return;
            if(pos_width >= neg_width) { hx = -e2x; hy = -e2y; }
            else                       { hx =  e2x; hy =  e2y; }
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
            if(norm < 1e-6f) return;
            worker_ycar_hx /= norm;
            worker_ycar_hy /= norm;
            snapshot->ycar_valid = 1u;
            snapshot->ycar_score = best_area;
            snapshot->ycar_x = (int16_t)(mx + 0.5f);
            snapshot->ycar_y = (int16_t)(my + 0.5f);
            snapshot->ycar_head_x = worker_ycar_hx;
            snapshot->ycar_head_y = worker_ycar_hy;
        }
    }
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
    worker_prev_x = -1;
    worker_prev_y = -1;
    worker_filter_x = -1;
    worker_filter_y = -1;
    worker_ycar_hx = 0.0f;
    worker_ycar_hy = 0.0f;
    worker_ycar_dir_valid = 0u;
    worker_last_ycar_x = 0;
    worker_last_ycar_y = 0;
    worker_last_ycar_age = 255u;
    vision_shared_writer_init();
#if VISION_ASSISTANT_STREAM_ENABLE
    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_DEBUG_UART);
    seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_GRAY,
                                                 &worker_undistorted[0][0],
                                                 MT9V03X_W,
                                                 MT9V03X_H);
#endif
#if VISION_IPS114_ENABLE
    ips114_set_dir(VISION_IPS114_DIR);
    ips114_init();
    ips114_full(RGB565_BLACK);
#endif
    mt9v03x_init();
}

uint8_t vision_worker_process(void)
{
    VisionDetectionSnapshot_t snapshot;
    VisionAttitudeSample_t attitude;

    if(mt9v03x_finish_flag == 0u) return 0u;
    mt9v03x_finish_flag = 0u;
    memset(&snapshot, 0, sizeof(snapshot));
    memcpy(worker_base, mt9v03x_image, MT9V03X_IMAGE_SIZE);
    worker_remap();
    threshold_fixed(&worker_undistorted_img, &worker_binary_img,
                    IR_BINARY_THRESHOLD, 0u, 255u);
    worker_detect(&snapshot);

    if(vision_attitude_shared_read(&attitude))
    {
        snapshot.attitude_generation = attitude.generation;
        snapshot.attitude_timestamp_us = attitude.timestamp_us;
        snapshot.timestamp_us = attitude.timestamp_us;
        snapshot.frame_roll_deg = attitude.roll_deg;
        snapshot.frame_pitch_deg = attitude.pitch_deg;
        snapshot.frame_yaw_deg = attitude.yaw_deg;
        snapshot.frame_height_cm = attitude.height_cm;
    }
    snapshot.frame_id = ++worker_frame_id;
    vision_shared_publish(&snapshot);
#if VISION_ASSISTANT_STREAM_ENABLE
    if((worker_frame_id % VISION_ASSISTANT_STREAM_FRAME_DIVIDER) == 0u)
    {
        seekfree_assistant_camera_send();
    }
#endif
#if VISION_IPS114_ENABLE
    if((worker_frame_id % VISION_IPS114_FRAME_DIVIDER) == 0u)
    {
        vision_worker_display(&snapshot);
    }
#endif
    return 1u;
}
