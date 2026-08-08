#include "zf_common_headfile.h"
#include "beacon.h"
#include "vision_nav.h"
#include "attitude_history.h"

// 信标信息结构
BeaconInfo beacon;
YCarInfo_t ycar_info;

// 误差相关变量
float erro;              // 当前误差（信标中心与图像中心的X偏移）
float error_back;        // 上一次误差
float erro_yawan;        // 偏航误差
float erro_yawan_back;   // 上一次偏航误差
float last_erro = 94;   // 最后一次误差（初始值94）

uint8_t debug_blob_cnt = 0;
float debug_beacon_score = 0.0f;
float debug_beacon_second_score = 0.0f;
int16_t debug_beacon_second_x = 0;
int16_t debug_beacon_second_y = 0;
uint8_t debug_beacon_lost = 0;
float debug_ycar_angle = 0.0f;
uint8_t debug_ycar_lost = 0;
VisionDetectionSnapshot_t vision_detection_snapshot;
uint32_t vision_last_frame_rx_us = 0u;
uint32_t vision_profile_camera_dt_us = 0u;
uint32_t vision_profile_process_us = 0u;
uint32_t vision_profile_remap_us = 0u;
uint32_t vision_profile_blob_us = 0u;
uint32_t vision_profile_detect_us = 0u;
uint32_t vision_profile_display_us = 0u;
float beacon_corr_x = 0.0f;
float beacon_corr_y = 0.0f;
float beacon_body_x = 0.0f;
float beacon_body_y = 0.0f;
float beacon_camera_x_cm = 0.0f;
float beacon_camera_y_cm = 0.0f;
float beacon_drone_x_cm = 0.0f;
float beacon_drone_y_cm = 0.0f;
float ycar_body_x = 0.0f;
float ycar_body_y = 0.0f;
float ycar_head_body_x = 0.0f;
float ycar_head_body_y = 0.0f;

static uint8_t label_buf[MT9V03X_H][MT9V03X_W];
static uint8_t beacon_lost_cnt = 0;
static uint8_t ycar_lost_cnt = 0;
static uint8_t vision_attitude_sync_ok = 0u;
static uint32_t vision_attitude_time_error_us = 0u;
#define VISION_SHARED_ATTITUDE_MAX_AGE_US 50000u
static VisionBeaconCandidate_t vision_beacon_candidates[VISION_MAX_BEACON_CANDIDATES];
static uint8_t vision_beacon_candidate_count = 0u;
static uint8_t mission_center_only_frames = 0u;
static uint8_t mission_selected_candidate_valid = 0u;
static int16_t mission_selected_candidate_x = 0;
static int16_t mission_selected_candidate_y = 0;

static void beacon_calibration_log(void)
{
#if BEACON_CAL_DEBUG_ENABLE
    static uint32_t last_log_time_us = 0u;
    static uint8_t header_printed = 0u;
    uint32_t now_us = system_time_us();

    if(now_us - last_log_time_us < BEACON_CAL_DEBUG_INTERVAL_US)
    {
        return;
    }
    last_log_time_us = now_us;

    if(header_printed == 0u)
    {
        printf("BCAL_HEADER,time_us,height_cm,roll_deg,pitch_deg,yaw_deg,valid,raw_x,raw_y,corr_x,corr_y,body_x_px,body_y_px,camera_x_cm,camera_y_cm,drone_x_cm,drone_y_cm,area,lost_frames,blob_count,score\r\n");
        header_printed = 1u;
    }

    printf("BCAL,%lu,%.2f,%.3f,%.3f,%.3f,%u,%d,%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%u,%u,%u,%.2f\r\n",
           (unsigned long)now_us,
           vehicle_state.current_height,
           imu_data.roll,
           imu_data.pitch,
           imu_data.yaw,
           (unsigned int)(beacon.status == BEACON_FOUND),
           beacon.centerX,
           beacon.centerY,
           beacon_corr_x,
           beacon_corr_y,
           beacon_body_x,
           beacon_body_y,
           beacon_camera_x_cm,
           beacon_camera_y_cm,
           beacon_drone_x_cm,
           beacon_drone_y_cm,
           (unsigned int)beacon.area,
           (unsigned int)debug_beacon_lost,
           (unsigned int)debug_blob_cnt,
           debug_beacon_score);
#endif
}

static void vision_test_log(void)
{
    static uint32_t last_log_time_us = 0u;
    uint32_t now_us = system_time_us();
    VisionDetectionSnapshot_t snapshot;
    uint8_t shared_ok;

    if(now_us - last_log_time_us < 200000u)
    {
        return;
    }
    last_log_time_us = now_us;

    shared_ok = vision_shared_read(&snapshot);
    if(shared_ok == 0u)
    {
        snapshot = vision_detection_snapshot;
    }

    printf("VTEST,shared=%u,att_sync=%u,att_dt_us=%lu,frame=%lu,frame_ts_us=%lu,cam_dt_us=%lu,proc_us=%lu,remap_us=%lu,blob_us=%lu,detect_us=%lu,display_us=%lu,blobs=%u,beacon=%u,best=%.1f,pos=(%d,%d),second=%.1f,pos=(%d,%d),margin=%.1f,ycar=%u,score=%u,pos=(%d,%d),head=(%.3f,%.3f)\r\n",
           (unsigned int)shared_ok,
           (unsigned int)vision_attitude_sync_ok,
           (unsigned long)vision_attitude_time_error_us,
           (unsigned long)snapshot.frame_id,
           (unsigned long)snapshot.timestamp_us,
           (unsigned long)vision_profile_camera_dt_us,
           (unsigned long)vision_profile_process_us,
           (unsigned long)vision_profile_remap_us,
           (unsigned long)vision_profile_blob_us,
           (unsigned long)vision_profile_detect_us,
           (unsigned long)vision_profile_display_us,
           (unsigned int)snapshot.blob_count,
           (unsigned int)snapshot.beacon_valid,
           snapshot.beacon_score,
           snapshot.beacon_x,
           snapshot.beacon_y,
           snapshot.beacon_second_score,
           snapshot.beacon_second_x,
           snapshot.beacon_second_y,
           snapshot.beacon_score - snapshot.beacon_second_score,
           (unsigned int)snapshot.ycar_valid,
           (unsigned int)snapshot.ycar_score,
           snapshot.ycar_x,
           snapshot.ycar_y,
           snapshot.ycar_head_x,
           snapshot.ycar_head_y);

    printf("BCANDS,frame=%lu,count=%u",
           (unsigned long)snapshot.frame_id,
           (unsigned int)snapshot.beacon_candidate_count);
    for(uint8_t i = 0u; i < snapshot.beacon_candidate_count; i++)
    {
        const VisionBeaconCandidate_t *candidate = &snapshot.beacon_candidates[i];
        printf(",%u:(%d,%d),area=%u,bright=%u,score=%.1f",
               (unsigned int)i,
               candidate->x,
               candidate->y,
               (unsigned int)candidate->area,
               (unsigned int)candidate->brightness,
               candidate->score);
    }
    printf("\r\n");
}
static uint8_t ir_blob_label_to_index[128];

static float beacon_candidate_base_score(const IrBlob_t *blob)
{
    float cx_dist = (float)(MT9V03X_W / 2 - blob->cx);
    if(cx_dist < 0.0f) cx_dist = -cx_dist;

    return (float)blob->brightness +
           (float)blob->area * 0.5f +
           ((float)MT9V03X_W / 2.0f - cx_dist) * 0.3f;
}

static void collect_beacon_candidates(const IrBlob_t blobs[], uint8_t blob_cnt)
{
    vision_beacon_candidate_count = 0u;
    memset(vision_beacon_candidates, 0, sizeof(vision_beacon_candidates));

    for(uint8_t i = 0u; i < blob_cnt; i++)
    {
        VisionBeaconCandidate_t candidate;
        uint8_t insert_at;

        if(!blobs[i].valid || blobs[i].type != IR_BLOB_BEACON_CANDIDATE)
        {
            continue;
        }

        candidate.x = blobs[i].cx;
        candidate.y = blobs[i].cy;
        candidate.area = blobs[i].area;
        candidate.brightness = blobs[i].brightness;
        candidate.score = beacon_candidate_base_score(&blobs[i]);

        insert_at = vision_beacon_candidate_count;
        if(insert_at >= VISION_MAX_BEACON_CANDIDATES)
        {
            insert_at = VISION_MAX_BEACON_CANDIDATES - 1u;
            if(candidate.score <= vision_beacon_candidates[insert_at].score)
            {
                continue;
            }
        }
        else
        {
            vision_beacon_candidate_count++;
        }

        while(insert_at > 0u &&
              candidate.score > vision_beacon_candidates[insert_at - 1u].score)
        {
            vision_beacon_candidates[insert_at] =
                vision_beacon_candidates[insert_at - 1u];
            insert_at--;
        }
        vision_beacon_candidates[insert_at] = candidate;
    }
}

static uint8_t ycar_label_gray(const image_t *img,
                               uint16_t area_table[],
                               int min_x[], int max_x[],
                               int min_y[], int max_y[]);

static void image_point_to_body_offset(float img_x, float img_y,
                                       float *body_x, float *body_y)
{
    *body_x = IMAGE_CENTER_Y - img_y;
    *body_y = img_x - IMAGE_CENTER_X;
}

static void beacon_update_metric_position_with_height(float height_cm)
{
    if(height_cm <= 0.0f)
    {
        beacon_camera_x_cm = 0.0f;
        beacon_camera_y_cm = 0.0f;
        beacon_drone_x_cm = 0.0f;
        beacon_drone_y_cm = 0.0f;
        return;
    }

    beacon_camera_x_cm = beacon_body_x * height_cm * BEACON_SCALE_X;
    beacon_camera_y_cm = beacon_body_y * height_cm * BEACON_SCALE_Y;
    beacon_drone_x_cm = beacon_camera_x_cm + CAMERA_OFFSET_BODY_X_CM;
    beacon_drone_y_cm = beacon_camera_y_cm + CAMERA_OFFSET_BODY_Y_CM;
}

static void mission_candidate_reset(void)
{
    mission_center_only_frames = 0u;
    mission_selected_candidate_valid = 0u;
    mission_selected_candidate_x = 0;
    mission_selected_candidate_y = 0;
}

static void mission_candidate_missed(void)
{
    mission_center_only_frames = 0u;
    if(vision_nav_target_active() == 0u)
    {
        mission_selected_candidate_valid = 0u;
    }
}

static uint8_t mission_select_beacon_candidate(
    const VisionDetectionSnapshot_t *snapshot,
    float frame_roll, float frame_pitch, float frame_height,
    int16_t *selected_x, int16_t *selected_y, uint16_t *selected_area)
{
    uint8_t center_candidate[VISION_MAX_BEACON_CANDIDATES] = {0u};
    uint8_t center_count = 0u;
    uint8_t outer_count = 0u;
    uint8_t selected = 0xFFu;
    uint8_t target_active = vision_nav_target_active();
    uint8_t locked_center = vision_nav_locked_target_is_center();
    uint8_t allow_center = 0u;
    int32_t best_distance_sq = 0x7FFFFFFF;

    if((snapshot == NULL) || (selected_x == NULL) ||
       (selected_y == NULL) || (selected_area == NULL) ||
       (snapshot->beacon_candidate_count == 0u))
    {
        mission_candidate_missed();
        return 0u;
    }

    if(vision_nav_beacon_selection_blocked() != 0u)
    {
        mission_candidate_reset();
        return 0u;
    }

    for(uint8_t i = 0u; i < snapshot->beacon_candidate_count; i++)
    {
        float corrected_x;
        float corrected_y;
        float body_x_px;
        float body_y_px;
        float body_x_cm;
        float body_y_cm;

        vision_attitude_compensation_with_attitude(
            (float)snapshot->beacon_candidates[i].x,
            (float)snapshot->beacon_candidates[i].y,
            frame_roll, frame_pitch, &corrected_x, &corrected_y);
        image_point_to_body_offset(corrected_x, corrected_y,
                                   &body_x_px, &body_y_px);
        body_x_cm = body_x_px * frame_height * BEACON_SCALE_X +
                    CAMERA_OFFSET_BODY_X_CM;
        body_y_cm = body_y_px * frame_height * BEACON_SCALE_Y +
                    CAMERA_OFFSET_BODY_Y_CM;
        center_candidate[i] = vision_nav_beacon_is_center_reference(
            body_x_cm, body_y_cm);
        if(center_candidate[i] != 0u) center_count++;
        else outer_count++;
    }

    if(target_active != 0u)
    {
        allow_center = locked_center;
        mission_center_only_frames = 0u;
    }
    else if(vision_nav_obs.search_center_settled == 0u)
    {
        /* A centre-looking lamp seen on the way in may be the only part of a
         * multi-lamp layout currently inside the camera view. Survey it from
         * the field centre before allowing the car to extinguish it. */
        allow_center = 0u;
        mission_center_only_frames = 0u;
    }
    else if((center_count != 0u) && (outer_count == 0u))
    {
        if(mission_center_only_frames < 255u)
        {
            mission_center_only_frames++;
        }
        allow_center = (mission_center_only_frames >=
                        VISION_NAV_CENTER_ONLY_CONFIRM_FRAMES) ? 1u : 0u;
    }
    else
    {
        /* Multiple lamps: keep the centre lamp as the return/search anchor
         * and select an outer lamp. */
        allow_center = 0u;
        mission_center_only_frames = 0u;
    }

    for(uint8_t i = 0u; i < snapshot->beacon_candidate_count; i++)
    {
        int32_t dx;
        int32_t dy;
        int32_t distance_sq;

        if((center_candidate[i] != 0u) != (allow_center != 0u)) continue;

        if((target_active == 0u) ||
           (mission_selected_candidate_valid == 0u))
        {
            selected = i;
            break;
        }

        dx = snapshot->beacon_candidates[i].x -
             mission_selected_candidate_x;
        dy = snapshot->beacon_candidates[i].y -
             mission_selected_candidate_y;
        distance_sq = dx * dx + dy * dy;
        if(distance_sq < best_distance_sq)
        {
            best_distance_sq = distance_sq;
            selected = i;
        }
    }

    if((selected == 0xFFu) ||
       ((target_active != 0u) &&
        (mission_selected_candidate_valid != 0u) &&
        (best_distance_sq >
         VISION_NAV_LOCKED_CANDIDATE_RADIUS_PX *
         VISION_NAV_LOCKED_CANDIDATE_RADIUS_PX)))
    {
        return 0u;
    }

    *selected_x = snapshot->beacon_candidates[selected].x;
    *selected_y = snapshot->beacon_candidates[selected].y;
    *selected_area = snapshot->beacon_candidates[selected].area;
    mission_selected_candidate_x = *selected_x;
    mission_selected_candidate_y = *selected_y;
    mission_selected_candidate_valid = 1u;
    return 1u;
}

static void image_vector_to_body_vector(float img_x, float img_y,
                                        float *body_x, float *body_y)
{
    *body_x = -img_y;
    *body_y = img_x;
}

#ifndef ABS
#define ABS(x) ((x) < 0 ? -(x) : (x))
#endif

static uint8_t label_find(uint8_t parent[], uint8_t x)
{
    while(parent[x] != x)
    {
        parent[x] = parent[parent[x]];
        x = parent[x];
    }
    return x;
}

static void label_union(uint8_t parent[], uint8_t a, uint8_t b)
{
    uint8_t ra = label_find(parent, a);
    uint8_t rb = label_find(parent, b);

    if(ra == rb) return;
    if(ra < rb) parent[rb] = ra;
    else        parent[ra] = rb;
}

uint8_t beacon_find_blobs_gray(const image_t *img, BeaconBlob_t blobs[], uint8_t max_blobs)
{
    uint8_t parent[128];
    uint8_t next_label = 1;
    uint8_t label_map[128] = {0};
    uint16_t area[128] = {0};
    uint32_t sum_x[128] = {0};
    uint32_t sum_y[128] = {0};
    uint32_t sum_v[128] = {0};
    uint8_t component_cnt = 0;

    memset(label_buf, 0, sizeof(label_buf));
    memset(blobs, 0, sizeof(BeaconBlob_t) * max_blobs);

    for(uint8_t i = 0; i < 128; i++) parent[i] = i;

    for(int y = 0; y < img->height; y++)
    {
        for(int x = 0; x < img->width; x++)
        {
            if(AT_IMAGE(img, x, y) < IR_THRESHOLD) continue;

            uint8_t best = 0;
            uint8_t n;

            if(x > 0)
            {
                n = label_buf[y][x - 1];
                if(n != 0) best = n;
            }
            if(y > 0)
            {
                n = label_buf[y - 1][x];
                if(n != 0 && (best == 0 || n < best)) best = n;
            }
            if(x > 0 && y > 0)
            {
                n = label_buf[y - 1][x - 1];
                if(n != 0 && (best == 0 || n < best)) best = n;
            }
            if((x + 1) < img->width && y > 0)
            {
                n = label_buf[y - 1][x + 1];
                if(n != 0 && (best == 0 || n < best)) best = n;
            }

            if(best == 0)
            {
                if(next_label >= 128) continue;
                best = next_label++;
            }

            label_buf[y][x] = best;

            if(x > 0)
            {
                n = label_buf[y][x - 1];
                if(n != 0) label_union(parent, best, n);
            }
            if(y > 0)
            {
                n = label_buf[y - 1][x];
                if(n != 0) label_union(parent, best, n);
            }
            if(x > 0 && y > 0)
            {
                n = label_buf[y - 1][x - 1];
                if(n != 0) label_union(parent, best, n);
            }
            if((x + 1) < img->width && y > 0)
            {
                n = label_buf[y - 1][x + 1];
                if(n != 0) label_union(parent, best, n);
            }
        }
    }

    for(int y = 0; y < img->height; y++)
    {
        for(int x = 0; x < img->width; x++)
        {
            uint8_t label = label_buf[y][x];
            if(label == 0) continue;

            label = label_find(parent, label);
            if(label_map[label] == 0)
            {
                if(component_cnt >= 127) continue;
                label_map[label] = ++component_cnt;
            }

            uint8_t id = label_map[label];
            area[id]++;
            sum_x[id] += (uint32_t)x;
            sum_y[id] += (uint32_t)y;
            sum_v[id] += (uint32_t)AT_IMAGE(img, x, y);
        }
    }

    uint8_t out_cnt = 0;
    for(uint8_t id = 1; id <= component_cnt && out_cnt < max_blobs; id++)
    {
        if(area[id] < BEACON_AREA_MIN || area[id] > BEACON_AREA_MAX) continue;

        blobs[out_cnt].cx = (int16_t)((sum_x[id] + area[id] / 2u) / area[id]);
        blobs[out_cnt].cy = (int16_t)((sum_y[id] + area[id] / 2u) / area[id]);
        blobs[out_cnt].area = area[id];
        blobs[out_cnt].brightness = (uint16_t)(sum_v[id] / area[id]);
        blobs[out_cnt].valid = 1;
        out_cnt++;
    }

    return out_cnt;
}

uint8_t ir_find_blobs_gray(const image_t *img, IrBlob_t blobs[], uint8_t max_blobs)
{
    uint16_t area_tbl[128];
    int minx[128], maxx[128], miny[128], maxy[128];
    uint32_t sumx[128] = {0};
    uint32_t sumy[128] = {0};
    uint32_t sumv[128] = {0};
    uint8_t labels;
    uint8_t out_cnt = 0;

    memset(blobs, 0, sizeof(IrBlob_t) * max_blobs);
    memset(ir_blob_label_to_index, 0xFF, sizeof(ir_blob_label_to_index));

    labels = ycar_label_gray(img, area_tbl, minx, maxx, miny, maxy);
    if(labels == 0) return 0;

    for(int y = 0; y < img->height; y++)
    {
        for(int x = 0; x < img->width; x++)
        {
            uint8_t lbl = label_buf[y][x];
            if(lbl == 0) continue;
            sumx[lbl] += (uint32_t)x;
            sumy[lbl] += (uint32_t)y;
            sumv[lbl] += (uint32_t)AT_IMAGE(img, x, y);
        }
    }

    for(uint8_t i = 1; i <= labels && out_cnt < max_blobs; i++)
    {
        uint16_t area = area_tbl[i];
        if(area == 0) continue;

        int bw = maxx[i] - minx[i] + 1;
        int bh = maxy[i] - miny[i] + 1;
        if(bw <= 0 || bh <= 0) continue;

        blobs[out_cnt].cx = (int16_t)((sumx[i] + area / 2u) / area);
        blobs[out_cnt].cy = (int16_t)((sumy[i] + area / 2u) / area);
        blobs[out_cnt].min_x = (int16_t)minx[i];
        blobs[out_cnt].max_x = (int16_t)maxx[i];
        blobs[out_cnt].min_y = (int16_t)miny[i];
        blobs[out_cnt].max_y = (int16_t)maxy[i];
        blobs[out_cnt].area = area;
        blobs[out_cnt].brightness = (uint16_t)(sumv[i] / area);
        blobs[out_cnt].aspect = (bw > bh) ? ((float)bw / (float)bh) : ((float)bh / (float)bw);
        blobs[out_cnt].fill = (float)area / ((float)bw * (float)bh);
        blobs[out_cnt].type = IR_BLOB_UNKNOWN;
        blobs[out_cnt].label = i;
        blobs[out_cnt].valid = 1;
        ir_blob_label_to_index[i] = out_cnt;
        out_cnt++;
    }

    return out_cnt;
}

void classify_ir_blobs(IrBlob_t blobs[], uint8_t blob_cnt)
{
    for(uint8_t i = 0; i < blob_cnt; i++)
    {
        if(!blobs[i].valid) continue;

        blobs[i].type = IR_BLOB_UNKNOWN;

        if(blobs[i].brightness >= IR_THRESHOLD &&
           blobs[i].area >= BEACON_AREA_MIN &&
           blobs[i].area <= BEACON_AREA_MAX &&
           blobs[i].aspect < 1.7f &&
           blobs[i].fill > 0.35f)
        {
            blobs[i].type = IR_BLOB_BEACON_CANDIDATE;
        }

        else if(blobs[i].area >= CAR_BLOB_MIN_AREA)
        {
            blobs[i].type = IR_BLOB_YCAR_PART;
        }
    }
}

void detect_beacon_from_ir_blobs(const IrBlob_t blobs[], uint8_t blob_cnt,
                                 float roll_deg, float pitch_deg,
                                 BeaconInfo *info)
{
    uint8_t best = 0;
    uint8_t found = 0;
    float best_score = -100000.0f;
    float second_score = -100000.0f;
    int16_t second_x = 0;
    int16_t second_y = 0;
    static int16_t prev_cx = -1;
    static int16_t prev_cy = -1;
    static int16_t filt_cx = -1;
    static int16_t filt_cy = -1;

    info->status = BEACON_NOT_FOUND;
    info->centerX = 0;
    info->centerY = 0;
    info->area = 0;
    info->sum = 0;

    for(uint8_t i = 0; i < blob_cnt; i++)
    {
        float score;
        if(!blobs[i].valid || blobs[i].type != IR_BLOB_BEACON_CANDIDATE) continue;

        score = beacon_candidate_base_score(&blobs[i]);

        if(prev_cx >= 0)
        {
            float dx = (float)(blobs[i].cx - prev_cx);
            float dy = (float)(blobs[i].cy - prev_cy);
            float dist2 = dx * dx + dy * dy;
            score += 40.0f / (1.0f + dist2 / 100.0f);
        }

        if(!found || score > best_score)
        {
            if(found)
            {
                second_score = best_score;
                second_x = blobs[best].cx;
                second_y = blobs[best].cy;
            }
            found = 1;
            best_score = score;
            best = i;
        }
        else if(score > second_score)
        {
            second_score = score;
            second_x = blobs[i].cx;
            second_y = blobs[i].cy;
        }
    }

    debug_blob_cnt = blob_cnt;
    debug_beacon_score = found ? best_score : 0.0f;
    debug_beacon_second_score = (second_score > -99999.0f) ? second_score : 0.0f;
    debug_beacon_second_x = second_x;
    debug_beacon_second_y = second_y;

    if(found)
    {
        float u_corr;
        float v_corr;

        info->status = BEACON_FOUND;
        info->area = blobs[best].area;
        info->sum = (int)(blobs[best].brightness * blobs[best].area);
        beacon_lost_cnt = 0;
        debug_beacon_lost = 0;

        if(filt_cx < 0)
        {
            filt_cx = blobs[best].cx;
            filt_cy = blobs[best].cy;
        }
        else
        {
            filt_cx = (int16_t)(filt_cx * (1.0f - BEACON_EMA_ALPHA) +
                                blobs[best].cx * BEACON_EMA_ALPHA + 0.5f);
            filt_cy = (int16_t)(filt_cy * (1.0f - BEACON_EMA_ALPHA) +
                                blobs[best].cy * BEACON_EMA_ALPHA + 0.5f);
        }

        info->centerX = filt_cx;
        info->centerY = filt_cy;
        prev_cx = filt_cx;
        prev_cy = filt_cy;

        vision_attitude_compensation_with_attitude(
            (float)info->centerX, (float)info->centerY,
            roll_deg, pitch_deg, &u_corr, &v_corr);
        beacon_corr_x = u_corr;
        beacon_corr_y = v_corr;
        image_point_to_body_offset(beacon_corr_x, beacon_corr_y,
                                   &beacon_body_x, &beacon_body_y);
        beacon_update_metric_position_with_height(vehicle_state.current_height);

        erro = 1.0f * (MT9V03X_W / 2 - u_corr);
        last_erro = erro;
        erro_yawan = erro;
        return;
    }

    if(beacon_lost_cnt < BEACON_LOST_SEARCH)
    {
        beacon_lost_cnt++;
        erro = last_erro;
    }
    else
    {
        if(ABS(last_erro) < 30)
            erro = (last_erro >= 0 ? 1 : -1) * 65.0f;
        else
            erro = last_erro;
    }

    debug_beacon_lost = beacon_lost_cnt;
    erro_yawan = erro;
}

static uint8_t ycar_label_gray(const image_t *img,
                               uint16_t area_table[],
                               int min_x[], int max_x[],
                               int min_y[], int max_y[])
{
    uint8_t parent[128];
    uint8_t next_label = 1;
    uint8_t label_map[128] = {0};
    uint8_t component_cnt = 0;

    memset(label_buf, 0, sizeof(label_buf));
    memset(area_table, 0, sizeof(uint16_t) * 128);
    for(uint8_t i = 0; i < 128; i++)
    {
        parent[i] = i;
        min_x[i] = img->width;
        max_x[i] = 0;
        min_y[i] = img->height;
        max_y[i] = 0;
    }

    for(int y = 0; y < img->height; y++)
    {
        for(int x = 0; x < img->width; x++)
        {
            if(AT_IMAGE(img, x, y) < CAR_IR_THRESHOLD) continue;

            uint8_t best = 0;
            uint8_t n;

            if(x > 0)
            {
                n = label_buf[y][x - 1];
                if(n != 0) best = n;
            }
            if(y > 0)
            {
                n = label_buf[y - 1][x];
                if(n != 0 && (best == 0 || n < best)) best = n;
            }
            if(x > 0 && y > 0)
            {
                n = label_buf[y - 1][x - 1];
                if(n != 0 && (best == 0 || n < best)) best = n;
            }
            if((x + 1) < img->width && y > 0)
            {
                n = label_buf[y - 1][x + 1];
                if(n != 0 && (best == 0 || n < best)) best = n;
            }

            if(best == 0)
            {
                if(next_label >= 128) continue;
                best = next_label++;
            }

            label_buf[y][x] = best;

            if(x > 0)
            {
                n = label_buf[y][x - 1];
                if(n != 0) label_union(parent, best, n);
            }
            if(y > 0)
            {
                n = label_buf[y - 1][x];
                if(n != 0) label_union(parent, best, n);
            }
            if(x > 0 && y > 0)
            {
                n = label_buf[y - 1][x - 1];
                if(n != 0) label_union(parent, best, n);
            }
            if((x + 1) < img->width && y > 0)
            {
                n = label_buf[y - 1][x + 1];
                if(n != 0) label_union(parent, best, n);
            }
        }
    }

    for(int y = 0; y < img->height; y++)
    {
        for(int x = 0; x < img->width; x++)
        {
            uint8_t label = label_buf[y][x];
            if(label == 0) continue;

            label = label_find(parent, label);
            if(label_map[label] == 0)
            {
                if(component_cnt >= 127) continue;
                label_map[label] = ++component_cnt;
            }

            uint8_t id = label_map[label];
            label_buf[y][x] = id;
            area_table[id]++;
            if(x < min_x[id]) min_x[id] = x;
            if(x > max_x[id]) max_x[id] = x;
            if(y < min_y[id]) min_y[id] = y;
            if(y > max_y[id]) max_y[id] = y;
        }
    }

    return component_cnt;
}

static uint8_t ycar_blob_is_near(const int minx[], const int maxx[],
                                 const int miny[], const int maxy[],
                                 uint8_t i, uint8_t j, int gap)
{
    int dx = (minx[i] > maxx[j]) ? (minx[i] - maxx[j])
           : (minx[j] > maxx[i]) ? (minx[j] - maxx[i]) : 0;
    int dy = (miny[i] > maxy[j]) ? (miny[i] - maxy[j])
           : (miny[j] > maxy[i]) ? (miny[j] - maxy[i]) : 0;
    return (dx <= gap && dy <= gap);
}

void ycar_detect_from_ir_blobs(const image_t *img, const IrBlob_t blobs[], uint8_t blob_cnt, YCarInfo_t *info)
{
    uint8_t keep[128];
    uint8_t best_keep[128];
    uint16_t best_score = 0;
    int32_t total_pixels = 0;
    int32_t sum_x = 0;
    int32_t sum_y = 0;
    int bb_min_x = img->width;
    int bb_max_x = 0;
    int bb_min_y = img->height;
    int bb_max_y = 0;

    info->valid = 0;
    info->cx = 0;
    info->cy = 0;
    info->hx = 0.0f;
    info->hy = 0.0f;
    info->score = 0u;

    memset(best_keep, 0, sizeof(best_keep));

    for(uint8_t seed = 0; seed < blob_cnt; seed++)
    {
        uint8_t tmp_keep[128];
        int tmp_min_x = img->width;
        int tmp_max_x = 0;
        int tmp_min_y = img->height;
        int tmp_max_y = 0;
        uint16_t tmp_area = 0;

        if(!blobs[seed].valid) continue;
        if(blobs[seed].area < CAR_BLOB_MIN_AREA) continue;

        memset(tmp_keep, 0, sizeof(tmp_keep));
        tmp_keep[blobs[seed].label] = 1;

        for(int changed = 1; changed; )
        {
            changed = 0;
            for(uint8_t i = 0; i < blob_cnt; i++)
            {
                if(!blobs[i].valid) continue;
                if(tmp_keep[blobs[i].label] || blobs[i].area < CAR_BLOB_MIN_AREA) continue;

                for(uint8_t j = 0; j < blob_cnt; j++)
                {
                    if(!blobs[j].valid) continue;
                    if(!tmp_keep[blobs[j].label]) continue;

                    int dx = (blobs[i].min_x > blobs[j].max_x) ? (blobs[i].min_x - blobs[j].max_x)
                           : (blobs[j].min_x > blobs[i].max_x) ? (blobs[j].min_x - blobs[i].max_x) : 0;
                    int dy = (blobs[i].min_y > blobs[j].max_y) ? (blobs[i].min_y - blobs[j].max_y)
                           : (blobs[j].min_y > blobs[i].max_y) ? (blobs[j].min_y - blobs[i].max_y) : 0;

                    if(dx <= CAR_BLOB_MERGE_GAP && dy <= CAR_BLOB_MERGE_GAP)
                    {
                        tmp_keep[blobs[i].label] = 1;
                        changed = 1;
                        break;
                    }
                }
            }
        }

        for(uint8_t i = 0; i < blob_cnt; i++)
        {
            if(!blobs[i].valid || !tmp_keep[blobs[i].label]) continue;
            tmp_area += blobs[i].area;
            if(blobs[i].min_x < tmp_min_x) tmp_min_x = blobs[i].min_x;
            if(blobs[i].max_x > tmp_max_x) tmp_max_x = blobs[i].max_x;
            if(blobs[i].min_y < tmp_min_y) tmp_min_y = blobs[i].min_y;
            if(blobs[i].max_y > tmp_max_y) tmp_max_y = blobs[i].max_y;
        }

        if(tmp_area >= CAR_MIN_PIX && tmp_area <= CAR_MAX_PIX)
        {
            int bw = tmp_max_x - tmp_min_x + 1;
            int bh = tmp_max_y - tmp_min_y + 1;
            float ratio;

            if(bw <= 0 || bh <= 0) continue;
            ratio = (bw > bh) ? ((float)bw / (float)bh) : ((float)bh / (float)bw);
            if(ratio < CAR_ASPECT_MIN || ratio > CAR_ASPECT_MAX) continue;

            if(tmp_area > best_score)
            {
                best_score = tmp_area;
                memcpy(best_keep, tmp_keep, sizeof(best_keep));
            }
        }
    }

    if(best_score == 0) return;
    info->score = best_score;
    memcpy(keep, best_keep, sizeof(keep));

    for(int y = 0; y < img->height; y++)
    {
        for(int x = 0; x < img->width; x++)
        {
            uint8_t lbl = label_buf[y][x];
            if(lbl != 0 && keep[lbl])
            {
                sum_x += x;
                sum_y += y;
                total_pixels++;
                if(x < bb_min_x) bb_min_x = x;
                if(x > bb_max_x) bb_max_x = x;
                if(y < bb_min_y) bb_min_y = y;
                if(y > bb_max_y) bb_max_y = y;
            }
        }
    }

    if(total_pixels < CAR_MIN_PIX || total_pixels > CAR_MAX_PIX) return;

    float mx = (float)sum_x / (float)total_pixels;
    float my = (float)sum_y / (float)total_pixels;
    float cxx = 0.0f;
    float cyy = 0.0f;
    float cxy = 0.0f;

    for(int y = 0; y < img->height; y++)
    {
        for(int x = 0; x < img->width; x++)
        {
            uint8_t lbl = label_buf[y][x];
            if(lbl == 0 || !keep[lbl]) continue;

            float dx = (float)x - mx;
            float dy = (float)y - my;
            cxx += dx * dx;
            cyy += dy * dy;
            cxy += dx * dy;
        }
    }

    cxx /= (float)total_pixels;
    cyy /= (float)total_pixels;
    cxy /= (float)total_pixels;

    float e1x;
    float e1y;
    if(fabsf(cxy) < 0.5f)
    {
        if(cxx >= cyy) { e1x = 1.0f; e1y = 0.0f; }
        else           { e1x = 0.0f; e1y = 1.0f; }
    }
    else
    {
        float d = (cxx - cyy) * 0.5f;
        float r = sqrtf(d * d + cxy * cxy);
        float nx = cxy;
        float ny = r - d;
        float norm = sqrtf(nx * nx + ny * ny);
        if(norm < 1e-8f) return;
        e1x = nx / norm;
        e1y = ny / norm;
    }

    float e2x = -e1y;
    float e2y = e1x;
    float p1_pos_max = -1e9f;
    float p1_pos_min = 1e9f;
    float p1_neg_max = -1e9f;
    float p1_neg_min = 1e9f;
    int cnt_pos = 0;
    int cnt_neg = 0;

    for(int y = 0; y < img->height; y++)
    {
        for(int x = 0; x < img->width; x++)
        {
            uint8_t lbl = label_buf[y][x];
            if(lbl == 0 || !keep[lbl]) continue;

            float dx = (float)x - mx;
            float dy = (float)y - my;
            float p1 = dx * e1x + dy * e1y;
            float p2 = dx * e2x + dy * e2y;

            if(p2 >= 0.0f)
            {
                if(p1 > p1_pos_max) p1_pos_max = p1;
                if(p1 < p1_pos_min) p1_pos_min = p1;
                cnt_pos++;
            }
            else
            {
                if(p1 > p1_neg_max) p1_neg_max = p1;
                if(p1 < p1_neg_min) p1_neg_min = p1;
                cnt_neg++;
            }
        }
    }

    float w_pos = (cnt_pos > 1) ? (p1_pos_max - p1_pos_min) : 0.0f;
    float w_neg = (cnt_neg > 1) ? (p1_neg_max - p1_neg_min) : 0.0f;
    float w_small = (w_pos < w_neg) ? w_pos : w_neg;
    float w_large = (w_pos > w_neg) ? w_pos : w_neg;
    float w_ratio = (w_small > 0.1f) ? (w_large / w_small) : 999.0f;
    float hx;
    float hy;

    if(w_ratio < CAR_W_RATIO_MIN) return;

    if(w_pos >= w_neg)
    {
        hx = -e2x;
        hy = -e2y;
    }
    else
    {
        hx = e2x;
        hy = e2y;
    }

    {
        static float filt_hx = 0.0f;
        static float filt_hy = 0.0f;
        static uint8_t ema_init = 0;

        if(!ema_init)
        {
            filt_hx = hx;
            filt_hy = hy;
            ema_init = 1;
        }
        else
        {
            float dot = filt_hx * hx + filt_hy * hy;
            if(dot < 0.0f)
            {
                hx = -hx;
                hy = -hy;
            }

            filt_hx = filt_hx * (1.0f - CAR_DIR_EMA_ALPHA) + hx * CAR_DIR_EMA_ALPHA;
            filt_hy = filt_hy * (1.0f - CAR_DIR_EMA_ALPHA) + hy * CAR_DIR_EMA_ALPHA;

            float norm = sqrtf(filt_hx * filt_hx + filt_hy * filt_hy);
            if(norm > 1e-6f)
            {
                filt_hx /= norm;
                filt_hy /= norm;
            }
        }

        hx = filt_hx;
        hy = filt_hy;
    }

    info->cx = (int16_t)(mx + 0.5f);
    info->cy = (int16_t)(my + 0.5f);
    info->hx = hx;
    info->hy = hy;
    info->valid = 1;
}

void ycar_detect_gray(const image_t *img, YCarInfo_t *info)
{
    uint16_t area_tbl[128];
    int minx[128], maxx[128], miny[128], maxy[128];
    uint8_t keep[128];
    uint8_t labels;
    uint8_t best_keep[128];
    uint16_t best_score = 0;
    int32_t total_pixels = 0;
    int32_t sum_x = 0;
    int32_t sum_y = 0;
    int bb_min_x = img->width;
    int bb_max_x = 0;
    int bb_min_y = img->height;
    int bb_max_y = 0;

    info->valid = 0;
    info->cx = 0;
    info->cy = 0;
    info->hx = 0.0f;
    info->hy = 0.0f;
    info->score = 0u;

    labels = ycar_label_gray(img, area_tbl, minx, maxx, miny, maxy);
    if(labels == 0) return;

    memset(best_keep, 0, sizeof(best_keep));

    for(uint8_t seed = 1; seed <= labels; seed++)
    {
        uint8_t tmp_keep[128];
        int tmp_min_x = img->width;
        int tmp_max_x = 0;
        int tmp_min_y = img->height;
        int tmp_max_y = 0;
        uint16_t tmp_area = 0;

        if(area_tbl[seed] < CAR_BLOB_MIN_AREA) continue;

        memset(tmp_keep, 0, sizeof(tmp_keep));
        tmp_keep[seed] = 1;

        for(int changed = 1; changed; )
        {
            changed = 0;
            for(uint8_t i = 1; i <= labels; i++)
            {
                if(tmp_keep[i] || area_tbl[i] < CAR_BLOB_MIN_AREA) continue;
                for(uint8_t j = 1; j <= labels; j++)
                {
                    if(!tmp_keep[j]) continue;
                    if(ycar_blob_is_near(minx, maxx, miny, maxy, i, j, CAR_BLOB_MERGE_GAP))
                    {
                        tmp_keep[i] = 1;
                        changed = 1;
                        break;
                    }
                }
            }
        }

        for(uint8_t i = 1; i <= labels; i++)
        {
            if(!tmp_keep[i]) continue;
            tmp_area += area_tbl[i];
            if(minx[i] < tmp_min_x) tmp_min_x = minx[i];
            if(maxx[i] > tmp_max_x) tmp_max_x = maxx[i];
            if(miny[i] < tmp_min_y) tmp_min_y = miny[i];
            if(maxy[i] > tmp_max_y) tmp_max_y = maxy[i];
        }

        if(tmp_area >= CAR_MIN_PIX && tmp_area <= CAR_MAX_PIX)
        {
            int bw = tmp_max_x - tmp_min_x + 1;
            int bh = tmp_max_y - tmp_min_y + 1;
            float ratio;

            if(bw <= 0 || bh <= 0) continue;
            ratio = (bw > bh) ? ((float)bw / (float)bh) : ((float)bh / (float)bw);
            if(ratio < CAR_ASPECT_MIN || ratio > CAR_ASPECT_MAX) continue;

            if(tmp_area > best_score)
            {
                best_score = tmp_area;
                memcpy(best_keep, tmp_keep, sizeof(best_keep));
            }
        }
    }

    if(best_score == 0) return;
    info->score = best_score;
    memcpy(keep, best_keep, sizeof(keep));

    for(int y = 0; y < img->height; y++)
    {
        for(int x = 0; x < img->width; x++)
        {
            uint8_t lbl = label_buf[y][x];
            if(lbl != 0 && keep[lbl])
            {
                sum_x += x;
                sum_y += y;
                total_pixels++;
                if(x < bb_min_x) bb_min_x = x;
                if(x > bb_max_x) bb_max_x = x;
                if(y < bb_min_y) bb_min_y = y;
                if(y > bb_max_y) bb_max_y = y;
            }
        }
    }

    if(total_pixels < CAR_MIN_PIX || total_pixels > CAR_MAX_PIX) return;

    {
        int bw = bb_max_x - bb_min_x + 1;
        int bh = bb_max_y - bb_min_y + 1;
        if(bw <= 0 || bh <= 0) return;

        float ratio = (bw > bh) ? ((float)bw / (float)bh) : ((float)bh / (float)bw);
        if(ratio < CAR_ASPECT_MIN || ratio > CAR_ASPECT_MAX) return;
    }

    float mx = (float)sum_x / (float)total_pixels;
    float my = (float)sum_y / (float)total_pixels;
    float cxx = 0.0f;
    float cyy = 0.0f;
    float cxy = 0.0f;

    for(int y = 0; y < img->height; y++)
    {
        for(int x = 0; x < img->width; x++)
        {
            uint8_t lbl = label_buf[y][x];
            if(lbl == 0 || !keep[lbl]) continue;

            float dx = (float)x - mx;
            float dy = (float)y - my;
            cxx += dx * dx;
            cyy += dy * dy;
            cxy += dx * dy;
        }
    }

    cxx /= (float)total_pixels;
    cyy /= (float)total_pixels;
    cxy /= (float)total_pixels;

    float e1x;
    float e1y;
    if(fabsf(cxy) < 0.5f)
    {
        if(cxx >= cyy)
        {
            e1x = 1.0f;
            e1y = 0.0f;
        }
        else
        {
            e1x = 0.0f;
            e1y = 1.0f;
        }
    }
    else
    {
        float d = (cxx - cyy) * 0.5f;
        float r = sqrtf(d * d + cxy * cxy);
        float nx = cxy;
        float ny = r - d;
        float norm = sqrtf(nx * nx + ny * ny);
        if(norm < 1e-8f) return;
        e1x = nx / norm;
        e1y = ny / norm;
    }

    float e2x = -e1y;
    float e2y = e1x;
    float p1_pos_max = -1e9f;
    float p1_pos_min = 1e9f;
    float p1_neg_max = -1e9f;
    float p1_neg_min = 1e9f;
    int cnt_pos = 0;
    int cnt_neg = 0;

    for(int y = 0; y < img->height; y++)
    {
        for(int x = 0; x < img->width; x++)
        {
            uint8_t lbl = label_buf[y][x];
            if(lbl == 0 || !keep[lbl]) continue;

            float dx = (float)x - mx;
            float dy = (float)y - my;
            float p1 = dx * e1x + dy * e1y;
            float p2 = dx * e2x + dy * e2y;

            if(p2 >= 0.0f)
            {
                if(p1 > p1_pos_max) p1_pos_max = p1;
                if(p1 < p1_pos_min) p1_pos_min = p1;
                cnt_pos++;
            }
            else
            {
                if(p1 > p1_neg_max) p1_neg_max = p1;
                if(p1 < p1_neg_min) p1_neg_min = p1;
                cnt_neg++;
            }
        }
    }

    float w_pos = (cnt_pos > 1) ? (p1_pos_max - p1_pos_min) : 0.0f;
    float w_neg = (cnt_neg > 1) ? (p1_neg_max - p1_neg_min) : 0.0f;
    float w_small = (w_pos < w_neg) ? w_pos : w_neg;
    float w_large = (w_pos > w_neg) ? w_pos : w_neg;
    float w_ratio = (w_small > 0.1f) ? (w_large / w_small) : 999.0f;
    float hx;
    float hy;

    if(w_ratio < CAR_W_RATIO_MIN) return;

    if(w_pos >= w_neg)
    {
        hx = -e2x;
        hy = -e2y;
    }
    else
    {
        hx = e2x;
        hy = e2y;
    }

    {
        static float filt_hx = 0.0f;
        static float filt_hy = 0.0f;
        static uint8_t ema_init = 0;

        if(!ema_init)
        {
            filt_hx = hx;
            filt_hy = hy;
            ema_init = 1;
        }
        else
        {
            float dot = filt_hx * hx + filt_hy * hy;
            if(dot < 0.0f)
            {
                hx = -hx;
                hy = -hy;
            }

            filt_hx = filt_hx * (1.0f - CAR_DIR_EMA_ALPHA) + hx * CAR_DIR_EMA_ALPHA;
            filt_hy = filt_hy * (1.0f - CAR_DIR_EMA_ALPHA) + hy * CAR_DIR_EMA_ALPHA;

            float norm = sqrtf(filt_hx * filt_hx + filt_hy * filt_hy);
            if(norm > 1e-6f)
            {
                filt_hx /= norm;
                filt_hy /= norm;
            }
        }

        hx = filt_hx;
        hy = filt_hy;
    }

    info->cx = (int16_t)(mx + 0.5f);
    info->cy = (int16_t)(my + 0.5f);
    info->hx = hx;
    info->hy = hy;
    info->valid = 1;
}

// ###########################################################################
// 函数名称: detect_beacon
// 函数功能: 从图像中检测信标（红外光源）
// 参数说明:
//           img  - 摄像头图像指针
//           info - 信标信息输出结构指针
// 返回说明: 无
// ###########################################################################
void detect_beacon(image_t *img, BeaconInfo *info)
{
    IrBlob_t blobs[MAX_IR_BLOBS];
    uint8_t blob_cnt = ir_find_blobs_gray(img, blobs, MAX_IR_BLOBS);
    classify_ir_blobs(blobs, blob_cnt);
    detect_beacon_from_ir_blobs(blobs, blob_cnt,
                                imu_data.roll, imu_data.pitch, info);
}

bool visited[MT9V03X_H][MT9V03X_W] = {false};

// 从二值化图像中寻找近似圆形的中心点
// 原理：找到上下左右距离近似相等的点（近似圆形）
uint16_t find_centers(const image_t *img, CenterPoint centers[MAX_CENTERS]) 
{
    // 初始化
    memset(visited, 0, sizeof(visited));               // 访问标记
    memset(centers, 0, sizeof(centers));             // 中心点数组
    uint16_t center_count = 0;                       // 找到的数量
    const uint8 max_error = 10;                        // 容许误差

    // 遍历图像
    for(uint16_t y = 0; y < MT9V03X_H; y++) {
        for(uint16_t x = 0; x < MT9V03X_W; x++) {
            // 找到白色像素且未访问
            if(AT_IMAGE(img, x, y) == 255 && !visited[y][x]) 
            {

                // 向下找最底端
                int bottom_y = y;
                while(bottom_y + 1 < MT9V03X_H && 
                      AT_IMAGE(img, x, bottom_y + 1) == 255 && 
                      !visited[bottom_y + 1][x])
                {
                    bottom_y++;
                }

                // 中点
                int middle_y = (y + bottom_y) / 2;
                int middle_x = x;

                // 向左找最左端
                int left_x = middle_x;
                while(left_x > 0 && 
                      AT_IMAGE(img, left_x - 1, middle_y) == 255 && 
                      !visited[middle_y][left_x - 1]) 
                {
                    left_x--;
                }

                // 向右找最右端
                int right_x = middle_x;
                while(right_x + 1 < MT9V03X_W && 
                      AT_IMAGE(img, right_x + 1, middle_y) == 255 && 
                      !visited[middle_y][right_x + 1]) 
                {
                    right_x++;
                }

                // 计算四个方向距离

                int up_distance = middle_y - y;
                int down_distance = bottom_y - middle_y;
                int left_distance = middle_x - left_x;
                int right_distance = right_x - middle_x;

                // 取最大最小值
                int max_dist = up_distance;
                int min_dist = up_distance;

                if(down_distance > max_dist) max_dist = down_distance;
                if(left_distance > max_dist) max_dist = left_distance;
                if(right_distance > max_dist) max_dist = right_distance;

                if(down_distance < min_dist) min_dist = down_distance;
                if(left_distance < min_dist) min_dist = left_distance;
                if(right_distance < min_dist) min_dist = right_distance;

                // 判断是否近似圆形（上下左右距离差值在容许范围内）

                if(max_dist - min_dist <= max_error) 
                {
                    // 记录中心点
                    if(center_count < MAX_CENTERS) 
                    {
                        centers[center_count].x = middle_x;
                        centers[center_count].y = middle_y;
                        centers[center_count].distance = max_dist;
                        // 权重：优先下方、靠近图像中心、半径大的点
                        centers[center_count].quanzhong = 
                            (1 - (float)middle_x / 94) * 0.4f +    // X居中
                            (float)middle_y / 120 * 0.3f +        // 下方优先
                            (float)max_dist * 0.3f;               // 半径大优先
                        center_count++;
                    }

                    // 标记该区域为已访问（避免重复检测）
 
                    int half_size = max_dist + 10;
                    int y_min = (middle_y > half_size) ? (middle_y - half_size) : 0;
                    int y_max = (middle_y + half_size < MT9V03X_H) ? 
                                (middle_y + half_size) : (MT9V03X_H - 1);
                    int x_min = (middle_x > half_size) ? (middle_x - half_size) : 0;
                    int x_max = (middle_x + half_size < MT9V03X_W) ? 
                                (middle_x + half_size) : (MT9V03X_W - 1);

                    for(int r = y_min; r <= y_max; r++) 
                    {
                        for(int c = x_min; c <= x_max; c++) 
                        {
                            if(AT_IMAGE(img, c, r) == 255) {
                                visited[r][c] = true;
                            }
                        }
                    }
                }
            }
        }
    }

    return center_count;
}

// --- 核心姿态解耦函数 (Attitude Compensation) ---
// 将倾斜画面中的像素，映射回一个"完美的虚拟水平相机"画面上。
// 输入：
//   - u_raw, v_raw: 图像处理算法识别到的、歪斜画面里的原始信标中心像素坐标。
//   - roll_deg: 图像采集时刻的横滚角，单位为度。
//   - pitch_deg: 图像采集时刻的俯仰角，单位为度。
// 输出：
//   - u_corr, v_corr: 指针，返回矫正后的、无姿态耦合的"上帝视角"坐标。
void vision_attitude_compensation_with_attitude(float u_raw, float v_raw,
                                                 float roll_deg, float pitch_deg,
                                                 float *u_corr, float *v_corr)
{
    // ==========================================
    // 步骤一：预计算 sin 和 cos 
    // ==========================================
    // 将角度转换为弧度的方法：(角度 * 3.14159f / 180.0f)
    // 确保 Roll > 0 代表右翼下倾；Pitch > 0 代表机头抬起。
    float roll_rad = roll_deg * 3.1415926f / 180.0f;
    float pitch_rad = pitch_deg * 3.1415926f / 180.0f;
    float cr = cosf(roll_rad);
    float sr = sinf(roll_rad);
    float cp = cosf(pitch_rad);
    float sp = sinf(pitch_rad);

    // ==========================================
    // 步骤二：图像坐标系 -> 相机3D射线向量 (Pc)
    // ==========================================
    // 这是一个以相机焦点为原点的三维坐标系：X_c右，Y_c下，Z_c正前(焦距)
    float Xc = u_raw - IMAGE_CENTER_X;
    float Yc = v_raw - IMAGE_CENTER_Y;
    float Zc = CAMERA_FOCAL_LENGTH_PIXEL;

    // ==========================================
    // 步骤三：3D旋转矩阵乘法（掰直 Pc 到 Pg）
    // ==========================================
    // Pg = R * Pc
    // 这里采用最通用的旋转矩阵（ Yaw-Pitch-Roll 顺序的逆变换），消除倾斜。    
    //  Xg: 转换后射线在水平面上的左右偏差
    float Xg = Zc * sp - Yc * cp * sr + Xc * cp * cr;

    //  Yg: 转换后射线在水平面上的前后偏差
    float Yg = Yc * cr + Xc * sr;

    // Zg: 转换后的视线深度 
    float Zg = Zc * cp + Yc * sp * sr - Xc * sp * cr;

    // ==========================================
    // 步骤四：重新投影回"虚拟水平图像平面" (深度除法)
    // ==========================================
    // 利用相似三角形原理，把 3D 的 P_g 投影回 2D 的 (u_corr, v_corr) 平面上。

    // 保护：如果飞机倾斜太夸张，导致 Zg 小于等于 0，说明目标在飞机后方或水平线上，不可见。
    if (Zg < 0.001f) {
        *u_corr = IMAGE_CENTER_X; // 丢弃该点，回到中心
        *v_corr = IMAGE_CENTER_Y;
        return;
    }
    // 最终解耦后的坐标输出
    *u_corr = IMAGE_CENTER_X + CAMERA_FOCAL_LENGTH_PIXEL * (Xg / Zg);
    *v_corr = IMAGE_CENTER_Y + CAMERA_FOCAL_LENGTH_PIXEL * (Yg / Zg);
}

void vision_attitude_compensation(float u_raw, float v_raw,
                                  float *u_corr, float *v_corr)
{
    vision_attitude_compensation_with_attitude(
        u_raw, v_raw, imu_data.roll, imu_data.pitch, u_corr, v_corr);
}

uint8_t image_process()
{
    IrBlob_t ir_blobs[MAX_IR_BLOBS];
    YCarInfo_t ycar_frame;
    mcar_guidance_t mcar_guidance;
    uint8_t ir_blob_cnt;
    uint32_t frame_timestamp_us;
    AttitudeHistorySample_t frame_attitude;
    static uint32_t last_frame_timestamp_us = 0u;
    uint32_t process_start_us;
    uint32_t stage_start_us;

    mcar_comm_poll();

    if(mt9v03x_finish_flag == 0u)
    {
        return 0u;
    }
    mt9v03x_finish_flag = 0u;
    frame_timestamp_us = mt9v03x_frame_timestamp_us;
    process_start_us = system_time_us();
    if(last_frame_timestamp_us != 0u)
    {
        vision_profile_camera_dt_us = frame_timestamp_us - last_frame_timestamp_us;
    }
    last_frame_timestamp_us = frame_timestamp_us;

    vision_attitude_sync_ok = attitude_history_find_nearest(
        frame_timestamp_us, &frame_attitude, &vision_attitude_time_error_us);
    if(vision_attitude_sync_ok == 0u ||
       vision_attitude_time_error_us > 10000u)
    {
        vision_attitude_sync_ok = 0u;
        frame_attitude.roll_deg = imu_data.roll;
        frame_attitude.pitch_deg = imu_data.pitch;
        frame_attitude.yaw_deg = imu_data.yaw;
    }

    stage_start_us = system_time_us();
    memcpy(base_image, mt9v03x_image, MT9V03X_IMAGE_SIZE); // 将采集到的图像数据复制到 base_image 中
    image_remap8(&base_img, &undistorted_img, &mapx_img, &mapy_img); // 去畸变重映射
    vision_profile_remap_us = system_time_us() - stage_start_us;

    stage_start_us = system_time_us();
    ir_blob_cnt = ir_find_blobs_gray(&undistorted_img, ir_blobs, MAX_IR_BLOBS);
    classify_ir_blobs(ir_blobs, ir_blob_cnt);
    collect_beacon_candidates(ir_blobs, ir_blob_cnt);
    vision_profile_blob_us = system_time_us() - stage_start_us;

    stage_start_us = system_time_us();
    detect_beacon_from_ir_blobs(ir_blobs, ir_blob_cnt,
                                frame_attitude.roll_deg,
                                frame_attitude.pitch_deg,
                                &beacon);
#if YCAR_DETECTION_ENABLE
    ycar_detect_from_ir_blobs(&undistorted_img, ir_blobs, ir_blob_cnt, &ycar_frame);
#else
    memset(&ycar_frame, 0, sizeof(ycar_frame));
#endif
    vision_profile_detect_us = system_time_us() - stage_start_us;

        if (ycar_frame.valid)
        {
            float mcar_corr_x;
            float mcar_corr_y;
            float mcar_head_corr_x;
            float mcar_head_corr_y;

            ycar_info = ycar_frame;
            ycar_lost_cnt = 0;
            debug_ycar_lost = 0;
            debug_ycar_angle = atan2f(ycar_info.hy, ycar_info.hx) * 57.29578f;
            vision_attitude_compensation_with_attitude(
                (float)ycar_info.cx, (float)ycar_info.cy,
                frame_attitude.roll_deg, frame_attitude.pitch_deg,
                &mcar_corr_x, &mcar_corr_y);
            vision_attitude_compensation_with_attitude(
                (float)ycar_info.cx + ycar_info.hx * 16.0f,
                (float)ycar_info.cy + ycar_info.hy * 16.0f,
                frame_attitude.roll_deg, frame_attitude.pitch_deg,
                &mcar_head_corr_x, &mcar_head_corr_y);
            image_point_to_body_offset(mcar_corr_x, mcar_corr_y,
                                       &ycar_body_x, &ycar_body_y);
            image_vector_to_body_vector(mcar_head_corr_x - mcar_corr_x,
                                        mcar_head_corr_y - mcar_corr_y,
                                        &ycar_head_body_x, &ycar_head_body_y);
            draw_o(&undistorted_img, ycar_info.cx, ycar_info.cy, 4, 180);
            draw_line(&undistorted_img,
                      (float)ycar_info.cx,
                      (float)ycar_info.cy,
                      (float)ycar_info.cx + ycar_info.hx * 16.0f,
                      (float)ycar_info.cy + ycar_info.hy * 16.0f,
                      180);
            {
                static uint32_t last_ycar_print_time = 0;
                if(system_time_us() - last_ycar_print_time > 100000)
                {
                    last_ycar_print_time = system_time_us();
                    printf("YCAR: img(%3d,%3d) body(%6.1f,%6.1f) head_body(%5.2f,%5.2f) angle=%.1f\n",
                           ycar_info.cx, ycar_info.cy,
                           ycar_body_x, ycar_body_y,
                           ycar_head_body_x, ycar_head_body_y,
                           debug_ycar_angle);
                }
            }
        }
        else
        {
            if(ycar_lost_cnt < 255) ycar_lost_cnt++;
            debug_ycar_lost = ycar_lost_cnt;
            if(ycar_lost_cnt > YCAR_LOST_HOLD)
            {
                ycar_info.valid = 0;
            }
        }

        // 3. 如果找到了信标点，进行解耦和标记
        if (beacon.status == BEACON_FOUND)
        {
            int u_raw = beacon.centerX;
            int v_raw = beacon.centerY;

            // 周期性通过串口打印，避免刷屏太快看不清
#if !BEACON_CAL_DEBUG_ENABLE
            static uint32_t last_vision_print_time = 0;
            if(system_time_us() - last_vision_print_time > 100000) {
                last_vision_print_time = system_time_us();
                printf("Beacon: raw(%3d,%3d) corr(%6.1f,%6.1f) body(%6.1f,%6.1f) area=%u blobs=%u score=%.1f\n",
                       u_raw, v_raw, beacon_corr_x, beacon_corr_y,
                       beacon_body_x, beacon_body_y,
                       beacon.area, debug_blob_cnt, debug_beacon_score);
            }
#endif
            // 在屏幕画面上画上标记，方便肉眼确认抓没抓对点
            draw_x(&undistorted_img, u_raw, v_raw, 5, 255);                       // 原始坐标画个白色的 'X'
            draw_o(&undistorted_img, (int)beacon_corr_x, (int)beacon_corr_y, 6, 128); // 解耦坐标画个灰色的 'O'
        }

    {
        static uint32_t frame_id = 0u;

        vision_detection_snapshot.timestamp_us = frame_timestamp_us;
        vision_detection_snapshot.blob_count = ir_blob_cnt;
        vision_detection_snapshot.beacon_valid = (beacon.status == BEACON_FOUND);
        vision_detection_snapshot.ycar_valid = ycar_frame.valid;
        vision_detection_snapshot.beacon_candidate_count = vision_beacon_candidate_count;
        vision_detection_snapshot.beacon_score = debug_beacon_score;
        vision_detection_snapshot.beacon_second_score = debug_beacon_second_score;
        vision_detection_snapshot.ycar_score = ycar_frame.score;
        vision_detection_snapshot.beacon_x = beacon.centerX;
        vision_detection_snapshot.beacon_y = beacon.centerY;
        vision_detection_snapshot.beacon_second_x = debug_beacon_second_x;
        vision_detection_snapshot.beacon_second_y = debug_beacon_second_y;
        vision_detection_snapshot.ycar_x = ycar_frame.cx;
        vision_detection_snapshot.ycar_y = ycar_frame.cy;
        vision_detection_snapshot.ycar_head_x = ycar_frame.hx;
        vision_detection_snapshot.ycar_head_y = ycar_frame.hy;
        memcpy(vision_detection_snapshot.beacon_candidates,
               vision_beacon_candidates,
               sizeof(vision_beacon_candidates));
        vision_detection_snapshot.frame_id = ++frame_id;
        vision_shared_publish(&vision_detection_snapshot);
    }

    vision_test_log();
    beacon_calibration_log();

    /* Vision navigation is run by param_update() at the 20 ms control
     * boundary.  This frame task must only publish observations. */

    mcar_guidance.valid = 0u;
    if(YCAR_GUIDANCE_ENABLE && ycar_info.valid)
    {
        if(vision_nav_obs.search_move_active != 0u)
        {
            mcar_guidance = mcar_guidance_calculate(
                0.0f, 0.0f, ycar_body_x, ycar_body_y,
                ycar_head_body_x, ycar_head_body_y, imu_data.yaw);
        }
        else if(vision_nav_beacon_command_valid() != 0u)
        {
            mcar_guidance = mcar_guidance_calculate(
                beacon_body_x, beacon_body_y, ycar_body_x, ycar_body_y,
                ycar_head_body_x, ycar_head_body_y, imu_data.yaw);
        }
    }

#if MCAR_COMM_HANDHELD_BEACON_FOLLOW_TEST_ENABLE
    mcar_guidance.valid = 0u;
    if((beacon.status == BEACON_FOUND) &&
       (debug_beacon_lost == 0u) &&
       (ycar_info.valid != 0u) &&
       (debug_ycar_lost == 0u))
    {
        mcar_guidance = mcar_guidance_calculate(
            beacon_body_x, beacon_body_y, ycar_body_x, ycar_body_y,
            ycar_head_body_x, ycar_head_body_y, imu_data.yaw);
    }
    if(mcar_guidance.valid != 0u)
    {
        uint8 flags = MCAR_COMM_FLAG_BEACON_VALID |
                      MCAR_COMM_FLAG_MCAR_VALID |
                      MCAR_COMM_FLAG_TARGET_ACTIVE;
        mcar_comm_send_target(vision_nav_target_seq(),
                              mcar_guidance.err_forward_px,
                              mcar_guidance.err_right_px,
                              0,
                              flags);
    }
    else
    {
        mcar_comm_send_stop(vision_nav_target_seq());
    }
#elif MCAR_COMM_FIXED_TARGET_TEST_ENABLE
    {
        uint8 flags = MCAR_COMM_FLAG_BEACON_VALID |
                      MCAR_COMM_FLAG_MCAR_VALID |
                      MCAR_COMM_FLAG_YAW_VALID |
                      MCAR_COMM_FLAG_TARGET_ACTIVE;
        mcar_comm_send_target(vision_nav_target_seq(), 50, 0, 0, flags);
    }
#else
    if(mcar_guidance.valid &&
       (vision_nav_target_active() ||
        vision_nav_obs.search_move_active != 0u) &&
       (vehicle_state.flight_mode == FLY_AUTOFLY) &&
       loc_1l_ct.loc_hold_ready != 0u &&
       (vision_nav_flight_ready() != 0u))
    {
        uint8 search_active =
            (vision_nav_obs.search_move_active != 0u) ? 1u : 0u;
        uint8 flags = MCAR_COMM_FLAG_MCAR_VALID |
                      MCAR_COMM_FLAG_TARGET_ACTIVE;
        if(search_active == 0u)
        {
            flags |= MCAR_COMM_FLAG_YAW_VALID;
        }
        /*
         * Existing car firmware treats BEACON_VALID as a generic guidance
         * target-valid gate.  During center return and lane scanning the
         * synthetic target is the camera center, so keep that compatibility
         * bit asserted and make the Y-car follow the aircraft continuously.
         */
        if((vision_nav_beacon_command_valid() != 0u) ||
           (vision_nav_obs.search_move_active != 0u))
        {
            flags |= MCAR_COMM_FLAG_BEACON_VALID;
        }
        if(debug_beacon_lost != 0u || debug_ycar_lost != 0u)
        {
            flags |= MCAR_COMM_FLAG_OBS_HELD;
        }
        if(mission_height_recovery_active != 0u)
        {
            /* Keep the car moving with the tethered aircraft, but ask the
             * car firmware to use its reduced recovery speed limit. */
            flags |= MCAR_COMM_FLAG_RECOVERY_SLOW;
        }

        mcar_comm_send_target(vision_nav_target_seq(),
                              mcar_guidance.err_forward_px,
                              mcar_guidance.err_right_px,
                              (search_active != 0u) ? 0 :
                                  mcar_guidance.mcar_yaw_earth_cdeg,
                              flags);
    }
    else
    {
        uint8 flags = MCAR_COMM_FLAG_STOP;
        if(vision_nav_beacon_command_valid() != 0u) flags |= MCAR_COMM_FLAG_BEACON_VALID;
        if(ycar_info.valid) flags |= MCAR_COMM_FLAG_MCAR_VALID;
        mcar_comm_send_target(vision_nav_target_seq(), 0, 0, 0, flags);
    }
#endif

    vision_profile_process_us = system_time_us() - process_start_us;
    return 1;
}

uint8_t vision_consumer_update(void)
{
    static uint32_t last_frame_id = 0u;
    static uint32_t last_frame_rx_us = 0u;
    static uint8_t stale_reported = 0u;
#if MCAR_COMM_HANDHELD_BEACON_FOLLOW_TEST_ENABLE || \
    MCAR_COMM_HANDHELD_FOLLOW_TEST_ENABLE
    static mcar_guidance_t handheld_guidance = {0};
    static uint32_t handheld_last_tx_us = 0u;
    static uint32_t handheld_beacon_last_seen_us = 0u;
    static uint8_t handheld_obs_held = 0u;
    static uint32_t debug_last_us = 0u;
    static uint32_t debug_last_frame_id = 0u;
    static uint32_t debug_last_tx_count = 0u;
    static uint32_t debug_last_rx_byte_count = 0u;
    static uint32_t debug_last_rx_bad_count = 0u;
    static uint32_t debug_last_call_us = 0u;
    static uint32_t debug_call_gap_max_us = 0u;
    static uint32_t debug_poll_max_us = 0u;
    static uint32_t debug_send_max_us = 0u;
#endif
    VisionDetectionSnapshot_t snapshot;
    mcar_guidance_t guidance = {0};
    uint32_t now_us = system_time_us();
    uint8_t have_new_frame = 0u;

#if MCAR_COMM_HANDHELD_BEACON_FOLLOW_TEST_ENABLE || \
    MCAR_COMM_HANDHELD_FOLLOW_TEST_ENABLE
    if(debug_last_call_us != 0u)
    {
        uint32_t call_gap_us = now_us - debug_last_call_us;
        if(call_gap_us > debug_call_gap_max_us)
        {
            debug_call_gap_max_us = call_gap_us;
        }
    }
    debug_last_call_us = now_us;
    {
        uint32_t poll_start_us = system_time_us();
        mcar_comm_poll();
        uint32_t poll_us = system_time_us() - poll_start_us;
        if(poll_us > debug_poll_max_us) debug_poll_max_us = poll_us;
    }
#else
    mcar_comm_poll();
#endif
    if(vision_shared_read(&snapshot) && snapshot.frame_id != last_frame_id)
    {
        float frame_roll = snapshot.frame_roll_deg;
        float frame_pitch = snapshot.frame_pitch_deg;
        float frame_yaw = snapshot.frame_yaw_deg;
        float frame_height = snapshot.frame_height_cm;

        have_new_frame = 1u;
        stale_reported = 0u;
        last_frame_id = snapshot.frame_id;
        last_frame_rx_us = now_us;
        vision_last_frame_rx_us = now_us;
        vision_detection_snapshot = snapshot;
        vision_attitude_time_error_us =
            (snapshot.attitude_generation != 0u) ?
            now_us - snapshot.attitude_timestamp_us : 0xFFFFFFFFu;
        vision_attitude_sync_ok =
            (snapshot.attitude_generation != 0u &&
             vision_attitude_time_error_us <=
             VISION_SHARED_ATTITUDE_MAX_AGE_US) ? 1u : 0u;
        debug_blob_cnt = snapshot.blob_count;
        debug_beacon_score = snapshot.beacon_score;
        debug_beacon_second_score = snapshot.beacon_second_score;
        debug_beacon_second_x = snapshot.beacon_second_x;
        debug_beacon_second_y = snapshot.beacon_second_y;

        {
            uint8_t selected_beacon_valid = 0u;
            int16_t selected_beacon_x = snapshot.beacon_x;
            int16_t selected_beacon_y = snapshot.beacon_y;
            uint16_t selected_beacon_area =
                (snapshot.beacon_candidate_count > 0u) ?
                snapshot.beacon_candidates[0].area : 0u;

            if(snapshot.beacon_valid && vision_attitude_sync_ok)
            {
                if(vision_nav_center_beacon_policy_active() != 0u)
                {
                    selected_beacon_valid = mission_select_beacon_candidate(
                        &snapshot, frame_roll, frame_pitch, frame_height,
                        &selected_beacon_x, &selected_beacon_y,
                        &selected_beacon_area);
                }
                else
                {
                    mission_candidate_reset();
                    selected_beacon_valid = 1u;
                }
            }
            else
            {
                mission_candidate_missed();
            }

            if(selected_beacon_valid != 0u)
            {
                beacon.status = BEACON_FOUND;
                beacon.centerX = selected_beacon_x;
                beacon.centerY = selected_beacon_y;
                beacon.area = selected_beacon_area;
                beacon_lost_cnt = 0u;
                debug_beacon_lost = 0u;
                vision_attitude_compensation_with_attitude(
                    (float)beacon.centerX, (float)beacon.centerY,
                    frame_roll, frame_pitch, &beacon_corr_x, &beacon_corr_y);
                image_point_to_body_offset(beacon_corr_x, beacon_corr_y,
                                           &beacon_body_x, &beacon_body_y);
                beacon_update_metric_position_with_height(frame_height);
            }
            else
            {
                beacon.status = BEACON_NOT_FOUND;
                if(beacon_lost_cnt < 255u) beacon_lost_cnt++;
                debug_beacon_lost = beacon_lost_cnt;
            }
        }

        if(snapshot.ycar_valid && vision_attitude_sync_ok)
        {
            float center_x;
            float center_y;
            float head_x;
            float head_y;
            ycar_info.valid = 1u;
            ycar_info.cx = snapshot.ycar_x;
            ycar_info.cy = snapshot.ycar_y;
            ycar_info.hx = snapshot.ycar_head_x;
            ycar_info.hy = snapshot.ycar_head_y;
            ycar_info.score = snapshot.ycar_score;
            vision_attitude_compensation_with_attitude(
                ycar_info.cx, ycar_info.cy, frame_roll, frame_pitch,
                &center_x, &center_y);
            vision_attitude_compensation_with_attitude(
                ycar_info.cx + ycar_info.hx * 16.0f,
                ycar_info.cy + ycar_info.hy * 16.0f,
                frame_roll, frame_pitch, &head_x, &head_y);
            image_point_to_body_offset(center_x, center_y,
                                       &ycar_body_x, &ycar_body_y);
            image_vector_to_body_vector(head_x - center_x, head_y - center_y,
                                        &ycar_head_body_x, &ycar_head_body_y);
            ycar_lost_cnt = 0u;
            debug_ycar_lost = 0u;
        }
        else
        {
            if(ycar_lost_cnt < 255u) ycar_lost_cnt++;
            debug_ycar_lost = ycar_lost_cnt;
            if(ycar_lost_cnt > YCAR_LOST_HOLD) ycar_info.valid = 0u;
        }

        //vision_test_log();
        //beacon_calibration_log();
        /* Setpoint ownership stays in the 20 ms control task. */

        if(YCAR_GUIDANCE_ENABLE && ycar_info.valid)
        {
            if(vision_nav_obs.search_move_active != 0u)
            {
                guidance = mcar_guidance_calculate(
                    0.0f, 0.0f, ycar_body_x, ycar_body_y,
                    ycar_head_body_x, ycar_head_body_y, frame_yaw);
            }
            else if(vision_nav_beacon_command_valid() != 0u)
            {
                guidance = mcar_guidance_calculate(
                    beacon_body_x, beacon_body_y, ycar_body_x, ycar_body_y,
                    ycar_head_body_x, ycar_head_body_y, frame_yaw);
            }
        }

#if MCAR_COMM_HANDHELD_BEACON_FOLLOW_TEST_ENABLE
        /* Handheld beacon-recognition test: bypass arming, flight mode,
         * height, LOC and mission gates.  Fresh Y-car and beacon observations
         * from the same processing stream normally update the command.  If a
         * previously confirmed beacon disappears only after entering the
         * Y-car's near-contact radius, keep its locked position for the same
         * bounded 200 ms cover grace used by the formal mission.  A fused
         * saturated lamp/tail blob has no honest two-object segmentation;
         * this lets the car finish the last few centimetres without allowing
         * a distant or prolonged dropout to continue motion. */
        guidance.valid = 0u;
        handheld_obs_held = 0u;
        if((beacon.status == BEACON_FOUND) &&
           (debug_beacon_lost == 0u) &&
           (ycar_info.valid != 0u) &&
           (debug_ycar_lost == 0u))
        {
            guidance = mcar_guidance_calculate(
                beacon_body_x, beacon_body_y, ycar_body_x, ycar_body_y,
                ycar_head_body_x, ycar_head_body_y, frame_yaw);
            handheld_beacon_last_seen_us = now_us;
        }
        else if((handheld_beacon_last_seen_us != 0u) &&
                ((now_us - handheld_beacon_last_seen_us) <=
                 MCAR_COMM_HANDHELD_BEACON_OVERLAP_HOLD_US) &&
                (ycar_info.valid != 0u) &&
                (debug_ycar_lost == 0u) &&
                (handheld_guidance.valid != 0u))
        {
            int32_t overlap_dx = (int32_t)beacon.centerX - ycar_info.cx;
            int32_t overlap_dy = (int32_t)beacon.centerY - ycar_info.cy;
            int32_t overlap_radius =
                MCAR_COMM_HANDHELD_BEACON_OVERLAP_RADIUS_PX;

            if((overlap_dx * overlap_dx + overlap_dy * overlap_dy) <=
               (overlap_radius * overlap_radius))
            {
                guidance = mcar_guidance_calculate(
                    beacon_body_x, beacon_body_y,
                    ycar_body_x, ycar_body_y,
                    ycar_head_body_x, ycar_head_body_y, frame_yaw);
                handheld_obs_held = (guidance.valid != 0u) ? 1u : 0u;
            }
        }

        handheld_guidance = guidance;
#elif MCAR_COMM_HANDHELD_FOLLOW_TEST_ENABLE
        /* Handheld communication test: intentionally bypass flight mode,
         * height, LOC, mission and arming gates.  Camera center is the
         * synthetic target, so the car follows the aircraft being carried.
         * Require a fresh observation; the first missed frame sends STOP. */
        guidance.valid = 0u;
        handheld_obs_held = 0u;
        if((ycar_info.valid != 0u) && (debug_ycar_lost == 0u))
        {
            guidance = mcar_guidance_calculate(
                0.0f, 0.0f, ycar_body_x, ycar_body_y,
                ycar_head_body_x, ycar_head_body_y, frame_yaw);
        }

        handheld_guidance = guidance;
#elif MCAR_COMM_FIXED_TARGET_TEST_ENABLE
        {
            uint8 flags = MCAR_COMM_FLAG_BEACON_VALID |
                          MCAR_COMM_FLAG_MCAR_VALID |
                          MCAR_COMM_FLAG_YAW_VALID |
                          MCAR_COMM_FLAG_TARGET_ACTIVE;
            mcar_comm_send_target(vision_nav_target_seq(), 20, 10, 9000, flags);
        }
#else
        if(guidance.valid &&
           (vision_nav_target_active() ||
            vision_nav_obs.search_move_active != 0u) &&
           (vehicle_state.flight_mode == FLY_AUTOFLY) &&
           loc_1l_ct.loc_hold_ready != 0u &&
           (vision_nav_flight_ready() != 0u))
        {
            uint8 search_active =
                (vision_nav_obs.search_move_active != 0u) ? 1u : 0u;
            uint8 flags = MCAR_COMM_FLAG_MCAR_VALID |
                          MCAR_COMM_FLAG_TARGET_ACTIVE;
            if(search_active == 0u)
            {
                flags |= MCAR_COMM_FLAG_YAW_VALID;
            }
            /* Camera center remains the valid guidance target throughout
             * center return and the continuous lane scan. */
            if((vision_nav_beacon_command_valid() != 0u) ||
               (vision_nav_obs.search_move_active != 0u))
            {
                flags |= MCAR_COMM_FLAG_BEACON_VALID;
            }
            if(vision_nav_beacon_command_held() != 0u ||
               debug_ycar_lost != 0u)
            {
                flags |= MCAR_COMM_FLAG_OBS_HELD;
            }
            if(mission_height_recovery_active != 0u)
            {
                flags |= MCAR_COMM_FLAG_RECOVERY_SLOW;
            }
            mcar_comm_send_target(vision_nav_target_seq(),
                                  guidance.err_forward_px,
                                  guidance.err_right_px,
                                  (search_active != 0u) ? 0 :
                                      guidance.mcar_yaw_earth_cdeg,
                                  flags);
        }
        else
        {
            uint8 flags = MCAR_COMM_FLAG_STOP;
            if(vision_nav_beacon_command_valid() != 0u) flags |= MCAR_COMM_FLAG_BEACON_VALID;
            if(ycar_info.valid) flags |= MCAR_COMM_FLAG_MCAR_VALID;
            mcar_comm_send_target(vision_nav_target_seq(), 0, 0, 0, flags);
        }
#endif
    }
#if !(MCAR_COMM_HANDHELD_BEACON_FOLLOW_TEST_ENABLE || \
      MCAR_COMM_HANDHELD_FOLLOW_TEST_ENABLE)
    else if(last_frame_rx_us != 0u &&
            now_us - last_frame_rx_us > 120000u && !stale_reported)
    {
        stale_reported = 1u;
        beacon.status = BEACON_NOT_FOUND;
        ycar_info.valid = 0u;
        /* The control task consumes the stale snapshot and owns setpoints. */
        mcar_comm_send_stop(vision_nav_target_seq());
    }
#endif

#if MCAR_COMM_HANDHELD_BEACON_FOLLOW_TEST_ENABLE || \
    MCAR_COMM_HANDHELD_FOLLOW_TEST_ENABLE
    /* Keep the protocol heartbeat independent of the CM7_1 vision cadence.
     * Fresh observations update the cached command above; a stale or invalid
     * observation is represented by a repeated STOP, which the car also
     * counts as a valid communication frame. */
    if((handheld_last_tx_us == 0u) ||
       ((now_us - handheld_last_tx_us) >= 20000u))
    {
        uint32_t send_start_us = system_time_us();
        if((handheld_guidance.valid != 0u) &&
           (last_frame_rx_us != 0u) &&
           ((now_us - last_frame_rx_us) <= 120000u))
        {
            uint8 flags = MCAR_COMM_FLAG_BEACON_VALID |
                          MCAR_COMM_FLAG_MCAR_VALID |
                          MCAR_COMM_FLAG_TARGET_ACTIVE;
            if(handheld_obs_held != 0u)
            {
                flags |= MCAR_COMM_FLAG_OBS_HELD;
            }
            mcar_comm_send_target(vision_nav_target_seq(),
                                  handheld_guidance.err_forward_px,
                                  handheld_guidance.err_right_px,
                                  0,
                                  flags);
        }
        else
        {
            handheld_guidance.valid = 0u;
            handheld_obs_held = 0u;
            mcar_comm_send_stop(vision_nav_target_seq());
        }
        {
            uint32_t send_us = system_time_us() - send_start_us;
            if(send_us > debug_send_max_us) debug_send_max_us = send_us;
        }
        handheld_last_tx_us = now_us;
    }

    if((debug_last_us == 0u) || ((now_us - debug_last_us) >= 1000000u))
    {
        char debug_line[384];
        int debug_len;
        uint32_t frame_delta = last_frame_id - debug_last_frame_id;
        uint32_t tx_delta = mcar_comm_diag.tx_count - debug_last_tx_count;
        uint32_t rx_byte_delta = mcar_comm_diag.rx_byte_count -
                                 debug_last_rx_byte_count;
        uint32_t rx_bad_delta = mcar_comm_diag.rx_bad_frame_count -
                                debug_last_rx_bad_count;
        uint32_t vision_age_us = (last_frame_rx_us == 0u) ?
            0xFFFFFFFFu : now_us - last_frame_rx_us;

        debug_len = snprintf(
            debug_line, sizeof(debug_line),
            "[DEBUG-MCAR-1FPS],t=%lu,frame=%lu,df=%lu,tx=%lu,dtx=%lu,txdt=%lu,age=%lu,camdt=%lu,proc=%lu,disp=%lu,flags=0x%02X,gapmax=%lu,pollmax=%lu,sendmax=%lu,rxbyte=%lu,rxbad=%lu\r\n",
            (unsigned long)now_us,
            (unsigned long)last_frame_id,
            (unsigned long)frame_delta,
            (unsigned long)mcar_comm_diag.tx_count,
            (unsigned long)tx_delta,
            (unsigned long)mcar_comm_diag.tx_period_us,
            (unsigned long)vision_age_us,
            (unsigned long)vision_detection_snapshot.camera_dt_us,
            (unsigned long)vision_detection_snapshot.process_us,
            (unsigned long)vision_detection_snapshot.display_us,
            (unsigned int)mcar_comm_diag.last_tx_flags,
            (unsigned long)debug_call_gap_max_us,
            (unsigned long)debug_poll_max_us,
            (unsigned long)debug_send_max_us,
            (unsigned long)rx_byte_delta,
            (unsigned long)rx_bad_delta);
        if(debug_len > 0)
        {
            uint32_t send_len = ((uint32_t)debug_len < sizeof(debug_line)) ?
                (uint32_t)debug_len : (uint32_t)(sizeof(debug_line) - 1u);
            debug_send_buffer((const uint8 *)debug_line, send_len);
        }

        debug_last_us = now_us;
        debug_last_frame_id = last_frame_id;
        debug_last_tx_count = mcar_comm_diag.tx_count;
        debug_last_rx_byte_count = mcar_comm_diag.rx_byte_count;
        debug_last_rx_bad_count = mcar_comm_diag.rx_bad_frame_count;
        debug_call_gap_max_us = 0u;
        debug_poll_max_us = 0u;
        debug_send_max_us = 0u;
    }
#endif

    return have_new_frame;
}
