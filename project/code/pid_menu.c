#include "zf_common_headfile.h"

#include <math.h>

#define FIELD_MAP_FLASH_MAGIC          0x464D5031u /* FMP1 */
#define FIELD_MAP_FLASH_VERSION        1u
#define FIELD_MAP_CENTER_TOLERANCE_CM  0.5f
#define FIELD_MAP_RAD_PER_DEG          0.01745329252f

typedef char field_map_flash_page_in_range[
    (FIELD_MAP_FLASH_PAGE < FLASH_PAGE_NUM) ? 1 : -1];
typedef char field_map_does_not_use_pid_page[
    (FIELD_MAP_FLASH_PAGE != FIELD_MAP_PID_RESERVED_PAGE) ? 1 : -1];

typedef struct
{
    uint32_t magic;
    uint32_t version;
    uint32_t length_bytes;
    uint32_t sequence;
    uint32_t point_count;
    uint32_t finalized;
    float point_forward_cm[VISION_FIELD_MAP_MAX_POINTS];
    float point_right_cm[VISION_FIELD_MAP_MAX_POINTS];
    float center_forward_cm;
    float center_right_cm;
    uint32_t checksum;
} field_map_flash_data_t;

typedef char field_map_record_fits_one_page[
    (sizeof(field_map_flash_data_t) <= FLASH_PAGE_SIZE) ? 1 : -1];
typedef char field_map_record_word_aligned[
    ((sizeof(field_map_flash_data_t) & 3u) == 0u) ? 1 : -1];

static field_map_flash_data_t field_map_record;
static uint8_t field_map_flash_valid = 0u;
static uint8_t field_map_valid = 0u;
static uint8_t field_map_session_active = 0u;
static uint8_t field_map_last_armed = 0u;
static uint8_t field_map_key1_long_latched = 0u;
static uint8_t field_map_key4_long_latched = 0u;
static uint8_t field_map_run_origin_valid = 0u;
static float field_map_session_start_x_cm = 0.0f;
static float field_map_session_start_y_cm = 0.0f;
static float field_map_session_start_yaw_deg = 0.0f;
static float field_map_session_yaw_sin = 0.0f;
static float field_map_session_yaw_cos = 1.0f;
static float field_map_run_origin_x_cm = 0.0f;
static float field_map_run_origin_y_cm = 0.0f;
static float field_map_run_origin_yaw_sin = 0.0f;
static float field_map_run_origin_yaw_cos = 1.0f;
static field_map_status_t field_map_status = FIELD_MAP_STATUS_EMPTY;

/* Main loop publishes this small display snapshot; the 2 ms IMU interrupt
 * reads it through a seqlock and forwards it to CM7_1.  No absolute RAM
 * placement is used, so this cannot recreate the old camera-DMA overlap. */
static volatile uint32_t field_map_display_generation = 0u;
static VisionFieldMapDisplay_t field_map_display_snapshot;

static uint8_t field_map_float_valid(float value)
{
    return ((value == value) &&
            (value <= FIELD_MAP_COORD_LIMIT_CM) &&
            (value >= -FIELD_MAP_COORD_LIMIT_CM)) ? 1u : 0u;
}

static uint8_t field_map_pose_valid(void)
{
    if((vehicle_state.current_pos_x != vehicle_state.current_pos_x) ||
       (vehicle_state.current_pos_y != vehicle_state.current_pos_y) ||
       (vehicle_state.current_yaw != vehicle_state.current_yaw))
    {
        return 0u;
    }
    if((vehicle_state.current_pos_x > 10000.0f) ||
       (vehicle_state.current_pos_x < -10000.0f) ||
       (vehicle_state.current_pos_y > 10000.0f) ||
       (vehicle_state.current_pos_y < -10000.0f) ||
       (vehicle_state.current_yaw > 720.0f) ||
       (vehicle_state.current_yaw < -720.0f))
    {
        return 0u;
    }
    return 1u;
}

static uint32_t field_map_flash_word_len(void)
{
    return (uint32_t)(sizeof(field_map_flash_data_t) / sizeof(uint32_t));
}

static uint32_t field_map_checksum(const field_map_flash_data_t *data)
{
    const uint32_t *words = (const uint32_t *)data;
    uint32_t checksum = 0x8D31F2A7u;
    uint32_t word_count = field_map_flash_word_len() - 1u;

    for(uint32_t index = 0u; index < word_count; index++)
    {
        checksum ^= words[index] + 0x9E3779B9u +
                    (checksum << 6) + (checksum >> 2);
    }
    return checksum;
}

static void field_map_calculate_center(field_map_flash_data_t *data)
{
    float sum_forward = 0.0f;
    float sum_right = 0.0f;

    if(data->point_count == 0u)
    {
        data->center_forward_cm = 0.0f;
        data->center_right_cm = 0.0f;
        return;
    }

    for(uint32_t index = 0u; index < data->point_count; index++)
    {
        sum_forward += data->point_forward_cm[index];
        sum_right += data->point_right_cm[index];
    }
    data->center_forward_cm = sum_forward / (float)data->point_count;
    data->center_right_cm = sum_right / (float)data->point_count;
}

static uint8_t field_map_record_valid(const field_map_flash_data_t *data)
{
    field_map_flash_data_t recomputed;

    if((data->magic != FIELD_MAP_FLASH_MAGIC) ||
       (data->version != FIELD_MAP_FLASH_VERSION) ||
       (data->length_bytes != sizeof(field_map_flash_data_t)) ||
       (data->point_count > VISION_FIELD_MAP_MAX_POINTS) ||
       (data->finalized > 1u) ||
       (field_map_checksum(data) != data->checksum))
    {
        return 0u;
    }

    for(uint32_t index = 0u; index < data->point_count; index++)
    {
        if((field_map_float_valid(data->point_forward_cm[index]) == 0u) ||
           (field_map_float_valid(data->point_right_cm[index]) == 0u))
        {
            return 0u;
        }
    }
    if((field_map_float_valid(data->center_forward_cm) == 0u) ||
       (field_map_float_valid(data->center_right_cm) == 0u))
    {
        return 0u;
    }
    if((data->finalized != 0u) &&
       (data->point_count < FIELD_MAP_MIN_BEACON_POINTS))
    {
        return 0u;
    }

    recomputed = *data;
    field_map_calculate_center(&recomputed);
    if((fabsf(recomputed.center_forward_cm - data->center_forward_cm) >
        FIELD_MAP_CENTER_TOLERANCE_CM) ||
       (fabsf(recomputed.center_right_cm - data->center_right_cm) >
        FIELD_MAP_CENTER_TOLERANCE_CM))
    {
        return 0u;
    }
    return 1u;
}

static uint8_t field_map_flash_save(void)
{
    field_map_flash_data_t verify;

    field_map_record.magic = FIELD_MAP_FLASH_MAGIC;
    field_map_record.version = FIELD_MAP_FLASH_VERSION;
    field_map_record.length_bytes = sizeof(field_map_flash_data_t);
    field_map_record.sequence++;
    field_map_record.checksum = field_map_checksum(&field_map_record);

    /* Work Flash page 94 is one independent 2 KiB large sector.  Page 95,
     * historically used by the PID menu, is never read, erased or written. */
    flash_write_page(0u, FIELD_MAP_FLASH_PAGE,
                     (const uint32_t *)&field_map_record,
                     field_map_flash_word_len());
    memset(&verify, 0, sizeof(verify));
    flash_read_page(0u, FIELD_MAP_FLASH_PAGE,
                    (uint32_t *)&verify,
                    field_map_flash_word_len());

    if((field_map_record_valid(&verify) != 0u) &&
       (memcmp(&verify, &field_map_record, sizeof(verify)) == 0))
    {
        field_map_flash_valid = 1u;
        return 1u;
    }

    field_map_flash_valid = 0u;
    field_map_valid = 0u;
    return 0u;
}

static void field_map_flash_load(void)
{
    memset(&field_map_record, 0, sizeof(field_map_record));
    field_map_flash_valid = 0u;
    field_map_valid = 0u;

    if(flash_check(0u, FIELD_MAP_FLASH_PAGE) == 0u)
    {
        field_map_status = FIELD_MAP_STATUS_EMPTY;
        return;
    }

    flash_read_page(0u, FIELD_MAP_FLASH_PAGE,
                    (uint32_t *)&field_map_record,
                    field_map_flash_word_len());
    if(field_map_record_valid(&field_map_record) == 0u)
    {
        memset(&field_map_record, 0, sizeof(field_map_record));
        field_map_status = FIELD_MAP_STATUS_MAP_BAD;
        return;
    }

    field_map_flash_valid = 1u;
    field_map_valid = (field_map_record.finalized != 0u) ? 1u : 0u;
    field_map_status = (field_map_valid != 0u) ?
        FIELD_MAP_STATUS_LOADED : FIELD_MAP_STATUS_NEED_START;
}

static uint8_t field_map_capture_allowed(void)
{
    if(vehicle_state.armed != 0u)
    {
        field_map_status = FIELD_MAP_STATUS_ARMED_LOCK;
        return 0u;
    }
    if((vehicle_state.flow_valid == 0u) || (field_map_pose_valid() == 0u))
    {
        field_map_status = FIELD_MAP_STATUS_FLOW_BAD;
        return 0u;
    }
    return 1u;
}

static void field_map_current_local(float *forward_cm, float *right_cm)
{
    float delta_x;
    float delta_y;

    *forward_cm = 0.0f;
    *right_cm = 0.0f;
    if((field_map_session_active == 0u) || (field_map_pose_valid() == 0u))
    {
        return;
    }

    delta_x = vehicle_state.current_pos_x - field_map_session_start_x_cm;
    delta_y = vehicle_state.current_pos_y - field_map_session_start_y_cm;
    *forward_cm = delta_x * field_map_session_yaw_cos +
                  delta_y * field_map_session_yaw_sin;
    *right_cm = -delta_x * field_map_session_yaw_sin +
                  delta_y * field_map_session_yaw_cos;
}

static void field_map_begin_session(void)
{
    uint32_t old_sequence;
    float yaw_rad;

    if(field_map_capture_allowed() == 0u)
    {
        return;
    }

    old_sequence = field_map_record.sequence;
    memset(&field_map_record, 0, sizeof(field_map_record));
    field_map_record.sequence = old_sequence;
    field_map_session_start_x_cm = vehicle_state.current_pos_x;
    field_map_session_start_y_cm = vehicle_state.current_pos_y;
    field_map_session_start_yaw_deg = vehicle_state.current_yaw;
    yaw_rad = field_map_session_start_yaw_deg * FIELD_MAP_RAD_PER_DEG;
    field_map_session_yaw_sin = sinf(yaw_rad);
    field_map_session_yaw_cos = cosf(yaw_rad);
    field_map_session_active = 1u;
    field_map_valid = 0u;
    field_map_calculate_center(&field_map_record);

    if(field_map_flash_save() != 0u)
    {
        field_map_status = FIELD_MAP_STATUS_START_SAVED;
    }
    else
    {
        field_map_status = FIELD_MAP_STATUS_FLASH_FAIL;
    }
}

static void field_map_add_point(void)
{
    float forward_cm;
    float right_cm;
    uint32_t index;

    if(field_map_capture_allowed() == 0u)
    {
        return;
    }
    if(field_map_session_active == 0u)
    {
        field_map_status = FIELD_MAP_STATUS_NEED_START;
        return;
    }
    if(field_map_record.point_count >= VISION_FIELD_MAP_MAX_POINTS)
    {
        field_map_status = FIELD_MAP_STATUS_FULL;
        return;
    }

    field_map_current_local(&forward_cm, &right_cm);
    if((field_map_float_valid(forward_cm) == 0u) ||
       (field_map_float_valid(right_cm) == 0u))
    {
        field_map_status = FIELD_MAP_STATUS_COORD_BAD;
        return;
    }

    index = field_map_record.point_count;
    field_map_record.point_forward_cm[index] = forward_cm;
    field_map_record.point_right_cm[index] = right_cm;
    field_map_record.point_count++;
    field_map_record.finalized = 0u;
    field_map_valid = 0u;
    field_map_calculate_center(&field_map_record);

    if(field_map_flash_save() != 0u)
    {
        field_map_status = FIELD_MAP_STATUS_POINT_SAVED;
    }
    else
    {
        field_map_status = FIELD_MAP_STATUS_FLASH_FAIL;
    }
}

static void field_map_undo_point(void)
{
    uint32_t index;

    if(vehicle_state.armed != 0u)
    {
        field_map_status = FIELD_MAP_STATUS_ARMED_LOCK;
        return;
    }
    if((field_map_session_active == 0u) ||
       (field_map_record.point_count == 0u))
    {
        field_map_status = FIELD_MAP_STATUS_NEED_START;
        return;
    }

    field_map_record.point_count--;
    index = field_map_record.point_count;
    field_map_record.point_forward_cm[index] = 0.0f;
    field_map_record.point_right_cm[index] = 0.0f;
    field_map_record.finalized = 0u;
    field_map_valid = 0u;
    field_map_calculate_center(&field_map_record);

    if(field_map_flash_save() != 0u)
    {
        field_map_status = FIELD_MAP_STATUS_POINT_UNDONE;
    }
    else
    {
        field_map_status = FIELD_MAP_STATUS_FLASH_FAIL;
    }
}

static void field_map_finalize(void)
{
    if(vehicle_state.armed != 0u)
    {
        field_map_status = FIELD_MAP_STATUS_ARMED_LOCK;
        return;
    }
    if(field_map_session_active == 0u)
    {
        field_map_status = FIELD_MAP_STATUS_NEED_START;
        return;
    }
    if(field_map_record.point_count < FIELD_MAP_MIN_BEACON_POINTS)
    {
        field_map_status = FIELD_MAP_STATUS_NEED_POINTS;
        return;
    }

    field_map_calculate_center(&field_map_record);
    field_map_record.finalized = 1u;
    if(field_map_flash_save() != 0u)
    {
        field_map_valid = 1u;
        field_map_session_active = 0u;
        field_map_status = FIELD_MAP_STATUS_READY;
    }
    else
    {
        field_map_record.finalized = 0u;
        field_map_valid = 0u;
        field_map_status = FIELD_MAP_STATUS_FLASH_FAIL;
    }
}

static int16_t field_map_round_i16(float value)
{
    if(value > 32767.0f) return 32767;
    if(value < -32768.0f) return -32768;
    return (int16_t)((value >= 0.0f) ? (value + 0.5f) : (value - 0.5f));
}

static void field_map_publish_display(void)
{
    VisionFieldMapDisplay_t display;
    float current_forward = 0.0f;
    float current_right = 0.0f;

    memset(&display, 0, sizeof(display));
    field_map_current_local(&current_forward, &current_right);
    display.display_active =
        ((FIELD_MAP_CALIBRATION_ENABLE != 0u) &&
         (vehicle_state.armed == 0u)) ? 1u : 0u;
    display.map_valid = field_map_valid;
    display.flash_valid = field_map_flash_valid;
    display.session_active = field_map_session_active;
    display.point_count = (uint8_t)field_map_record.point_count;
    display.status = (uint8_t)field_map_status;
    display.current_forward_cm = field_map_round_i16(current_forward);
    display.current_right_cm = field_map_round_i16(current_right);
    display.center_forward_cm =
        field_map_round_i16(field_map_record.center_forward_cm);
    display.center_right_cm =
        field_map_round_i16(field_map_record.center_right_cm);

    for(uint32_t index = 0u; index < VISION_FIELD_MAP_MAX_POINTS; index++)
    {
        display.point_forward_cm[index] =
            field_map_round_i16(field_map_record.point_forward_cm[index]);
        display.point_right_cm[index] =
            field_map_round_i16(field_map_record.point_right_cm[index]);
    }

    field_map_display_generation++;
    __DMB();
    field_map_display_snapshot = display;
    __DMB();
    field_map_display_generation++;
}

static void field_map_update_run_origin(void)
{
    uint8_t armed = (vehicle_state.armed != 0u) ? 1u : 0u;

    if((armed != 0u) && (field_map_last_armed == 0u))
    {
        if(field_map_pose_valid() != 0u)
        {
            float yaw_rad = vehicle_state.current_yaw * FIELD_MAP_RAD_PER_DEG;
            field_map_run_origin_x_cm = vehicle_state.current_pos_x;
            field_map_run_origin_y_cm = vehicle_state.current_pos_y;
            field_map_run_origin_yaw_sin = sinf(yaw_rad);
            field_map_run_origin_yaw_cos = cosf(yaw_rad);
            field_map_run_origin_valid = 1u;
        }
        else
        {
            field_map_run_origin_valid = 0u;
        }
    }
    else if((armed == 0u) && (field_map_last_armed != 0u))
    {
        field_map_run_origin_valid = 0u;
    }
    field_map_last_armed = armed;
}

void field_map_init(void)
{
    field_map_session_active = 0u;
    field_map_run_origin_valid = 0u;
    field_map_last_armed = (vehicle_state.armed != 0u) ? 1u : 0u;
    field_map_key1_long_latched = 0u;
    field_map_key4_long_latched = 0u;
    memset(&field_map_display_snapshot, 0,
           sizeof(field_map_display_snapshot));

    if(FIELD_MAP_CALIBRATION_ENABLE == 0u)
    {
        memset(&field_map_record, 0, sizeof(field_map_record));
        field_map_flash_valid = 0u;
        field_map_valid = 0u;
        field_map_status = FIELD_MAP_STATUS_EMPTY;
        field_map_publish_display();
        return;
    }

    flash_init();
    key_init(FIELD_MAP_KEY_SCAN_PERIOD_MS);
    key_clear_all_state();
    field_map_flash_load();
    field_map_publish_display();
}

void field_map_task(void)
{
    if(FIELD_MAP_CALIBRATION_ENABLE == 0u)
    {
        return;
    }

    field_map_update_run_origin();
    key_scanner();

    if(key_get_state(KEY_1) == KEY_RELEASE)
    {
        field_map_key1_long_latched = 0u;
    }
    if(key_get_state(KEY_4) == KEY_RELEASE)
    {
        field_map_key4_long_latched = 0u;
    }

    if(vehicle_state.armed != 0u)
    {
        /* A key held during flight must be released after disarming before
         * it can start or finalize a calibration session. */
        if(key_get_state(KEY_1) != KEY_RELEASE)
            field_map_key1_long_latched = 1u;
        if(key_get_state(KEY_4) != KEY_RELEASE)
            field_map_key4_long_latched = 1u;
        key_clear_all_state();
        field_map_status = FIELD_MAP_STATUS_ARMED_LOCK;
        field_map_publish_display();
        return;
    }

    if((key_get_state(KEY_1) == KEY_LONG_PRESS) &&
       (field_map_key1_long_latched == 0u))
    {
        field_map_key1_long_latched = 1u;
        key_clear_state(KEY_1);
        field_map_begin_session();
    }
    else if(key_get_state(KEY_1) == KEY_SHORT_PRESS)
    {
        key_clear_state(KEY_1);
        field_map_status = FIELD_MAP_STATUS_NEED_START;
    }

    if(key_get_state(KEY_2) == KEY_SHORT_PRESS)
    {
        key_clear_state(KEY_2);
        field_map_add_point();
    }

    if(key_get_state(KEY_3) == KEY_SHORT_PRESS)
    {
        key_clear_state(KEY_3);
        field_map_undo_point();
    }

    if((key_get_state(KEY_4) == KEY_LONG_PRESS) &&
       (field_map_key4_long_latched == 0u))
    {
        field_map_key4_long_latched = 1u;
        key_clear_state(KEY_4);
        field_map_finalize();
    }
    else if(key_get_state(KEY_4) == KEY_SHORT_PRESS)
    {
        key_clear_state(KEY_4);
    }

    field_map_publish_display();
}

uint8_t field_map_get_display(VisionFieldMapDisplay_t *display)
{
    uint32_t generation_before;
    uint32_t generation_after;

    if(display == NULL)
    {
        return 0u;
    }

    generation_before = field_map_display_generation;
    if((generation_before & 1u) != 0u)
    {
        return 0u;
    }
    __DMB();
    *display = field_map_display_snapshot;
    __DMB();
    generation_after = field_map_display_generation;
    return ((generation_before == generation_after) &&
            ((generation_after & 1u) == 0u)) ? 1u : 0u;
}

static uint8_t field_map_local_to_run_earth(float forward_cm,
                                             float right_cm,
                                             float *x_cm, float *y_cm)
{
    if((x_cm == NULL) || (y_cm == NULL) ||
       (field_map_valid == 0u) ||
       (field_map_run_origin_valid == 0u) ||
       (vehicle_state.armed == 0u))
    {
        return 0u;
    }

    *x_cm = field_map_run_origin_x_cm +
            forward_cm * field_map_run_origin_yaw_cos -
            right_cm * field_map_run_origin_yaw_sin;
    *y_cm = field_map_run_origin_y_cm +
            forward_cm * field_map_run_origin_yaw_sin +
            right_cm * field_map_run_origin_yaw_cos;
    return 1u;
}

uint8_t field_map_get_search_center_earth(float *x_cm, float *y_cm)
{
    if(FIELD_MAP_CALIBRATION_ENABLE == 0u)
    {
        return 0u;
    }
    return field_map_local_to_run_earth(
        field_map_record.center_forward_cm,
        field_map_record.center_right_cm,
        x_cm, y_cm);
}

uint8_t field_map_get_beacon_point_earth(uint8_t index,
                                         float *x_cm, float *y_cm)
{
    if(FIELD_MAP_CALIBRATION_ENABLE == 0u)
    {
        return 0u;
    }
    if(index >= field_map_record.point_count)
    {
        return 0u;
    }
    return field_map_local_to_run_earth(
        field_map_record.point_forward_cm[index],
        field_map_record.point_right_cm[index],
        x_cm, y_cm);
}

uint8_t field_map_get_beacon_point_count(void)
{
    if(FIELD_MAP_CALIBRATION_ENABLE == 0u)
    {
        return 0u;
    }
    return (field_map_valid != 0u) ?
        (uint8_t)field_map_record.point_count : 0u;
}

void pid_menu_init(void)
{
    field_map_init();
}

void pid_menu_task(void)
{
    field_map_task();
}
