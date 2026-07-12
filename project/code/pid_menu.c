#include "zf_common_headfile.h"

typedef struct {
    const char *name;
    pid_param_t *pid;
} pid_menu_item_t;

typedef struct {
    uint32_t magic;
    uint32_t version;
    uint32_t sequence;
    float pid[12][3];
    uint32_t checksum;
} pid_flash_data_t;

static pid_menu_item_t pid_items[] = {
    {"ANG R",  &att_ctrl.angle_pid[0]},
    {"ANG P",  &att_ctrl.angle_pid[1]},
    {"ANG Y",  &att_ctrl.angle_pid[2]},
    {"RATE R", &att_ctrl.rate_pid[0]},
    {"RATE P", &att_ctrl.rate_pid[1]},
    {"RATE Y", &att_ctrl.rate_pid[2]},
    {"ALT H",  &alt_ctrl.height_pid},
    {"ALT V",  &alt_ctrl.vel_pid},
    {"POS X",  &loc_ctrl.pos_pid[0]},
    {"POS Y",  &loc_ctrl.pos_pid[1]},
    {"VEL X",  &loc_ctrl.vel_pid[0]},
    {"VEL Y",  &loc_ctrl.vel_pid[1]},
};

static const float step_list[] = {0.0001f, 0.0005f, 0.001f, 0.01f, 0.1f, 1.0f};
static uint8_t item_index = 0;
static uint8_t param_index = 0;
static uint8_t step_index = 2;
static uint8_t refresh_flag = 1;
static uint8_t key1_long_latched = 0;
static uint8_t key3_long_latched = 0;
static uint8_t key4_long_latched = 0;
static uint32_t pid_save_sequence = 0;
static const char *save_status = "DFLT";

#define PID_MENU_ITEM_NUM   ((uint8_t)(sizeof(pid_items) / sizeof(pid_items[0])))
#define PID_MENU_STEP_NUM   ((uint8_t)(sizeof(step_list) / sizeof(step_list[0])))
#define PID_FLASH_PAGE      (95u)
#define PID_FLASH_MAGIC     (0x50494431u)
#define PID_FLASH_VERSION   (1u)

static uint32_t pid_flash_word_len(void)
{
    return (uint32_t)((sizeof(pid_flash_data_t) + sizeof(uint32_t) - 1u) / sizeof(uint32_t));
}

static uint32_t pid_flash_checksum(const pid_flash_data_t *data)
{
    const uint32_t *words = (const uint32_t *)data;
    uint32_t word_len = pid_flash_word_len() - 1u;
    uint32_t checksum = 0xA5A55A5Au;

    for(uint32_t i = 0; i < word_len; i++) {
        checksum ^= words[i] + 0x9E3779B9u + (checksum << 6) + (checksum >> 2);
    }

    return checksum;
}

static void pid_flash_collect(pid_flash_data_t *data)
{
    memset(data, 0, sizeof(pid_flash_data_t));
    data->magic = PID_FLASH_MAGIC;
    data->version = PID_FLASH_VERSION;
    data->sequence = pid_save_sequence + 1u;

    for(uint8_t i = 0; i < PID_MENU_ITEM_NUM; i++) {
        data->pid[i][0] = pid_items[i].pid->kp;
        data->pid[i][1] = pid_items[i].pid->ki;
        data->pid[i][2] = pid_items[i].pid->kd;
    }

    data->checksum = pid_flash_checksum(data);
}

static uint8_t pid_flash_valid(const pid_flash_data_t *data)
{
    if(PID_FLASH_MAGIC != data->magic) {
        return 0;
    }
    if(PID_FLASH_VERSION != data->version) {
        return 0;
    }
    if(pid_flash_checksum(data) != data->checksum) {
        return 0;
    }

    return 1;
}

static void pid_flash_apply(const pid_flash_data_t *data)
{
    for(uint8_t i = 0; i < PID_MENU_ITEM_NUM; i++) {
        pid_items[i].pid->kp = data->pid[i][0];
        pid_items[i].pid->ki = data->pid[i][1];
        pid_items[i].pid->kd = data->pid[i][2];
        stan_pid_reset(pid_items[i].pid);
    }

    pid_save_sequence = data->sequence;
}

static uint8_t pid_flash_load(void)
{
    pid_flash_data_t data;

    if(0 == flash_check(0, PID_FLASH_PAGE)) {
        save_status = "DFLT";
        return 0;
    }

    flash_read_page(0, PID_FLASH_PAGE, (uint32_t *)&data, pid_flash_word_len());
    if(0 == pid_flash_valid(&data)) {
        save_status = "BAD";
        return 0;
    }

    pid_flash_apply(&data);
    save_status = "LOAD";
    return 1;
}

static void pid_flash_save(void)
{
    pid_flash_data_t data;

    pid_flash_collect(&data);
    flash_write_page(0, PID_FLASH_PAGE, (const uint32_t *)&data, pid_flash_word_len());

    pid_save_sequence = data.sequence;
    save_status = "SAVE";
    refresh_flag = 1;
}

static float *pid_menu_current_value(void)
{
    pid_param_t *pid = pid_items[item_index].pid;

    if(0 == param_index) {
        return &pid->kp;
    } else if(1 == param_index) {
        return &pid->ki;
    } else {
        return &pid->kd;
    }
}

static const char *pid_menu_param_name(uint8_t index)
{
    if(0 == index) {
        return "KP";
    } else if(1 == index) {
        return "KI";
    } else {
        return "KD";
    }
}

static void pid_menu_adjust(float delta)
{
    float *value = pid_menu_current_value();

    *value += delta;
    if(*value < 0.0f) {
        *value = 0.0f;
    }

    stan_pid_reset(pid_items[item_index].pid);
    save_status = "SAVE*";
    refresh_flag = 1;
}

static void pid_menu_show_pid_row(uint16 y, const char *name, float value, uint8_t selected)
{
    if(selected) {
        ips114_set_color(RGB565_WHITE, RGB565_BLACK);
    } else {
        ips114_set_color(RGB565_BLACK, RGB565_WHITE);
    }

    ips114_show_string(0, y, "                ");
    ips114_show_string(0, y, name);
    ips114_show_float(32, y, value, 2, 5);
}

static void pid_menu_show(void)
{
    pid_param_t *pid = pid_items[item_index].pid;

    ips114_set_color(RGB565_BLACK, RGB565_WHITE);
    ips114_clear();
    ips114_show_string(0, 0, "PID MENU");
    ips114_show_string(88, 0, save_status);
    ips114_show_string(0, 16, "ITEM:");
    ips114_show_string(48, 16, pid_items[item_index].name);

    ips114_show_string(0, 32, "EDIT:");
    ips114_show_string(48, 32, pid_menu_param_name(param_index));
    ips114_show_string(0, 48, "STEP:");
    ips114_show_float(48, 48, step_list[step_index], 1, 5);

    pid_menu_show_pid_row(64, "KP:", pid->kp, (0 == param_index));
    pid_menu_show_pid_row(80, "KI:", pid->ki, (1 == param_index));
    pid_menu_show_pid_row(96, "KD:", pid->kd, (2 == param_index));

    ips114_set_color(RGB565_BLACK, RGB565_WHITE);
    ips114_show_string(0, 112, "K1L>N K3L<P K4");
}

void pid_menu_init(void)
{
    flash_init();
    pid_flash_load();
    key_init(100);
    refresh_flag = 1;
}

void pid_menu_task(void)
{
    key_scanner();

    if(KEY_RELEASE == key_get_state(KEY_1)) {
        key1_long_latched = 0;
    }
    if(KEY_RELEASE == key_get_state(KEY_3)) {
        key3_long_latched = 0;
    }
    if(KEY_RELEASE == key_get_state(KEY_4)) {
        key4_long_latched = 0;
    }

    if((KEY_LONG_PRESS == key_get_state(KEY_1)) && (0 == key1_long_latched)) {
        item_index++;
        if(PID_MENU_ITEM_NUM <= item_index) {
            item_index = 0;
        }
        key1_long_latched = 1;
        key_clear_state(KEY_1);
        refresh_flag = 1;
    } else if((KEY_SHORT_PRESS == key_get_state(KEY_1)) && (0 == key1_long_latched)) {
        param_index++;
        if(3 <= param_index) {
            param_index = 0;
        }
        key_clear_state(KEY_1);
        refresh_flag = 1;
    }

    if(KEY_SHORT_PRESS == key_get_state(KEY_2)) {
        pid_menu_adjust(step_list[step_index]);
        key_clear_state(KEY_2);
    }

    if((KEY_LONG_PRESS == key_get_state(KEY_3)) && (0 == key3_long_latched)) {
        if(0 == item_index) {
            item_index = PID_MENU_ITEM_NUM - 1u;
        } else {
            item_index--;
        }
        key3_long_latched = 1;
        key_clear_state(KEY_3);
        refresh_flag = 1;
    } else if((KEY_SHORT_PRESS == key_get_state(KEY_3)) && (0 == key3_long_latched)) {
        pid_menu_adjust(-step_list[step_index]);
        key_clear_state(KEY_3);
    }

    if((KEY_LONG_PRESS == key_get_state(KEY_4)) && (0 == key4_long_latched)) {
        pid_flash_save();
        key4_long_latched = 1;
        key_clear_state(KEY_4);
    } else if((KEY_SHORT_PRESS == key_get_state(KEY_4)) && (0 == key4_long_latched)) {
        step_index++;
        if(PID_MENU_STEP_NUM <= step_index) {
            step_index = 0;
        }
        key_clear_state(KEY_4);
        refresh_flag = 1;
    }

    if(refresh_flag) {
        pid_menu_show();
        refresh_flag = 0;
    }
}
