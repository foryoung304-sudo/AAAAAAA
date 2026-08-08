#include "zf_common_headfile.h"
#include "vision_shared.h"

#define VISION_SHARED_MAGIC        0x56534E31u
#define VISION_SHARED_SLOT_SIZE    160u
#define VISION_SHARED_READ_RETRIES 3u
#define VISION_ATTITUDE_MAGIC      0x56415431u

typedef struct {
    uint32_t magic;
    uint32_t generation;
    uint32_t active_slot;
    uint32_t reserved[5];
    uint8_t slot[2][VISION_SHARED_SLOT_SIZE];
} VisionSharedMailbox_t;

#pragma data_alignment = 32
#pragma location = 0x28001000
__no_init static volatile VisionSharedMailbox_t vision_shared_mailbox;

typedef struct {
    uint32_t magic;
    uint32_t generation;
    VisionAttitudeSample_t sample;
} VisionAttitudeMailbox_t;

#pragma data_alignment = 32
#pragma location = 0x28001200
__no_init static volatile VisionAttitudeMailbox_t vision_attitude_mailbox;

typedef char vision_snapshot_fits_shared_slot[
    (sizeof(VisionDetectionSnapshot_t) <= VISION_SHARED_SLOT_SIZE) ? 1 : -1];
typedef char vision_attitude_mailbox_stays_in_reserved_window[
    (sizeof(VisionAttitudeMailbox_t) <= 256u) ? 1 : -1];

static void vision_shared_copy_to_volatile(volatile uint8_t *dst,
                                           const uint8_t *src,
                                           uint32_t length)
{
    for(uint32_t i = 0u; i < length; i++)
    {
        dst[i] = src[i];
    }
}

static void vision_shared_copy_from_volatile(uint8_t *dst,
                                             const volatile uint8_t *src,
                                             uint32_t length)
{
    for(uint32_t i = 0u; i < length; i++)
    {
        dst[i] = src[i];
    }
}

void vision_shared_writer_init(void)
{
    vision_shared_mailbox.magic = VISION_SHARED_MAGIC;
    vision_shared_mailbox.generation = 0u;
    vision_shared_mailbox.active_slot = 0u;

    for(uint32_t slot = 0u; slot < 2u; slot++)
    {
        for(uint32_t i = 0u; i < VISION_SHARED_SLOT_SIZE; i++)
        {
            vision_shared_mailbox.slot[slot][i] = 0u;
        }
    }

    __DMB();
    SCB_CleanInvalidateDCache_by_Addr((void *)&vision_shared_mailbox,
                                      sizeof(vision_shared_mailbox));
}

__vfp void vision_attitude_shared_publish(uint32_t timestamp_us,
                                    float roll_deg, float pitch_deg,
                                    float yaw_deg, float height_cm,
                                    int16_t car_tx_err_forward_px,
                                    int16_t car_tx_err_right_px,
                                    uint8_t flight_mode,
                                    uint8_t aircraft_armed,
                                    const VisionFieldMapDisplay_t *field_map)
{
    uint32_t generation = vision_attitude_mailbox.generation + 1u;

    vision_attitude_mailbox.magic = VISION_ATTITUDE_MAGIC;
    vision_attitude_mailbox.sample.generation = generation;
    vision_attitude_mailbox.sample.timestamp_us = timestamp_us;
    vision_attitude_mailbox.sample.roll_deg = roll_deg;
    vision_attitude_mailbox.sample.pitch_deg = pitch_deg;
    vision_attitude_mailbox.sample.yaw_deg = yaw_deg;
    vision_attitude_mailbox.sample.height_cm = height_cm;
    vision_attitude_mailbox.sample.car_tx_err_forward_px =
        car_tx_err_forward_px;
    vision_attitude_mailbox.sample.car_tx_err_right_px =
        car_tx_err_right_px;
    vision_attitude_mailbox.sample.flight_mode = flight_mode;
    vision_attitude_mailbox.sample.aircraft_armed = aircraft_armed;
    vision_attitude_mailbox.sample.reserved[0] = 0u;
    vision_attitude_mailbox.sample.reserved[1] = 0u;
    if(field_map != NULL)
    {
        vision_attitude_mailbox.sample.field_map = *field_map;
    }
    else
    {
        memset((void *)&vision_attitude_mailbox.sample.field_map, 0,
               sizeof(vision_attitude_mailbox.sample.field_map));
    }
    __DMB();
    vision_attitude_mailbox.generation = generation;
    SCB_CleanDCache_by_Addr((void *)&vision_attitude_mailbox,
                            sizeof(vision_attitude_mailbox));
}

uint8_t vision_attitude_shared_read(VisionAttitudeSample_t *sample)
{
    uint32_t before;
    uint32_t after;

    if(sample == NULL) return 0u;
    SCB_InvalidateDCache_by_Addr((void *)&vision_attitude_mailbox,
                                 sizeof(vision_attitude_mailbox));
    __DMB();
    if(vision_attitude_mailbox.magic != VISION_ATTITUDE_MAGIC) return 0u;
    before = vision_attitude_mailbox.generation;
    *sample = vision_attitude_mailbox.sample;
    __DMB();
    after = vision_attitude_mailbox.generation;
    return (before == after && before == sample->generation) ? 1u : 0u;
}

void vision_shared_publish(const VisionDetectionSnapshot_t *snapshot)
{
    uint32_t next_slot;

    if(snapshot == NULL)
    {
        return;
    }

    next_slot = (vision_shared_mailbox.active_slot ^ 1u) & 1u;
    vision_shared_copy_to_volatile(vision_shared_mailbox.slot[next_slot],
                                   (const uint8_t *)snapshot,
                                   sizeof(*snapshot));
    __DMB();
    SCB_CleanInvalidateDCache_by_Addr(
        (void *)&vision_shared_mailbox.slot[next_slot][0],
        VISION_SHARED_SLOT_SIZE);

    vision_shared_mailbox.active_slot = next_slot;
    vision_shared_mailbox.generation++;
    vision_shared_mailbox.magic = VISION_SHARED_MAGIC;
    __DMB();
    SCB_CleanInvalidateDCache_by_Addr((void *)&vision_shared_mailbox, 32u);
}

uint8_t vision_shared_read(VisionDetectionSnapshot_t *snapshot)
{
    if(snapshot == NULL)
    {
        return 0u;
    }

    for(uint8_t retry = 0u; retry < VISION_SHARED_READ_RETRIES; retry++)
    {
        uint32_t generation_before;
        uint32_t generation_after;
        uint32_t slot_before;
        uint32_t slot_after;

        SCB_InvalidateDCache_by_Addr((void *)&vision_shared_mailbox, 32u);
        __DMB();
        if(vision_shared_mailbox.magic != VISION_SHARED_MAGIC)
        {
            return 0u;
        }

        generation_before = vision_shared_mailbox.generation;
        slot_before = vision_shared_mailbox.active_slot & 1u;
        SCB_InvalidateDCache_by_Addr(
            (void *)&vision_shared_mailbox.slot[slot_before][0],
            VISION_SHARED_SLOT_SIZE);
        __DMB();
        vision_shared_copy_from_volatile((uint8_t *)snapshot,
                                         vision_shared_mailbox.slot[slot_before],
                                         sizeof(*snapshot));

        SCB_InvalidateDCache_by_Addr((void *)&vision_shared_mailbox, 32u);
        __DMB();
        generation_after = vision_shared_mailbox.generation;
        slot_after = vision_shared_mailbox.active_slot & 1u;

        if(generation_before == generation_after && slot_before == slot_after)
        {
            return 1u;
        }
    }

    return 0u;
}
