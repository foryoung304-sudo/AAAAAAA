#include "zf_common_headfile.h"

/**
 * @brief 解析遥控器原始数据
 * @param dT_s 时间间隔(秒)
 * @note 此函数现在只负责将遥控器原始杆量数据，转换为标准化的、带死区的物理期望值（角度、角速度、爬升率）。
 *       它不再关心飞行模式，飞行模式的管理和期望值生成由 param.c 中的 param_update 统一处理。
 */
void prase_remote_ctrl_data(float dT_s)
{
    (void)dT_s;

    static uint8_t arm_switch_seen_locked = 0u;
    static uint8_t last_arm_switch = 0u;

    if(!lora3a22_state_flag)
    {
        /* Autonomous competition flight must survive a receiver dropout.
         * Never alter armed state here; zero only manual input and leave the
         * active autonomous/hold controller in ownership of the aircraft. */
        if(vehicle_state.armed != 0u)
        {
#if RADIO_LOSS_AUTOLAND_ENABLE
            auto_landing_request = 1u;
#endif
        }
        else
        {
            auto_landing_request = 0u;
            auto_landing_active = 0u;
            preflight_error_flags = 0u;
        }
        arm_switch_seen_locked = 0u;
        last_arm_switch = 0u;
        manual_input.roll = 0.0f;
        manual_input.pitch = 0.0f;
        manual_input.yaw_rate = 0.0f;
        manual_input.climb_rate = 0.0f;
        manual_input.test_duty = 0.0f;
        return;
    }

    uint8_t arm_switch = lora3a22_uart_transfer.switch_key[1];

    if(arm_switch == 0u)
    {
#if (TEST_MODE_ENABLE && TRIPOD_MODE_ENABLE)
        vehicle_state.armed = 0u;
        auto_landing_request = 0u;
        auto_landing_active = 0u;
#else
        if(vehicle_state.armed != 0u)
        {
            /* The lock switch is a land request in every armed state.  The
             * guarded auto-land path decides when it is actually safe to
             * disarm; never use this switch as an in-air motor kill. */
            auto_landing_request = 1u;
        }
        else
        {
            vehicle_state.armed = 0;
            auto_landing_request = 0u;
            auto_landing_active = 0u;
        }
#endif
        arm_switch_seen_locked = 1u;
    }
    else if((arm_switch_seen_locked != 0u) && (last_arm_switch == 0u))
    {
        /* A 0->1 edge while still armed is an in-flight cancellation of a
         * switch/radio initiated auto-land, not a new arm attempt.  Running
         * preflight here is guaranteed to fail because the preflight Flow
         * ready timer is intentionally zero while armed; the old failure
         * branch then cut all motors in the air.  Sensor-failsafe landings
         * remain latched, but even they must never clear armed here. */
        if(vehicle_state.armed != 0u)
        {
            if(flight_sensor_failsafe_flags == 0u)
            {
                auto_landing_request = 0u;
                auto_landing_active = 0u;
            }
        }
        else if(preflight_check() != 0u)
        {
            vehicle_state.armed = 1;
            auto_landing_request = 0u;
            auto_landing_active = 0u;
        }
        else
        {

            vehicle_state.armed = 0u;
            auto_landing_request = 0u;
            auto_landing_active = 0u;
        }
    }


    last_arm_switch = arm_switch;
    
    // joystick[0]:左边左右(Yaw)   joystick[1]:左边上下(Throttle)
    // joystick[2]:右边左右(Roll)  joystick[3]:右边上下(Pitch)
    float stick_yaw  = (float)lora3a22_uart_transfer.joystick[0];
    float stick_z    = (float)lora3a22_uart_transfer.joystick[1];
    float stick_roll = (float)lora3a22_uart_transfer.joystick[2];
    float stick_pit  = (float)lora3a22_uart_transfer.joystick[3];

    
    // 摇杆死区 (发送端已处理，此处注释掉以保留微小打杆的精度。若发现微弱漂移可将阈值设为 5~10 兜底)
    // if(fabsf(stick_yaw) < 50.0f)  stick_yaw = 0.0f;
    // if(fabsf(stick_z) < 50.0f)    stick_z = 0.0f;
    // if(fabsf(stick_roll) < 50.0f) stick_roll = 0.0f;
    // if(fabsf(stick_pit) < 50.0f)  stick_pit = 0.0f;
 
    // 归一化并映射到物理量
    float max_val   = (float)JOYSTICK_MAX_VALUE;
    manual_input.climb_rate = stick_z / max_val * MAX_HEIGHT;
    manual_input.roll       = -stick_roll / max_val * MAX_ROLL_PITCH; // 摇杆左为正，需加负号映射为左倾(负横滚)
    manual_input.pitch      = stick_pit / max_val * MAX_ROLL_PITCH;
    manual_input.yaw_rate   = -stick_yaw / max_val * MAX_YAW_RATE;    // 偏航通常需加负号符合右手定则
   
}
