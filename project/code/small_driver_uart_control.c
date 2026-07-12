#include "small_driver_uart_control.h"

small_device_value_struct motor_value;      // 定义通讯参数结构体


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     无刷驱动 串口接收回调函数
// 参数说明     void
// 返回参数     void
// 使用示例     uart_control_callback(1000, -1000);
// 备注信息     用于解析接收到的速度数据  该函数需要在对应的串口接收中断中调用
//-------------------------------------------------------------------------------------------------------------------
void uart_control_callback(void)
{
    uint8 receive_data;                                                                     // 定义临时变量

    if(uart_query_byte(SMALL_DRIVER_UART, &receive_data))                                   // 接收串口数据
    {
        if(receive_data == 0xA5 && motor_value.receive_data_buffer[0] != 0xA5)              // 判断是否收到帧头 并且 当前接收内容中是否正确包含帧头
        {
            motor_value.receive_data_count = 0;                                             // 未收到帧头或者未正确包含帧头则重新接收
        }

        motor_value.receive_data_buffer[motor_value.receive_data_count ++] = receive_data;  // 保存串口数据

        if(motor_value.receive_data_count >= 11)                                            // 判断是否接收到指定数量的数据
        {
            if(motor_value.receive_data_buffer[0] == 0xA5)                                  // 判断帧头是否正确
            {
                motor_value.sum_check_data = 0;                                             // 清除校验位数据

                for(int i = 0; i < 10; i ++)
                {
                    motor_value.sum_check_data += motor_value.receive_data_buffer[i];       // 重新计算校验位
                }

                if(motor_value.sum_check_data == motor_value.receive_data_buffer[10])       // 校验数据准确性
                {
                    for(int i = 0; i < 4; i ++)
                    {
                        motor_value.receive_speed_data[i]   = (((int)motor_value.receive_data_buffer[i * 2 + 2] << 8) | (int)motor_value.receive_data_buffer[i * 2 + 3]);
                    }

                    motor_value.receive_data_count = 0;                                     // 清除缓冲区计数值

                    memset(motor_value.receive_data_buffer, 0, 11);                         // 清除缓冲区数据
                }
                else
                {
                    motor_value.receive_data_count = 0;                                     // 清除缓冲区计数值

                    memset(motor_value.receive_data_buffer, 0, 11);                         // 清除缓冲区数据
                }
            }
            else
            {
                motor_value.receive_data_count = 0;                                         // 清除缓冲区计数值

                memset(motor_value.receive_data_buffer, 0, 11);                             // 清除缓冲区数据
            }
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     无刷驱动 设置电机占空比
// 参数说明     left_duty       左侧电机占空比  范围 -10000 ~ 10000  负数为反转
// 参数说明     right_duty      右侧电机占空比  范围 -10000 ~ 10000  负数为反转
// 返回参数     void
// 使用示例     small_driver_set_duty(1000, -1000, 1000, -1000);
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void small_driver_set_duty(int16 motor_duty_1, int16 motor_duty_2, int16 motor_duty_3, int16 motor_duty_4)
{
    motor_value.send_data_buffer[0] = 0xA5;                                         // 配置帧头

    motor_value.send_data_buffer[1] = 0X01;                                         // 配置功能字

    motor_value.send_data_buffer[2] = (uint8)((motor_duty_1 & 0xFF00) >> 8);        // 拆分 电机1占空比 的高八位

    motor_value.send_data_buffer[3] = (uint8)(motor_duty_1 & 0x00FF);               // 拆分 电机1占空比 的低八位

    motor_value.send_data_buffer[4] = (uint8)((motor_duty_2 & 0xFF00) >> 8);        // 拆分 电机2占空比 的高八位

    motor_value.send_data_buffer[5] = (uint8)(motor_duty_2 & 0x00FF);               // 拆分 电机2占空比 的低八位

    motor_value.send_data_buffer[6] = (uint8)((motor_duty_3 & 0xFF00) >> 8);        // 拆分 电机3占空比 的高八位

    motor_value.send_data_buffer[7] = (uint8)(motor_duty_3 & 0x00FF);               // 拆分 电机3占空比 的低八位

    motor_value.send_data_buffer[8] = (uint8)((motor_duty_4 & 0xFF00) >> 8);        // 拆分 电机4占空比 的高八位

    motor_value.send_data_buffer[9] = (uint8)(motor_duty_4 & 0x00FF);               // 拆分 电机4占空比 的低八位

    motor_value.send_data_buffer[10] = 0;                                           // 和校验清除

    for(int i = 0; i < 10; i ++)
    {
        motor_value.send_data_buffer[10] += motor_value.send_data_buffer[i];        // 计算校验位
    }

    uart_write_buffer(SMALL_DRIVER_UART, motor_value.send_data_buffer, 11);                     // 发送设置占空比的 字节包 数据
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     无刷驱动 获取速度信息
// 参数说明     void
// 返回参数     void
// 使用示例     small_driver_get_speed();
// 备注信息     仅需发送一次 驱动将周期发出速度信息(默认10ms)
//-------------------------------------------------------------------------------------------------------------------
void small_driver_get_speed(void)
{
    motor_value.send_data_buffer[0] = 0xA5;                                         // 配置帧头

    motor_value.send_data_buffer[1] = 0X02;                                         // 配置功能字

    motor_value.send_data_buffer[2] = 0x00;                                         // 数据位清空

    motor_value.send_data_buffer[3] = 0x00;                                         // 数据位清空

    motor_value.send_data_buffer[4] = 0x00;                                         // 数据位清空

    motor_value.send_data_buffer[5] = 0x00;                                         // 数据位清空

    motor_value.send_data_buffer[6] = 0x00;                                         // 数据位清空

    motor_value.send_data_buffer[7] = 0x00;                                         // 数据位清空

    motor_value.send_data_buffer[8] = 0x00;                                         // 数据位清空

    motor_value.send_data_buffer[9] = 0x00;                                         // 数据位清空

    motor_value.send_data_buffer[10] = 0xA7;                                        // 配置校验位

    uart_write_buffer(SMALL_DRIVER_UART, motor_value.send_data_buffer, 11);         // 发送获取转速数据的 字节包 数据
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     无刷驱动 参数初始化
// 参数说明     void
// 返回参数     void
// 使用示例     small_driver_init();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void small_driver_init(void)
{
    memset(motor_value.send_data_buffer, 0, 11);                             // 清除缓冲区数据

    memset(motor_value.receive_data_buffer, 0, 11);                          // 清除缓冲区数据

    memset(motor_value.receive_speed_data, 0, 8);                            // 清除接收数据

    motor_value.receive_data_count          = 0;

    motor_value.sum_check_data              = 0;
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     无刷驱动 串口通讯初始化
// 参数说明     void
// 返回参数     void
// 使用示例     small_driver_uart_init();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void small_driver_uart_init(void)
{
    uart_init(SMALL_DRIVER_UART, SMALL_DRIVER_BAUDRATE, SMALL_DRIVER_RX, SMALL_DRIVER_TX);      // 串口初始化

    uart_rx_interrupt(SMALL_DRIVER_UART, 1);                                                    // 使能串口接收中断

    small_driver_init();                                                                        // 结构体参数初始化

    small_driver_set_duty(0, 0, 0, 0);                                                          // 设置0占空比

    //small_driver_get_speed();                                                                   // 获取实时速度数据
}
/*
// ==================================================================================================================
//                                                新增自定义安全函数组
// ==================================================================================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     [自定义] 严格校验的串口接收回调函数
// 备注信息     替代原 uart_control_callback。加入了强校验与正确的溢出计算
//-------------------------------------------------------------------------------------------------------------------
void small_driver_custom_rx_callback(void)
{
    uint8 receive_data;
    while(uart_query_byte(SMALL_DRIVER_UART, &receive_data))
    {
        if(motor_value.receive_data_count == 0)
        {
            if(receive_data == 0xA5) // 必须是合法的帧头才开始接收
            {
                motor_value.receive_data_buffer[0] = receive_data;
                motor_value.receive_data_count = 1;
            }
        }
        else
        {
            motor_value.receive_data_buffer[motor_value.receive_data_count ++] = receive_data;
            
            if(motor_value.receive_data_count >= 11) // 满11字节开始校验
            {
                uint8 sum_check = 0; // 利用uint8溢出截断特性
                for(int i = 0; i < 10; i ++)
                {
                    sum_check += motor_value.receive_data_buffer[i];
                }
                
                // 校验和比对一致，且是返回速度的命令码 0x02
                if(sum_check == motor_value.receive_data_buffer[10]) 
                {
                    if(motor_value.receive_data_buffer[1] == 0x02) 
                    {
                        for(int i = 0; i < 4; i ++)
                        {
                            motor_value.receive_speed_data[i] = (int16)(((uint16)motor_value.receive_data_buffer[i * 2 + 2] << 8) | motor_value.receive_data_buffer[i * 2 + 3]);
                        }
                    }
                }
                motor_value.receive_data_count = 0; // 准备接下一帧
            }
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     [自定义] 规范拆包的占空比发送
//-------------------------------------------------------------------------------------------------------------------
void small_driver_custom_set_duty(int16 m1, int16 m2, int16 m3, int16 m4)
{
    uint8 sum_check = 0;
    motor_value.send_data_buffer[0] = 0xA5;
    motor_value.send_data_buffer[1] = 0x01;
    motor_value.send_data_buffer[2] = (uint8)((m1 >> 8) & 0xFF);
    motor_value.send_data_buffer[3] = (uint8)(m1 & 0xFF);
    motor_value.send_data_buffer[4] = (uint8)((m2 >> 8) & 0xFF);
    motor_value.send_data_buffer[5] = (uint8)(m2 & 0xFF);
    motor_value.send_data_buffer[6] = (uint8)((m3 >> 8) & 0xFF);
    motor_value.send_data_buffer[7] = (uint8)(m3 & 0xFF);
    motor_value.send_data_buffer[8] = (uint8)((m4 >> 8) & 0xFF);
    motor_value.send_data_buffer[9] = (uint8)(m4 & 0xFF);
    
    for(int i = 0; i < 10; i++) sum_check += motor_value.send_data_buffer[i];
    motor_value.send_data_buffer[10] = sum_check;
    
    uart_write_buffer(SMALL_DRIVER_UART, motor_value.send_data_buffer, 11);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     [自定义] 查转速命令 (0x02)
//-------------------------------------------------------------------------------------------------------------------
void small_driver_custom_get_speed(void)
{
    uint8 sum_check = 0;
    motor_value.send_data_buffer[0] = 0xA5;
    motor_value.send_data_buffer[1] = 0x02;
    for(int i = 2; i < 10; i++) motor_value.send_data_buffer[i] = 0x00;
    for(int i = 0; i < 10; i++) sum_check += motor_value.send_data_buffer[i];
    motor_value.send_data_buffer[10] = sum_check;
    uart_write_buffer(SMALL_DRIVER_UART, motor_value.send_data_buffer, 11);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     [自定义] 紧急停止命令 (0x03)
//-------------------------------------------------------------------------------------------------------------------
void small_driver_custom_set_zero(void)
{
    uint8 sum_check = 0;
    motor_value.send_data_buffer[0] = 0xA5;
    motor_value.send_data_buffer[1] = 0x03;
    for(int i = 2; i < 10; i++) motor_value.send_data_buffer[i] = 0x00;
    for(int i = 0; i < 10; i++) sum_check += motor_value.send_data_buffer[i];
    motor_value.send_data_buffer[10] = sum_check;
    uart_write_buffer(SMALL_DRIVER_UART, motor_value.send_data_buffer, 11);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     [自定义] 9:1交替调度策略
//-------------------------------------------------------------------------------------------------------------------
void small_driver_custom_update(int16 m1, int16 m2, int16 m3, int16 m4)
{
    static uint8_t schedule_cnt = 0;
    schedule_cnt++;
    if(schedule_cnt >= 10) {
        small_driver_custom_get_speed();
        schedule_cnt = 0;
    } 
    else
    {
        small_driver_custom_set_duty(m1, m2, m3, m4);
    }
}*/
