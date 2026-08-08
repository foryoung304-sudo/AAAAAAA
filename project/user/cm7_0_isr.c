/*********************************************************************************************************************
* CYT4BB Opensourec Library ���� CYT4BB ��Դ�⣩��һ�����ڹٷ� SDK �ӿڵĵ�������Դ��
* Copyright (c) 2022 SEEKFREE ��ɿƼ�
*
* ���ļ��� CYT4BB ��Դ���һ����
*
* CYT4BB ��Դ�� ���������
* �����Ը���������������ᷢ���� GPL��GNU General Public License���� GNUͨ�ù�������֤��������
* �� GPL �ĵ�3�棨�� GPL3.0������ѡ��ģ��κκ����İ汾�����·�����/���޸���
*
* ����Դ��ķ�����ϣ�����ܷ������ã�����δ�������κεı�֤
* ����û�������������Ի��ʺ��ض���;�ı�֤
* ����ϸ����μ� GPL
*
* ��Ӧ�����յ�����Դ���ͬʱ�յ�һ�� GPL �ĸ���
* ���û�У������<https://www.gnu.org/licenses/>
*
* ����ע����
* ����Դ��ʹ�� GPL3.0 ��Դ����֤Э�� ������������Ϊ���İ汾
* ��������Ӣ�İ��� libraries/doc �ļ����µ� GPL3_permission_statement.txt �ļ���
* ����֤������ libraries �ļ����� �����ļ����µ� LICENSE �ļ�
* ��ӭ��λʹ�ò����������� ���޸�����ʱ���뱣����ɿƼ��İ�Ȩ����������������
*
* �ļ�����          cm7_0_isr
* ��˾����          �ɶ���ɿƼ����޹�˾
* �汾��Ϣ          �鿴 libraries/doc �ļ����� version �ļ� �汾˵��
* ��������          IAR 9.40.1
* ����ƽ̨          CYT4BB
* ��������          https://seekfree.taobao.com/
*
* �޸ļ�¼
* ����              ����                ��ע
* 2024-1-9      pudding            first version
* 2024-5-14     pudding            ����12��pit�����ж� ���Ӳ���ע��˵��
* 2025-2-4      pudding            �Ż������ж��߼�����ֹ������ŵ��µĿ������⣬�Ż����ڲ����ʼ����߼�
* 2025-2-4      pudding            �����������ڽӿ�
********************************************************************************************************************/

#include "zf_common_headfile.h"

uint32_t isr_time_cost = 0; // ���ڼ�¼�ж�ʵ�ʺ�ʱ(us)
uint32_t loop_cnt=0;
volatile uint8 imu660rc_chip_id_debug = 0xFF;

// ����ʵ�����׳��������� main �����д�ӡ�鿴
float real_dt_2ms = 0.002f;
float real_dt_20ms = 0.02f;
uint32_t duty =3000; // ������ռ�ձȱ���

volatile uint32_t diag_pit_count = 0;
volatile uint32_t diag_dt2_max_us = 0;
volatile uint32_t diag_dt20_max_us = 0;
volatile uint32_t diag_isr_cost_max_us = 0;
volatile uint32_t diag_dt2_over_3ms = 0;
volatile uint32_t diag_dt2_over_5ms = 0;
volatile uint32_t diag_dt2_over_8ms = 0;
volatile uint32_t diag_dt20_over_30ms = 0;
volatile uint32_t diag_power_max_us = 0;
volatile uint32_t diag_height_max_us = 0;
volatile uint32_t diag_ekf_h_max_us = 0;
volatile uint32_t diag_flow_max_us = 0;
volatile uint32_t diag_ekf_xy_max_us = 0;
volatile uint32_t diag_param_max_us = 0;
volatile uint32_t diag_att2_max_us = 0;
volatile uint32_t diag_alt2_max_us = 0;
volatile uint32_t diag_alt1_max_us = 0;
volatile uint8_t height_update_request = 0;
volatile uint32_t height_update_drop_count = 0;
volatile uint8_t field_map_request = 0;
volatile uint8_t debug_print_request = 0;
volatile uint8_t diag_print_request = 0;

/* ALT V corruption probe.  The ISR only freezes the first observed write;
 * printing is deferred to the main loop so flight timing is unchanged. */
volatile uint8_t diag_pidwrite_pending = 0u;
volatile uint8_t diag_pidwrite_stage = 0u;
volatile uint32_t diag_pidwrite_time_us = 0u;
volatile uint32_t diag_pidwrite_before[3] = {0u, 0u, 0u};
volatile uint32_t diag_pidwrite_after[3] = {0u, 0u, 0u};
volatile uint8_t diag_pidwrite_rearm_request = 0u;

void diag_pidwrite_rearm(void)
{
    diag_pidwrite_pending = 0u;
    diag_pidwrite_rearm_request = 1u;
}

static void diag_pidwrite_read(uint32_t bits[3])
{
    memcpy(&bits[0], &alt_ctrl.vel_pid.kp, sizeof(uint32_t));
    memcpy(&bits[1], &alt_ctrl.vel_pid.ki, sizeof(uint32_t));
    memcpy(&bits[2], &alt_ctrl.vel_pid.kd, sizeof(uint32_t));
}

void diag_pidwrite_check(uint8_t stage)
{
    static uint8_t initialized = 0u;
    static uint8_t captured = 0u;
    static uint32_t previous[3] = {0u, 0u, 0u};
    uint32_t current[3];

    diag_pidwrite_read(current);
    if(diag_pidwrite_rearm_request != 0u)
    {
        diag_pidwrite_rearm_request = 0u;
        initialized = 1u;
        captured = 0u;
        previous[0] = current[0];
        previous[1] = current[1];
        previous[2] = current[2];
        return;
    }
    if(initialized == 0u)
    {
        initialized = 1u;
        previous[0] = current[0];
        previous[1] = current[1];
        previous[2] = current[2];
        return;
    }
    if((captured == 0u) &&
       ((current[0] != previous[0]) ||
        (current[1] != previous[1]) ||
        (current[2] != previous[2])))
    {
        diag_pidwrite_before[0] = previous[0];
        diag_pidwrite_before[1] = previous[1];
        diag_pidwrite_before[2] = previous[2];
        diag_pidwrite_after[0] = current[0];
        diag_pidwrite_after[1] = current[1];
        diag_pidwrite_after[2] = current[2];
        diag_pidwrite_stage = stage;
        diag_pidwrite_time_us = system_time_us();
        diag_pidwrite_pending = 1u;
        captured = 1u;
    }
    previous[0] = current[0];
    previous[1] = current[1];
    previous[2] = current[2];
}



// **************************** PIT�жϺ��� ****************************
void pit0_ch0_isr()                     // ��ʱ��ͨ�� 0 �����жϷ�����      
{
    uint32_t time_start = system_time_us(); // ��¼�����жϵ�ʱ��
    static uint32_t last_2ms_time = 0;
    static uint32_t last_20ms_time = 0;

    pit_isr_flag_clear(PIT_CH0);

    loop_cnt++;
    diag_pit_count++;
    diag_pidwrite_check(15u); /* PIT entry, before M-car UART polling */
    mcar_comm_poll();
    diag_pidwrite_check(1u);  /* after M-car UART polling */

    // ================= 2ms ���� =================
    if(last_2ms_time != 0)
    {
        uint32_t dt2_us = time_start - last_2ms_time;
        if(dt2_us > diag_dt2_max_us) diag_dt2_max_us = dt2_us;
        if(dt2_us > 3000) diag_dt2_over_3ms++;
        if(dt2_us > 5000) diag_dt2_over_5ms++;
        if(dt2_us > 8000) diag_dt2_over_8ms++;
    }
    real_dt_2ms = (time_start - last_2ms_time) / 1000000.0f;
    if(real_dt_2ms <= 0.0f || real_dt_2ms > 0.01f) real_dt_2ms = 0.002f; // �״����л��쳣��������
    last_2ms_time = time_start;

   

    imu_calc();
    diag_pidwrite_check(2u);
    //imu_log_sample_isr();


   flow_gyro_integrate(real_dt_2ms);
    ekf_lite_predict_height(real_dt_2ms);
    ekf_lite_predict_xy(real_dt_2ms);
    att_1level_ctrl(real_dt_2ms);     
    diag_pidwrite_check(3u);  /* flow + EKF predict + attitude rate */
     

    // ================= ң����ʧ�ر������ =================
    lora3a22_response_time += 2;           // ÿ���жϼ� 2ms
    if(lora3a22_response_time > 500)       // ������� 500ms û���յ��κ���Ч�� LoRa ���ݰ�
    {
        lora3a22_response_time = 500;      // ��ֹ�������
        lora3a22_state_flag = 0;           // ǿ�����㣺���� remote_ctrl.c �е�ʧ������Ͱ�ȫ�����߼�
    }

    motor_mixing_output();
    //diag_pidwrite_check(4u);
    //small_driver_set_duty(duty,duty,duty,duty);
    
    // =================  20ms ���� =================
    if(loop_cnt % 10 == 0) 
    {
           

        uint32_t current_20ms_time = system_time_us();
        static uint32_t main_20ms_cnt = 0;
        main_20ms_cnt++;
        /* The key driver is initialized for this exact 20 ms scan period. */
        debug_print_request = 1;
        field_map_request = 1;
        if(main_20ms_cnt >= 50)
        {
            main_20ms_cnt = 0;
            diag_print_request = 1;
        }

        if(last_20ms_time != 0)
        {
            uint32_t dt20_us = current_20ms_time - last_20ms_time;
            if(dt20_us > diag_dt20_max_us) diag_dt20_max_us = dt20_us;
            if(dt20_us > 30000) diag_dt20_over_30ms++;
        }
        real_dt_20ms = (current_20ms_time - last_20ms_time) / 1000000.0f;
        if(real_dt_20ms <= 0.0f || real_dt_20ms > 0.1f) real_dt_20ms = 0.02f; // �쳣��������
        last_20ms_time = current_20ms_time;

       uint32_t seg_start = system_time_us();
       system_power_update(real_dt_20ms);
       diag_pidwrite_check(5u);
       uint32_t seg_cost = system_time_us() - seg_start;
       if(seg_cost > diag_power_max_us) diag_power_max_us = seg_cost;

       seg_start = system_time_us();
       if(height_update_request == 0)
       {
           height_update_request = 1;
       }
       else
       {
           height_update_drop_count++;
       }
       seg_cost = system_time_us() - seg_start;
       if(seg_cost > diag_height_max_us) diag_height_max_us = seg_cost;

       seg_start = system_time_us();
       ekf_lite_update_height();
       diag_pidwrite_check(6u);
       seg_cost = system_time_us() - seg_start;
       if(seg_cost > diag_ekf_h_max_us) diag_ekf_h_max_us = seg_cost;

       seg_start = system_time_us();
       update_position_from_flow(real_dt_20ms); 
       diag_pidwrite_check(7u);
       seg_cost = system_time_us() - seg_start;
       if(seg_cost > diag_flow_max_us) diag_flow_max_us = seg_cost;
        
        seg_start = system_time_us();
        ekf_lite_update_xy(real_dt_20ms);
        diag_pidwrite_check(8u);
        seg_cost = system_time_us() - seg_start;
        if(seg_cost > diag_ekf_xy_max_us) diag_ekf_xy_max_us = seg_cost;

        seg_start = system_time_us();
        param_update(real_dt_20ms);        // ����ģʽ����������ָ������ (50Hz)
        diag_pidwrite_check(9u);
        seg_cost = system_time_us() - seg_start;
        if(seg_cost > diag_param_max_us) diag_param_max_us = seg_cost;
        
        // ================= ���ƻ�=================

        seg_start = system_time_us();
        alt_2level_ctrl(real_dt_20ms);
        diag_pidwrite_check(10u);
        seg_cost = system_time_us() - seg_start;
        if(seg_cost > diag_alt2_max_us) diag_alt2_max_us = seg_cost;

        loc_ctrl_update(real_dt_20ms);          
        diag_pidwrite_check(11u);

        seg_start = system_time_us();
        att_2level_ctrl(real_dt_20ms);     
        diag_pidwrite_check(12u);
        seg_cost = system_time_us() - seg_start;
        if(seg_cost > diag_att2_max_us) diag_att2_max_us = seg_cost;

        seg_start = system_time_us();
       alt_1level_ctrl(real_dt_20ms);
        diag_pidwrite_check(13u);
        seg_cost = system_time_us() - seg_start;
        if(seg_cost > diag_alt1_max_us) diag_alt1_max_us = seg_cost;

        // RAM only: no printf/UART activity while flying.
       debug_capture_states_20ms();
       diag_pidwrite_check(14u);
       //height_log_update_50hz();
      //loc_log_update_50hz();
 
   // λ��˫��
    }

      isr_time_cost = system_time_us() - time_start; // �����ܺ�ʱ
      if(isr_time_cost > diag_isr_cost_max_us) diag_isr_cost_max_us = isr_time_cost;

   // dl1b_int_handler();
    
}

void pit0_ch1_isr()                     // ��ʱ��ͨ�� 1 �����жϷ�����      
{
    pit_isr_flag_clear(PIT_CH1);
    
}

void pit0_ch2_isr()                     // ��ʱ��ͨ�� 2 �����жϷ�����      
{
    pit_isr_flag_clear(PIT_CH2);
    
}

void pit0_ch10_isr()                    // ��ʱ��ͨ�� 10 �����жϷ�����      
{
    pit_isr_flag_clear(PIT_CH10);
    
}

void pit0_ch11_isr()                    // ��ʱ��ͨ�� 11 �����жϷ�����      
{
    pit_isr_flag_clear(PIT_CH11);
    
}

void pit0_ch12_isr()                    // ��ʱ��ͨ�� 12 �����жϷ�����      
{
    pit_isr_flag_clear(PIT_CH12);
    
}

void pit0_ch13_isr()                    // ��ʱ��ͨ�� 13 �����жϷ�����      
{
    pit_isr_flag_clear(PIT_CH13);
    
}

void pit0_ch14_isr()                    // ��ʱ��ͨ�� 14 �����жϷ�����      
{
    pit_isr_flag_clear(PIT_CH14);
    
}

void pit0_ch15_isr()                    // ��ʱ��ͨ�� 15 �����жϷ�����      
{
    pit_isr_flag_clear(PIT_CH15);
    
}

void pit0_ch16_isr()                    // ��ʱ��ͨ�� 16 �����жϷ�����      
{
    pit_isr_flag_clear(PIT_CH16);
    
}

void pit0_ch17_isr()                    // ��ʱ��ͨ�� 17 �����жϷ�����      
{
    pit_isr_flag_clear(PIT_CH17);
    
}

void pit0_ch18_isr()                    // ��ʱ��ͨ�� 18 �����жϷ�����      
{
    pit_isr_flag_clear(PIT_CH18);
    
}

void pit0_ch19_isr()                    // ��ʱ��ͨ�� 19 �����жϷ�����      
{
    pit_isr_flag_clear(PIT_CH19);
    
}

void pit0_ch20_isr()                    // ��ʱ��ͨ�� 20 �����жϷ�����      
{
    pit_isr_flag_clear(PIT_CH20);
    
}

void pit0_ch21_isr()                    // ��ʱ��ͨ�� 21 �����жϷ�����      
{
    pit_isr_flag_clear(PIT_CH21);
    tsl1401_collect_pit_handler();
}
// **************************** PIT�жϺ��� ****************************


// **************************** �����жϺ��� ****************************
// ����0Ĭ����Ϊ���Դ���
void uart0_isr (void)
{
    diag_pidwrite_check(40u);
    if(uart_isr_mask(UART_0))            // ����0�����ж�
    {
        
#if DEBUG_UART_USE_INTERRUPT             // ������� debug �����ж�
        debug_interrupr_handler();       // ���� debug ���ڽ��մ������� ���ݻᱻ debug ���λ�������ȡ
#endif                                   // ����޸��� DEBUG_UART_INDEX ����δ�����Ҫ�ŵ���Ӧ�Ĵ����ж�ȥ
      
    }
    else                                 // ����0�����ж�
    {           
        
        
        
    }
    diag_pidwrite_check(41u);
}

void uart1_isr (void)
{
    diag_pidwrite_check(42u);
    if(uart_isr_mask(UART_1))            // ����1�����ж�
    {
        
       // wireless_module_uart_handler();  // ����ģ��ͳһ�ص�����
        lora3a22_uart_callback();          // LoRa3A22 ģ��ص�����
      
    }
    else                                // ����1�����ж�
    {
      
        
        
    }
    diag_pidwrite_check(43u);
}

void uart2_isr (void)
{
    diag_pidwrite_check(44u);
    if(uart_isr_mask(UART_2))            // ����2�����ж�
    {
        
        //gnss_uart_callback();            // GPSģ��ص�����      
        wireless_uart_callback();          // ����ģ��ص�����
    }
    else                                // ����2�����ж�
    {
        
        
       
    }
    diag_pidwrite_check(45u);
}

void uart3_isr (void)
{
    diag_pidwrite_check(46u);
    if(uart_isr_mask(UART_3))            // ����3�����ж�
    {
        
        lc302_uart_callback();
        
    }
    else                                // ����3�����ж�
    {
      
        
        
    }
    diag_pidwrite_check(47u);
}

void uart4_isr (void)
{
    diag_pidwrite_check(48u);
    if(uart_isr_mask(UART_4))            // ����4�����ж�
    {
        // ���UART4ֻ���˵����������������Ľ��ջص�����ֹ������
        // uart_receiver_handler();                                                                // ���ڽ��ջ��ص�����
       uart_control_callback();                                                                // ��ˢ�������ڻص�����
    }
    else                                // ����4�����ж�
    {
      
        
        
    }
    diag_pidwrite_check(49u);
}

void uart5_isr (void)
{
    diag_pidwrite_check(50u);
    if(uart_isr_mask(UART_5))            // ����5�����ж�
    {
    }
    else                                // ����5�����ж�
    {
      
        
        
    }
    diag_pidwrite_check(51u);
}

void uart6_isr (void)
{
    diag_pidwrite_check(52u);
    if(uart_isr_mask(UART_6))            // ����6�����ж�
    {
        mcar_comm_diag.rx_irq_count++;
        mcar_comm_poll();
       
    }
    else                                // ����6�����ж�
    {
      
        
        
    }
    diag_pidwrite_check(53u);
}
// **************************** �����жϺ��� ****************************

// **************************** �ⲿ�жϺ��� ****************************
void gpio_0_exti_isr()                  // �ⲿ GPIO_0 �жϷ�����     
{
    
  
  
}

void gpio_1_exti_isr()                  // �ⲿ GPIO_1 �жϷ�����     
{
    if(exti_flag_get(P01_0))		// ʾ��P1_0�˿��ⲿ�ж��ж�
    {

      
      
            
    }
    if(exti_flag_get(P01_1))
    {

            
            
    }
}

void gpio_2_exti_isr()                  // �ⲿ GPIO_2 �жϷ�����     
{
    if(exti_flag_get(P02_0))
    {
            
            
    }
    if(exti_flag_get(P02_4))
    {
            
            
    }

}

void gpio_3_exti_isr()                  // �ⲿ GPIO_3 �жϷ�����     
{



}

void gpio_4_exti_isr()                  // �ⲿ GPIO_4 �жϷ�����     
{



}

void gpio_5_exti_isr()                  // �ⲿ GPIO_5 �жϷ�����     
{



}


void gpio_6_exti_isr()                  // �ⲿ GPIO_6 �жϷ�����     
{

   /*if(exti_flag_get(P06_7))
    {

        imu660rc_callback();
    }*/

}

void gpio_7_exti_isr()                  // �ⲿ GPIO_7 �жϷ�����     
{



}

void gpio_8_exti_isr()                  // �ⲿ GPIO_8 �жϷ�����     
{



}

void gpio_9_exti_isr()                  // �ⲿ GPIO_9 �жϷ�����     
{



}

void gpio_10_exti_isr()                  // �ⲿ GPIO_10 �жϷ�����     
{



}

void gpio_11_exti_isr()                  // �ⲿ GPIO_11 �жϷ�����     
{



}

void gpio_12_exti_isr()                  // �ⲿ GPIO_12 �жϷ�����     
{



}

void gpio_13_exti_isr()                  // �ⲿ GPIO_13 �жϷ�����     
{



}

void gpio_14_exti_isr()                  // �ⲿ GPIO_14 �жϷ�����     
{



}

void gpio_15_exti_isr()                  // �ⲿ GPIO_15 �жϷ�����     
{



}

void gpio_16_exti_isr()                  // �ⲿ GPIO_16 �жϷ�����     
{



}

void gpio_17_exti_isr()                  // �ⲿ GPIO_17 �жϷ�����     
{



}

void gpio_18_exti_isr()                  // �ⲿ GPIO_18 �жϷ�����     
{



}

void gpio_19_exti_isr()                  // �ⲿ GPIO_19 �жϷ�����     
{



}

void gpio_20_exti_isr()                  // �ⲿ GPIO_20 �жϷ�����     
{



}

void gpio_21_exti_isr()                  // �ⲿ GPIO_21 �жϷ�����     
{



}

void gpio_22_exti_isr()                  // �ⲿ GPIO_22 �жϷ�����     
{



}

void gpio_23_exti_isr()                  // �ⲿ GPIO_23 �жϷ�����     
{



}
// **************************** �ⲿ�жϺ��� ****************************
