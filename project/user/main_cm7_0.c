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
* �ļ�����          main_cm7_0
* ��˾����          �ɶ���ɿƼ����޹�˾
* �汾��Ϣ          �鿴 libraries/doc �ļ����� version �ļ� �汾˵��
* ��������          IAR 9.40.1
* ����ƽ̨          CYT4BB
* ��������          https://seekfree.taobao.com/
*
* �޸ļ�¼
* ����              ����                ��ע
* 2024-1-4       pudding            first version
********************************************************************************************************************/

#include "zf_common_headfile.h"
// ���µĹ��̻��߹����ƶ���λ�����ִ�����²���
// ��һ�� �ر��������д򿪵��ļ�
// �ڶ��� project->clean  �ȴ��·�����������

// �������ǿ�Դ��չ��� ��������ֲ���߲��Ը���������
// �������ǿ�Դ��չ��� ��������ֲ���߲��Ը���������
// �������ǿ�Դ��չ��� ��������ֲ���߲��Ը���������

// **************************** �������� ****************************

float norm_temp = 0.0f;
extern uint32_t isr_time_cost;
extern float real_dt_2ms;
extern float real_dt_20ms;
extern uint32_t duty;
extern volatile uint8 imu660rc_chip_id_debug;
extern uint32_t loop_cnt;
extern volatile uint8_t height_update_request;
extern volatile uint8_t field_map_request;
extern volatile uint8_t debug_print_request;
void field_map_init(void);
void field_map_task(void);
void diag_pidwrite_check(uint8_t stage);
void diag_pidwrite_rearm(void);

int main(void)
{
    clock_init(SYSTEM_CLOCK_250M); 	// ʱ�����ü�ϵͳ��ʼ��<��ر���>
    debug_init();                       // ���Դ�����Ϣ��ʼ��
    {
        uint32_t reset_reason = Cy_SysReset_GetResetReason();
        uint32_t mcwdt_mask = CY_SYSRESET_MCWDT0 |
                              CY_SYSRESET_MCWDT1 |
                              CY_SYSRESET_MCWDT2 |
                              CY_SYSRESET_MCWDT3;

        printf("RESETCAUSE,raw=0x%08lX,wdt=%u,act_fault=%u,soft=%u,mcwdt=%u,xres=%u,porvddd=%u\r\n",
               (unsigned long)reset_reason,
               (reset_reason & CY_SYSRESET_WDT) ? 1u : 0u,
               (reset_reason & CY_SYSRESET_ACT_FAULT) ? 1u : 0u,
               (reset_reason & CY_SYSRESET_SOFT) ? 1u : 0u,
               (reset_reason & mcwdt_mask) ? 1u : 0u,
               (reset_reason & CY_SYSRESET_PXRES) ? 1u : 0u,
               (reset_reason & CY_SYSRESET_PORVDDD) ? 1u : 0u);
        Cy_SysReset_ClearAllResetReasons();
    }
    // �˴���д�û����� ���������ʼ�������
 

  lora3a22_init();
   //imu660rc_init(IMU660RC_QUARTERNION_DISABLE);
   icm42688_init();
   //imu660rc_init(IMU660RC_QUARTERNION_120HZ);

   tof_init();
#if (FLOW_SENSOR_TYPE == FLOW_SENSOR_LC302)
   lc302_init();
#else
   pmw3901_init();
#endif
   imu_data_init();
   imu_calibrate();
   (void)system_time_us();

   // ================= ����ɿظ�����ƻ��� PID ���� =================
   param_init();
   att_ctrl_init();
   alt_ctrl_init();
 //  diag_pidwrite_rearm();
 //  diag_pidwrite_check(30u);
   loc_ctrl_init();
 //  diag_pidwrite_check(31u);
 //  field_map_init();
   vision_nav_init();
 //  diag_pidwrite_check(32u);
   //pid_menu_init();
 //  diag_pidwrite_check(33u);
 //  ips114_init();
  // diag_pidwrite_check(34u);

   small_driver_uart_init();
  // diag_pidwrite_check(35u);
   mcar_comm_init();
   //diag_pidwrite_check(36u);
    //wireless_uart_init();
//   imu_log_start();
    pit_ms_init(PIT_CH0, 2); 
    ////diag_pidwrite_check(37u);
    pit_enable(PIT_CH0);





    
  
    while(true)
    {
        static uint32_t main_height_max_us = 0;
        //imu_log_dump_task();
        //height_log_dump_task();
        //loc_log_dump_task();
       // printf("main loop, dt20ms=%.3fms, dt2ms=%.3fms, isr_cost=%luus, duty=%lu\r\n", real_dt_20ms*1000.0f, real_dt_2ms*1000.0f, isr_time_cost, duty);
      if(height_update_request)
        {
            uint32_t height_start_us = system_time_us();
            height_update_request = 0;
            update_current_height(0.02f);
           // diag_pidwrite_check(20u);
            uint32_t height_cost_us = system_time_us() - height_start_us;
            if(height_cost_us > main_height_max_us) main_height_max_us = height_cost_us;
        }

/*       if(field_map_request)
        {
            field_map_request = 0;
            field_map_task();
            diag_pidwrite_check(21u);
        }
*/
       vision_consumer_update();
      // diag_pidwrite_check(22u);

       if(debug_print_request)
        {
            debug_print_request = 0;
            debug_print_states();
            //diag_pidwrite_check(23u);
        }
      //memcpy(base_image, mt9v03x_image, MT9V03X_IMAGE_SIZE); // 将采集到的图像数据复制到 base_image 中



        //uint8_t chip_id = icm42688_get_chip_id();
        //printf("%d",chip_id);
        //lc302_update();
        //lc302_debug_print();
       //flow_debug_print();
        // printf("channel_data:%.3f,%.3f,%.3f\r\n", imu_data.pitch, imu_data.roll, imu_data.yaw);
      /* printf("rpy %.3f %.3f %.3f | e %.5f %.5f %.5f | i %.5f %.5f %.5f | kp %.3f ki %.3f w %.2f gn %.4f\r\n",
            imu_data.roll, imu_data.pitch, imu_data.yaw,
            imu_debug_ex, imu_debug_ey, imu_debug_ez,
            imu_debug_ex_int, imu_debug_ey_int, imu_debug_ez_int,
            imu_debug_kp, imu_debug_ki,
            imu_debug_acc_weight,
            imu_debug_gyro_rad_norm);
            printf("gyro_actual: %.3f, %.3f, %.3f\r\n", imu_data.gyro_actual[0], imu_data.gyro_actual[1], imu_data.gyro_actual[2]);
            printf("acc_actual: %.3f, %.3f, %.3f\r\n", imu_data.acc_actual[0], imu_data.acc_actual[1], imu_data.acc_actual[2]);
           //pid_menu_task();
           /*printf("armed: %d |actual: %.3f, %.3f, %.3f | ct: %.3f, %.3f, %.3f|throttle: %.3f |motor: %d, %d, %d, %d\r\n",
            vehicle_state.armed,
            imu_data.gyro_actual[0], imu_data.gyro_actual[1], imu_data.gyro_actual[2],
            ct_val.rol, ct_val.pit, ct_val.yaw,
            vehicle_setpoint.target_throttle,
            motor_out.m1, motor_out.m2, motor_out.m3, motor_out.m4);*/
       // printf("height: %.3f \r\n",
           // vehicle_state.current_height);

        //debug_pit_diag_task(&main_height_max_us);
      

        // �˴���д��Ҫѭ��ִ�еĴ���*/
    }
}

// **************************** �������� ****************************
