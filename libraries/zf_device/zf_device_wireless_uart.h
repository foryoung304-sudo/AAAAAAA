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
* �ļ�����          zf_device_wireless_uart
* ��˾����          �ɶ���ɿƼ����޹�˾
* �汾��Ϣ          �鿴 libraries/doc �ļ����� version �ļ� �汾˵��
* ��������          IAR 9.40.1
* ����ƽ̨          CYT4BB
* ��������          https://seekfree.taobao.com/
*
* �޸ļ�¼
* ����              ����                ��ע
* 2024-01-12       pudding           first version
********************************************************************************************************************/
/*********************************************************************************************************************
* ���߶��壺
*                  ------------------------------------
*                  ģ��ܽ�             ��Ƭ���ܽ�
*                  RX                 �鿴 zf_device_wireless_uart.h �� WIRELESS_UART_RX_PINx �궨��
*                  TX                 �鿴 zf_device_wireless_uart.h �� WIRELESS_UART_TX_PINx �궨��
*                  RTS                �鿴 zf_device_wireless_uart.h �� WIRELESS_UART_RTS_PINx �궨��
*                  VCC                3.3V��Դ
*                  GND                ��Դ��
*                  ������������
*                  ------------------------------------
********************************************************************************************************************/

#ifndef _zf_device_wireless_uart_h_
#define _zf_device_wireless_uart_h_

#include "zf_common_typedef.h"
//================================================���� ���ߴ��� ��������===================================================
#define WIRELESS_UART_INDEX         (UART_2		   )                    // ���ߴ��ڶ�Ӧʹ�õĴ��ں�
#define WIRELESS_UART_BUAD_RATE     (115200		   )                    // ���ߴ��ڶ�Ӧʹ�õĴ��ڲ�����
#define WIRELESS_UART_TX_PIN        (UART2_RX_P10_0        )                    // ���ߴ��ڶ�Ӧģ��� TX Ҫ�ӵ���Ƭ���� RX
#define WIRELESS_UART_RX_PIN        (UART2_TX_P10_1        )                    // ���ߴ��ڶ�Ӧģ��� RX Ҫ�ӵ���Ƭ���� TX
#define WIRELESS_UART_RTS_PIN       (P22_6		   )                    // ���ߴ��ڶ�Ӧģ��� RTS ����
//====================================================�Զ�������====================================================
// ע������1������ת����ģ��汾��V2.0���µ����޷������Զ������ʵġ�
// ע������2�������Զ��������������RTS���� ����Ὺ��ʧ�ܡ�
// ע������3��ģ���Զ�������ʧ�ܵĻ� ���Գ��Զϵ�����

// �����Զ�����������Ķ��������� ע������
// �����Զ�����������Ķ��������� ע������
// �����Զ�����������Ķ��������� ע������

// 0���ر��Զ�������  
// 1�������Զ������� �Զ������ʵ��������޸� WIRELESS_UART_BAUD ֮����Ҫ��ģ��������� ģ����Զ�����Ϊ��Ӧ�Ĳ�����

#define WIRELESS_UART_AUTO_BAUD_RATE    ( 0 )
//====================================================�Զ�������====================================================
#if (1 == WIRELESS_UART_AUTO_BAUD_RATE)
typedef enum
{
    WIRELESS_UART_AUTO_BAUD_RATE_SUCCESS,
    WIRELESS_UART_AUTO_BAUD_RATE_INIT,
    WIRELESS_UART_AUTO_BAUD_RATE_START,
    WIRELESS_UART_AUTO_BAUD_RATE_GET_ACK,
}wireless_uart_auto_baudrate_state_enum;
#endif

#define WIRELESS_UART_BUFFER_SIZE       ( 64   )
#define WIRELESS_UART_TIMEOUT_COUNT     ( 0x64 )
//================================================���� ���ߴ��� ��������===================================================


//================================================���� ���ߴ��� ��������===================================================
uint32      wireless_uart_send_byte         (const uint8 data);
uint32      wireless_uart_send_buffer      (const uint8 *buff, uint32 len);
uint32      wireless_uart_send_string       (const char *str);

uint32      wireless_uart_read_buffer       (uint8 *buff, uint32 len);

void        wireless_uart_callback          (void);

uint8       wireless_uart_init              (void);
//================================================���� ���ߴ��� ��������===================================================

#endif


