#ifndef __SERIAL_H
#define __SERIAL_H


void Serial_Init(void);
void usart1_send_char(u8 c);
void usart1_niming_report(u8 fun,u8*data,u8 len);
void usart1_report_imu_V7_1(short roll,short pitch,short yaw,u8 state);
#endif
