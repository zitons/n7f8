#include "stm32f10x.h"                  // Device header
#include "delay.h"
#include "ioi2c.h"
#include "mpu6050.h"
//#include "eeprom.h"
#include "myexti.h"
uint8_t addr[14];
float yaw              =0;           //ת��������
float yaw_acc_error    =0;           //yaw�ۻ����
#define FIVE_MS_ERROR   0.00002115   //yawÿ5ms������Ư�ƵĶ���������������ԣ�����������Сʱƫ1�ȣ�ÿ���˵����ֵ����������ͬ���������м���

int main(void)
{
    Delay_init();	    	        //=====��ʱ������ʼ��
    IIC_Init();                     //=====IIC��ʼ��    ��ȡMPU6050����
	MPU6050_initialize();           //=====MPU6050��ʼ��	
    addr[0]=MPU6050_getDeviceID();
	DMP_Init();                     //=====��ʼ��DMP 
	MBOT_EXTI_Init();               //=====MPU6050 5ms��ʱ�жϳ�ʼ��
    while(1)
    {
	getAngle(&yaw,&yaw_acc_error);  


    }
    
}

void EXTI3_IRQHandler(void) 
{                                                         
	EXTI_ClearITPendingBit(EXTI_Line3);                            //===���LINE12��·����λ
	
	yaw_acc_error += FIVE_MS_ERROR;								    //===yawƯ������ۼ�


} 



