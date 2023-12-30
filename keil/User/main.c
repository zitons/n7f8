#include "stm32f10x.h"                  // Device header
#include "delay.h"
#include "ioi2c.h"
#include "mpu6050.h"
//#include "eeprom.h"
#include "myexti.h"
uint8_t addr[14];
float yaw              =0;           //转向陀螺仪
float yaw_acc_error    =0;           //yaw累积误差
#define FIVE_MS_ERROR   0.00002115   //yaw每5ms的向上漂移的度数，这里近似线性，可以做到半小时偏1度，每个人的这个值可能有所不同，可以自行计算

int main(void)
{
    Delay_init();	    	        //=====延时函数初始化
    IIC_Init();                     //=====IIC初始化    读取MPU6050数据
	MPU6050_initialize();           //=====MPU6050初始化	
    addr[0]=MPU6050_getDeviceID();
	DMP_Init();                     //=====初始化DMP 
	MBOT_EXTI_Init();               //=====MPU6050 5ms定时中断初始化
    while(1)
    {
	getAngle(&yaw,&yaw_acc_error);  


    }
    
}

void EXTI3_IRQHandler(void) 
{                                                         
	EXTI_ClearITPendingBit(EXTI_Line3);                            //===清除LINE12线路挂起位
	
	yaw_acc_error += FIVE_MS_ERROR;								    //===yaw漂移误差累加


} 



