#include "stm32f10x.h"                  // Device header
#include "delay.h"
#include "ioi2c.h"
#include "mpu6050.h"
//#include "eeprom.h"
#include "serial.h"
#include "myexti.h"

//uint8_t addr[14];
short gx,gy,gz,ax,ay,az;
float yaw              =0;
float yaw_acc_error    =0;
float roll,pitch;
uint8_t addr;
#define FIVE_MS_ERROR   0.00002115

int main(void)
{

    Delay_init();
    Serial_Init();
    IIC_Init();
	MPU6050_initialize();
    addr=MPU6050_testConnection();
    




	DMP_Init();
	MBOT_EXTI_Init();

    //usart1_report_imu_V7_1(12349,2486,3536,0x00);
    while(1)
    {
        getAngle(&yaw,&yaw_acc_error);  
        MPU_Get_Gyroscope(&gx,&gy,&gz);
        MPU_Get_Accelerometer(&ax,&ay,&az);
        //mpu6050_send_data(ax,ay,az,gx,gy,gz);

//        if(Roll!=0&&roll!=(short)(Roll*100))
//        {
//        roll=(short)(Roll*100);
        //usart1_report_imu(ax,ay,az,gx,gy,gz,(short)(Roll*100),(short)(Pitch*100),(short)(Yaw*10));
        
        usart1_report_imu_V7_1((short)(Roll*100),(short)(Pitch*100),(short)(Yaw*100),0x03);
        //Delay_ms(38000);
        //}
        
    }
    
}

void EXTI3_IRQHandler(void) 
{                                                         
    
	EXTI_ClearITPendingBit(EXTI_Line3);
	
	yaw_acc_error += FIVE_MS_ERROR;

   // usart1_report_imu(ax,ay,az,gx,gy,gz,(short)(Roll*100),(short)(Pitch*100),(short)(Yaw*10));
    //Delay_ms(1000);
} 



