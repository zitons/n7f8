#include "stm32f10x.h"                  // Device header
#include "delay.h"
#include "eeprom.h"
//#include "OLED.h"
//#include "led.h"
//#include "key.h"
//#include "Timer.h"

uint16_t Num;
uint8_t data = 45;
uint8_t data_rec = 0;
uint8_t addr = 11;
 /**
  * @brief  等待EEPROM写入完成
  * @param  无
  * @retval	无
  */


int main(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitTypeDef GPIO_InitStructure; //配置GPIO口结构体  -PB6 -PB7
    I2C_InitTypeDef I2C_InitStructure;   //配置硬件I2C结构体
    GPIO_InitStruct.GPIO_Mode=GPIO_Mode_Out_PP;
    GPIO_InitStruct.GPIO_Pin=GPIO_Pin_0;
    GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
    GPIO_Init(GPIOC,&GPIO_InitStruct);
    GPIO_SetBits(GPIOC,GPIO_Pin_0);

    


    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); //使能GPIOB时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);  //使能I2C1时钟
    
    //PB6——SCL PB7——SDA
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD; //复用开漏
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    
    GPIO_Init(GPIOB, &GPIO_InitStructure); //初始化GPIOB结构体
    
    
    I2C_DeInit(I2C1);
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;        //I2C应答信号使能
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit; //设置从机地址长度为7位
    I2C_InitStructure.I2C_ClockSpeed = 10000;         //设置目标周期 400khz
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2; //设置占空比为2：1
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;         //设置为I2C模式
    I2C_InitStructure.I2C_OwnAddress1 = 0x30;          //设置主机地址
    
    I2C_Init(I2C1, &I2C_InitStructure); //初始化I2C1结构体
    I2C_Cmd(I2C1, ENABLE); //使能I2C1

	EEPROM_ByteWrite(addr, data);
	EEPROM_RandomRead(addr, &data_rec);

    //EEPROM_ByteWrite(addr, data);
	//EEPROM_RandomRead(addr, &data_rec);
	
    while(1)
    {
      //LED_ON();
         
      //  N=TIM_GetCounter(TIM2);
        
    }
    
}




