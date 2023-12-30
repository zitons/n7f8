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
  * @brief  �ȴ�EEPROMд�����
  * @param  ��
  * @retval	��
  */


int main(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitTypeDef GPIO_InitStructure; //����GPIO�ڽṹ��  -PB6 -PB7
    I2C_InitTypeDef I2C_InitStructure;   //����Ӳ��I2C�ṹ��
    GPIO_InitStruct.GPIO_Mode=GPIO_Mode_Out_PP;
    GPIO_InitStruct.GPIO_Pin=GPIO_Pin_0;
    GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
    GPIO_Init(GPIOC,&GPIO_InitStruct);
    GPIO_SetBits(GPIOC,GPIO_Pin_0);

    


    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); //ʹ��GPIOBʱ��
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);  //ʹ��I2C1ʱ��
    
    //PB6����SCL PB7����SDA
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD; //���ÿ�©
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    
    GPIO_Init(GPIOB, &GPIO_InitStructure); //��ʼ��GPIOB�ṹ��
    
    
    I2C_DeInit(I2C1);
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;        //I2CӦ���ź�ʹ��
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit; //���ôӻ���ַ����Ϊ7λ
    I2C_InitStructure.I2C_ClockSpeed = 10000;         //����Ŀ������ 400khz
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2; //����ռ�ձ�Ϊ2��1
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;         //����ΪI2Cģʽ
    I2C_InitStructure.I2C_OwnAddress1 = 0x30;          //����������ַ
    
    I2C_Init(I2C1, &I2C_InitStructure); //��ʼ��I2C1�ṹ��
    I2C_Cmd(I2C1, ENABLE); //ʹ��I2C1

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




