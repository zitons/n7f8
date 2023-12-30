#include "stm32f10x.h"                  // Device header
#include "Delay.h"
void KEY_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Mode=GPIO_Mode_IPU;
    GPIO_InitStruct.GPIO_Pin=GPIO_Pin_1;
    GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
    GPIO_Init(GPIOB,&GPIO_InitStruct);
}
uint8_t Key_GetNum(void)
{
    //uint8_t KeyNum=0;
   /* if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_1)==0)
    {
        Delay_ms(20);
        
    }
    while(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_1)==0);
    Delay_ms(20);
    KeyNum=1;*/
    return GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_1);
}