#include "stm32f10x.h"                  // Device header
void LED_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Mode=GPIO_Mode_Out_PP;
    GPIO_InitStruct.GPIO_Pin=GPIO_Pin_2;
    GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
    GPIO_Init(GPIOA,&GPIO_InitStruct);

}
void LED_ON()
{
    GPIO_ResetBits(GPIOA,GPIO_Pin_2);
}

void LED_OFF()
{
    GPIO_SetBits(GPIOA,GPIO_Pin_2);
}