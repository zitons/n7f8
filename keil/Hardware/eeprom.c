#include "stm32f10x.h"                  // Device header
#include "Delay.h"
/**
  * @brief  软件模拟EEPROM字节写入
  * @param  addr:要写入数据的地址
			data:要写入的数据
  * @retval	无
  */
  
u16 i,j;

void EEPROM_ByteWrite(uint8_t addr, uint8_t data)
{
	/* 产生起始信号 */
	I2C_GenerateSTART(I2C1, ENABLE);
	/* 检测EV5事件 */
	while( I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT) != SUCCESS );
	
	/* 发送7位从机地址 */
	I2C_Send7bitAddress(I2C1, 0xA0, I2C_Direction_Transmitter);
    
    Delay_ms(1000);

	/* 检测EV6事件 */
	while( I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) != SUCCESS );
	
	/* 发送要写入数据的地址 */
	I2C_SendData(I2C1, addr);
	/* 检测EV8事件 */
	while( I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTING) != SUCCESS );
	
	/* 发送要写入的数据 */
	I2C_SendData(I2C1, data);
	/* 检测EV8_2事件 */
	while( I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED) != SUCCESS );
	
	/* 产生停止信号 */
	I2C_GenerateSTOP(I2C1, ENABLE);	
	/* 重新使能ACK */
	I2C_AcknowledgeConfig(I2C1, ENABLE);
}


/**
  * @brief  软件模拟EEPROM随机读取
  * @param  addr:要读取数据的地址
			*data:要接收数据的容器
  * @retval	无
  */
void EEPROM_RandomRead(uint8_t addr, uint8_t* data)
{
	/* 产生第一次起始信号 */
	I2C_GenerateSTART(I2C1, ENABLE);
	/* 检测EV5事件 */
	while( I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT) != SUCCESS );
	/* 发送7位从机地址 */
	I2C_Send7bitAddress(I2C1, 0xA0, I2C_Direction_Transmitter);
	/* 检测EV6事件 */
	while( I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED ) != SUCCESS );
	/* 发送要读取数据的地址 */
	I2C_SendData(I2C1, addr);
	/* 检测EV8事件 */
	while( I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTING) != SUCCESS );
	
	/* 产生第二次起始信号 */
	I2C_GenerateSTART(I2C1, ENABLE);
	/* 检测EV5事件 */
	while( I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT) != SUCCESS );
	/* 发送7位从机地址 */
	I2C_Send7bitAddress(I2C1, 0xA1, I2C_Direction_Receiver);
	/* 检测EV6事件 */
	while( I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) != SUCCESS );
	
	/* 检测EV7事件 */
	while( I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED) != SUCCESS );
	/* 读取数据寄存器中的数据 */
	*data = I2C_ReceiveData(I2C1);
	
	/* 产生停止信号 */
	I2C_GenerateSTOP(I2C1, ENABLE);
	/* 重新使能ACK */
	I2C_AcknowledgeConfig(I2C1, ENABLE);	
}

