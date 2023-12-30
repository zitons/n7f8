#include "stm32f10x.h"                  // Device header
#include "Delay.h"
/**
  * @brief  ���ģ��EEPROM�ֽ�д��
  * @param  addr:Ҫд�����ݵĵ�ַ
			data:Ҫд�������
  * @retval	��
  */
  
u16 i,j;

void EEPROM_ByteWrite(uint8_t addr, uint8_t data)
{
	/* ������ʼ�ź� */
	I2C_GenerateSTART(I2C1, ENABLE);
	/* ���EV5�¼� */
	while( I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT) != SUCCESS );
	
	/* ����7λ�ӻ���ַ */
	I2C_Send7bitAddress(I2C1, 0xA0, I2C_Direction_Transmitter);
    
    Delay_ms(1000);

	/* ���EV6�¼� */
	while( I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) != SUCCESS );
	
	/* ����Ҫд�����ݵĵ�ַ */
	I2C_SendData(I2C1, addr);
	/* ���EV8�¼� */
	while( I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTING) != SUCCESS );
	
	/* ����Ҫд������� */
	I2C_SendData(I2C1, data);
	/* ���EV8_2�¼� */
	while( I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED) != SUCCESS );
	
	/* ����ֹͣ�ź� */
	I2C_GenerateSTOP(I2C1, ENABLE);	
	/* ����ʹ��ACK */
	I2C_AcknowledgeConfig(I2C1, ENABLE);
}


/**
  * @brief  ���ģ��EEPROM�����ȡ
  * @param  addr:Ҫ��ȡ���ݵĵ�ַ
			*data:Ҫ�������ݵ�����
  * @retval	��
  */
void EEPROM_RandomRead(uint8_t addr, uint8_t* data)
{
	/* ������һ����ʼ�ź� */
	I2C_GenerateSTART(I2C1, ENABLE);
	/* ���EV5�¼� */
	while( I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT) != SUCCESS );
	/* ����7λ�ӻ���ַ */
	I2C_Send7bitAddress(I2C1, 0xA0, I2C_Direction_Transmitter);
	/* ���EV6�¼� */
	while( I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED ) != SUCCESS );
	/* ����Ҫ��ȡ���ݵĵ�ַ */
	I2C_SendData(I2C1, addr);
	/* ���EV8�¼� */
	while( I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTING) != SUCCESS );
	
	/* �����ڶ�����ʼ�ź� */
	I2C_GenerateSTART(I2C1, ENABLE);
	/* ���EV5�¼� */
	while( I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT) != SUCCESS );
	/* ����7λ�ӻ���ַ */
	I2C_Send7bitAddress(I2C1, 0xA1, I2C_Direction_Receiver);
	/* ���EV6�¼� */
	while( I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) != SUCCESS );
	
	/* ���EV7�¼� */
	while( I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED) != SUCCESS );
	/* ��ȡ���ݼĴ����е����� */
	*data = I2C_ReceiveData(I2C1);
	
	/* ����ֹͣ�ź� */
	I2C_GenerateSTOP(I2C1, ENABLE);
	/* ����ʹ��ACK */
	I2C_AcknowledgeConfig(I2C1, ENABLE);	
}

