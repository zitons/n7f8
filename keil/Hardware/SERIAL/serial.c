#include "stm32f10x.h"                  // Device header
#include "string.h"
uint8_t txd_buf[4]={1,2,3,4};//数据包
uint8_t rxd_buf[28];//定义数组接收数据包，定长
uint8_t rxd_flag = 0;//接收标志
uint8_t rxd_index = 0;//接收索引
u8 recv_dat;
u8 last_sent_buffer[7]; 
/**
  * 函    数：串口初始化
  * 参    数：无
  * 返 回 值：无
  */
  
void Serial_Init(void)
{	
    GPIO_InitTypeDef GPIO_Tx_InitStructure;
    GPIO_InitTypeDef GPIO_Rx_InitStructure;
    USART_InitTypeDef USART_InitStructure;					//定义结构体变量
    NVIC_InitTypeDef NVIC_InitStructure;
	/*开启时钟*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);	//开启USART1的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	//开启GPIOA的时钟
	
	/*GPIO初始化*/

	GPIO_Tx_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Tx_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_Tx_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_Tx_InitStructure);					//将PA9引脚初始化为复用推挽输出
    
    //RX端
	GPIO_Rx_InitStructure.GPIO_Mode=GPIO_Mode_IPU;  //上拉输入
    GPIO_Rx_InitStructure.GPIO_Pin=GPIO_Pin_10;
    GPIO_Rx_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;  //50MhZ
    GPIO_Init(GPIOA,&GPIO_Rx_InitStructure);

	
	/*USART初始化*/

	USART_InitStructure.USART_BaudRate = 9600;				//波特率
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	//硬件流控制，不需要
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;			//模式，选择为发送模式
	USART_InitStructure.USART_Parity = USART_Parity_No;		//奇偶校验，不需要
	USART_InitStructure.USART_StopBits = USART_StopBits_1;	//停止位，选择1位
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;		//字长，选择8位
	USART_Init(USART1, &USART_InitStructure);				//将结构体变量交给USART_Init，配置USART1
	
	/*USART使能*/
	USART_Cmd(USART1, ENABLE);								//使能USART1，串口开始运行
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启获取中断

	//Usart1 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//串口1中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//响应优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、	

}


void usart1_send_char(u8 c)
{
	USART_SendData(USART1, c);		//将字节数据写入数据寄存器，写入后USART自动生成时序波形
	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);	//等待发送完成
}

/*^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^上传数据给上位机V7.1版本^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^*/	 


//传送数据给匿名四轴上位机软件(V7.1版本)
//fun:功能字. 0X01~0X1C
//data:数据缓存区,最多28字节!!
//len:data区有效数据个数
void usart1_niming_report(u8 fun,u8*data,u8 len)
{
	int i;
    u8 sumcheck = 0;
    u8 addcheck = 0;

	u8 send_buf[32];
	if(len>28)return;	//最多28字节数据
	send_buf[len+4]=0;	//校验和数置零 7+4=11 12个数据
	send_buf[len+5]=0;	//附加校验数置零 第13个数据
	send_buf[0]=0xAA;//帧头
	send_buf[1]=0xFF;//目标地址
	send_buf[2]=fun;//功能码
	send_buf[3]=len;//数据长度
    
	for(i=0;i<len;i++)send_buf[4+i]=data[i];			//复制数据
	for(i=0;i<len+4;i++)//0-len+3
	{
        send_buf[len+4]+=send_buf[i];	//计算校验和	
		send_buf[len+5]+=send_buf[len+4];	//计算校验和		
	}
    
    
    
//    for(i=0; i < (send_buf[3]+4); i++)
//    {
//    sumcheck += send_buf[i]; //从帧头开始，对每一字节进行求和，直到DATA区结束
//    addcheck += sumcheck; //每一字节的求和操作，进行一次sumcheck的累加
//    }
//    //如果计算出的sumcheck和addcheck和接收到的check数据相等，代表校验通过，反之数据有误
//    if(sumcheck == send_buf[send_buf[3]+4] && addcheck == send_buf[send_buf[3]+5])
//    {
        
        for(i=0;i<len+6;i++)usart1_send_char(send_buf[i]);	//发送数据到串口1 
        
    
//    }
	
    //0-len+5
}




//发送加速度传感器数据+陀螺仪数据(传感器帧)
//aacx,aacy,aacz:x,y,z三个方向上面的加速度值
//gyrox,gyroy,gyroz:x,y,z三个方向上面的陀螺仪值 
void send_sensorData(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz)
{
	u8 tbuf[12]; 
	tbuf[0]=aacx&0XFF;
	tbuf[1]=(aacx>>8)&0XFF;
  tbuf[2]=aacy&0XFF;
	tbuf[3]=(aacy>>8)&0XFF;
	tbuf[4]=aacz&0XFF;
	tbuf[5]=(aacz>>8)&0XFF;
	tbuf[6]=gyrox&0XFF;
	tbuf[7]=(gyrox>>8)&0XFF;
	tbuf[8]=gyroy&0XFF;
	tbuf[9]=(gyroy>>8)&0XFF;
	tbuf[10]=gyroz&0XFF;
	tbuf[11]=(gyroz>>8)&0XFF;
	usart1_niming_report(0XF1,tbuf,12);	
}

//通过串口1上报结算后的姿态数据给电脑(状态帧)
//roll:横滚角.单位0.01度。 -18000 -> 18000 对应 -180.00  ->  180.00度
//pitch:俯仰角.单位 0.01度。-9000 - 9000 对应 -90.00 -> 90.00 度
//yaw:航向角.单位为0.1度 0 -> 3600  对应 0 -> 360.0度
//csb:超声波高度,单位:cm
//prs:气压计高度,单位:mm
void usart1_report_imu_V7_1(short roll,short pitch,short yaw,u8 state)
{
	u8 tbuf[7];  
    tbuf[0]=roll&0XFF; 	
	tbuf[1]=(roll>>8)&0XFF;
	tbuf[2]=pitch&0XFF;
	tbuf[3]=(pitch>>8)&0XFF;
	tbuf[4]=yaw&0XFF;//低位
	tbuf[5]=(yaw>>8)&0XFF;//高位
	tbuf[6]=state;
    if (tbuf != last_sent_buffer) {
	usart1_niming_report(0X03,tbuf,7);//功能码ID，0X03，飞控姿态：欧拉角格式
    strcpy(last_sent_buffer,tbuf);
    }
}



void USART1_IRQHandler(void)                	//串口1中断服务程序
{
	
    u8 tbuf[7];  
	static uint8_t recv_state = 0;//默认从索引0开始
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //接收中断
	{
		recv_dat =USART_ReceiveData(USART1);//(USART1->DR);	//读取接收到的数据
		switch(recv_state)
		{ 
			case 0:
                 if(recv_dat == 0XAA)//接收到包头
				  {
				     recv_state =1;//切换状态
					 rxd_index = 0;
				  }
				  else
				  {
				     recv_state =0;//切换状态
				  }
			      break;
			case 1:
                  rxd_buf[rxd_index]=recv_dat;//接收字符
			      rxd_index++;
				  if(rxd_buf[0]==0xff&&rxd_buf[1]==0xe1)
				  {
					if(rxd_index>=4+(int)rxd_buf[2])//判断是否接收数据包完成1
					{
						recv_state =2;//切换状态
                        
                        
                        tbuf[0]=rxd_buf[3]; 	
                        tbuf[1]=rxd_buf[4];
                        tbuf[2]=0x00;
                        tbuf[3]=0x00;
                        tbuf[4]=0x00;//低位
                        tbuf[5]=0x01;//高位
                        
                        usart1_niming_report(0XE2,tbuf,6);//功能码ID，0X03，飞控姿态：欧拉角格式
                        rxd_index=0;
					}
				  }
			      
			      break;
			case 2:
                  //rxd_index=0;
				  rxd_flag = 1;//接收位置1
				  recv_state =0;//并将状态清零，接收下一包数据 
				  
			      break;
		}
		USART_ClearITPendingBit(USART1,USART_IT_RXNE);//清除中断标志位	 
	
	} 
}

