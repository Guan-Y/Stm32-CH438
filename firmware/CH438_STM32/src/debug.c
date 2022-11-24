/********************************** (C) COPYRIGHT ******************************
* File Name          : debug.c
* Author             : WCH
* Version            : 
* Date               : 2014/9/11
* Description        : stm32中串口调试驱动
*******************************************************************************/

#include "stm32f10x.h"
#include <stdio.h>
#include "globalhead.h"

#define Debug_PORT			GPIOA
#define Debug_RX_PIN		GPIO_Pin_10
#define Debug_TX_PIN		GPIO_Pin_9

#define Debug_UART			USART1
#define PUTCHAR_PROTOTYPE 	int fputc(int ch, FILE *f)
#define USART_REC_LEN				128

#define Amp_com		5  //功放级串口号

#define DMA_size	256

static void D_GPIO_Config(void);
static void D_UART_Config(void);
static void D_NVIC_Config(void);
static void USART_TX_DMA_Config(void);
void DMA_USART_Send(u8* data, u16 size);

u8 DMA_DATA_BUF[DMA_size];


uint8_t 	USART_RX_BUF[USART_REC_LEN];
uint16_t 	USART_RX_STA;       //接收状态标记
uint8_t 	p_send[USART_REC_LEN];
uint8_t		Amp_first_time = 1;
uint8_t		Amp_data_len;

uint8_t CH438_Data[128];
u8 n = 5;
uint8_t t;
uint8_t i;
uint8_t len;
//uint16_t times;
uint8_t frame_tail = 0;
uint8_t SelectCom;
uint8_t datalen;

u16 time_delay;
u16 time_begin;
u16 time_stop;


//AA 55 00 11 01 03 0C 00 64 FF FF 00 05 00 00 00 00 03 AE DC 0A 1E 0D 0A
//AA 55 07 05 EF 01 00 00 EF EA 0D 0A
//AA 55 01 11 01 03 0C 00 01 FF FF 00 05 00 00 00 00 03 AE 33 C7 D0 0D 0A
//AA 55 03 1B 01 03 16 00 08 0C 85 03 5C 03 58 03 56 A0 AB 00 00 00 00 00 00 03 E8 00 01 E2 AB A7 0D 0A
//AA 55 06 05 EF 01 00 00 EF E9 0D 0A
//AA 55 06 04 EF 04 01 EF EC 0D 0A 开启

//端口1
u8 debug_send_middle[] 			= {0xaa, 0x55, 0x01, 0x11, 0x01, 0x03, 0x0c, 0x00, 0x01, 0xff, 0xff, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0x03, 0xae, 0x33, 0xc7, 0xd0, 0x0d, 0x0a}; //模拟第一级序列
//端口3
u8 debug_send_long[]				= {0xAA, 0x55, 0x03, 0x1B, 0x01, 0x03, 0x16, 0x00, 0x08, 0x0C, 0x85, 0x03, 0x5C, 0x03, 0x58, 0x03, 0x56, 0xA0, 0xAB, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xE8, 0x00, 0x01, 0xE2, 0xAB, 0xA7, 0x0D, 0x0A};//模拟第三级长序列
//端口6
u8 debug_send_short[] 			= {0xAA, 0x55, 0x06, 0x05, 0xEF, 0x01, 0x00, 0x00, 0xEF, 0xE9, 0x0D, 0x0A};	//模拟大电流短序列
//端口3
u8 debug_send_wrong_long[] 	= {0xAA, 0x55, 0x03, 0x1B, 0x01, 0x03, 0x16, 0x00, 0x00, 0x0A, 0x80, 0x03, 0xBC, 0x03, 0xB4, 0x03, 0xAC, 0xA0, 0xAB, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xE8, 0x00, 0x01, 0xD2, 0x09, 0xF8, 0x0d, 0x0a};//模拟出错的长序列
//端口6
u8 debug_open1[]						= {0xAA, 0x55, 0x06, 0x04, 0xEF, 0x04, 0x01, 0xEF, 0xEC, 0x0D, 0x0A};
//端口0
u8 debug_open2[]						=	{0xAA, 0x55, 0x00, 0x11, 0x01, 0x03, 0x0C, 0x00, 0x64, 0xFF, 0xFF, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0x03, 0xAE, 0xDC, 0x0A, 0x1E, 0x0D, 0x0A};
//端口2
u8 debug_open3[]						=	{0xaa, 0x55, 0x02, 0x11, 0x01, 0x03, 0x0c, 0x00, 0x01, 0xff, 0xff, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0x03, 0xae, 0x33, 0xc7, 0xd1, 0x0d, 0x0a};
//端口4
u8 debug_open4[]						= {0xAA, 0x55, 0x04, 0x04, 0xEF, 0x04, 0x01, 0xEF, 0xEA, 0x0D, 0x0A};
//端口5
u8 debug_open5[]						= {0xAA, 0x55, 0x05, 0x04, 0xEF, 0x04, 0x01, 0xEF, 0xEB, 0x0D, 0x0A};
//端口7
u8 debug_open7[]						= {0xAA, 0x55, 0x07, 0x04, 0xEF, 0x04, 0x01, 0xEF, 0xED, 0x0D, 0x0A};


u8 len_middle = 24;
u8 len_long 	= 34;
u8 len_short 	= 12;
u8 len_open 	= 11;
	
	/**
  * Function Name  : DEBUG_Config()
  * Description    : 调试串口初始化
  * Input          : None
  * Return         : None
  */
void DEBUG_Config(void)
{	
	D_GPIO_Config();
	D_UART_Config();
	D_NVIC_Config();
	USART_TX_DMA_Config();
}

////////  以下是内部函数  ///////////////////////////////////////////

static void D_GPIO_Config(void)
{
	GPIO_InitTypeDef debug_gpio;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	debug_gpio.GPIO_Pin = Debug_RX_PIN;
	debug_gpio.GPIO_Speed = GPIO_Speed_50MHz;
	debug_gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(Debug_PORT, &debug_gpio);

	debug_gpio.GPIO_Pin = Debug_TX_PIN;
	debug_gpio.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(Debug_PORT, &debug_gpio);
}

static void D_UART_Config(void)
{
	USART_InitTypeDef	debug_uart1;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);	
	debug_uart1.USART_BaudRate = 115200;
	debug_uart1.USART_WordLength = USART_WordLength_8b;
	debug_uart1.USART_StopBits = USART_StopBits_1;
	debug_uart1.USART_Parity = USART_Parity_No;
	debug_uart1.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	debug_uart1.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	
	USART_Init(Debug_UART, &debug_uart1);
	USART_Cmd(Debug_UART, ENABLE);
	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
}

static void D_NVIC_Config(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1 ;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
	
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_IRQn; //嵌套通道为DMA1_Channel4_IRQn
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; //抢占优先级为 2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3; //响应优先级为 3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //通道中断使能
	NVIC_Init(&NVIC_InitStructure);
}

void send_test(u8*data, u8 len)
{
	u8 t =0;
	for(t = 0; t< len; t++)
		{
			USART_SendData(USART1, data[t]);
			while(USART_GetFlagStatus(USART1, USART_FLAG_TC)==RESET);
		}		
}

void Forward_CH438()
{
	u8 clear;
	
	if(USART_RX_STA & 0x8000)
		{ 	
			// ============  debug时使用 ===================================
//			TIM_Cmd(TIM2, ENABLE); //计算延时
//			time_begin = TIM_GetCounter(TIM2);
// =============================================================
			//len = rx_len;
			len					=			USART_RX_STA&0x3f; 				//得到此次接收到的数据长度
			SelectCom 	= 	  USART_RX_BUF[2];					//发送的串口号
			datalen 		=			USART_RX_BUF[3];					//发送数据长度
			
			if(len<37)
			{
				i=0;
			}
			
			USART_RX_STA=0;
		
			
				for(i=0; i<len; i++) 											//串口数据入缓冲区
				{
					p_send[i] = USART_RX_BUF[i];
					
				}
			
			if(p_send[0] == 0x55 && p_send[1] == 0xaa)		//帧头校验
			{
				for(i=0; i<len-1; i++) 						//获取帧尾校验和
				{
					frame_tail += p_send[i];
		
				}
			}
			else
			{
				frame_tail = 0;
				USART_RX_STA = 0;
				return;
			}
			

			
			if((frame_tail&0xff) == p_send[len-1]) 					//帧尾校验
			{
				for(t=4;t<len-1;t++){CH438_Data[t-4] = p_send[t];}	//装入发送数据
					
				
				
				CH438_SendDatas(SelectCom, CH438_Data, datalen); //检验到帧头，转发消息至CH438
				frame_tail = 0;
						
			}
			else
			{ 
				USART_RX_STA=0;
				frame_tail = 0;
				return;
			} 
		
		clear = USART1->SR;
		clear = USART1->DR;
		
// ============ debug时使用 ===============================================				
//				//获取“接收->转发”这一段的延时
//				TIM_Cmd(TIM2,DISABLE);
//			time_stop = TIM_GetCounter(TIM2);
//				time_delay = time_stop - time_begin;
//			TIM_SetCounter(TIM2, 0x00);
//				printf("Receive -> Forward_Ch438 runtime: %d us \n", time_delay);

// ========================================================================	
		}

		
}


u8 flag = 0;

void USART1_IRQHandler(void)                	//串口1中断服务程序
	{
		//uint8_t i;
		uint8_t Res;

		
		
#if SYSTEM_SUPPORT_OS 		//如果SYSTEM_SUPPORT_OS为真，则需要支持OS.
	OSIntEnter();    
#endif
		if(USART_RX_STA == 0)
					{
						TIM_Cmd(TIM2, ENABLE); //计算延时
						time_begin = TIM_GetCounter(TIM2);
					}
					
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
		{
		Res =USART_ReceiveData(USART1);	//读取接收到的数据
			
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);

		if((USART_RX_STA&0x8000)==0)//接收未完成
			{
			if(USART_RX_STA&0x4000)//接收到了0x0d
				{
				if(Res!=0x0a)
					USART_RX_STA=0;//接收错误,重新开始
				else

					USART_RX_STA|=0x8000;	//接收完成了 
					{
						TIM_Cmd(TIM2,DISABLE);
			time_stop = TIM_GetCounter(TIM2);
				time_delay = time_stop - time_begin;
				TIM_SetCounter(TIM2, 0x00);
				printf("Receive time: %d us \n", time_delay);
					}

					
				}
			else //还没收到0X0D
				{	
				if(Res==0x0d)
					{	
					USART_RX_STA|=0x4000;
					}
				else
					{
					USART_RX_BUF[USART_RX_STA&0X3FFF]=Res ;
					USART_RX_STA++;
					if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;//接收数据错误,重新开始接收	  
					}		 
				}
			}   		 
     } 
#if SYSTEM_SUPPORT_OS 	//如果SYSTEM_SUPPORT_OS为真，则需要支持OS.
	OSIntExit();  											 
#endif
} 



PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART */
  USART_SendData(Debug_UART, (uint8_t) ch);

  /* Loop until the end of transmission */
  while (USART_GetFlagStatus(Debug_UART, USART_FLAG_TC) == RESET)
  {}

  return ch;
}

static void USART_TX_DMA_Config(void)
{
	DMA_InitTypeDef DMA_InitStructure;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);
	DMA_DeInit(DMA1_Channel4);
	
 /*设置DMA源：内存地址&串口数据寄存器地址*/
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART1->DR;       

	/*内存地址(要传输的变量的指针)*/
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)DMA_DATA_BUF;
	
	/*方向：从外设到内存*/        
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;    
	
	/*传输大小DMA_BufferSize=SENDBUFF_SIZE*/    
	DMA_InitStructure.DMA_BufferSize = DMA_size;
	
	/*外设地址不增*/        
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; 
	
	/*内存地址自增*/
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;    
	
	/*外设数据单位*/    
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	
	/*内存数据单位 8bit*/
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;     
	
	/*DMA模式：一次传输，循环*/
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal ;     
	
	/*优先级：中*/    
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;  
	
	/*禁止内存到内存的传输    */
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	
	/*配置DMA1的4通道*/           
	DMA_Init(DMA1_Channel4, &DMA_InitStructure);    
	
	USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);

	DMA_Cmd(DMA1_Channel4, ENABLE);
	
	DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);

}

void DMA_USART_Send(u8* data, u16 size)
{
	DMA_Cmd(DMA1_Channel4, DISABLE);
	memcpy(DMA_DATA_BUF, data, size);
	while (DMA_GetCurrDataCounter(DMA1_Channel4));  // 检查DMA发送通道内是否还有数据
	DMA_SetCurrDataCounter(DMA1_Channel4, size);   // 重新写入要传输的数据数量
	DMA_Cmd(DMA1_Channel4, ENABLE);     // 启动DMA发送
}

