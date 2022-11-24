/********************************** (C) COPYRIGHT ******************************
* File Name          : debug.c
* Author             : WCH
* Version            : 
* Date               : 2014/9/11
* Description        : stm32�д��ڵ�������
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

#define Amp_com		5  //���ż����ں�

#define DMA_size	256

static void D_GPIO_Config(void);
static void D_UART_Config(void);
static void D_NVIC_Config(void);
static void USART_TX_DMA_Config(void);
void DMA_USART_Send(u8* data, u16 size);

u8 DMA_DATA_BUF[DMA_size];


uint8_t 	USART_RX_BUF[USART_REC_LEN];
uint16_t 	USART_RX_STA;       //����״̬���
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
//AA 55 06 04 EF 04 01 EF EC 0D 0A ����

//�˿�1
u8 debug_send_middle[] 			= {0xaa, 0x55, 0x01, 0x11, 0x01, 0x03, 0x0c, 0x00, 0x01, 0xff, 0xff, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0x03, 0xae, 0x33, 0xc7, 0xd0, 0x0d, 0x0a}; //ģ���һ������
//�˿�3
u8 debug_send_long[]				= {0xAA, 0x55, 0x03, 0x1B, 0x01, 0x03, 0x16, 0x00, 0x08, 0x0C, 0x85, 0x03, 0x5C, 0x03, 0x58, 0x03, 0x56, 0xA0, 0xAB, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xE8, 0x00, 0x01, 0xE2, 0xAB, 0xA7, 0x0D, 0x0A};//ģ�������������
//�˿�6
u8 debug_send_short[] 			= {0xAA, 0x55, 0x06, 0x05, 0xEF, 0x01, 0x00, 0x00, 0xEF, 0xE9, 0x0D, 0x0A};	//ģ������������
//�˿�3
u8 debug_send_wrong_long[] 	= {0xAA, 0x55, 0x03, 0x1B, 0x01, 0x03, 0x16, 0x00, 0x00, 0x0A, 0x80, 0x03, 0xBC, 0x03, 0xB4, 0x03, 0xAC, 0xA0, 0xAB, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xE8, 0x00, 0x01, 0xD2, 0x09, 0xF8, 0x0d, 0x0a};//ģ�����ĳ�����
//�˿�6
u8 debug_open1[]						= {0xAA, 0x55, 0x06, 0x04, 0xEF, 0x04, 0x01, 0xEF, 0xEC, 0x0D, 0x0A};
//�˿�0
u8 debug_open2[]						=	{0xAA, 0x55, 0x00, 0x11, 0x01, 0x03, 0x0C, 0x00, 0x64, 0xFF, 0xFF, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0x03, 0xAE, 0xDC, 0x0A, 0x1E, 0x0D, 0x0A};
//�˿�2
u8 debug_open3[]						=	{0xaa, 0x55, 0x02, 0x11, 0x01, 0x03, 0x0c, 0x00, 0x01, 0xff, 0xff, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0x03, 0xae, 0x33, 0xc7, 0xd1, 0x0d, 0x0a};
//�˿�4
u8 debug_open4[]						= {0xAA, 0x55, 0x04, 0x04, 0xEF, 0x04, 0x01, 0xEF, 0xEA, 0x0D, 0x0A};
//�˿�5
u8 debug_open5[]						= {0xAA, 0x55, 0x05, 0x04, 0xEF, 0x04, 0x01, 0xEF, 0xEB, 0x0D, 0x0A};
//�˿�7
u8 debug_open7[]						= {0xAA, 0x55, 0x07, 0x04, 0xEF, 0x04, 0x01, 0xEF, 0xED, 0x0D, 0x0A};


u8 len_middle = 24;
u8 len_long 	= 34;
u8 len_short 	= 12;
u8 len_open 	= 11;
	
	/**
  * Function Name  : DEBUG_Config()
  * Description    : ���Դ��ڳ�ʼ��
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

////////  �������ڲ�����  ///////////////////////////////////////////

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
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1 ;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
	
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_IRQn; //Ƕ��ͨ��ΪDMA1_Channel4_IRQn
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; //��ռ���ȼ�Ϊ 2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3; //��Ӧ���ȼ�Ϊ 3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //ͨ���ж�ʹ��
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
			// ============  debugʱʹ�� ===================================
//			TIM_Cmd(TIM2, ENABLE); //������ʱ
//			time_begin = TIM_GetCounter(TIM2);
// =============================================================
			//len = rx_len;
			len					=			USART_RX_STA&0x3f; 				//�õ��˴ν��յ������ݳ���
			SelectCom 	= 	  USART_RX_BUF[2];					//���͵Ĵ��ں�
			datalen 		=			USART_RX_BUF[3];					//�������ݳ���
			
			if(len<37)
			{
				i=0;
			}
			
			USART_RX_STA=0;
		
			
				for(i=0; i<len; i++) 											//���������뻺����
				{
					p_send[i] = USART_RX_BUF[i];
					
				}
			
			if(p_send[0] == 0x55 && p_send[1] == 0xaa)		//֡ͷУ��
			{
				for(i=0; i<len-1; i++) 						//��ȡ֡βУ���
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
			

			
			if((frame_tail&0xff) == p_send[len-1]) 					//֡βУ��
			{
				for(t=4;t<len-1;t++){CH438_Data[t-4] = p_send[t];}	//װ�뷢������
					
				
				
				CH438_SendDatas(SelectCom, CH438_Data, datalen); //���鵽֡ͷ��ת����Ϣ��CH438
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
		
// ============ debugʱʹ�� ===============================================				
//				//��ȡ������->ת������һ�ε���ʱ
//				TIM_Cmd(TIM2,DISABLE);
//			time_stop = TIM_GetCounter(TIM2);
//				time_delay = time_stop - time_begin;
//			TIM_SetCounter(TIM2, 0x00);
//				printf("Receive -> Forward_Ch438 runtime: %d us \n", time_delay);

// ========================================================================	
		}

		
}


u8 flag = 0;

void USART1_IRQHandler(void)                	//����1�жϷ������
	{
		//uint8_t i;
		uint8_t Res;

		
		
#if SYSTEM_SUPPORT_OS 		//���SYSTEM_SUPPORT_OSΪ�棬����Ҫ֧��OS.
	OSIntEnter();    
#endif
		if(USART_RX_STA == 0)
					{
						TIM_Cmd(TIM2, ENABLE); //������ʱ
						time_begin = TIM_GetCounter(TIM2);
					}
					
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //�����ж�(���յ������ݱ�����0x0d 0x0a��β)
		{
		Res =USART_ReceiveData(USART1);	//��ȡ���յ�������
			
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);

		if((USART_RX_STA&0x8000)==0)//����δ���
			{
			if(USART_RX_STA&0x4000)//���յ���0x0d
				{
				if(Res!=0x0a)
					USART_RX_STA=0;//���մ���,���¿�ʼ
				else

					USART_RX_STA|=0x8000;	//��������� 
					{
						TIM_Cmd(TIM2,DISABLE);
			time_stop = TIM_GetCounter(TIM2);
				time_delay = time_stop - time_begin;
				TIM_SetCounter(TIM2, 0x00);
				printf("Receive time: %d us \n", time_delay);
					}

					
				}
			else //��û�յ�0X0D
				{	
				if(Res==0x0d)
					{	
					USART_RX_STA|=0x4000;
					}
				else
					{
					USART_RX_BUF[USART_RX_STA&0X3FFF]=Res ;
					USART_RX_STA++;
					if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;//�������ݴ���,���¿�ʼ����	  
					}		 
				}
			}   		 
     } 
#if SYSTEM_SUPPORT_OS 	//���SYSTEM_SUPPORT_OSΪ�棬����Ҫ֧��OS.
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
	
 /*����DMAԴ���ڴ��ַ&�������ݼĴ�����ַ*/
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART1->DR;       

	/*�ڴ��ַ(Ҫ����ı�����ָ��)*/
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)DMA_DATA_BUF;
	
	/*���򣺴����赽�ڴ�*/        
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;    
	
	/*�����СDMA_BufferSize=SENDBUFF_SIZE*/    
	DMA_InitStructure.DMA_BufferSize = DMA_size;
	
	/*�����ַ����*/        
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; 
	
	/*�ڴ��ַ����*/
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;    
	
	/*�������ݵ�λ*/    
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	
	/*�ڴ����ݵ�λ 8bit*/
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;     
	
	/*DMAģʽ��һ�δ��䣬ѭ��*/
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal ;     
	
	/*���ȼ�����*/    
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;  
	
	/*��ֹ�ڴ浽�ڴ�Ĵ���    */
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	
	/*����DMA1��4ͨ��*/           
	DMA_Init(DMA1_Channel4, &DMA_InitStructure);    
	
	USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);

	DMA_Cmd(DMA1_Channel4, ENABLE);
	
	DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);

}

void DMA_USART_Send(u8* data, u16 size)
{
	DMA_Cmd(DMA1_Channel4, DISABLE);
	memcpy(DMA_DATA_BUF, data, size);
	while (DMA_GetCurrDataCounter(DMA1_Channel4));  // ���DMA����ͨ�����Ƿ�������
	DMA_SetCurrDataCounter(DMA1_Channel4, size);   // ����д��Ҫ�������������
	DMA_Cmd(DMA1_Channel4, ENABLE);     // ����DMA����
}

