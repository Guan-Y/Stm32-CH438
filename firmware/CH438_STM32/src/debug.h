/********************************** (C) COPYRIGHT ******************************
* File Name          : debug.h
* Author             : WCH
* Version            : 
* Date               : 2014/9/11
* Description        :stm32中串口调试驱动头文件
*******************************************************************************/

#ifndef _DEBUG_UART
#define _DEBUG_UART
#define  USART_REC_LEN				128

void DEBUG_Config(void);		/* 调试串口配置 */
extern unsigned char	USART_RX_BUF[USART_REC_LEN];
extern unsigned short 	USART_RX_STA; 
void DMA_USART_Send(unsigned char* data, unsigned short size);
#endif

