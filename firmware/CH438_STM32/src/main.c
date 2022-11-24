/********************************** (C) COPYRIGHT ******************************
* File Name          : main.c
* Author             : WCH
* Version            : 
* Date               : 2014/9/11
* Description        : 主函数
*******************************************************************************/

#include "globalhead.h"

#define SerilNUM 	3	//串口号

void CH438_INIT(void);
unsigned char test[] = "Hello!!\n";
unsigned char test1[] = "!";
u32 times;
u32 d;
u8 ERR_OCCUR;
u8 Get_EXTI;

u8 int_status;
u8 i1 = 0;

u16 test_time_begin, test_time_end, test_time_delay;
u8 dma_end;
extern u16 USART_RX_STA;

int main(void)
{		
#if	DEBUG_EN	
	DEBUG_Config();
	DELAY(50);
//	printf("debug tool set!\n");
#endif
	
	DELAY(150);
	CH438PortConfig();		   				/* CH438接口配置 */
	CH438_INIT();
	stm32_XINT_Config();					/* 开启单片机中断 */

	
	
	
	stm32_TIM2_Config();					/*开启定时器*/
 


	while(1)
	{	

		Forward_CH438();	

		if(Get_EXTI)
		{	
//TIM_Cmd(TIM2, ENABLE); //计算延时
//				test_time_begin = TIM_GetCounter(TIM2);
			Send2PC();	
//			while(!dma_end);
//			dma_end = 0;
//TIM_Cmd(TIM2,DISABLE);
//				test_time_end = TIM_GetCounter(TIM2);
//				test_time_delay = test_time_end - test_time_begin;
//			TIM_SetCounter(TIM2, 0x00);
//				printf("sen2pc runtime: %d us \n", test_time_delay);
		}

		IWDG_ReloadCounter(); // feed dog
		if(ERR_OCCUR)
		{
			printf("error occur");
			CH438_INIT();
			ERR_OCCUR = 0;
		}
		

	}
}


void CH438_INIT(void)
{
	
	CH438_ResetSeril(0);				/* CH438串口1复位 */
	CH438_ResetSeril(1);				/* CH438串口1复位 */
	CH438_ResetSeril(2);				/* CH438串口1复位 */
	CH438_ResetSeril(3);				/* CH438串口1复位 */
	CH438_ResetSeril(4);				/* CH438串口1复位 */
	CH438_ResetSeril(5);				/* CH438串口1复位 */
	CH438_ResetSeril(6);				/* CH438串口1复位 */
	CH438_ResetSeril(7);				/* CH438串口1复位 */
	DELAY(50);
	
	
	CH438_UARTInit(0);
	CH438_INTConfig(0);
	
	
	CH438_UARTInit(1);
	CH438_INTConfig(1);
	
	CH438_UARTInit(2);
	CH438_INTConfig(2);
	
	CH438_UARTInit(3);
	CH438_INTConfig(3);
	
	CH438_UARTInit(4);
	CH438_INTConfig(4);
	
	CH438_UARTInit(5);
	CH438_INTConfig(5);
	
	CH438_UARTInit(6);
	CH438_INTConfig(6);
	
	CH438_UARTInit(7);
	CH438_INTConfig(7);
}
