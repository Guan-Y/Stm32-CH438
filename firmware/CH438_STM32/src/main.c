/********************************** (C) COPYRIGHT ******************************
* File Name          : main.c
* Author             : WCH
* Version            : 
* Date               : 2014/9/11
* Description        : ������
*******************************************************************************/

#include "globalhead.h"

#define SerilNUM 	3	//���ں�

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
	CH438PortConfig();		   				/* CH438�ӿ����� */
	CH438_INIT();
	stm32_XINT_Config();					/* ������Ƭ���ж� */

	
	
	
	stm32_TIM2_Config();					/*������ʱ��*/
 


	while(1)
	{	

		Forward_CH438();	

		if(Get_EXTI)
		{	
//TIM_Cmd(TIM2, ENABLE); //������ʱ
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
	
	CH438_ResetSeril(0);				/* CH438����1��λ */
	CH438_ResetSeril(1);				/* CH438����1��λ */
	CH438_ResetSeril(2);				/* CH438����1��λ */
	CH438_ResetSeril(3);				/* CH438����1��λ */
	CH438_ResetSeril(4);				/* CH438����1��λ */
	CH438_ResetSeril(5);				/* CH438����1��λ */
	CH438_ResetSeril(6);				/* CH438����1��λ */
	CH438_ResetSeril(7);				/* CH438����1��λ */
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
