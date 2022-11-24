/********************************** (C) COPYRIGHT ******************************
* File Name          : interface.c
* Author             : WCH
* Version            : 
* Date               : 2014/9/11
* Description        : ch438��stm32���ӽӿ����ü��Ĵ�����д
*******************************************************************************/

#include "stm32f10x.h"
#include <stdio.h>

/*Ӳ���ӿ�*/
/*											*********************************************************		
       CH438          DIR        STM32		* f103ZET���İ���������Щ��ͬ����˲���ֱ��ʹ��ԭ����PB��
        D0-D7        <==>       PB_8~PB_15	* => PE8~15
         WR          <==>        PB_0		* => PE0
         RD          <==>        PB_1		* => PE1		
		 CS          <==>        PB_5  		* => PE2
		INT          <==>        PB_4  		* => PE3   �ⲿ�ж���
		ALE          <==>        PB_6		* => PE6
*/										  //**********************************************************					

/* ����CH438��ַ���ݸ��ö˿ڵ�Ƭ��IO�˿�ģ������ʱ��ӿ� */

#define CH438_DATA_RCC     RCC_APB2Periph_GPIOB		//���ݵ�ַʱ��
#define CH438_DATA_PORT    GPIOB //#
#define CH438_DATA_PIN     ((uint16_t)0xFF00)	    //���ݵ�ַ����  PB8-15

#define CH438_CONL_RCC     RCC_APB2Periph_GPIOB		//���ƹܽ�ʱ��
#define CH438_CONL_PORT    GPIOB//#
#define CH438_WR_PIN       GPIO_Pin_0    // WR���� 	  PB_0	##################
#define CH438_RD_PIN       GPIO_Pin_1    // RD����    PB_1	#	  PB->PE	 #
#define CH438_CS_PIN       GPIO_Pin_5    // CS����    PB_5	#	  �иĶ�		 #	
#define CH438_ALE_PIN      GPIO_Pin_6    // ALE����   PB_6	#   CS��INT�Ÿı� #
#define CH438_INT_PIN      GPIO_Pin_4    // �ж�����  PB_4	##################

static void SetInputMode(uint16_t GPIO_Pin);		/* ���ùܽ�Ϊ���뷽ʽ */
static void SetOupputMode(uint16_t GPIO_Pin);		/* ���ùܽ�Ϊ�����ʽ */
static void stm32_GPIO_Config(void);	/* ����GPIO��ʽ */
static void stm32_NVIC_Config(void);
static void stm32_IWDG_Init(void);

/**
  * Function Name  : CH438InterfaceConfig()
  * Description    : CH438�ӿ�����
  * Input          : None
  * Return         : None
  */
void CH438PortConfig(void)
{
	stm32_GPIO_Config();
	stm32_NVIC_Config();
	stm32_IWDG_Init();
}

/**
  * Function Name  : CH438WriteReg()
  * Description    : дCH438�Ĵ���
  * Input          : �Ĵ�����ַ; д����ֵ
  * Return         : None
  */
void CH438WriteReg(unsigned char add,unsigned char data)	/* дCH438 */
{
	uint16_t value;
	
	GPIO_SetBits(CH438_CONL_PORT, CH438_CS_PIN);				//CS = 1
	GPIO_SetBits(CH438_CONL_PORT, CH438_WR_PIN);  				//WR = 1
	GPIO_SetBits(CH438_CONL_PORT, CH438_RD_PIN);  				//RD = 1
	
	SetOupputMode(CH438_DATA_PIN);								//��Ϊ���ģʽ
	
	value = GPIO_ReadOutputData(CH438_DATA_PORT)&0x00ff;
	
	GPIO_Write(CH438_DATA_PORT, ((uint16_t)add<<8)|value);	   	//д��ַ
	GPIO_WriteBit(CH438_CONL_PORT, CH438_CS_PIN, Bit_RESET);    //��Ƭѡ CS = 0
	GPIO_WriteBit(CH438_CONL_PORT, CH438_ALE_PIN, Bit_SET);  	//�����ź� ALE = 1	
	GPIO_WriteBit(CH438_CONL_PORT, CH438_ALE_PIN, Bit_RESET);  	//ALE = 0
	
	value = GPIO_ReadOutputData(CH438_DATA_PORT)&0x00ff;
	GPIO_Write(CH438_DATA_PORT, ((uint16_t)data<<8)|value);	   	//д����
	GPIO_ResetBits(CH438_CONL_PORT, CH438_WR_PIN);  			//WR = 0
		
	GPIO_SetBits(CH438_CONL_PORT, CH438_WR_PIN);  				//WR = 1
	GPIO_SetBits(CH438_CONL_PORT, CH438_CS_PIN);				//CS = 1
}

/**
  * Function Name  : CH438ReadReg()
  * Description    : ��CH438�Ĵ���
  * Input          : �Ĵ�����ַ; 
  * Return         : ������ֵ
  */
unsigned char CH438ReadReg(unsigned char add)	/* ��CH438 */
{
	uint8_t value;
	
	
	GPIO_SetBits(CH438_CONL_PORT, CH438_CS_PIN);				//CS = 1
	GPIO_SetBits(CH438_CONL_PORT, CH438_WR_PIN);  				//WR = 1
	GPIO_SetBits(CH438_CONL_PORT, CH438_RD_PIN);  				//RD = 1
	
	SetOupputMode(CH438_DATA_PIN);								//��Ϊ���ģʽ
	
	value = GPIO_ReadOutputData(CH438_DATA_PORT)&0x00ff;
	
	GPIO_Write(CH438_DATA_PORT, ((uint16_t)add<<8)|value);	   	//д��ַadd
	GPIO_ResetBits(CH438_CONL_PORT, CH438_CS_PIN);              //��Ƭѡ CS = 0
	GPIO_SetBits(CH438_CONL_PORT, CH438_ALE_PIN);           	//�����ź� ALE = 1
	GPIO_ResetBits(CH438_CONL_PORT, CH438_ALE_PIN);  	        //ALE = 0

	
	SetInputMode(CH438_DATA_PIN);								//��Ϊ����ģʽ
	GPIO_ResetBits(CH438_CONL_PORT, CH438_RD_PIN);  			//RD = 0
	value = (uint8_t)(GPIO_ReadInputData(CH438_DATA_PORT)>>8);	//������
	
	GPIO_SetBits(CH438_CONL_PORT, CH438_RD_PIN);  				//RD = 1
	GPIO_SetBits(CH438_CONL_PORT, CH438_CS_PIN);				//CS = 1	
	
	return value;
}

/**
  * Function Name  : XINT_Config()
  * Description    : stm32�ⲿ�ж�����
  * Input          : None 
  * Return         : None
  */
void stm32_XINT_Config(void)
{
	EXTI_InitTypeDef CH438_exit;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);		
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource4);	
	
	CH438_exit.EXTI_Line = EXTI_Line4;							
	CH438_exit.EXTI_Mode = EXTI_Mode_Interrupt;
	CH438_exit.EXTI_Trigger = EXTI_Trigger_Falling;
	CH438_exit.EXTI_LineCmd = ENABLE;//ENABLE;
	EXTI_Init(&CH438_exit);
	
	EXTI_ClearITPendingBit(EXTI_Line4);//����жϱ�־λ			
}

////////  �������ڲ�����  ///////////////////////////////////////////
static void SetInputMode(uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef CH438_gpio;
	
	CH438_gpio.GPIO_Pin = GPIO_Pin;
	CH438_gpio.GPIO_Speed = GPIO_Speed_50MHz;
	CH438_gpio.GPIO_Mode = GPIO_Mode_IPU;		//��������
	GPIO_Init(CH438_DATA_PORT, &CH438_gpio);
}

static void SetOupputMode(uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef CH438_gpio;
	
	CH438_gpio.GPIO_Pin = GPIO_Pin;
	CH438_gpio.GPIO_Speed = GPIO_Speed_50MHz;
	CH438_gpio.GPIO_Mode = GPIO_Mode_Out_PP;	//�������
	GPIO_Init(CH438_DATA_PORT, &CH438_gpio);
}

static void stm32_GPIO_Config(void)
{
	GPIO_InitTypeDef CH438_gpio;
	
	RCC_APB2PeriphClockCmd(CH438_DATA_RCC, ENABLE);
	RCC_APB2PeriphClockCmd(CH438_CONL_RCC, ENABLE);
	
	CH438_gpio.GPIO_Pin = CH438_DATA_PIN;			/* ���ݵ�ַ�ܽ� */
	CH438_gpio.GPIO_Speed = GPIO_Speed_50MHz;
	CH438_gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;	//��������
	GPIO_Init(CH438_DATA_PORT, &CH438_gpio);
	
	CH438_gpio.GPIO_Pin = CH438_WR_PIN|CH438_RD_PIN|CH438_CS_PIN|CH438_ALE_PIN;
	CH438_gpio.GPIO_Mode = GPIO_Mode_Out_PP;		//�������
	GPIO_Init(CH438_CONL_PORT, &CH438_gpio);
	
	CH438_gpio.GPIO_Pin = CH438_INT_PIN;
	CH438_gpio.GPIO_Speed = GPIO_Speed_50MHz;
	CH438_gpio.GPIO_Mode = GPIO_Mode_IPU;			 //��������,�ⲿ�жϹܽ�
	GPIO_Init(CH438_CONL_PORT, &CH438_gpio);

	GPIO_ResetBits(CH438_CONL_PORT, CH438_ALE_PIN);    //ALE�ź�Ĭ��Ϊ��
	GPIO_SetBits(CH438_CONL_PORT, CH438_CS_PIN);       //csƬѡ�ź�Ϊ��
	GPIO_SetBits(CH438_CONL_PORT, CH438_WR_PIN);       //WRƬѡ�ź�Ϊ��
	GPIO_SetBits(CH438_CONL_PORT, CH438_RD_PIN);       //RDƬѡ�ź�Ϊ��
}

static void stm32_NVIC_Config(void)
{
	NVIC_InitTypeDef CH438_nvic;
//	NVIC_InitTypeDef Tim2_nvic;

	
	
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	CH438_nvic.NVIC_IRQChannel = EXTI4_IRQn;    	
	CH438_nvic.NVIC_IRQChannelPreemptionPriority = 1;
	CH438_nvic.NVIC_IRQChannelSubPriority = 0;
	CH438_nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&CH438_nvic);
	
//	//��ʱ���жϿ���
//	Tim2_nvic.NVIC_IRQChannel = TIM2_IRQn;
//	Tim2_nvic.NVIC_IRQChannelCmd = ENABLE;
//	Tim2_nvic.NVIC_IRQChannelPreemptionPriority = 2;
//	Tim2_nvic.NVIC_IRQChannelSubPriority = 2;
//	NVIC_Init(&Tim2_nvic);
	

}


void stm32_TIM2_Config(void)
{
	TIM_TimeBaseInitTypeDef TIM_2;	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	
	TIM_2.TIM_Period = 5000;
	TIM_2.TIM_Prescaler = 71;
	TIM_2.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_2.TIM_CounterMode = TIM_CounterMode_Up;
	
	TIM_TimeBaseInit(TIM2,&TIM_2);
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
}

static void stm32_IWDG_Init(void)
{
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
	IWDG_SetPrescaler(IWDG_Prescaler_64);
	IWDG_SetReload(5000);
	IWDG_ReloadCounter();
	IWDG_Enable();
}



