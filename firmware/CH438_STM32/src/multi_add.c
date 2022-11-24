/********************************** (C) COPYRIGHT ******************************
* File Name          : interface.c
* Author             : WCH
* Version            : 
* Date               : 2014/9/11
* Description        : ch438与stm32连接接口配置及寄存器读写
*******************************************************************************/

#include "stm32f10x.h"
#include <stdio.h>

/*硬件接口*/
/*											*********************************************************		
       CH438          DIR        STM32		* f103ZET核心板引出脚有些不同，因此不能直接使用原来的PB口
        D0-D7        <==>       PB_8~PB_15	* => PE8~15
         WR          <==>        PB_0		* => PE0
         RD          <==>        PB_1		* => PE1		
		 CS          <==>        PB_5  		* => PE2
		INT          <==>        PB_4  		* => PE3   外部中断线
		ALE          <==>        PB_6		* => PE6
*/										  //**********************************************************					

/* 定义CH438地址数据复用端口单片机IO端口模拟总线时序接口 */

#define CH438_DATA_RCC     RCC_APB2Periph_GPIOB		//数据地址时钟
#define CH438_DATA_PORT    GPIOB //#
#define CH438_DATA_PIN     ((uint16_t)0xFF00)	    //数据地址引脚  PB8-15

#define CH438_CONL_RCC     RCC_APB2Periph_GPIOB		//控制管脚时钟
#define CH438_CONL_PORT    GPIOB//#
#define CH438_WR_PIN       GPIO_Pin_0    // WR引脚 	  PB_0	##################
#define CH438_RD_PIN       GPIO_Pin_1    // RD引脚    PB_1	#	  PB->PE	 #
#define CH438_CS_PIN       GPIO_Pin_5    // CS引脚    PB_5	#	  有改动		 #	
#define CH438_ALE_PIN      GPIO_Pin_6    // ALE引脚   PB_6	#   CS和INT脚改变 #
#define CH438_INT_PIN      GPIO_Pin_4    // 中断引脚  PB_4	##################

static void SetInputMode(uint16_t GPIO_Pin);		/* 设置管脚为输入方式 */
static void SetOupputMode(uint16_t GPIO_Pin);		/* 设置管脚为输出方式 */
static void stm32_GPIO_Config(void);	/* 设置GPIO方式 */
static void stm32_NVIC_Config(void);
static void stm32_IWDG_Init(void);

/**
  * Function Name  : CH438InterfaceConfig()
  * Description    : CH438接口配置
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
  * Description    : 写CH438寄存器
  * Input          : 寄存器地址; 写入数值
  * Return         : None
  */
void CH438WriteReg(unsigned char add,unsigned char data)	/* 写CH438 */
{
	uint16_t value;
	
	GPIO_SetBits(CH438_CONL_PORT, CH438_CS_PIN);				//CS = 1
	GPIO_SetBits(CH438_CONL_PORT, CH438_WR_PIN);  				//WR = 1
	GPIO_SetBits(CH438_CONL_PORT, CH438_RD_PIN);  				//RD = 1
	
	SetOupputMode(CH438_DATA_PIN);								//设为输出模式
	
	value = GPIO_ReadOutputData(CH438_DATA_PORT)&0x00ff;
	
	GPIO_Write(CH438_DATA_PORT, ((uint16_t)add<<8)|value);	   	//写地址
	GPIO_WriteBit(CH438_CONL_PORT, CH438_CS_PIN, Bit_RESET);    //打开片选 CS = 0
	GPIO_WriteBit(CH438_CONL_PORT, CH438_ALE_PIN, Bit_SET);  	//锁存信号 ALE = 1	
	GPIO_WriteBit(CH438_CONL_PORT, CH438_ALE_PIN, Bit_RESET);  	//ALE = 0
	
	value = GPIO_ReadOutputData(CH438_DATA_PORT)&0x00ff;
	GPIO_Write(CH438_DATA_PORT, ((uint16_t)data<<8)|value);	   	//写数据
	GPIO_ResetBits(CH438_CONL_PORT, CH438_WR_PIN);  			//WR = 0
		
	GPIO_SetBits(CH438_CONL_PORT, CH438_WR_PIN);  				//WR = 1
	GPIO_SetBits(CH438_CONL_PORT, CH438_CS_PIN);				//CS = 1
}

/**
  * Function Name  : CH438ReadReg()
  * Description    : 读CH438寄存器
  * Input          : 寄存器地址; 
  * Return         : 读出数值
  */
unsigned char CH438ReadReg(unsigned char add)	/* 读CH438 */
{
	uint8_t value;
	
	
	GPIO_SetBits(CH438_CONL_PORT, CH438_CS_PIN);				//CS = 1
	GPIO_SetBits(CH438_CONL_PORT, CH438_WR_PIN);  				//WR = 1
	GPIO_SetBits(CH438_CONL_PORT, CH438_RD_PIN);  				//RD = 1
	
	SetOupputMode(CH438_DATA_PIN);								//设为输出模式
	
	value = GPIO_ReadOutputData(CH438_DATA_PORT)&0x00ff;
	
	GPIO_Write(CH438_DATA_PORT, ((uint16_t)add<<8)|value);	   	//写地址add
	GPIO_ResetBits(CH438_CONL_PORT, CH438_CS_PIN);              //打开片选 CS = 0
	GPIO_SetBits(CH438_CONL_PORT, CH438_ALE_PIN);           	//锁存信号 ALE = 1
	GPIO_ResetBits(CH438_CONL_PORT, CH438_ALE_PIN);  	        //ALE = 0

	
	SetInputMode(CH438_DATA_PIN);								//设为输入模式
	GPIO_ResetBits(CH438_CONL_PORT, CH438_RD_PIN);  			//RD = 0
	value = (uint8_t)(GPIO_ReadInputData(CH438_DATA_PORT)>>8);	//读数据
	
	GPIO_SetBits(CH438_CONL_PORT, CH438_RD_PIN);  				//RD = 1
	GPIO_SetBits(CH438_CONL_PORT, CH438_CS_PIN);				//CS = 1	
	
	return value;
}

/**
  * Function Name  : XINT_Config()
  * Description    : stm32外部中断配置
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
	
	EXTI_ClearITPendingBit(EXTI_Line4);//清除中断标志位			
}

////////  以下是内部函数  ///////////////////////////////////////////
static void SetInputMode(uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef CH438_gpio;
	
	CH438_gpio.GPIO_Pin = GPIO_Pin;
	CH438_gpio.GPIO_Speed = GPIO_Speed_50MHz;
	CH438_gpio.GPIO_Mode = GPIO_Mode_IPU;		//上拉输入
	GPIO_Init(CH438_DATA_PORT, &CH438_gpio);
}

static void SetOupputMode(uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef CH438_gpio;
	
	CH438_gpio.GPIO_Pin = GPIO_Pin;
	CH438_gpio.GPIO_Speed = GPIO_Speed_50MHz;
	CH438_gpio.GPIO_Mode = GPIO_Mode_Out_PP;	//推挽输出
	GPIO_Init(CH438_DATA_PORT, &CH438_gpio);
}

static void stm32_GPIO_Config(void)
{
	GPIO_InitTypeDef CH438_gpio;
	
	RCC_APB2PeriphClockCmd(CH438_DATA_RCC, ENABLE);
	RCC_APB2PeriphClockCmd(CH438_CONL_RCC, ENABLE);
	
	CH438_gpio.GPIO_Pin = CH438_DATA_PIN;			/* 数据地址管脚 */
	CH438_gpio.GPIO_Speed = GPIO_Speed_50MHz;
	CH438_gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;	//悬浮输入
	GPIO_Init(CH438_DATA_PORT, &CH438_gpio);
	
	CH438_gpio.GPIO_Pin = CH438_WR_PIN|CH438_RD_PIN|CH438_CS_PIN|CH438_ALE_PIN;
	CH438_gpio.GPIO_Mode = GPIO_Mode_Out_PP;		//推挽输出
	GPIO_Init(CH438_CONL_PORT, &CH438_gpio);
	
	CH438_gpio.GPIO_Pin = CH438_INT_PIN;
	CH438_gpio.GPIO_Speed = GPIO_Speed_50MHz;
	CH438_gpio.GPIO_Mode = GPIO_Mode_IPU;			 //上拉输入,外部中断管脚
	GPIO_Init(CH438_CONL_PORT, &CH438_gpio);

	GPIO_ResetBits(CH438_CONL_PORT, CH438_ALE_PIN);    //ALE信号默认为低
	GPIO_SetBits(CH438_CONL_PORT, CH438_CS_PIN);       //cs片选信号为高
	GPIO_SetBits(CH438_CONL_PORT, CH438_WR_PIN);       //WR片选信号为高
	GPIO_SetBits(CH438_CONL_PORT, CH438_RD_PIN);       //RD片选信号为高
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
	
//	//定时器中断开启
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



