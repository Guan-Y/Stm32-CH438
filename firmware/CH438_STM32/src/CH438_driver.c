/********************************** (C) COPYRIGHT ******************************
* File Name          : CH438_driver.c
* Author             : WCH
* Version            : 
* Date               : 2014/9/11
* Description        :ch438控制驱动
*******************************************************************************/

#include "globalhead.h"

#define Fpclk    	  1843200         /* 定义内部时钟频率,默认外部晶振的12分频    */
#define MaxRecvLen    128         	  /* 接收缓冲区大小    */

#define com_seed	0
#define com_1		1
#define com_2		2
#define com_3		3
#define com_4		4
#define com_amp		5

uint8_t Read_BackLight_I[] 	= 	{0xef, 0xef, 0x03, 0xff, 0x03, 0xe3};
uint8_t Read_Driving_I[] 	= 	{0xef, 0xef, 0x03, 0xff, 0x01, 0xe1};
uint8_t Read_Temperature[] 	= 	{0xef, 0xef, 0x03, 0xff, 0x02, 0xe2};

const unsigned char offsetadd[] = {0x00,0x10,0x20,0x30,0x08,0x18,0x28,0x38,};		/* 串口号的偏移地址 */
const unsigned char Interruptnum[] = {0x01,0x02,0x04,0x08,0x10,0x20,0x40,0x80,};	/* SSR寄存器中断号对应值 */

unsigned char Revbuff[MaxRecvLen];		/* 接收缓存区 */
unsigned char RevLen;					/* 接收计数 */
unsigned char DataSendbuff[MaxRecvLen]; /* 发送数据缓存*/
unsigned char SendLen;					/* 发送数据计数*/
unsigned char INT_Com;

extern u8 Get_EXTI;

extern u8 ERR_OCCUR;;
extern u16 time_delay;
extern u16 time_begin;
extern u16 time_stop;

static void CH438_TranConfig(unsigned char num);								/* 串口数据格式配置及FIFO */
static void CH438_SetBandrate(unsigned char num, unsigned long value);			/* 设置串口波特率 */

/**
  * Function Name  : CH438_CheckIIR()
  * Description    : 读IIR寄存器
  * Input          : 串口号（0-7）
  * Return         : IIR寄存器值
  */
unsigned char CH438_CheckIIR(unsigned char num)
{
	unsigned char value;
	
	value = CH438ReadReg( offsetadd[num] | REG_IIR_ADDR );
	return value;
}

/**
  * Function Name  : CH438_CloseSeril()
  * Description    : 关闭串口
  * Input          : 串口号（0-7）
  * Return         : None
  */
void CH438_CloseSeril(unsigned char num)
{
	CH438WriteReg(offsetadd[num]|REG_IER_ADDR, BIT_IER_LOWPOWER);
}

/**
  * Function Name  : CH438_CloseALLSeril()
  * Description    : 关闭所有串口
  * Input          : None
  * Return         : None
  */
void CH438_CloseALLSeril(void)
{
	CH438WriteReg(offsetadd[0]|REG_IER_ADDR, BIT_IER_LOWPOWER|BIT_IER_SLP);
}

/**
  * Function Name  : CH438_ResetSeril()
  * Description    : 复位串口
  * Input          : 串口号（0-7）
  * Return         : None
  */
void CH438_ResetSeril(unsigned char num)
{
	CH438WriteReg(offsetadd[num]|REG_IER_ADDR, BIT_IER_RESET);
}

/**
  * Function Name  : CH438_UARTInit()
  * Description    : 串口初始化
  * Input          : 串口号（0-7）
  * Return         : None
  */
void CH438_UARTInit(unsigned char num)
{
	//，波特率为19200
		switch(num)
		{
			case com_3:
				CH438_SetBandrate(num, 19200);	/* CH438串口波特率设置 */
				CH438_TranConfig(num); 					/* CH438串口数据格式配置及FIFO大小 */	
				break;
			case com_amp:
				CH438_SetBandrate(num, 4800);	/* CH438串口波特率设置 */
				CH438_TranConfig(num); 					/* CH438串口数据格式配置及FIFO大小 */	
				break;
			default:
				CH438_SetBandrate(num, 19200);	/* CH438串口波特率设置 */
				CH438_TranConfig(num); 					/* CH438串口数据格式配置及FIFO大小 */	
				break;		
		}

	
}

/**
  * Function Name  : CH438_SetBandrate()
  * Description    : 设置串口波特率
  * Input          : 串口号（0-7）;波特率值
  * Return         : None
  */
void CH438_SetBandrate(unsigned char num, unsigned long value)
{
	uint8_t dlab=0;
	uint16_t bandspeed;
	
	dlab = CH438ReadReg(offsetadd[num]|REG_LCR_ADDR);
	dlab |= 0x80;		//置LCR寄存器DLAB位为1
	CH438WriteReg(offsetadd[num]|REG_LCR_ADDR, dlab);
	
	bandspeed = Fpclk/16/value;
	CH438WriteReg(offsetadd[num]|REG_DLL_ADDR, (uint8_t)bandspeed);
	CH438WriteReg(offsetadd[num]|REG_DLM_ADDR, (uint8_t)(bandspeed>>8));
	
#if	DEBUG_EN	
	//printf("bandrate: %x\n", bandspeed);
	//printf("DLM: %x\n", CH438ReadReg(offsetadd[num]|REG_DLM_ADDR));
	//printf("DLL: %x\n", CH438ReadReg(offsetadd[num]|REG_DLL_ADDR));
#endif	
	
	dlab &= 0x7F;		//置IIR寄存器DLAB位为0
	CH438WriteReg(offsetadd[num]|REG_LCR_ADDR, dlab);
}

/**
  * Function Name  : CH438_SendDatas()
  * Description    : 查询方式:发送数据
  * Input          : 串口号（0-7）; 数据首地址; 数量
  * Return         : None
  */
void CH438_SendDatas(unsigned char num, unsigned char* sendbuff,unsigned char len)
{
	// ============  debug时使用 ===================================
//			TIM_Cmd(TIM2, ENABLE); //计算延时
//			time_begin = TIM_GetCounter(TIM2);
// =============================================================
	while(len)
	{		
//if((CH438ReadReg(offsetadd[num]|REG_LSR_ADDR)&BIT_LSR_TEMT))	    //LSR->THRE==1  保持寄存器空
//{
		CH438WriteReg(offsetadd[num]|REG_THR_ADDR, *sendbuff++);
		len--;
//}
	}
								// ============ debug时使用 ===============================================				
//				//获取“接收->转发”这一段的延时
//				TIM_Cmd(TIM2,DISABLE);
//				time_stop = TIM_GetCounter(TIM2);
//				TIM_SetCounter(TIM2, 0x00);
//				time_delay = time_stop - time_begin;
				//printf("CH438 Send time: %d us \n", time_delay);		
	
}

/**
  * Function Name  : CH438_RecvDatas()
  * Description    : 查询方式：接受数据
  * Input          : 串口号（0-7）; 存储首地址
  * Return         : 接受数量
  */
unsigned char CH438_RecvDatas(unsigned char num, unsigned char* revbuff)
{
	
	
	uint8_t len=0;
	uint8_t *p_rev;
	
	p_rev = revbuff;
// ============  debug时使用 ===================================
//			TIM_Cmd(TIM2, ENABLE); //计算延时
//			time_begin = TIM_GetCounter(TIM2);
// =============================================================
	while( ( CH438ReadReg( offsetadd[num]|REG_LSR_ADDR ) & BIT_LSR_DATARDY ) == 0 );    /*等待数据准备好 */
	while((CH438ReadReg(offsetadd[num]|REG_LSR_ADDR)&BIT_LSR_DATARDY))	//LSR->DATARDY==1
	{
		*p_rev = CH438ReadReg(offsetadd[num]|REG_RBR_ADDR);
		p_rev++;
		len++;
	}
	
// ============ debug时使用 ===============================================				
//				//获取“接收->转发”这一段的延时
//				TIM_Cmd(TIM2,DISABLE);
//				time_stop = TIM_GetCounter(TIM2);
//				TIM_SetCounter(TIM2, 0x00);
//				time_delay = time_stop - time_begin;
				//printf("CH438 Receive time: %d us \n", time_delay);		
	
	return len;
}

/**
  * Function Name  : CH438_TranConfig()
  * Description    : 串口数据格式配置及FIFO
  * Input          : 串口号（0-7）
  * Return         : None
  */
void CH438_TranConfig(unsigned char num)
{	
	/* 发送数据格式:8位数据，无校验，1个停止位  */
	CH438WriteReg(offsetadd[num]|REG_LCR_ADDR, BIT_LCR_WORDSZ1|BIT_LCR_WORDSZ0);
	/* 设置FIFO模式，触发点为112字节 */ 
	CH438WriteReg(offsetadd[num]|REG_FCR_ADDR, BIT_FCR_RECVTG0|BIT_FCR_RECVTG1|BIT_FCR_FIFOEN|BIT_FCR_TFIFORST|BIT_FCR_RFIFORST);	
}

/**
  * Function Name  : CH438_IntConfig()
  * Description    : 串口中断开启
  * Input          : 串口号（0-7）
  * Return         : None
  */
void CH438_INTConfig(unsigned char num)
{	
	/* 注意: CH438打开BIT_IER_IETHRE中断(0->1),会产生一个发生空中断 */	
	CH438WriteReg(offsetadd[num]|REG_IER_ADDR, BIT_IER_IELINES|BIT_IER_IERECV);
	CH438WriteReg(offsetadd[num]|REG_MCR_ADDR, BIT_MCR_OUT2);//可以产生一个实际的中断	
}

/**
  * Function Name  : CH438_IntDisable()
  * Description    : 串口中断关闭
  * Input          : 串口号（0-7）
  * Return         : None
  */
void CH438_INTDisableAll(void)
{	
	unsigned char num;
	for( num = 0; num<=7; num++){
	/* 注意: CH438打开BIT_IER_IETHRE中断(0->1),会产生一个发生空中断 */	
	CH438WriteReg(offsetadd[num]|REG_IER_ADDR, 0x00);
	};
}


/**
  * Function Name  : CH438_AutoHFCtrl()
  * Description    : 硬件自动流开启
  * Input          : 串口号（0-7）
  * Return         : None
  */
void CH438_AutoHFCtrl(unsigned char num)
{
    CH438WriteReg( offsetadd[num]|REG_MCR_ADDR, BIT_MCR_AFE | BIT_MCR_OUT2 | BIT_MCR_RTS );/* 设置MCR寄存器的AFE和RTS为1 */
}

/**
  * Function Name  : DELAY()
  * Description    : 延时函数
  * Input          : None
  * Return         : None
  */
void DELAY(unsigned char time)
{
	unsigned char	i, j, c;
	for ( i = time; i != 0; i -- ) for ( j = 200; j != 0; j -- ) c+=3;
}

/**
  * Function Name  : CH438REG_RWTEST()
  * Description    : 测试438寄存器读写
  * Input          : 串口号（0-7）
  * Return         : None
  */
void CH438_RegTEST(unsigned char num)
{
#if  DEBUG_EN	
	printf("current test serilnum: %02x\n",(unsigned short)offsetadd[num]);
	printf("IER: %02x\n",(unsigned short)CH438ReadReg(offsetadd[num] | REG_IER_ADDR));//读IER
	printf("IIR: %02x\n",(unsigned short)CH438ReadReg(offsetadd[num] | REG_IIR_ADDR));//读IIR
	printf("LCR: %02x\n",(unsigned short)CH438ReadReg(offsetadd[num] | REG_LCR_ADDR));//读LCR
	printf("MCR: %02x\n",(unsigned short)CH438ReadReg(offsetadd[num] | REG_MCR_ADDR));//读MCR
	printf("LSR: %02x\n",(unsigned short)CH438ReadReg(offsetadd[num] | REG_LSR_ADDR));//读LSR
	printf("MSR: %02x\n",(unsigned short)CH438ReadReg(offsetadd[num] | REG_MSR_ADDR));//读MSR
	CH438WriteReg(offsetadd[num] | REG_SCR_ADDR, 0x78);
	printf("SCR: %02x\n",(unsigned short)CH438ReadReg(offsetadd[num] | REG_SCR_ADDR));//读SCR
#endif
}


/* 设置指定串口号*/
void SetSerial(unsigned char SelectCom )
{
	CH438_INTDisableAll();
	CH438_INTConfig(SelectCom);
	SelectCom = SelectCom;
}


/*处理返回数据									  */
/*返回上位机格式： AA 55 串口号 数据长度 数据 校验和 */
unsigned char Load_data(uint8_t rev_len, uint8_t rev_com, uint8_t* data_send, uint8_t* data_rev)
{
	uint8_t frame_tail = 0;
	uint8_t i;
	uint8_t* p_s;
	uint8_t* p_r;
	uint8_t len = 4+rev_len+1+2;
	
	p_s = &data_send[0];
	p_r  = &data_rev[0];
	//装入帧头以及数据帧信息
	*p_s = 0xaa;
	p_s ++;
	*p_s = 0x55;
	p_s ++;
	*p_s = rev_com;
	p_s ++;
	*p_s = rev_len;
	p_s ++;
	
	frame_tail = 0xaa + 0x55 + rev_com + rev_len;
	
	//装入发送数据
	for(i=0; i<rev_len; i++)
	{
		*p_s = *p_r;
		frame_tail += *p_s;
		p_s++;
		p_r ++;
	
	}
	
	//装入帧尾
	*p_s = frame_tail;
	
	p_s++;
	
	*p_s = 0x0d;
	
	p_s++;
	*p_s = 0x0a;
	
	p_s++;
	
	return len;
	
}

void Send2PC(void)
{
	u8 t;
	if(RevLen<30)
	{
		t = 0;
		printf("length error");
	}
	// ============  debug时使用 ===================================
//			TIM_Cmd(TIM2, ENABLE); //计算延时
//			time_begin = TIM_GetCounter(TIM2);
// =============================================================
	SendLen = Load_data(RevLen,INT_Com, DataSendbuff, Revbuff);
	// ============ debug时使用 ===============================================				
//				//获取“接收->转发”这一段的延时
				TIM_Cmd(TIM2,DISABLE);
				time_stop = TIM_GetCounter(TIM2);
				time_delay = time_stop - time_begin;
				//printf("Send data load time: %d us \n", time_delay);		
	
	
	// ============  debug时使用 ===================================
			TIM_Cmd(TIM2, ENABLE); //计算延时
			time_begin = TIM_GetCounter(TIM2);
// =============================================================
//	for(t = 0; t< SendLen; t++)
//		{
//			USART_SendData(USART1, DataSendbuff[t]);
//			while(USART_GetFlagStatus(USART1, USART_FLAG_TC)==RESET);
//		}			
		DMA_USART_Send(DataSendbuff, SendLen);
		Get_EXTI = 0;
		
// ============ debug时使用 ===============================================				
//				//获取“接收->转发”这一段的延时
//				TIM_Cmd(TIM2,DISABLE);
//				time_stop = TIM_GetCounter(TIM2);
//				time_delay = time_stop - time_begin;
				//printf("send to pc time: %d us \n", time_delay);		
}


/**
  * Function Name  : CH438InterruptFun()
  * Description    : ch438中断方式处理
  * Input          : None
  * Return         : None
  */
void CH438InterruptFun (void)
{
	uint8_t gInterruptStatus;		/* 全局中断状态 */
	uint8_t InterruptStatus;		/* 独立串口中断状态 */	
	uint8_t i;						/* 串口号*/
//	uint8_t t;

		
	
//******* 自写逻辑 *******//
	
	gInterruptStatus = CH438ReadReg( REG_SSR_ADDR );

	if(!gInterruptStatus)
		return ;
for(i = 0; i<8; i++){
	if( gInterruptStatus & Interruptnum[i] )    /* 检测哪个串口发生中断 */
		{
			InterruptStatus = CH438ReadReg( offsetadd [i] | REG_IIR_ADDR ) & 0x0f;    /* 读串口的中断状态 */	
			switch( InterruptStatus )
			{
				case INT_NOINT:			/* 没有中断 */					
					break;
				case INT_THR_EMPTY:		/* THR空中断 */						
					break;
				case INT_RCV_OVERTIME:	/* 接收超时中断 */
				
					if(!Get_EXTI){
						RevLen = CH438_RecvDatas(i, Revbuff);
						INT_Com = i;
						Get_EXTI = 1;
						//Send2PC();
					}
					return;					

				case INT_RCV_SUCCESS:	/* 接收数据可用中断 */
					if(!Get_EXTI){
						RevLen = CH438_RecvDatas(i, Revbuff);
						INT_Com = i;
						Get_EXTI = 1;
						//Send2PC();
					}
					return;				
					
				case INT_RCV_LINES:		/* 接收线路状态中断 */
					CH438ReadReg( offsetadd[i] | REG_LSR_ADDR );
					ERR_OCCUR = 1;
					break;
				case INT_MODEM_CHANGE:	/* MODEM输入变化中断 */
					CH438ReadReg( offsetadd[i] | REG_MSR_ADDR );
					break;
				default:
					break;

			}
		}
	}
	
}


uint8_t TIM2_status = 0;


void TIM2_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM2, TIM_IT_Update)!= RESET)
	{

	}
}   




