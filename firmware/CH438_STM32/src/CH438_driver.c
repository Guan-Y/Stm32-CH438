/********************************** (C) COPYRIGHT ******************************
* File Name          : CH438_driver.c
* Author             : WCH
* Version            : 
* Date               : 2014/9/11
* Description        :ch438��������
*******************************************************************************/

#include "globalhead.h"

#define Fpclk    	  1843200         /* �����ڲ�ʱ��Ƶ��,Ĭ���ⲿ�����12��Ƶ    */
#define MaxRecvLen    128         	  /* ���ջ�������С    */

#define com_seed	0
#define com_1		1
#define com_2		2
#define com_3		3
#define com_4		4
#define com_amp		5

uint8_t Read_BackLight_I[] 	= 	{0xef, 0xef, 0x03, 0xff, 0x03, 0xe3};
uint8_t Read_Driving_I[] 	= 	{0xef, 0xef, 0x03, 0xff, 0x01, 0xe1};
uint8_t Read_Temperature[] 	= 	{0xef, 0xef, 0x03, 0xff, 0x02, 0xe2};

const unsigned char offsetadd[] = {0x00,0x10,0x20,0x30,0x08,0x18,0x28,0x38,};		/* ���ںŵ�ƫ�Ƶ�ַ */
const unsigned char Interruptnum[] = {0x01,0x02,0x04,0x08,0x10,0x20,0x40,0x80,};	/* SSR�Ĵ����жϺŶ�Ӧֵ */

unsigned char Revbuff[MaxRecvLen];		/* ���ջ����� */
unsigned char RevLen;					/* ���ռ��� */
unsigned char DataSendbuff[MaxRecvLen]; /* �������ݻ���*/
unsigned char SendLen;					/* �������ݼ���*/
unsigned char INT_Com;

extern u8 Get_EXTI;

extern u8 ERR_OCCUR;;
extern u16 time_delay;
extern u16 time_begin;
extern u16 time_stop;

static void CH438_TranConfig(unsigned char num);								/* �������ݸ�ʽ���ü�FIFO */
static void CH438_SetBandrate(unsigned char num, unsigned long value);			/* ���ô��ڲ����� */

/**
  * Function Name  : CH438_CheckIIR()
  * Description    : ��IIR�Ĵ���
  * Input          : ���ںţ�0-7��
  * Return         : IIR�Ĵ���ֵ
  */
unsigned char CH438_CheckIIR(unsigned char num)
{
	unsigned char value;
	
	value = CH438ReadReg( offsetadd[num] | REG_IIR_ADDR );
	return value;
}

/**
  * Function Name  : CH438_CloseSeril()
  * Description    : �رմ���
  * Input          : ���ںţ�0-7��
  * Return         : None
  */
void CH438_CloseSeril(unsigned char num)
{
	CH438WriteReg(offsetadd[num]|REG_IER_ADDR, BIT_IER_LOWPOWER);
}

/**
  * Function Name  : CH438_CloseALLSeril()
  * Description    : �ر����д���
  * Input          : None
  * Return         : None
  */
void CH438_CloseALLSeril(void)
{
	CH438WriteReg(offsetadd[0]|REG_IER_ADDR, BIT_IER_LOWPOWER|BIT_IER_SLP);
}

/**
  * Function Name  : CH438_ResetSeril()
  * Description    : ��λ����
  * Input          : ���ںţ�0-7��
  * Return         : None
  */
void CH438_ResetSeril(unsigned char num)
{
	CH438WriteReg(offsetadd[num]|REG_IER_ADDR, BIT_IER_RESET);
}

/**
  * Function Name  : CH438_UARTInit()
  * Description    : ���ڳ�ʼ��
  * Input          : ���ںţ�0-7��
  * Return         : None
  */
void CH438_UARTInit(unsigned char num)
{
	//��������Ϊ19200
		switch(num)
		{
			case com_3:
				CH438_SetBandrate(num, 19200);	/* CH438���ڲ��������� */
				CH438_TranConfig(num); 					/* CH438�������ݸ�ʽ���ü�FIFO��С */	
				break;
			case com_amp:
				CH438_SetBandrate(num, 4800);	/* CH438���ڲ��������� */
				CH438_TranConfig(num); 					/* CH438�������ݸ�ʽ���ü�FIFO��С */	
				break;
			default:
				CH438_SetBandrate(num, 19200);	/* CH438���ڲ��������� */
				CH438_TranConfig(num); 					/* CH438�������ݸ�ʽ���ü�FIFO��С */	
				break;		
		}

	
}

/**
  * Function Name  : CH438_SetBandrate()
  * Description    : ���ô��ڲ�����
  * Input          : ���ںţ�0-7��;������ֵ
  * Return         : None
  */
void CH438_SetBandrate(unsigned char num, unsigned long value)
{
	uint8_t dlab=0;
	uint16_t bandspeed;
	
	dlab = CH438ReadReg(offsetadd[num]|REG_LCR_ADDR);
	dlab |= 0x80;		//��LCR�Ĵ���DLABλΪ1
	CH438WriteReg(offsetadd[num]|REG_LCR_ADDR, dlab);
	
	bandspeed = Fpclk/16/value;
	CH438WriteReg(offsetadd[num]|REG_DLL_ADDR, (uint8_t)bandspeed);
	CH438WriteReg(offsetadd[num]|REG_DLM_ADDR, (uint8_t)(bandspeed>>8));
	
#if	DEBUG_EN	
	//printf("bandrate: %x\n", bandspeed);
	//printf("DLM: %x\n", CH438ReadReg(offsetadd[num]|REG_DLM_ADDR));
	//printf("DLL: %x\n", CH438ReadReg(offsetadd[num]|REG_DLL_ADDR));
#endif	
	
	dlab &= 0x7F;		//��IIR�Ĵ���DLABλΪ0
	CH438WriteReg(offsetadd[num]|REG_LCR_ADDR, dlab);
}

/**
  * Function Name  : CH438_SendDatas()
  * Description    : ��ѯ��ʽ:��������
  * Input          : ���ںţ�0-7��; �����׵�ַ; ����
  * Return         : None
  */
void CH438_SendDatas(unsigned char num, unsigned char* sendbuff,unsigned char len)
{
	// ============  debugʱʹ�� ===================================
//			TIM_Cmd(TIM2, ENABLE); //������ʱ
//			time_begin = TIM_GetCounter(TIM2);
// =============================================================
	while(len)
	{		
//if((CH438ReadReg(offsetadd[num]|REG_LSR_ADDR)&BIT_LSR_TEMT))	    //LSR->THRE==1  ���ּĴ�����
//{
		CH438WriteReg(offsetadd[num]|REG_THR_ADDR, *sendbuff++);
		len--;
//}
	}
								// ============ debugʱʹ�� ===============================================				
//				//��ȡ������->ת������һ�ε���ʱ
//				TIM_Cmd(TIM2,DISABLE);
//				time_stop = TIM_GetCounter(TIM2);
//				TIM_SetCounter(TIM2, 0x00);
//				time_delay = time_stop - time_begin;
				//printf("CH438 Send time: %d us \n", time_delay);		
	
}

/**
  * Function Name  : CH438_RecvDatas()
  * Description    : ��ѯ��ʽ����������
  * Input          : ���ںţ�0-7��; �洢�׵�ַ
  * Return         : ��������
  */
unsigned char CH438_RecvDatas(unsigned char num, unsigned char* revbuff)
{
	
	
	uint8_t len=0;
	uint8_t *p_rev;
	
	p_rev = revbuff;
// ============  debugʱʹ�� ===================================
//			TIM_Cmd(TIM2, ENABLE); //������ʱ
//			time_begin = TIM_GetCounter(TIM2);
// =============================================================
	while( ( CH438ReadReg( offsetadd[num]|REG_LSR_ADDR ) & BIT_LSR_DATARDY ) == 0 );    /*�ȴ�����׼���� */
	while((CH438ReadReg(offsetadd[num]|REG_LSR_ADDR)&BIT_LSR_DATARDY))	//LSR->DATARDY==1
	{
		*p_rev = CH438ReadReg(offsetadd[num]|REG_RBR_ADDR);
		p_rev++;
		len++;
	}
	
// ============ debugʱʹ�� ===============================================				
//				//��ȡ������->ת������һ�ε���ʱ
//				TIM_Cmd(TIM2,DISABLE);
//				time_stop = TIM_GetCounter(TIM2);
//				TIM_SetCounter(TIM2, 0x00);
//				time_delay = time_stop - time_begin;
				//printf("CH438 Receive time: %d us \n", time_delay);		
	
	return len;
}

/**
  * Function Name  : CH438_TranConfig()
  * Description    : �������ݸ�ʽ���ü�FIFO
  * Input          : ���ںţ�0-7��
  * Return         : None
  */
void CH438_TranConfig(unsigned char num)
{	
	/* �������ݸ�ʽ:8λ���ݣ���У�飬1��ֹͣλ  */
	CH438WriteReg(offsetadd[num]|REG_LCR_ADDR, BIT_LCR_WORDSZ1|BIT_LCR_WORDSZ0);
	/* ����FIFOģʽ��������Ϊ112�ֽ� */ 
	CH438WriteReg(offsetadd[num]|REG_FCR_ADDR, BIT_FCR_RECVTG0|BIT_FCR_RECVTG1|BIT_FCR_FIFOEN|BIT_FCR_TFIFORST|BIT_FCR_RFIFORST);	
}

/**
  * Function Name  : CH438_IntConfig()
  * Description    : �����жϿ���
  * Input          : ���ںţ�0-7��
  * Return         : None
  */
void CH438_INTConfig(unsigned char num)
{	
	/* ע��: CH438��BIT_IER_IETHRE�ж�(0->1),�����һ���������ж� */	
	CH438WriteReg(offsetadd[num]|REG_IER_ADDR, BIT_IER_IELINES|BIT_IER_IERECV);
	CH438WriteReg(offsetadd[num]|REG_MCR_ADDR, BIT_MCR_OUT2);//���Բ���һ��ʵ�ʵ��ж�	
}

/**
  * Function Name  : CH438_IntDisable()
  * Description    : �����жϹر�
  * Input          : ���ںţ�0-7��
  * Return         : None
  */
void CH438_INTDisableAll(void)
{	
	unsigned char num;
	for( num = 0; num<=7; num++){
	/* ע��: CH438��BIT_IER_IETHRE�ж�(0->1),�����һ���������ж� */	
	CH438WriteReg(offsetadd[num]|REG_IER_ADDR, 0x00);
	};
}


/**
  * Function Name  : CH438_AutoHFCtrl()
  * Description    : Ӳ���Զ�������
  * Input          : ���ںţ�0-7��
  * Return         : None
  */
void CH438_AutoHFCtrl(unsigned char num)
{
    CH438WriteReg( offsetadd[num]|REG_MCR_ADDR, BIT_MCR_AFE | BIT_MCR_OUT2 | BIT_MCR_RTS );/* ����MCR�Ĵ�����AFE��RTSΪ1 */
}

/**
  * Function Name  : DELAY()
  * Description    : ��ʱ����
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
  * Description    : ����438�Ĵ�����д
  * Input          : ���ںţ�0-7��
  * Return         : None
  */
void CH438_RegTEST(unsigned char num)
{
#if  DEBUG_EN	
	printf("current test serilnum: %02x\n",(unsigned short)offsetadd[num]);
	printf("IER: %02x\n",(unsigned short)CH438ReadReg(offsetadd[num] | REG_IER_ADDR));//��IER
	printf("IIR: %02x\n",(unsigned short)CH438ReadReg(offsetadd[num] | REG_IIR_ADDR));//��IIR
	printf("LCR: %02x\n",(unsigned short)CH438ReadReg(offsetadd[num] | REG_LCR_ADDR));//��LCR
	printf("MCR: %02x\n",(unsigned short)CH438ReadReg(offsetadd[num] | REG_MCR_ADDR));//��MCR
	printf("LSR: %02x\n",(unsigned short)CH438ReadReg(offsetadd[num] | REG_LSR_ADDR));//��LSR
	printf("MSR: %02x\n",(unsigned short)CH438ReadReg(offsetadd[num] | REG_MSR_ADDR));//��MSR
	CH438WriteReg(offsetadd[num] | REG_SCR_ADDR, 0x78);
	printf("SCR: %02x\n",(unsigned short)CH438ReadReg(offsetadd[num] | REG_SCR_ADDR));//��SCR
#endif
}


/* ����ָ�����ں�*/
void SetSerial(unsigned char SelectCom )
{
	CH438_INTDisableAll();
	CH438_INTConfig(SelectCom);
	SelectCom = SelectCom;
}


/*����������									  */
/*������λ����ʽ�� AA 55 ���ں� ���ݳ��� ���� У��� */
unsigned char Load_data(uint8_t rev_len, uint8_t rev_com, uint8_t* data_send, uint8_t* data_rev)
{
	uint8_t frame_tail = 0;
	uint8_t i;
	uint8_t* p_s;
	uint8_t* p_r;
	uint8_t len = 4+rev_len+1+2;
	
	p_s = &data_send[0];
	p_r  = &data_rev[0];
	//װ��֡ͷ�Լ�����֡��Ϣ
	*p_s = 0xaa;
	p_s ++;
	*p_s = 0x55;
	p_s ++;
	*p_s = rev_com;
	p_s ++;
	*p_s = rev_len;
	p_s ++;
	
	frame_tail = 0xaa + 0x55 + rev_com + rev_len;
	
	//װ�뷢������
	for(i=0; i<rev_len; i++)
	{
		*p_s = *p_r;
		frame_tail += *p_s;
		p_s++;
		p_r ++;
	
	}
	
	//װ��֡β
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
	// ============  debugʱʹ�� ===================================
//			TIM_Cmd(TIM2, ENABLE); //������ʱ
//			time_begin = TIM_GetCounter(TIM2);
// =============================================================
	SendLen = Load_data(RevLen,INT_Com, DataSendbuff, Revbuff);
	// ============ debugʱʹ�� ===============================================				
//				//��ȡ������->ת������һ�ε���ʱ
				TIM_Cmd(TIM2,DISABLE);
				time_stop = TIM_GetCounter(TIM2);
				time_delay = time_stop - time_begin;
				//printf("Send data load time: %d us \n", time_delay);		
	
	
	// ============  debugʱʹ�� ===================================
			TIM_Cmd(TIM2, ENABLE); //������ʱ
			time_begin = TIM_GetCounter(TIM2);
// =============================================================
//	for(t = 0; t< SendLen; t++)
//		{
//			USART_SendData(USART1, DataSendbuff[t]);
//			while(USART_GetFlagStatus(USART1, USART_FLAG_TC)==RESET);
//		}			
		DMA_USART_Send(DataSendbuff, SendLen);
		Get_EXTI = 0;
		
// ============ debugʱʹ�� ===============================================				
//				//��ȡ������->ת������һ�ε���ʱ
//				TIM_Cmd(TIM2,DISABLE);
//				time_stop = TIM_GetCounter(TIM2);
//				time_delay = time_stop - time_begin;
				//printf("send to pc time: %d us \n", time_delay);		
}


/**
  * Function Name  : CH438InterruptFun()
  * Description    : ch438�жϷ�ʽ����
  * Input          : None
  * Return         : None
  */
void CH438InterruptFun (void)
{
	uint8_t gInterruptStatus;		/* ȫ���ж�״̬ */
	uint8_t InterruptStatus;		/* ���������ж�״̬ */	
	uint8_t i;						/* ���ں�*/
//	uint8_t t;

		
	
//******* ��д�߼� *******//
	
	gInterruptStatus = CH438ReadReg( REG_SSR_ADDR );

	if(!gInterruptStatus)
		return ;
for(i = 0; i<8; i++){
	if( gInterruptStatus & Interruptnum[i] )    /* ����ĸ����ڷ����ж� */
		{
			InterruptStatus = CH438ReadReg( offsetadd [i] | REG_IIR_ADDR ) & 0x0f;    /* �����ڵ��ж�״̬ */	
			switch( InterruptStatus )
			{
				case INT_NOINT:			/* û���ж� */					
					break;
				case INT_THR_EMPTY:		/* THR���ж� */						
					break;
				case INT_RCV_OVERTIME:	/* ���ճ�ʱ�ж� */
				
					if(!Get_EXTI){
						RevLen = CH438_RecvDatas(i, Revbuff);
						INT_Com = i;
						Get_EXTI = 1;
						//Send2PC();
					}
					return;					

				case INT_RCV_SUCCESS:	/* �������ݿ����ж� */
					if(!Get_EXTI){
						RevLen = CH438_RecvDatas(i, Revbuff);
						INT_Com = i;
						Get_EXTI = 1;
						//Send2PC();
					}
					return;				
					
				case INT_RCV_LINES:		/* ������·״̬�ж� */
					CH438ReadReg( offsetadd[i] | REG_LSR_ADDR );
					ERR_OCCUR = 1;
					break;
				case INT_MODEM_CHANGE:	/* MODEM����仯�ж� */
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




