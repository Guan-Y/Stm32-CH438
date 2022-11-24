/********************************** (C) COPYRIGHT ******************************
* File Name          : CH438_driver.h
* Author             : WCH
* Version            : 
* Date               : 2014/9/11
* Description        : ch438��������ͷ�ļ�
*******************************************************************************/

#ifndef _CH438_DRIVER
#define _CH438_DRIVER

unsigned char CH438_CheckIIR(unsigned char num);		/* ��IIR�Ĵ��� */
void CH438_CloseSeril(unsigned char num);				/* �رմ��� */
void CH438_CloseALLSeril(void);							/* �ر����д���,����͹��� */
void CH438_ResetSeril(unsigned char num);				/* ��λ���� */

void CH438_UARTInit(unsigned char num);					/* ���ڳ�ʼ�� */

void CH438_INTConfig(unsigned char num);				/* �����жϿ��� */
void CH438_AutoHFCtrl(unsigned char num);				/* Ӳ���Զ������� */

//void CH438_TranConfig(unsigned char num);
void CH438_SendDatas(unsigned char num, unsigned char* sendbuff,unsigned char len);		/* ��ѯ��ʽ:�������� */
unsigned char CH438_RecvDatas(unsigned char num, unsigned char* revbuff);				/* ��ѯ��ʽ���������� */

void CH438InterruptFun (void);				/* �����жϺ��� */

void CH438_RegTEST(unsigned char num);	/* ���Դ��ڼĴ���ͨѶ */
void DELAY(unsigned char time);


void SetSerial(unsigned char SelectCom);

unsigned char  Load_data(unsigned char rev_len, unsigned char rev_com, unsigned char *data_send, unsigned char * data_rev);
void Forward_CH438(void);
void Send2PC(void);

void Clear_FIFO(void);

#endif
