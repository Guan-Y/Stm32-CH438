/********************************** (C) COPYRIGHT ******************************
* File Name          : CH438_driver.h
* Author             : WCH
* Version            : 
* Date               : 2014/9/11
* Description        : ch438控制驱动头文件
*******************************************************************************/

#ifndef _CH438_DRIVER
#define _CH438_DRIVER

unsigned char CH438_CheckIIR(unsigned char num);		/* 读IIR寄存器 */
void CH438_CloseSeril(unsigned char num);				/* 关闭串口 */
void CH438_CloseALLSeril(void);							/* 关闭所有串口,进入低功耗 */
void CH438_ResetSeril(unsigned char num);				/* 复位串口 */

void CH438_UARTInit(unsigned char num);					/* 串口初始化 */

void CH438_INTConfig(unsigned char num);				/* 串口中断开启 */
void CH438_AutoHFCtrl(unsigned char num);				/* 硬件自动流开启 */

//void CH438_TranConfig(unsigned char num);
void CH438_SendDatas(unsigned char num, unsigned char* sendbuff,unsigned char len);		/* 查询方式:发送数据 */
unsigned char CH438_RecvDatas(unsigned char num, unsigned char* revbuff);				/* 查询方式：接受数据 */

void CH438InterruptFun (void);				/* 串口中断函数 */

void CH438_RegTEST(unsigned char num);	/* 测试串口寄存器通讯 */
void DELAY(unsigned char time);


void SetSerial(unsigned char SelectCom);

unsigned char  Load_data(unsigned char rev_len, unsigned char rev_com, unsigned char *data_send, unsigned char * data_rev);
void Forward_CH438(void);
void Send2PC(void);

void Clear_FIFO(void);

#endif
