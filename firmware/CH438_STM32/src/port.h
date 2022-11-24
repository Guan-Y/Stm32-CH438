/********************************** (C) COPYRIGHT ******************************
* File Name          : port.h
* Author             : WCH
* Version            : 
* Date               : 2014/9/1
* Description        :ch438��stm32���ӽӿ�����ͷ�ļ�
*******************************************************************************/

#ifndef _PORT_CON
#define _PORT_CON

void CH438PortConfig(void);										/* CH438�ӿ����� */
void CH438WriteReg(unsigned char add,unsigned char data);				/* дCH438 */
unsigned char CH438ReadReg(unsigned char add);	        				/* ��CH438 */

void stm32_XINT_Config(void);													/* ��Ƭ���ж����� */
void stm32_TIM2_Config(void);
void stm32_TIM3_Config(void);

#endif
