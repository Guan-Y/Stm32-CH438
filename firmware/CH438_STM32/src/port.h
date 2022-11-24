/********************************** (C) COPYRIGHT ******************************
* File Name          : port.h
* Author             : WCH
* Version            : 
* Date               : 2014/9/1
* Description        :ch438与stm32连接接口配置头文件
*******************************************************************************/

#ifndef _PORT_CON
#define _PORT_CON

void CH438PortConfig(void);										/* CH438接口配置 */
void CH438WriteReg(unsigned char add,unsigned char data);				/* 写CH438 */
unsigned char CH438ReadReg(unsigned char add);	        				/* 读CH438 */

void stm32_XINT_Config(void);													/* 单片机中断配置 */
void stm32_TIM2_Config(void);
void stm32_TIM3_Config(void);

#endif
