


#ifndef _CH438_USART
#define _CH438_USART

/* 芯片寄存器定义 */

/* CH438串口寄存器地址 */
#define REG_RBR_ADDR        0x00      /* 串口0接收缓冲寄存器地址 */
#define REG_THR_ADDR        0x00      /* 串口0发送保持寄存器地址 */
#define REG_IER_ADDR        0x01      /* 串口0中断使能寄存器地址 */
#define REG_IIR_ADDR        0x02      /* 串口0中断识别寄存器地址 */
#define REG_FCR_ADDR        0x02      /* 串口0FIFO控制寄存器地址 */
#define REG_LCR_ADDR        0x03      /* 串口0线路控制寄存器地址 */
#define REG_MCR_ADDR        0x04      /* 串口0MODEM控制寄存器地址 */
#define REG_LSR_ADDR        0x05      /* 串口0线路状态寄存器地址 */
#define REG_MSR_ADDR        0x06      /* 串口0MODEM状态寄存器地址 */
#define REG_SCR_ADDR        0x07      /* 串口0用户可定义寄存器地址 */
#define REG_DLL_ADDR        0x00      /* 波特率除数锁存器低8位字节地址 */
#define REG_DLM_ADDR        0x01      /* 波特率除数锁存器高8位字节地址 */

///* CH438串口1寄存器地址 */
//#define REG_RBR1_ADDR       0x10      /* 串口1接收缓冲寄存器地址 */
//#define REG_THR1_ADDR       0x10      /* 串口1发送保持寄存器地址 */
//#define REG_IER1_ADDR       0x11      /* 串口1中断使能寄存器地址 */
//#define REG_IIR1_ADDR       0x12      /* 串口1中断识别寄存器地址 */
//#define REG_FCR1_ADDR       0x12      /* 串口1FIFO控制寄存器地址 */
//#define REG_LCR1_ADDR       0x13      /* 串口1线路控制寄存器地址 */
//#define REG_MCR1_ADDR       0x14      /* 串口1MODEM控制寄存器地址 */
//#define REG_LSR1_ADDR       0x15      /* 串口1线路状态寄存器地址 */
//#define REG_MSR1_ADDR       0x16      /* 串口1MODEM状态寄存器地址 */
//#define REG_SCR1_ADDR       0x17      /* 串口1用户可定义寄存器地址 */
//#define REG_DLL1_ADDR       0x10      /* 波特率除数锁存器低8位字节地址 */
//#define REG_DLM1_ADDR       0x11      /* 波特率除数锁存器高8位字节地址 */


///* CH438串口2寄存器地址 */
//#define REG_RBR2_ADDR       0x20      /* 串口2接收缓冲寄存器地址 */
//#define REG_THR2_ADDR       0x20      /* 串口2发送保持寄存器地址 */
//#define REG_IER2_ADDR       0x21      /* 串口2中断使能寄存器地址 */
//#define REG_IIR2_ADDR       0x22      /* 串口2中断识别寄存器地址 */
//#define REG_FCR2_ADDR       0x22      /* 串口2FIFO控制寄存器地址 */
//#define REG_LCR2_ADDR       0x23      /* 串口2线路控制寄存器地址 */
//#define REG_MCR2_ADDR       0x24      /* 串口2MODEM控制寄存器地址 */
//#define REG_LSR2_ADDR       0x25      /* 串口2线路状态寄存器地址 */
//#define REG_MSR2_ADDR       0x26      /* 串口2MODEM状态寄存器地址 */
//#define REG_SCR2_ADDR       0x27      /* 串口2用户可定义寄存器地址 */
//#define REG_DLL2_ADDR       0x20      /* 波特率除数锁存器低8位字节地址 */
//#define REG_DLM2_ADDR       0x21      /* 波特率除数锁存器高8位字节地址 */


///* CH438串口3寄存器地址 */
//#define REG_RBR3_ADDR       0x30      /* 串口3接收缓冲寄存器地址 */
//#define REG_THR3_ADDR       0x30      /* 串口3发送保持寄存器地址 */
//#define REG_IER3_ADDR       0x31      /* 串口3中断使能寄存器地址 */
//#define REG_IIR3_ADDR       0x32      /* 串口3中断识别寄存器地址 */
//#define REG_FCR3_ADDR       0x32      /* 串口3FIFO控制寄存器地址 */
//#define REG_LCR3_ADDR       0x33      /* 串口3线路控制寄存器地址 */
//#define REG_MCR3_ADDR       0x34      /* 串口3MODEM控制寄存器地址 */
//#define REG_LSR3_ADDR       0x35      /* 串口3线路状态寄存器地址 */
//#define REG_MSR3_ADDR       0x36      /* 串口3MODEM状态寄存器地址 */
//#define REG_SCR3_ADDR       0x37      /* 串口3用户可定义寄存器地址 */
//#define REG_DLL3_ADDR       0x30      /* 波特率除数锁存器低8位字节地址 */
//#define REG_DLM3_ADDR       0x31      /* 波特率除数锁存器高8位字节地址 */


///* CH438串口4寄存器地址 */
//#define REG_RBR4_ADDR       0x08      /* 串口4接收缓冲寄存器地址 */
//#define REG_THR4_ADDR       0x08      /* 串口4发送保持寄存器地址 */
//#define REG_IER4_ADDR       0x09      /* 串口4中断使能寄存器地址 */
//#define REG_IIR4_ADDR       0x0A      /* 串口4中断识别寄存器地址 */
//#define REG_FCR4_ADDR       0x0A      /* 串口4FIFO控制寄存器地址 */
//#define REG_LCR4_ADDR       0x0B      /* 串口4线路控制寄存器地址 */
//#define REG_MCR4_ADDR       0x0C      /* 串口4MODEM控制寄存器地址 */
//#define REG_LSR4_ADDR       0x0D      /* 串口4线路状态寄存器地址 */
//#define REG_MSR4_ADDR       0x0E      /* 串口4MODEM状态寄存器地址 */
//#define REG_SCR4_ADDR       0x0F      /* 串口4用户可定义寄存器地址 */
//#define REG_DLL4_ADDR       0x08      /* 波特率除数锁存器低8位字节地址 */
//#define REG_DLM4_ADDR       0x09      /* 波特率除数锁存器高8位字节地址 */


///* CH438串口5寄存器地址 */
//#define REG_RBR5_ADDR       0x18      /* 串口5接收缓冲寄存器地址 */
//#define REG_THR5_ADDR       0x18      /* 串口5发送保持寄存器地址 */
//#define REG_IER5_ADDR       0x19      /* 串口5中断使能寄存器地址 */
//#define REG_IIR5_ADDR       0x1A      /* 串口5中断识别寄存器地址 */
//#define REG_FCR5_ADDR       0x1A      /* 串口5FIFO控制寄存器地址 */
//#define REG_LCR5_ADDR       0x1B      /* 串口5线路控制寄存器地址 */
//#define REG_MCR5_ADDR       0x1C      /* 串口5MODEM控制寄存器地址 */
//#define REG_LSR5_ADDR       0x1D      /* 串口5线路状态寄存器地址 */
//#define REG_MSR5_ADDR       0x1E      /* 串口5MODEM状态寄存器地址 */
//#define REG_SCR5_ADDR       0x1F      /* 串口5用户可定义寄存器地址 */
//#define REG_DLL5_ADDR       0x18      /* 波特率除数锁存器低8位字节地址 */
//#define REG_DLM5_ADDR       0x19      /* 波特率除数锁存器高8位字节地址 */


///* CH438串口6寄存器地址 */
//#define REG_RBR6_ADDR       0x28      /* 串口6接收缓冲寄存器地址 */
//#define REG_THR6_ADDR       0x28      /* 串口6发送保持寄存器地址 */
//#define REG_IER6_ADDR       0x29      /* 串口6中断使能寄存器地址 */
//#define REG_IIR6_ADDR       0x2A      /* 串口6中断识别寄存器地址 */
//#define REG_FCR6_ADDR       0x2A      /* 串口6FIFO控制寄存器地址 */
//#define REG_LCR6_ADDR       0x2B      /* 串口6线路控制寄存器地址 */
//#define REG_MCR6_ADDR       0x2C      /* 串口6MODEM控制寄存器地址 */
//#define REG_LSR6_ADDR       0x2D      /* 串口6线路状态寄存器地址 */
//#define REG_MSR6_ADDR       0x2E      /* 串口6MODEM状态寄存器地址 */
//#define REG_SCR6_ADDR       0x2F      /* 串口6用户可定义寄存器地址 */
//#define REG_DLL6_ADDR       0x28      /* 波特率除数锁存器低8位字节地址 */
//#define REG_DLM6_ADDR       0x29      /* 波特率除数锁存器高8位字节地址 */


///* CH438串口7寄存器地址 */
#define REG_RBR7_ADDR       0x38      /* 串口7接收缓冲寄存器地址 */
#define REG_THR7_ADDR       0x38      /* 串口7发送保持寄存器地址 */
#define REG_IER7_ADDR       0x39      /* 串口7中断使能寄存器地址 */
#define REG_IIR7_ADDR       0x3A      /* 串口7中断识别寄存器地址 */
#define REG_FCR7_ADDR       0x3A      /* 串口7FIFO控制寄存器地址 */
#define REG_LCR7_ADDR       0x3B      /* 串口7线路控制寄存器地址 */
#define REG_MCR7_ADDR       0x3C      /* 串口7MODEM控制寄存器地址 */
#define REG_LSR7_ADDR       0x3D      /* 串口7线路状态寄存器地址 */
#define REG_MSR7_ADDR       0x3E      /* 串口7MODEM状态寄存器地址 */
#define REG_SCR7_ADDR       0x3F      /* 串口7用户可定义寄存器地址 */
#define REG_DLL7_ADDR       0x38      /* 波特率除数锁存器低8位字节地址 */
#define REG_DLM7_ADDR       0x39      /* 波特率除数锁存器高8位字节地址 */


/* CH438内部串口0~7 专用状态寄存器 */

#define REG_SSR_ADDR        0x4F       /* 专用状态寄存器地址 */


/* IER寄存器的位 */

#define BIT_IER_RESET       0x80      /* 该位置1则软复位该串口 */
#define BIT_IER_LOWPOWER    0x40      /* 该位为1则关闭该串口的内部基准时钟 */
#define BIT_IER_SLP         0x20      /* 串口0是SLP,为1则关闭时钟震荡器 */
#define BIT_IER1_CK2X       0x20      /* 串口1是CK2X,为1则强制将外部时钟信号2倍频后作为内部基准时钟 */
#define BIT_IER_IEMODEM     0x08      /* 该位为1允许MODEM输入状态变化中断 */
#define BIT_IER_IELINES     0x04      /* 该位为1允许接收线路状态中断 */
#define BIT_IER_IETHRE      0x02      /* 该位为1允许发送保持寄存器空中断 */
#define BIT_IER_IERECV      0x01      /* 该位为1允许接收到数据中断 */

/* IIR寄存器的位 */

#define BIT_IIR_FIFOENS1    0x80
#define BIT_IIR_FIFOENS0    0x40      /* 该2位为1表示起用FIFO */

/* 中断类型：0001没有中断，0110接收线路状态中断，0100接收数据可用中断，1100接收数据超时中断，0010THR寄存器空中断，0000MODEM输入变化中断 */
#define BIT_IIR_IID3        0x08
#define BIT_IIR_IID2        0x04	//接受数据可用
#define BIT_IIR_IID1        0x02	//THR寄存器空中断
#define BIT_IIR_NOINT       0x01

/* FCR寄存器的位 */

/* 触发点： 00对应1个字节，01对应16个字节，10对应64个字节，11对应112个字节 */
#define BIT_FCR_RECVTG1     0x80      /* 设置FIFO的中断和自动硬件流控制的触发点 */
#define BIT_FCR_RECVTG0     0x40      /* 设置FIFO的中断和自动硬件流控制的触发点 */

#define BIT_FCR_TFIFORST    0x04      /* 该位置1则清空发送FIFO中的数据 */
#define BIT_FCR_RFIFORST    0x02      /* 该位置1则清空接收FIFO中的数据 */
#define BIT_FCR_FIFOEN      0x01      /* 该位置1则起用FIFO,为0则禁用FIFO */

/* LCR寄存器的位 */

#define BIT_LCR_DLAB        0x80      /* 为1才能存取DLL，DLM，为0才能存取RBR/THR/IER */
#define BIT_LCR_BREAKEN     0x40      /* 为1则强制产生BREAK线路间隔*/

/* 设置校验格式：当PAREN为1时，00奇校验，01偶校验，10标志位（MARK，置1)，11空白位（SPACE，清0) */
#define BIT_LCR_PARMODE1    0x20      /* 设置奇偶校验位格式 */
#define BIT_LCR_PARMODE0    0x10      /* 设置奇偶校验位格式 */

#define BIT_LCR_PAREN       0x08      /* 为1则允许发送时产生和接收校验奇偶校验位 */
#define BIT_LCR_STOPBIT     0x04      /* 为1则两个停止位,为0一个停止位 */

/* 设置字长度：00则5个数据位，01则6个数据位，10则7个数据位，11则8个数据位 */
#define BIT_LCR_WORDSZ1     0x02      /* 设置字长长度 */
#define BIT_LCR_WORDSZ0     0x01

/* MCR寄存器的位 */

#define BIT_MCR_AFE         0x20      /* 为1允许CTS和RTS硬件自动流控制 */
#define BIT_MCR_LOOP        0x10      /* 为1使能内部回路的测试模式 */
#define BIT_MCR_OUT2        0x08      /* 为1允许该串口的中断请求输出 */
#define BIT_MCR_OUT1        0x04      /* 为用户定义的MODEM控制位 */
#define BIT_MCR_RTS         0x02      /* 该位为1则RTS引脚输出有效 */
#define BIT_MCR_DTR         0x01      /* 该位为1则DTR引脚输出有效 */

/* LSR寄存器的位 */

#define BIT_LSR_RFIFOERR    0x80      /* 为1表示在接收FIFO中存在至少一个错误 */
#define BIT_LSR_TEMT        0x40      /* 为1表示THR和TSR全空 */
#define BIT_LSR_THRE        0x20      /* 为1表示THR空*/
#define BIT_LSR_BREAKINT    0x10      /* 该位为1表示检测到BREAK线路间隔 */
#define BIT_LSR_FRAMEERR    0x08      /* 该位为1表示读取数据帧错误 */
#define BIT_LSR_PARERR      0x04      /* 该位为1表示奇偶校验错误 */
#define BIT_LSR_OVERR       0x02      /* 为1表示接收FIFO缓冲区溢出 */
#define BIT_LSR_DATARDY     0x01      /* 该位为1表示接收FIFO中有接收到的数据 */

/* MSR寄存器的位 */

#define BIT_MSR_DCD         0x80      /* 该位为1表示DCD引脚有效 */
#define BIT_MSR_RI          0x40      /* 该位为1表示RI引脚有效 */
#define BIT_MSR_DSR         0x20      /* 该位为1表示DSR引脚有效 */
#define BIT_MSR_CTS         0x10      /* 该位为1表示CTS引脚有效 */
#define BIT_MSR_DDCD        0x08      /* 该位为1表示DCD引脚输入状态发生变化过 */
#define BIT_MSR_TERI        0x04      /* 该位为1表示RI引脚输入状态发生变化过 */
#define BIT_MSR_DDSR        0x02      /* 该位为1表示DSR引脚输入状态发生变化过 */
#define BIT_MSR_DCTS        0x01      /* 该位为1表示CTS引脚输入状态发生变化过 */

/* 中断状态码 */

#define INT_NOINT           0x01      /* 没有中断 */
#define INT_THR_EMPTY       0x02      /* THR空中断 */
#define INT_RCV_OVERTIME    0x0C      /* 接收超时中断 */
#define INT_RCV_SUCCESS     0x04      /* 接收数据可用中断 */
#define INT_RCV_LINES       0x06      /* 接收线路状态中断 */
#define INT_MODEM_CHANGE    0x00      /* MODEM输入变化中断 */

#define CH438_IIR_FIFOS_ENABLED 0xC0  /* 起用FIFO */




#endif

