/*
***************************************************************************************************
*                                              APPLICATION CODE
*
*                          (c) Copyright 2015; Guangdong Hydrogen Energy Science And Technology Co.,Ltd
*
*               All rights reserved.  Protected by international copyright laws.
*               Knowledge of the source code may NOT be used without authorization.
***************************************************************************************************
*/
/*
***************************************************************************************************
* Filename      : bsp_ser.c
* Version       : V1.00
* Programmer(s) : SunKing.Yun
***************************************************************************************************
*/


/*
***************************************************************************************************
*                                             INCLUDE FILES
***************************************************************************************************
*/

#define  BSP_SER_MODULE
#include <bsp.h>
#include "bsp_ser.h"
#include <app_system_real_time_parameters.h>
#include "app_wireness_communicate_task.h"
#include "string.h"
#include "bsp_MF210.h"
#include "app_mf210_communicate_task.h"
/*
***************************************************************************************************
*                                            LOCAL DEFINES
***************************************************************************************************
*/
#define UART4_DR_Address    ((uint32_t)UART4_BASE+0x04)

/*
***************************************************************************************************
*                                           LOCAL CONSTANTS
***************************************************************************************************
*/


/*
***************************************************************************************************
*                                          LOCAL DATA TYPES
***************************************************************************************************
*/


/*
***************************************************************************************************
*                                           GLOBAL VARIABLES
***************************************************************************************************
*/

uint8_t g_u8WifiCommandReceived;
uint8_t g_u83GCommandReceived;
/*
***************************************************************************************************
*                                       LOCAL GLOBAL VARIABLES
***************************************************************************************************
*/

OS_SEM      g_stWirenessCommSem;
OS_MUTEX    g_stWIFISerTxMutex;

static  BSP_OS_SEM   BSP_SerTxWait;
static  BSP_OS_SEM   BSP_SerRxWait;
static  BSP_OS_SEM   BSP_SerLock;
static  CPU_INT08U   BSP_SerRxData;

static  BSP_OS_SEM   BSP_Ser2TxWait;
static  BSP_OS_SEM   BSP_Ser2RxWait;
static  CPU_INT08U   BSP_Ser2RxData;

#if (BSP_CFG_SER_CMD_HISTORY_LEN > 0u)
    static  CPU_CHAR     BSP_SerCmdHistory[BSP_CFG_SER_CMD_HISTORY_LEN];
#endif

/*
***************************************************************************************************
*                                      LOCAL FUNCTION PROTOTYPES
***************************************************************************************************
*/

static  void        BSP_Ser_WrByteUnlocked(CPU_INT08U  c);
static  CPU_INT08U  BSP_Ser_RdByteUnlocked(void);
static  void        BSP_Ser_ISR_Handler(void);
static  void        WIFI_DataRx_IRQHandler(void);
static  void        WIFI_DataTx_IRQHandler(void);

static  void        BSP_SerTo3G_ISR_Handler(void);

static  void        UART4_DMA_NVIC_Init(void);
static  void        BSP_SerToWIFI_ISR_Handler(void);

        uint8_t     *GetPrgmRxBuffAddr(void);
        uint8_t     GetPrgmRxBuffLen(void);

/*
***************************************************************************************************
*                                     LOCAL CONFIGURATION ERRORS
***************************************************************************************************
*/


/*
***************************************************************************************************
***************************************************************************************************
**                                         GLOBAL FUNCTIONS
***************************************************************************************************
***************************************************************************************************
*/


/*
***************************************************************************************************
*                                          BSP_Ser_Init()
*
* Description : Initialize a serial port for communication.
*                               
* Argument(s) : baud_rate           The desire RS232 baud rate.
*
* Return(s)   : none.
*
* Caller(s)   : Application.
*
* Note(s)     : none.
***************************************************************************************************
*/

void  BSP_Ser_Init(CPU_INT32U  baud_rate)
{
    FlagStatus              tc_status;
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    USART_ClockInitTypeDef  USART_ClockInitStructure;

    /* 时钟使能 */
    BSP_PeriphEn(BSP_PERIPH_ID_USART1);
    BSP_PeriphEn(BSP_PERIPH_ID_IOPA);
    BSP_PeriphEn(BSP_PERIPH_ID_AFIO);

    /* ------------------ INIT OS OBJECTS ----------------- */
    BSP_OS_SemCreate(&BSP_SerTxWait,   0, "Serial Tx Wait");
    BSP_OS_SemCreate(&BSP_SerRxWait,   0, "Serial Rx Wait");
    BSP_OS_SemCreate(&BSP_SerLock,     1, "Serial Lock");

#if (BSP_CFG_SER_CMD_HISTORY_LEN > 0u)
    BSP_SerCmdHistory[0] = (CPU_CHAR)'\0';
#endif

    /* ----------------- INIT USART STRUCT ---------------- */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;            //USART1_TX PA9
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;           //USART1_RX    PA10
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    USART_DeInit(USART1);  //复位串口1

    USART_ClockInitStructure.USART_Clock = USART_Clock_Disable;
    USART_ClockInitStructure.USART_CPOL = USART_CPOL_Low;
    USART_ClockInitStructure.USART_CPHA = USART_CPHA_2Edge;
    USART_ClockInitStructure.USART_LastBit = USART_LastBit_Disable;
    USART_ClockInit(USART1, &USART_ClockInitStructure);

    USART_InitStructure.USART_BaudRate = 115200;                //波特率设置
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;  //字长为8位数据格式
    USART_InitStructure.USART_StopBits = USART_StopBits_1;          //一个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;                 //无奇偶校验位
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; //收发模式
    USART_Init(USART1, &USART_InitStructure);  //初始化串口
    USART_Cmd(USART1, ENABLE);

    USART_ITConfig(USART1, USART_IT_TC, DISABLE);
    USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
    tc_status = USART_GetFlagStatus(USART1, USART_FLAG_TC);

    while(tc_status == SET) {
        USART_ClearITPendingBit(USART1, USART_IT_TC);
        USART_ClearFlag(USART1, USART_IT_TC);
        BSP_OS_TimeDlyMs(10);
        tc_status = USART_GetFlagStatus(USART1, USART_FLAG_TC);
    }

    BSP_IntVectSet(BSP_INT_ID_USART1, BSP_Ser_ISR_Handler);
    BSP_IntEn(BSP_INT_ID_USART1);
}
/*
***************************************************************************************************
*                                         BSP_Ser_ISR_Handler()
*
* Description : Serial ISR.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : This is an ISR.
*
* Note(s)     : none.
***************************************************************************************************
*/

void  BSP_Ser_ISR_Handler(void)
{
    FlagStatus tc_status;
    FlagStatus rxne_status;

    rxne_status = USART_GetFlagStatus(USART1, USART_FLAG_RXNE);

    if(rxne_status == SET) {
        BSP_SerRxData = USART_ReceiveData(USART1) & 0xFF;       /* Read one byte from the receive data register.      */
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);         /* Clear the USART1 receive interrupt.                */
        BSP_OS_SemPost(&BSP_SerRxWait);                         /* Post to the sempahore                              */
    }

    tc_status = USART_GetFlagStatus(USART1, USART_FLAG_TC);

    if(tc_status == SET) {
        USART_ITConfig(USART1, USART_IT_TC, DISABLE);
        USART_ClearITPendingBit(USART1, USART_IT_TC);           /* Clear the USART1 receive interrupt.                */
        BSP_OS_SemPost(&BSP_SerTxWait);                         /* Post to the semaphore                              */
    }
}


/*
***************************************************************************************************
*                                 BSP_Ser_Init2()
*
* Description : Initialize a serial port for 3G communication.
*                               
* Argument(s) : baud_rate   The desire 3G baud rate.
*
* Return(s)   : none.
*
* Caller(s)   : Application.
*
* Note(s)     : none.
***************************************************************************************************
*/
uint8_t usart2_recv_buff[1024] = {0};
uint16_t recv_cursor = 0;

void  BSP_Ser_To_3G_Init(CPU_INT32U  baud_rate)
{   
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    BSP_OS_SemCreate(&BSP_Ser2TxWait,   0, "Serial Tx Wait");
    BSP_OS_SemCreate(&BSP_Ser2RxWait,   0, "Serial Rx Wait");

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE); //使能USART2，GPIOD时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); //使能USART2
    BSP_PeriphEn(BSP_PERIPH_ID_AFIO);

    GPIO_PinRemapConfig(GPIO_Remap_USART2, ENABLE);

    //USART2_TX   PD5
    GPIO_InitStructure.GPIO_Pin = BSP_GPIOD_PIN5_UART2_TX_PORT_NMB; 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //复用推挽输出
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    //USART2_RX   PD6
    GPIO_InitStructure.GPIO_Pin = BSP_GPIOD_PIN6_UART2_RX_PORT_NMB;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = baud_rate;//一般设置为9600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
    USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; //收发模式
    USART_Init(USART2, &USART_InitStructure); //初始化串口

    USART_ITConfig(USART2, USART_IT_TC, DISABLE);
    USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
    FlagStatus tc_status = USART_GetFlagStatus(USART2, USART_FLAG_TC);

    while(tc_status == SET)
    {
        USART_ClearITPendingBit(USART2, USART_IT_TC);
        USART_ClearFlag(USART2, USART_IT_TC);
        BSP_OS_TimeDlyMs(10);
        tc_status = USART_GetFlagStatus(USART2, USART_FLAG_TC);
    }

    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//开接收中断
    BSP_IntVectSet(BSP_INT_ID_USART2, BSP_SerTo3G_ISR_Handler);   //中断向量表设置
    BSP_IntEn(BSP_INT_ID_USART2);

    USART_Cmd(USART2, ENABLE);                //使能串口

}

/*
***************************************************************************************************
*                                 BSP_SerTo3G_ISR_Handler()
*
* Description : Initialize a serial port for 3G communication.
*                               
* Argument(s) : baud_rate   The desire 3G baud rate.
*
* Return(s)   : none.
*
* Note(s)     : none.
***************************************************************************************************
*/
static void  BSP_SerTo3G_ISR_Handler(void)
{
    FlagStatus tc_status;
    FlagStatus rxne_status;

    rxne_status = USART_GetFlagStatus(USART2, USART_FLAG_RXNE);

    if(rxne_status == SET)
    {
       BSP_Ser2RxData = USART_ReceiveData(USART2) & 0xFF;       /* Read one byte from the receive data register.      */
        usart2_recv_buff[recv_cursor++] = BSP_Ser2RxData;
        USART_ClearITPendingBit(USART2, USART_IT_RXNE);         /* Clear the USART1 receive interrupt.                */
        BSP_OS_SemPost(&BSP_Ser2RxWait);                        /* Post to the sempahore                              */
    }

    tc_status = USART_GetFlagStatus(USART2, USART_FLAG_TC);

    if(tc_status == SET)
    {
        USART_ITConfig(USART2, USART_IT_TC, DISABLE);
        USART_ClearITPendingBit(USART2, USART_IT_TC);           /* Clear the USART1 receive interrupt.                */
        BSP_OS_SemPost(&BSP_Ser2TxWait);                        /* Post to the semaphore                              */
    }
}

/*
***************************************************************************************************
*                                 BSP_Ser_ISR2_Handler()
*
* Description : Initialize a serial port for 3G communication.
*                               
* Argument(s) : baud_rate   The desire 3G baud rate.
*
* Return(s)   : none.
*
* Note(s)     : none.
***************************************************************************************************
*/
void SendString(USART_TypeDef *USARTx, const char *s)
{
    while(*s)
    {
        USART_ITConfig(USARTx, USART_IT_TC, ENABLE);
        USART_SendData(USARTx , *s++);
        BSP_OS_SemWait(&BSP_Ser2TxWait, 0);
        USART_ITConfig(USARTx, USART_IT_TC, DISABLE);
    }
}
/*
***************************************************************************************************
*                                 BSP_Ser_ISR2_Handler()
*
* Description : Initialize a serial port for 3G communication.
*                               
* Argument(s) : baud_rate   The desire 3G baud rate.
*
* Return(s)   : none.
*
* Note(s)     : none.
***************************************************************************************************
*/
void CleanUsartRecvBuf(USART_TypeDef *USARTx)
{
    if(USART2 == USARTx)
    {
        memset(usart2_recv_buff, 0x00, recv_cursor);
        recv_cursor = 0;
    }
}
/*
***************************************************************************************************
*                                 BSP_Ser_ISR2_Handler()
*
* Description : Initialize a serial port for 3G communication.
*                               
* Argument(s) : baud_rate   The desire 3G baud rate.
*
* Return(s)   : none.
*
* Note(s)     : 向串口发送一个数组    buffer指向数组首地址。.
***************************************************************************************************
*/
void Uart_Send_array1(u8 *buffer, u8 count)
{
    SocketSend(1, buffer, count);
}

/*
***************************************************************************************************
*                                          BSP_SerToWIFI_Init()
*
* Description : Initialize a serial port for communication to WIFI.
*               
* Argument(s) : none.
*
* Return(s)   : none.
*
* Note(s)     : none.
***************************************************************************************************
*/

void  BSP_SerToWIFI_Init()
{
    FlagStatus              tc_status;
    GPIO_InitTypeDef        gpio_init;
    NVIC_InitTypeDef NVIC_InitStructure;
    USART_InitTypeDef       usart_init;
    USART_ClockInitTypeDef  usart_clk_init;
#if 0
    BSP_OS_SemCreate(&BSP_SerTxWait,   0, "Serial Tx Wait");
    BSP_OS_SemCreate(&BSP_SerRxWait,   0, "Serial Rx Wait");
    BSP_OS_SemCreate(&BSP_SerLock,     1, "Serial Lock");

#if (BSP_CFG_SER_CMD_HISTORY_LEN > 0u)
    BSP_SerCmdHistory[0] = (CPU_CHAR)'\0';
#endif
#endif
    gpio_init.GPIO_Pin   = GPIO_Pin_10;
    gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
    gpio_init.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOC, &gpio_init);

    gpio_init.GPIO_Pin   = GPIO_Pin_11;
    gpio_init.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOC, &gpio_init);

    //外部时钟使能
    BSP_PeriphEn(BSP_PERIPH_ID_USART4);
    BSP_PeriphEn(BSP_PERIPH_ID_IOPC);
    BSP_PeriphEn(BSP_PERIPH_ID_AFIO);

    usart_init.USART_BaudRate            = 115200;
    usart_init.USART_WordLength          = USART_WordLength_8b;
    usart_init.USART_StopBits            = USART_StopBits_1;
    usart_init.USART_Parity              = USART_Parity_No ;
    usart_init.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart_init.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(UART4, &usart_init);

    usart_clk_init.USART_Clock           = USART_Clock_Disable;
    usart_clk_init.USART_CPOL            = USART_CPOL_Low;
    usart_clk_init.USART_CPHA            = USART_CPHA_2Edge;
    usart_clk_init.USART_LastBit         = USART_LastBit_Disable;
    USART_ClockInit(UART4, &usart_clk_init);

    USART_Cmd(UART4, ENABLE);
    //传输完成、发送中断禁能
    USART_ITConfig(UART4, USART_IT_TC, DISABLE);
    USART_ITConfig(UART4, USART_IT_TXE, DISABLE);
    tc_status  = USART_GetFlagStatus(UART4, USART_FLAG_TC);

    while(tc_status == SET) {
        USART_ClearITPendingBit(UART4, USART_IT_TC);
        USART_ClearFlag(UART4, USART_IT_TC);
        BSP_OS_TimeDlyMs(10);
        tc_status = USART_GetFlagStatus(UART4, USART_FLAG_TC);
    }

    /* Configure and enable USART4 interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    USART_ITConfig(UART4, USART_IT_RXNE, DISABLE);//关接收中断
    USART_ITConfig(UART4, USART_IT_IDLE, ENABLE);//开启空闲总线中断
    BSP_IntVectSet(BSP_INT_ID_USART4, BSP_SerToWIFI_ISR_Handler);   //中断向量表设置
    BSP_IntEn(BSP_INT_ID_USART4);

    UART4_DMA_NVIC_Init();

    USART_Cmd(UART4, ENABLE);
}

/*
***************************************************************************************************
*                                   UART4_DMA_NVIC_Init()
*
* Description : Uart4 DMA NUIV init.
*               
* Argument(s) : none.
*
* Return(s)   : none.
*
* Note(s)     : none.
***************************************************************************************************
*/
void UART4_DMA_NVIC_Init()
{
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);  //使能DMA2时钟

    NVIC_InitStructure.NVIC_IRQChannel = DMA2_Channel5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = DMA2_Channel3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
   
    OSMutexCreate(&g_stWIFISerTxMutex,
                  "WIFI serial mutex",
                  NULL);
}


void BSP_SerToWIFI_TxMsgInit(u8 *TxBuffAddr, uint8_t i_u8TxBuffSize)
{
    DMA_InitTypeDef DMA_InitStructure;

    /* DMA2 channel5 configuration ----------------------------------------------*/
    DMA_DeInit(DMA2_Channel5);
    DMA_InitStructure.DMA_PeripheralBaseAddr = UART4_DR_Address;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) TxBuffAddr;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;              //外设作为数据传输目的地
    DMA_InitStructure.DMA_BufferSize = i_u8TxBuffSize;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设地址寄存器不变
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;     //内存地址寄存器递增
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;    //禁止内存到内存
    DMA_Init(DMA2_Channel5, &DMA_InitStructure);
    /* Enable DMA2 channel5 */

    DMA_ITConfig(DMA2_Channel5, DMA_IT_TC, ENABLE);

    BSP_IntVectSet(BSP_INT_ID_DMA2_CH5, WIFI_DataTx_IRQHandler);
    BSP_IntEn(BSP_INT_ID_DMA2_CH5);

//    DMA_Cmd(DMA2_Channel5, DISABLE);   //关闭USART1 TX DMA1 所指示的通道
//    DMA_SetCurrDataCounter(DMA2_Channel5, i_u8TxBuffSize); //DMA通道的DMA缓存的大小
    DMA_Cmd(DMA2_Channel5, ENABLE);
    USART_DMACmd(UART4, USART_DMAReq_Tx, ENABLE);

}


void BSP_SerToWIFI_RxMsgInit(uint8_t *i_u32RxBuffAddr, uint8_t i_u8RxBuffSize)
{
    DMA_InitTypeDef DMA_InitStructure;

    /* DMA2 channel3 configuration */
    DMA_DeInit(DMA2_Channel3);
    DMA_InitStructure.DMA_PeripheralBaseAddr = UART4_DR_Address;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) i_u32RxBuffAddr;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = i_u8RxBuffSize;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA2_Channel3, &DMA_InitStructure);
    /* Enable DMA2 channel3 */

    DMA_ITConfig(DMA2_Channel3, DMA_IT_TC, ENABLE);

    USART_DMACmd(UART4, USART_DMAReq_Rx, ENABLE);
    DMA_Cmd(DMA2_Channel3, ENABLE);

    BSP_IntVectSet(BSP_INT_ID_DMA2_CH3, WIFI_DataRx_IRQHandler);
    BSP_IntEn(BSP_INT_ID_DMA2_CH3);
}


void BSP_PrgmDataDMASend(uint8_t i_u8TxBuffSize, uint8_t *TxBuff)
{
    OS_ERR      err;
    //获得一个互斥信号量
    OSMutexPend(&g_stWIFISerTxMutex,    //一个指向互斥锁的指针
                10,                     //超时时间
                OS_OPT_PEND_BLOCKING,   //决定是否阻止用户，如果这个互斥锁不可用
                NULL,                   //是一个指针指向一个时间戳当互斥锁发送时，当发送终止或互斥锁被删除
                &err);

    BSP_SerToWIFI_TxMsgInit(TxBuff, i_u8TxBuffSize);    //发送数据地址初始化
}


/*
***************************************************************************************************
*                                         WIFI_DataRx_IRQHandler()
*
* Description : WIFI DATA receive ISR.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : This is an ISR.
*
* Note(s)     : none.
***************************************************************************************************
*/
void WIFI_DataRx_IRQHandler(void)
{
    OS_ERR      err;
    CPU_SR_ALLOC();

    if(DMA_GetITStatus(DMA2_IT_TC3) != RESET) {
        if(g_u8WifiCommandReceived == NO) {
            CPU_CRITICAL_ENTER();
            OSTimeDlyResume(&CommunicateTaskTCB,
                            &err);
            g_u8WifiCommandReceived = YES;
            CPU_CRITICAL_EXIT();
        }

        DMA_ClearFlag(DMA2_FLAG_TC3);
        DMA_ClearITPendingBit(DMA2_IT_TC3);  //清除DMA1_IT_TC3更新中断标志
    }

    DMA_ClearITPendingBit(DMA2_IT_TE3);
    DMA_ClearITPendingBit(DMA2_IT_HT3);
    DMA_ClearITPendingBit(DMA2_IT_GL3);
}

/*
***************************************************************************************************
*                                         WIFI_DataTx_IRQHandler()
*
* Description : WIFI DATA send over ISR.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : This is an ISR.
*
* Note(s)     : 释放互斥锁.
***************************************************************************************************
*/
void WIFI_DataTx_IRQHandler(void)
{
    OS_ERR      err;

    if(DMA_GetITStatus(DMA2_IT_TC5) != RESET) { //检查DMA1_IT_TC5中断发生与否
        OSMutexPost(&g_stWIFISerTxMutex,
                    OS_OPT_POST_NONE,
                    &err);

        DMA_ClearFlag(DMA2_FLAG_TC5);
        DMA_ClearITPendingBit(DMA2_IT_TC5);  //清除DMA1_IT_TC5更新中断标志
    }

    DMA_ClearITPendingBit(DMA2_IT_TE5);         //清传输错误标志
    DMA_ClearITPendingBit(DMA2_IT_HT5);         //清传输过半中断
    DMA_ClearITPendingBit(DMA2_IT_GL5);         //清全局中断
}
/*
***************************************************************************************************
*                                         BSP_SerToWIFI_ISR_Handler()
*
* Description : WIFI DATA ISR.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : This is an ISR.
*
* Note(s)     : none.
***************************************************************************************************
*/
void BSP_SerToWIFI_ISR_Handler(void)
{
    uint8_t u8RxByte;

    if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET) {
    }

    if(USART_GetITStatus(UART4, USART_IT_IDLE) != RESET) {
        USART_DMACmd(UART4, USART_DMAReq_Rx, ENABLE);       //使能串口4DMA请求
        DMA_ITConfig(DMA2_Channel3, DMA_IT_TC, DISABLE);
        DMA_ITConfig(DMA2_Channel3, DMA_IT_TE, DISABLE);
        DMA_Cmd(DMA2_Channel3, DISABLE);

        BSP_SerToWIFI_RxMsgInit(GetPrgmRxBuffAddr(), GetPrgmRxBuffLen());
        /*
            DMA_InitTypeDef DMA_InitStructure;

            DMA_DeInit(DMA2_Channel3);
            DMA_InitStructure.DMA_PeripheralBaseAddr = UART4_DR_Address;
            DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) i_u32RxBuffAddr;
            DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
            DMA_InitStructure.DMA_BufferSize = i_u8RxBuffSize;
            DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
            DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
            DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
            DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//DMA_MemoryDataSize_HalfWord;
            DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
            DMA_InitStructure.DMA_Priority = DMA_Priority_High;
            DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
            DMA_Init(DMA2_Channel3, &DMA_InitStructure);

            DMA_ITConfig(DMA2_Channel3,DMA_IT_TC,ENABLE);
            USART_DMACmd(UART4, USART_DMAReq_Rx, ENABLE);
            DMA_Cmd(DMA2_Channel3, ENABLE);

            BSP_IntVectSet(BSP_INT_ID_DMA2_CH3, WIFI_DataRx_IRQHandler);
            BSP_IntEn(BSP_INT_ID_DMA2_CH3);
        */
        u8RxByte = UART4->SR;
        u8RxByte = UART4->DR;
        u8RxByte = u8RxByte;
        USART_ClearITPendingBit(UART4, USART_IT_IDLE);
    }
}

/*
***************************************************************************************************
*                                           BSP_Ser_Printf()
*
* Description : Print formatted data to the output serial port.
*
* Argument(s) : format      String that contains the text to be written.
*
* Return(s)   : none.
*
* Caller(s)   : Application.
*
* Note(s)     : (1) This function output a maximum of BSP_SER_PRINTF_STR_BUF_SIZE number of bytes to the
*                   serial port.  The calling function hence has to make sure the formatted string will
*                   be able fit into this string buffer or hence the output string will be truncated.
***************************************************************************************************
*/

void  BSP_Ser_Printf(CPU_CHAR  *format, ...)
{
    CPU_CHAR  buf_str[BSP_SER_PRINTF_STR_BUF_SIZE + 1u];
    va_list   v_args;


    va_start(v_args, format);
    (void)vsnprintf((char *)&buf_str[0],
                    (size_t) sizeof(buf_str),
                    (char const *) format,
                    v_args);
    va_end(v_args);

    BSP_Ser_WrStr(buf_str);
}


/*
***************************************************************************************************
*                                                BSP_Ser_RdByte()
*
* Description : Receive a single byte.
*
* Argument(s) : none.
*
* Return(s)   : The received byte.
*
* Caller(s)   : Application.
*
* Note(s)     : (1) This functions blocks until a data is received.
*
*               (2) It can not be called from an ISR.
***************************************************************************************************
*/

CPU_INT08U  BSP_Ser_RdByte(void)
{
    CPU_INT08U  rx_byte;


    BSP_OS_SemWait(&BSP_SerLock, 0);                            /* Obtain access to the serial interface.             */

    rx_byte = BSP_Ser_RdByteUnlocked();

    BSP_OS_SemPost(&BSP_SerLock);                               /* Release access to the serial interface.            */

    return (rx_byte);
}


/*
***************************************************************************************************
*                                       BSP_Ser_RdByteUnlocked()
*
* Description : Receive a single byte.
*
* Argument(s) : none.
*
* Return(s)   : The received byte.
*
* Caller(s)   : BSP_Ser_RdByte()
*               BSP_Ser_RdStr()
*
* Note(s)     : none.
***************************************************************************************************
*/

CPU_INT08U  BSP_Ser_RdByteUnlocked(void)
{

    CPU_INT08U   rx_byte;


    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);              /* Enable the Receive not empty interrupt             */

    BSP_OS_SemWait(&BSP_SerRxWait, 0);                          /* Wait until data is received                        */

    USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);             /* Disable the Receive not empty interrupt            */

    rx_byte = BSP_SerRxData;                                    /* Read the data from the temporary register          */

    return (rx_byte);
}

/*
***************************************************************************************************
*                                                BSP_Ser_RdStr()
*
* Description : This function reads a string from a UART.
*
* Argument(s) : p_str       A pointer to a buffer at which the string can be stored.
*
*               len         The size of the string that will be read.
*
* Return(s)   : none.
*
* Caller(s)   : Application
*
* Note(s)     : none.
***************************************************************************************************
*/

void  BSP_Ser_RdStr(CPU_CHAR    *p_str,
                    CPU_INT16U   len)
{
    CPU_CHAR     *p_char;
    CPU_BOOLEAN   rxd_history_char0;
    CPU_CHAR      rx_data;
    CPU_BOOLEAN   err;


    rxd_history_char0 = DEF_NO;
    p_str[0]          = (CPU_CHAR)'\0';
    p_char            = p_str;

    err = BSP_OS_SemWait(&BSP_SerLock, 0);                      /* Obtain access to the serial interface                */

    if(err != DEF_OK) {
        return;
    }

    while(DEF_TRUE) {
        rx_data = BSP_Ser_RdByteUnlocked();

        if((rx_data == ASCII_CHAR_CARRIAGE_RETURN) ||           /* Is it '\r' or '\n' character  ?                      */
                (rx_data == ASCII_CHAR_LINE_FEED)) {

            BSP_Ser_WrByteUnlocked((CPU_INT08U)ASCII_CHAR_LINE_FEED);
            BSP_Ser_WrByteUnlocked((CPU_INT08U)ASCII_CHAR_CARRIAGE_RETURN);
            *p_char = (CPU_CHAR)'\0';                            /* set the null character at the end of the string      */
#if (BSP_CFG_SER_CMD_HISTORY_LEN > 0u)
            Str_Copy(BSP_SerCmdHistory, p_str);
#endif
            break;                                              /* exit the loop                                        */
        }

        if(rx_data == ASCII_CHAR_BACKSPACE) {                   /* Is backspace character                               */
            if(p_char > p_str) {
                BSP_Ser_WrByteUnlocked((CPU_INT08U)ASCII_CHAR_BACKSPACE);
                p_char--;                                       /* Decrement the index                                  */
            }
        }

        if((ASCII_IsPrint(rx_data)) &&
                (rxd_history_char0 == DEF_NO)) {                    /* Is it a printable character ... ?                    */
            BSP_Ser_WrByteUnlocked((CPU_INT08U)rx_data);        /* Echo-back                                            */
            *p_char = rx_data;                                   /* Save the received character in the buffer            */
            p_char++;                                           /* Increment the buffer index                           */

            if(p_char >= &p_str[len]) {
                p_char  = &p_str[len];
            }

        } else if((rx_data           == ASCII_CHAR_ESCAPE) &&
                  (rxd_history_char0 == DEF_NO)) {
            rxd_history_char0 = DEF_YES;

#if (BSP_CFG_SER_CMD_HISTORY_LEN > 0u)
        } else if((rx_data           == ASCII_CHAR_LEFT_SQUARE_BRACKET) &&
                  (rxd_history_char0 == DEF_YES)) {

            while(p_char != p_str) {
                BSP_Ser_WrByteUnlocked((CPU_INT08U)ASCII_CHAR_BACKSPACE);
                p_char--;                                       /* Decrement the index                                  */
            }

            Str_Copy(p_str, BSP_SerCmdHistory);

            while(*p_char != '\0') {
                BSP_Ser_WrByteUnlocked(*p_char++);
            }

#endif
        } else {
            rxd_history_char0 = DEF_NO;
        }
    }

    BSP_OS_SemPost(&BSP_SerLock);                               /* Release access to the serial interface               */
}


/*
***************************************************************************************************
*                                          BSP_Ser_WrByteUnlocked()
*
* Description : Writes a single byte to a serial port.
*
* Argument(s) : c           The character to output.
*
* Return(s)   : none.
*
* Caller(s)   : BSP_Ser_WrByte()
*               BSP_Ser_WrByteUnlocked()
*
* Note(s)     : (1) This function blocks until room is available in the UART for the byte to be sent.
***************************************************************************************************
*/

void  BSP_Ser_WrByteUnlocked(CPU_INT08U c)
{
    USART_ITConfig(USART1, USART_IT_TC, ENABLE);
    USART_SendData(USART1, c);
    BSP_OS_SemWait(&BSP_SerTxWait, 0);
    USART_ITConfig(USART1, USART_IT_TC, DISABLE);
}


/*
***************************************************************************************************
*                                                BSP_Ser_WrByte()
*
* Description : Writes a single byte to a serial port.
*
* Argument(s) : tx_byte     The character to output.
*
* Return(s)   : none.
*
* Caller(s)   : Application.
*
* Note(s)     : none.
***************************************************************************************************
*/

void  BSP_Ser_WrByte(CPU_INT08U  c)
{
    BSP_OS_SemWait(&BSP_SerLock, 0);                            /* Obtain access to the serial interface              */

    BSP_Ser_WrByteUnlocked(c);

    BSP_OS_SemPost(&BSP_SerLock);                               /* Release access to the serial interface             */
}


/*
***************************************************************************************************
*                                                BSP_Ser_WrStr()
*
* Description : Transmits a string.
*
* Argument(s) : p_str       Pointer to the string that will be transmitted.
*
* Caller(s)   : Application.
*
* Return(s)   : none.
*
* Note(s)     : none.
***************************************************************************************************
*/

void  BSP_Ser_WrStr(CPU_CHAR  *p_str)
{
    CPU_BOOLEAN  err;


    if(p_str == (CPU_CHAR *)0) {
        return;
    }


    err = BSP_OS_SemWait(&BSP_SerLock, 0);                      /* Obtain access to the serial interface              */

    if(err != DEF_OK) {
        return;
    }

    while((*p_str) != (CPU_CHAR)0) {
        if(*p_str == ASCII_CHAR_LINE_FEED) {
            BSP_Ser_WrByteUnlocked(ASCII_CHAR_CARRIAGE_RETURN);
            BSP_Ser_WrByteUnlocked(ASCII_CHAR_LINE_FEED);
            p_str++;
        } else {
            BSP_Ser_WrByteUnlocked(*p_str++);
        }
    }

    BSP_OS_SemPost(&BSP_SerLock);                               /* Release access to the serial interface             */
}

