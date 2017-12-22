/*
***************************************************************************************************
*                                         APPLICATION CODE
*
*                      (c) Copyright 2016; Guangdong ENECO Science And Technology Co.,Ltd
*
*               All rights reserved.  Protected by international copyright laws.
*               Knowledge of the source code may NOT be used without authorization.
***************************************************************************************************
*/
/*
***************************************************************************************************
* Filename      : bsp_scale_data_read.c
* Version       : V1.00
* Programmer(s) : JasonFan
***************************************************************************************************
*/


/*
***************************************************************************************************
*                                             INCLUDE FILES
***************************************************************************************************
*/

#include <bsp.h>
#include "bsp_scale_data_read.h"
#include "app_system_real_time_parameters.h"
#include "stdlib.h"
/*
***************************************************************************************************
*                                            MACRO DEFINES
***************************************************************************************************
*/
#define USART5_REC_LEN   20          //串口接收数据长度
/*
***************************************************************************************************
*                                       LOCAL GLOBAL VARIABLES
***************************************************************************************************
*/
static  BSP_OS_SEM   BSP_Ser5TxWait;
static  BSP_OS_SEM   BSP_Ser5RxWait;
static  BSP_OS_SEM   BSP_Ser5Lock;

uint8_t UART5_RX_CNT = 0;
u8 g_Rx485Flag = 0; //485接收判断
u8 u8VariationWeightOfGram[61] = {0}; //保存一分钟的读秤变化量(多保存一个，用于减去最久的变化量)
u8 u8LiquidFlowPointer = 0;             //指向当前保存变化量位置的指针

uint8_t   g_DataReadFlag = 0;   //串口数据读取标志,0:表示数据读取完成;1:表示可以开始下一次数据读取;

//接收状态标志,bit15:接收完成标志;bit14:接收到0x0d;bit13~0:接收到的有效字节数目
uint16_t  USART5_RX_STA;
uint8_t   Uart5_RxBuf[USART5_REC_LEN];//485接收数据

/*
***************************************************************************************************
*                                      LOCAL FUNCTION PROTOTYPES
***************************************************************************************************
*/
static  void            BSP_Scale_SerTo485_ISR_Handler(void);

/*
*********************************************************************************************************
*                                          BSP_SerTo485_Init()
*
* Description : Initialize a serial port for communication to 485.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Note(s)     : none.
*********************************************************************************************************
*/
void  BSP_Scale_SerTo485_Init()
{
    FlagStatus              tc_status;
    GPIO_InitTypeDef        GPIO_InitStructure;
    NVIC_InitTypeDef        NVIC_InitStructure;
    USART_InitTypeDef       USART_InitStructure;
    USART_ClockInitTypeDef  usart_clk_init;

    BSP_PeriphEn(BSP_PERIPH_ID_USART5);
    BSP_PeriphEn(BSP_PERIPH_ID_IOPC);
    BSP_PeriphEn(BSP_PERIPH_ID_IOPD);
    BSP_PeriphEn(BSP_PERIPH_ID_AFIO);

    GPIO_InitStructure.GPIO_Pin   = BSP_GPIOC_UART5_TX_PORT_NMB;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin   = BSP_GPIOD_PIN2_UART5_RX_PORT_NMB;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate            = 9600;
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;//字长为8位数据格式
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;
    USART_InitStructure.USART_Parity              = USART_Parity_No ;//无奇偶校验位
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
    USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx; //收发模式
    USART_Init(UART5, &USART_InitStructure);

    usart_clk_init.USART_Clock           = USART_Clock_Disable;
    usart_clk_init.USART_CPOL            = USART_CPOL_Low;
    usart_clk_init.USART_CPHA            = USART_CPHA_2Edge;
    usart_clk_init.USART_LastBit         = USART_LastBit_Disable;
    USART_ClockInit(UART5, &usart_clk_init);

    USART_Cmd(UART5, ENABLE);

    USART_ITConfig(UART5, USART_IT_TC, DISABLE);
    USART_ITConfig(UART5, USART_IT_TXE, DISABLE);
    tc_status  = USART_GetFlagStatus(UART5, USART_FLAG_TC);

    while(tc_status == SET) {
        USART_ClearITPendingBit(UART5, USART_IT_TC);
        USART_ClearFlag(UART5, USART_IT_TC);
        BSP_OS_TimeDlyMs(10);
        tc_status = USART_GetFlagStatus(UART5, USART_FLAG_TC);
    }

    /* Configure and enable UART5 interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);
    BSP_IntVectSet(BSP_INT_ID_USART5, BSP_Scale_SerTo485_ISR_Handler);
    BSP_IntEn(BSP_INT_ID_USART5);
    USART_Cmd(UART5, ENABLE);
}

/*
*********************************************************************************************************
*                                BSP_Ser_WrByteUnlocked_USART5()
*
* Description : Initialize a serial port for communication to 485.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Note(s)     : none.
*********************************************************************************************************
*/
void  BSP_Ser_WrByteUnlocked_USART5(CPU_INT08U c)
{
    USART_ITConfig(UART5, USART_IT_TC, ENABLE);
    USART_SendData(UART5, c);
    BSP_OS_SemWait(&BSP_Ser5TxWait, 0);
    USART_ITConfig(UART5, USART_IT_TC, DISABLE);
}

/*
*********************************************************************************************************
*                                          BSP_SerTo485_Init()
*
* Description : Initialize a serial port for communication to 485.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Note(s)     : none.
*********************************************************************************************************
*/
void  BSP_Ser_WrByte_USART5(CPU_INT08U  c)
{
    BSP_OS_SemWait(&BSP_Ser5Lock, 0);                            /* Obtain access to the serial interface              */
    BSP_Ser_WrByteUnlocked_USART5(c);
    BSP_OS_SemPost(&BSP_Ser5Lock);                               /* Release access to the serial interface             */

}
/*
*********************************************************************************************************
*                           BSP_Scale_SerTo485_ISR_Handler()
*
* Description : Initialize a serial port for communication to 485.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : Application.
*
* Note(s)     : none.
*********************************************************************************************************
*/
void  BSP_Scale_SerTo485_ISR_Handler(void)
{
    uint8_t     Rs585_RxData = 0;
    FlagStatus  tc_status;
    FlagStatus  rxne_status;

    rxne_status = USART_GetFlagStatus(UART5, USART_FLAG_RXNE);

    if(rxne_status == SET) {

        Rs585_RxData = USART_ReceiveData(UART5) & 0xFF;     /* Read one byte from the receive data register.      */
        USART_ClearITPendingBit(UART5, USART_IT_RXNE);      /* Clear the UART5 receive interrupt.                 */
        BSP_OS_SemPost(&BSP_Ser5RxWait);                    /* Post to the sempahore                                */

        //485读取秤数据
        if(g_DataReadFlag == 1) { //数据读取完成，可以开始下一次接收
            Uart5_RxBuf[UART5_RX_CNT++] = Rs585_RxData;

            if(UART5_RX_CNT > USART5_REC_LEN) {
                UART5_RX_CNT = 0;//数据溢出
            }
        }
    }

    tc_status = USART_GetFlagStatus(UART5, USART_FLAG_TC);

    if(tc_status == SET) {
        USART_ITConfig(UART5, USART_IT_TC, DISABLE);
        USART_ClearITPendingBit(UART5, USART_IT_TC);            /* Clear the UART5 receive interrupt.                 */
        BSP_OS_SemPost(&BSP_Ser5TxWait);                        /* Post to the semaphore                              */
    }
}
/*
*********************************************************************************************************
*                                     ReadElectronicScaleData_gram()
*
* Description : 485读取电子称函数
*
* Argument(s) : none.
*
* Return(s)   : 读取重量（单位:g  ； 返回值放大两倍）范围2~30000  超出或小于2或出错都返回0
*
* Note(s)     : 只能解析g并返回相应读数，无法解析其他单位（如：kg）
*********************************************************************************************************
*/
uint16_t ReadElectronicScaleData_gram(void)
{
    uint16_t GRAM_CHANGE_TABLE[15][2] = { {0x4000, 0x4000}, //2g   第0位对应当前重量的值（前两个字节）   第1位对应前一个重量到当前重量每偏移一位（0.5g）增加的值（中间两个字节） ps：一个float型数共4个字节
        {0x4080, 0x2000}, //4g
        {0x4100, 0x1000}, //8g
        {0x4180, 0x0800}, //16g
        {0x4200, 0x0400}, //32g
        {0x4280, 0x0200}, //64g
        {0x4300, 0x0100}, //128g
        {0x4380, 0x0080}, //256g
        {0x4400, 0x0040}, //512g
        {0x4480, 0x0020}, //1024g
        {0x4500, 0x0010}, //2048g
        {0x4580, 0x0008}, //4096g
        {0x4600, 0x0004}, //8192g
        {0x4680, 0x0002}, //16384g
        {0x4700, 0x0001}, //32768g
    };
    u8 i;
    uint16_t u16GrossWeightOfGram;//所得毛重（重量x2   单位:g）
    static uint16_t u16LastWeightOfGram = 0;//记录上一次读取的重量
    int TableAddData;//比菜单数值多于的位数(每位代表0.5g)
    u8 CHECK_485_CMD[8] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x02, 0xC4, 0x0B}; //485发送查询命令（modbus格式）

    for(i = 0; i < 8; i++) {   //发送查询指令
        BSP_Ser_WrByte_USART5(CHECK_485_CMD[i]);
    }

    for(i = 0; i < 255; i++); //简易延时，等待接收数据

    if((Uart5_RxBuf[3] & 0xf0) == 0x40 && g_Rx485Flag == 0 && Uart5_RxBuf[0] == 0x01 && Uart5_RxBuf[1] == 0x03 && Uart5_RxBuf[2] == 0x04) { //返回的值大于或等于2g 且接收数据不会出错
        i = 14;

        while(i < 255) { //防止数据超出范围时跳不出死循环
            if((uint16_t)(Uart5_RxBuf[3] << 8 | Uart5_RxBuf[4]) < GRAM_CHANGE_TABLE[i][0]) { //对比数据菜单得到对应的数值
                i--;
            } else {
                TableAddData = ((uint16_t)(Uart5_RxBuf[4] << 8 | Uart5_RxBuf[5]) - (uint16_t)(GRAM_CHANGE_TABLE[i][0] << 8)) / GRAM_CHANGE_TABLE[i + 1][1]; //得到比数据菜单数值多于的位数
                break;
            }
        }

        if(i <= 14) { //防止数据溢出
            u16GrossWeightOfGram = pow(2, i + 2) + TableAddData; //计算出总重量并保存到浮点数中
        } else {
            u16GrossWeightOfGram = 0;
        }
    } else {
        u16GrossWeightOfGram = 0;
    }

    for(i = 0; i < 20; i++) {
        Uart5_RxBuf[i] = '\0';
    }

    g_Rx485Flag = 1;//代表数据读取完毕，可以开始下一次数据读取

    //保存重量变化量,用于计算每分钟进液量
    if((u16GrossWeightOfGram >= u16LastWeightOfGram) || (u16LastWeightOfGram > u16GrossWeightOfGram + 10)) {
        u8VariationWeightOfGram[u8LiquidFlowPointer] = 0;
    } else {
        u8VariationWeightOfGram[u8LiquidFlowPointer] = (u8)(u16LastWeightOfGram - u16GrossWeightOfGram);
    }

    u16LastWeightOfGram = u16GrossWeightOfGram;

    return u16GrossWeightOfGram;
}

/*
*********************************************************************************************************
*                                     ReadLiquidFlowRate()
*
* Description : 读取进液量.
*
* Argument(s) : none.
*
* Return(s)   : 进液量(单位:g/min ； 返回值放大两倍)
*
* Note(s)     :读取进液量范围0-256(含小数)
*********************************************************************************************************
*/
u8 ReadLiquidFlowRate(void)
{
    static u8 u8LiquidFlowRate = 0;

    if(u8LiquidFlowPointer >= 60) {
        u8LiquidFlowRate = u8LiquidFlowRate + u8VariationWeightOfGram[60] - u8VariationWeightOfGram[0];
        u8LiquidFlowPointer = 0;
    } else {
        u8LiquidFlowRate = u8LiquidFlowRate + u8VariationWeightOfGram[u8LiquidFlowPointer] - u8VariationWeightOfGram[u8LiquidFlowPointer + 1];
        u8LiquidFlowPointer++;
    }

    if(u8LiquidFlowRate > 200) { //出现进液量负值或者超出正常进液范围时清零
        u8LiquidFlowRate = 0;
    } else if(EN_SHUTTING_DOWN == GetSystemWorkStatu() && 0 != u8LiquidFlowRate) { //关机时清空进液量
        u8LiquidFlowRate = 0;
    }

    return u8LiquidFlowRate;
}

/******************* (C) COPYRIGHT 2016 Guangdong ENECO POWER *****END OF FILE****/
