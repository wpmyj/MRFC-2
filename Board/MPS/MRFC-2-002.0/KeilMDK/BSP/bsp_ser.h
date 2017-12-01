/*
***************************************************************************************************
*
*                                    MICRIUM BOARD SUPPORT PACKAGE
*
*                          (c) Copyright 2003-2010; Micrium, Inc.; Weston, FL
*
*               All rights reserved.  Protected by international copyright laws.
*
*               This BSP is provided in source form to registered licensees ONLY.  It is
*               illegal to distribute this source code to any third party unless you receive
*               written permission by an authorized Micrium representative.  Knowledge of
*               the source code may NOT be used to develop a similar product.
*
*               Please help us continue to provide the Embedded community with the finest
*               software available.  Your honesty is greatly appreciated.
*
*               You can contact us at www.micrium.com.
***************************************************************************************************
*/

/*
***************************************************************************************************
*
*                                    MICRIUM BOARD SUPPORT PACKAGE
*                                       SERIAL (UART) INTERFACE
*
* Filename      : bsp_ser.h
* Version       : V1.00
* Programmer(s) : EHS
*                 SR
***************************************************************************************************
*/


/*
***************************************************************************************************
*                                             INCLUDE FILES
***************************************************************************************************
*/
#include <includes.h>
#include "cpu.h"
/*
***************************************************************************************************
*                                               MODULE
*
* Note(s) : (1) This BSP serial header file is protected from multiple pre-processor inclusion through
*               use of the BSP serial present pre-processor macro definition.
***************************************************************************************************
*/

#ifndef  BSP_SER_PRESENT                                        /* See Note #1.                                         */
#define  BSP_SER_PRESENT


/*
***************************************************************************************************
*                                               EXTERNS
***************************************************************************************************
*/

#ifdef   BSP_SER_MODULE
    #define  BSP_SER_EXT
#else
    #define  BSP_SER  extern
#endif


/*
***************************************************************************************************
*                                               DEFINES
***************************************************************************************************
*/

#define  BSP_SER_COMM_UART_NONE                0xFF
#define  BSP_SER_COMM_UART_01                     1

#define  BSP_SER_PRINTF_STR_BUF_SIZE             80u


/*
***************************************************************************************************
*                                          GLOBAL VARIABLES
***************************************************************************************************
*/


/*
***************************************************************************************************
*                                               MACRO'S
***************************************************************************************************
*/


/*
***************************************************************************************************
*                                   BACKWARD COMPATIBILITY MACRO'S
*
* Note(s) : (1) Those macro's are provided to preserve backward compatibility with earlier versions of
*               this BSP.
***************************************************************************************************
*/

/* See Note #1.                                         */
#define  Ser_Init                       BSP_Ser_Init
#define  Ser_WrByte                     BSP_Ser_WrByte
#define  Ser_WrStr                      BSP_Ser_WrStr
#define  Ser_RdByte                     BSP_Ser_RdByte
#define  Ser_RdStr                      BSP_Ser_RdStr
#define  Ser_Printf                     BSP_Ser_Printf


/*
***************************************************************************************************
*                                              DATA TYPES
***************************************************************************************************
*/
extern uint8_t  g_u8WifiCmdRec;
extern uint8_t  g_u83GCommandReceived;

extern uint8_t  usart2_recv_buff[1024];
extern uint16_t recv_cursor;

extern u8  HydrogenProducerTxBuf[32]; //注意变量数组不要溢出
extern u8  FuelCellTxBuf[32];//燃料电池发送

/*
***************************************************************************************************
*                                         FUNCTION PROTOTYPES
***************************************************************************************************
*/

/* --------------- RS-232 SERVICE FNCTS --------------- */
void        BSP_Ser_Init(CPU_INT32U   baud_rate);

CPU_INT08U  BSP_Ser_RdByte(void);
void        BSP_Ser_RdStr(CPU_CHAR    *p_str,
                          CPU_INT16U   len);

void        BSP_Ser_WrByte(CPU_INT08U   c);
void        BSP_Ser_WrStr(CPU_CHAR    *p_str);

void        BSP_Ser_Printf(CPU_CHAR    *format,
                           ...);
//void        BSP_Ser_WrByteUnlocked  (CPU_INT08U  c);

void  BSP_SerToWIFI_Init(void);
void  BSP_SerToWIFI_TxMsgInit(uint8_t *TxBuffAddr, uint8_t);
void  BSP_SerToWIFI_RxMsgInit(uint8_t *RxBuffAddr, uint8_t);
void  BSP_PrgmDataDMASend(uint8_t i_u8TxBuffSize, uint8_t *TxBuff);

void  BSP_Ser_To_3G_Init(CPU_INT32U  baud_rate);

void  CleanUsartRecvBuf(USART_TypeDef *USARTx);
void  Uart2SendBuffer(u8 *buffer, u8 count);
void  SendString(USART_TypeDef *USARTx, const char *s);
/*
***************************************************************************************************
*                                        CONFIGURATION ERRORS
***************************************************************************************************
*/

#ifndef  BSP_CFG_SER_COMM_SEL
    #error  "BSP_CFG_SER_COMM_SEL                 not #define'd in 'app_cfg.h'        "
    #error  "                               [MUST be BSP_SER_COMM_UART_01]            "

#elif   (BSP_CFG_SER_COMM_SEL != BSP_SER_COMM_UART_01)
    #error  "BSP_CFG_SER_COMM_SEL           illegally #define'd in 'app_cfg.h'        "
    #error  "                               [MUST be BSP_SER_COMM_UART_01]            "
#endif


/*
***************************************************************************************************
*                                             MODULE END
*
* Note(s) : (1) See 'bsp_ser.h  MODULE'.
***************************************************************************************************
*/

#endif                                                          /* End of BSP ser module include.                       */
