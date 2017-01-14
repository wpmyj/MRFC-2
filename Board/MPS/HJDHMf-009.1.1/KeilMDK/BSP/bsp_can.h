/*
*********************************************************************************************************
*                                         APPLICATION CODE
*
*                      (c) Copyright 2016; Guangdong ENECO Science And Technology Co.,Ltd
*
*               All rights reserved.  Protected by international copyright laws.
*               Knowledge of the source code may NOT be used without authorization.
*********************************************************************************************************
*/

/********************************************************************************************************
* Filename      :  bsp_can.h
* Programmer(s) :  Fanjun
* Version       :  V1.0
* data          :  2016.10.26
* brief         :  This file contains all the functions prototypes for the can communicaion.
*
*********************************************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _CAN_H_
#define _CAN_H_


/*
***************************************************************************************************
*                                           INCLUDE FILES
***************************************************************************************************
*/
#include "includes.h"
#include "stm32f10x_can.h"

/*
***************************************************************************************************
*                                           EXPORTED VARIABLE
***************************************************************************************************
*/
extern uint8_t CAN1_RX_STA;
extern uint8_t CAN2_RX_STA;

extern uint8_t CAN1_RX_DATA_BUF[8];
extern uint8_t CAN2_RX_DATA_BUF[8];
/*
***************************************************************************************************
*                                           MACRO DEFINITIONS
***************************************************************************************************
*/

#define CAN1_RX0_INT_ENABLE  1       //CAN1接收RX0中断使能:0,不使能;1,使能.
#define CAN2_RX0_INT_ENABLE  0       //CAN2接收RX0中断使能:0,不使能;1,使能.

#define NB_MB               8
#define NB_RX_MB            4
#define NB_TX_MB            (NB_MB - NB_RX_MB)

//预编译错误提醒
#if (NB_TX_MB < 1)
    #error define less RX MBs, you must have at least 1 TX MB!
#elif (NB_RX_MB < 1)
    #error define at least 1 RX MBs!
#endif

#define START_TX_MB         NB_RX_MB
#define TX_INT_MSK          ((0xFF << (NB_MB - NB_TX_MB)) & 0xFF)
#define RX_INT_MSK          (0xFF >> (NB_MB - NB_RX_MB))


#define CAN1_ID  0x0012
#define CAN2_ID  0x0013
/*
***************************************************************************************************
*                                           EXPORTED FUNCTION
***************************************************************************************************
*/
void CAN_Configuration(void);

u8 CANx_Send_Msg(CAN_TypeDef *CANx, u16 ID, u8 *i_u8TxMsg, u8 i_u8LenOfFrame); //CAN发送数据
u8 CANx_Receive_Msg(CAN_TypeDef *CANx, u8 *i_u8RxBuf);         //CAN接收数据


#endif













