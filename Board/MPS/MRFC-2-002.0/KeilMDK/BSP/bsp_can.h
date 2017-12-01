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
#include "app_wireness_communicate_task.h"
/*
***************************************************************************************************
*                                           EXPORTED VARIABLE
***************************************************************************************************
*/
extern  		uint8_t   				g_eCanMsgRxStatu;
extern          uint8_t                 g_u8CanRxMsg[PRGM_RX_BUFF_SIZE];
extern  		uint8_t   				g_eCAN_BusOnLineFlag;
/*
***************************************************************************************************
*                                           EXPORTED TYPE
***************************************************************************************************
*/
typedef struct {

    uint16_t cob_id;  /**< message's  combine ID */
    uint8_t  rtr;     /**< remote transmission request. (0 if not rtr message, 1 if rtr message) */
    uint8_t  len;     /**< message's length (0 to 8) */
    uint8_t  data[8]; /**< message's datas */

} Message;
/*
***************************************************************************************************
*                                           MACRO DEFINITIONS
***************************************************************************************************
*/

#define CAN1_RX0_INT_ENABLE  1      //CAN1����RX1�ж�ʹ��:0,��ʹ��;1,ʹ��.

/*
***************************************************************************************************
*                                           EXPORTED FUNCTION
***************************************************************************************************
*/
void CAN1_Init(void);

uint8_t CANx_Send_Msg(CAN_TypeDef *CANx, Message *m);
uint8_t SendCanMsgContainNodeId(uint32_t i_Msglen, uint8_t *msg, uint8_t i_NodeId);
uint8_t GetCanBusOnlineFlag(void);
void SetCanBusOnlineFlag(uint8_t i_NewStatus);
u8 CANx_Receive_Msg(CAN_TypeDef *CANx, u8 *i_u8RxBuf);         //CAN��������

#endif













