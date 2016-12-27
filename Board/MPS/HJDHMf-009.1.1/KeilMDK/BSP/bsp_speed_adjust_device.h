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

/********************************************************************************
  * @file    bsp_speed_adjust_device.h
  * @author  Fanjun
  * @version V1.0
  * @date    6-December-2016
  * @brief   This file contains all the functions prototypes for the analog sensor
  *          firmware library.
*********************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __APP_SPEED_ADJUST_DEVICE_H
#define __APP_SPEED_ADJUST_DEVICE_H
/*
***************************************************************************************************
*                                           INCLUDE FILES
***************************************************************************************************
*/
#include "stm32f10x.h"
/*
***************************************************************************************************
*                                           MACRO DEFINITIONS
***************************************************************************************************
*/
typedef enum {
    PUMP_SPD_MONITOR = 0,
    HYDROGEN_FAN_SPD_MONITOR,
    STACK_FAN_SPD_MONITOR,

} SPEED_MONITOR_CHANNEL_Typedef;


/*
***************************************************************************************************
*                                           EXPORTED TYPE
***************************************************************************************************
*/
/*
***************************************************************************************************
*                                           EXPORTED CONSTANTS
***************************************************************************************************
*/

/*
***************************************************************************************************
*                                           EXPORTED MACRO
***************************************************************************************************
*/

/*
***************************************************************************************************
*                                           EXPORTED FUNCTION
***************************************************************************************************
*/

/*
***************************************************************************************************
*                                            FUNCTION PROTOTYPES
***************************************************************************************************
*/

void        PumpSpdInc(void);
void        PumpSpdDec(void);
uint16_t    GetPumpCtlSpd(void);
uint16_t    GetPumpFeedBackSpd(void);
void        SetPumpCtlSpd(uint16_t);

void        HydrgFanSpdInc(void);
void        HydrgFanSpdDec(void);
uint16_t    GetHydrgFanCurrentCtlSpd(void);
uint16_t    GetHydrgFanExpectCtlSpd(void);
uint16_t    GetHydrgFanFeedBackSpd(void);
void        SetHydrgFanCtlSpd(uint16_t);
void        SetHydrgFanCtlSpdSmoothly(uint16_t i_u16CurrentSpdValue,uint8_t i_u8StartDlyAdjTime, uint8_t i_u8CycleDlyAdjTime,uint16_t i_u16ExpSpdValue);


void        StackFanSpdInc(void);
void        StackFanSpdDec(void);
uint16_t    GetStackFanSpdFeedBack(void);
void        SetStackFanCtlSpd(uint16_t);
uint16_t    GetStackFanCtlSpd(void);

void        SetSpdMonitorSwitch(SPEED_MONITOR_CHANNEL_Typedef i_SpdMonitorChannel);
void        ResetSpdMonitorSwitch(SPEED_MONITOR_CHANNEL_Typedef i_SpdMonitorChannel);

void        BSP_DevSpdCaptureFinishedHandler(void);
#endif

