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

/********************************************************************************************************
* Filename      :  app_stack_manager.h
* Programmer(s) :  JasonFan
* Version       :  V1.0
* data          :  2016.4.4
* brief         :  This file contains all the functions prototypes for the system run
*                  config parameters firmware library.
*********************************************************************************************************/
#ifndef __APP_STACK_MANAGER_H__
#define __APP_STACK_MANAGER_H__
/*
***************************************************************************************************
*                                                 MODULE
*
* Note(s) : (1) This header file is protected from multiple pre-processor inclusion through use of the
*               BSP present pre-processor macro definition.
***************************************************************************************************
*/
/*
***************************************************************************************************
*                                           INCLUDE FILES
***************************************************************************************************
*/
#include "includes.h"
/*
***************************************************************************************************
*                                           MACRO DEFINITIONS
***************************************************************************************************
*/
//等待排杂和工作气压状态
#define  DO_NOT_WAIT      0
#define  WAIT_FOR_36KPA   1
#define  WAIT_FOR_50KPA   2



#define  STACK_FAN_SPOOL_UP_SPD       800
#define  STACK_FAN_SPOOL_DOWN_SPD     200
/*
***************************************************************************************************
*                                    EXPORTED OS VARIABLE DECLARATIONS
***************************************************************************************************
*/
extern      OS_SEM      StackStartSem;
extern      OS_TCB      StackManagerTaskTCB;
extern      OS_TCB      StackManagerDlyStopTaskTCB;
extern      OS_TCB      StackHydrogenYieldMatchingOffsetValueMonitorTaskTCB;


/*
***************************************************************************************************
*                                   EXPORTED GLOABLE VARIABLE DECLARATIONS
***************************************************************************************************
*/
extern      uint16_t                                u16AmpIntegralTimeCount;
extern      uint8_t                                 g_u8DecompressCountPerMinute;
/*
***************************************************************************************************
*                                           EXPORTED FUNCTION
***************************************************************************************************
*/
void    StackManagerTaskCreate(void);
void    StackManagerDlyStopTaskCreate(void);
void 	StackStartUpCtrlTaskCreate(void);

void    SetStackAnaSigAlarmRunningMonitorHookSwitch(u8);

uint8_t GetPassiveDecompressCntPerMin(void);
void 	UpdatePassiveDecompressCntPerMin(void);

uint8_t GetStackStartPurifySwitchStatus(void);

uint8_t GetStackStopDlyStatu(void);

void  SetStackAmpIntegralSum(float i_NewValue);
float GetStackAmpIntegralSum(void);

uint8_t GetStackFanSpdPidControlSwitchStatus(void);
void SetStackFanSpdPidControlSwitch(uint8_t i_NewStatu);

void SetStackStartUpShortTaskSwitch(uint8_t i_NewStatu);
uint8_t GetStackStartUpShortTaskSwitchStatus(void);
/*
***************************************************************************************************
*                                             MODULE END
***************************************************************************************************
*/


#endif                                                          /* End of module include */
