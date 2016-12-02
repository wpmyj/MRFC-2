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
* Programmer(s) :  Fanjun
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


/*
***************************************************************************************************
*                                    EXPORTED OS VARIABLE DECLARATIONS
***************************************************************************************************
*/

extern      OS_TCB      StackManagerTaskTCB;
extern      OS_TCB      StackManagerDlyStopTaskTCB;
extern      OS_TCB      StackHydrogenYieldMatchingOffsetValueMonitorTaskTCB;
extern      OS_TCB      StackProgramControlAirPressureReleaseTaskTCB;

/*
***************************************************************************************************
*                                           EXPORTED TYPE
***************************************************************************************************
*/
typedef struct {
    uint64_t u32_TimeRecordNum; //计时基数,加1为加0.001s
    float  fVentAirTimeIntervalValue;//排气间隔
    float  fDecompressVentTimeValue;//泄压时间

} STACK_VENTING_TIME_PARAMETER_Typedef;

/*
***************************************************************************************************
*                                   EXPORTED GLOABLE VARIABLE DECLARATIONS
***************************************************************************************************
*/
extern      uint16_t                                u16AmpIntegralTimeCount;
extern      STACK_VENTING_TIME_PARAMETER_Typedef    StackVentAirTimeParameter;

/*
***************************************************************************************************
*                                           EXPORTED FUNCTION
***************************************************************************************************
*/
void    StackManagerTaskCreate(void);
void    StackManagerDlyStopTaskCreate(void);
void    StackHydrogenYieldMatchingOffsetValueMonitorTaskCreate(void);
void    StackProgramControlPressureReleaseTaskCreate(void);


uint8_t GetStackProgramControlAirPressureReleaseTaskSwitch(void);

void    SetStackAnaSigAlarmRunningMonitorHookSwitch(u8);
void    SetStackOutAirValveStatus(SWITCH_TYPE_VARIABLE_Typedef i_NewStatu);


void    ResetStackExhaustTimesCountPerMinutes(void);
float   GetStackHydrogenYieldMatchOffsetValue(void);

uint8_t GetStackStartPurifySwitchStatus(void);

void SetStackNeedVentByAmpIntegralSumSwitch(uint8_t i_NewStatu);
uint8_t GetStackNeedVentByAmpIntegralSumSwitch(void);

uint8_t GetSStackHydrogenMarginMonitorTaskSwitchStatus(void);
SWITCH_TYPE_VARIABLE_Typedef GetStackOutAirValveStatus(void);

uint8_t GetStackStopDlyStatu(void);

void  SetStackAmpIntegralSum(float i_NewValue);
float GetStackAmpIntegralSum(void);
/*
***************************************************************************************************
*                                             MODULE END
***************************************************************************************************
*/


#endif                                                          /* End of module include */
