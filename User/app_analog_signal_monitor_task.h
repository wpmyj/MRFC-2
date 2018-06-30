/*********************************************************************************************************
*                                         APPLICATION CODE
*
*                      (c) Copyright 2016; Guangdong ENECO Science And Technology Co.,Ltd
*
*               All rights reserved.  Protected by international copyright laws.
*               Knowledge of the source code may NOT be used without authorization.
***************************************************************************************************
*/

/*
*******************************************************************************************************
* Filename      :  app_analog_signal_monitor_task.h
* Programmer(s) :  JasonFan
* Version       :  V1.0
* data          :  2016.5.8
* brief         :  This file contains all the functions prototypes for the system run
*                  config parameters firmware library.
********************************************************************************************************
*/

#ifndef __APP_ANALOG_SIGNAL_MONITOR_TASK_H__
#define __APP_ANALOG_SIGNAL_MONITOR_TASK_H__


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
*                                    EXTERNAL OS VARIABLE DECLARATIONS
***************************************************************************************************
*/
extern      OS_SEM      g_stAnaSigConvertFinishSem;
extern      OS_TCB      AnaSigMonitorTaskTCB;
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
void    AnaSigMonitorTaskCreate(void);
void    SetHydrgProducerAnaSigAlarmRunningMonitorHookSwitch(unsigned char);
void    SetPumpStartAutoAdjHookSwitch(uint8_t i_NewStatu);
void    SetStackIsPulledStoppedMonitorHookSwitch(uint8_t i_NewStatu);
uint8_t GetRestartLimitCurrentFlagStatus(void);
void 	SetRestartLimitCurrentFlagStatus(uint8_t i_NewStatu);

void    StartRunningBeepAlarm(SYSTEM_ALARM_GRADE_Typedef , uint8_t);
/*
***************************************************************************************************
*                                             MODULE END
***************************************************************************************************
*/


#endif                                                          /* End of module include */
