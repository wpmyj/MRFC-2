
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
  * @file    app_stack_short_circuit_task.h
  * @author  Fanjun
  * @version V1.0
  * @date    2017.1.1
  * @brief   This file contains all the functions prototypes for the analog sensor
  *          firmware library.
*********************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __APP_DIGITAL_SIGNAL_MONITOR_TASK_H__
#define __APP_DIGITAL_SIGNAL_MONITOR_TASK_H__
/*
***************************************************************************************************
*                                           INCLUDE FILES
***************************************************************************************************
*/
#include "stm32f10x.h"
#include "os.h"
/*
***************************************************************************************************
*                                           MACRO DEFINITIONS
***************************************************************************************************
*/

/*
***************************************************************************************************
*                                           EXPORTED TYPE
***************************************************************************************************
*/

/*
***************************************************************************************************
*                                           EXPORTED VARIABLE
***************************************************************************************************
*/
extern OS_TCB      DigSigMonitorTaskTCB;
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
float   GetReformerTemp(void);
float   GetFireOrRodTemp(void);
void    SetHydrgProducerDigSigIgniteFirstTimeBehindMonitorHookSwitch(uint8_t i_NewStatu);
void    SetHydrgProducerDigSigAlarmRunningMonitorHookSwitch(uint8_t i_NewStatu);
void    SetStackExhaustTimesCountPerMinutesMonitorHookSwitch(uint8_t i_NewStatu);
void    DigSigMonitorTaskCreate(void);

#endif

