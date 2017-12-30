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
  * @author  JasonFan
  * @version V1.0
  * @date    2017.1.1
  * @brief   This file contains all the functions prototypes for the analog sensor
  *          firmware library.
*********************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __APP_STACK_SHORT_CIRCUIT_TASK_H_
#define __APP_STACK_SHORT_CIRCUIT_TASK_H_
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
extern OS_TCB       StackRunningShortTaskTCB;

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
void StackRunningShortCtrlTaskCreate(void);

void SetStackShortCtrlTaskSwitch(uint8_t i_NewStatu);

uint8_t StackShortCtrl(void);


uint8_t GetDlyShortCtrlFlagStatus(void);
void SetDlyShortCtrlFlagStatus(uint8_t i_NewStatu);

void SetInShortControlFlagStatus(uint8_t i_NewStatu);
uint8_t GetInShortControlFlagStatus(void);
#endif
