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
  * @file    bsp_delay_task_timer.h
  * @author  Fanjun
  * @version V1.0
  * @date    7-March-2017
  * @brief   This file contains all the functions prototypes for the analog sensor
  *          firmware library.
*********************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BSP_DELAY_TASK_TIMER_H_
#define __BSP_DELAY_TASK_TIMER_H_
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

/*
***************************************************************************************************
*                                           EXPORTED TYPE
***************************************************************************************************
*/
typedef enum {

    CAN_BUS_AUTO_RECONNECT_AFTER_30_SEC = 0,
    START_UP_SWITCH_CHECK_DELAY_1S,
    SHUT_DOWN_SWITCH_CHECK_DELAY_3S,
    TASK_MAX_NUM = 19,

} TIM6_DELAY_TASK_TYPE_Typedef;

typedef struct {
    TIM6_DELAY_TASK_TYPE_Typedef DelayTask;
    uint16_t DelayTime;
} TIM6_DELAY_TASK_TYPE_AND_TIME_PARA_Typedef;

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
void Delay_Queue_Timer6_Init(void);
void AddNewDelayTaskToTimer6Queue(TIM6_DELAY_TASK_TYPE_Typedef i_eTaskType, u16 ARR);
void StartTim6DelayTask(TIM6_DELAY_TASK_TYPE_Typedef i_eTaskType, u16 i_u16TaskDelayMs);

#endif
