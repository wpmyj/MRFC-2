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
  * @file    task_prio_defines.h
  * @author  Fanjun
  * @version V1.0
  * @date    3-March-2016
  * @brief   This file contains all the macro define about task priority.
  *
*********************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TASK_PRIO_DEFINES_H__
#define __TASK_PRIO_DEFINES_H__

/*
***************************************************************************************************
*                                           EXPORTED MACRO
***************************************************************************************************
*/
//前面的1-3优先级分配给系统时钟任务、定时器和统计任务
#define  SYSTEM_TIME_STATISTIC_TASK_PRIO            4
#define  COMMUNICATE_TASK_PRIO                      5
#define  COMMUNICATE_DATA_SEND_TASK_PRIO            6
#define  COMMUNICATE_REQUEST_SEND_TASK_PRIO         7

#define  HYDRG_PRODUCER_DELAY_STOP_TASK_PRIO        8
#define  STACK_MANAGER_DELAY_STOP_TASK_PRIO         9

#define  ANA_SIGNAL_MONITOR_TASK_PRIO                                   10

#define  STACK_MANAGER_TASK_PRIO                                         11
#define  STACK_PROGRAM_CONTROL_AIR_PRESSURE_RELEASE_TASK_PRIO            11 
#define  STACK_HYDROGEN_YIELD_MATCHING_OFFSET_VALUE_MONITOR_TASK_PRIO    11
#define  DC_MODULE_ADJUST_TASK_PRIO                             12

#define  DIG_SIGNAL_MONITOR_TASK_PRIO                       14
#define  Make_Vaccuum_Task_PRIO                             15 
#define  APP_TASK_START_PRIO                                16
#define  HYDROGEN_PRODUCER_MANAGER_TASK_PRIO                17
#define  HYDRG_PRODUCER_IGNITE_SECOND_TIME_MANAGE_TASK_PRIO 18

#define  IGNITER_WORK_TASK_PRIO                             20
  


#endif
