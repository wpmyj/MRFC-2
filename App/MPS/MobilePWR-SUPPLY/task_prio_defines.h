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



#define  HYDRG_PRODUCER_DELAY_STOP_TASK_PRIO        10
#define  STACK_MANAGER_DELAY_STOP_TASK_PRIO         11


#define  ANA_SIGNAL_MONITOR_TASK_PRIO                                   12

#define  STACK_MANAGER_TASK_PRIO                                         13
#define  STACK_HYDROGEN_YIELD_MATCHING_OFFSET_VALUE_MONITOR_TASK_PRIO    13
#define  DC_MODULE_ADJUST_TASK_PRIO                             14

#define  DIG_SIGNAL_MONITOR_TASK_PRIO                       15


#define  HYDROGEN_PRODUCER_MANAGER_TASK_PRIO                16
#define  APP_TASK_START_PRIO                                17
#define  MAKE_VACCUUM_TASK_PRIO                             18
#define  STACK_SHORT_CIRCUIT_TASK_PRIO                      19
#define  IGNITER_WORK_TASK_PRIO                             20

#define  SPEED_CONTROL_DEVICE_MANAGE_TASK_PRIO              21

#endif
