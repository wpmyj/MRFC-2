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
  * @file    app_top_task.h
  * @author  Fanjun
  * @version V1.0
  * @date    6-March-2016
  * @brief   This file contains all the functions prototypes for the analog sensor
  *          firmware library.
*********************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __APP_TOP_TASK_H
#define __APP_TOP_TASK_H
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
extern      OS_SEM      WaitSelcetWorkModeSem;
extern      OS_SEM      IgniteFirstBehindWaitSem;
extern      OS_SEM      IgniteSecondBehindWaitSem;
/*
***************************************************************************************************
*                                           EXPORTED TYPE
***************************************************************************************************
*/
typedef enum {
    EN_THROUGH,
    EN_NOT_THROUGH = !EN_THROUGH,
} VERIFY_RESULT_TYPE_VARIABLE_Typedef;

typedef enum {
    EN_PASS,
    EN_NOT_PASS = !EN_PASS,
} IGNITE_CHECK_STATU_Typedef;


typedef enum {
    EN_STOP_ALL_DIRECT,
    EN_DELAY_STOP_PART_ONE,
    EN_DELAY_STOP_PART_TWO,
    EN_DELAY_STOP_BOTH_PARTS,
} SYSTEM_SHUT_DOWN_ACTION_FLAG_Typedef;
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



VERIFY_RESULT_TYPE_VARIABLE_Typedef     CheckAuthorization(void);
VERIFY_RESULT_TYPE_VARIABLE_Typedef     WaittingCommand(void);
void                                    Starting(void);
void                                    Running(void);
void                                    KeepingWarm(void);
void                                    DeviceFaultAlarm(void);
void                                    ResetDeviceAlarmStatu(void);

void     SetShutDownActionFlag(SYSTEM_SHUT_DOWN_ACTION_FLAG_Typedef);
void     UpdateBuzzerStatuInCruise(void);



#endif
