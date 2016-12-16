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
* Filename      :  app_dc_module_communicate_task.h
* Programmer(s) :  Fanjun
* Version       :  V1.0
* data          :  2016.12.10
* brief         :  This file contains all the functions prototypes for the system run
*                  config parameters firmware library.
*********************************************************************************************************/
#ifndef __APP_DC_MODULE_COMMUNICATE_H__
#define __APP_DC_MODULE_COMMUNICATE_H__
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
#define VOLTAGE_LIMIT_MAX  (53.50)

#define CURRENT_LIMIT_MAX  (33.00)
#define CURRENT_LIMIT_MIN  (20.00)

/*
***************************************************************************************************
*                                    EXPORTED OS VARIABLE DECLARATIONS
***************************************************************************************************
*/
extern      OS_TCB      DcModuleAdjustTaskTCB;
/*
***************************************************************************************************
*                                   EXPORTED GLOABLE VARIABLE DECLARATIONS
***************************************************************************************************
*/

/*
***************************************************************************************************
*                                           EXPORTED FUNCTION
***************************************************************************************************
*/

void DcModuleAdjustTaskCreate(void);

void SetDcModuleCurrentLimitingPointImproveFlag(uint8_t i_NewStatu);
uint8_t GetDcModuleCurrentLimitingPointImproveFlagStatus(void);

void SetDcModuleCurrentLimitingPointReduceFlag(uint8_t i_NewStatu);
uint8_t GetDcModuleCurrentLimitingPointReduceFlagStatus(void);

/*
***************************************************************************************************
*                                             MODULE END
***************************************************************************************************
*/


#endif                                                          /* End of module include */
