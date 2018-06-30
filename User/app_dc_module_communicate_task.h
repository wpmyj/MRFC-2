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
* Programmer(s) :  JasonFan
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
#define VOLTAGE_LIMIT_MAX  							(53.5)

#define CURRENT_LIMIT_MAX                            25.0 //30
#define HOLD_LIMIT_POINT                             10.0 //电堆启动阶段持续运行电流值，最佳为13A
#define CURRENT_LIMIT_MIN                             2.0
 
/*
***************************************************************************************************
*                                    EXPORTED OS VARIABLE DECLARATIONS
***************************************************************************************************
*/
extern      OS_TCB      DCLimitCurrentSmoothlyTaskTCB;
/*
***************************************************************************************************
*                                   EXPORTED GLOABLE VARIABLE DECLARATIONS
***************************************************************************************************
*/
extern      float   g_fIvalueNow;
extern      float   g_fVvalueMax;
extern		float   g_fIvalueMax;
/*
***************************************************************************************************
*                                           EXPORTED TYPE
***************************************************************************************************
*/

/*
***************************************************************************************************
*                                           EXPORTED FUNCTION
***************************************************************************************************
*/

void CurrentLimitTaskCreate(void);

void SetDCModuleLimitCurrentSmoothlyTaskSwitch(uint8_t i_NewStatu);

void CurrentSmoothlyLimitTaskCreate(void);


void HoldSlaveDCOutCurrentCmdSendThroughCAN(void);
void ResumeSlaveDCOutCurrentCmdSendThroughCAN(void);

float IncCurrentLimitPoint(void);
float DecCurrentLimitPoint(void);

void UpdateVIParaToPrgm(void);
/*
***************************************************************************************************
*                                             MODULE END
***************************************************************************************************
*/


#endif                                                          /* End of module include */
