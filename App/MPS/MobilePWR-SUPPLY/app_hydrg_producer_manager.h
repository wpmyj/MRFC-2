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
* Filename      :  app_hydrg_producer_manager.h
* Programmer(s) :  Fanjun
* Version       :  V1.0
* data          :  2016.12.12
* brief         :  This file contains all the functions prototypes for the system run
*                  config parameters firmware library.
*********************************************************************************************************/
#ifndef __APP_HYDRG_PRODUCER_MANAGER_H__
#define __APP_HYDRG_PRODUCER_MANAGER_H__


/*
***************************************************************************************************
*                                           INCLUDE FILES
***************************************************************************************************
*/
#include "app_top_task.h"

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
*                                           EXPORTED DATA TYPE
***************************************************************************************************
*/
extern      OS_TCB      HydrgProducerManagerTaskTCB;
extern      OS_TCB      HydrgProducerManagerDlyStopTaskTCB;

/*
***************************************************************************************************
*                                           EXPORTED FUNCTION
***************************************************************************************************
*/
IGNITE_CHECK_STATU_Typedef      IgniteFirstTime(float, float, uint8_t , uint8_t);
IGNITE_CHECK_STATU_Typedef      IgniteSecondTime(float, float, uint8_t , uint8_t);
WHETHER_TYPE_VARIABLE_Typedef   GetAheadRunningFlag(void);

void                 SetAheadRunningFlag(WHETHER_TYPE_VARIABLE_Typedef m_NES_STATU);
void                 IgniterWorkForSeconds(uint16_t);
void                 HydrgProducerManagerTaskCreate(void);
void                 HydrgProducerDlyStopTaskCreate(void);
void                 IgniterWorkTaskCreate(void);
uint8_t              GetHydrgProducerStopDlyStatu(void);

uint16_t GetHydrgProducerIgniteSecondTimeReformerTemp(void);
/*
***************************************************************************************************
*                                             MODULE END
***************************************************************************************************
*/


#endif                                                          /* End of module include */

