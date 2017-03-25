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
/*
***************************************************************************************************
* Filename      : app_auto_make_vacuum.c
* Version       : V1.00
* Programmer(s) : Fanjun
*
***************************************************************************************************
*/
/*
***************************************************************************************************
*                                             INCLUDE FILES
***************************************************************************************************
*/
#include "app_auto_make_vacuum.h"
#include <includes.h>
/*
***************************************************************************************************
*                                           MACRO DEFINITIONS
***************************************************************************************************
*/
#define MAKE_VACCUUM_TASK_STK_SIZE 100
/*
***************************************************************************************************
*                                  OS-RELATED    VARIABLES
***************************************************************************************************
*/
OS_TCB     Make_Vaccuum_FunctionTaskTCB;
static CPU_STK    MAKE_VACCUUM_TASK_STK[MAKE_VACCUUM_TASK_STK_SIZE];
/*
***************************************************************************************************
*                                           GLOBAL VARIABLES
***************************************************************************************************
*/

/*
***************************************************************************************************
*                                           LOCAL VARIABLES
***************************************************************************************************
*/

/*
***************************************************************************************************
*                                         FUNCTION PROTOTYPES
***************************************************************************************************
*/
static void  Make_Vacuum_FunctionTask(void *p_arg);

/*
***************************************************************************************************
*                               Make_Vacuum_FunctionTaskCreate()
*
* Description : The use of the the funciton is to create the task that monitor the analog signal.
*
* Arguments   : none.
*
* Returns     : none.
*
* Notes       : none.
***************************************************************************************************
*/
void Make_Vacuum_FunctionTaskCreate(void)
{
    OS_ERR  err;

    OSTaskCreate((OS_TCB *)&Make_Vaccuum_FunctionTaskTCB,
                 (CPU_CHAR *)"Make Vaccuum Function Task Start",
                 (OS_TASK_PTR)Make_Vacuum_FunctionTask,
                 (void *) 0,
                 (OS_PRIO) MAKE_VACCUUM_TASK_PRIO,
                 (CPU_STK *)&MAKE_VACCUUM_TASK_STK[0],
                 (CPU_STK_SIZE) MAKE_VACCUUM_TASK_STK_SIZE / 10,
                 (CPU_STK_SIZE) MAKE_VACCUUM_TASK_STK_SIZE,
                 (OS_MSG_QTY) 5u,
                 (OS_TICK) 0u,
                 (void *) 0,
                 (OS_OPT)(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR *)&err);
    APP_TRACE_INFO(("Created Make Vaccuum  Task, and err code is %d...\n\r", err));
}



/*
***************************************************************************************************
*                               Make_Vacuum_FunctionTask()
*
* Description : The use of the the funciton is to create the task that monitor the analog signal.
*
* Arguments   : none.
*
* Returns     : none.
*
* Notes       : none.
***************************************************************************************************
*/
static void  Make_Vacuum_FunctionTask(void *p_arg)
{
    OS_ERR  err;
    static uint8_t u8timeCount = 0;
    static uint8_t u8MakeVavuumValve4PwrOnFlag = 0;

    while(DEF_TRUE) {
        OSTaskSuspend(NULL, &err);
        APP_TRACE_INFO(("Start make vacuum task...\n\r"));

        BSP_PureHydrogenGasOutValvePwrOff();
        BSP_TailGasOutValvePwrOn();//关闭常开尾气电磁阀
        BSP_MakeVavuumValve2PwrOn();
        BSP_MakeVavuumValve4PwrOn();
        u8MakeVavuumValve4PwrOnFlag = YES;

        while(DEF_TRUE) {

            OSTaskSemPend(OS_CFG_TICK_RATE_HZ,
                          OS_OPT_PEND_BLOCKING,
                          NULL,
                          &err);

            if(err == OS_ERR_TIMEOUT) {
                if(NO != u8MakeVavuumValve4PwrOnFlag) {
                    u8timeCount++;

                    if(u8timeCount >= 60) {
                        BSP_MakeVavuumValve4PwrOff();
                        BSP_MakeVavuumValve3PwrOn();
                        BSP_MakeVavuumPumpPwrOn();
                        u8MakeVavuumValve4PwrOnFlag = NO;
                        u8timeCount = 0;
                    }
                }

                //需要考虑如果在启动过程中，关机后采取什么操作
            } else if(err == OS_ERR_NONE) { //切换后（重整温度达到后）全部关闭
                APP_TRACE_INFO(("Stop make vacuum task...\n\r"));
                BSP_PureHydrogenGasOutValvePwrOn();//纯氢出口
                BSP_TailGasOutValvePwrOff();
                BSP_MakeVavuumValve2PwrOff();
                BSP_MakeVavuumValve3PwrOff();
                BSP_MakeVavuumValve4PwrOff();
                BSP_MakeVavuumPumpPwrOff();
                break;
            }
        }
    }
}





