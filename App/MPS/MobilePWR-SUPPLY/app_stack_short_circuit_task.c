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
* Filename      : app_stack_short_circuit_task.c
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
#include "app_stack_short_circuit_task.h"
#include "bsp_dc_module_adjust.h"
#include "app_dc_module_communicate_task.h"
/*
***************************************************************************************************
*                                           MACRO DEFINITIONS
***************************************************************************************************
*/
#define STACK_SHORT_CIRCUIT_TASK_STK_SIZE 100
/*
***************************************************************************************************
*                                  OS-RELATED    VARIABLES
***************************************************************************************************
*/
       OS_TCB     StackShortCircuitTaskTCB;
static CPU_STK    STACK_SHORT_CIRCUIT_TASK_STK[STACK_SHORT_CIRCUIT_TASK_STK_SIZE];
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
static void  StackShortCircuitTask(void *p_arg);    
    
/*
***************************************************************************************************
*                               StackShortCircuitTaskCreate()
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
void StackShortCircuitTaskCreate(void)
{
    OS_ERR  err;

    OSTaskCreate((OS_TCB *)&StackShortCircuitTaskTCB,
                 (CPU_CHAR *)"Stack Short Circuit Task Start",
                 (OS_TASK_PTR)StackShortCircuitTask,
                 (void *) 0,
                 (OS_PRIO) STACK_SHORT_CIRCUIT_TASK_PRIO,
                 (CPU_STK *)&STACK_SHORT_CIRCUIT_TASK_STK[0],
                 (CPU_STK_SIZE) STACK_SHORT_CIRCUIT_TASK_STK_SIZE / 10,
                 (CPU_STK_SIZE) STACK_SHORT_CIRCUIT_TASK_STK_SIZE,
                 (OS_MSG_QTY) 5u,
                 (OS_TICK) 0u,
                 (void *) 0,
                 (OS_OPT)(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR *)&err);
    APP_TRACE_INFO(("Created Stack Short Circuit Task, and err code is %d...\n\r", err));
}



/*
***************************************************************************************************
*                               StackShortCircuitTask()
*
* Description : The use of the the funciton is to create the task that monitor the analog signal.
*
* Arguments   : none.
*
* Returns     : none.
*
* Notes       : 一小时一次.
***************************************************************************************************
*/
static void  StackShortCircuitTask(void *p_arg)                   
{ 
    OS_ERR  err; 
    
    while(DEF_TRUE)
    {
        OSTaskSuspend(NULL, &err);
        OSTaskSuspend(&DCLimitCurrentSmoothlyTaskTCB, &err);//挂起平滑限流任务
        OSTaskSuspend(&DCModuleAutoAdjustTaskTCB, &err);//挂起动态限流任务
        
        APP_TRACE_INFO(("Stack short circuit task resume...\n\r"));   
             
        while(DEF_TRUE)
        {
            OSTaskSemPend(OS_CFG_TICK_RATE_HZ * 60 * 60, 
                          OS_OPT_PEND_BLOCKING, 
                          NULL,
                          &err);
			if(err == OS_ERR_TIMEOUT){
                
                APP_TRACE_INFO(("Start tack short circuit...\n\r"));   
				Bsp_SendAddressByDifferentCmdType(TRANSPOND_COMMAND);
                Bsp_SetDcModuleOutPutVIvalue((float)VOLTAGE_LIMIT_MAX, 0);//限流点降为0
                
                BSP_StackShortCircuitActivationOn();
				BSP_DCConnectValvePwrOff(); 
				OSTimeDlyHMSM(0, 0, 0,200,OS_OPT_TIME_HMSM_STRICT,&err);
                BSP_StackShortCircuitActivationOff();
                APP_TRACE_INFO(("Stop tack short circuit...\n\r"));
				BSP_DCConnectValvePwrOn();
                OSTaskResume(&DCLimitCurrentSmoothlyTaskTCB,&err);//恢复平滑限流任务
                
			}else{}
          
        }
    }
}





