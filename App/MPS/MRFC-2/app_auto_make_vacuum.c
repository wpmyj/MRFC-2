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
#define MAKE_VACCUUM_TASK_STK_SIZE 			100
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
	uint8_t InVacuumFlag = DEF_NO;
	uint8_t SysWorkStatus = 0;
	uint8_t WaitSec = 0;

    while(DEF_TRUE) {
		
        OSTaskSuspend(NULL, &err);
        APP_TRACE_INFO(("Start make vacuum task...\n\r"));

        while(DEF_TRUE) {

			OSTimeDlyHMSM(0, 0, 1, 000,OS_OPT_TIME_HMSM_STRICT,&err);
			
			if(InVacuumFlag != DEF_YES){
				WaitSec ++;
				if(WaitSec > 60){
					BSP_PureHydrogenGasOutValvePwrOff();//�رմ�����ڷ�
					BSP_TailGasOutValvePwrOn();//�����С���ϵ�
					InVacuumFlag = DEF_YES;
					WaitSec = 0;
				}
			}else{
				if(EN_RUNNING == GetSystemWorkStatu()){
					BSP_PureHydrogenGasOutValvePwrOn();
					BSP_TailGasOutValvePwrOff();
					break;
				}
			}
        }
    }
}





