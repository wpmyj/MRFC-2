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
#include "bsp_speed_adjust_device.h"
#include "app_stack_manager.h"
#include "bsp_pid.h"
/*
***************************************************************************************************
*                                           MACRO DEFINITIONS
***************************************************************************************************
*/
#define STACK_RUNNING_SHORT_TASK_STK_SIZE             100

/*
***************************************************************************************************
*                                  OS-RELATED    VARIABLES
***************************************************************************************************
*/
OS_TCB     StackRunningShortTaskTCB;
#ifdef  STACK_SHORT_CONTROL_ENABLE
		
static 	CPU_STK    STACK_SHORT_CIRCUIT_TASK_STK[STACK_RUNNING_SHORT_TASK_STK_SIZE];

/*
***************************************************************************************************
*                                           GLOBAL VARIABLES
***************************************************************************************************
*/
static   uint8_t     g_ShortCtrlTaskSwitch = DEF_ENABLED;


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

static void  StackRunningShortTask(void *p_arg);

/*
***************************************************************************************************
*                               StackShortCtrlTaskCreate()
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
void StackShortCtrlTaskCreate(void)
{
    OS_ERR  err;

    OSTaskCreate((OS_TCB *)&StackRunningShortTaskTCB,
                 (CPU_CHAR *)"Stack Short Circuit Task Start",
                 (OS_TASK_PTR)StackRunningShortTask,
                 (void *) 0,
                 (OS_PRIO) STACK_SHORT_CIRCUIT_TASK_PRIO,
                 (CPU_STK *)&STACK_SHORT_CIRCUIT_TASK_STK[0],
                 (CPU_STK_SIZE) STACK_RUNNING_SHORT_TASK_STK_SIZE / 10,
                 (CPU_STK_SIZE) STACK_RUNNING_SHORT_TASK_STK_SIZE,
                 (OS_MSG_QTY) 5u,
                 (OS_TICK) 0u,
                 (void *) 0,
                 (OS_OPT)(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR *)&err);
    APP_TRACE_INFO(("Created Stack Short Circuit Task, and err code is %d...\n\r", err));
}

/*
***************************************************************************************************
*                               StackShortCtrl()
*
* Description : Stack start up short control.
*
* Arguments   : none.
*
* Returns     : none.
*
* Notes       : none.
***************************************************************************************************
*/
uint8_t StackShortCtrl(void)
{
    OS_ERR  err;
    static  uint8_t fail_cnt = 0;
    static  uint8_t status_flag =  DEF_TRUE;

    BSP_DCConnectValvePwrOff();

    OSTimeDlyHMSM(0, 0, 0, 600, OS_OPT_TIME_HMSM_STRICT, &err);

    if((GetSrcAnaSig(STACK_VOLTAGE) >= 35) && (GetSrcAnaSig(HYDROGEN_PRESS_1) >= 20)) {
//        APP_TRACE_INFO(("Start stack short control...\n\r"));
        BSP_StackShortCircuitActivationOn();
        OSTimeDlyHMSM(0, 0, 0, 200, OS_OPT_TIME_HMSM_STRICT, &err);
        BSP_StackShortCircuitActivationOff();
        BSP_HydrgOutValvePwrOn();//��·������
        OSTimeDlyHMSM(0, 0, 0, 600, OS_OPT_TIME_HMSM_STRICT, &err);
        BSP_HydrgOutValvePwrOff();
        fail_cnt = 0;
        status_flag = DEF_TRUE;
    } else {
        APP_TRACE_INFO(("Not meet the short conditions...\r\n"));
        fail_cnt ++;

        if(fail_cnt >= 2) {
            status_flag = DEF_FALSE;
        }
    }

    BSP_OS_TimeDlyMs(100);
    BSP_DCConnectValvePwrOn();

    return status_flag;
}
/*
***************************************************************************************************
*                               StackRunningShortTask()
*
* Description : The use of the the funciton is to create the task that monitor the analog signal.
*
* Arguments   : none.
*
* Returns     : none.
*
* Notes       : 10����һ��
***************************************************************************************************
*/

static void  StackRunningShortTask(void *p_arg)
{
    OS_ERR  err;

	while(DEF_TRUE) {
		
		while(DEF_TRUE) {
			
			APP_TRACE_INFO(("Stack running short task suspend...\n\r"));
			OSTaskSuspend(NULL, &err);
			APP_TRACE_INFO(("Stack running short task resume...\n\r"));

			while(DEF_TRUE) {

				OSTaskSemPend(OS_CFG_TICK_RATE_HZ * 60 * 10,
							  OS_OPT_PEND_BLOCKING,
							  NULL,
							  &err);
				
				if(err == OS_ERR_TIMEOUT) {
					if(DEF_SET != GetStackNeedRestartLimitCurrentFlag()){//��ͣ�����������׶β����ж�·�
						SetStackFanSpdPidControlSwitch(DEF_DISABLED);//��·�ʱ�ر�PID����
						IPID.Err = 0;
						IPID.Err_Next = 0;
						SetStackFanCtrlSpd(300);

						//��·�ǰ�׶�����
						SetDcModeOutPutNominalVoltageButDifferentCurrent(LIMIT_POINT_1_IN_SHORT_PRGM);
						BSP_OS_TimeDlyMs(5000);
						SetDcModeOutPutNominalVoltageButDifferentCurrent(LIMIT_POINT_2_IN_SHORT_PRGM);
						BSP_OS_TimeDlyMs(5000);
						SetDcModeOutPutNominalVoltageButDifferentCurrent(HOLD_LIMIT_POINT);
						BSP_OS_TimeDlyMs(5000);

						BSP_DcOutPutValvePwrOn();//����ʱ������
						OSTimeDlyHMSM(0, 0, 1, 0, OS_OPT_TIME_HMSM_STRICT, &err);

						if(DEF_TRUE == StackShortCtrl()) {
							APP_TRACE_INFO(("Stack running short first success...\n\r"));
						} else {
							SetStackShortCtrlTaskSwitch(DEF_DISABLED);
						}

						//��һ�ζ�·��ж�·���£����5��
						OSTimeDlyHMSM(0, 0, 5, 0, OS_OPT_TIME_HMSM_STRICT, &err);

						if(DEF_TRUE == StackShortCtrl()) {
							APP_TRACE_INFO(("Stack running short second success...\n\r"));
						} else {
							SetStackShortCtrlTaskSwitch(DEF_DISABLED);
						}

						OSTimeDlyHMSM(0, 0, 0, 200, OS_OPT_TIME_HMSM_STRICT, &err);
						BSP_DcOutPutValvePwrOff();

						//����ʽ����������
						BSP_OS_TimeDlyMs(5000);
						SetStackFanCtrlSpd(800);
						BSP_OS_TimeDlyMs(5000);
						SetStackFanCtrlSpd(300);

						SetDcModeOutPutNominalVoltageButDifferentCurrent(LIMIT_POINT_2_IN_SHORT_PRGM);
						BSP_OS_TimeDlyMs(10000);
						SetDcModeOutPutNominalVoltageButDifferentCurrent(LIMIT_POINT_1_IN_SHORT_PRGM);
						BSP_OS_TimeDlyMs(10000);
						SetDcModeOutPutNominalVoltageButDifferentCurrent(CURRENT_LIMIT_MAX);
						SetStackFanSpdPidControlSwitch(DEF_ENABLED);//��·���ɴ�PID����
					}

				}else if(err == OS_ERR_NONE){
					if((g_ShortCtrlTaskSwitch == DEF_DISABLED) || (EN_SHUTTING_DOWN == GetSystemWorkStatu())) {
						APP_TRACE_INFO(("Stack running short task break...\n\r"));
						break;
					}
				}else{}
			}
		}
	}
}


/*
***************************************************************************************************
*                      SetStackShortCtrlTaskSwitch()
*
* Description:  Enable or Disable the stack short control task.
*
* Arguments  :  none
*
* Returns    :  none
***************************************************************************************************
*/
void SetStackShortCtrlTaskSwitch(uint8_t i_NewStatu)
{
    g_ShortCtrlTaskSwitch = i_NewStatu;
}

uint8_t GetStackStackShortCtrlTaskSwitchStatus()
{
    return g_ShortCtrlTaskSwitch ;
}
#endif
