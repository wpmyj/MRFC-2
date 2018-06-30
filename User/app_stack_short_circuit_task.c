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
* Version       : V2.10
* Programmer(s) : JasonFan
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
#include "app_analog_signal_monitor_task.h"
#include "app_stack_manager.h"
#include "bsp_pid.h"
/*
***************************************************************************************************
*                                           MACRO DEFINITIONS
***************************************************************************************************
*/
//��ͬ��·�ѡ�񿪹�
#define  STACK_MOSFET_SHORT_CTRL_EN		0
#define  STACK_IGBT_SHORT_CTRL_EN		1


#define STACK_RUNNING_SHORT_TASK_STK_SIZE             128

/*
***************************************************************************************************
*                                  OS-RELATED    VARIABLES
***************************************************************************************************
*/
OS_TCB     StackRunningShortTaskTCB;
		
static 	CPU_STK    STACK_SHORT_CIRCUIT_TASK_STK[STACK_RUNNING_SHORT_TASK_STK_SIZE];

/*
***************************************************************************************************
*                                           GLOBAL VARIABLES
***************************************************************************************************
*/
static   uint8_t     g_ShortCtrlTaskSwitch = DEF_ENABLED;
static   uint8_t	 g_InShortCtrlFlag = DEF_CLR;
static   uint8_t	 g_ShortCtrlDlyFlag = DEF_CLR;
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
*                               StackRunningShortCtrlTaskCreate()
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
void StackRunningShortCtrlTaskCreate(void)
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

    BSP_DcOutPutValvePwrOff();
	BSP_OS_TimeDlyMs(100);	
    BSP_DCConnectValvePwrOff();
	BSP_OS_TimeDlyMs(100);
    BSP_ShortProtectValvePwrOn();

    OSTimeDlyHMSM(0, 0, 0, 600, OS_OPT_TIME_HMSM_STRICT, &err);

    if((GetSrcAnaSig(STACK_VOLTAGE) >= 35) && (GetSrcAnaSig(HYDROGEN_PRESS_1) >= 20)) {
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
			fail_cnt = 0;
        }
    }
		
    BSP_ShortProtectValvePwrOff();
    BSP_OS_TimeDlyMs(100);
    BSP_DCConnectValvePwrOn();   
    BSP_OS_TimeDlyMs(100);
    BSP_DcOutPutValvePwrOn();

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
* Notes       : 45����һ��
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
				
				if((g_ShortCtrlTaskSwitch == DEF_DISABLED) || (EN_SHUTTING_DOWN == GetSystemWorkStatu())) {
					APP_TRACE_INFO(("Stack running short task break...\n\r"));
					ResumeSlaveDCOutCurrentCmdSendThroughCAN();//�ָ���һ̨����������
					break;
				}

				OSTaskSemPend(OS_CFG_TICK_RATE_HZ * 60 * 45,//45
							  OS_OPT_PEND_BLOCKING,
							  NULL,
							  &err);
				
				if(err == OS_ERR_TIMEOUT) {
					
					if(DEF_SET == GetDlyShortCtrlFlagStatus()){
						APP_TRACE_INFO(("Stack running short delay begin...\n\r"));
						OSTimeDlyHMSM(0, 0, 30, 0, OS_OPT_TIME_HMSM_STRICT, &err);//�������һ̨���������ڶ�·������1���Ӻ��ٽ��ж�·�
						APP_TRACE_INFO(("Stack running short delay finished...\n\r"));
					}

					if(DEF_SET != GetRestartLimitCurrentFlagStatus()){//��ͣ�����������׶β����ж�·�
						
						SetInShortControlFlagStatus(DEF_SET);
						HoldSlaveDCOutCurrentCmdSendThroughCAN();//������Ϣ��ϵͳ������һ̨������������
						SetStackFanSpdPidControlSwitch(DEF_DISABLED);//��·�ʱ�ر�PID����
						ResetPidErr(&IPID);//����PID������
						SetStackFanCtrlSpd(800);

						//��·�ǰ�׶�����
						while(DecCurrentLimitPoint() != CURRENT_LIMIT_MIN){
							SetDcOutPutCurrentLimitPoint(g_fIvalueNow);	
							BSP_OS_TimeDlyMs(5000);
						}
		
						if(DEF_TRUE == StackShortCtrl()) {
							APP_TRACE_INFO(("Stack running short first success...\n\r"));
						} else {
							SetStackShortCtrlTaskSwitch(DEF_DISABLED);
							APP_TRACE_INFO(("Stack running short first failed...\n\r"));
						}
						
						//��һ�ζ�·��ж�·���£����5��
						BSP_OS_TimeDlyMs(5000);

						if(DEF_TRUE == StackShortCtrl()) {
							APP_TRACE_INFO(("Stack running short second success...\n\r"));
						} else {
							SetStackShortCtrlTaskSwitch(DEF_DISABLED);
							APP_TRACE_INFO(("Stack running short second failed...\n\r"));
						}

						//����ʽ����������
						SetStackFanCtrlSpd(300);
						while(IncCurrentLimitPoint() != g_fIvalueMax){
							SetDcOutPutCurrentLimitPoint(g_fIvalueNow);
							BSP_OS_TimeDlyMs(5000);								
						}						
						SetStackFanSpdPidControlSwitch(DEF_ENABLED);//��·���ɴ�PID����
						ResumeSlaveDCOutCurrentCmdSendThroughCAN();//�ָ���һ̨����������
					}
				}
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

uint8_t GetStackStackShortCtrlTaskSwitchStatus(void)
{
    return g_ShortCtrlTaskSwitch ;
}

void SetDlyShortCtrlFlagStatus(uint8_t i_NewStatu)
{
	g_ShortCtrlDlyFlag = i_NewStatu;
	APP_TRACE_INFO(("Dly Short Ctrl Flag is %d...\n\r",g_ShortCtrlDlyFlag));
}


uint8_t GetDlyShortCtrlFlagStatus(void)
{
    return g_ShortCtrlDlyFlag ;
}

void SetInShortControlFlagStatus(uint8_t i_NewStatu)
{
	g_InShortCtrlFlag = i_NewStatu;
}

uint8_t GetInShortControlFlagStatus(void)
{
	return g_InShortCtrlFlag;
}
/******************* (C) COPYRIGHT 2016 Guangdong ENECO POWER *****END OF FILE****/

