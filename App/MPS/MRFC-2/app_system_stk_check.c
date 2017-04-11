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
* Filename      : app_system_stk_check.c
* Version       : V1.00
* Programmer(s) : Fanjun
***************************************************************************************************
*/

/*
***************************************************************************************************
*                                             INCLUDE FILES
***************************************************************************************************
*/
#include "app_system_stk_check.h"
#include "app_stack_manager.h"
#include "app_wireness_communicate_task.h"
#include "app_mf210_communicate_task.h"
#include "app_system_real_time_parameters.h"
#include "app_top_task.h"
#include "app_hydrg_producer_manager.h"
#include "app_analog_signal_monitor_task.h"
#include "app_digital_signal_monitor_task.h"
#include "app_auto_make_vacuum.h"
#include "app_dc_module_communicate_task.h"
#include "app_stack_short_circuit_task.h"
#include "app_mf210_communicate_task.h"
#include "app_digital_signal_monitor_task.h"
/*
***************************************************************************************************
*                                       MACRO DEFINITIONS
***************************************************************************************************
*/

#define  SYSTEM_STACK_USAGE_CHECK_TASK_STK_SIZE       128 // ����Ķ�ջ��С

/*
***************************************************************************************************
*                                         OS-RELATED    VARIABLES
***************************************************************************************************
*/
OS_TCB  SystemStackUsageCheckTaskTCB;        // ����ͳ�������TCB  
CPU_STK SystemStackUsageCheckTaskSTK [SYSTEM_STACK_USAGE_CHECK_TASK_STK_SIZE];// ����������Ϊ����ջ������ʹ��  


/*
***************************************************************************************************
*                                         FUNCTION PROTOTYPES
***************************************************************************************************
*/
static void  SystemStackUsageCheckTask (void *p_arg);


/*
***************************************************************************************************
*                                   SystemStackUsageCheckTaskCreate()
*
* Description:  This function creates the System stack usage check task.
*
* Arguments  :  none
*
* Returns    :  none
***************************************************************************************************
*/
void  SystemStackUsageCheckTaskCreate(void)  
{  
    OS_ERR err; 
    OSTaskCreate( (OS_TCB     *)&SystemStackUsageCheckTaskTCB,  
                (CPU_CHAR   *)"System system stack usage check task",  
                (OS_TASK_PTR ) SystemStackUsageCheckTask,  
                (void       *) 0,  
                (OS_PRIO     ) SYSTEM_STACK_USAGE_CHECK_TASK_PRIO,  
                (CPU_STK    *)&SystemStackUsageCheckTaskSTK[0],  
                (CPU_STK_SIZE) SYSTEM_STACK_USAGE_CHECK_TASK_STK_SIZE/10,/*ջ����ٽ�ֵ������ջ��С��90%��*/  
                (CPU_STK_SIZE) SYSTEM_STACK_USAGE_CHECK_TASK_STK_SIZE,  
                (OS_MSG_QTY  ) 0,  
                (OS_TICK     ) 0,  
                (void       *) 0,  
                (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),   
                (OS_ERR     *) &err); 
    APP_TRACE_INFO(("Created system stack usage check task, and err code is %d...\r\n", err));
                    
}  


static void  SystemStackUsageCheckTask (void *p_arg)  
{  
    OS_ERR err;  
    CPU_STK_SIZE free,used;  
    (void)p_arg;  
    
		while(DEF_TRUE)  
		{          
				OSTaskStkChk (&SystemStackUsageCheckTaskTCB,&free,&used,&err);//ͳ��������Ķ�ջʹ����                              
				APP_TRACE_INFO(("SystemStackUsageCheck------>used/free:%d/%d  usage:%d%%\r\n",used,free,(used*100)/(used+free)));  

				OSTaskStkChk (&HydrgProducerManagerTaskTCB,&free,&used,&err);  
				APP_TRACE_INFO(("HydrgProducerManagerTask--->used/free:%d/%d  usage:%d%%\r\n",used,free,(used*100)/(used+free)));  
				
				OSTaskStkChk (&CommunicateDataSendTaskTCB,&free,&used,&err);  
				APP_TRACE_INFO(("CommunicateTask------------>used/free:%d/%d  usage:%d%%\r\n",used,free,(used*100)/(used+free)));  
				
				OSTaskStkChk (&CommunicateDataSendTaskTCB,&free,&used,&err);  
				APP_TRACE_INFO(("CommunicateDataSendTask---->used/free:%d/%d  usage:%d%%\r\n",used,free,(used*100)/(used+free)));  

				OSTaskStkChk (&MF210_CommunicateTaskTCB,&free,&used,&err);  
				APP_TRACE_INFO(("MF210_CommunicateTask------>used/free:%d/%d  usage:%d%%\r\n",used,free,(used*100)/(used+free)));  

				OSTaskStkChk (&Make_Vaccuum_FunctionTaskTCB,&free,&used,&err);  
				APP_TRACE_INFO(("Make_Vaccuum_FunctionTask-->used/free:%d/%d  usage:%d%%\r\n",used,free,(used*100)/(used+free)));  

				OSTaskStkChk (&CommunicateDataSendTaskTCB,&free,&used,&err);  
				APP_TRACE_INFO(("StackManagerTask----------->used/free:%d/%d  usage:%d%%\r\n",used,free,(used*100)/(used+free)));  
				
				OSTaskStkChk (&DCLimitCurrentSmoothlyTaskTCB,&free,&used,&err);  
				APP_TRACE_INFO(("DCLimitCurrentSmoothlyTask-->used/free:%d/%d  usage:%d%%\r\n",used,free,(used*100)/(used+free)));  
				
				OSTaskStkChk (&DCModuleDynamicAdjustTaskTCB,&free,&used,&err);  
				APP_TRACE_INFO(("DCModuleDynamicAdjustTask--->used/free:%d/%d  usage:%d%%\r\n",used,free,(used*100)/(used+free)));  
				
				OSTaskStkChk (&DigSigMonitorTaskTCB,&free,&used,&err);  
				APP_TRACE_INFO(("DigSigMonitorTaskTCB-------->used/free:%d/%d  usage:%d%%\r\n",used,free,(used*100)/(used+free)));  
								
				OSTaskStkChk (&StackShortCircuitTaskTCB,&free,&used,&err);  
				APP_TRACE_INFO(("StackShortCircuitTaskTCB---->used/free:%d/%d  usage:%d%%\r\n",used,free,(used*100)/(used+free)));
				
				APP_TRACE_INFO(("\r\n\r\n"));  
				
				OSTimeDlyHMSM(0,0,4,0,(OS_OPT)OS_OPT_TIME_DLY,(OS_ERR*)&err);
				
				if(EN_SHUTTING_DOWN == GetSystemWorkStatu())
				{   
						OSTaskSuspend(&SystemStackUsageCheckTaskTCB,             
													&err);
				}
		} 
}
/******************* (C) COPYRIGHT 2016 Guangdong ENECO *****END OF FILE****/
