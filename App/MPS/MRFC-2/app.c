/*
***************************************************************************************************
*                                              APPLICATION CODE
*
*                          (c) Copyright 2016; Guangdong ENECO Science And Technology Co.,Ltd
*
*               All rights reserved.  Protected by international copyright laws.
*               Knowledge of the source code may NOT be used without authorization.
***************************************************************************************************
*/
/*
***************************************************************************************************
* Filename      : app.c
* Version       : V1.00
* Programmer(s) : JasonFan
***************************************************************************************************
*/
/*
***************************************************************************************************
*                                             INCLUDE FILES
***************************************************************************************************/
#include <includes.h>
#include <os_app_hooks.h>
#include "app_system_real_time_parameters.h"
#include "app_system_run_cfg_parameters.h"
#include "app_top_task.h"
#include "app_hydrg_producer_manager.h"
#include "app_stack_manager.h"
#include "app_wireness_communicate_task.h"
#include "app_analog_signal_monitor_task.h"
#include "app_digital_signal_monitor_task.h"
#include "app_auto_make_vacuum.h"
#include "app_dc_module_communicate_task.h"
#include "bsp_dc_module_adjust.h"
#include "bsp_speed_adjust_device.h"
#include "app_system_stk_check.h"
#include "app_stack_short_circuit_task.h"
#include "app_mf210_communicate_task.h"
#include "bsp_can.h"

/*
***************************************************************************************************
*                                           MACRO DEFINITIONS
***************************************************************************************************
*/
#define  APP_TASK_START_STK_SIZE                    256

/*
***************************************************************************************************
*                                         OS-RELATED    VARIABLES
***************************************************************************************************
*/
OS_TCB      AppTaskStartTCB;

static      CPU_STK_8BYTE_ALIGNED     AppTaskStartStk[APP_TASK_START_STK_SIZE];

/*
***************************************************************************************************
*                                         FUNCTION PROTOTYPES
***************************************************************************************************
*/

static      void        AppTaskStart(void *p_arg);
static      void        USER_NVIC_Cfg(void);

/*
***************************************************************************************************
*                                                main()
*
* Description : This is the standard entry point for C code.  It is assumed that your code will call
*               main() once you have performed all necessary initialization.
*
* Arguments   : none
*
* Returns     : none
***************************************************************************************************
*/
int  main(void)
{
    OS_ERR  err;
    CPU_SR_ALLOC();
    BSP_IntDisAll();                                            /* Disable all interrupts.                              */

    OSInit(&err);                                               /* Init uC/OS-III.                                      */
    OS_CRITICAL_ENTER();

    OSTaskCreate((OS_TCB *)&AppTaskStartTCB,                    
                 (CPU_CHAR *)"App Task Start",                                  
                 (OS_TASK_PTR) AppTaskStart,                                       
                 (void *) 0,                                                                
                 (OS_PRIO) APP_TASK_START_PRIO,                             
                 (CPU_STK *)&AppTaskStartStk[0],                            
                 (CPU_STK_SIZE) APP_TASK_START_STK_SIZE / 10,       
                 (CPU_STK_SIZE) APP_TASK_START_STK_SIZE,            
                 (OS_MSG_QTY) 5u,                                   
                 (OS_TICK) 0u,                                      
                 (void *) 0,                                       
                 (OS_OPT)(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),       
                 (OS_ERR *)&err);                                   

    OS_CRITICAL_EXIT(); 
    OSStart(&err);                                              /* Start multitasking (i.e. give control to uC/OS-III). */
}

/*
***************************************************************************************************
*                                          STARTUP TASK
*
* Description : This is the code of the startup task.
*
* Arguments   : p_arg   is the argument passed to 'AppTaskStart()' by 'OSTaskCreate()'.
*
* Returns     : none
*
* Notes       : 1) The first line of code is used to prevent a compiler warning because 'p_arg' is not
*                  used.  The compiler should not generate any code for this statement.
***************************************************************************************************
*/
static  void  AppTaskStart(void *p_arg)
{
    CPU_INT32U  cpu_clk_freq;
    CPU_INT32U  cnts;
    OS_ERR      err;

    VERIFY_RESULT_TYPE_VARIABLE_Typedef eWaitCmdCheckStatu;

    (void)p_arg;

    BSP_Init();

    CPU_Init();

    cpu_clk_freq = BSP_CPU_ClkFreq();                           /* Determine SysTick reference freq.                    */
    cnts = cpu_clk_freq / (CPU_INT32U)OSCfg_TickRate_Hz;        /* Determine nbr SysTick increments                     */
    OS_CPU_SysTickInit(cnts);                                   /* Init uC/OS periodic time src (SysTick).              */

    USER_NVIC_Cfg();

    Mem_Init();                                                 

#if OS_CFG_STAT_TASK_EN > 0u                                   
    OSStatTaskCPUUsageInit(&err);                             /* Compute CPU capacity with no task running            */
#endif

#ifdef CPU_CFG_INT_DIS_MEAS_EN                                
    CPU_IntDisMeasMaxCurReset();                              
#endif


#if OS_CFG_SCHED_ROUND_ROBIN_EN                                                     
    
    OSSchedRoundRobinCfg(DEF_ENABLED, 1, &err);
#endif


#if (APP_CFG_SERIAL_EN == DEF_ENABLED)         
    BSP_Ser_Init(115200);                      
#endif

    OSSemCreate(&g_stAnaSigConvertFinishSem, "Ana Signal convert finish sem", 0, &err);
    OSSemCreate(&IgniteFirstBehindWaitSem, "Ignite First Behind Wait Sem", 0, &err);
    OSSemCreate(&IgniteSecondBehindWaitSem, "Ignite Second Behind Wait Sem...", 0, &err);

    LoadDriverLayerParameters();     

    LoadApplicationLayerParameters();

    App_OS_SetAllHooks();             //钩子函数设置

    CAN1_Init();                      //需先载入组网ID后再进行CAN总线配置，波特率50K

    SystemTimeStatTaskCreate();       

    AnaSigMonitorTaskCreate();

    DigSigMonitorTaskCreate();

    CommTaskCreate();
	
	CommDataSendTaskCreate();

    MF210_CommunicateTaskCreate();//3G模块数据发送任务

    Make_Vacuum_FunctionTaskCreate(); //自动抽真空任务

//    SpeedControlDevManageTaskCreate();//调速设备管理任务

    IgniterWorkTaskCreate();

    HydrgProducerManagerTaskCreate();

    StackManagerTaskCreate();
	
#ifdef  STACK_SHORT_CTRL_EN
    StackShortCtrlTaskCreate();
	
	StackStartUpCtrlTaskCreate();//启动阶段短路以及风机控制任务创建
#endif

    CurrentSmoothlyLimitTaskCreate();

    HydrgProducerDlyStopTaskCreate();

    StackManagerDlyStopTaskCreate();
		
	SystemStackUsageCheckTaskCreate();//任务堆栈使用情况监测任务

    BSP_BuzzerOn();
    OSTimeDlyHMSM(0, 0, 0, 150, OS_OPT_TIME_HMSM_STRICT, &err);
    BSP_BuzzerOff();

    APP_TRACE_INFO(("Running top Task...\n\r"));
	
	OSTaskResume(&MembraneTubeProtectTaskTCB, &err); //开始抽真空
	//测试代码段
	OSTaskResume(&StackRunningShortTaskTCB, &err);//恢复短路活化任务
	SetSystemWorkStatu(EN_RUNNING);
	OSTaskResume(&StackManagerTaskTCB,&err);
	OSTaskSuspend(&AppTaskStartTCB, //阻塞主任务，由制氢机管理任务和电堆管理任务管理机器
				  &err);
	SetSystemWorkStatu(EN_SHUTTING_DOWN);
	
	//测试代码段
    while(DEF_TRUE) {
        if(EN_THROUGH == CheckAuthorization()) {

            eWaitCmdCheckStatu = WaittingCommand();

            if(EN_THROUGH == eWaitCmdCheckStatu) {
                Starting();
                Running();
                KeepingWarm();
            } else {
                SetSystemWorkStatu(EN_ALARMING);
                DeviceFaultAlarm();
            }
        } else {
            SetSystemWorkStatu(EN_ALARMING);
            DeviceFaultAlarm();
        }
    }
}


/*
***************************************************************************************************
*                                   USER_NVIC_Cfg()
*
* Description : The use of this funciton is to set the interrupt group.
*
* Arguments   : none.
*
* Returns     : none
*
* Notes       : none.
***************************************************************************************************
*/
void USER_NVIC_Cfg(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);     //设置NVIC中断分组3位抢占优先级，1位从占优先级
}

/******************* (C) COPYRIGHT 2016 Guangdong ENECO POWER *****END OF FILE****/
