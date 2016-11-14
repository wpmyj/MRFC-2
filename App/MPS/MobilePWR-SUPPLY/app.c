/*
*********************************************************************************************************
*                                              APPLICATION CODE
*
*                          (c) Copyright 2015; Guangdong Hydrogen Energy Science And Technology Co.,Ltd
*
*               All rights reserved.  Protected by international copyright laws.
*               Knowledge of the source code may NOT be used without authorization.
*********************************************************************************************************
*/
/*
*********************************************************************************************************
* Filename      : app.c
* Version       : V1.00
* Programmer(s) : 
*********************************************************************************************************
*/
/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/
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
#include "app_speed_control_device_monitor_task.h"
#include "app_screen_display_task.h"
#include "RS485CommWithUart5TaskCreate.h"
#include "Make_Vacuum.h"
#include "Time.h"
/*
*********************************************************************************************************
*                                           MACRO DEFINITIONS
*********************************************************************************************************
*/
#define  APP_TASK_START_STK_SIZE                    1024         /*size of TASK STK*/

/*
*********************************************************************************************************
*                                         OS-RELATED    VARIABLES
*********************************************************************************************************
*/
extern      OS_SEM      g_stAnaSigConvertFinishSem;
extern      OS_SEM      IgniteFirstBehindWaitSem;


extern      OS_SEM      IgniteSecondBehindWaitSem;
extern      OS_SEM      MannualSelcetWorkModeSem;

OS_TCB      AppTaskStartTCB;

static CPU_STK_8BYTE_ALIGNED AppTaskStartStk[APP_TASK_START_STK_SIZE];
/*
*********************************************************************************************************
*                                         FUNCTION PROTOTYPES
*********************************************************************************************************
*/

static      void        AppTaskStart(void *p_arg);
static      void        USER_NVIC_Cfg(void);

/*
*********************************************************************************************************
*                                                main()
*
* Description : This is the standard entry point for C code.  It is assumed that your code will call
*               main() once you have performed all necessary initialization.
*
* Arguments   : none
*
* Returns     : none
*********************************************************************************************************
*/
int  main(void)
{
    OS_ERR  err;
    CPU_SR_ALLOC();
    BSP_IntDisAll();                                            /* Disable all interrupts.                              */

    OSInit(&err);                                               /* Init uC/OS-III.                                      */
    OS_CRITICAL_ENTER();//进入临界区

    OSTaskCreate((OS_TCB *)&AppTaskStartTCB,                    //任务控制块
                 (CPU_CHAR *)"App Task Start",                                  //任务名字
                 (OS_TASK_PTR) AppTaskStart,                                        //任务函数
                 (void *) 0,                                                                //传递给任务函数的参数
                 (OS_PRIO) APP_TASK_START_PRIO,                             //任务优先级
                 (CPU_STK *)&AppTaskStartStk[0],                            //任务堆栈基地址
                 (CPU_STK_SIZE) APP_TASK_START_STK_SIZE / 10,       //任务堆栈深度限位
                 (CPU_STK_SIZE) APP_TASK_START_STK_SIZE,            //任务堆栈大小
                 (OS_MSG_QTY) 5u,                                   //任务内部消息队列能够接收的最大消息数目,为0时禁止接收消息
                 (OS_TICK) 0u,                                      //当使能时间片轮转时的时间片长度，为0时为默认长度，
                 (void *) 0,                                        //用户补充的存储区
                 (OS_OPT)(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),       //任务选项
                 (OS_ERR *)&err);                                   //存放该函数错误时的返回值
                 
    OS_CRITICAL_EXIT(); //退出临界区
    OSStart(&err);                                              /* Start multitasking (i.e. give control to uC/OS-III). */
}

/*
*********************************************************************************************************
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
*********************************************************************************************************
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

    Mem_Init();                                                 /*初始化内存管理模块     */

#if OS_CFG_STAT_TASK_EN > 0u                                                                    /*统计任务*/
    OSStatTaskCPUUsageInit(&err);                             /* Compute CPU capacity with no task running            */
#endif

#ifdef CPU_CFG_INT_DIS_MEAS_EN                                //如果使能了测量中断关闭时间
    CPU_IntDisMeasMaxCurReset();                              //重置当前最大禁用中断时间
#endif

    /*
    #if OS_CFG_SCHED_ROUND_ROBIN_EN                                                     //当使用时间片轮转的时候
        //使能时间片轮转调度功能,时间片长度为1个系统时钟节拍，既1*5=5ms
        OSSchedRoundRobinCfg(DEF_ENABLED,1,&err);
    #endif
    */
    
#if (APP_CFG_SERIAL_EN == DEF_ENABLED)                                                    /*串口初始化*/
    BSP_Ser_Init(115200);                                       /* Enable Serial Interface                              */
#endif

    OSSemCreate(&g_stAnaSigConvertFinishSem, "Ana Signal convert finish sem", 0, &err);
    OSSemCreate(&IgniteFirstBehindWaitSem, "Fast heater finish sem", 0, &err);
    OSSemCreate(&IgniteSecondBehindWaitSem, "IgniteSecondBehindWaitSem...", 0, &err);
    
    LoadParameters();                   // 运行参数的初始化，后面需要用到的参数，故需要放在前面。

    App_OS_SetAllHooks();               // 用户应用相关介入函数设置

    SystemTimeStatTaskCreate();         // 系统时钟统计任务创建

    AnaSigMonitorTaskCreate();          // 模拟信号监测任务创建

    DigSigMonitorTaskCreate();          // 数字信号监测任务创建

    SpdCtlDevMonitorTaskCreate();       // 调速设备监测任务

    WirenessCommTaskCreate();           // 无线通信任务创建
    
    RS485CommWithUart5TaskCreate();                     //485与串口通信的任务搭建

    Make_Vacuum_FunctionTaskCreate();                   //抽真空函数任务的搭建
    
    SerToScreenDisplayTaskCreate();     // 串口显示屏显示任务创建

    IgniterWorkTaskCreate();            // 点火工作任务创建

    HydrgProducerManagerTaskCreate();   // 制氢机管理任务

    StackManagerTaskCreate();           // 电堆管理任务

    HydrgProducerDlyStopTaskCreate();   // 制氢机延时关闭任务

    StackManagerDlyStopTaskCreate();    // 电堆延时关闭任务

    while(DEF_TRUE)
    {  
        eWaitCmdCheckStatu = EN_THROUGH;                                        //EN_NOT_THROUGH          
        if( EN_THROUGH == CheckAuthorization() )
        {          
            if( EN_THROUGH == eWaitCmdCheckStatu ) 
            {
                SetWorkMode( EN_WORK_MODE_HYDROGEN_PRODUCER_AND_FUEL_CELL );    // ucosIII test code.
                Starting();            
                Running();
                KeepingWarm();                                                  
            }
            else
            {
                SetSystemWorkStatu( EN_ALARMING );
                DeviceFaultAlarm();
            }
        }
        else
        {
            SetSystemWorkStatu( EN_ALARMING );
            DeviceFaultAlarm();
        }

    }

}


/*
*********************************************************************************************************
*                                          USER NVICConfiguration
*
* Description : The use of this funciton is to set the interrupt group.
*               用户中断优先级配置
* Arguments   : none.
*
* Returns     : none
*
* Notes       : none.
*********************************************************************************************************
*/
void USER_NVIC_Cfg( void )
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);     //设置NVIC中断分组3位抢占优先级，1位从占优先级
}

/******************* (C) COPYRIGHT 2015 Guangdong Hydrogen *****END OF FILE****/
