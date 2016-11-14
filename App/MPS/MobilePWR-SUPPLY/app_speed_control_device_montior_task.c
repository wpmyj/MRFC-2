/*********************************************************************************************************
*                                              APPLICATION CODE
*
*                          (c) Copyright 2016; Guangdong Hydrogen Energy Science And Technology Co.,Ltd
*
*               All rights reserved.  Protected by international copyright laws.
*               Knowledge of the source code may NOT be used without authorization.
*********************************************************************************************************
*/
/*
*********************************************************************************************************
* Filename      : app_speed_control_device_monitor_task.c
* Version       : V1.00
* Programmer(s) : SunKing.Yun
*********************************************************************************************************
*/
/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/

#include <includes.h>
#include <bsp.h>
#include <app_speed_control_device_monitor_task.h>
#include "app_system_run_cfg_parameters.h"
#include "app_top_task.h"

/*
*********************************************************************************************************
*                                           MACRO DEFINITIONS
*********************************************************************************************************
*/

#define     SPEED_CONTROL_DEVICE_MONITOR_TASK_STK_SIZE          64
/*
*********************************************************************************************************
*                                         OS-RELATED    VARIABLES
*********************************************************************************************************
*/
OS_TCB      SpdCtlDevMonitorTaskTCB;

static      CPU_STK     SpdCtlMonitorTaskStk[SPEED_CONTROL_DEVICE_MONITOR_TASK_STK_SIZE];

/*
*********************************************************************************************************
*                                       LOCAL GLOBAL VARIABLES
*********************************************************************************************************
*/
//调速设备运行速度监测开关
static  uint8_t     g_u8SetPumpRunningSpeedMonitorHookSw = DEF_DISABLED;
static  uint8_t     g_u8HydrgProducerFanRunningSpeedMonitorHookSw = DEF_DISABLED;
static  uint8_t     g_u8SetStackFan1RunningSpeedMonitorHookSw = DEF_DISABLED;
static  uint8_t     g_u8SetStackFan2RunningSpeedMonitorHookSw = DEF_DISABLED;


extern SWITCH_TYPE_VARIABLE_Typedef g_eSpdCaptureWorkSwitch[4];
/*
*********************************************************************************************************
*                                         FUNCTION PROTOTYPES
*********************************************************************************************************
*/

static              void                    SpdCtlDevMonitorTask(void *p_arg);

static      void        HydrgProducerPumpSpeedRunningMonitorHook(void);
static      void        HydrgProducerFanSpeedRunningMonitorHook(void);
//static      void        StackFan1SpeedRunningMonitorHook(void);
//static      void        StackFan2SpeedRunningMonitorHook(void);
/*
*********************************************************************************************************
*                                          SpdCtlDevMonitorTaskCreate()
*
* Description : create the task that monitor the digital signal.
*
* Arguments   : none.
*
* Returns     : none.
*
* Notes       : none.
*********************************************************************************************************
*/
void  SpdCtlDevMonitorTaskCreate(void)
{
    OS_ERR      err;

    OSTaskCreate((OS_TCB *)&SpdCtlDevMonitorTaskTCB,
                 (CPU_CHAR *)"Speed control device Monitor Task Start",
                 (OS_TASK_PTR) SpdCtlDevMonitorTask,
                 (void *) 0,
                 (OS_PRIO) SPEED_CONTROL_DEVICE_MONITOR_TASK_PRIO,
                 (CPU_STK *)&SpdCtlMonitorTaskStk[0],
                 (CPU_STK_SIZE) SPEED_CONTROL_DEVICE_MONITOR_TASK_STK_SIZE / 10,
                 (CPU_STK_SIZE) SPEED_CONTROL_DEVICE_MONITOR_TASK_STK_SIZE,
                 (OS_MSG_QTY) 5u,
                 (OS_TICK) 0u,
                 (void *) 0,
                 (OS_OPT)(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR *)&err);
    APP_TRACE_INFO(("Created Speed control device monitor Task, and err code is %d...\n\r", err));
}

/*
*********************************************************************************************************
*                                          DigSigMonitorTask()
*
* Description : the task monitor all the speed of device
*
* Arguments   : none.
*
* Returns     : none.
*
* Notes       : none.
*********************************************************************************************************
*/
static void   SpdCtlDevMonitorTask(void *p_arg)
{
    OS_ERR      err;

    while(DEF_TRUE)
    {
        OSTimeDlyHMSM(0, 0, 0, 200,
                      OS_OPT_TIME_HMSM_STRICT,
                      &err);

        //监测各速度监测设备开关是否打开
        if(g_u8SetPumpRunningSpeedMonitorHookSw == DEF_ENABLED)
        {
            HydrgProducerPumpSpeedRunningMonitorHook();
        }

        if(g_u8HydrgProducerFanRunningSpeedMonitorHookSw == DEF_ENABLED)
        {
            HydrgProducerFanSpeedRunningMonitorHook();
        }

        if(g_u8SetStackFan1RunningSpeedMonitorHookSw == DEF_ENABLED)
        {
//            StackFan1SpeedRunningMonitorHook();
        }

        if(g_u8SetStackFan2RunningSpeedMonitorHookSw == DEF_ENABLED)
        {
//            StackFan2SpeedRunningMonitorHook();
        }
    }
}

/*
*********************************************************************************************************
*                     SetHydrgProducerPumpRunningSpeedMonitorHookSwitch()
*
* Description : En/Disable the digital signal wait for the ignite first time to behind switchover.
*
* Arguments   : none.
*
* Returns     : none.
*
* Notes       : none.
*********************************************************************************************************
*/
void SetHydrgProducerPumpRunningSpeedMonitorHookSwitch(u8 i_NewStatu)
{
    if(i_NewStatu == DEF_ENABLED)
    {
        g_eSpdCaptureWorkSwitch[0] = ON;
    }
    else
    {
        g_eSpdCaptureWorkSwitch[0] = OFF;
    }

    g_u8SetPumpRunningSpeedMonitorHookSw = i_NewStatu;
}


/*
*********************************************************************************************************
*                     GetHydrgProducerPumpRunningSpeedMonitorHookSwitch()
*
* Description : get the status of hydrg producer pump running speed monitor switch.
*
* Arguments   : none.
*
* Returns     : none.
*
* Notes       : none.
*********************************************************************************************************
*/
SWITCH_TYPE_VARIABLE_Typedef GetHydrgProducerPumpRunningSpeedMonitorSwitchStatus(void)
{
    return g_eSpdCaptureWorkSwitch[0];
}

/*
*********************************************************************************************************
*                     HydrgProducerPumpSpeedRunningMonitorHook()
*
* Description : Hydrg producer pump speed running monitor start.
*
* Arguments   : none.
*
* Returns     : none.
*
* Notes       : none.
*********************************************************************************************************
*/

static void HydrgProducerPumpSpeedRunningMonitorHook(void)
{
    CPU_SR_ALLOC();
    CPU_CRITICAL_ENTER();
    BSP_HydrgProducerPumpMonitorStart();
    CPU_CRITICAL_EXIT();

}
/*
*********************************************************************************************************
*                     SetHydrgProducerFanRunningSpeedMonitorHookSwitch()
*
* Description : En/Disable the running speed control device monitor.
*
* Arguments   : none.
*
* Returns     : none.
*
* Notes       : none.
*********************************************************************************************************
*/
void SetHydrgProducerFanRunningSpeedMonitorHookSwitch(uint8_t i_NewStatu)
{
    if(i_NewStatu == DEF_ENABLED)
    {
        g_eSpdCaptureWorkSwitch[1] = ON;
    }
    else
    {
        g_eSpdCaptureWorkSwitch[1] = OFF;
    }

    g_u8HydrgProducerFanRunningSpeedMonitorHookSw = i_NewStatu;
}


/*
*********************************************************************************************************
*                     GetHydrgProducerFanRunningSpeedMonitorSwitchStatus()
*
* Description : get the status of hydrg producer fan running speed monitor switch.
*
* Arguments   : none.
*
* Returns     : none.
*
* Notes       : none.
*********************************************************************************************************
*/
SWITCH_TYPE_VARIABLE_Typedef GetHydrgProducerFanRunningSpeedMonitorSwitchStatus(void)
{
    return g_eSpdCaptureWorkSwitch[1];
}

/*
*********************************************************************************************************
*                     HydrgProducerFanSpeedRunningMonitorHook()
*
* Description : Hydrg producer fan speed running monitor start.
*
* Arguments   : none.
*
* Returns     : none.
*
* Notes       : none.
*********************************************************************************************************
*/

static void HydrgProducerFanSpeedRunningMonitorHook(void)
{
    CPU_SR_ALLOC();
    CPU_CRITICAL_ENTER();
    BSP_HydrgProducerFanMonitorStart();
    CPU_CRITICAL_EXIT();

}
/*
*********************************************************************************************************
*                     SetStackFanRunningSpeedMonitorHookSwitch()
*
* Description : En/Disable the stack fan 1 running speed monitor hook switchover.
*
* Arguments   : none.
*
* Returns     : none.
*
* Notes       : none.
*********************************************************************************************************
*/
void SetStackFan1RunningSpeedMonitorHookSwitch(u8 i_NewStatu)
{
    if(i_NewStatu == DEF_ENABLED)
    {
        g_eSpdCaptureWorkSwitch[2] = ON;
    }
    else
    {
        g_eSpdCaptureWorkSwitch[2] = OFF;
    }

    g_u8SetStackFan1RunningSpeedMonitorHookSw = i_NewStatu;
}


/*
*********************************************************************************************************
*                     GetStackFan1RunningSpeedMonitorSwitchStatus()
*
* Description : get the status of stack fan 1 running speed monitor switch.
*
* Arguments   : none.
*
* Returns     : none.
*
* Notes       : none.
*********************************************************************************************************
*/
SWITCH_TYPE_VARIABLE_Typedef GetStackFan1RunningSpeedMonitorSwitchStatus(void)
{
    return g_eSpdCaptureWorkSwitch[2];
}


/*
*********************************************************************************************************
*                     StackFan1SpeedRunningMonitorHook()
*
* Description : Stack fan 1 speed running monitor start.
*
* Arguments   : none.
*
* Returns     : none.
*
* Notes       : none.
*********************************************************************************************************
*/

//static void StackFan1SpeedRunningMonitorHook(void)
//{
//    CPU_SR_ALLOC();
//    CPU_CRITICAL_ENTER();
////    BSP_StackFan1SpdMonitorStart();
//    CPU_CRITICAL_EXIT();
//}


/*
*********************************************************************************************************
*                     SetStackFanRunningSpeedMonitorHookSwitch()
*
* Description : En/Disable the stack fan 2 running speed monitor hook switchover.
*
* Arguments   : none.
*
* Returns     : none.
*
* Notes       : none.
*********************************************************************************************************
*/
void SetStackFan2RunningSpeedMonitorHookSwitch(u8 i_NewStatu)
{
    if(i_NewStatu == DEF_ENABLED)
    {
        g_eSpdCaptureWorkSwitch[3] = ON;
    }
    else
    {
        g_eSpdCaptureWorkSwitch[3] = OFF;
    }

    g_u8SetStackFan2RunningSpeedMonitorHookSw = i_NewStatu;
}


/*
*********************************************************************************************************
*                     GetStackFan2RunningSpeedMonitorSwitchStatus()
*
* Description : get the status of stack fan 2 running speed monitor switch.
*
* Arguments   : none.
*
* Returns     : none.
*
* Notes       : none.
*********************************************************************************************************
*/
SWITCH_TYPE_VARIABLE_Typedef GetStackFan2RunningSpeedMonitorSwitchStatus(void)
{
    return g_eSpdCaptureWorkSwitch[3];
}


/*
*********************************************************************************************************
*                     StackFan2SpeedRunningMonitorHook()
*
* Description : Stack fan 2 speed running monitor start.
*
* Arguments   : none.
*
* Returns     : none.
*
* Notes       : none.
*********************************************************************************************************
*/

//static void StackFan2SpeedRunningMonitorHook(void)
//{
//    CPU_SR_ALLOC();
//    CPU_CRITICAL_ENTER();
////    BSP_StackFan2SpdMonitorStart();
//    CPU_CRITICAL_EXIT();
//}
