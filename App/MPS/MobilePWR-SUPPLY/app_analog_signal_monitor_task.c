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
* Filename      : app_analog_signal_monitor_task.c
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
#include <includes.h>
#include "app_analog_signal_monitor_task.h"
#include "bsp_ana_sensor.h"
#include "app_system_run_cfg_parameters.h"
#include "app_top_task.h"
#include "bsp_speed_adjust_device.h"
#include "app_wireness_communicate_task.h"
#include "app_stack_manager.h"
#include "app_hydrg_producer_manager.h"
/*
***************************************************************************************************
*                                           MACRO DEFINES
***************************************************************************************************
*/
#define         ANA_SIGNAL_MONITOR_TASK_STK_SIZE        200

/*
***************************************************************************************************
*                                         OS-RELATED    VARIABLES
***************************************************************************************************
*/

OS_TCB      AnaSigMonitorTaskTCB;

OS_SEM      g_stAnaSigConvertFinishSem;
OS_SEM      DevSpdCptureFinishSem;

static      CPU_STK_8BYTE_ALIGNED     AnaSigMonitorTaskStk[ANA_SIGNAL_MONITOR_TASK_STK_SIZE];
/*
***************************************************************************************************
*                                       LOCAL VARIABLES
***************************************************************************************************
*/
static      uint8_t      g_u8HydrgProducerAnaSigRunningMonitorAlarmHookSw = DEF_DISABLED;//制氢机运行模拟信号警报监测开关
static      uint8_t      g_u8StackHydrgPressHighEnoughHookSw = DEF_DISABLED; //等待电堆压力满足开关
static      uint8_t      g_u8StackAnaSigRunningMonitorAlarmHookSw = DEF_DISABLED;//电堆运行模拟信号警报监测开关
static      uint8_t      g_u8StackIsPulledStoppedMonitorHookSw = DEF_DISABLED;//电堆是否被拉停监测开关
static      uint8_t      g_u8StackNeedRestartLimitCurrentFlag = DEF_CLR;//电堆被拉停后重新限流标志

static      uint16_t     g_u16StackManagerHydrgPressBelow10KPaHoldSeconds = 0;       //电堆气压小于10Kpa的秒数
/*
***************************************************************************************************
*                                         FUNCTION PROTOTYPES
***************************************************************************************************
*/
static      void        AnaSigMonitorTask(void *p_arg);

static      void        HydrgProducerAnaSigAlarmRunningMonitorHook(void);
static      void        StackHydrgPressHighEnoughWaitHook(void);
static      void        StackAnaSigAlarmRunningMonitorHook(void);
static      void        JudgeWhetherTheStackIsPulledStoppedMonitorHook(void);
//static      void        HydrgProducerPumpAutoAdjByDecompressCountHook(void);
/*
***************************************************************************************************
*                                 AnaSigMonitorTaskCreate()
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
void  AnaSigMonitorTaskCreate()
{
    OS_ERR      err;

    OSTaskCreate((OS_TCB *)&AnaSigMonitorTaskTCB,                    // Create the start task
                 (CPU_CHAR *)"AnaSignal Monitor Task Start",
                 (OS_TASK_PTR) AnaSigMonitorTask,
                 (void *) 0,
                 (OS_PRIO) ANA_SIGNAL_MONITOR_TASK_PRIO,
                 (CPU_STK *)&AnaSigMonitorTaskStk[0],
                 (CPU_STK_SIZE) ANA_SIGNAL_MONITOR_TASK_STK_SIZE / 10,
                 (CPU_STK_SIZE) ANA_SIGNAL_MONITOR_TASK_STK_SIZE,
                 (OS_MSG_QTY) 5u,
                 (OS_TICK) 0u,
                 (void *) 0,
                 (OS_OPT)(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR *)&err);
    APP_TRACE_INFO(("Created analog signal monitor Task, and err code is %d...\n\r", err));

}

/*
***************************************************************************************************
*                                          ANALOG SIGNAL MONITOR TASK
*
* Description : The use of the the funciton is to create the task that monitor the analog signal.
*
* Arguments   : none.
*
* Returns     : none.
*
* Notes       : The task manager the related analog signals accord to the switches.
***************************************************************************************************
*/
void  AnaSigMonitorTask(void *p_arg)
{
    OS_ERR      err;

    while(DEF_TRUE) {
        OSTimeDlyHMSM(0, 0, 0, 100, OS_OPT_TIME_HMSM_STRICT, &err);

        AnaSigSampleStart();
        OSSemPend(&g_stAnaSigConvertFinishSem,
                  0,                              //等待模拟信号采样完成
                  OS_OPT_PEND_BLOCKING,
                  NULL,
                  &err);
        UpdateAnaSigDigValue();//更新采样值

        /* 制氢机、电堆运行信号监测 */
        if(g_u8HydrgProducerAnaSigRunningMonitorAlarmHookSw == DEF_ENABLED) {
            HydrgProducerAnaSigAlarmRunningMonitorHook();
        }

        if(g_u8StackAnaSigRunningMonitorAlarmHookSw == DEF_ENABLED) {
            StackAnaSigAlarmRunningMonitorHook();
        }

        if(g_u8StackHydrgPressHighEnoughHookSw == DEF_ENABLED) {
            StackHydrgPressHighEnoughWaitHook();
        }

        //监测电堆是否被拉停
        if(g_u8StackIsPulledStoppedMonitorHookSw == DEF_ENABLED) {
            JudgeWhetherTheStackIsPulledStoppedMonitorHook();
        }
        //制氢泵速根据排气次数动态调整
//        if(g_u8HydrgProducerPumpAutoAdjByDecompressCountHookSw == DEF_ENABLED) {
//            HydrgProducerPumpAutoAdjByDecompressCountHook();
//        }
    }
}

/*
***************************************************************************************************
*                               hydrogen producer analog signal alarm hook
*
* Description : The use of the the funciton is to manager the analog signal alarm of the hydrogen producer.
*
* Arguments   : none.
*
* Returns     : none.
*
* Notes       : none.
***************************************************************************************************
*/
void HydrgProducerAnaSigAlarmRunningMonitorHook(void)
{
    float fLqdPress, flqdHeight;

    fLqdPress = GetSrcAnaSig(LIQUID_PRESS);

    if(fLqdPress > g_stLqdPressCmpTbl.AlarmUpperLimit) {
        AlarmCmd(LIQUID_PRESS_HIGH_ALARM, ON);

        if(fLqdPress > g_stLqdPressCmpTbl.ShutDownUpperLimit) {
            APP_TRACE_INFO(("Hydrogen producer liquid press is above the high press protect line...\n\r"));
        }
    } else {
        AlarmCmd(LIQUID_PRESS_HIGH_ALARM, OFF);
    }


    flqdHeight = GetSrcAnaSig(LIQUID_LEVEL);

    if(flqdHeight < g_stLqdHeightCmpTbl.AlarmlowerLiquidLevellimit) {
        AlarmCmd(FUEL_SHORTAGE_ALARM, ON);
    } else {
        AlarmCmd(FUEL_SHORTAGE_ALARM, OFF);

        if(flqdHeight <= g_stLqdHeightCmpTbl.OpenAutomaticliquidValue) { //自动加液水泵
            BSP_OutsidePumpPwrOn();
        } else if(flqdHeight >= g_stLqdHeightCmpTbl.CloseAutomaticliquidValue) {
            BSP_OutsidePumpPwrOff();
        } else {}
    }

//  UpdateBuzzerStatuInCruise();
}

/*
***************************************************************************************************
*                               StackHydrgPressHighEnoughWaitHook
*
* Description : The use of the the funciton is to wait for the hydrogen press up to 45KPa, then start
*                   the work of the stack.
*
* Arguments   : none.
*
* Returns     : none.
*
* Notes       : none.
***************************************************************************************************
*/
void StackHydrgPressHighEnoughWaitHook(void)
{
    OS_ERR err;

    if(GetSrcAnaSig(HYDROGEN_PRESS_1) >= 45.0) {
        OSTaskSemPost(&StackManagerTaskTCB, OS_OPT_POST_NO_SCHED, &err);
        g_u8StackHydrgPressHighEnoughHookSw = DEF_DISABLED;
    }
}

/*
***************************************************************************************************
*                               StackAnaSigAlarmRunningMonitorHook
*
* Description : The use of the the funciton is to manager the analog signal alarms of the stack.
*
* Arguments   : none.
*
* Returns     : none.
*
* Notes       : none.
***************************************************************************************************
*/
void StackAnaSigAlarmRunningMonitorHook(void)
{
    float fStackTemp;
    float fHydrgPress;

    /* 监测温度 */
    fStackTemp = GetSrcAnaSig(STACK_TEMP);

    if(fStackTemp > 60) {
        AlarmCmd(STACK_TEMP_HIGH_ALARM, ON);

        if(fStackTemp > 70) {
//            APP_TRACE_INFO(("Stack temp is above the high temp protect line...\n\r"));
        }
    } else if(fStackTemp < 20) {
        AlarmCmd(STACK_TEMP_LOW_ALARM, ON);

        if(fStackTemp < 10) {
//            APP_TRACE_INFO(("Stack temp is below the low temp protect line...\n\r"));
        }
    } else {
        AlarmCmd(STACK_TEMP_HIGH_ALARM, OFF);
        AlarmCmd(STACK_TEMP_LOW_ALARM, OFF);
    }

    /* 监测气压-可设置低气压关机 */
    fHydrgPress = GetSrcAnaSig(HYDROGEN_PRESS_1);

    if(fHydrgPress >= 10) {
        g_u16StackManagerHydrgPressBelow10KPaHoldSeconds = 0;
        AlarmCmd(HYDROGEN_PRESS_LOW_ALARM, OFF);
    } else {
        AlarmCmd(HYDROGEN_PRESS_LOW_ALARM, ON);
        g_u16StackManagerHydrgPressBelow10KPaHoldSeconds++;

        if(g_u16StackManagerHydrgPressBelow10KPaHoldSeconds >= 300) {//气压过低30s
//            CmdShutDown();      //关机命令
            g_u16StackManagerHydrgPressBelow10KPaHoldSeconds = 0;
        }
    }

}
/*
***************************************************************************************************
*                               SetHydrgProducerAnaSigAlarmRunningMonitorHookSwitch
*
* Description : open the analog signal manager alarms monitor switch when running.
*
* Arguments   : none.
*
* Returns     : none.
*
* Notes       : none.
***************************************************************************************************
*/
void SetHydrgProducerAnaSigAlarmRunningMonitorHookSwitch(uint8_t i_NewStatu)
{
    g_u8HydrgProducerAnaSigRunningMonitorAlarmHookSw = i_NewStatu;
}

/*
***************************************************************************************************
*                               SetStackHydrgPressHighEnoughHookSwitch
*
* Description : open the switch of the hydrogen press monitor to start the stack.
*
* Arguments   : none.
*
* Returns     : none.
*
* Notes       : none.
***************************************************************************************************
*/
void SetStackHydrgPressHighEnoughHookSwitch(uint8_t i_NewStatu)
{
    g_u8StackHydrgPressHighEnoughHookSw = i_NewStatu;
}

/*
***************************************************************************************************
*                               SetStackAnaSigAlarmRunningMonitorHookSwitch
*
* Description : open the switch of the stack analog signal alarm manager switch.
*
* Arguments   : none.
*
* Returns     : none.
*
* Notes       : none.
***************************************************************************************************
*/
void SetStackAnaSigAlarmRunningMonitorHookSwitch(uint8_t i_NewStatu)
{
    g_u8StackAnaSigRunningMonitorAlarmHookSw = i_NewStatu;
}

/*
***************************************************************************************************
*                               SetStackIsPulledStoppedMonitorHookSwitch()
*
* Description : open the switch of the stack analog signal alarm manager switch.
*
* Arguments   : none.
*
* Returns     : none.
*
* Notes       : none.
***************************************************************************************************
*/
void SetStackIsPulledStoppedMonitorHookSwitch(uint8_t i_NewStatu)
{
    g_u8StackIsPulledStoppedMonitorHookSw = i_NewStatu;
}
/*
***************************************************************************************************
*                      JudgeWhetherTheStackIsPulledStoppedMonitorHook()
*
* Description : Judge whether the stack was need restart limit current.
*
* Arguments   : none.
*
* Returns     : none.
*
* Notes       : none.
***************************************************************************************************
*/
static void JudgeWhetherTheStackIsPulledStoppedMonitorHook(void)
{
    float fStackVoltage;
    float fStackCurrent;
    static uint16_t u16RestartLimitCurrentCount = 0;
    
    fStackVoltage = GetSrcAnaSig(STACK_VOLTAGE);
    fStackCurrent = GetSrcAnaSig(STACK_CURRENT);
    
    u16RestartLimitCurrentCount ++;
    if(u16RestartLimitCurrentCount >= 30)//每3秒监测一次是否需要开始重新限流
    if(EN_IN_WORK == GetStackWorkStatu()){
        if((fStackVoltage >= 51.0) && (fStackCurrent <= 3.0)){
            g_u8StackNeedRestartLimitCurrentFlag = DEF_SET;
        }else{
            g_u8StackNeedRestartLimitCurrentFlag = DEF_CLR;
        }
    }
    
}

uint8_t GetStackNeedRestartLimitCurrentFlag(void)
{
    return g_u8StackNeedRestartLimitCurrentFlag;
}
/*
***************************************************************************************************
*                          HydrgProducerPumpAutoAdjByDecompressCountHook()
*
* Description : open the switch of the stack analog signal alarm manager switch.
*
* Arguments   : none.
*
* Returns     : none.
*
* Notes       : none.
***************************************************************************************************
*/
//static void HydrgProducerPumpAutoAdjByDecompressCountHook(void)
//{
//    if((GetSrcAnaSig(STACK_VOLTAGE) <= 51.0) && (GetSrcAnaSig(STACK_CURRENT) >= 3.0) && (GetSrcAnaSig(LIQUID_PRESS) <= 17.5 )){
//        
//    }else{
//        
//    }
//}

/******************* (C) COPYRIGHT 2015 Guangdong Hydrogen *****END OF FILE****/
