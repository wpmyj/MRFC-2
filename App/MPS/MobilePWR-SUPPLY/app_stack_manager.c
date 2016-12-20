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
* Filename      : app_stack_manager.c
* Version       : V1.00
* Programmer(s) : FanJun
***************************************************************************************************
*/
/*
***************************************************************************************************
*                                             INCLUDE FILES
***************************************************************************************************
*/
#include <includes.h>
#include <app_top_task.h>
#include <app_system_run_cfg_parameters.h>
#include <app_system_real_time_parameters.h>
#include "bsp_speed_adjust_device.h"
#include "app_stack_manager.h"
#include "app_dc_module_communicate_task.h"
#include "app_analog_signal_monitor_task.h"
#include "bsp_pid.h"
/*
***************************************************************************************************
*                                           MACRO DEFINITIONS
***************************************************************************************************
*/
#define STACK_MANAGER_TASK_STK_SIZE                             128
#define STACK_MANAGER_DELAY_STOP_TASK_STK_SIZE                  100
#define STACK_HYDROGEN_YIELD_MATCHING_OFFSET_VALUE_MONITOR_TASK_STK_SIZE     256
#define STACK_PROGRAM_CONTROL_AIR_PRESSURE_RELEASE_TASK_STK_SIZE 200

#define  STACK_FAN_CONTROL_CYCLE   6  //风机控制周期
/*
***************************************************************************************************
*                                         OS-RELATED    VARIABLES
***************************************************************************************************
*/

OS_TCB      StackManagerTaskTCB;
OS_TCB      StackManagerDlyStopTaskTCB;
OS_TCB      StackHydrogenYieldMatchingOffsetValueMonitorTaskTCB;
OS_TCB      StackProgramControlAirPressureReleaseTaskTCB;

static      OS_SEM      StackManagerStopSem;
static      CPU_STK     StackManagerTaskStk[STACK_MANAGER_TASK_STK_SIZE];
static      CPU_STK     StackManagerDlyStopTaskStk[STACK_MANAGER_DELAY_STOP_TASK_STK_SIZE];
static      CPU_STK     StackHydrogenYieldMatchingOffsetValueMonitorStk[STACK_HYDROGEN_YIELD_MATCHING_OFFSET_VALUE_MONITOR_TASK_STK_SIZE];
static      CPU_STK     StackProgramControlAirPressureReleaseStk[STACK_PROGRAM_CONTROL_AIR_PRESSURE_RELEASE_TASK_STK_SIZE];

/*
***************************************************************************************************
*                                          GLOBAL VARIABLES
***************************************************************************************************
*/
SWITCH_TYPE_VARIABLE_Typedef            g_eStackManagerStopDlyStatu = OFF;
STACK_VENTING_TIME_PARAMETER_Typedef    StackVentAirTimeParameter = {0, 0.0, 0.0}; //电堆排气时间参数

/*
***************************************************************************************************
*                                           LOCAL VARIABLES
***************************************************************************************************
*/
static  float                           g_fAmpIntegralSum = 0.0;//电流时间积分累加和
static  float                           g_fHydrogenYieldMatchOffsetValue = 0.0;//电堆匹氢偏移值
static  uint8_t                         g_u8DecompressCountPerMinute = 0; //每分钟电堆尾部泄压排气次数
static SWITCH_TYPE_VARIABLE_Typedef     g_u8StackOutAirValveStatus = OFF;//电堆排气阀的打开状态

static  uint8_t                         g_StackStartPurifySw = DEF_DISABLED;
static  uint8_t                         g_u8StackFanAutoAdjSw = DEF_ENABLED;//电堆风扇自动调速开关
static  uint8_t                         g_u8StackProgramControlAirPressureReleaseTaskSw = DEF_DISABLED;
static  uint8_t                         g_u8StackHydrogenYieldMatchingOffsetValueMonitorTaskSw = DEF_DISABLED;
static  uint8_t                         g_u8StackNeedVentByAmpIntegralSumSw = DEF_DISABLED;//电堆安培秒排气开关

/*
***************************************************************************************************
*                                         FUNCTION PROTOTYPES
***************************************************************************************************
*/
static      void        StackManagerTask(void);
static      void        StackManagerDlyStopTask(void);
static      void        StackHydrogenYieldMatchingOffsetValueMonitorTask(void);
static      void        StackProgramControlAirPressureReleaseTask(void);

static      void        SetStackStartPurifySwitch(uint8_t i_NewStatu);
static      void        SetStackFanSpdAutoAdjSwitch(uint8_t);
static      void        SetStackProgramControlAirPressureReleaseTaskSwitch(uint8_t i_NewStatu);
static      void        SetStackHydrogenYieldMatchingOffsetValueMonitorTaskSwitch(uint8_t i_NewStatu);

static      void        StackManagerTask(void);
static      void        SetStackFanSpdAutoAdjSwitch(uint8_t);

static      void        StackManagerDlyStopTask(void);

/*
***************************************************************************************************
*                                      CREATE STACK MANAGER TASK
*
* Description:  This function creates the stack manager task.
*
* Arguments  :  none
*
* Returns    :  none
***************************************************************************************************
*/
void StackManagerTaskCreate(void)
{
    OS_ERR  err;

    OSSemCreate(&StackManagerStopSem, "Stack manager sem", 0, &err);

    OSTaskCreate((OS_TCB *)&StackManagerTaskTCB,                    // Create the start task
                 (CPU_CHAR *)"Stack manager task",
                 (OS_TASK_PTR) StackManagerTask,
                 (void *) 0,
                 (OS_PRIO) STACK_MANAGER_TASK_PRIO,
                 (CPU_STK *)&StackManagerTaskStk[0],
                 (CPU_STK_SIZE) STACK_MANAGER_TASK_STK_SIZE / 10,
                 (CPU_STK_SIZE) STACK_MANAGER_TASK_STK_SIZE,
                 (OS_MSG_QTY) 5u,
                 (OS_TICK) 0u,
                 (void *) 0,
                 (OS_OPT)(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR *)&err);
    APP_TRACE_INFO(("Created stack manager task, and err code is %d...\n\r", err));
}

/*
***************************************************************************************************
*                                      STACK MANAGER TASK
*
* Description:  The stack manager task.
*
* Arguments  :  none
*
* Returns    :  none
***************************************************************************************************
*/
void StackManagerTask(void)
{
    OS_ERR      err;
    static      uint8_t u8StackFansControlCycleCount = 0;
    float       fVoltage = 0;
    float       fStackTemp = 0;
    float       fCurrent = 0;

    while(DEF_TRUE) {
        OSTaskSuspend(NULL, &err);
        //运行提醒
        BSP_BuzzerOn();
        OSTimeDlyHMSM(0, 0, 0, 150, OS_OPT_TIME_HMSM_STRICT, &err);
        BSP_BuzzerOff();
        OSTimeDlyHMSM(0, 0, 0, 150, OS_OPT_TIME_HMSM_STRICT, &err);

        StackWorkTimesInc();
#ifdef STACK_FAN_PID_CONTROL
        IncrementType_PID_Init();   //增量式PID初始化
#endif
        BSP_HydrgInValvePwrOn();

        APP_TRACE_INFO(("Stack manager start, waitting the hydrogen press up to 45KPa ...\n\r"));
        SetStackHydrgPressHighEnoughHookSwitch(DEF_ENABLED);
        SetStackStartPurifySwitch(DEF_ENABLED);//正在排杂状态

        OSTaskSemPend(0, OS_OPT_PEND_BLOCKING, NULL, &err); //接收模拟信号监测任务中气压监测任务信号量
        SetStackFanCtlSpd(500);
        APP_TRACE_INFO(("Start the stack start purify...\n\r"));
        BSP_HydrgOutValvePwrOn();
        OSTimeDlyHMSM(0, 0, 5, 0, OS_OPT_TIME_HMSM_STRICT, &err);
        BSP_HydrgOutValvePwrOff();
        SetStackStartPurifySwitch(DEF_DISABLED);
        APP_TRACE_INFO(("Finish the stack start purify...\n\r"));

        fVoltage = GetSrcAnaSig(STACK_VOLTAGE);

        if(fVoltage >= 48) {        //设置为当气压达到45KPA的时候，电压48v打开直流接触器
            BSP_DCConnectValvePwrOn();
            SetStackWorkStatu(EN_IN_WORK);
            OSTaskResume(&DcModuleAdjustTaskTCB,  //华为限流调节任务开始
                         &err);
        }

        StackHydrogenYieldMatchingOffsetValueMonitorTaskCreate();//电堆匹氢偏移量监测任务创建

        SetStackHydrogenYieldMatchingOffsetValueMonitorTaskSwitch(DEF_ENABLED);
        SetStackExhaustTimesCountPerMinutesMonitorHookSwitch(DEF_ENABLED);//1分钟清零一次排气次数
        SetStackAnaSigAlarmRunningMonitorHookSwitch(DEF_ENABLED);
        SetStackFanSpdAutoAdjSwitch(DEF_ENABLED);

        OSTaskResume(&StackHydrogenYieldMatchingOffsetValueMonitorTaskTCB,  //恢复电堆氢气裕度监测任务
                     &err);

        while(DEF_TRUE) {
            OSSemPend(&StackManagerStopSem,
                      OS_CFG_TICK_RATE_HZ,
                      OS_OPT_PEND_BLOCKING,
                      NULL,
                      &err);

            if(err == OS_ERR_NONE) {
                break;
            }

            //安培积分值累加和
            fCurrent = GetSrcAnaSig(STACK_CURRENT);
            g_fAmpIntegralSum += fCurrent;

            if(g_fAmpIntegralSum >= g_u16RunPurifyAmpIntegralValue) {
                BSP_HydrgOutValvePwrOn();
                OSTimeDlyHMSM(0, 0, 5, 0, OS_OPT_TIME_HMSM_STRICT, &err);
                BSP_HydrgOutValvePwrOff();
                g_fAmpIntegralSum = 0.0;
            }

#ifdef STACK_FAN_PID_CONTROL
            IPID.CalcCycleCount++;
            fOptimumStackTemp = CalcStackOptimumTemperatureByCurrent();
            IncrementType_PID_Process(fOptimumStackTemp);
            APP_TRACE_INFO(("--> IPID.Sv-Pv-Err-Err_Next-Err_Last-Pout-Iout-Dout-Out: %d--%d--%d--%d--%d--%.2f--%.2f--%.2f--%d:\r\n", IPID.Sv, IPID.Pv, IPID.Err, IPID.Err_Next, IPID.Err_Last, IPID.Pout, IPID.Iout, IPID.Dout, IPID.OutValue));
#else
            //自动模式下根据电堆温度自动调节电堆风扇速度
            u8StackFansControlCycleCount++;

            if(g_u8StackFanAutoAdjSw == DEF_ENABLED) {
                if(u8StackFansControlCycleCount >=  STACK_FAN_CONTROL_CYCLE) {
                    APP_TRACE_INFO(("Stack fans auto adjust ...\r\n"));
                    fStackTemp = GetSrcAnaSig(STACK_TEMP);

                    if(fStackTemp <= 25) {
                        SetStackFanCtlSpd(500);
                    } else if(fStackTemp <= 40) {
                        SetStackFanCtlSpd(500 + (uint16_t)((fStackTemp - 25) * 120 / 4));
                    } else if(fStackTemp <= 50) {
                        SetStackFanCtlSpd(950 + (uint16_t)((fStackTemp - 40) * 235 / 5));//47
                    } else if(fStackTemp <= 60) {
                        SetStackFanCtlSpd(1420 + (uint16_t)((fStackTemp - 50) * 153 / 3));//51
                    } else {
                        SetStackFanCtlSpd(2000);
                    }

                    u8StackFansControlCycleCount = 0;
                }
            } else {}

#endif
        }

        SetStackExhaustTimesCountPerMinutesMonitorHookSwitch(DEF_DISABLED);
        SetStackAnaSigAlarmRunningMonitorHookSwitch(DEF_DISABLED);
        SetStackFanSpdAutoAdjSwitch(DEF_ENABLED);//若正常运行，会由电堆延时关闭程序关闭，默认应打开，故在此处恢复
        APP_TRACE_INFO(("Stack manager stop...\n\r"));
    }
}
/*
***************************************************************************************************
*                            SetStackFanSpdAutoAdjSwitch()
*
* Description:  Enable or Disable the stack fan auto adjust.
*
* Arguments  :  none
*
* Returns    :  none
***************************************************************************************************
*/
void SetStackFanSpdAutoAdjSwitch(uint8_t i_NewStatu)
{
    g_u8StackFanAutoAdjSw = i_NewStatu;
}

static void SetStackStartPurifySwitch(uint8_t i_NewStatu)
{
    g_StackStartPurifySw = i_NewStatu;
}

uint8_t GetStackStartPurifySwitchStatus()
{
    return g_StackStartPurifySw;
}
/*
***************************************************************************************************
*                                      CREATE THE STACK MANAGER DELAY STOP TASK
*
* Description:  This function creates the stack manager delay stop task.
*
* Arguments  :  none
*
* Returns    :  none
***************************************************************************************************
*/
void StackManagerDlyStopTaskCreate()
{
    OS_ERR  err;
    OSTaskCreate((OS_TCB *)&StackManagerDlyStopTaskTCB,
                 (CPU_CHAR *)"Stack manager delay stop task",
                 (OS_TASK_PTR) StackManagerDlyStopTask,
                 (void *) 0,
                 (OS_PRIO) STACK_MANAGER_DELAY_STOP_TASK_PRIO,
                 (CPU_STK *)&StackManagerDlyStopTaskStk[0],
                 (CPU_STK_SIZE) STACK_MANAGER_DELAY_STOP_TASK_STK_SIZE / 10,
                 (CPU_STK_SIZE) STACK_MANAGER_DELAY_STOP_TASK_STK_SIZE,
                 (OS_MSG_QTY) 5u,
                 (OS_TICK) 0u,
                 (void *) 0,
                 (OS_OPT)(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR *)&err);
    APP_TRACE_INFO(("Created the stack manager delay stop task, err code is %d...\n\r", err));
}
/*
***************************************************************************************************
*                                   THE STACK MANAGER DELAY STOP TASK
*
* Description:  The stack manager delay stop task.
*
* Arguments  :  none
*
* Returns    :  none
***************************************************************************************************
*/
void StackManagerDlyStopTask(void)
{
    OS_ERR      err;
    uint16_t    u16ShutDownStackFanDlySeconds = 0;  //电堆延时关闭时间计时

    while(DEF_TRUE) {

        OSTaskSuspend(NULL,
                      &err);
        APP_TRACE_INFO(("The stack manager start to delay stop...\r\n"));

        g_eStackManagerStopDlyStatu = ON;
        SetStackFanSpdAutoAdjSwitch(DEF_DISABLED);  //关闭风机自动调速

        SetStackFanCtlSpd(1200);

        while(DEF_TRUE) {
            OSTimeDlyHMSM(0, 0, 1, 0,
                          OS_OPT_TIME_HMSM_STRICT,
                          &err);
            u16ShutDownStackFanDlySeconds ++;

            //电堆风扇延时30S后关闭，同时挂起电堆停止任务
            if(u16ShutDownStackFanDlySeconds >= 30) {

                OSSemPost(&StackManagerStopSem,
                          OS_OPT_POST_1,
                          &err);
                OSTaskSemPost(&AppTaskStartTCB, //发送给主任务内的shutdown函数任务信号量响应半机2电堆延时关闭任务
                              OS_OPT_POST_NO_SCHED,
                              &err);
                u16ShutDownStackFanDlySeconds = 0;

                APP_TRACE_INFO(("The stack delay stop time arrive 30s,task sem send...\r\n"));
                break;
            }
        }

        BSP_HydrgInValvePwrOff();
        BSP_HydrgOutValvePwrOff();

        SetStackWorkStatu(EN_NOT_IN_WORK);
        SetStackFanCtlSpd(0);

        BSP_DCConnectValvePwrOff();
        g_eStackManagerStopDlyStatu = OFF;
    }
}
/*
***************************************************************************************************
*                             GetStackStopDlyStatu()
*
* Description:  Get the delay statu of the stack stop work.
*
* Arguments  :  none
*
* Returns    :  none
***************************************************************************************************
*/
uint8_t GetStackStopDlyStatu(void)
{
    return g_eStackManagerStopDlyStatu;
}
/*
***************************************************************************************************
*                       StackHydrogenYieldMatchingOffsetValueMonitorTaskCreate()
*
* Description : Stack hydrogen yield matching offset value monitor task creat.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Note(s)     : none.
***************************************************************************************************
*/
void    StackHydrogenYieldMatchingOffsetValueMonitorTaskCreate()
{
    OS_ERR      err;

    OSTaskCreate((OS_TCB *)&StackHydrogenYieldMatchingOffsetValueMonitorTaskTCB,
                 (CPU_CHAR *)"Stack hydrogen yield matching offset value monitor Task",
                 (OS_TASK_PTR) StackHydrogenYieldMatchingOffsetValueMonitorTask,
                 (void *) 0,
                 (OS_PRIO) STACK_HYDROGEN_YIELD_MATCHING_OFFSET_VALUE_MONITOR_TASK_PRIO,
                 (CPU_STK *) &StackHydrogenYieldMatchingOffsetValueMonitorStk[0],
                 (CPU_STK_SIZE) STACK_HYDROGEN_YIELD_MATCHING_OFFSET_VALUE_MONITOR_TASK_STK_SIZE / 10,
                 (CPU_STK_SIZE) STACK_HYDROGEN_YIELD_MATCHING_OFFSET_VALUE_MONITOR_TASK_STK_SIZE,
                 (OS_MSG_QTY) 5u,
                 (OS_TICK) 0u,
                 (void *) 0,
                 (OS_OPT)(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR *)&err);
    APP_TRACE_INFO(("Created the Stack V-hymo value monitor task, and err code is %d...\r\n", err));
}
/*
***************************************************************************************************
*                               StackHydrogenYieldMatchingOffsetValueMonitorTask()
*
* Description : Stack hydrogen yield matching offset value monitor task.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Note(s)     : If hydrogen yield matching offset value greater than zero show the hydrogen is excessive，
*               otherwise the hydrogen is not enough.
***************************************************************************************************
*/
void StackHydrogenYieldMatchingOffsetValueMonitorTask()
{
    OS_ERR      err;
    float   fCurrent = 0.0;
    float   fAmpIntegralValue = 0.0;        //安培时间积分值
    float   fActualHydrogenMatchValue = 0.0;
    static  uint8_t u8HydrogenYieldEnoughCount = 0;
    static  uint8_t u8HydrogenYieldLackCount = 0;

    while(DEF_TRUE) {
        OSTaskSuspend(NULL, &err);

        while(DEF_TRUE) {
            OSTaskSemPend(0,   //接收泄压阀输入脉冲中断中的时间参数记录完成任务信号量
                          OS_OPT_PEND_BLOCKING,
                          NULL,
                          &err);
            {
                //安培时间积分值计算
                fCurrent = GetSrcAnaSig(STACK_CURRENT);
                fAmpIntegralValue = (StackVentAirTimeParameter.fVentAirTimeIntervalValue * 0.001) * fCurrent;

                //实际匹氢值计算
                fActualHydrogenMatchValue = fAmpIntegralValue / (StackVentAirTimeParameter.fDecompressVentTimeValue * 0.001);

                //匹氢偏移值ln(600/ActualHydrogenMatchValue)
                g_fHydrogenYieldMatchOffsetValue = log(600 / fActualHydrogenMatchValue); //600为测试值A0

                APP_TRACE_INFO(("-->Current-VentingInterval-DecompressTime:-#%.3f   -$%.3f   -&%.3f   \r\n", \
                                fCurrent, StackVentAirTimeParameter.fVentAirTimeIntervalValue * 0.001, StackVentAirTimeParameter.fDecompressVentTimeValue * 0.001));

                if(g_fHydrogenYieldMatchOffsetValue > 1.20) {//匹氢值小于标准的0.3倍(偏移值为正)，产气充足，可提高输出功率
                    //限流点上升和下降都采用逐次周期增加
                    u8HydrogenYieldEnoughCount ++;
                    u8HydrogenYieldLackCount = 0;

                    if(u8HydrogenYieldEnoughCount >= 10) {
                        OSTaskSemPost(&DcModuleAdjustTaskTCB,
                                      OS_OPT_POST_1,
                                      &err);
                        SetDcModuleCurrentLimitingPointImproveFlag(DEF_SET);
                        u8HydrogenYieldEnoughCount = 0;
                    }
                } else if(g_fHydrogenYieldMatchOffsetValue < -0.40) {//匹氢值大于标准的1.5倍，产气不足,限制输出功率(限流限压)
                    u8HydrogenYieldLackCount ++;
                    u8HydrogenYieldEnoughCount = 0;

                    if(u8HydrogenYieldLackCount >= 10) {
                        OSTaskSemPost(&DcModuleAdjustTaskTCB,
                                      OS_OPT_POST_1,
                                      &err);
                        SetDcModuleCurrentLimitingPointReduceFlag(DEF_SET);
                        u8HydrogenYieldLackCount = 0;
                    }
                } else {
                    //气量合适,清不正常计数值
                    u8HydrogenYieldEnoughCount = 0;
                    u8HydrogenYieldLackCount = 0;
                }
            }

            if(g_u8StackHydrogenYieldMatchingOffsetValueMonitorTaskSw == DEF_DISABLED) {
                break;
            }
        }
    }
}
/*
***************************************************************************************************
*                        GetStackHydrogenYieldMatchOffsetValue()
*
* Description:  Get the  hydrogen yield match offset value.
*
* Arguments  :  none
*
* Returns    :  none
***************************************************************************************************
*/
float GetStackHydrogenYieldMatchOffsetValue(void)
{
    /* To prevent data transmission overflow */
    if(g_fHydrogenYieldMatchOffsetValue > 255.0 || g_fHydrogenYieldMatchOffsetValue < -255.0) {
        g_fHydrogenYieldMatchOffsetValue = 0;
    }

    return g_fHydrogenYieldMatchOffsetValue;
}

/*
***************************************************************************************************
*                            ResetStackExhaustTimesCountPerMinutes()
*
* Description:  电堆尾部泄压排气次数监测.
*
* Arguments  :  none
*
* Returns    :  none
***************************************************************************************************
*/

void DecompressCountPerMinuteInc(void)
{
    if(EN_IN_WORK == GetStackWorkStatu()) {
        g_u8DecompressCountPerMinute ++;
    }
}
void ResetStackExhaustTimesCountPerMinutes(void)
{
    g_u8DecompressCountPerMinute = 0;
}

uint8_t GetPassiveDecompressCountPerMinutes(void)
{
    return g_u8DecompressCountPerMinute;
}
/*
***************************************************************************************************
*                            SetStackOutAirValveStatus()
*
* Description:  Set stack out valve status.
*
* Arguments  :  none
*
* Returns    :  none
***************************************************************************************************
*/
void SetStackOutAirValveStatus(SWITCH_TYPE_VARIABLE_Typedef i_NewStatu)
{
    g_u8StackOutAirValveStatus = i_NewStatu;
}

SWITCH_TYPE_VARIABLE_Typedef GetStackOutAirValveStatus()
{
    return g_u8StackOutAirValveStatus;
}
/*
***************************************************************************************************
*                            SetStackProgramControlAirPressureReleaseTaskSwitch()
*
* Description:  Enable or Disable the stack fan auto adjust.
*
* Arguments  :  none
*
* Returns    :  none
***************************************************************************************************
*/
static void SetStackHydrogenYieldMatchingOffsetValueMonitorTaskSwitch(uint8_t i_NewStatu)
{
    g_u8StackHydrogenYieldMatchingOffsetValueMonitorTaskSw = i_NewStatu;
}
uint8_t GetStackHydrogenYieldMatchingOffsetValueMonitorTaskSwitchStatus()
{
    return g_u8StackHydrogenYieldMatchingOffsetValueMonitorTaskSw ;
}

static void SetStackProgramControlAirPressureReleaseTaskSwitch(uint8_t i_NewStatu)
{
    g_u8StackProgramControlAirPressureReleaseTaskSw = i_NewStatu;
}

uint8_t GetStackProgramControlAirPressureReleaseTaskSwitch()
{
    return g_u8StackProgramControlAirPressureReleaseTaskSw;
}

/*
***************************************************************************************************
*                        SetStackNeedVentByAmpIntegralSumSwitch()
*
* Description:  Enable or Disable the stack fan auto adjust.
*
* Arguments  :  none
*
* Returns    :  none
***************************************************************************************************
*/
void SetStackNeedVentByAmpIntegralSumSwitch(uint8_t i_NewStatu)
{
    g_u8StackNeedVentByAmpIntegralSumSw = i_NewStatu;
}

uint8_t GetStackNeedVentByAmpIntegralSumSwitch(void)
{
    return g_u8StackNeedVentByAmpIntegralSumSw;
}

void SetStackAmpIntegralSum(float i_NewValue)
{
    g_fAmpIntegralSum = i_NewValue;
}
float GetStackAmpIntegralSum(void)
{
    return g_fAmpIntegralSum;
}
/******************* (C) COPYRIGHT 2015 Guangdong Hydrogen *****END OF FILE****/
