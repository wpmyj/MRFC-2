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
* Filename      : app_stack_manager.c
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
#include <app_top_task.h>
#include <app_system_run_cfg_parameters.h>
#include <app_system_real_time_parameters.h>
#include "bsp_speed_adjust_device.h"
#include "app_stack_manager.h"
#include "app_speed_control_device_monitor_task.h"
#include "app_huawei_communicate_task.h"
/*
*********************************************************************************************************
*                                           MACRO DEFINITIONS
*********************************************************************************************************
*/
#define STACK_MANAGER_TASK_STK_SIZE                             128
#define STACK_MANAGER_DELAY_STOP_TASK_STK_SIZE                  100
#define STACK_HYDROGEN_YIELD_MATCHING_OFFSET_VALUE_MONITOR_TASK_STK_SIZE     256
#define STACK_PROGRAM_CONTROL_AIR_PRESSURE_RELEASE_TASK_STK_SIZE 200

#define  STACK_FAN_CONTROL_CYCLE   6
/*
*********************************************************************************************************
*                                         OS-RELATED    VARIABLES
*********************************************************************************************************
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
*********************************************************************************************************
*                                           LOCAL VARIABLES
*********************************************************************************************************
*/
SWITCH_TYPE_VARIABLE_Typedef            g_eStackManagerStopDlyStatu = OFF;
STACK_VENTING_TIME_PARAMETER_Typedef    StackVentAirTimeParameter = {0, 0.0, 0.0}; //电堆排气时间参数

static  float                           g_fAmpIntegralSum = 0.0;//电流时间积分累加和
static  float                           g_fHydrogenYieldMatchOffsetValue = 0.0;//电堆匹氢偏移值
static  uint16_t                        g_u16StackExhaustTimesCount = 0;//电堆排气次数
static SWITCH_TYPE_VARIABLE_Typedef     g_u8StackOutAirValveStatus = OFF;//电堆排气阀的打开状态

static  uint8_t                         g_StackStartPurifySw = DEF_DISABLED;
static  uint8_t                         g_u8StackFanAutoAdjSw = DEF_ENABLED;//电堆风扇自动调速开关
static  uint8_t                         g_u8StackProgramControlAirPressureReleaseTaskSw = DEF_DISABLED;
static  uint8_t                         g_u8StackHydrogenMarginMonitorTaskSw = DEF_DISABLED;
static  uint8_t                         g_u8StackNeedVentByAmpIntegralSumSw = DEF_DISABLED;//电堆安培秒排气开关


u8  ChangePWM = 30;
u8  Last_ChangePWM = 0;
/*
*********************************************************************************************************
*                                         FUNCTION PROTOTYPES
*********************************************************************************************************
*/
static      void        StackManagerTask(void);
static      void        StackManagerDlyStopTask(void);
static      void        StackHydrogenYieldMatchingOffsetValueMonitorTask(void);
static      void        StackProgramControlAirPressureReleaseTask(void);

static      void        SetStackStartPurifySwitch(uint8_t i_NewStatu);
static      void        SetStackFanSpdAutoAdjSwitch(uint8_t);
static      void        SetStackProgramControlAirPressureReleaseTaskSwitch(uint8_t i_NewStatu);
static      void        SetStackHydrogenMarginMonitorTaskSwitch(uint8_t i_NewStatu);

void        SetStackHydrgPressHighEnoughHookSwitch(u8);
void        SetStackAnaSigAlarmRunningMonitorHookSwitch(u8);

static      void        StackManagerTask(void);
static      void        SetStackFanSpdAutoAdjSwitch(uint8_t);

static      void        StackManagerDlyStopTask(void);

/*
*********************************************************************************************************
*                                      CREATE STACK MANAGER TASK
*
* Description:  This function creates the stack manager task.
*
* Arguments  :  none
*
* Returns    :  none
*********************************************************************************************************
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
*********************************************************************************************************
*                                      STACK MANAGER TASK
*
* Description:  The stack manager task.
*
* Arguments  :  none
*
* Returns    :  none
*********************************************************************************************************
*/
void StackManagerTask(void)
{
    OS_ERR      err;
    static      uint8_t u8StackFansControlCycleCount = 0;
    float       fVoltage = 0;
    float       fStackTemp = 0;
    float       fCurrent = 0;
    
    while(DEF_TRUE)
    {
        OSTaskSuspend(NULL, &err);
        StackWorkTimesInc();
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
        SetStackWorkStatu(EN_IN_WORK);
        APP_TRACE_INFO(("Finish the stack start purify...\n\r"));
        OSTaskResume(&HuaWeiModuleAdjustTaskTCB,  //华为限流调节任务开始
                     &err);
        fVoltage = GetSrcAnaSig(STACK_VOLTAGE);
        if( fVoltage >= 48)                           //设置为当气压达到45KPA的时候，电压48v打开直流接触器
        {
            BSP_DCConnectValvePwrOn();
//            OSTaskResume(&HuaWeiModuleAdjustTaskTCB,  //华为限流调节任务开始
//                          &err);
        } 
        SetStackProgramControlAirPressureReleaseTaskSwitch(DEF_ENABLED);
        SetStackHydrogenMarginMonitorTaskSwitch(DEF_ENABLED);
        SetStackExhaustTimesCountPerMinutesMonitorHookSwitch(DEF_ENABLED);//1分钟清零一次排气次数
        SetStackAnaSigAlarmRunningMonitorHookSwitch(DEF_ENABLED);
        SetStackFanSpdAutoAdjSwitch(DEF_ENABLED);

        while(DEF_TRUE)
        {  
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
            if(g_fAmpIntegralSum >= 650.0){
                if(OFF == GetStackOutAirValveStatus()){
                    SetStackNeedVentByAmpIntegralSumSwitch(DEF_ON);
//                    g_fAmpIntegralSum = 0.0;//在排气开关处归零
                }
            }
                          
            //自动模式下根据电堆温度自动调节电堆风扇速度
            u8StackFansControlCycleCount++;

            if(g_u8StackFanAutoAdjSw == DEF_ENABLED) { 
                if(u8StackFansControlCycleCount >=  STACK_FAN_CONTROL_CYCLE) {
                    APP_TRACE_DEBUG(("Stack fans auto adjust ...\r\n"));
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
        }
        SetStackAnaSigAlarmRunningMonitorHookSwitch(DEF_DISABLED);
        SetStackFanSpdAutoAdjSwitch(DEF_ENABLED);//若正常运行，会由电堆延时关闭程序关闭，默认应打开，故在此处恢复
        APP_TRACE_INFO(("Stack manager stop...\n\r"));
    }
}
/*
*********************************************************************************************************
*                            SetStackFanSpdAutoAdjSwitch()
*
* Description:  Enable or Disable the stack fan auto adjust.
*               电堆风扇自动校准开关
* Arguments  :  none
*
* Returns    :  none
*********************************************************************************************************
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
*********************************************************************************************************
*                                      CREATE THE STACK MANAGER DELAY STOP TASK
*
* Description:  This function creates the stack manager delay stop task.
*
* Arguments  :  none
*
* Returns    :  none
*********************************************************************************************************
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
*********************************************************************************************************
*                                   THE STACK MANAGER DELAY STOP TASK
*
* Description:  The stack manager delay stop task.
*
* Arguments  :  none
*
* Returns    :  none
*********************************************************************************************************
*/
void StackManagerDlyStopTask(void)
{
    OS_ERR      err;
//    uint16_t    u16ShutDownHydrgPressBelow30KPaHoldSeconds = 0; //制氢机压力小于30Kpa持续时间
    uint16_t    u16ShutDownStackFanDlySeconds = 0;              //关闭电堆管理任务时间

    while(DEF_TRUE) {
        
        OSTaskSuspend(NULL,
                      &err);
        APP_TRACE_DEBUG(("The stack manager start to delay stop...\r\n"));

        g_eStackManagerStopDlyStatu = ON;
        SetStackFanSpdAutoAdjSwitch(DEF_DISABLED);  //关闭风机自动调速

        SetStackFanCtlSpd(1200);

        while(DEF_TRUE) {
            OSTimeDlyHMSM(0, 0, 1, 0,
                          OS_OPT_TIME_HMSM_STRICT,
                          &err);
            u16ShutDownStackFanDlySeconds ++;

            //电堆风扇延时80S后关闭，同时挂起电堆停止任务
            if(u16ShutDownStackFanDlySeconds >= 80) {
                                
                OSSemPost(&StackManagerStopSem,
                          OS_OPT_POST_1,
                          &err);
                OSTaskSemPost(&AppTaskStartTCB, //发送给主任务内的shutdown函数任务信号量响应半机2电堆延时关闭任务
                              OS_OPT_POST_NO_SCHED,
                              &err);
                u16ShutDownStackFanDlySeconds = 0;

                APP_TRACE_DEBUG(("The stack delay stop time arrive 80s,task sem send...\r\n"));
                break;
            }

//            if(GetSrcAnaSig(HYDROGEN_PRESS_1) >= 30) {
//                u16ShutDownHydrgPressBelow30KPaHoldSeconds = 0;
//            } else {
//                u16ShutDownHydrgPressBelow30KPaHoldSeconds++;

//                //电堆气压小于30Kpa的时间达30s
//                if(u16ShutDownHydrgPressBelow30KPaHoldSeconds >= 30) {
//                    OSSemPost(&StackManagerStopSem,
//                              OS_OPT_POST_1,
//                              &err);
//                    u16ShutDownHydrgPressBelow30KPaHoldSeconds = 0;
//                }
//            }
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
    APP_TRACE_DEBUG(("Created the Stack V-hymo value monitor task, and err code is %d...\r\n", err));
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
    float   fAmpIntegralSum = 0.0;        //安培时间积分值
    float   fProportionalConstantA = 0.0;
    
    while(DEF_TRUE) {
        OSTaskSuspend(NULL, &err);

        while(DEF_TRUE) {
            OSTaskSemPend(0,   //接收电堆程控泄压任务中的时间记录完成任务信号量
                          OS_OPT_PEND_BLOCKING,
                          NULL,
                          &err);
            {
                APP_TRACE_DEBUG(("-->fPurifyVentingTimeInterval:%.3f\r\n", StackVentAirTimeParameter.fVentAirTimeIntervalValue * 0.001));
                APP_TRACE_DEBUG(("-->fDecompressVentingTime:%.3f\r\n", StackVentAirTimeParameter.fDecompressVentTimeValue * 0.001));

                //安培秒积分值计算
                fCurrent = GetSrcAnaSig(STACK_CURRENT);
                APP_TRACE_DEBUG(("-->fCurrent:%.3f\r\n", fCurrent)); 
                fAmpIntegralSum = (StackVentAirTimeParameter.fVentAirTimeIntervalValue * 0.001) * fCurrent;

                //安培时间常数计算
                fProportionalConstantA = fAmpIntegralSum / (StackVentAirTimeParameter.fDecompressVentTimeValue * 0.001);
                APP_TRACE_DEBUG(("-->fProportionalConstantA:%.3f\r\n", fProportionalConstantA));
                //匹氢偏移值
                g_fHydrogenYieldMatchOffsetValue = log(600 / fProportionalConstantA); //600为测试值A0
                APP_TRACE_DEBUG(("-->g_fHydrogenYieldMatchOffsetValue:%.3f\r\r\n\n", g_fHydrogenYieldMatchOffsetValue));

                if(g_fHydrogenYieldMatchOffsetValue > 0.5122) { //氢气产出时间小于标准的0.6倍，产气过多
                    //气量充足,可提高输出功率
                    //限流点上升采用逐次周期增加
                    

                } else if(g_fHydrogenYieldMatchOffsetValue < -0.4041) { //氢气产出时间大于标准的1.5倍，产气不足
                    //气量不足,限制输出功率(限流限压)
                    //限流点下降采用线性阶段减小
                    
                } else {
                    //气量合适,清气量异常状态标志位
                }
            }

            if(g_u8StackHydrogenMarginMonitorTaskSw == DEF_DISABLED) {
                break;
            }
        }
    }
}
/*
***************************************************************************************************
*                                 GetStackHydrogenYieldMatchOffsetValue()
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
    //限定匹氢偏移量的值，防止数据传输溢出
    if(g_fHydrogenYieldMatchOffsetValue > 255.0 || g_fHydrogenYieldMatchOffsetValue < -255.0){
        g_fHydrogenYieldMatchOffsetValue = 0;
    }
    return g_fHydrogenYieldMatchOffsetValue;
}

/*
***************************************************************************************************
*                           StackProgramControlPressureReleaseTaskCreate()
*
* Description : Stack Program Control Pressure Release Task Create.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Note(s)     : none.
***************************************************************************************************
*/
void  StackProgramControlPressureReleaseTaskCreate(void)
{
    OS_ERR      err;

    OSTaskCreate((OS_TCB *) & StackProgramControlAirPressureReleaseTaskTCB,
                 (CPU_CHAR *)"Stack program control air pressure release task",
                 (OS_TASK_PTR) StackProgramControlAirPressureReleaseTask,
                 (void *) 0,
                 (OS_PRIO) STACK_PROGRAM_CONTROL_AIR_PRESSURE_RELEASE_TASK_PRIO,
                 (CPU_STK *) &StackProgramControlAirPressureReleaseStk[0],
                 (CPU_STK_SIZE) STACK_PROGRAM_CONTROL_AIR_PRESSURE_RELEASE_TASK_STK_SIZE / 10,
                 (CPU_STK_SIZE) STACK_PROGRAM_CONTROL_AIR_PRESSURE_RELEASE_TASK_STK_SIZE,
                 (OS_MSG_QTY) 5u,
                 (OS_TICK) 0u,
                 (void *) 0,
                 (OS_OPT)(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR *)&err);
    APP_TRACE_DEBUG(("Created the program control air pressure release task, and err code is %d...\r\n", err));
}
/*
***************************************************************************************************
*                               StackHydrogenYieldMatchingOffsetValueMonitorTask()
*
* Description : 程序控制排气阀泄压任务.
*
* Argument(s) : none.
*
* Note(s)     : none.
***************************************************************************************************
*/
void StackProgramControlAirPressureReleaseTask(void)
{
    OS_ERR      err;
    uint8_t u8VentAirTimeIntervalRecordFlag = NO;
    uint8_t u8DecompressVentTimeRecordFlag = NO;
    float   fVolatageBefore = 0.0,fVolatageAfter = 0.0;    
    
    while(DEF_TRUE) {
        OSTaskSuspend(NULL, &err);
        
        while(DEF_TRUE) {
            OSTaskSemPend(0,   //接收AD采样中断中的气压监测判定任务信号量
                          OS_OPT_PEND_BLOCKING,
                          NULL,
                          &err);

            if(err == OS_ERR_NONE) {
                //监测到气压达到气压上限需要打开排气阀
                if(OFF == GetStackOutAirValveStatus()) {
                    BSP_HydrgOutValvePwrOn();  //程控泄压排气
                    fVolatageBefore = GetSrcAnaSig(STACK_VOLTAGE);
                    g_u16StackExhaustTimesCount ++; //排气次数更新
                    StackVentAirTimeParameter.fVentAirTimeIntervalValue = StackVentAirTimeParameter.u32_TimeRecordNum;//记录排气间隔
                    StackVentAirTimeParameter.u32_TimeRecordNum = 0;//清零计数值
                    u8VentAirTimeIntervalRecordFlag = YES;
                    SetStackOutAirValveStatus(ON);
                    APP_TRACE_DEBUG(("-->fVolatageBefore:%.3f\r\n", fVolatageBefore));
                }
                //监测到气压达到气压上限需要关闭排气阀
                else if(ON == GetStackOutAirValveStatus()) {
                    BSP_HydrgOutValvePwrOff();
                    fVolatageAfter = GetSrcAnaSig(STACK_VOLTAGE); 
                    BSP_StartRunningVentingTimeRecord();    //开始记录排气时间参数记录
                    StackVentAirTimeParameter.fDecompressVentTimeValue = StackVentAirTimeParameter.u32_TimeRecordNum;//记录泄压时间
                    StackVentAirTimeParameter.u32_TimeRecordNum = 0;//清零计数值
                    u8DecompressVentTimeRecordFlag = YES;
                    SetStackOutAirValveStatus(OFF);
                    APP_TRACE_DEBUG(("-->fVolatageAfter:%.3f\r\n", fVolatageAfter));
                    APP_TRACE_DEBUG(("-->fVpp:%.3f\r\n\r\n", fVolatageBefore - fVolatageAfter));
                } else {}


                //发送记录的时间参数给氢气裕度检测任务
                if((u8VentAirTimeIntervalRecordFlag == YES) && (u8DecompressVentTimeRecordFlag == YES)) {
                    OSTaskSemPost(&StackHydrogenYieldMatchingOffsetValueMonitorTaskTCB,
                                  OS_OPT_POST_NO_SCHED,
                                  &err);
                    u8VentAirTimeIntervalRecordFlag = NO;//置位标志
                    u8DecompressVentTimeRecordFlag = NO;
                }

                if(g_u8StackProgramControlAirPressureReleaseTaskSw == DEF_DISABLED) {

                    BSP_StopRunningVentingTimeRecord();
                    break;
                }
            }
        }
    }
}

/*
***************************************************************************************************
*                            ResetStackExhaustTimesCountPerMinutes()
*
* Description:  Enable or Disable the stack fan auto adjust.
*
* Arguments  :  none
*
* Returns    :  none
***************************************************************************************************
*/
//复位每分钟电堆排气次数
void ResetStackExhaustTimesCountPerMinutes()
{
    g_u16StackExhaustTimesCount = 0;
}

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
static void SetStackHydrogenMarginMonitorTaskSwitch(uint8_t i_NewStatu)
{
    g_u8StackHydrogenMarginMonitorTaskSw = i_NewStatu;
}
uint8_t GetSStackHydrogenMarginMonitorTaskSwitchStatus()
{
    return g_u8StackHydrogenMarginMonitorTaskSw ;
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
