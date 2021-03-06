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
* Programmer(s) : JasonFan
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
#include "app_dc_module_communicate_task.h"
#include "app_mf210_communicate_task.h"
#include "bsp_MF210v2.h"
#include "app_stack_short_circuit_task.h"
/*
***************************************************************************************************
*                                           MACRO DEFINES
***************************************************************************************************
*/
#define         ANA_SIGNAL_MONITOR_TASK_STK_SIZE        200

//保护报警开关
#define         STACK_TEMP_MONITOR_SWITCH                1u
#define         STACK_AIR_PRESS_MONITOR_SWITCH           1u
#define         STACK_VOLETAGE_MONITOR_SWITCH            0u
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
static      uint8_t      g_HP_PumpRunStartAutoAdjHookSw = DEF_DISABLED;   //制氢机泵速自动调节开关
static      uint8_t      g_u8StackHydrgPressArriveWaitSw = DO_NOT_WAIT; //等待电堆压力满足开关
static      uint8_t      g_u8StackAnaSigRunningMonitorAlarmHookSw = DEF_DISABLED;//电堆运行模拟信号警报监测开关
static      uint8_t      g_u8StackIsPulledStoppedMonitorHookSw = DEF_DISABLED;//电堆是否被拉停监测开关
static      uint8_t      g_u8RestartLimitCurrentFlag = DEF_CLR;//电堆被拉停后重新限流标志
static      uint8_t      g_u8PumpAutoAdjFinishStatu = DEF_NO;//运行首次泵速调整状态

/*
***************************************************************************************************
*                                         FUNCTION PROTOTYPES
***************************************************************************************************
*/
static      void        AnaSigMonitorTask(void *p_arg);

static      void        HydrgProducerAnaSigAlarmRunningMonitorHook(void);
static      void        StackAnaSigAlarmRunningMonitorHook(void);
static      void        JudgeWhetherTheStackIsPulledStoppedMonitorHook(void);
static      void        HydrgProducerPumpRunningStartAutoAdjHook(void);
static 		void 		StackHydrgPressHighEnoughWaitHook(uint8_t i_u8WaitStatus);
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

        //等待氢气压力达到
        if(DO_NOT_WAIT != g_u8StackHydrgPressArriveWaitSw) {
            StackHydrgPressHighEnoughWaitHook(g_u8StackHydrgPressArriveWaitSw);
        }

        //监测电堆是否被拉停
        if(g_u8StackIsPulledStoppedMonitorHookSw == DEF_ENABLED) {
            JudgeWhetherTheStackIsPulledStoppedMonitorHook();
        }

        if(g_HP_PumpRunStartAutoAdjHookSw == DEF_ENABLED) {
            HydrgProducerPumpRunningStartAutoAdjHook();//液压大于4公斤，制氢机自动调节泵速，只调节一次
        }

//        if(SocketRecv(SOCKET_ID, Uart2RxBuf)) {
//            RespondRemoteControlCmd();    //接收服务器控制指令
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
* Notes       : New alarm program.
***************************************************************************************************
*/
void HydrgProducerAnaSigAlarmRunningMonitorHook(void)
{
    float fLqdPress, flqdHeight;

    fLqdPress = GetSrcAnaSig(LIQUID_PRESS);

    if(fLqdPress > g_stLqdPressCmpTbl.AlarmUpperLimit) {
        AlarmCmd(LIQUID_PRESS_HIGH_ALARM, SERIOUS_GRADE, ON);

        if(fLqdPress > g_stLqdPressCmpTbl.ShutDownUpperLimit) {
            APP_TRACE_INFO(("Hydrogen producer liquid press is above the high press protect line...\n\r"));
        }
    } else {
        AlarmCmd(LIQUID_PRESS_HIGH_ALARM, SERIOUS_GRADE, OFF);
    }


    flqdHeight = GetSrcAnaSig(LIQUID_LEVEL1);

    if(flqdHeight < g_stLqdHeightCmpTbl.AlarmlowerLiquidLevellimit) {
        AlarmCmd(FUEL_SHORTAGE_ALARM, GENERAL_GRADE, ON);

    } else {
        AlarmCmd(FUEL_SHORTAGE_ALARM, GENERAL_GRADE, OFF);

        if(flqdHeight <= g_stLqdHeightCmpTbl.OpenAutomaticliquidValue) { //自动加液水泵
            BSP_OutsidePumpPwrOn();
        } else if(flqdHeight >= g_stLqdHeightCmpTbl.CloseAutomaticliquidValue) {
            BSP_OutsidePumpPwrOff();
        } else {}
    }
}

/*
***************************************************************************************************
*                               HydrgProducerAnaSigRunningStartAutoAdjHook()
*
* Description : The funciton is to auto adjust the pump at beginning of the run process with the analog signal.
*
* Arguments   : none.
*
* Returns     : none.
*
* Notes       : none.
***************************************************************************************************
*/

void HydrgProducerPumpRunningStartAutoAdjHook(void)
{
    float fLqdPress;
    static u8 i = 0;

    fLqdPress = GetSrcAnaSig(LIQUID_PRESS);
    i++;

    if(i >= 10) { //模拟信号检测周期是100ms一次，计10次也即1秒调节1次
        i = 0;

        if((fLqdPress >= 4) && (g_u8PumpAutoAdjFinishStatu == DEF_NO)) {
            PumpSpdDec();

            if(GetPumpCurrentCtrlSpd() <= g_stStartHydrgPumpSpdPara.PumpSpdAfterLiquidPressExceed4Kg) {
                g_u8PumpAutoAdjFinishStatu = DEF_YES;
                SetPumpStartAutoAdjHookSwitch(DEF_DISABLED);//关调节开关
            }
        }
    }
}

/*
***************************************************************************************************
*                               SetPumpStartAutoAdjHookSwitch
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
void SetPumpStartAutoAdjHookSwitch(uint8_t i_NewStatu)
{
    g_HP_PumpRunStartAutoAdjHookSw = i_NewStatu;
}

/*
***************************************************************************************************
*                               StackHydrgPressHighEnoughWaitHook()
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
static void StackHydrgPressHighEnoughWaitHook(uint8_t i_u8WaitStatus)
{
    OS_ERR err;
    float fHydrgPress = 0;

    fHydrgPress = GetSrcAnaSig(HYDROGEN_PRESS_1);

    //等待气压达到电堆进入排杂状态
    if((fHydrgPress >= 50.0) && (WAIT_FOR_50KPA == i_u8WaitStatus)) {
		
        OSSemPost(&StackStartSem, OS_OPT_POST_1, &err);
        g_u8StackHydrgPressArriveWaitSw = DO_NOT_WAIT;
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
* Notes       : New alarm program.
***************************************************************************************************
*/
void StackAnaSigAlarmRunningMonitorHook(void)
{
#if STACK_TEMP_MONITOR_SWITCH
    float fStackTemp = 0;
    static uint8_t temp_high_cnt = 0;
    static uint8_t StackTempHighFlag = NO;
#endif

    float fHydrgPress = 0;
    static uint8_t FcPressHighFlag = NO;
    static uint8_t FcPressLowFlag = NO;
    static uint8_t g_FcPressOver75KPaHoldSec = 0;
    static uint8_t g_FcPressBelow10KPaHoldSec = 0;       //电堆气压小于10Kpa的秒数
#if STACK_VOLETAGE_MONITOR_SWITCH
    float fStackVoletage = 0;
    static uint8_t StackVoletageLowFlag = NO;
    static uint16_t g_u16StackVoletageBelow40VHoldSeconds = 0;
    static uint16_t g_u16StackVoletageExceed40VHoldSeconds = 0;
#endif

    if(EN_SHUTTING_DOWN != GetSystemWorkStatu()) {
#if STACK_TEMP_MONITOR_SWITCH
        /* 监测温度 */
        fStackTemp = GetSrcAnaSig(STACK_TEMP);

        if(fStackTemp > 60) {
            AlarmCmd(STACK_TEMP_HIGH_ALARM, GENERAL_GRADE, ON);

            if(fStackTemp > 62) {
                temp_high_cnt ++;
                if((temp_high_cnt > 30) && (StackTempHighFlag != YES)){
                    BSP_DCConnectValvePwrOff();
//                    CmdShutDown();      //关机命令
                    StackTempHighFlag = YES;
                    APP_TRACE_INFO(("Stack temp is above the high temp protect line,stop output...\n\r"));
                }
            }
        } else {
            if((fStackTemp >= 20) && (fStackTemp <= 55)) {//系统恢复到正常温度,恢复输出
                if(StackTempHighFlag == YES) {
                    BSP_DCConnectValvePwrOn();
                    StackTempHighFlag = NO;
                }
                AlarmCmd(STACK_TEMP_HIGH_ALARM, GENERAL_GRADE, OFF);
                AlarmCmd(STACK_TEMP_LOW_ALARM, GENERAL_GRADE, OFF);
            }
        }

#endif

#if STACK_AIR_PRESS_MONITOR_SWITCH
        /* 监测气压 */
        fHydrgPress = GetSrcAnaSig(HYDROGEN_PRESS_1);

        if(fHydrgPress >= 75){
            if(FcPressHighFlag != YES){
                g_FcPressOver75KPaHoldSec ++;
                if(g_FcPressOver75KPaHoldSec >= 30) {
//                    CmdShutDown();      //关机命令
                    FcPressHighFlag = YES;
                    g_FcPressOver75KPaHoldSec = 0;
                    APP_TRACE_INFO(("Stack hydrogen press to high so shut down it...\n\r"));
                }
            }

        }else if(fHydrgPress >= 70) {
            g_FcPressOver75KPaHoldSec = 0;
            AlarmCmd(HYDROGEN_PRESS_HIGH_ALARM, SERIOUS_GRADE, ON);
        }else if(fHydrgPress >= 50) {
//            APP_TRACE_INFO(("Stack hydrogen press resume...\n\r"));
            AlarmCmd(HYDROGEN_PRESS_HIGH_ALARM, SERIOUS_GRADE, OFF);
            AlarmCmd(HYDROGEN_PRESS_LOW_ALARM, SERIOUS_GRADE, OFF);
            FcPressHighFlag = NO;
            FcPressLowFlag = NO;
        }else if(fHydrgPress >= 30) {
//            AlarmCmd(HYDROGEN_PRESS_LOW_ALARM, SERIOUS_GRADE, ON);
            g_FcPressBelow10KPaHoldSec = 0;
        }else {
//            AlarmCmd(HYDROGEN_PRESS_LOW_ALARM, SERIOUS_GRADE, ON);
            if(FcPressLowFlag != YES){
                g_FcPressBelow10KPaHoldSec ++;
                if(g_FcPressBelow10KPaHoldSec >= 20) {//气压小于20超过3s,电堆停止输出
                    FcPressLowFlag = YES;
                    g_FcPressBelow10KPaHoldSec = 0;
                    APP_TRACE_INFO(("Stack hydrogen press is below the low temp protect line...\n\r"));
//                    CmdShutDown();      //关机命令
                }
            }
        }


#endif

#if STACK_VOLETAGE_MONITOR_SWITCH
        /*监测电压*/
        fStackVoletage = GetSrcAnaSig(STACK_VOLTAGE);

        if(EN_IN_WORK == GetStackWorkStatu()) {
            if(fStackVoletage >= 39) {
                g_u16StackVoletageBelow40VHoldSeconds = 0;
                AlarmCmd(STACK_VOLTAGE_LOW_ALARM, GENERAL_GRADE, OFF);

                if(StackVoletageLowFlag != NO) {
                    g_u16StackVoletageExceed40VHoldSeconds ++;

                    if(g_u16StackVoletageExceed40VHoldSeconds >= 300) {
                        BSP_DCConnectValvePwrOn();
                        StackVoletageLowFlag = NO;
                        g_u16StackVoletageExceed40VHoldSeconds = 0;
                    }
                }
            } else {
                g_u16StackVoletageBelow40VHoldSeconds ++;

                if(g_u16StackVoletageBelow40VHoldSeconds > 300) { //电压低于39V超过3S停止输出
                    AlarmCmd(STACK_VOLTAGE_LOW_ALARM, GENERAL_GRADE, ON);
                    StackVoletageLowFlag = YES;
                    BSP_DCConnectValvePwrOff();
                    g_u16StackVoletageBelow40VHoldSeconds = 0;
                }
            }
        }

#endif
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
*                               SetHydrogenPressArrivedWaitSwitch
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
void  SetHydrogenPressArrivedWaitSwitch(uint8_t i_WaitStatus)
{
    g_u8StackHydrgPressArriveWaitSw = i_WaitStatus;
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
    OS_ERR err;
    float fStackVoltage;
    float fStackCurrent;
    static uint16_t u16RestartLimitCurrentCount = 0;   

	if((GetRestartLimitCurrentFlagStatus() != DEF_SET) && (GetInShortControlFlagStatus() != DEF_SET)){
		u16RestartLimitCurrentCount ++;			
		if(u16RestartLimitCurrentCount >= 100){//每10秒监测一次是否需要开始重新限流
						
			fStackVoltage = GetSrcAnaSig(STACK_VOLTAGE);
			fStackCurrent = GetSrcAnaSig(STACK_CURRENT);
			
			if((fStackVoltage >= 55.0) && (fStackCurrent <= 5.0)) {
				SetRestartLimitCurrentFlagStatus(DEF_SET);
				SetDCModuleLimitCurrentSmoothlyTaskSwitch(DEF_ENABLED);//打开平滑限流开关
				OSTaskResume(&DCLimitCurrentSmoothlyTaskTCB, &err);//恢复平滑限流任务
			}
			u16RestartLimitCurrentCount = 0;	
		}
	}
}
/*
***************************************************************************************************
*                               GetRestartLimitCurrentFlagStatus()
*
* Description : Start beep alarm by alarm grade.
*
* Arguments   : none.
*
* Returns     : none.
*
* Notes       : none.
***************************************************************************************************
*/

uint8_t GetRestartLimitCurrentFlagStatus(void)
{
    return g_u8RestartLimitCurrentFlag;
}
void SetRestartLimitCurrentFlagStatus(uint8_t i_NewStatu)
{
	g_u8RestartLimitCurrentFlag = i_NewStatu;
}
/*
***************************************************************************************************
*                               StartRunningAlarm()
*
* Description : Start beep alarm by alarm grade.
*
* Arguments   : none.
*
* Returns     : none.
*
* Notes       : none.
***************************************************************************************************
*/
void StartRunningBeepAlarm(SYSTEM_ALARM_GRADE_Typedef i_AlarmGrade, uint8_t i_u8NewStatus)
{
    static uint8_t  u8AlarmCount = 0;

    if(ON == i_u8NewStatus) {
        u8AlarmCount ++;

        if(i_AlarmGrade <= SLIGHT_GRADE) {
            if(u8AlarmCount >= 50) { //低等级报警为2s蜂鸣器响一次
                BSP_BuzzerTurnover();
                u8AlarmCount = 0;
            }
        } else if(i_AlarmGrade <= GENERAL_GRADE) {
            if(u8AlarmCount >= 30) {
                BSP_BuzzerTurnover();
                u8AlarmCount = 0;
            }
        } else {
            if(u8AlarmCount >= 10) { //严重等级报警
                BSP_BuzzerTurnover();
                u8AlarmCount = 0;
            }
        }
    } else {
        BSP_BuzzerOff();
    }
}
/******************* (C) COPYRIGHT 2016 Guangdong ENECO POWER *****END OF FILE****/
