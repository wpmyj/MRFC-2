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
#include "app_stack_short_circuit_task.h"
#include "bsp_dc_module_adjust.h"
/*
***************************************************************************************************
*                                           MACRO DEFINITIONS
***************************************************************************************************
*/
#define STACK_MANAGER_TASK_STK_SIZE                             128
#define STACK_MANAGER_DELAY_STOP_TASK_STK_SIZE                  100
#define STACK_HYDROGEN_YIELD_MATCHING_OFFSET_VALUE_MONITOR_TASK_STK_SIZE     256


#define  STACK_FAN_CONTROL_CYCLE   8  //风机控制周期
/*
***************************************************************************************************
*                                         OS-RELATED    VARIABLES
***************************************************************************************************
*/

OS_TCB      StackManagerTaskTCB;
OS_TCB      StackManagerDlyStopTaskTCB;
OS_TCB      StackHydrogenYieldMatchingOffsetValueMonitorTaskTCB;

			OS_SEM                    StackStartSem;
static      OS_SEM                    StackManagerStopSem;

static      CPU_STK_8BYTE_ALIGNED     StackManagerTaskStk[STACK_MANAGER_TASK_STK_SIZE];
static      CPU_STK                   StackManagerDlyStopTaskStk[STACK_MANAGER_DELAY_STOP_TASK_STK_SIZE];
static      CPU_STK_8BYTE_ALIGNED     StackHydrogenYieldMatchingOffsetValueMonitorStk[STACK_HYDROGEN_YIELD_MATCHING_OFFSET_VALUE_MONITOR_TASK_STK_SIZE];

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
static  float                           g_fHydrogenYieldMatchOffsetValue = 0.0;//电堆匹氢偏移值
static  uint8_t                         g_u8StackFanPidControlSw = DEF_DISABLED;//电堆风扇自动调速开关
static  uint8_t                         g_u8StackHydrogenYieldMatchingOffsetValueMonitorTaskSw = DEF_DISABLED;
static  uint8_t                         g_u8DecompressCountPerMinute = 0;

/*
***************************************************************************************************
*                                         FUNCTION PROTOTYPES
***************************************************************************************************
*/
static      void        StackManagerTask(void);
static      void        StackManagerDlyStopTask(void);
static      void        StackHydrogenYieldMatchingOffsetValueMonitorTask(void);

static      void        SetStackFanSpdPidControlSwitch(uint8_t);
static      void        SetStackHydrogenYieldMatchingOffsetValueMonitorTaskSwitch(uint8_t i_NewStatu);

static      void        StackManagerTask(void);
static      void        SetStackFanSpdPidControlSwitch(uint8_t);

static      void        StackManagerDlyStopTask(void);
static 		void 		ResetMonitorTaskSwitch(void);
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

	OSSemCreate(&StackStartSem, "Stack manager sem", 0, &err);
	
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
    uint16_t    u16StackFanSpeed = 0;
    static      uint16_t u16AmpIntegralSum = 0;
	static		uint16_t short_congtol_cnt = 0;
    float       fCurrent = 0.0;
    float       fOptimumStackTemp = 0.0;

    while(DEF_TRUE) {
		
		APP_TRACE_INFO(("Suspend stack manager task ...\r\n"));
        OSTaskSuspend(NULL, &err);
 
		while(DEF_TRUE) {
			
			APP_TRACE_INFO(("Get into the stack manager task ...\r\n"));
			BSP_HydrgInValvePwrOn();
			SetDcModeOutPutNominalVoltageButDifferentCurrent(HOLD_LIMIT_POINT);

			APP_TRACE_INFO(("Stack manager start, waitting the hydrogen press up to 36KPa ...\r\n"));
			SetHydrogenPressArrivedWaitSwitch(WAIT_FOR_36KPA);
			OSTaskSemPend(0, OS_OPT_PEND_BLOCKING, NULL, &err);//接收模拟信号监测任务中气压监测任务信号量
			APP_TRACE_INFO(("Start the stack start purify...\r\n"));
			SetStackFanCtrlSpd(1000);
			BSP_HydrgOutValvePwrOn();
			OSTimeDlyHMSM(0, 0, 3, 0, OS_OPT_TIME_HMSM_STRICT, &err);
			SetStackFanCtrlSpd(200);
			OSTimeDlyHMSM(0, 0, 2, 0, OS_OPT_TIME_HMSM_STRICT, &err);
			BSP_HydrgOutValvePwrOff();
			APP_TRACE_INFO(("Finish the stack start purify...\r\n"));

			APP_TRACE_INFO(("Stack manager start, waitting the hydrogen press up to 45KPa ...\n\r"));
			SetHydrogenPressArrivedWaitSwitch(WAIT_FOR_45KPA);
			
			OSSemPend(&StackStartSem,//等待接收启动阶段信号量:气压达到和关机都能结束
                      0,
                      OS_OPT_PEND_BLOCKING,
                      NULL,
                      &err);
			if(err == OS_ERR_NONE) {
                if(EN_SHUTTING_DOWN == GetSystemWorkStatu()) {
                    APP_TRACE_INFO(("Fuel cell not start and froce to stop...\n\r"));
                    break;
                } else {
                    APP_TRACE_INFO(("Fuel cell wait hydrogen press arrived...\n\r"));
                }
            }

			BSP_DCConnectValvePwrOn();
			StackWorkTimesInc();
			IncrementType_PID_Init();   //PID参数初始化
			SetStackWorkStatu(EN_IN_WORK);
			SetExternalScreenUpdateStatu(YES);//串口显示屏界面更新

			StackHydrogenYieldMatchingOffsetValueMonitorTaskCreate();//电堆匹氢偏移量监测任务创建

			SetStackIsPulledStoppedMonitorHookSwitch(DEF_ENABLED);//监测电堆是否被拉停
			SetStackHydrogenYieldMatchingOffsetValueMonitorTaskSwitch(DEF_ENABLED);
			SetDCModuleLimitCurrentSmoothlyTaskSwitch(DEF_ENABLED);//平滑限流
			SetStackAnaSigAlarmRunningMonitorHookSwitch(DEF_ENABLED);
			SetStackFanSpdPidControlSwitch(DEF_ENABLED);

			OSTaskResume(&StackHydrogenYieldMatchingOffsetValueMonitorTaskTCB,&err);  //恢复匹氢偏移监测任务                                 
			OSTaskResume(&DCLimitCurrentSmoothlyTaskTCB,&err);  //恢复平滑限流任务
						 
			while(DEF_TRUE) {

				OSSemPend(&StackManagerStopSem,
						  OS_CFG_TICK_RATE_HZ,
						  OS_OPT_PEND_BLOCKING,
						  NULL,
						  &err);

				if(err == OS_ERR_NONE) {
					break;
				}

				//安培秒排气
				fCurrent = GetSrcAnaSig(STACK_CURRENT);
				u16AmpIntegralSum += (uint16_t)fCurrent;

				if(u16AmpIntegralSum >= g_u16RunPurifyAmpIntegralValue) {
					BSP_HydrgOutValvePwrOn();
					OSTimeDlyHMSM(0, 0, 0, 500, OS_OPT_TIME_HMSM_STRICT, &err);
					BSP_HydrgOutValvePwrOff();
					u16AmpIntegralSum = 0;
				}

				//风机PID控制程序
				if(EN_CONTROL_MODE_AUTO == GetControlMode()) {

					if(g_u8StackFanPidControlSw == DEF_ENABLED) {
						IPID.CalcCycleCnt++;

						if(IPID.CalcCycleCnt >= IPID.Tsam) {
							fOptimumStackTemp = CalcStackOptimumTemperatureByCurrent();//计算最佳温度
							u16StackFanSpeed = IncrementType_PID_Process(fOptimumStackTemp);
							SetStackFanCtrlSpd(u16StackFanSpeed);
							IPID.CalcCycleCnt = 0;
						}
					}
				}
					
				//保持限流不中断
                short_congtol_cnt ++;
                if(short_congtol_cnt >= 50) {
                    SetDcModeOutPutNominalVoltageButDifferentCurrent(g_fIvalueNow);
                    short_congtol_cnt = 0;
                }
			}
			APP_TRACE_INFO(("Stack manager stop...\n\r"));
			break;//再次break挂起电堆管理任务
		}
		ResetMonitorTaskSwitch();
	}
}

/*
***************************************************************************************************
*                          ResetMonitorAndSubTaskSwitch()
*
* Description:  Enable or Disable the stack fan auto adjust.
*
* Arguments  :  none
*
* Returns    :  none
***************************************************************************************************
*/
static void ResetMonitorTaskSwitch(void)
{
    SetStackHydrogenYieldMatchingOffsetValueMonitorTaskSwitch(DEF_DISABLED);
	SetStackAnaSigAlarmRunningMonitorHookSwitch(DEF_DISABLED);
	SetDCModuleLimitCurrentSmoothlyTaskSwitch(DEF_DISABLED);
	SetStackIsPulledStoppedMonitorHookSwitch(DEF_DISABLED);
}
/*
***************************************************************************************************
*                            SetStackFanSpdPidControlSwitch()
*
* Description:  Enable or Disable the stack fan auto adjust.
*
* Arguments  :  none
*
* Returns    :  none
***************************************************************************************************
*/
void SetStackFanSpdPidControlSwitch(uint8_t i_NewStatu)
{
    g_u8StackFanPidControlSw = i_NewStatu;
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
        APP_TRACE_INFO(("The stack manager start to delay stop...\n\r"));

        g_eStackManagerStopDlyStatu = ON;
		OSSemPost(&StackStartSem, OS_OPT_POST_1, &err);//电堆未正常开启需要发送信号量跳出循环
		OSSemPost(&StackManagerStopSem, OS_OPT_POST_1, &err);//使电堆管理任务挂起
		OSTaskSemPost(&StackRunningShortTaskTCB,OS_OPT_POST_NO_SCHED,&err);//使运行短路活化任务挂起
        SetStackFanSpdPidControlSwitch(DEF_DISABLED);  //关闭风机自动调速

        SetStackFanCtrlSpd(1000);

        while(DEF_TRUE) {
            OSTimeDlyHMSM(0, 0, 1, 0,
                          OS_OPT_TIME_HMSM_STRICT,
                          &err);
            u16ShutDownStackFanDlySeconds ++;

            if(u16ShutDownStackFanDlySeconds == 5) { //关闭直流接触器,停止输出，排气三秒
                BSP_DCConnectValvePwrOff();
				BSP_HydrgOutValvePwrOn();
            } else if(u16ShutDownStackFanDlySeconds == 8) { //关闭进气电磁阀
                BSP_HydrgInValvePwrOff();
				BSP_HydrgOutValvePwrOff();
            } else if(u16ShutDownStackFanDlySeconds >= 30) {//电堆风扇延时30S后关闭，同时挂起电堆停止任务

                OSSemPost(&StackManagerStopSem,
                          OS_OPT_POST_1,
                          &err);
                OSTaskSemPost(&AppTaskStartTCB, //发送给主任务内的shutdown函数任务信号量响应半机2电堆延时关闭任务
                              OS_OPT_POST_NO_SCHED,
                              &err);
                u16ShutDownStackFanDlySeconds = 0;

                APP_TRACE_INFO(("The stack delay stop time arrive 30s,task sem send...\n\r"));
                break;
            } else {}
        }

        BSP_HydrgInValvePwrOff();
        BSP_HydrgOutValvePwrOff();

        SetStackWorkStatu(EN_NOT_IN_WORK);
        SetStackFanCtrlSpd(0);

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
    APP_TRACE_INFO(("Created the Stack V-hymo value monitor task, and err code is %d...\n\r", err));
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
    float   fAmpIntegralValue = 0.0; //安培时间积分值
    float   fActualHydrogenMatchValue = 0.0;

    while(DEF_TRUE) {

        OSTaskSuspend(NULL, &err);
        APP_TRACE_INFO(("Resume Stack V-hymo value monitor task...\n\r"));

        while(DEF_TRUE) {

            if(g_u8StackHydrogenYieldMatchingOffsetValueMonitorTaskSw == DEF_DISABLED) {
                APP_TRACE_INFO(("Stack V-hymo value monitor task break...\n\r"));
                break;
            }

            OSTaskSemPend(0,   //接收泄压阀输入脉冲中断中的时间参数记录完成任务信号量
                          OS_OPT_PEND_BLOCKING,
                          NULL,
                          &err);
            {
                //排气次数
                g_u8DecompressCountPerMinute = (uint8_t)(60.0 / ((StackVentAirTimeParameter.fVentAirTimeIntervalValue + StackVentAirTimeParameter.fDecompressVentTimeValue) * 0.001));

                //安培时间积分值计算
                fCurrent = GetSrcAnaSig(STACK_CURRENT);
                fAmpIntegralValue = (StackVentAirTimeParameter.fVentAirTimeIntervalValue * 0.001) * fCurrent;

                //实际匹氢值计算
                fActualHydrogenMatchValue = fAmpIntegralValue / (StackVentAirTimeParameter.fDecompressVentTimeValue * 0.001);

                //匹氢偏移值ln(600/ActualHydrogenMatchValue)
                g_fHydrogenYieldMatchOffsetValue = log(600 / fActualHydrogenMatchValue); //600为测试值A0

//                APP_TRACE_INFO(("-->RTDCPM-DCPM:-#%3d   -$%3d   \n\r", g_u8RealTimeDecompressCountPerMinute,g_u8DecompressCountPerMinute));
//                APP_TRACE_INFO(("-->Current-VentingInterval-DecompressTime:-#%.3f   -$%.3f   -&%.3f   \n\r", \
//                                fCurrent, StackVentAirTimeParameter.fVentAirTimeIntervalValue * 0.001, StackVentAirTimeParameter.fDecompressVentTimeValue * 0.001));
                //匹氢值小于标准的0.3倍(偏移值为正)，产气充足，并且电堆电压等参数正常,可提高输出功率
                if((g_fHydrogenYieldMatchOffsetValue > 1.50) && (GetSrcAnaSig(STACK_VOLTAGE) > 41.0)) {
                    //限流点上升和下降都采用逐次周期增加

                    //匹氢值大于标准的1.5倍，产气不足,限制输出功率(限流限压)
                } else if((g_fHydrogenYieldMatchOffsetValue < -0.40) || ((GetSrcAnaSig(STACK_VOLTAGE) <= 41.0))) {
                    
                } else {
                    //气量合适,清不正常计数值
                }
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
*                            GetPassiveDecompressCountPerMinutes()
*
* Description:  电堆尾部泄压排气次数.
*
* Arguments  :  none
*
* Returns    :  none
***************************************************************************************************
*/
uint8_t GetPassiveDecompressCountPerMinutes(void)
{
    return g_u8DecompressCountPerMinute;
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



/******************* (C) COPYRIGHT 2015 Guangdong Hydrogen *****END OF FILE****/
