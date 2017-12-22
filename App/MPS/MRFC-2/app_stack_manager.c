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
* Programmer(s) : JasonFan
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
#include "bsp_delay_task_timer.h"
#include "app_stack_short_circuit_task.h"
#include "bsp_dc_module_adjust.h"
#include "bsp.h"
/*
***************************************************************************************************
*                                           MACRO DEFINITIONS
***************************************************************************************************
*/
#define STACK_MANAGER_TASK_STK_SIZE                             				128
#define STACK_MANAGER_DELAY_STOP_TASK_STK_SIZE                  			 	100
#define STACK_HYDROGEN_YIELD_MATCHING_OFFSET_VALUE_MONITOR_TASK_STK_SIZE     	256
#define STACK_START_UP_CRTL_TASK_STK_SIZE                                    	100


#define  STACK_FAN_CONTROL_CYCLE   8  //风机控制周期
/*
***************************************************************************************************
*                                         OS-RELATED    VARIABLES
***************************************************************************************************
*/

OS_TCB      StackManagerTaskTCB;
OS_TCB      StackManagerDlyStopTaskTCB;
OS_TCB      StackStartUpCtrlTaskTCB;

			OS_SEM                    StackStartSem;
static      OS_SEM                    StackManagerStopSem;
static      OS_SEM                    StackStartUpShortCtrlFinishedSem;

static      CPU_STK_8BYTE_ALIGNED     StackManagerTaskStk[STACK_MANAGER_TASK_STK_SIZE];
static      CPU_STK                   StackManagerDlyStopTaskStk[STACK_MANAGER_DELAY_STOP_TASK_STK_SIZE];
static      CPU_STK                   StackStartUpCtrlTaskStk[STACK_START_UP_CRTL_TASK_STK_SIZE];

/*
***************************************************************************************************
*                                          GLOBAL VARIABLES
***************************************************************************************************
*/

SWITCH_TYPE_VARIABLE_Typedef            g_eStackManagerStopDlyStatu = OFF;
uint8_t                                 g_u8DecompressCountPerMinute = 0;
/*
***************************************************************************************************
*                                           LOCAL VARIABLES
***************************************************************************************************
*/
static  uint8_t                         g_u8StackFanPidControlSw = DEF_DISABLED;//电堆风扇自动调速开关

static  uint8_t                         g_u8StackStartUpShortTaskSw = DEF_ENABLED;
/*
***************************************************************************************************
*                                         FUNCTION PROTOTYPES
***************************************************************************************************
*/
static      void        StackManagerTask(void);
static      void        StackManagerDlyStopTask(void);

static      void        StackManagerTask(void);

static      void        StackManagerDlyStopTask(void);
static 		void 		ResetMonitorTaskSwitch(void);
static      void        StackStartUpCtrlTask(void);
static		uint8_t 	PendSemByKeepWaiting(OS_SEM   *p_sem);
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
			
			APP_TRACE_INFO(("Start the stack manager task ...\r\n"));
			BSP_HydrgInValvePwrOn();
			SetDcOutPutCurrentLimitPoint(HOLD_LIMIT_POINT);

			APP_TRACE_INFO(("Waitting the hydrogen press up to 45KPa ...\r\n"));
			SetHydrogenPressArrivedWaitSwitch(WAIT_FOR_45KPA);
			if(DEF_TRUE != PendSemByKeepWaiting(&StackStartSem)){
				APP_TRACE_INFO(("Stack start sem pend err...\r\n"));
				break;
			}
			
			APP_TRACE_INFO(("Start the stack start purify...\r\n"));
			SetStackFanCtrlSpd(1000);
			BSP_HydrgOutValvePwrOn();
			OSTimeDlyHMSM(0, 0, 3, 0, OS_OPT_TIME_HMSM_STRICT, &err);
			SetStackFanCtrlSpd(200);
			OSTimeDlyHMSM(0, 0, 2, 0, OS_OPT_TIME_HMSM_STRICT, &err);
			BSP_HydrgOutValvePwrOff();
			APP_TRACE_INFO(("Finish the stack start purify...\r\n"));
						
			BSP_DCConnectValvePwrOn();
			StackWorkTimesInc();
			IncrementType_PID_Init();   //PID参数初始化
			SetStackWorkStatu(EN_IN_WORK);
			StartTimerDelayTask(UPDATE_DECOMPRESS_CNT_EVER_1MIN,60000);//开始排气次数监测,一分钟更新一次

			SetStackIsPulledStoppedMonitorHookSwitch(DEF_ENABLED);//监测电堆是否被拉停			
			SetStackAnaSigAlarmRunningMonitorHookSwitch(DEF_ENABLED);
			SetStackFanSpdPidControlSwitch(DEF_ENABLED);		
			SetStackStartUpShortTaskSwitch(DEF_ENABLED);            

            OSTaskResume(&StackStartUpCtrlTaskTCB, &err);//开始启动阶段短路以及风机控制流程

			if(DEF_TRUE != PendSemByKeepWaiting(&StackStartUpShortCtrlFinishedSem)){
				APP_TRACE_INFO(("Stack start up short ctrl finished sem pend err...\r\n"));
				break;
			}
			
			SetDCModuleLimitCurrentSmoothlyTaskSwitch(DEF_ENABLED);//打开平滑限流开关
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
                    SetDcOutPutCurrentLimitPoint(g_fIvalueNow);
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
	SetStackAnaSigAlarmRunningMonitorHookSwitch(DEF_DISABLED);
	SetDCModuleLimitCurrentSmoothlyTaskSwitch(DEF_DISABLED);
	SetStackIsPulledStoppedMonitorHookSwitch(DEF_DISABLED);
}


/*
***************************************************************************************************
*                          PendSemByKeepWaiting()
*
* Description:  Pend the specified sem by keep waiting.
*
* Arguments  :  none
*
* Returns    :  none
***************************************************************************************************
*/
uint8_t PendSemByKeepWaiting(OS_SEM   *p_sem)
{
	OS_ERR      err;

	OSSemPend(p_sem,//一直等待信号量
			  0,
			  OS_OPT_PEND_BLOCKING,
			  NULL,
			  &err);
	if(err == OS_ERR_NONE) {
		if(EN_SHUTTING_DOWN == GetSystemWorkStatu()) {
			return DEF_FALSE;
		} 
	}else{
		return	DEF_FALSE;
	}
	
	return DEF_TRUE;
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

        OSTaskSuspend(NULL,&err);
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
*                        StackStartUpCtrlTaskCreate()
*
* Description:  Create the cycle vent task.
*
* Arguments  :  none
*
* Returns    :  none
***************************************************************************************************
*/
void StackStartUpCtrlTaskCreate(void)
{
    OS_ERR  err;
    OSTaskCreate((OS_TCB *)&StackStartUpCtrlTaskTCB,
                 (CPU_CHAR *)"Stack cycle vent task",
                 (OS_TASK_PTR) StackStartUpCtrlTask,
                 (void *) 0,
                 (OS_PRIO) STACK_START_UP_CRTL_TASK_PRO,
                 (CPU_STK *)&StackStartUpCtrlTaskStk[0],
                 (CPU_STK_SIZE) STACK_START_UP_CRTL_TASK_STK_SIZE / 10,
                 (CPU_STK_SIZE) STACK_START_UP_CRTL_TASK_STK_SIZE,
                 (OS_MSG_QTY) 5u,
                 (OS_TICK) 0u,
                 (void *) 0,
                 (OS_OPT)(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR *)&err);
    APP_TRACE_INFO(("Create stack start up task, err code is %d...\n\r", err));

    OSSemCreate(&StackStartUpShortCtrlFinishedSem, "StackStartUpShortCtrlFinishedSem...", 0, &err);
}


/*
***************************************************************************************************
*                        StackStartUpCtrlTask()
*
* Description:  The cycle vent task.
*
* Arguments  :  none
*
* Returns    :  none
***************************************************************************************************
*/
static void StackStartUpCtrlTask(void)
{
    OS_ERR      err;
    static     uint8_t    st_StartUpShortCtrlFinishedFlag = DEF_NO;
    static     uint8_t    u8StartUpShortCtrlTimeCnt = 0;
    static     uint8_t    u8ShortCtrlTimeCnt = 0;
    static     uint8_t    u8FanRiseAndFallCtrlTimeCnt = 0;
    static     uint8_t    u8FanCtrlHoldTimeCnt = 0;

    while(DEF_TRUE) {

		APP_TRACE_INFO(("Stack startup control task suspend...\n\r"));
        OSTaskSuspend(NULL, &err);
		st_StartUpShortCtrlFinishedFlag = DEF_NO;
        APP_TRACE_INFO(("Resume stack startup control task...\n\r"));
        
        while(DEF_TRUE) {

            if((g_u8StackStartUpShortTaskSw == DEF_DISABLED) || (EN_SHUTTING_DOWN == GetSystemWorkStatu())) {
				//强制中断时清零计数值
				OSSemPost(&StackStartUpShortCtrlFinishedSem, OS_OPT_POST_1, &err);
				st_StartUpShortCtrlFinishedFlag = DEF_NO;
				u8FanRiseAndFallCtrlTimeCnt = 0;
				u8FanCtrlHoldTimeCnt = 0;
				u8StartUpShortCtrlTimeCnt = 0;
                APP_TRACE_INFO(("Stack start up control task break...\n\r"));
                break;
            }

            OSTimeDlyHMSM(0, 0, 1, 0, OS_OPT_TIME_HMSM_STRICT, &err);

            u8FanRiseAndFallCtrlTimeCnt ++;

            if(u8FanRiseAndFallCtrlTimeCnt <= 240) { //持续4分钟
                u8FanCtrlHoldTimeCnt ++;

                if(u8FanCtrlHoldTimeCnt == 10) { //风机跳跃控制，高速持续5s，低速持续10s
                    SetStackFanCtrlSpd(STACK_FAN_SPOOL_UP_SPD);
                    SetDcOutPutCurrentLimitPoint(HOLD_LIMIT_POINT);
                } else if(u8FanCtrlHoldTimeCnt == 15) {
                    SetStackFanCtrlSpd(STACK_FAN_SPOOL_DOWN_SPD);
                    u8FanCtrlHoldTimeCnt = 0;
                } else {}
            } else {
                u8FanRiseAndFallCtrlTimeCnt = 0;
                u8StartUpShortCtrlTimeCnt = 0;
                SetStackFanSpdPidControlSwitch(DEF_ENABLED);
				OSSemPost(&StackStartUpShortCtrlFinishedSem, OS_OPT_POST_1, &err);//4分钟后才恢复平滑限流
                break;
            }

            //电堆启动阶段短路活化控制
            if(st_StartUpShortCtrlFinishedFlag != DEF_YES) {
                u8StartUpShortCtrlTimeCnt ++;

                if(u8StartUpShortCtrlTimeCnt == 119) {
                    BSP_DcOutPutValvePwrOn();
                }

                if(u8StartUpShortCtrlTimeCnt >= 120) {
                    if((u8StartUpShortCtrlTimeCnt % 10) == 0) {
                        if(DEF_TRUE == StackShortCtrl()) {
                            u8ShortCtrlTimeCnt ++;
							APP_TRACE_INFO(("Startup %d short ctrl...\n\r",u8ShortCtrlTimeCnt));

                            if(u8ShortCtrlTimeCnt >= 5) {
                                APP_TRACE_INFO(("Startup short ctrl finished...\n\r"));
                                u8ShortCtrlTimeCnt = 0;
                                BSP_DcOutPutValvePwrOff();
                                st_StartUpShortCtrlFinishedFlag = DEF_YES;
                            }
                        } else {
                            st_StartUpShortCtrlFinishedFlag = DEF_YES;
                        }
                    }
                }
            } else {
                u8StartUpShortCtrlTimeCnt = 0; 
            }
        }
    }
}




/*
***************************************************************************************************
*                            GetPassiveDecompressCntPerMin()
*
* Description:  电堆尾部泄压排气次数.
*
* Arguments  :  none
*
* Returns    :  none
***************************************************************************************************
*/
uint8_t GetPassiveDecompressCntPerMin(void)
{
    return g_u8DecompressCountPerMinute;
}

void UpdatePassiveDecompressCntPerMin(void)
{
	g_u8DecompressCountPerMinute = GetPassiveDecompressCnt();
	ResetPassiveDecompressCnt();//清零中断排气计数值
}
/*
***************************************************************************************************
*                            SetStackStartUpShortTaskSwitch()
*
* Description:  Enable or Disable the stack short control switch.
*
* Arguments  :  none
*
* Returns    :  none
***************************************************************************************************
*/
void SetStackStartUpShortTaskSwitch(uint8_t i_NewStatu)
{
    g_u8StackStartUpShortTaskSw = i_NewStatu;
}

uint8_t GetStackStartUpShortTaskSwitchStatus()
{
    return g_u8StackStartUpShortTaskSw;
}

/******************* (C) COPYRIGHT 2016 Guangdong ENECO POWER *****END OF FILE****/
