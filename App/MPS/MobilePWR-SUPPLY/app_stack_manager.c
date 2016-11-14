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
#include "bsp_pvd.h"
/*
*********************************************************************************************************
*                                           MACRO DEFINITIONS
*********************************************************************************************************
*/
#define STACK_MANAGER_TASK_STK_SIZE                             100
#define STACK_MANAGER_DELAY_STOP_TASK_STK_SIZE                  100

/*
*********************************************************************************************************
*                                         OS-RELATED    VARIABLES
*********************************************************************************************************
*/
extern      OS_TCB      AppTaskStartTCB;
//extern      OS_TCB      RS485CommWithUart5TaskTCB ;

OS_TCB      StackManagerTaskTCB;
OS_TCB      StackManagerDlyStopTaskTCB;

static      OS_SEM      StackManagerStopSem;
static      CPU_STK     StackManagerTaskStk[STACK_MANAGER_TASK_STK_SIZE];
static      CPU_STK     StackManagerDlyStopTaskStk[STACK_MANAGER_DELAY_STOP_TASK_STK_SIZE];

/*
*********************************************************************************************************
*                                           LOCAL VARIABLES
*********************************************************************************************************
*/
SWITCH_TYPE_VARIABLE_Typedef    g_eStackManagerStopDlyStatu = OFF;
uint8_t             g_u8StackFanAutoAdjSw = DEF_ENABLED;            //电堆风扇自动调速开关

u8  ChangePWM = 30;
u8  Last_ChangePWM = 0;
/*
*********************************************************************************************************
*                                         FUNCTION PROTOTYPES
*********************************************************************************************************
*/
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
    float       fCurrent, fVoltage,fStackTemp;
    float       fAmpIntegralSum;

    while(DEF_TRUE)
    {
        fAmpIntegralSum = 0;
  //      OSTaskSuspend(NULL, &err);
        StackWorkTimesInc();
        BSP_HydrgInValvePwrOn();
        APP_TRACE_INFO(("Stack manager start, waitting the hydrogen press up to 45KPa ...\n\r"));

        /* waitting the hydrogen press up to 45KPa */
        SetStackHydrgPressHighEnoughHookSwitch(DEF_ENABLED);
        OSTaskSemPend(0, OS_OPT_PEND_BLOCKING, NULL, &err);
        SetStackHydrgPressHighEnoughHookSwitch(DEF_DISABLED);

        APP_TRACE_INFO(("Start the stack start purify...\n\r"));
        BSP_HydrgOutValvePwrOn();
        OSTimeDlyHMSM(0, 0, 5, 0, OS_OPT_TIME_HMSM_STRICT, &err);
        BSP_HydrgOutValvePwrOff();
        SetStackWorkStatu(EN_IN_WORK);
        APP_TRACE_INFO(("Finish the stack start purify...\n\r"));

 //       SetStackFanCtlSpd(50);
 
        fVoltage = GetSrcAnaSig(STACK_VOLTAGE);
        if( fVoltage >= 48)                           //设置为当气压达到45KPA的时候，电压操作48v打开直流接触器
        {
            BSP_DCConnectValvePwrOn();
//      OSTaskResume(&RS485CommWithUart5TaskTCB,  //RS485通信开始
//                      &err);
        }
  
        //........关闭断电保护继电器，可以用关机键关闭机器
        SetStackAnaSigAlarmRunningMonitorHookSwitch(DEF_ENABLED);
        SetStackFanSpdAutoAdjSwitch(DEF_ENABLED);
//        SetStackFan1RunningSpeedMonitorHookSwitch(DEF_ENABLED);
//        SetStackFan2RunningSpeedMonitorHookSwitch(DEF_ENABLED);

        while(DEF_TRUE)
        {
            OSSemPend(&StackManagerStopSem, OS_CFG_TICK_RATE_HZ, OS_OPT_PEND_BLOCKING, NULL, &err);

            if(err == OS_ERR_NONE)
            {
                break;
            }

            fCurrent = GetSrcAnaSig(STACK_CURRENT);
            fAmpIntegralSum += fCurrent;
          //  APP_TRACE_INFO(("the Vvalue is  %d...\n\r",GetCurrentPower() ));
            
            //累加值相当于化学反应中转移的电荷量(氢气的消耗量、水的生成量成正比关系),电堆排气以此为依据，2300是厂家测试值，根据氢气纯度会有所改变
            if( fAmpIntegralSum >= 2300 )
            {
                BSP_HydrgOutValvePwrOn();
                OSTimeDlyHMSM(0, 0, 0, 500, OS_OPT_TIME_HMSM_STRICT, &err);
                BSP_HydrgOutValvePwrOff();
                fAmpIntegralSum = 0;
               
            }

            if(g_u8StackFanAutoAdjSw == DEF_ENABLED)
            {
                if(GetControlMode() == EN_CONTROL_MODE_AUTO)
                {
                    fStackTemp = GetSrcAnaSig(STACK_TEMP);
                    if(fStackTemp >= 30)
                    {
                        if((ChangePWM - 2 >= fStackTemp) || (fStackTemp >= ChangePWM + 2)) //温度范围
                        {
                            ChangePWM = fStackTemp;

                            if(ChangePWM != Last_ChangePWM)
                            {
                                if(fStackTemp <= 60)
                                {
                                    Set_PWM(0, 200 - (3.33 * fStackTemp));
                                    Last_ChangePWM = ChangePWM;
                                    
                                }
                                else
                                {
                                    Set_PWM(0, 0); //最大速度
                                    Last_ChangePWM = ChangePWM;
                                    
                                }
                            }
                        }
                    }
                    else
                    {
                        ChangePWM = 29;

                        if(ChangePWM != Last_ChangePWM)
                        {
                            Set_PWM(0, 100); //2.5V
                            Last_ChangePWM = ChangePWM;
                           
                        }
                    }

//                    /* 计算温度 */
//                    float fStackBestTemp = 0.53 * fCurrent + 26;     // 电堆最佳温度 = 0.53 * 电堆电流 + 26
//                    float fStackLowestTemp = 0.53 * fCurrent + 6;    // 电堆最低温度 = 0.53 * 电堆电流 +  6
//                    float fStackHighestTemp = 0.34 * fCurrent + 52;  // 电堆最高温度 = 0.34 * 电堆电流 + 52
//                    
//                    /* 电堆风扇自动调速 */
//                    if(fStackTemp > fStackHighestTemp)
//                    {
//                        SetStackFanCtlSpd(200);
//                    }
//                    else if(fStackTemp < fStackLowestTemp)
//                    {
//                        SetStackFanCtlSpd(50);
//                    }
//                    else 
//                    {
//                        if(fStackTemp > fStackBestTemp)
//                        {
//                            StackFanSpdInc();
//                        }
//                        else if(fStackTemp < fStackBestTemp)
//                        {
//                            StackFanSpdDec();
//                        }
//                    }
//                    
//                    APP_TRACE_INFO(("EN_CONTROL_MODE_AUTO...\n\r"));

//                    if(fStackTemp <= 20)
//                    {
//                        SetStackFanCtlSpd(50);
//                    }
//                    else if(fStackTemp <= 55)
//                    {
//                        SetStackFanCtlSpd(50 + (uint8_t)((fStackTemp - 20) * 26 / 7));
//                    }
//                    else if(fStackTemp <= 70)
//                    {
//                        SetStackFanCtlSpd(180 + (uint8_t)((fStackTemp - 55) * 4 / 3));
//                    }
//                    else
//                    {
//                        SetStackFanCtlSpd(200);
//                    }
                }
                else
                {
                    APP_TRACE_INFO(("Stack fan speed manager mannual...\n\r"));
                }
            }
            else
            {}//否则由延时任务调节风速
        }

        SetStackAnaSigAlarmRunningMonitorHookSwitch(DEF_DISABLED);
//        SetStackFan1RunningSpeedMonitorHookSwitch(DEF_DISABLED);
//        SetStackFan2RunningSpeedMonitorHookSwitch(DEF_DISABLED);

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
    uint16_t    u16ShutDownHydrgPressBelow30KPaHoldSeconds = 0;
    uint16_t    u16ShutDownStackFanDlySeconds = 0;

    while(DEF_TRUE)
    {
        OSTaskSuspend( NULL, &err );
        APP_TRACE_INFO(("The stack manager start to delay stop...\n\r"));

        g_eStackManagerStopDlyStatu = ON;
        SetStackFanSpdAutoAdjSwitch( DEF_DISABLED );

        SetStackFanCtlSpd(0);

        u16ShutDownHydrgPressBelow30KPaHoldSeconds = 0;

        while(DEF_TRUE)
        {
            OSTimeDlyHMSM(0, 0, 1, 0, OS_OPT_TIME_HMSM_STRICT, &err);
            u16ShutDownStackFanDlySeconds++;

            //电堆风机延时达到三分钟
            if(u16ShutDownStackFanDlySeconds >= 180)
            {
                OSSemPost(&StackManagerStopSem, OS_OPT_POST_1, &err);
                break;
            }

            if( GetSrcAnaSig(HYDROGEN_PRESS_1) >= 3 )
            {
                u16ShutDownHydrgPressBelow30KPaHoldSeconds = 0;
            }
            else
            {
                u16ShutDownHydrgPressBelow30KPaHoldSeconds++;

                //氢气压力小于30Kpa的时间达30s
                if( u16ShutDownHydrgPressBelow30KPaHoldSeconds >= 30 )
                {
                    OSSemPost(&StackManagerStopSem, OS_OPT_POST_1, &err);
                    OSTaskSemPost(&AppTaskStartTCB, OS_OPT_POST_NO_SCHED, &err);
                    break;
                }
            }
        }

        BSP_HydrgInValvePwrOff();
        BSP_HydrgOutValvePwrOff();
        SetStackWorkStatu(EN_NOT_IN_WORK);
        SetStackFanCtlSpd(200);

        Bsp_PVD_ProtectStatuCmd(OFF);
        BSP_DCConnectValvePwrOff();
        g_eStackManagerStopDlyStatu = OFF;
    }
}
/*
*********************************************************************************************************
*                                   GET THE STACK STOP DELAY STATU
*
* Description:  Get the delay statu of the stack stop work.
*
* Arguments  :  none
*
* Returns    :  none
*********************************************************************************************************
*/
uint8_t GetStackStopDlyStatu(void)
{
    return g_eStackManagerStopDlyStatu;
}

/******************* (C) COPYRIGHT 2015 Guangdong Hydrogen *****END OF FILE****/
