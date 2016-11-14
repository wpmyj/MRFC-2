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
* Filename      : app_hydrg_producer_manager.c
* Version       : V1.00
* Programmer(s) : EHS
*                 DC
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
#include "app_hydrg_producer_manager.h"
#include "app_analog_signal_monitor_task.h"
#include "app_speed_control_device_monitor_task.h"
/*
*********************************************************************************************************
*                                           MACRO DEFINITIONS
*********************************************************************************************************
*/
#define HYDROGEN_PRODUCER_MANAGER_TASK_STK_SIZE                 100
#define HYDROGEN_PRODUCER_MANAGER_DLY_STOP_TASK_STK_SIZE        100
#define IGNITER_WORK_TASK_STK_SIZE                              100

/*
*********************************************************************************************************
*                                         OS-RELATED    VARIABLES
*********************************************************************************************************
*/
extern      OS_SEM      IgniteFirstBehindWaitSem;

extern      OS_SEM      IgniteSecondBehindWaitSem;
extern      OS_TCB      AppTaskStartTCB;

OS_TCB      HydrgProducerManagerTaskTCB;
OS_TCB      HydrgProducerManagerDlyStopTaskTCB;
OS_TCB      IgniterWorkTaskTCB;

static      OS_SEM      HydrgProducerManagerStopSem;

static      CPU_STK     HydrgProducerManagerTaskStk[HYDROGEN_PRODUCER_MANAGER_TASK_STK_SIZE];
static      CPU_STK     HydrgProducerManagerDlyStopTaskStk[HYDROGEN_PRODUCER_MANAGER_DLY_STOP_TASK_STK_SIZE];
static      CPU_STK     IgniterWorkStk[IGNITER_WORK_TASK_STK_SIZE];

/*
*********************************************************************************************************
*                                           LOCAL VARIABLES
*********************************************************************************************************
*/
static  SWITCH_TYPE_VARIABLE_Typedef    g_eHydrgProducerManagerStopDlyStatu = OFF;
static          u16                     g_u16IgniterDelayOffSeconds = 0;
static  SWITCH_TYPE_VARIABLE_Typedef    g_eIgniterWorkStatu = OFF;
static  WHETHER_TYPE_VARIABLE_Typedef   g_eIgniterDirectWork = NO;
static  WHETHER_TYPE_VARIABLE_Typedef   enAheadRunningFlag = NO;    //响应安卓指令，在第一次点火后，提前启动发电

static              void                IgniterWorkTask(uint16_t *);

static              void                HydrgProducerManagerTask(void);
static              void                HydrgProducerManagerDlyStopTask(void);

/*
*********************************************************************************************************
*                                      IgniteFirstTime()
*
* Description:  execute the first time ignite.
*
* Arguments  :  m_IgniteCheckTable1 - the compare level that whether the ignite is success
*               m_GoToNextStepTempTable1 - the internal jump point of the process
*               MaxTryTimes    - the maximal retry time if the ignite if failed
*               m_CheckDelayMinute      - the delay minute that wait for check whether the ignite if success    点火结果检查延时
*
* Returns    :  the information of whether the ignite process is success
*********************************************************************************************************
*/
IGNITE_CHECK_STATU_Typedef IgniteFirstTime(float m_IgniteCheckTable1, float m_GoToNextStepTempTable1, uint8_t MaxTryTimes, uint8_t m_CheckDelayMinute)
{
    OS_ERR      err;
    IGNITE_CHECK_STATU_Typedef m_eIgniteStatu;

    if(EN_START_PRGM_ONE_FRONT == GetSystemWorkStatu())
    {

        SetSystemWorkStatu(EN_START_PRGM_ONE_BEHIND);

        if(EN_START_PRGM_ONE_BEHIND == GetSystemWorkStatu())
        {
            APP_TRACE_INFO(("Ignite first time behind...\n\r"));
            BSP_LqdValve1_PwrOn();
            SetPumpCtlSpd(19);
            SetHydrgFanCtlSpd(200);
            IgniterWorkForSeconds(240);

            SetHydrgProducerDigSigIgniteFirstTimeBehindMonitorHookSwitch(DEF_ENABLED);
            OSSemPend(&IgniteFirstBehindWaitSem,                        //  因有多个之一的条件满足即可，且有多处等待点，故使用信号量传递信息，而非任务信号量。
                      (OS_CFG_TICK_RATE_HZ * 60 * 30),                 //三十分钟点火检查时间
                      OS_OPT_PEND_BLOCKING,
                      NULL,
                      &err);
            
            
            if(err == OS_ERR_NONE)//正常得到信号
            {
                if((GetAheadRunningFlag() == YES))          //收到提前启动指令
                {
                    APP_TRACE_INFO(("Ignite first time behind ahead start command...\n\r"));
                    SetAheadRunningFlag(NO);
                    m_eIgniteStatu = EN_PASS;
                }
                else if(EN_START_PRGM_ONE_BEHIND == GetSystemWorkStatu())    //温度达到要求
                {
                    APP_TRACE_INFO(("Ignite first time behind temperature meet requirement ...\n\r"));
                    m_eIgniteStatu = EN_PASS;
                }
                else       //手动控制指令
                {
                    APP_TRACE_INFO(("Ignite first time behind wait has been broken...\n\r"));
                    IgniterWorkForSeconds(0);
                    m_eIgniteStatu = EN_NOT_PASS;
                }
            }
            else if(err == OS_ERR_TIMEOUT)                   //达到延时时间
            {
                APP_TRACE_INFO(("Ignite first time behind wait timeout...\n\r"));
                m_eIgniteStatu = EN_PASS;
            }
            else                                        //其他
            {
                APP_TRACE_INFO(("Ignite first time behind wait err...\n\r"));
                IgniterWorkForSeconds(0);
                m_eIgniteStatu = EN_NOT_PASS;
            }

            SetPumpCtlSpd(0);
            BSP_LqdValve1_PwrOff();
        }
        else
        {
            APP_TRACE_INFO(("The first time ignite first time statu is failed, ignite for the first time behind has not start...\n\r"));
            m_eIgniteStatu = EN_NOT_PASS;
        }
    }
    else
    {
        APP_TRACE_INFO(("The program start the ignite first time front err ...\n\r"));
        m_eIgniteStatu = EN_NOT_PASS;
    }

    return m_eIgniteStatu;
}

/*
*********************************************************************************************************
*                                      IgniteSecondTime()
*
* Description:  execute the second time ignite.
*               第二次点火
* Arguments  :  m_IgniteCheckTable1 - the compare level that whether the ignite is success
*               m_GoToNextStepTempTable1 - the internal jump point of the process
*               MaxTryTimes    - the maximal retry time if the ignite if failed
*               m_CheckDelayMinute      - the delay minute that wait for check whethe the ignite if success
*
* Returns    :  the information of whether the ignite process is success
*********************************************************************************************************
*/
IGNITE_CHECK_STATU_Typedef IgniteSecondTime(float m_IgniteCheckTable2, float m_GoToNextStepTempTable2, uint8_t maxtrytime, uint8_t m_CheckDelayTimeFlag)
{
//    OS_ERR      err;
    IGNITE_CHECK_STATU_Typedef m_eIgniteStatu;

    APP_TRACE_INFO(("Ignite second time...\n\r"));
    BSP_FastHeaterPwrOff();  
    BSP_LqdValve2_PwrOn();
    SetPumpCtlSpd(40);
    SetHydrgFanCtlSpd(200);
    IgniterWorkForSeconds(120);
    
//    SetHydrgProducerDigSigIgniteFirstTimeBehindMonitorHookSwitch(DEF_ENABLED);
//    OSSemPend(&IgniteSecondBehindWaitSem,                        //  因有多个之一的条件满足即可，且有多处等待点，故使用信号量传递信息，而非任务信号量。
//              (OS_CFG_TICK_RATE_HZ * 60 * 2),                 //三十分钟后点火检查时间
//              OS_OPT_PEND_BLOCKING,
//              NULL,
//              &err);
//    
//    
//    if(err == OS_ERR_NONE)                            //正常得到信号
//    {
//        
//        APP_TRACE_INFO(("The temperature meets the requirement...\n\r"));
//        m_eIgniteStatu = EN_PASS;
//       
//    }
//    else  if(err == OS_ERR_TIMEOUT)                   //达到延时时间
//    {
//        APP_TRACE_INFO(("Ignite first time behind wait timeout...\n\r"));
//        m_eIgniteStatu = EN_NOT_PASS ;//;
//        BSP_LqdValve2_PwrOff();
//        SetPumpCtlSpd(0);
//    }
//    else                                             //其他
//    {
//        APP_TRACE_INFO(("Ignite second time behind wait err...\n\r"));
//        IgniterWorkForSeconds(0);
//        m_eIgniteStatu = EN_NOT_PASS ;//;
//        BSP_LqdValve2_PwrOff();
//        SetPumpCtlSpd(0);
//    }
    m_eIgniteStatu = EN_PASS;
    return m_eIgniteStatu;
}

/*
*********************************************************************************************************
*                                      SetAheadRunningFlag()
*
* Description:  set the flag the the system need to start early.
*
* Arguments  :  new statu
*
* Returns    :  none
*********************************************************************************************************
*/
void SetAheadRunningFlag(WHETHER_TYPE_VARIABLE_Typedef m_NEW_STATU)
{
    enAheadRunningFlag = m_NEW_STATU;
}

/*
*********************************************************************************************************
*                                      GetAheadRunningFlag()
*
* Description:  获取提前启动标志
*
* Arguments  :  none
*
* Returns    :  the statu
*********************************************************************************************************
*/
WHETHER_TYPE_VARIABLE_Typedef GetAheadRunningFlag(void)
{
    return enAheadRunningFlag;
}

/*
*********************************************************************************************************
*                                      HydrgProducerManagerTaskCreate()
*
* Description:  create the hydrogen producer manager task.
*
* Arguments  :  none
*
* Returns    :  none
*********************************************************************************************************
*/
void HydrgProducerManagerTaskCreate(void)
{
    OS_ERR  err;
    OSSemCreate(&HydrgProducerManagerStopSem, "Hydrogen producer manager sem", 0, &err);

    OSTaskCreate((OS_TCB *)&HydrgProducerManagerTaskTCB,                    // Create the start task
                 (CPU_CHAR *)"Hydrogen producer manager task",
                 (OS_TASK_PTR) HydrgProducerManagerTask,
                 (void *) 0,
                 (OS_PRIO) HYDROGEN_PRODUCER_MANAGER_TASK_PRIO,
                 (CPU_STK *)&HydrgProducerManagerTaskStk[0],
                 (CPU_STK_SIZE) HYDROGEN_PRODUCER_MANAGER_TASK_STK_SIZE / 10,
                 (CPU_STK_SIZE) HYDROGEN_PRODUCER_MANAGER_TASK_STK_SIZE,
                 (OS_MSG_QTY) 5u,
                 (OS_TICK) 0u,
                 (void *) 0,
                 (OS_OPT)(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR *)&err);
    APP_TRACE_INFO(("Created hydrogen producer manager task,and err code is %d...\n\r", err));
}

/*
*********************************************************************************************************
*                                      HydrgProducerManagerTask()
*
* Description:  the hydrogen producer manager task.
*
* Arguments  :  none
*
* Returns    :  none
*********************************************************************************************************
*/
void HydrgProducerManagerTask()
{
    OS_ERR      err;

    while(DEF_TRUE)
    {
        OSTaskSuspend(NULL, &err);
        APP_TRACE_INFO(("Hydrogen producer managing...\n\r"));
        SetHydrgProducerDigSigAlarmRunningMonitorHookSwitch(DEF_ENABLED);//开启运行数字信号警报监测开关
        SetHydrgProducerAnaSigAlarmRunningMonitorHookSwitch(DEF_ENABLED);//开启运行模拟信号警报监测开关
        SetHydrgProducerAnaSigRunningStartAutoAdjHookSwitch(DEF_ENABLED);//允許自動減泵速,在泵速降到30以后会自动失能
        SetHydrgProducerFanRunningSpeedMonitorHookSwitch(DEF_ENABLED);   //开启泵和风机的速度监测
        SetHydrgProducerPumpRunningSpeedMonitorHookSwitch(DEF_ENABLED);

        while(DEF_TRUE)
        {
            OSSemPend(&HydrgProducerManagerStopSem, OS_CFG_TICK_RATE_HZ, OS_OPT_PEND_BLOCKING, NULL, &err);

            if(err == OS_ERR_NONE)          //一旦正确接收到信号量，说明机器进入关机阶段
            {
                break;
            }
        }

        SetHydrgProducerDigSigAlarmRunningMonitorHookSwitch(DEF_DISABLED);//停止数字信号监控任务中运行阶段信号监测
        SetHydrgProducerAnaSigAlarmRunningMonitorHookSwitch(DEF_DISABLED);//停止模拟信号监控任务中运行阶段信号监测
        SetHydrgProducerFanRunningSpeedMonitorHookSwitch(DEF_DISABLED);         //停止泵和风机的速度监测
        SetHydrgProducerPumpRunningSpeedMonitorHookSwitch(DEF_DISABLED);
        APP_TRACE_INFO(("Hydrogen producer manager stop...\n\r"));
    }
}

/*
*********************************************************************************************************
*                                      HydrgProducerDlyStopTaskCreate()
*
* Description:  create the hydrogen producer delay stop task.
*
* Arguments  :  none
*
* Returns    :  none
*********************************************************************************************************
*/
void HydrgProducerDlyStopTaskCreate()
{
    OS_ERR  err;
    OSTaskCreate((OS_TCB *)&HydrgProducerManagerDlyStopTaskTCB,
                 (CPU_CHAR *)"Hydrg Producer manager delay stop task",
                 (OS_TASK_PTR) HydrgProducerManagerDlyStopTask,
                 (void *) 0,
                 (OS_PRIO) HYDRG_PRODUCER_DELAY_STOP_TASK_PRIO,
                 (CPU_STK *)&HydrgProducerManagerDlyStopTaskStk[0],
                 (CPU_STK_SIZE) HYDROGEN_PRODUCER_MANAGER_DLY_STOP_TASK_STK_SIZE / 10,
                 (CPU_STK_SIZE) HYDROGEN_PRODUCER_MANAGER_DLY_STOP_TASK_STK_SIZE,
                 (OS_MSG_QTY) 5u,
                 (OS_TICK) 0u,
                 (void *) 0,
                 (OS_OPT)(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR *)&err);
    APP_TRACE_INFO(("Created the hydrogen producer manager delay stop task, err code is %d...\n\r", err));
}

/*
*********************************************************************************************************
*                                      HydrgProducerManagerDlyStopTask()
*
* Description:  the hydrogen producer delay stop task.
*               制氢机延时停止任务(5分钟)
* Arguments  :  none
*
* Returns    :  none
*********************************************************************************************************
*/
void HydrgProducerManagerDlyStopTask(void)
{
    OS_ERR      err;

    while(DEF_TRUE)
    {
        OSTaskSuspend(NULL, &err);

        OSSemPost(&HydrgProducerManagerStopSem,     //结束制氢机管理任务的本次运行，进入阻塞，等待下一次启动
         OS_OPT_POST_1,
        &err);
          
        g_eHydrgProducerManagerStopDlyStatu = ON;
        APP_TRACE_INFO(("The Hydrogen producer manager start to delay stop...\n\r"));
        IgniterWorkForSeconds(0);                   //防止关机时，点火器因未到定时时间而继续运行，故将其关闭
        SetPumpCtlSpd(0);
        BSP_LqdValve2_PwrOff();
        SetHydrgFanCtlSpd(200);
        OSTimeDlyHMSM(0, 5, 0, 0, OS_OPT_TIME_HMSM_STRICT, &err);    
        OSTaskSemPost(&AppTaskStartTCB, OS_OPT_POST_NO_SCHED, &err);
        SetHydrgFanCtlSpd(0);
           
         APP_TRACE_INFO(("aaaa...\n\r"));
        g_eHydrgProducerManagerStopDlyStatu = OFF;
    }
}

/*
*********************************************************************************************************
*                                      GetHydrgProducerStopDlyStatu()
*
* Description:  get the statu of whether the hydrogen producer delay stop is finish.
*
* Arguments  :  none
*
* Returns    :  the statu
*********************************************************************************************************
*/
uint8_t GetHydrgProducerStopDlyStatu(void)
{
    return g_eHydrgProducerManagerStopDlyStatu;
}

/*
*********************************************************************************************************
*                                      IgniterWorkTaskCreate()
*
* Description:  create the igniter work task.
*
* Arguments  :  none
*
* Returns    :  none
*********************************************************************************************************
*/
void    IgniterWorkTaskCreate()
{
    OS_ERR      err;

    OSTaskCreate((OS_TCB *)&IgniterWorkTaskTCB,
                 (CPU_CHAR *)"Igniter work task",
                 (OS_TASK_PTR) IgniterWorkTask,
                 (void *) 0,
                 (OS_PRIO) IGNITER_WORK_TASK_PRIO,
                 (CPU_STK *)&IgniterWorkStk[0],
                 (CPU_STK_SIZE) IGNITER_WORK_TASK_STK_SIZE / 10,
                 (CPU_STK_SIZE) IGNITER_WORK_TASK_STK_SIZE,
                 (OS_MSG_QTY) 5u,
                 (OS_TICK) 0u,
                 (void *) 0,
                 (OS_OPT)(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR *)&err);
    APP_TRACE_INFO(("Created the igniter work task, and err code is %d...\n\r", err));
}

/*
*********************************************************************************************************
*                                      IgniterWorkForSeconds()
*
* Description:  set the igniter work time.
*
* Arguments  :  work seconds
*
* Returns    :  none
*********************************************************************************************************
*/
void  IgniterWorkForSeconds(uint16_t i_WorkSeconds)
{
    OS_ERR      err;
    CPU_SR_ALLOC();

    APP_TRACE_INFO(("Igniter (re)start to work for %d seconds...\n\r", i_WorkSeconds));
    CPU_CRITICAL_EXIT();
    g_u16IgniterDelayOffSeconds = i_WorkSeconds;

    if(i_WorkSeconds > 0)
    {
        if(g_eIgniterWorkStatu == ON)
        {
            //延时恢复点火任务
            OSTimeDlyResume(&IgniterWorkTaskTCB, &err);
            g_eIgniterDirectWork = YES;
        }
        else
        {
            OSTaskResume(&IgniterWorkTaskTCB,
                         &err);
        }
    }
    else
    {
        if(g_eIgniterWorkStatu == ON)
        {
            OSTimeDlyResume(&IgniterWorkTaskTCB,
                            &err);
        }
    }

    CPU_CRITICAL_EXIT();
}

/*
*********************************************************************************************************
*                                      IgniterWorkForSeconds()
*
* Description:  the igniter work task.
*
* Arguments  :  none
*
* Returns    :  none
*********************************************************************************************************
*/
void IgniterWorkTask(uint16_t *p_arg)
{
    OS_ERR      err;

    while(DEF_TRUE)
    {
        g_eIgniterWorkStatu = OFF;

        if(g_eIgniterDirectWork == NO)
        {
            OSTaskSuspend(NULL, &err);          //暂停当前任务
        }
        else
        {
            g_eIgniterDirectWork = NO;
        }

        if(g_u16IgniterDelayOffSeconds > 0)
        {
            APP_TRACE_INFO(("Igniter on...\n\r"));
            BSP_IgniterPwrOn();                 //点火
            g_eIgniterWorkStatu = ON;

            OSTimeDlyHMSM(0, 0, g_u16IgniterDelayOffSeconds, 0, OS_OPT_TIME_HMSM_NON_STRICT, &err);
            BSP_IgniterPwrOff();
            APP_TRACE_INFO(("Igniter off...\n\r"));
        }
        else
        {
            BSP_IgniterPwrOff();
            APP_TRACE_INFO(("Igniter off...\n\r"));
        }
    }
}
/******************* (C) COPYRIGHT 2015 Guangdong Hydrogen *****END OF FILE****/

