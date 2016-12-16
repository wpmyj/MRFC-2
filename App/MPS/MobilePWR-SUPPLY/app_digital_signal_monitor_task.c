/*
***************************************************************************************************
*                                              APPLICATION CODE
*
*                          (c) Copyright 2015; Guangdong Hydrogen Energy Science And Technology Co.,Ltd
*
*               All rights reserved.  Protected by international copyright laws.
*               Knowledge of the source code may NOT be used without authorization.
***************************************************************************************************
*/
/*
***************************************************************************************************
* Filename      : app_thermocouple_temp.c
* Version       : V1.00
* Programmer(s) : SunKing.Yun
***************************************************************************************************
*/
/*
***************************************************************************************************
*                                             INCLUDE FILES
***************************************************************************************************
*/

#include <includes.h>
#include <bsp.h>
#include <app_digital_signal_monitor_task.h>
#include "app_system_run_cfg_parameters.h"
#include "app_top_task.h"
#include "app_stack_manager.h"
/*
***************************************************************************************************
*                                           MACRO DEFINITIONS
***************************************************************************************************
*/
#define     NMB_OF_AVERAGE_TEMPERATURE_SAMPLE           4   //平均温度采样样本数
#define     DIG_SIGNAL_MONITOR_TASK_STK_SIZE            1024

/*
***************************************************************************************************
*                                         OS-RELATED    VARIABLES
***************************************************************************************************
*/
extern      OS_SEM      IgniteFirstBehindWaitSem;

extern      OS_SEM      IgniteSecondBehindWaitSem;
OS_TCB      DigSigMonitorTaskTCB;

static      CPU_STK_8BYTE_ALIGNED     AnaSigMonitorTaskStk[DIG_SIGNAL_MONITOR_TASK_STK_SIZE];

/*
***************************************************************************************************
*                                       LOCAL GLOBAL VARIABLES
***************************************************************************************************
*/
extern          REFORMER_TEMP_CMP_LINES_Typedef             g_stReformerTempCmpTbl;

static  uint16_t g_u16OriginalTempFilter[2][NMB_OF_AVERAGE_TEMPERATURE_SAMPLE] = {0};   //原始温度滤波数组
static  uint8_t  g_u8DigTempFilterOperationCursor = 0;
static  float    g_fAnaTempSum[2] = {0};
static  float    g_fAnaTemp[2] = {0};

static  uint8_t  g_u8HydrgProducerDigSigIgniteFirstTimeBehindMonitorHookSw = DEF_DISABLED;   //制氢机第一次点火后的数字信号监测任务开关
static  uint8_t  g_u8HydrgProducerDigSigRunningMonitorAlarmHookSw = DEF_DISABLED;            //制氢机运行数字信号监测警报开关
static  uint8_t  g_u8StackExhaustTimesCountPerMinutesMonitorHookSw = DEF_DISABLED;//电堆每分钟排气次数监测开关
/*
***************************************************************************************************
*                                         FUNCTION PROTOTYPES
***************************************************************************************************
*/
static   void   UpdateThermocoupleTemp(uint8_t *i_TempErr);
static   void   DigSigMonitorTask(void *p_arg);
static   void   HydrgProducerDigSigIgniteFirstTimeBehindMonitorHook(void);
static   void   HydrgProducerDigSigAlarmRunningMonitorHook(void);
static   void   SetStackExhaustTimesCountPerMinutesMonitorHook(void);
/*
***************************************************************************************************
*                                                UpdateThermocoupleTemp()
*
* Description : 1.average filter the digtal value.                          数字量平均滤波
*               2.and then convert the digtal sersor to natural temperature.单位转换
*               3.get the information if the thermocouple is broken.        热电偶损坏信息反馈
*
* Arguments   : the address to store the thermocouple broken error information.
*               TempErr[0]-重整室温度错误标志，TempErr[1]-点火温度错误标志
* Returns     : none
***************************************************************************************************
*/
static void UpdateThermocoupleTemp(uint8_t *i_TempErr)
{
    uint8_t     i, j;
    uint8_t     u8TempErr[2] = {0};
    float       fltOriginalValue[2];
    CPU_SR_ALLOC();

    BSP_MAX6675_Temp_Read(fltOriginalValue, u8TempErr);

    if(g_u8DigTempFilterOperationCursor >= NMB_OF_AVERAGE_TEMPERATURE_SAMPLE)
    {
        g_u8DigTempFilterOperationCursor = 0;
    }

    CPU_CRITICAL_ENTER();

    for(i = 0; i < 2; i++)
    {
        if(u8TempErr[i] == 0)
        {
            g_fAnaTempSum[i] -= g_u16OriginalTempFilter[i][g_u8DigTempFilterOperationCursor];
            g_u16OriginalTempFilter[i][g_u8DigTempFilterOperationCursor] = fltOriginalValue[i];
            g_fAnaTempSum[i] += g_u16OriginalTempFilter[i][g_u8DigTempFilterOperationCursor];
            g_fAnaTemp[i] = g_fAnaTempSum[i] / NMB_OF_AVERAGE_TEMPERATURE_SAMPLE;
        }
        else
        {
            g_fAnaTempSum[i] = 999 * NMB_OF_AVERAGE_TEMPERATURE_SAMPLE;

            for(j = 0; j < NMB_OF_AVERAGE_TEMPERATURE_SAMPLE; j++)
            {
                g_u16OriginalTempFilter[i][j] = 999;
            }

            g_fAnaTemp[i] = 999;
        }
    }

    CPU_CRITICAL_EXIT();

    g_u8DigTempFilterOperationCursor++;
    *i_TempErr = *u8TempErr;
    *(i_TempErr + 1) = *(u8TempErr + 1);
}

/*
***************************************************************************************************
*                                          GetReformerTemp()
*
* Description : get the temperature of the reformer.
*
* Arguments   : none.
*
* Returns     : the reformer temperature.
*
* Notes       : none
***************************************************************************************************
*/
float GetReformerTemp(void)
{
    return g_fAnaTemp[1];
}

/*
***************************************************************************************************
*                                         GetFireOrRodTemp()
*
* Description : get the temperature of the fire or rod.
*
* Arguments   : none.
*
* Returns     : the fire of rod temperature.
*
* Notes       : none
***************************************************************************************************
*/
float GetFireOrRodTemp(void)
{
    return g_fAnaTemp[0];
}

/*
***************************************************************************************************
*                                          DigSigMonitorTaskCreate()
*
* Description : create the task that monitor the digital signal.
*
* Arguments   : none.
*
* Returns     : none.
*
* Notes       : none.
***************************************************************************************************
*/
void  DigSigMonitorTaskCreate(void)
{
    OS_ERR      err;

    OSTaskCreate((OS_TCB *)&DigSigMonitorTaskTCB,                    // Create the start task
                 (CPU_CHAR *)"DigSignal Monitor Task Start",
                 (OS_TASK_PTR) DigSigMonitorTask,
                 (void *) 0,
                 (OS_PRIO) DIG_SIGNAL_MONITOR_TASK_PRIO,
                 (CPU_STK *)&AnaSigMonitorTaskStk[0],
                 (CPU_STK_SIZE) DIG_SIGNAL_MONITOR_TASK_STK_SIZE / 10,
                 (CPU_STK_SIZE) DIG_SIGNAL_MONITOR_TASK_STK_SIZE,
                 (OS_MSG_QTY) 5u,
                 (OS_TICK) 0u,
                 (void *) 0,
                 (OS_OPT)(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR *)&err);
    APP_TRACE_INFO(("Created thermocouple temperature Task, and err code is %d...\n\r", err));
}

/*
***************************************************************************************************
*                                          DigSigMonitorTask()
*
* Description : 1.the task monitor the digital signal.
*               2.run the hooks that related to the digital signal.
*
* Arguments   : none.
*
* Returns     : none.
*
* Notes       : none.
***************************************************************************************************
*/
static void  DigSigMonitorTask(void *p_arg)
{
    OS_ERR      err;
    uint8_t     TempErr[2];
    //数字温度传感器转换引脚复位
    DigTempSensorConvertStart();

    while(DEF_TRUE)
    {
        OSTimeDlyHMSM(0, 0, 0, 250, OS_OPT_TIME_HMSM_STRICT, &err);
        
        //温度传感器片选选中,更新热电偶温度数据，获得错误标志数组
        UpdateThermocoupleTemp(TempErr);

        //根据采样错误位设置自检码指定位
        if(TempErr[0] == 1)
        {
            SetMachinePartASelfCheckCodeBit(SelfCheckCodeGrpHydrgReformerThermocoupleBit);
        }
        else
        {
            ResetMachinePartASelfCheckCodeBit(SelfCheckCodeGrpHydrgReformerThermocoupleBit);
        }

        if(TempErr[1] == 1)
        {
            SetMachinePartASelfCheckCodeBit(SelfCheckCodeGrpHydrgFireThermocoupleBit);
        }
        else
        {
            ResetMachinePartASelfCheckCodeBit(SelfCheckCodeGrpHydrgReformerThermocoupleBit);
        }

        //开启监测任务
        if(g_u8HydrgProducerDigSigIgniteFirstTimeBehindMonitorHookSw == DEF_ENABLED)
        {
            HydrgProducerDigSigIgniteFirstTimeBehindMonitorHook();
        }

        if(g_u8HydrgProducerDigSigRunningMonitorAlarmHookSw == DEF_ENABLED)
        {
            HydrgProducerDigSigAlarmRunningMonitorHook();
        }
        if(g_u8StackExhaustTimesCountPerMinutesMonitorHookSw == DEF_ENABLED) {
            SetStackExhaustTimesCountPerMinutesMonitorHook();
        }
        
    }
}

/*
***************************************************************************************************
*                     SetHydrgProducerDigSigIgniteFirstTimeBehindMonitorHookSwitch()
*
* Description : En/Disable the digital signal wait for the ignite first time to behind switchover.
*
* Arguments   : none.
*
* Returns     : none.
*
* Notes       : none.
***************************************************************************************************
*/
void SetHydrgProducerDigSigIgniteFirstTimeBehindMonitorHookSwitch(u8 i_NewStatu)
{
    g_u8HydrgProducerDigSigIgniteFirstTimeBehindMonitorHookSw = i_NewStatu;
}

/*
***************************************************************************************************
*                     HydrgProducerDigSigIgniteFirstTimeBehindMonitorHook()
*
* Description : wait for the ignite first time to behind switchover.
*               制氢机第一次点火后重整室温度监测
* Arguments   : none.
*
* Returns     : none.
*
* Notes       : none.
***************************************************************************************************
*/

static void HydrgProducerDigSigIgniteFirstTimeBehindMonitorHook(void)
{
    OS_ERR err;
    CPU_SR_ALLOC();
    CPU_CRITICAL_ENTER();

    if(GetReformerTemp() >= g_stReformerTempCmpTbl.IgFstTimeOverTmpPnt)
    {
        OSSemPost(&IgniteFirstBehindWaitSem, OS_OPT_POST_1, &err);
        g_u8HydrgProducerDigSigIgniteFirstTimeBehindMonitorHookSw = DEF_DISABLED;
    }

    CPU_CRITICAL_EXIT();

}

/*
***************************************************************************************************
*                     SetStackExhaustTimesCountPerMinutesMonitorHookSwitch()
*
* Description : Monitor the digital signal that fluid weight per minute.
*
* Arguments   : none.
*
* Returns     : none.
*
* Notes       : .
***************************************************************************************************
*/
void SetStackExhaustTimesCountPerMinutesMonitorHookSwitch(uint8_t i_NewStatu)
{
    g_u8StackExhaustTimesCountPerMinutesMonitorHookSw = i_NewStatu;
}

static void SetStackExhaustTimesCountPerMinutesMonitorHook()
{
    static uint16_t u16CountPerMinutes = 0;

    u16CountPerMinutes ++;

    if(u16CountPerMinutes >= 240) { //一分钟清零一次电堆排气次数
        ResetStackExhaustTimesCountPerMinutes();
        u16CountPerMinutes = 0;
    }
}
/*
***************************************************************************************************
*                     SetHydrgProducerDigSigAlarmRunningMonitorHookSwitch()
*
* Description : En/Disable the digital signal concerned alarms.
*
* Arguments   : none.
*
* Returns     : none.
*
* Notes       : none.
***************************************************************************************************
*/
void SetHydrgProducerDigSigAlarmRunningMonitorHookSwitch(uint8_t i_NewStatu)
{
    g_u8HydrgProducerDigSigRunningMonitorAlarmHookSw = i_NewStatu;
}

/*
***************************************************************************************************
*                     HydrgProducerDigSigAlarmRunningMonitorHook()
*
* Description : manager the digital signal concerned alarms.
*               制氢机运行重整室温度警报信号监测
* Arguments   : none.
*
* Returns     : none.
*
* Notes       : none.
***************************************************************************************************
*/

static void HydrgProducerDigSigAlarmRunningMonitorHook(void)
{
    float fReformerTemp;
    fReformerTemp = GetReformerTemp();

    if(fReformerTemp > g_stReformerTempCmpTbl.AlarmUpperLimit)
    {
        AlarmCmd(REFORMER_TEMP_HIGH_ALARM, ON);
    }
    else    if(fReformerTemp < g_stReformerTempCmpTbl.AlarmLowerLimit)
    {
        AlarmCmd(REFORMER_TEMP_LOW_ALARM, ON);
    }
    else
    {
        AlarmCmd(REFORMER_TEMP_HIGH_ALARM, OFF);
        AlarmCmd(REFORMER_TEMP_LOW_ALARM, OFF);
    }

//  UpdateBuzzerStatuInCruise();
}

/******************* (C) COPYRIGHT 2015 Guangdong Hydrogen *****END OF FILE****/

