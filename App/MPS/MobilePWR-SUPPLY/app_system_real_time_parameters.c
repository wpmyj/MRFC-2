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
* Filename      : app_system_real_time_parameters.c
* Version       : V1.00
* Programmer(s) : SunKing.Yun
***************************************************************************************************
*/
/*
***************************************************************************************************
*                                           INCLUDE FILES
***************************************************************************************************
*/
#include "app_system_run_cfg_parameters.h"
#include "app_system_real_time_parameters.h"
#include "app_analog_signal_monitor_task.h"
#include "app_wireness_communicate_task.h"
#include "includes.h"
#include <cpu.h>
#include <bsp_ana_sensor.h>
#include "app_analog_signal_monitor_task.h"
/*
***************************************************************************************************
*                                           MACRO DEFINITIONS
***************************************************************************************************
*/
#define SYSTEM_TIME_STATISTIC_TASK_STK_SIZE     64

/*
***************************************************************************************************
*                                         OS-RELATED    VARIABLES
***************************************************************************************************
*/
static      OS_TCB      SysTimeStatTaskTCB;

OS_SEM      g_stSystemTimeUpdateSem;

static      CPU_STK     SysTimeStatTaskStk[SYSTEM_TIME_STATISTIC_TASK_STK_SIZE];

/*
***************************************************************************************************
*                                           LOCAL VARIABLES
***************************************************************************************************
*/
static  SYSTEM_CONTROL_MODE_Typedef     g_eSystemControlMode = EN_CONTROL_MODE_AUTO;
static  SYSTEM_WORK_MODE_Typedef        g_eSystemWorkMode = EN_WORK_MODE_HYDROGEN_PRODUCER_AND_FUEL_CELL;

/* The system time variable */
static  SYSTEM_TIME_Typedef             g_stSystemTime = {0, 0, 0};

static  SYSTEM_TIME_Typedef             g_stHydrgProduceTimeThisTime = {0, 0, 0};
static  SYSTEM_TIME_Typedef             g_stHydrgProduceTimeTotal = {0, 0, 0};

static  SYSTEM_TIME_Typedef             g_stStackProductTimeThisTime = {0, 0, 0};
static  SYSTEM_TIME_Typedef             g_stStackProductTimeTotal = {0, 0, 0};

/* Statistics run number */
static          uint16_t                g_u16SystemWorkTimes = 0;
static          uint16_t                g_u16HydrgProducerWorkTimes = 0;
static          uint16_t                g_u16StackWorkTimes = 0;

static        SELF_CHECK_CODE_Typedef   g_stSelfCheckCode;
static  RUNNING_ALARM_STATUS_Typedef    g_stSystemAlarmsInf;    //报警码及保持时间

static          uint16_t                g_u16CtlAndCommunicateCode = 0x0000;
static          uint32_t                g_SystemRunningStatuCode = 0x00000000;

static  SYSTEM_WORK_STATU_Typedef       g_eSystemWorkStatu = EN_WAITTING_COMMAND;
static  STACK_WORK_STATU_Typedef        g_eStackWorkStatu = EN_NOT_IN_WORK;

static          float                   g_fSystemIsolatedGeneratedEnergyThisTime = 0.0;

static          uint8_t                 g_u8WaitWorkModeSelectSwitch = DEF_ENABLED;

static          uint32_t                                g_u32SysErrCode = 0;
//系统预定错误类型有32种，对应g_u32ErrCode中的32位
static  SHUT_DOWN_REQUEST_RESPONSE_WAIT_MASK_Typedef    g_stShutDownRequestWaitMask[32] = {{EN_UN_MASK, 0 , {0 , 0, 0}},
    {EN_UN_MASK, 0 , {0 , 0, 0}},
    {EN_UN_MASK, 0 , {0 , 0, 0}},
    {EN_UN_MASK, 0 , {0 , 0, 0}},
    {EN_UN_MASK, 0 , {0 , 0, 0}},
    {EN_UN_MASK, 0 , {0 , 0, 0}},
    {EN_UN_MASK, 0 , {0 , 0, 0}},
    {EN_UN_MASK, 0 , {0 , 0, 0}},
    {EN_UN_MASK, 0 , {0 , 0, 0}},
    {EN_UN_MASK, 0 , {0 , 0, 0}},
    {EN_UN_MASK, 0 , {0 , 0, 0}},
    {EN_UN_MASK, 0 , {0 , 0, 0}},
    {EN_UN_MASK, 0 , {0 , 0, 0}},
    {EN_UN_MASK, 0 , {0 , 0, 0}},
    {EN_UN_MASK, 0 , {0 , 0, 0}},
    {EN_UN_MASK, 0 , {0 , 0, 0}},
    {EN_UN_MASK, 0 , {0 , 0, 0}},
    {EN_UN_MASK, 0 , {0 , 0, 0}},
    {EN_UN_MASK, 0 , {0 , 0, 0}},
    {EN_UN_MASK, 0 , {0 , 0, 0}},
    {EN_UN_MASK, 0 , {0 , 0, 0}},
    {EN_UN_MASK, 0 , {0 , 0, 0}},
    {EN_UN_MASK, 0 , {0 , 0, 0}},
    {EN_UN_MASK, 0 , {0 , 0, 0}},
    {EN_UN_MASK, 0 , {0 , 0, 0}},
    {EN_UN_MASK, 0 , {0 , 0, 0}},
    {EN_UN_MASK, 0 , {0 , 0, 0}},
    {EN_UN_MASK, 0 , {0 , 0, 0}},
    {EN_UN_MASK, 0 , {0 , 0, 0}},
    {EN_UN_MASK, 0 , {0 , 0, 0}},
    {EN_UN_MASK, 0 , {0 , 0, 0}},
    {EN_UN_MASK, 0 , {0 , 0, 0}},
};

/*
***************************************************************************************************
*                                         FUNCTION PROTOTYPES
***************************************************************************************************
*/
static          void                    UpdateSysTime(SYSTEM_TIME_Typedef *);

static          void                    SetCtlAndCommunicaeCodeWorkModeSection(SYSTEM_WORK_MODE_Typedef i_u8NewStatu);

static          void                    SysTimeStatTask(void *p_arg);

/*
***************************************************************************************************
*                                      GetWorkMode()
*
* Description:  Get the work mode of the system.
*
* Arguments  :  none
*
* Returns    :  the work mode of the system.
***************************************************************************************************
*/
SYSTEM_WORK_MODE_Typedef GetWorkMode(void)
{
    return g_eSystemWorkMode;
}

/*
***************************************************************************************************
*                                      SetWorkMode()
*
* Description:  Set the work mode of the system.
*
* Arguments  :  the excepted work mode of the system.
*
* Returns    :  none.
***************************************************************************************************
*/
void SetWorkMode(SYSTEM_WORK_MODE_Typedef i_eNewWorkModeStatu)
{
    CPU_SR_ALLOC();

    if(i_eNewWorkModeStatu >= 3) { //判断
        i_eNewWorkModeStatu = EN_WORK_MODE_MALFUNCTION;
    }

    CPU_CRITICAL_ENTER();
    g_eSystemWorkMode = i_eNewWorkModeStatu;
    SetCtlAndCommunicaeCodeWorkModeSection(i_eNewWorkModeStatu);
    CPU_CRITICAL_EXIT();
}


/*
***************************************************************************************************
*                                      SetWorkModeWaittingForSelectFlag()
*
* Description:  Set the flag that the system is waitting for the control side to select the work mode.
*
* Arguments  :  none.
*
* Returns    :  none.
*
* Note(s)    :  The flag is included in the message that will send to the control side.
***************************************************************************************************
*/
void SetWorkModeWaittingForSelectFlag(void)
{
    CPU_SR_ALLOC();
    CPU_CRITICAL_ENTER();
    g_u8WaitWorkModeSelectSwitch = DEF_ENABLED;
    CPU_CRITICAL_EXIT();
}

void ResetWorkModeWaittingForSelectFlag(void)
{
    CPU_SR_ALLOC();
    CPU_CRITICAL_ENTER();
    g_u8WaitWorkModeSelectSwitch = DEF_DISABLED;
    CPU_CRITICAL_EXIT();
}

/*
***************************************************************************************************
*                                      GetWorkModeWaittingForSelectFlag()
*
* Description:  Get the flag that the system is waitting for the control side to select the work mode.
*
* Arguments  :  none.
*
* Returns    :  none.
*
* Note(s)    :  The flag is included in the message that will send to the control side.
***************************************************************************************************
*/
uint8_t GetWorkModeWaittingForSelectFlag(void)
{
    return g_u8WaitWorkModeSelectSwitch;
}

/*
***************************************************************************************************
*                                      ControlModeTurnOver()
*
* Description:  Turn over the control mode.
*               
* Arguments  :  none.
*
* Returns    :  none.
*
* Note(s)    :  none.
***************************************************************************************************
*/
void ControlModeTurnOver(void)
{
    if(g_eSystemControlMode == EN_CONTROL_MODE_AUTO) {
        g_eSystemControlMode = EN_CONTROL_MODE_MANNUAL;
        SetConrolAndCommunicateStatuCodeBit(ConrtolStatusCodeCtrlMode);
    } else {
        g_eSystemControlMode = EN_CONTROL_MODE_AUTO;
        ResetConrolAndCommunicateStatuCodeBit(ConrtolStatusCodeCtrlMode);
    }
}

/*
***************************************************************************************************
*                                      GetControlMode()
*
* Description:  Get control mode of the system.
*
* Arguments  :  none.
*
* Returns    :  none.
*
* Note(s)    :  none.
***************************************************************************************************
*/
SYSTEM_CONTROL_MODE_Typedef GetControlMode(void)
{
    return g_eSystemControlMode;
}

/*
***************************************************************************************************
*                                      ResetSystemWorkTimes()
*
* Description:  Reset the system work times.
*
* Arguments  :  none.
*
* Returns    :  none.
*
* Note(s)    :  none.
***************************************************************************************************
*/
void ResetSystemWorkTimes(void)
{
    g_u16SystemWorkTimes = 0;
    StoreSystemWorkTimes(g_u16SystemWorkTimes);
}

/*
***************************************************************************************************
*                                      LoadSystemWorkTimes()
*
* Description:  load the system work times.
*
* Arguments  :  newest work time number.
*
* Returns    :  none.
*
* Note(s)    :  none.
***************************************************************************************************
*/
void LoadSystemWorkTimesToPrgm(u16 i_u16WorkTimes)
{
    g_u16SystemWorkTimes = i_u16WorkTimes;
}

/*
***************************************************************************************************
*                                      SystemWorkTimesInc()
*
* Description:  Add a system work times.
*
* Arguments  :  none.
*
* Returns    :  none.
*
* Note(s)    :  none.
***************************************************************************************************
*/
void SystemWorkTimesInc(void)
{
    g_u16SystemWorkTimes ++;
    StoreSystemWorkTimes(g_u16SystemWorkTimes);
}
/*
***************************************************************************************************
*                                      GetSystemTime()
*
* Description:  Get the system run time that from the boot time.
*
* Arguments  :  none.
*
* Returns    :  none.
*
* Note(s)    :  none.
***************************************************************************************************
*/

SYSTEM_TIME_Typedef GetSystemTime(void)
{
    return g_stSystemTime;
}

/*
***************************************************************************************************
*                                      GetSystemWorkTimes()
*
* Description:  get the system work times.
*
* Arguments  :  none.
*
* Returns    :  the system work times
*
* Note(s)    :  none.
***************************************************************************************************
*/
uint16_t GetSystemWorkTimes()
{
    return g_u16SystemWorkTimes;
}

/*
***************************************************************************************************
*                                      LoadTotalWorkTimeToPrgm()
*
* Description:  Load total work time to Program.
*
* Arguments  :  none.
*
* Returns    :  none.
*
* Note(s)    :  none.
***************************************************************************************************
*/
void LoadTotalWorkTimeToPrgm(SYSTEM_TIME_Typedef i_stTotalTime)
{
    g_stHydrgProduceTimeTotal.hour = i_stTotalTime.hour;
    g_stHydrgProduceTimeTotal.minute = i_stTotalTime.minute;
    g_stHydrgProduceTimeTotal.second = i_stTotalTime.second;
}

/*
***************************************************************************************************
*                         ResetHydrgProduceTimeThisTime()
*
* Description:  Reset the hydrogen produce time this produce cycle.
*
* Arguments  :  none.
*
* Returns    :  none.
*
* Note(s)    :  none.
***************************************************************************************************
*/
void ResetHydrgProduceTimeThisTime()
{
    g_stHydrgProduceTimeThisTime.second = 0;
    g_stHydrgProduceTimeThisTime.minute = 0;
    g_stHydrgProduceTimeThisTime.hour = 0;
}

/*
***************************************************************************************************
*                                      GetHydrgProduceTimeThisTime()
*
* Description:  Get the hydrogen produce time this produce cycle.
*
* Arguments  :  none.
*
* Returns    :  hydrogen produce time this cycle.
*
* Note(s)    :  none.
***************************************************************************************************
*/
SYSTEM_TIME_Typedef GetHydrgProduceTimeThisTime(void)
{
    return g_stHydrgProduceTimeThisTime;
}

/*
***************************************************************************************************
*                                      GetHydrgProduceTimeTotal()
*
* Description:  Get the hydrogen produce time total.
*
* Arguments  :  none.
*
* Returns    :  hydrogen produce time total.
*
* Note(s)    :  none.
***************************************************************************************************
*/
SYSTEM_TIME_Typedef GetHydrgProduceTimeTotal(void)
{
    return g_stHydrgProduceTimeTotal;
}

/*
***************************************************************************************************
*                                      ResetStackProductTimeThisTime()
*
* Description:  Reset the stack product time this produce cycle.
*
* Arguments  :  none.
*
* Returns    :  none.
*
* Note(s)    :  none.
***************************************************************************************************
*/
void ResetStackProductTimeThisTime()
{
    g_stStackProductTimeThisTime.second = 0;
    g_stStackProductTimeThisTime.minute = 0;
    g_stStackProductTimeThisTime.hour = 0;
}

/*
***************************************************************************************************
*                                      GetStackProductTimeThisTime()
*
* Description:  Get the stack product time this produce cycle.
*
* Arguments  :  none.
*
* Returns    :  stack produce time this cycle.
*
* Note(s)    :  none.
***************************************************************************************************
*/
SYSTEM_TIME_Typedef GetStackProductTimeThisTime(void)
{
    return g_stStackProductTimeThisTime;
}

/*
***************************************************************************************************
*                                      GetStackProductTimeTotal()
*
* Description:  Get the stack product time total.
*
* Arguments  :  none.
*
* Returns    :  stack produce time total.
*
* Note(s)    :  none.
***************************************************************************************************
*/
SYSTEM_TIME_Typedef GetStackProductTimeTotal(void)
{
    return g_stStackProductTimeTotal;
}

/*
***************************************************************************************************
*                        ResetHydrgProducerWorkTimes()
*
* Description:  reset the hydrogen produce times.
*
* Arguments  :  none.
*
* Returns    :  none.
*
* Note(s)    :  none.
***************************************************************************************************
*/
void ResetHydrgProducerWorkTimes(void)
{
    g_u16HydrgProducerWorkTimes = 0;
}

/*
***************************************************************************************************
*                                      LoadHydrgProducerWorkTimes()
*
* Description:  load the hydrogen produce times.
*
* Arguments  :  newest the work time number.
*
* Returns    :  none.
*
* Note(s)    :  none.
***************************************************************************************************
*/
void LoadHydrgProducerWorkTimes(u16 u16WorkTimes)
{
    g_u16HydrgProducerWorkTimes = u16WorkTimes;
}

/*
***************************************************************************************************
*                                      HydrgProducerWorkTimesInc()
*
* Description:  increase the hydrogen produce times.
*
* Arguments  :  none.
*
* Returns    :  none.
*
* Note(s)    :  none.
***************************************************************************************************
*/
void HydrgProducerWorkTimesInc(void)
{
    g_u16HydrgProducerWorkTimes ++;
}

/*
***************************************************************************************************
*                                      GetHydrgProducerWorkTimes()
*
* Description:  get the hydrogen produce times.
*
* Arguments  :  none.
*
* Returns    :  unsigned short int.
*
* Note(s)    :  none.
***************************************************************************************************
*/
uint16_t GetHydrgProducerWorkTimes()
{
    return g_u16HydrgProducerWorkTimes;
}

/*
***************************************************************************************************
*                                      ResetStackWorkTimes()
*
* Description:  reset the stack produce times.
*
* Arguments  :  none.
*
* Returns    :  none.
*
* Note(s)    :  none.
***************************************************************************************************
*/
void ResetStackWorkTimes(void)
{
    g_u16StackWorkTimes = 0;
}

/*
***************************************************************************************************
*                                      LoadStackWorkTimes()
*
* Description:  get the stack produce times.
*
* Arguments  :  unsigned short int.
*
* Returns    :  none.
*
* Note(s)    :  none.
***************************************************************************************************
*/
void LoadStackWorkTimes(u16 u16WorkTimes)
{
    g_u16StackWorkTimes = u16WorkTimes;
}

/*
***************************************************************************************************
*                                      StackWorkTimesInc()
*
* Description:  increase the stack produce times.
*
* Arguments  :  none.
*
* Returns    :  none.
*
* Note(s)    :  none.
***************************************************************************************************
*/
void StackWorkTimesInc(void)
{
    g_u16StackWorkTimes ++;
}

/*
***************************************************************************************************
*                                      GetStackWorkTimes()
*
* Description:  get the stack produce times.
*
* Arguments  :  none.
*
* Returns    :  unsigned short int.
*
* Note(s)    :  none.
***************************************************************************************************
*/
uint16_t GetStackWorkTimes()
{
    return g_u16StackWorkTimes;
}

/*
***************************************************************************************************
*                                      SetSystemWorkStatu()
*
* Description:  set the system work statu.
*
* Arguments  :  enum type.
*
* Returns    :  none.
*
* Note(s)    :  none.
***************************************************************************************************
*/
void SetSystemWorkStatu(SYSTEM_WORK_STATU_Typedef m_enNewStatu)
{
    CPU_SR_ALLOC();

    if(m_enNewStatu >= 7) {
        m_enNewStatu = EN_SHUTTING_DOWN;
    }

    CPU_CRITICAL_ENTER();
    g_eSystemWorkStatu = m_enNewStatu;
    SetSystemRunningStatuCodeSysWorkStatuSection(g_eSystemWorkStatu);
    CPU_CRITICAL_EXIT();
}

/*
***************************************************************************************************
*                                      GetSystemWorkStatu()
*
* Description:  get the system work statu.
*
* Arguments  :  none.
*
* Returns    :  enum type.
*
* Note(s)    :  none.
***************************************************************************************************
*/
SYSTEM_WORK_STATU_Typedef GetSystemWorkStatu(void)
{
    return g_eSystemWorkStatu;
}

/*
***************************************************************************************************
*                                      SetStackWorkStatu()
*
* Description:  set the stack work statu.
*
* Arguments  :  enum type.
*
* Returns    :  none.
*
* Note(s)    :  none.
***************************************************************************************************
*/
void SetStackWorkStatu(STACK_WORK_STATU_Typedef i_eNewStatu)
{
    CPU_SR_ALLOC();
    CPU_CRITICAL_ENTER();
    g_eStackWorkStatu = i_eNewStatu;
    SetSystemRunningStatuCodeBit(RuningStatuCodeFuelCellRuningStatuBit_1);
    CPU_CRITICAL_EXIT();
}

/*
***************************************************************************************************
*                                      GetStackWorkStatu()
*
* Description:  get the stack work statu.
*
* Arguments  :  none.
*
* Returns    :  enum type.
*
* Note(s)    :  none.
***************************************************************************************************
*/
STACK_WORK_STATU_Typedef GetStackWorkStatu(void)
{
    return g_eStackWorkStatu;
}

/*
***************************************************************************************************
*                                      ResetAllAlarms()
*
* Description:  reset the all alarms statu.
*
* Arguments  :  none.
*
* Returns    :  none.
*
* Note(s)    :  none.
***************************************************************************************************
*/
void ResetAllAlarms()
{
    int i;
    g_stSystemAlarmsInf.AlarmCode = 0;

    for(i = 0; i < 32; i++) { //警报码为32位的数，时间全部清零
        g_stSystemAlarmsInf.HoldTime[i].second = 0x00;
        g_stSystemAlarmsInf.HoldTime[i].minute = 0x00;
        g_stSystemAlarmsInf.HoldTime[i].hour = 0x00;
    }
}

/*
***************************************************************************************************
*                                      AlarmCmd()
*
* Description:  command a kind of alarm statu.
*
* Arguments  :  m_enSystemAlarmKind - the kind of the alarm
*               m_enNewStatu - the excepted statu of the alarm
*
* Returns    :  none.
*
* Note(s)    :  none.
***************************************************************************************************
*/
void AlarmCmd(SYSTEM_ALARM_ADDR_Typedef m_enSystemAlarmKind, SYSTEM_ALARM_GRADE_Typedef m_enAlarmGrade,SWITCH_TYPE_VARIABLE_Typedef m_enNewStatu)
{
    if((g_stSystemAlarmsInf.AlarmCode ^ (m_enNewStatu << (u8)m_enSystemAlarmKind)) != 0) { //若状态有变化
        g_stSystemAlarmsInf.HoldTime[m_enSystemAlarmKind].second = 0x00;
        g_stSystemAlarmsInf.HoldTime[m_enSystemAlarmKind].minute = 0x00;
        g_stSystemAlarmsInf.HoldTime[m_enSystemAlarmKind].hour = 0x00;

        if(m_enNewStatu == OFF) {
            g_stSystemAlarmsInf.AlarmCode &= ~(1 << (u8)m_enSystemAlarmKind);
        } else {
            g_stSystemAlarmsInf.AlarmCode |= (1 << (u8)m_enSystemAlarmKind);
            StartRunningBeepAlarm(m_enAlarmGrade);//报警码不为零,开始报警
        }
    } else { //否则什么也不做
    }
}

/*
***************************************************************************************************
*                                      GetAlarmStatu()
*
* Description:  get the statu of a kind of alarm.
*
* Arguments  :  m_enSystemAlarmKind - the kind of the alarm.
*
* Returns    :  the statu of the alarm.
*
* Note(s)    :  none.
***************************************************************************************************
*/
SWITCH_TYPE_VARIABLE_Typedef GetAlarmStatu(SYSTEM_ALARM_ADDR_Typedef m_enSystemAlarmKind)
{
//  return stSYSTEM_ALARM[m_enSystemAlarmKind].FlagStatu;
    return (SWITCH_TYPE_VARIABLE_Typedef)((g_stSystemAlarmsInf.AlarmCode >> (u8)m_enSystemAlarmKind) & 0x1);
}

/*
***************************************************************************************************
*                                      GetAlarmHoldTime()
*
* Description:  get the hold time of a kind of alarm.
*
* Arguments  :  m_enSystemAlarmKind - the kind of the alarm.
*
* Returns    :  the hold time of the kind of alarm.
*
* Note(s)    :  none.
***************************************************************************************************
*/
SYSTEM_TIME_Typedef GetAlarmHoldTime(SYSTEM_ALARM_ADDR_Typedef m_enSystemAlarmKind)
{
    return g_stSystemAlarmsInf.HoldTime[m_enSystemAlarmKind];
}

/*
***************************************************************************************************
*                                      GetRunAlarmCode()
*
* Description:  get the alarm time.
*
* Arguments  :  none.
*
* Returns    :  the alarm code.
*
* Note(s)    :  none.
***************************************************************************************************
*/
uint32_t GetRunAlarmCode(void)
{
    return g_stSystemAlarmsInf.AlarmCode;
}

/*
***************************************************************************************************
*                                      ResetSystemIsolatedGeneratedEnergyThisTime()
*
* Description:  reset the isolated generated energy this time.
*
* Arguments  :  none.
*
* Returns    :  none.
*
* Note(s)    :  none.
***************************************************************************************************
*/
void ResetSystemIsolatedGeneratedEnergyThisTime(void)
{
    g_fSystemIsolatedGeneratedEnergyThisTime = 0.0;
}

/*
***************************************************************************************************
*                                      UpdateSystemIsolatedGeneratedEnergyThisTime()
*
* Description:  update the isolated generated energy this time.
*
* Arguments  :  none.
*
* Returns    :  none.
*
* Note(s)    :  none.
***************************************************************************************************
*/
void UpdateSystemIsolatedGeneratedEnergyThisTime()
{
    if(g_eStackWorkStatu == EN_IN_WORK) {
        g_fSystemIsolatedGeneratedEnergyThisTime += ((double) GetCurrentPower()) / 3600 / 1000 * 1;
        //3600为每小时的分钟数，1000为1kW, 1为每次更新的时间间隔为1s。
    }
}

/*
***************************************************************************************************
*                                      GetIsolatedGenratedEnergyThisTime()
*
* Description:  get the isolated generated energy this time.
*
* Arguments  :  none.
*
* Returns    :  double.
*
* Note(s)    :  none.
***************************************************************************************************
*/
float GetIsolatedGenratedEnergyThisTime()
{
    return g_fSystemIsolatedGeneratedEnergyThisTime;
}

/*
***************************************************************************************************
*                                      SetSelfCheckCodeBit()
*
* Description:  set specified bit of the self-check code.
*
* Arguments  :  select the bit of the code.
*
* Returns    :  none.
*
* Note(s)    :  none.
***************************************************************************************************
*/
void SetDevSelfCheckSensorStatusCodeBit(uint8_t i_u8BitNmb)
{
    if(i_u8BitNmb <= 8) {
        g_stSelfCheckCode.DevSelfCheckSensorStatusCode  |= (1 << i_u8BitNmb);
    }
}

void SetMachinePartASelfCheckCodeBit(uint8_t i_u8BitNmb)
{
    if(i_u8BitNmb <= 32) {
        g_stSelfCheckCode.MachinePartASelfCheckCode  |= (1 << i_u8BitNmb);
    } else {
    }
}

void SetMachinePartBSelfCheckCodeBit(uint8_t i_u8BitNmb)
{
    if(i_u8BitNmb <= 32) {
        g_stSelfCheckCode.MachinePartBSelfCheckCode  |= (1 << i_u8BitNmb);
    } else {
    }
}
/*
***************************************************************************************************
*                                      ResetSelfCheckCodeBit()
*
* Description:  reset specified bit of the self-check code.
*
* Arguments  :  select the bit of the code.
*
* Returns    :  none.
*
* Note(s)    :  none.
***************************************************************************************************
*/
void ResetDevSelfCheckSensorStatusCodeBit(uint8_t i_u8BitNmb)
{
    if(i_u8BitNmb <= 8) {
        g_stSelfCheckCode.DevSelfCheckSensorStatusCode  &= ~(1 << i_u8BitNmb);
    } else {
    }
}

void ResetMachinePartASelfCheckCodeBit(uint8_t i_u8BitNmb)
{
    if(i_u8BitNmb <= 32) {
        g_stSelfCheckCode.MachinePartASelfCheckCode  &= ~(1 << i_u8BitNmb);
    } else {
    }
}

void ResetMachinePartBSelfCheckCodeBit(uint8_t i_u8BitNmb)
{
    if(i_u8BitNmb <= 32) {
        g_stSelfCheckCode.MachinePartBSelfCheckCode  &= ~(1 << i_u8BitNmb);
    } else {
    }
}
/*
***************************************************************************************************
*                           SELF_CHECK_CODE_Typedef GetSysSelfCheckCode()
*
* Description:  reset specified bit of the self-check code.
*
* Arguments  :  select the bit of the code.
*
* Returns    :  none.
*
* Note(s)    :  none.
***************************************************************************************************
*/
SELF_CHECK_CODE_Typedef GetSysSelfCheckCode()
{
    SYSTEM_WORK_MODE_Typedef    eWorkMode;
    eWorkMode = GetWorkMode();

    switch((u8)eWorkMode) {
        case EN_WORK_MODE_HYDROGEN_PRODUCER_AND_FUEL_CELL:
            //不做屏蔽操作
            break;

        case EN_WORK_MODE_HYDROGEN_PRODUCER:
            g_stSelfCheckCode.MachinePartBSelfCheckCode = 0;
            break;

        case EN_WORK_MODE_FUEL_CELL:
            g_stSelfCheckCode.MachinePartASelfCheckCode = 0;
            break;

        default:
            break;
    }

    return g_stSelfCheckCode;
}
/*
***************************************************************************************************
*                                      SetCtlAndCommunicaeCodeWorkModeSection()
*
* Description:  set the section, that accord to the work mode, of the self-check code.
*
* Arguments  :  the new statu of the work mode.
*
* Returns    :  none.
*
* Note(s)    :  none.
***************************************************************************************************
*/
void SetCtlAndCommunicaeCodeWorkModeSection(SYSTEM_WORK_MODE_Typedef i_u8NewStatu)
{
    CPU_SR_ALLOC();
    CPU_CRITICAL_ENTER();
    g_u16CtlAndCommunicateCode &= 0x8EFFu;
    g_u16CtlAndCommunicateCode |= ((uint16_t)i_u8NewStatu << 11);
    CPU_CRITICAL_EXIT();
}


/*
***************************************************************************************************
*                                      GetSystemRunningStatuCode()
*
* Description:  get the system running statu code.
*
* Arguments  :  none.
*
* Returns    :  running statu code.
*
* Note(s)    :  none.
***************************************************************************************************
*/
u32 GetSystemRunningStatuCode(void)
{
    return g_SystemRunningStatuCode;
}

/*
***************************************************************************************************
*                                      SetSystemRunningStatuCodeBit()
*
* Description:  set specified bit of the system running statu code.
*
* Arguments  :  select the bit of the code.
*
* Returns    :  none.
*
* Note(s)    :  none.
***************************************************************************************************
*/
void SetSystemRunningStatuCodeBit(uint8_t i_u8BitNmb)
{
    g_SystemRunningStatuCode |= 1u << i_u8BitNmb;
}

/*
***************************************************************************************************
*                                      ResetSystemRunningStatuCodeBit()
*
* Description:  reset specified bit of the system running statu code.
*
* Arguments  :  select the bit of the code.
*
* Returns    :  none.
*
* Note(s)    :  none.
***************************************************************************************************
*/
void ResetSystemRunningStatuCodeBit(uint8_t i_u8BitNmb)
{
    g_SystemRunningStatuCode &= ~(1u << i_u8BitNmb);
}


/*
***************************************************************************************************
*                                      ResetSystemRunningStatuCode()
*
* Description:  reset the system running statu code.
*
* Arguments  :  none.
*
* Returns    :  none.
*
* Note(s)    :  none.
***************************************************************************************************
*/
void ResetSystemRunningStatuCode(void)
{
    g_SystemRunningStatuCode = 0;
}
/*
***************************************************************************************************
*                             SetSystemRunningStatuCodeSysWorkStatuSection()
*
* Description:  reset the section, that accord to the work statu, of  the system running statu code.
*
* Arguments  :  select the bit of the code.
*
* Returns    :  none.
*
* Note(s)    :  none.
***************************************************************************************************
*/
void SetSystemRunningStatuCodeSysWorkStatuSection(SYSTEM_WORK_STATU_Typedef i_u8NewStatu)
{
    CPU_SR_ALLOC();

    if(i_u8NewStatu <= 7) { //判断是否为正常工作状态
        CPU_CRITICAL_ENTER();
        g_SystemRunningStatuCode &= 0x0FFFFFFFu;
        g_SystemRunningStatuCode |= (i_u8NewStatu << 28);
        CPU_CRITICAL_EXIT();
    } else {
    }
}

/*
***************************************************************************************************
*                             SetSystemRunningStatuCodeStackRunStatuSection()
*
* Description:  Set the system running statu code.
*
* Arguments  :  select the bit of the code.
*
* Returns    :  none.
*
* Note(s)    :  none.
***************************************************************************************************
*/
void SetSystemRunningStatuCodeStackRunStatuSection(SYSTEM_WORK_STATU_Typedef i_u8NewStatu)
{
    CPU_SR_ALLOC();

    if(i_u8NewStatu <= 7) { //判断是否为正常工作状态
        CPU_CRITICAL_ENTER();
        g_SystemRunningStatuCode &= 0xF0FFFFFFu;
        g_SystemRunningStatuCode |= (i_u8NewStatu << 24);
        CPU_CRITICAL_EXIT();
    } else {
    }
}
/*
***************************************************************************************************
*                                      SetSystemRunningStatuCodeBit()
*
* Description:  set specified bit of the system running statu code.
*
* Arguments  :  select the bit of the code.
*
* Returns    :  none.
*
* Note(s)    :  none.
***************************************************************************************************
*/
void SetConrolAndCommunicateStatuCodeBit(uint8_t i_u8BitNmb)
{
    g_u16CtlAndCommunicateCode |= (1u << i_u8BitNmb);
}

/*
***************************************************************************************************
*                                      ResetConrolAndCommunicateStatuCodeBit()
*
* Description:  reset specified bit of the system running statu code.
*
* Arguments  :  select the bit of the code.
*
* Returns    :  none.
*
* Note(s)    :  none.
***************************************************************************************************
*/
void ResetConrolAndCommunicateStatuCodeBit(uint8_t i_u8BitNmb)
{
    g_u16CtlAndCommunicateCode &= ~(1u << i_u8BitNmb);
}

/*
***************************************************************************************************
*                                      GetConrolAndCommunicateStatuCode()
*
* Description:  reset specified bit of the system running statu code.
*
* Arguments  :  select the bit of the code.
*
* Returns    :  none.
*
* Note(s)    :  none.
***************************************************************************************************
*/
uint16_t GetConrolAndCommunicateStatuCode()
{
    return g_u16CtlAndCommunicateCode;
}

/*
***************************************************************************************************
*                               CalcStackOptimumTemperatureByCurrent()
*
* Description : Calculate the optimum temperature that the stack can accept from the current.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Note(s)     : Calculated according to the stack manual curve.
***************************************************************************************************
*/
uint8_t CalcStackOptimumTemperatureByCurrent(void)
{
    float m_StackCurrent;
    float m_StackOptimumTemperature;
    
    m_StackCurrent = GetSrcAnaSig(STACK_CURRENT);
    m_StackOptimumTemperature = (0.53 * m_StackCurrent) + 26.01;

    return (uint8_t)m_StackOptimumTemperature;
}

/*
***************************************************************************************************
*                               GetStackMaxTemperatureByCurrent()
*
* Description : Calculate the maximum temperature that the stack can withstand from the current.
*
* Argument(s) : none.
*
* Return(s)   : fStackMaxTemperature.
*
* Note(s)     : none.
***************************************************************************************************
*/
uint8_t CalcStackMaximumTemperatureByCurrent(void)
{
    float fStackCurrent;
    float fStackMaxTemperature;

    fStackCurrent = GetSrcAnaSig(STACK_CURRENT);

    if(fStackCurrent < 65.3) {
        fStackMaxTemperature = (0.352 * fStackCurrent) + 52;
    } else {
        fStackMaxTemperature = 75;
    }

    return (uint8_t)fStackMaxTemperature;
}

/*
***************************************************************************************************
*                               GetStackMaxTemperatureByCurrent()
*
* Description : Calculate the minimum temperature that the stack can withstand from the current.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Note(s)     : none.
***************************************************************************************************
*/
uint8_t CalcStackMinimumTemperatureByCurrent(void)
{
    float fStackCurrent;
    float fStackMinTemperature;
    fStackCurrent = GetSrcAnaSig(STACK_CURRENT);

    if(fStackCurrent < 65.3) {
        fStackMinTemperature = (0.532 * fStackCurrent) + 6;
    } else if(fStackCurrent < 87.1) {
        fStackMinTemperature = (1.782 * fStackCurrent) - 90.2;
    } else {
        fStackMinTemperature = 65;
    }

    return (uint8_t)fStackMinTemperature;
}



/*
***************************************************************************************************
*                               GetSysErrCode()
*
* Description : Calculate the minimum temperature that the stack can withstand from the current.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Note(s)     : none.
***************************************************************************************************
*/
uint32_t GetSysErrCode()
{
    return g_u32SysErrCode;
}

void SysErrStatuCmd(SYSTEM_ALARM_ADDR_Typedef i_eErrKind, SWITCH_TYPE_VARIABLE_Typedef i_eNewStatu)
{
    if((g_u32SysErrCode ^ (i_eNewStatu << (uint8_t)i_eErrKind)) != 0) {
        if(i_eNewStatu == ON) {
            g_u32SysErrCode |= (1 << i_eErrKind);
        } else {
            g_u32SysErrCode &= ~(1 << i_eErrKind);
        }
    }
}

void SetShutDownRequestMaskStatu(SYSTEM_ALARM_ADDR_Typedef i_ErrKind, REAL_TIME_REQUEST_MASK_Typedef i_eNewStatu, uint16_t i_u8DelaySeconds)
{
    if(i_eNewStatu == EN_DELAY) { //若更新至延时状态，则开启计时，若延时状态未变，则更新延时时间即可
        g_stShutDownRequestWaitMask[i_ErrKind].MaskStatu = i_eNewStatu;
        g_stShutDownRequestWaitMask[i_ErrKind].DelaySecond = i_u8DelaySeconds;//记录延时等待的时间
        g_stShutDownRequestWaitMask[i_ErrKind].RecordStartTime = GetSystemTime();//记录延时开始的时间
    } else { //切换为非延时状态，则清零计时
        g_stShutDownRequestWaitMask[i_ErrKind].MaskStatu = i_eNewStatu;
        g_stShutDownRequestWaitMask[i_ErrKind].DelaySecond = 0;
        g_stShutDownRequestWaitMask[i_ErrKind].RecordStartTime.hour = 0;
        g_stShutDownRequestWaitMask[i_ErrKind].RecordStartTime.minute = 0;
        g_stShutDownRequestWaitMask[i_ErrKind].RecordStartTime.second = 0;
    }
}

REAL_TIME_REQUEST_MASK_Typedef GetShutDownRequestMaskStatu(SYSTEM_ALARM_ADDR_Typedef i_ErrKind)
{
    SYSTEM_TIME_Typedef stCurrentTime;

    if(g_stShutDownRequestWaitMask[i_ErrKind].MaskStatu == EN_DELAY) {
        stCurrentTime = GetSystemTime();

        if((g_stShutDownRequestWaitMask[i_ErrKind].DelaySecond > 0)
                && ((((stCurrentTime.hour - g_stShutDownRequestWaitMask[i_ErrKind].RecordStartTime.hour) * 60
                      + stCurrentTime.minute - g_stShutDownRequestWaitMask[i_ErrKind].RecordStartTime.minute) * 60
                     + stCurrentTime.second - g_stShutDownRequestWaitMask[i_ErrKind].RecordStartTime.second)
                    >= g_stShutDownRequestWaitMask[i_ErrKind].DelaySecond)) { //若延时到达，这自动切换为非屏蔽状态
            g_stShutDownRequestWaitMask[i_ErrKind].MaskStatu = EN_UN_MASK;
            g_stShutDownRequestWaitMask[i_ErrKind].DelaySecond = 0;
            g_stShutDownRequestWaitMask[i_ErrKind].RecordStartTime.hour = 0;
            g_stShutDownRequestWaitMask[i_ErrKind].RecordStartTime.minute = 0;
            g_stShutDownRequestWaitMask[i_ErrKind].RecordStartTime.second = 0;
            BSP_BuzzerOff();
        } else { //否则继续等待
        }
    }

    return g_stShutDownRequestWaitMask[i_ErrKind].MaskStatu;
}
/*
***************************************************************************************************
*                             SystemTimeStatTaskCreate()
*
* Description:  create the system time statistics task.
*
* Arguments  :  none.
*
* Returns    :  none.
*
* Note(s)    :  none.
***************************************************************************************************
*/
void SystemTimeStatTaskCreate(void)
{
    OS_ERR  err;

    OSTaskCreate((OS_TCB *)&SysTimeStatTaskTCB,                    // Create the start task
                 (CPU_CHAR *)"System time statistic Task",
                 (OS_TASK_PTR) SysTimeStatTask,
                 (void *) 0,
                 (OS_PRIO) SYSTEM_TIME_STATISTIC_TASK_PRIO,
                 (CPU_STK *)&SysTimeStatTaskStk[0],
                 (CPU_STK_SIZE) SYSTEM_TIME_STATISTIC_TASK_STK_SIZE / 10,
                 (CPU_STK_SIZE) SYSTEM_TIME_STATISTIC_TASK_STK_SIZE,
                 (OS_MSG_QTY) 5u,
                 (OS_TICK) 0u,
                 (void *) 0,
                 (OS_OPT)(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR *)&err);
    APP_TRACE_INFO(("Created system time statistic task, and err code is %d...\n\r", err));
}

/*
***************************************************************************************************
*                             SysTimeStatTask()
*
* Description:  the system time statistics task.
*               
* Arguments  :  none.
*
* Returns    :  none.
*
* Note(s)    :
***************************************************************************************************
*/
static  void  SysTimeStatTask(void *p_arg)
{
    OS_ERR      err;
    uint32_t    u32AlarmCode = 0;
    uint8_t     i;

    OSSemCreate(&g_stSystemTimeUpdateSem,
                "System time update sem",
                0,
                &err);

    while(DEF_TRUE) {
        OSSemPend(&g_stSystemTimeUpdateSem,
                  OS_CFG_TICK_RATE_HZ,
                  OS_OPT_PEND_BLOCKING,
                  NULL,
                  &err);

        if(err == OS_ERR_NONE) {
            UpdateSysTime(&g_stSystemTime);
            UpdateSystemIsolatedGeneratedEnergyThisTime();

            if((g_eSystemWorkMode == EN_WORK_MODE_HYDROGEN_PRODUCER_AND_FUEL_CELL)
                    || (g_eSystemWorkMode == EN_WORK_MODE_HYDROGEN_PRODUCER)) {
                if((g_eSystemWorkStatu == EN_START_PRGM_ONE_FRONT)
                        || (g_eSystemWorkStatu == EN_START_PRGM_ONE_BEHIND)
                        || (g_eSystemWorkStatu == EN_START_PRGM_TWO)
                        || (g_eSystemWorkStatu == EN_RUNNING)) {
                    //更新制氢时间
                    UpdateSysTime(&g_stHydrgProduceTimeThisTime);
                    UpdateSysTime(&g_stHydrgProduceTimeTotal);

                    u32AlarmCode = g_stSystemAlarmsInf.AlarmCode;
                    i = 16;
                    u32AlarmCode &= 0xFFFFFFF;  //滤掉前面4位与制氢机警报无关的公共组警报
                    u32AlarmCode >>= 16;            //与制氢机相关的警报从bit16开始

                    while(u32AlarmCode != 0) {
                        if((u32AlarmCode & 1) == 1) {
                            UpdateSysTime(&g_stSystemAlarmsInf.HoldTime[i]);
                        }

                        u32AlarmCode >>= 1;
                        i++;
                    }

                    //          if(g_stSystemAlarmsInf.HoldTime[FUEL_SHORTAGE_ALARM].minute >= 30)//液位低持续超过30分钟
                    //          {
                    //              CmdShutDown();
                    //          }
                }
            }

            if(g_eStackWorkStatu == EN_IN_WORK) {
                UpdateSysTime(&g_stStackProductTimeThisTime);
                UpdateSysTime(&g_stStackProductTimeTotal);
                u32AlarmCode = g_stSystemAlarmsInf.AlarmCode;

                i = 0;
                u32AlarmCode &= 0xFFFF;//滤掉前面4位与电堆警报无关的公共组警报和制氢机警报

                while(u32AlarmCode != 0) {
                    if((u32AlarmCode & 1) == 1) {
                        UpdateSysTime(&g_stSystemAlarmsInf.HoldTime[i]);
                    }

                    u32AlarmCode >>= 1;
                    i++;
                }
            }

        }
    }
}

/*
***************************************************************************************************
*                             UpdateSysTime()
*
* Description:  update all kinds of time in the system.
*
* Arguments  :  kind of time.
*
* Returns    :  none.
*
* Note(s)    :  none.
***************************************************************************************************
*/
static void UpdateSysTime(SYSTEM_TIME_Typedef *i_stTargetTime)
{
    CPU_SR_ALLOC();
    CPU_CRITICAL_ENTER();

    if(++i_stTargetTime->second >= (uint16_t) 60) {
        i_stTargetTime->second = 0;

        if(++i_stTargetTime->minute >= 60) {
            i_stTargetTime->minute = 0;
            ++i_stTargetTime->hour;
        }
    }

    CPU_CRITICAL_EXIT();
}

/*
***************************************************************************************************
*                             GetCurrentPower()
*
* Description:  get current power.
*
* Arguments  :  none.
*
* Returns    :  float.
*
* Note(s)    :  none.
***************************************************************************************************
*/
float GetCurrentPower(void)
{
    return GetSrcAnaSig(STACK_VOLTAGE) * GetSrcAnaSig(STACK_CURRENT);
}
/******************* (C) COPYRIGHT 2015 Guangdong Hydrogen *****END OF FILE****/

