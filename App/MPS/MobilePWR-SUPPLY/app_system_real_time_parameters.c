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
* Filename      : app_system_real_time_parameters.c
* Version       : V1.00
* Programmer(s) : SunKing.Yun
*********************************************************************************************************
*/
/*
*********************************************************************************************************
*                                           INCLUDE FILES
*********************************************************************************************************
*/
#include "stm32f10x.h"
#include "stm32f10x_conf.h"
#include "app_system_run_cfg_parameters.h"
#include "app_system_real_time_parameters.h"
#include "app_wireness_communicate_task.h"
#include "includes.h"
#include "bsp.h"
#include <cpu.h>
#include <bsp_ana_sensor.h>

/*
*********************************************************************************************************
*                                           MACRO DEFINITIONS
*********************************************************************************************************
*/
#define SYSTEM_TIME_STATISTIC_TASK_STK_SIZE     64

/*
*********************************************************************************************************
*                                         OS-RELATED    VARIABLES
*********************************************************************************************************
*/
static      OS_TCB      SysTimeStatTaskTCB;

OS_SEM      g_stSystemTimeUpdateSem;

static      CPU_STK     SysTimeStatTaskStk[SYSTEM_TIME_STATISTIC_TASK_STK_SIZE];

/*
*********************************************************************************************************
*                                           LOCAL VARIABLES
*********************************************************************************************************
*/
static  SYSTEM_CONTROL_MODE_Typedef     g_eSystemControlMode = EN_CONTROL_MODE_AUTO;
static  SYSTEM_WORK_MODE_Typedef        g_eSystemWorkMode = EN_WORK_MODE_HYDROGEN_PRODUCER_AND_FUEL_CELL;

// ϵͳʱ�����
static  SYSTEM_TIME_Typedef             g_stSystemTime = {0, 0, 0};                     //ϵͳ����ʱ��

static  SYSTEM_TIME_Typedef             g_stHydrgProduceTimeThisTime = {0, 0, 0};        //��������ʱ��
static  SYSTEM_TIME_Typedef             g_stHydrgProduceTimeTotal = {0, 0, 0};          //�ۼ�����ʱ��

static  SYSTEM_TIME_Typedef             g_stStackProductTimeThisTime = {0, 0, 0};       //���η���ʱ��
static  SYSTEM_TIME_Typedef             g_stStackProductTimeTotal = {0, 0, 0};       //�ۼƷ���ʱ��

//ϵͳ�ۼƴ���
static          u16                     g_u16SystemWorkTimes = 0;                   //ϵͳ�ۼ����д���
static          u16                     g_u16HydrgProducerWorkTimes = 0;   //��������д���
static          u16                     g_u16StackWorkTimes = 0;            //������д���

//ϵͳʵʱ����
//  SELF_CHECK_CODE_Typedef             g_stSysSelfCheckCode;
static          uint64_t                g_u64SysSelfCheckCode = 0;

//ϵͳʵʱ����
static  RUNNING_ALARM_STATUS_Typedef    g_stSystemAlarmsInf;    //�����뼰����ʱ��
//  ϵͳ״̬��
static          u32                     g_SystemRunningStatuCode = 0x80000000;
//ϵͳ���н׶α�ʶ
static  SYSTEM_WORK_STATU_Typedef       g_eSystemWorkStatu = EN_WAITTING_COMMAND;
static  STACK_WORK_STATU_Typedef        g_eStackWorkStatu = EN_NOT_IN_WORK;

//״̬��ʾ��Ļ����״̬
static WHETHER_TYPE_VARIABLE_Typedef g_eExternalScreenUpdateStatu = YES;

//ϵͳ���η�����(KWh)
static          double                  g_dSystemIsolatedGeneratedEnergyThisTime = 0.0;

static          uint8_t                 g_u8WaitWorkModeSelectSwitch = DEF_DISABLED;

/*
*********************************************************************************************************
*                                         FUNCTION PROTOTYPES
*********************************************************************************************************
*/
static          void                    UpdateSysTime(SYSTEM_TIME_Typedef *);

static          void                    SetSelfCheckCodeWorkModeSection(SYSTEM_WORK_MODE_Typedef);

static          void                    SysTimeStatTask(void *p_arg);

/*
*********************************************************************************************************
*                                      GetWorkMode()
*
* Description:  Get the work mode of the system.
*
* Arguments  :  none
*
* Returns    :  the work mode of the system.
*********************************************************************************************************
*/
SYSTEM_WORK_MODE_Typedef GetWorkMode(void)
{
    return g_eSystemWorkMode;
}

/*
*********************************************************************************************************
*                                      SetWorkMode()
*
* Description:  Set the work mode of the system.
*
* Arguments  :  the excepted work mode of the system.
*
* Returns    :  none.
*********************************************************************************************************
*/
void SetWorkMode(SYSTEM_WORK_MODE_Typedef i_eNewWorkModeStatu)
{
    CPU_SR_ALLOC();

    if( i_eNewWorkModeStatu >= 3 )//�ж�
    {
        i_eNewWorkModeStatu = EN_WORK_MODE_MALFUNCTION;
    }

    CPU_CRITICAL_ENTER();
    g_eSystemWorkMode = i_eNewWorkModeStatu;
    SetSelfCheckCodeWorkModeSection(i_eNewWorkModeStatu);
    CPU_CRITICAL_EXIT();
}

/*
*********************************************************************************************************
*                                      SetWorkModeWaittingForSelectFlag()
*
* Description:  Set the flag that the system is waitting for the control side to select the work mode.
*
* Arguments  :  none.
*
* Returns    :  none.
*
* Note(s)    :  The flag is included in the message that will send to the control side.
*********************************************************************************************************
*/
void SetWorkModeWaittingForSelectFlag(void)
{
    CPU_SR_ALLOC();
    CPU_CRITICAL_ENTER();
    g_u8WaitWorkModeSelectSwitch = DEF_ENABLED;
    SetSystemRunningStatuCodeBit(RuningStatuCodeControlModeSelectWaitFlagBit);
    CPU_CRITICAL_EXIT();
}

/*
*********************************************************************************************************
*                                      ResetWorkModeWaittingForSelectFlag()
*
* Description:  Reset the flag that the system is waitting for the control side to select the work mode.
*               �رյȴ���־
* Arguments  :  none.
*
* Returns    :  none.
*
* Note(s)    :  The flag is included in the message that will send to the control side.
*********************************************************************************************************
*/
void ResetWorkModeWaittingForSelectFlag(void)
{
    CPU_SR_ALLOC();
    CPU_CRITICAL_ENTER();
    g_u8WaitWorkModeSelectSwitch = DEF_DISABLED;
    ResetSystemRunningStatuCodeBit(RuningStatuCodeControlModeSelectWaitFlagBit);
    CPU_CRITICAL_EXIT();
}

/*
*********************************************************************************************************
*                                      GetWorkModeWaittingForSelectFlag()
*
* Description:  Get the flag that the system is waitting for the control side to select the work mode.
*
* Arguments  :  none.
*
* Returns    :  none.
*
* Note(s)    :  The flag is included in the message that will send to the control side.
*********************************************************************************************************
*/
uint8_t GetWorkModeWaittingForSelectFlag(void)
{
    return g_u8WaitWorkModeSelectSwitch;
}

/*
*********************************************************************************************************
*                                      ControlModeTurnOver()
*
* Description:  Turn over the control mode.
*               ����ģʽ��ת
* Arguments  :  none.
*
* Returns    :  none.
*
* Note(s)    :  none.
*********************************************************************************************************
*/
void ControlModeTurnOver(void)
{
    if(g_eSystemControlMode == EN_CONTROL_MODE_AUTO)
    {
        g_eSystemControlMode = EN_CONTROL_MODE_MANNUAL;
        SetSystemRunningStatuCodeBit(RuningStatuCodeCtrlMode);
    }
    else
    {
        g_eSystemControlMode = EN_CONTROL_MODE_AUTO;
        ResetSystemRunningStatuCodeBit(RuningStatuCodeCtrlMode);
    }
}

/*
*********************************************************************************************************
*                                      GetControlMode()
*
* Description:  Get control mode of the system.
*
* Arguments  :  none.
*
* Returns    :  none.
*
* Note(s)    :  none.
*********************************************************************************************************
*/
SYSTEM_CONTROL_MODE_Typedef GetControlMode(void)
{
    return g_eSystemControlMode;
}

/*
*********************************************************************************************************
*                                      GetSystemTime()
*
* Description:  Get the system run time that from the boot time.
*               ��ȡϵͳ����ʱ��
* Arguments  :  none.
*
* Returns    :  none.
*
* Note(s)    :  none.
*********************************************************************************************************
*/

SYSTEM_TIME_Typedef GetSystemTime(void)
{
    return g_stSystemTime;
}

/*
*********************************************************************************************************
*                                      ResetHydrgProduceTimeThisTime()
*
* Description:  Reset the hydrogen produce time this produce cycle.
*               ��λ��������ʱ��
* Arguments  :  none.
*
* Returns    :  none.
*
* Note(s)    :  none.
*********************************************************************************************************
*/
void ResetHydrgProduceTimeThisTime()
{
    g_stHydrgProduceTimeThisTime.second = 0;
    g_stHydrgProduceTimeThisTime.minute = 0;
    g_stHydrgProduceTimeThisTime.hour = 0;
}

/*
*********************************************************************************************************
*                                      GetHydrgProduceTimeThisTime()
*
* Description:  Get the hydrogen produce time this produce cycle.//��ȡ��������ʱ��
*
* Arguments  :  none.
*
* Returns    :  hydrogen produce time this cycle.
*
* Note(s)    :  none.
*********************************************************************************************************
*/
SYSTEM_TIME_Typedef GetHydrgProduceTimeThisTime(void)
{
    return g_stHydrgProduceTimeThisTime;
}

/*
*********************************************************************************************************
*                                      GetHydrgProduceTimeTotal()
*
* Description:  Get the hydrogen produce time total.//��ȡ�ۼ�����ʱ��
*
* Arguments  :  none.
*
* Returns    :  hydrogen produce time total.
*
* Note(s)    :  none.
*********************************************************************************************************
*/
SYSTEM_TIME_Typedef GetHydrgProduceTimeTotal(void)
{
    return g_stHydrgProduceTimeTotal;
}

/*
*********************************************************************************************************
*                                      ResetStackProductTimeThisTime()
*
* Description:  Reset the stack product time this produce cycle.
*               ��λ���η���ʱ��
* Arguments  :  none.
*
* Returns    :  none.
*
* Note(s)    :  none.
*********************************************************************************************************
*/
void ResetStackProductTimeThisTime()
{
    g_stStackProductTimeThisTime.second = 0;
    g_stStackProductTimeThisTime.minute = 0;
    g_stStackProductTimeThisTime.hour = 0;
}

/*
*********************************************************************************************************
*                                      GetStackProductTimeThisTime()
*
* Description:  Get the stack product time this produce cycle.//��ȡ���η���ʱ��
*
* Arguments  :  none.
*
* Returns    :  stack produce time this cycle.
*
* Note(s)    :  none.
*********************************************************************************************************
*/
SYSTEM_TIME_Typedef GetStackProductTimeThisTime(void)
{
    return g_stStackProductTimeThisTime;
}

/*
*********************************************************************************************************
*                                      ResetStackProductTimeThisTime()
*
* Description:  Get the stack product time total.//��ȡ�ۼƷ���ʱ��
*
* Arguments  :  none.
*
* Returns    :  stack produce time total.
*
* Note(s)    :  none.
*********************************************************************************************************
*/
SYSTEM_TIME_Typedef GetStackProductTimeTotal(void)
{
    return g_stStackProductTimeTotal;
}

/*
*********************************************************************************************************
*                                      ResetSystemWorkTimes()
*
* Description:  Reset the system work times.//����ϵͳ���д���
*
* Arguments  :  none.
*
* Returns    :  none.
*
* Note(s)    :  none.
*********************************************************************************************************
*/
void ResetSystemWorkTimes(void)
{
    g_u16SystemWorkTimes = 0;
    SaveSystemWorkTimes();
}

/*
*********************************************************************************************************
*                                      LoadSystemWorkTimes()
*
* Description:  load the system work times.//�����ۼƹ�������
*
* Arguments  :  newest work time number.
*
* Returns    :  none.
*
* Note(s)    :  none.
*********************************************************************************************************
*/
void LoadSystemWorkTimes(u16 u16WorkTimes)
{
    g_u16SystemWorkTimes = u16WorkTimes;
}

/*
*********************************************************************************************************
*                                      SystemWorkTimesInc()
*
* Description:  update the system work times.
*               ϵͳ���д��� + 1
* Arguments  :  none.
*
* Returns    :  none.
*
* Note(s)    :  none.
*********************************************************************************************************
*/
void SystemWorkTimesInc(void)
{
    g_u16SystemWorkTimes ++;
    SaveSystemWorkTimes();
}

/*
*********************************************************************************************************
*                                      SystemWorkTimesInc()
*
* Description:  get the system work times.//��ȡϵͳ���д���
*
* Arguments  :  none.
*
* Returns    :  the system work times
*
* Note(s)    :  none.
*********************************************************************************************************
*/
uint16_t GetSystemWorkTimes()
{
    return g_u16SystemWorkTimes;//YSTEM_RUNNING_TIMES;
}

/*
*********************************************************************************************************
*                                      ResetHydrgProducerWorkTimes()
*
* Description:  reset the hydrogen produce times.//������������д���
*
* Arguments  :  none.
*
* Returns    :  none.
*
* Note(s)    :  none.
*********************************************************************************************************
*/
void ResetHydrgProducerWorkTimes(void)
{
    g_u16HydrgProducerWorkTimes = 0;
}

/*
*********************************************************************************************************
*                                      LoadHydrgProducerWorkTimes()
*
* Description:  load the hydrogen produce times.//������������д���
*
* Arguments  :  newest the work time number.
*
* Returns    :  none.
*
* Note(s)    :  none.
*********************************************************************************************************
*/
void LoadHydrgProducerWorkTimes(u16 u16WorkTimes)
{
    g_u16HydrgProducerWorkTimes = u16WorkTimes;
}

/*
*********************************************************************************************************
*                                      HydrgProducerWorkTimesInc()
*
* Description:  increase the hydrogen produce times.//��������д��� + 1
*
* Arguments  :  none.
*
* Returns    :  none.
*
* Note(s)    :  none.
*********************************************************************************************************
*/
void HydrgProducerWorkTimesInc(void)
{
    g_u16HydrgProducerWorkTimes ++;
}

/*
*********************************************************************************************************
*                                      GetHydrgProducerWorkTimes()
*
* Description:  get the hydrogen produce times.//��ȡϵͳ���д���
*
* Arguments  :  none.
*
* Returns    :  unsigned short int.
*
* Note(s)    :  none.
*********************************************************************************************************
*/
uint16_t GetHydrgProducerWorkTimes()
{
    return g_u16HydrgProducerWorkTimes;
}

/*
*********************************************************************************************************
*                                      ResetStackWorkTimes()
*
* Description:  reset the stack produce times.//���õ�����д���
*
* Arguments  :  none.
*
* Returns    :  none.
*
* Note(s)    :  none.
*********************************************************************************************************
*/
void ResetStackWorkTimes(void)
{
    g_u16StackWorkTimes = 0;
}

/*
*********************************************************************************************************
*                                      LoadStackWorkTimes()
*
* Description:  get the stack produce times.//��ȡϵͳ���д���
*
* Arguments  :  unsigned short int.
*
* Returns    :  none.
*
* Note(s)    :  none.
*********************************************************************************************************
*/
void LoadStackWorkTimes(u16 u16WorkTimes)
{
    g_u16StackWorkTimes = u16WorkTimes;
}

/*
*********************************************************************************************************
*                                      StackWorkTimesInc()
*
* Description:  increase the stack produce times.//������д��� + 1
*
* Arguments  :  none.
*
* Returns    :  none.
*
* Note(s)    :  none.
*********************************************************************************************************
*/
void StackWorkTimesInc(void)
{
    g_u16StackWorkTimes++;
}

/*
*********************************************************************************************************
*                                      GetStackWorkTimes()
*
* Description:  get the stack produce times.//��ȡ������д���
*
* Arguments  :  none.
*
* Returns    :  unsigned short int.
*
* Note(s)    :  none.
*********************************************************************************************************
*/
uint16_t GetStackWorkTimes()
{
    return g_u16StackWorkTimes;//YSTEM_RUNNING_TIMES;
}

/*
*********************************************************************************************************
*                                      SetSystemWorkStatu()
*
* Description:  set the system work statu.
*               �趨ϵͳ����״̬
* Arguments  :  enum type.
*
* Returns    :  none.
*
* Note(s)    :  none.
*********************************************************************************************************
*/
void SetSystemWorkStatu(SYSTEM_WORK_STATU_Typedef m_enNewStatu)
{
//  OS_ERR err;
    CPU_SR_ALLOC();

    if(m_enNewStatu >= 7)
    {
        m_enNewStatu = EN_SHUTTING_DOWN;
    }

    CPU_CRITICAL_ENTER();
    g_eSystemWorkStatu = m_enNewStatu;
    SetSystemRunningStatuCodeSysWorkStatuSection(g_eSystemWorkStatu);
    CPU_CRITICAL_EXIT();
}

/*
*********************************************************************************************************
*                                      GetSystemWorkStatu()
*
* Description:  get the system work statu.
*
* Arguments  :  none.
*
* Returns    :  enum type.
*
* Note(s)    :  none.
*********************************************************************************************************
*/
SYSTEM_WORK_STATU_Typedef GetSystemWorkStatu(void)
{
    return g_eSystemWorkStatu;
}



/*
*********************************************************************************************************
*                                      SetExternalScreenUpdateStatu()
*
* Description:  set the screen update statu.
*
* Arguments  :  none.
*
* Returns    :  enum type.
*
* Note(s)    :  none.
*********************************************************************************************************
*/
void SetExternalScreenUpdateStatu(WHETHER_TYPE_VARIABLE_Typedef i_NewStatu)
{
    CPU_SR_ALLOC();
    CPU_CRITICAL_ENTER();
    g_eExternalScreenUpdateStatu = i_NewStatu;
    //****������Ļ����״̬��?
    CPU_CRITICAL_EXIT();

}
/*
*********************************************************************************************************
*                                      GetExternalScreenUpdateStatu()
*
* Description:  get the screen update statu.
*
* Arguments  :  none.
*
* Returns    :  g_eExternalScreenUpdateStatu.
*
* Note(s)    :  none.
*********************************************************************************************************
*/
WHETHER_TYPE_VARIABLE_Typedef GetExternalScreenUpdateStatu(void)
{
    return g_eExternalScreenUpdateStatu;
}



/*
*********************************************************************************************************
*                                      SetStackWorkStatu()
*
* Description:  set the stack work statu.
*
* Arguments  :  enum type.
*
* Returns    :  none.
*
* Note(s)    :  none.
*********************************************************************************************************
*/
void SetStackWorkStatu(STACK_WORK_STATU_Typedef i_eNewStatu)
{
    CPU_SR_ALLOC();
    CPU_CRITICAL_ENTER();
    g_eStackWorkStatu = i_eNewStatu;
    SetSystemRunningStatuCodeBit(RuningStatuCodeStackStatuBit);
    CPU_CRITICAL_EXIT();
}

/*
*********************************************************************************************************
*                                      GetStackWorkStatu()
*
* Description:  get the stack work statu.
*
* Arguments  :  none.
*
* Returns    :  enum type.
*
* Note(s)    :  none.
*********************************************************************************************************
*/
STACK_WORK_STATU_Typedef GetStackWorkStatu(void)
{
    return g_eStackWorkStatu;
}

/*
*********************************************************************************************************
*                                      ResetAllAlarms()
*
* Description:  reset the all alarms statu.
*               ����ϵͳ����
* Arguments  :  none.
*
* Returns    :  none.
*
* Note(s)    :  none.
*********************************************************************************************************
*/
void ResetAllAlarms()
{
    int i;
    g_stSystemAlarmsInf.AlarmCode = 0;

    for(i = 0; i < 32; i++)//������Ϊ32λ������ʱ��ȫ������
    {
        g_stSystemAlarmsInf.HoldTime[i].second = 0x00;
        g_stSystemAlarmsInf.HoldTime[i].minute = 0x00;
        g_stSystemAlarmsInf.HoldTime[i].hour = 0x00;
    }
}

/*
*********************************************************************************************************
*                                      AlarmCmd()
*
* Description:  command a kind of alarm statu.
*               �趨ϵͳ����״̬��(ÿ��λ��Ӧ��ͬ�ı�����־)
* Arguments  :  m_enSystemAlarmKind - the kind of the alarm
*               m_enNewStatu - the excepted statu of the alarm
*
* Returns    :  none.
*
* Note(s)    :  none.
*********************************************************************************************************
*/
void AlarmCmd(SYSTEM_ALARM_ADDR_Typedef m_enSystemAlarmKind, SWITCH_TYPE_VARIABLE_Typedef m_enNewStatu)
{
    if((g_stSystemAlarmsInf.AlarmCode ^ (m_enNewStatu << (u8)m_enSystemAlarmKind)) != 0)//��״̬�б仯
    {
        g_stSystemAlarmsInf.HoldTime[m_enSystemAlarmKind].second = 0x00;
        g_stSystemAlarmsInf.HoldTime[m_enSystemAlarmKind].minute = 0x00;
        g_stSystemAlarmsInf.HoldTime[m_enSystemAlarmKind].hour = 0x00;

        if(m_enNewStatu == OFF)
        {
            g_stSystemAlarmsInf.AlarmCode &= ~(1 << (u8)m_enSystemAlarmKind);
        }
        else
        {
            g_stSystemAlarmsInf.AlarmCode |= (1 << (u8)m_enSystemAlarmKind);
        }
    }
    else//����ʲôҲ����
    {}
}

/*
*********************************************************************************************************
*                                      GetAlarmStatu()
*
* Description:  get the statu of a kind of alarm.
*               ��ȡϵͳ����״̬
* Arguments  :  m_enSystemAlarmKind - the kind of the alarm.
*
* Returns    :  the statu of the alarm.
*
* Note(s)    :  none.
*********************************************************************************************************
*/
SWITCH_TYPE_VARIABLE_Typedef GetAlarmStatu(SYSTEM_ALARM_ADDR_Typedef m_enSystemAlarmKind)
{
//  return stSYSTEM_ALARM[m_enSystemAlarmKind].FlagStatu;
    return (SWITCH_TYPE_VARIABLE_Typedef)((g_stSystemAlarmsInf.AlarmCode >> (u8)m_enSystemAlarmKind) & 0x1);
}

/*
*********************************************************************************************************
*                                      GetAlarmHoldTime()
*
* Description:  get the hold time of a kind of alarm.
*               ��ȡϵͳ��������ʱ��
* Arguments  :  m_enSystemAlarmKind - the kind of the alarm.
*
* Returns    :  the hold time of the kind of alarm.
*
* Note(s)    :  none.
*********************************************************************************************************
*/
SYSTEM_TIME_Typedef GetAlarmHoldTime(SYSTEM_ALARM_ADDR_Typedef m_enSystemAlarmKind)
{
    return g_stSystemAlarmsInf.HoldTime[m_enSystemAlarmKind];
}

/*
*********************************************************************************************************
*                                      GetRunAlarmCode()
*
* Description:  get the alarm time.
*               ��ȡϵͳ���о�����
* Arguments  :  none.
*
* Returns    :  the alarm code.
*
* Note(s)    :  none.
*********************************************************************************************************
*/
uint32_t GetRunAlarmCode(void)
{
    return g_stSystemAlarmsInf.AlarmCode;
}

/*
*********************************************************************************************************
*                                      ResetSystemIsolatedGeneratedEnergyThisTime()
*
* Description:  reset the isolated generated energy this time.
*               ��λϵͳ������
* Arguments  :  none.
*
* Returns    :  none.
*
* Note(s)    :  none.
*********************************************************************************************************
*/
void ResetSystemIsolatedGeneratedEnergyThisTime(void)
{
    g_dSystemIsolatedGeneratedEnergyThisTime = 0.0;
}

/*
*********************************************************************************************************
*                                      UpdateSystemIsolatedGeneratedEnergyThisTime()
*
* Description:  update the isolated generated energy this time.
*               ����ϵͳ������(�ܵķ�����)
* Arguments  :  none.
*
* Returns    :  none.
*
* Note(s)    :  none.
*********************************************************************************************************
*/
void UpdateSystemIsolatedGeneratedEnergyThisTime()
{
    if(g_eStackWorkStatu == EN_IN_WORK)
    {
        g_dSystemIsolatedGeneratedEnergyThisTime += ((double) GetCurrentPower()) / 3600 / 1000 * 1;
        //3600ΪÿСʱ�ķ�������1000Ϊ1kW, 1Ϊÿ�θ��µ�ʱ����Ϊ1s��
    }
}

/*
*********************************************************************************************************
*                                      GetIsolatedGenratedEnergyThisTime()
*
* Description:  get the isolated generated energy this time.
*
* Arguments  :  none.
*
* Returns    :  double.
*
* Note(s)    :  none.
*********************************************************************************************************
*/
double GetIsolatedGenratedEnergyThisTime()
{
    return g_dSystemIsolatedGeneratedEnergyThisTime;
}

/*
*********************************************************************************************************
*                                      SetSelfCheckCodeBit()
*
* Description:  set specified bit of the self-check code.
*
* Arguments  :  select the bit of the code.
*
* Returns    :  none.
*
* Note(s)    :  none.
*********************************************************************************************************
*/
void SetSelfCheckCodeBit(uint8_t i_u8BitNmb)
{
    if(i_u8BitNmb <= 64)
    {
        g_u64SysSelfCheckCode |= (1ll << i_u8BitNmb);//long long 1��ʾ64λ��;
    }
    else
    {}
}

/*
*********************************************************************************************************
*                                      ResetSelfCheckCodeBit()
*
* Description:  reset specified bit of the self-check code.
*
* Arguments  :  select the bit of the code.
*
* Returns    :  none.
*
* Note(s)    :  none.
*********************************************************************************************************
*/
void ResetSelfCheckCodeBit(uint8_t i_u8BitNmb)
{
    if(i_u8BitNmb <= 64)
    {
        g_u64SysSelfCheckCode  &= ~(1ll << i_u8BitNmb);
    }
    else
    {}
}

/*
*********************************************************************************************************
*                                      SetSelfCheckCodeWorkModeSection()
*
* Description:  set the section, that accord to the work mode, of the self-check code.
*
* Arguments  :  the new statu of the work mode.
*
* Returns    :  none.
*
* Note(s)    :  none.
*********************************************************************************************************
*/
void SetSelfCheckCodeWorkModeSection(SYSTEM_WORK_MODE_Typedef i_u8NewStatu)
{
    CPU_SR_ALLOC();
    CPU_CRITICAL_ENTER();
    g_u64SysSelfCheckCode &= 0x3FFFFFFFFFFFFFFFu;
    g_u64SysSelfCheckCode |= ((uint64_t)i_u8NewStatu << 62);
    CPU_CRITICAL_EXIT();
}

/*
*********************************************************************************************************
*                                      GetSelfCheckCode()
*
* Description:  get the self-check code.
*
* Arguments  :  none.
*
* Returns    :  none.
*
* Note(s)    :  none.
*********************************************************************************************************
*/
uint64_t GetSelfCheckCode(void)
{
    return g_u64SysSelfCheckCode;
}

/*
*********************************************************************************************************
*                                      GetSystemRunningStatuCode()
*
* Description:  get the system running statu code.
*
* Arguments  :  none.
*
* Returns    :  running statu code.
*
* Note(s)    :  none.
*********************************************************************************************************
*/
u32 GetSystemRunningStatuCode(void)
{
    return g_SystemRunningStatuCode;
}

/*
*********************************************************************************************************
*                                      SetSystemRunningStatuCodeBit()
*
* Description:  set specified bit of the system running statu code.
*
* Arguments  :  select the bit of the code.
*
* Returns    :  none.
*
* Note(s)    :  none.
*********************************************************************************************************
*/
void SetSystemRunningStatuCodeBit(uint8_t i_u8BitNmb)
{
    g_SystemRunningStatuCode |= 1u << i_u8BitNmb;
}

/*
*********************************************************************************************************
*                                      ResetSystemRunningStatuCodeBit()
*
* Description:  reset specified bit of the system running statu code.
*
* Arguments  :  select the bit of the code.
*
* Returns    :  none.
*
* Note(s)    :  none.
*********************************************************************************************************
*/
void ResetSystemRunningStatuCodeBit(uint8_t i_u8BitNmb)
{
    g_SystemRunningStatuCode &= ~(1u << i_u8BitNmb);
}

/*
*********************************************************************************************************
*                             SetSystemRunningStatuCodeSysWorkStatuSection()
*
* Description:  reset the section, that accord to the work statu, of  the system running statu code.
*                               ����ϵͳ���й���ģʽ
* Arguments  :  select the bit of the code.
*
* Returns    :  none.
*
* Note(s)    :  none.
*********************************************************************************************************
*/
void SetSystemRunningStatuCodeSysWorkStatuSection(SYSTEM_WORK_STATU_Typedef i_u8NewStatu)
{
    CPU_SR_ALLOC();

//  APP_TRACE_INFO(("The run statu code is %d<---\r\n", m_enNewStatu));
    if(i_u8NewStatu <= 7)//�ж�
    {
        CPU_CRITICAL_ENTER();
        g_SystemRunningStatuCode &= 0x8FFFFFFFu;
        g_SystemRunningStatuCode |= (i_u8NewStatu << 28);
        CPU_CRITICAL_EXIT();
    }
    else
    {}
}

/*
*********************************************************************************************************
*                             SystemTimeStatTaskCreate()
*
* Description:  create the system time statistics task.
*
* Arguments  :  none.
*
* Returns    :  none.
*
* Note(s)    :  none.
*********************************************************************************************************
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
*********************************************************************************************************
*                             SysTimeStatTask()
*
* Description:  the system time statistics task.
*               ˢ��ϵͳʱ�䡢�������⼰�ۼ�����ʱ�䣬�����౨������ʱ��
* Arguments  :  none.
*
* Returns    :  none.
*
* Note(s)    :
*********************************************************************************************************
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

    while(DEF_TRUE)
    {
        OSSemPend(&g_stSystemTimeUpdateSem,
                  OS_CFG_TICK_RATE_HZ,
                  OS_OPT_PEND_BLOCKING,
                  NULL,
                  &err);
        
        if( err == OS_ERR_NONE )                             
        {
            UpdateSysTime(&g_stSystemTime);
            UpdateSystemIsolatedGeneratedEnergyThisTime();
            
            if((g_eSystemWorkMode == EN_WORK_MODE_HYDROGEN_PRODUCER_AND_FUEL_CELL)
                    || (g_eSystemWorkMode == EN_WORK_MODE_HYDROGEN_PRODUCER))
            {
                if((g_eSystemWorkStatu == EN_START_PRGM_ONE_FRONT)
                        || (g_eSystemWorkStatu == EN_START_PRGM_ONE_BEHIND)
                        || (g_eSystemWorkStatu == EN_START_PRGM_TWO)
                        || (g_eSystemWorkStatu == EN_RUNNING))
                {
                    //��������ʱ��
                    UpdateSysTime(&g_stHydrgProduceTimeThisTime);
                    UpdateSysTime(&g_stHydrgProduceTimeTotal);

                    u32AlarmCode = g_stSystemAlarmsInf.AlarmCode;
                    i = 16;
                    u32AlarmCode &= 0xFFFFFFF;  //�˵�ǰ��4λ������������޹صĹ����龯��
                    u32AlarmCode >>= 16;            //���������صľ�����bit16��ʼ

                    while(u32AlarmCode != 0)
                    {
                        if((u32AlarmCode & 1) == 1)
                        {
                            UpdateSysTime(&g_stSystemAlarmsInf.HoldTime[i]);
                        }

                        u32AlarmCode >>= 1;
                        i++;
                    }

                    //          if(g_stSystemAlarmsInf.HoldTime[FUEL_SHORTAGE_ALARM].minute >= 30)//Һλ�ͳ�������30����
                    //          {
                    //              CmdShutDown();
                    //          }
                }
            }

        if(g_eStackWorkStatu == EN_IN_WORK)
        {
            UpdateSysTime(&g_stStackProductTimeThisTime);
            UpdateSysTime(&g_stStackProductTimeTotal);
            u32AlarmCode = g_stSystemAlarmsInf.AlarmCode;

            i = 0;
            u32AlarmCode &= 0xFFFF;//�˵�ǰ��4λ���Ѿ����޹صĹ����龯�������������

//          u32AlarmCode >>= 16;//������صľ�����bit0��ʼ����������
            while(u32AlarmCode != 0)
            {
                if((u32AlarmCode & 1) == 1)
                {
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
*********************************************************************************************************
*                             UpdateSysTime()
*
* Description:  update all kinds of time in the system.
*
* Arguments  :  kind of time.
*
* Returns    :  none.
*
* Note(s)    :  none.
*********************************************************************************************************
*/
static void UpdateSysTime(SYSTEM_TIME_Typedef *i_stTargetTime)
{
    CPU_SR_ALLOC();
    CPU_CRITICAL_ENTER();

    if(++i_stTargetTime->second >= (uint16_t) 60)
    {
        i_stTargetTime->second = 0;

        if(++i_stTargetTime->minute >= 60)
        {
            i_stTargetTime->minute = 0;
            ++i_stTargetTime->hour;
        }
    }

    CPU_CRITICAL_EXIT();
}

/*
*********************************************************************************************************
*                             GetCurrentPower()
*
* Description:  get current power.
*               ��õ�ǰ������
* Arguments  :  none.
*
* Returns    :  float.
*
* Note(s)    :  none.
*********************************************************************************************************
*/
float GetCurrentPower(void)
{
     return GetSrcAnaSig(STACK_VOLTAGE) * GetSrcAnaSig(STACK_CURRENT);
    // return GetSrcAnaSig(STACK_CURRENT);
}
/******************* (C) COPYRIGHT 2015 Guangdong Hydrogen *****END OF FILE****/

