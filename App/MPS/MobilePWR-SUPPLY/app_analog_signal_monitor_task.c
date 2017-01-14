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
* Programmer(s) : Fanjun
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

/*
***************************************************************************************************
*                                           MACRO DEFINES
***************************************************************************************************
*/
#define         ANA_SIGNAL_MONITOR_TASK_STK_SIZE        256

//������������
#define         STACK_TEMP_MONITOR_SWITCH                1u
#define         STACK_AIR_PRESS_MONITOR_SWITCH           1u
#define         STACK_VOLETAGE_MONITOR_SWITCH            1u
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
static      uint8_t      g_u8HydrgProducerAnaSigRunningMonitorAlarmHookSw = DEF_DISABLED;//���������ģ���źž�����⿪��
static      uint8_t      g_u8HydrgProducerPumpRunningStartAutoAdjHookSw = DEF_DISABLED;   //����������Զ����ڿ���
static      uint8_t      g_u8StackHydrgPressHighEnoughHookSw = DEF_DISABLED; //�ȴ����ѹ�����㿪��
static      uint8_t      g_u8StackAnaSigRunningMonitorAlarmHookSw = DEF_DISABLED;//�������ģ���źž�����⿪��
static      uint8_t      g_u8StackIsPulledStoppedMonitorHookSw = DEF_DISABLED;//����Ƿ���ͣ��⿪��
static      uint8_t      g_u8StackNeedRestartLimitCurrentFlag = DEF_CLR;//��ѱ���ͣ������������־
static      uint8_t      g_u8PumpAutoAdjFinishStatu = DEF_NO;//�״����б��ٵ���״̬
static      uint16_t     g_u16StackHydrgPressBelow10KPaHoldSeconds = 0;       //�����ѹС��10Kpa������
/*
***************************************************************************************************
*                                         FUNCTION PROTOTYPES
***************************************************************************************************
*/
static      void        AnaSigMonitorTask(void *p_arg);

static      void        HydrgProducerAnaSigAlarmRunningMonitorHook(void);
static      void        StackHydrgPressHighEnoughWaitHook(void);
static      void        StackAnaSigAlarmRunningMonitorHook(void);
static      void        JudgeWhetherTheStackIsPulledStoppedMonitorHook(void);
static      void        HydrgProducerPumpAutoAdjByDecompressCountHook(void);
static      void        HydrgProducerPumpRunningStartAutoAdjHook(void);
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
                  0,                              //�ȴ�ģ���źŲ������
                  OS_OPT_PEND_BLOCKING,
                  NULL,
                  &err);
        UpdateAnaSigDigValue();//���²���ֵ

        /* ���������������źż�� */
        if(g_u8HydrgProducerAnaSigRunningMonitorAlarmHookSw == DEF_ENABLED) {
            HydrgProducerAnaSigAlarmRunningMonitorHook();
        }

        if(g_u8StackAnaSigRunningMonitorAlarmHookSw == DEF_ENABLED) {
            StackAnaSigAlarmRunningMonitorHook();
        }

        if(g_u8StackHydrgPressHighEnoughHookSw == DEF_ENABLED) {
            StackHydrgPressHighEnoughWaitHook();
        }

        //������Ƿ���ͣ
        if(g_u8StackIsPulledStoppedMonitorHookSw == DEF_ENABLED) {
            JudgeWhetherTheStackIsPulledStoppedMonitorHook();
        }
        
        if(g_u8HydrgProducerPumpRunningStartAutoAdjHookSw == DEF_ENABLED) {
            HydrgProducerPumpRunningStartAutoAdjHook();//Һѹ����4���������Զ����ڱ��٣�ֻ����һ��
        }
        //������ٸ�������������̬����
//        if(g_u8HydrgProducerPumpAutoAdjByDecompressCountHookSw == DEF_ENABLED) {
//            HydrgProducerPumpAutoAdjByDecompressCountHook();
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
        AlarmCmd(LIQUID_PRESS_HIGH_ALARM,SERIOUS_GRADE,ON);

        if(fLqdPress > g_stLqdPressCmpTbl.ShutDownUpperLimit) {
            APP_TRACE_INFO(("Hydrogen producer liquid press is above the high press protect line...\n\r"));
        }
    } else {
        AlarmCmd(LIQUID_PRESS_HIGH_ALARM,SERIOUS_GRADE,OFF);
    }


    flqdHeight = GetSrcAnaSig(LIQUID_LEVEL);

    if(flqdHeight < g_stLqdHeightCmpTbl.AlarmlowerLiquidLevellimit) {
        AlarmCmd(FUEL_SHORTAGE_ALARM,GENERAL_GRADE,ON);
        
    } else {
        AlarmCmd(FUEL_SHORTAGE_ALARM,GENERAL_GRADE,OFF);

        if(flqdHeight <= g_stLqdHeightCmpTbl.OpenAutomaticliquidValue) { //�Զ���Һˮ��
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

    if(i >= 10) { //ģ���źż��������100msһ�Σ���10��Ҳ��1�����1��
        i = 0;

        if((fLqdPress >= 4) && (g_u8PumpAutoAdjFinishStatu == DEF_NO)) {
            PumpSpdDec();

            if(GetPumpCtlSpd() <= g_stStartHydrgPumpSpdPara.PumpSpdAfterLiquidPressExceed4Kg) {
                g_u8PumpAutoAdjFinishStatu = DEF_YES;
                g_u8HydrgProducerPumpRunningStartAutoAdjHookSw = DEF_DISABLED;//�ص��ڿ���
            }
        }
    }
}

/*
***************************************************************************************************
*                               StackHydrgPressHighEnoughWaitHook
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
void StackHydrgPressHighEnoughWaitHook(void)
{
    OS_ERR err;

    if(GetSrcAnaSig(HYDROGEN_PRESS_1) >= 45.0) {
        OSTaskSemPost(&StackManagerTaskTCB, OS_OPT_POST_NO_SCHED, &err);
        SetStackHydrgPressHighEnoughHookSwitch(DEF_DISABLED);
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
    float fStackTemp = 0;
    float fHydrgPress = 0;
    float fStackVoletage = 0;
    static uint8_t StackTempHighFlag = NO;
    static uint8_t StackHydrogenPressLowFlag = NO;
    static uint8_t StackVoletageLowFlag = NO;
    static uint16_t g_u16StackVoletageBelow40VHoldSeconds = 0;
    static uint16_t g_u16StackVoletageExceed40VHoldSeconds = 0;
    
#if STACK_TEMP_MONITOR_SWITCH 
    /* ����¶� */
    fStackTemp = GetSrcAnaSig(STACK_TEMP);

    if(fStackTemp > 60){
        AlarmCmd(STACK_TEMP_HIGH_ALARM,GENERAL_GRADE,ON);

        if(fStackTemp > 68) {
            if(StackTempHighFlag != YES){
                BSP_DCConnectValvePwrOff();
                StackTempHighFlag = YES;
                APP_TRACE_INFO(("Stack temp is above the high temp protect line,stop output...\n\r"));
            }
        }
    } else if(fStackTemp < 15) {
        AlarmCmd(STACK_TEMP_LOW_ALARM,GENERAL_GRADE,ON);

        if(fStackTemp < 10) {
//            CmdShutDown();      //�ػ�����
            APP_TRACE_INFO(("Stack temp is below the low temp protect line...\n\r"));
        }
    } else {
        if((fStackTemp >= 20) && (fStackTemp <= 50)) {//ϵͳ�ָ��������¶�,�ָ����
            if(StackTempHighFlag == YES){ 
                BSP_DCConnectValvePwrOn();
                StackTempHighFlag = NO;
            }
            AlarmCmd(STACK_TEMP_HIGH_ALARM, GENERAL_GRADE,OFF);  
        }
        AlarmCmd(STACK_TEMP_LOW_ALARM,GENERAL_GRADE, OFF);
    }
#endif
    
#if STACK_AIR_PRESS_MONITOR_SWITCH 
    /* �����ѹ */
    fHydrgPress = GetSrcAnaSig(HYDROGEN_PRESS_1);

    if(fHydrgPress >= 30) {
        AlarmCmd(HYDROGEN_PRESS_LOW_ALARM,SERIOUS_GRADE,OFF);
        if(StackHydrogenPressLowFlag != NO){
            BSP_DCConnectValvePwrOn();
            StackHydrogenPressLowFlag = NO;
        }
    } else if(fHydrgPress >= 10){
        g_u16StackHydrgPressBelow10KPaHoldSeconds = 0;
    }else {
        AlarmCmd(HYDROGEN_PRESS_LOW_ALARM,SERIOUS_GRADE,ON);
        g_u16StackHydrgPressBelow10KPaHoldSeconds++;

        if(g_u16StackHydrgPressBelow10KPaHoldSeconds >= 30) {//��ѹ���ͳ���3s,���ֹͣ���
            BSP_DCConnectValvePwrOff();
            StackHydrogenPressLowFlag = YES;
            g_u16StackHydrgPressBelow10KPaHoldSeconds = 0;
//            CmdShutDown();      //�ػ�����
        }       
    }
#endif
    
#if STACK_VOLETAGE_MONITOR_SWITCH    
    /*����ѹ*/
    fStackVoletage = GetSrcAnaSig(STACK_VOLTAGE);
    if(EN_IN_WORK == GetStackWorkStatu()){
        if(fStackVoletage >= 40) {
            g_u16StackVoletageBelow40VHoldSeconds = 0;
            AlarmCmd(HYDROGEN_PRESS_LOW_ALARM,GENERAL_GRADE,OFF);
            if(StackVoletageLowFlag != NO ){
                g_u16StackVoletageExceed40VHoldSeconds ++;
                if(g_u16StackVoletageExceed40VHoldSeconds >= 300){
                    BSP_DCConnectValvePwrOn();
                    StackVoletageLowFlag = NO;
                    g_u16StackVoletageExceed40VHoldSeconds = 0;
                }
            }
        }else {
            g_u16StackVoletageBelow40VHoldSeconds ++;
            if(g_u16StackVoletageBelow40VHoldSeconds > 300){
                AlarmCmd(STACK_VOLTAGE_LOW_ALARM,GENERAL_GRADE,ON);
                StackVoletageLowFlag = YES;
                BSP_DCConnectValvePwrOff();
                g_u16StackVoletageBelow40VHoldSeconds = 0;
            }
        } 
    }
#endif
    
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
*                               SetStackHydrgPressHighEnoughHookSwitch
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
void SetStackHydrgPressHighEnoughHookSwitch(uint8_t i_NewStatu)
{
    g_u8StackHydrgPressHighEnoughHookSw = i_NewStatu;
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
    
    fStackVoltage = GetSrcAnaSig(STACK_VOLTAGE);
    fStackCurrent = GetSrcAnaSig(STACK_CURRENT);
    
    u16RestartLimitCurrentCount ++;
    if(u16RestartLimitCurrentCount >= 30)//ÿ3����һ���Ƿ���Ҫ��ʼ��������
    if(EN_IN_WORK == GetStackWorkStatu()){
        if((fStackVoltage >= 51.0) && (fStackCurrent <= 3.0)){
            
            g_u8StackNeedRestartLimitCurrentFlag = DEF_SET;
            SetDCModuleAutoAdjustTaskSwitch(DEF_DISABLED);
            OSTaskResume(&DCLimitCurrentSmoothlyTaskTCB,&err);
            
        }else{
            g_u8StackNeedRestartLimitCurrentFlag = DEF_CLR;
        }
    }   
}

uint8_t GetStackNeedRestartLimitCurrentFlag(void)
{
    return g_u8StackNeedRestartLimitCurrentFlag;
}
/*
***************************************************************************************************
*                          HydrgProducerPumpAutoAdjByDecompressCountHook()
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
//static void HydrgProducerPumpAutoAdjByDecompressCountHook(void)
//{
//    if((GetSrcAnaSig(STACK_VOLTAGE) <= 51.0) && (GetSrcAnaSig(STACK_CURRENT) >= 3.0) && (GetSrcAnaSig(LIQUID_PRESS) <= 17.5 )){
//        
//    }else{
//        
//    }
//}



/*
***************************************************************************************************
*                    HydrgProducerPumpAutoAdjByDecompressCountHook()
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
void ExcuteStackProtectProgram()
{
    BSP_DCConnectValvePwrOff();
    
    BSP_DCConnectValvePwrOn();
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
void StartRunningBeepAlarm(SYSTEM_ALARM_GRADE_Typedef i_AlarmGrade)
{
    uint8_t     u8Count = 0;
    u32 u32AlarmCode;
    u32AlarmCode = GetRunAlarmCode();
    
    if(u32AlarmCode != 0) {
        u8Count ++;
        if(i_AlarmGrade <= SLIGHT_GRADE){//��͵ȼ�����
            if(u8Count >= 20){//�͵ȼ�����Ϊ2s��������һ��
                BSP_BuzzerTurnover();
                u8Count = 0;
            }
        }else if(i_AlarmGrade <= GENERAL_GRADE){
            if(u8Count >= 10){
                BSP_BuzzerTurnover();
                u8Count = 0;
            }
        }else{  //��ߵȼ�����
            if(u8Count >= 5){
                BSP_BuzzerTurnover();
                u8Count = 0;
            }
        }
    }else {
        BSP_BuzzerOff();
    }
}
/******************* (C) COPYRIGHT 2016 Guangdong Eneco *****END OF FILE****/
