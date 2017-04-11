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
* Filename      : app_top_task.c
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
#include "bsp_speed_adjust_device.h"
#include "app_top_task.h"
#include "app_system_run_cfg_parameters.h"
#include "app_system_real_time_parameters.h"
#include "app_hydrg_producer_manager.h"
#include "app_stack_manager.h"
#include "app_wireness_communicate_task.h"
#include "app_auto_make_vacuum.h"
/*
***************************************************************************************************
*                                         OS-RELATED    VARIABLES
***************************************************************************************************
*/
OS_SEM   IgniteFirstBehindWaitSem;
OS_SEM   IgniteSecondBehindWaitSem;
OS_SEM   WaitSelcetWorkModeSem;

/*
***************************************************************************************************
*                                           LOCAL VARIABLES
***************************************************************************************************
*/

static  SWITCH_TYPE_VARIABLE_Typedef                g_eDeviceFaultAlarm = OFF;
static  SYSTEM_SHUT_DOWN_ACTION_FLAG_Typedef        g_eShutDownActionFlag = EN_DELAY_STOP_BOTH_PARTS;

/*
***************************************************************************************************
*                                           GLOBAL VARIABLES
***************************************************************************************************
*/


/*
***************************************************************************************************
*                                         FUNCTION PROTOTYPES
***************************************************************************************************
*/
static  void    ShutDown(void);

/*
***************************************************************************************************
*                                          CheckAuthorization()
*
* Description : The use of this funciton is to check the authorization of the system.
*               s
* Arguments   : none.
*
* Returns     : none
*
* Notes       : none.
***************************************************************************************************
*/
VERIFY_RESULT_TYPE_VARIABLE_Typedef CheckAuthorization(void)
{
    return EN_THROUGH;
}

/*
***************************************************************************************************
*                                          DeviceFaultAlarm()
*
* Description : The funciton is the device alarm process.
*
* Arguments   : none.
*
* Returns     : none
*
* Notes       : none.
***************************************************************************************************
*/
void DeviceFaultAlarm(void)
{
    OS_ERR      err;
    g_eDeviceFaultAlarm = ON;
    APP_TRACE_INFO(("Device fault alarming...\n\r"));

    while(DEF_TRUE) {
        BSP_BuzzerTurnover();

        if(g_eDeviceFaultAlarm == OFF) {
            BSP_BuzzerOff();
            break;
        }

        OSTimeDlyHMSM(0, 0, 1, 0,
                      OS_OPT_TIME_HMSM_STRICT,
                      &err);
    }

    APP_TRACE_INFO(("The device fault alarm has been reset...\n\r"));
}

/*
***************************************************************************************************
*                                          ResetDeviceAlarmStatu()
*
* Description : The use of the funciton is to exit the device alarm process.
*
* Arguments   : none.
*
* Returns     : none
*
* Notes       : none.
***************************************************************************************************
*/
void ResetDeviceAlarmStatu(void)
{
    g_eDeviceFaultAlarm = OFF;
}

/*
***************************************************************************************************
*                                          DeviceSelfCheck()
*
* Description : The use of the funciton is to self-check the devices.
*
* Arguments   : none.
*
* Returns     : none.
*
* Notes       : none.
***************************************************************************************************
*/
VERIFY_RESULT_TYPE_VARIABLE_Typedef DeviceSelfCheck(void)
{
    VERIFY_RESULT_TYPE_VARIABLE_Typedef MachineSelfCheckResult;
    SELF_CHECK_CODE_Typedef stSelfCheckCode;
    SYSTEM_WORK_MODE_Typedef    eWorkMode;

    APP_TRACE_INFO(("Self-checking...\r\n"));
    ResetAllAlarms();
    AnaSensorSelfCheck();   //ģ���źŴ������Լ�

    stSelfCheckCode = GetSysSelfCheckCode();
    SendRealTimeAssistInfo();  //����ʵʱ������Ϣ���Լ���Ϣ

    eWorkMode = GetWorkMode();

    switch((u8)eWorkMode) {
        case EN_WORK_MODE_HYDROGEN_PRODUCER_AND_FUEL_CELL:
            APP_TRACE_INFO(("Both parts of the machine are expected to work...\r\n"));

            if(stSelfCheckCode.DevSelfCheckSensorStatusCode == 0) { //�豸�Լ촫��������
                if((stSelfCheckCode.MachinePartASelfCheckCode == 0) && (stSelfCheckCode.MachinePartBSelfCheckCode == 0)) {  //�������豸����,ֻ����ģ���������豸�Լ�
                    APP_TRACE_INFO(("Device of hydrogen producer group and stack group is at right statu...\r\n"));
                    MachineSelfCheckResult = EN_THROUGH;
                } else {
                    if(stSelfCheckCode.MachinePartASelfCheckCode != 0) { //�������豸����
                        APP_TRACE_INFO(("Device of hydrogen producer group is not at right statu,can not work...\r\n"));
                    } else {
                        APP_TRACE_INFO(("Device of stack group is not at right statu,can not work...\r\n"));
                        MachineSelfCheckResult = EN_NOT_THROUGH;
                    }
                }
            } else {
                APP_TRACE_INFO(("Device of device selfcheck sensor is not at right statu,can not work...\r\n"));
                MachineSelfCheckResult = EN_NOT_THROUGH;
            }

            break;

        case EN_WORK_MODE_HYDROGEN_PRODUCER:
            APP_TRACE_INFO(("hydrogen producer of the machine is expected to work...\r\n"));

            if(stSelfCheckCode.DevSelfCheckSensorStatusCode == 0) { //�豸�Լ촫��������
                if(stSelfCheckCode.MachinePartASelfCheckCode == 0) { //�������豸����,ֻ����ģ���������豸�Լ�
                    APP_TRACE_INFO(("Device of hydrogen producer group is at right statu...\r\n"));
                    MachineSelfCheckResult = EN_THROUGH;
                } else {
                    APP_TRACE_INFO(("Device of hydrogen producer group is not at right statu,can not work...\r\n"));
                    MachineSelfCheckResult = EN_NOT_THROUGH;
                }
            } else {
                APP_TRACE_INFO(("Device of device selfcheck sensor is not at right statu,can not work...\r\n"));
                MachineSelfCheckResult = EN_NOT_THROUGH;
            }

            break;

        case EN_WORK_MODE_FUEL_CELL:
            APP_TRACE_INFO(("Stack of the machine is expected to work...\r\n"));

            if(stSelfCheckCode.DevSelfCheckSensorStatusCode == 0) { //�豸�Լ촫��������
                if(stSelfCheckCode.MachinePartBSelfCheckCode == 0) { //�������豸����
                    APP_TRACE_INFO(("Device of stack group is at right statu, the machine can work normal...\r\n"));
                    MachineSelfCheckResult = EN_THROUGH;
                } else {
                    APP_TRACE_INFO(("Device of stack group is not at right statu,can not work...\r\n"));
                    MachineSelfCheckResult = EN_NOT_THROUGH;
                }
            } else {
                APP_TRACE_INFO(("Device of device selfcheck sensor is not at right statu,can not work...\r\n"));
                MachineSelfCheckResult = EN_NOT_THROUGH;
            }

            break;

        case EN_WORK_MODE_MALFUNCTION:  //����ģʽ
            APP_TRACE_INFO(("Device of machine is not at right statu,can not work...\r\n"));

        default:
            APP_TRACE_INFO(("Fault: System expected to work on the malfunction mode...\r\n"));
            MachineSelfCheckResult = EN_NOT_THROUGH;
            break;
    }

//  return MachineSelfCheckResult;
    return EN_THROUGH;//��ʱ�ر��Լ�
}
/*
***************************************************************************************************
*                                          WaittingCommand()
*
* Description : The use of the funciton is to wait the command and cyclic self-check.
*
* Arguments   : none.
*
* Returns     : none.
*
* Notes       : none.
***************************************************************************************************
*/
VERIFY_RESULT_TYPE_VARIABLE_Typedef WaittingCommand(void)
{
    OS_ERR      err;
    VERIFY_RESULT_TYPE_VARIABLE_Typedef WaitCmdStatu;
    SYSTEM_WORK_MODE_Typedef    eWorkMode;

#if __INTERNAL_TEST_FLAG > 0u

    APP_TRACE_INFO(("Waitting for the command to select the work mode...\r\n"));
    SetWorkModeWaittingForSelectFlag();//��λ�ȴ�����ģʽѡ��ȴ���־
    OSTaskResume(&CommunicateRequsetInfSendTaskTCB, &err);//����������ʼ����
    OSSemPend(&WaitSelcetWorkModeSem,
              0,
              OS_OPT_PEND_BLOCKING,
              NULL,
              &err);
    ResetWorkModeWaittingForSelectFlag(); //��λѡ��ȴ���־
    OSTimeDlyResume(&CommunicateTaskTCB, &err);

#elif __HYDROGEN_GENERATOR_MODULE
    SetWorkMode(EN_WORK_MODE_HYDROGEN_PRODUCER);
    OSTaskResume(&Make_Vaccuum_FunctionTaskTCB, &err); //��ʼ�����
#elif __FUEL_CELL_MODULE
    SetWorkMode(EN_WORK_MODE_FUEL_CELL);//����ģʽ�²���Ҫ�����
#else
    SetWorkMode(EN_WORK_MODE_HYDROGEN_PRODUCER_AND_FUEL_CELL);
    OSTaskResume(&Make_Vaccuum_FunctionTaskTCB, &err); //��ʼ�����
#endif

    eWorkMode = GetWorkMode();
    APP_TRACE_INFO(("The work mode is: %d...\r\n", eWorkMode));

    while(DEF_TRUE) {
        if(EN_THROUGH == DeviceSelfCheck()) {
            APP_TRACE_INFO(("Self-check success...\n\r"));

            OSTaskSemPend(OS_CFG_TICK_RATE_HZ * 30 * 1, //ÿ30s�Լ�һ��
                          OS_OPT_PEND_BLOCKING,
                          NULL,
                          &err);

            if(err == OS_ERR_NONE) {  //��ȷ�õ��ź�������ʼ����
                APP_TRACE_INFO(("Receive the run command...\n\r"));
                WaitCmdStatu = EN_THROUGH;
                SetSystemWorkStatu(EN_START_PRGM_ONE_FRONT);
                break;
            } else if(err == OS_ERR_TIMEOUT) { //�ȴ���ʱ�������Լ�
                APP_TRACE_INFO(("Not receive the run command, restart the self-check...\n\r"));
            } else {
                APP_TRACE_INFO(("Waitting run command failed, err code %d...\n\r", err));
                WaitCmdStatu = EN_NOT_THROUGH;
                break;
            }

            SendRealTimeAssistInfo();   //ÿ30s����λ������һ���Լ���Ϣ
        } else {
            APP_TRACE_INFO(("Self-check failed...\n\r"));
            WaitCmdStatu = EN_NOT_THROUGH;
            break;
        }
    }

    return WaitCmdStatu;
}

/*
***************************************************************************************************
*                                          Starting()
*
* Description : The use of the funciton is to start the system.
*
* Arguments   : none.
*
* Returns     : none.
*
* Notes       : none.
***************************************************************************************************
*/
void Starting(void)
{
    OS_ERR      err;
    SYSTEM_WORK_MODE_Typedef eWorkMode;
    float IgniteCheckTable1, GoToNextStepTempTable1;
    float IgniteCheckTable2, GoToNextStepTempTable2;

    IgniteCheckTable1 = g_stReformerTempCmpTbl.IgFstTimeFrtToBhdTmpPnt;
    GoToNextStepTempTable1 = g_stReformerTempCmpTbl.IgFstTimeOverTmpPnt;

    //��λϵͳ���η�����
    ResetSystemIsolatedGeneratedEnergyThisTime();
    ResetHydrgProduceTimeThisTime();
    ResetStackProductTimeThisTime();

    SystemWorkTimesInc();
    eWorkMode = GetWorkMode();

    BSP_BuzzerOn();
    OSTimeDlyHMSM(0, 0, 0, 200, OS_OPT_TIME_HMSM_STRICT, &err);
    BSP_BuzzerOff();
    OSTimeDlyHMSM(0, 0, 0, 200, OS_OPT_TIME_HMSM_STRICT, &err);


    //һ���ģʽ��������ģʽ
    if((eWorkMode == EN_WORK_MODE_HYDROGEN_PRODUCER_AND_FUEL_CELL) || (eWorkMode == EN_WORK_MODE_HYDROGEN_PRODUCER)) {
        if(EN_START_PRGM_ONE_FRONT == GetSystemWorkStatu()) {
            HydrgProducerWorkTimesInc();

            //��һ�ε��
            if((EN_PASS == IgniteFirstTime(IgniteCheckTable1, GoToNextStepTempTable1, 3, 1))) {
                SetSystemWorkStatu(EN_START_PRGM_TWO);
            } else {
                APP_TRACE_INFO(("Ignite for the first time is failed...\n\r"));

                if(GetSystemWorkStatu() != EN_WAITTING_COMMAND) {   //��λ�������ػ���������������ǰ����
                    SetShutDownActionFlag(EN_DELAY_STOP_PART_ONE);
                }
            }

            //�ڶ��ε��
            if(GetSystemWorkStatu() == EN_START_PRGM_TWO) {
                IgniteCheckTable2 = g_stReformerTempCmpTbl.IgScdTimeFrtToBhdTmpPnt;
                GoToNextStepTempTable2 = g_stReformerTempCmpTbl.IgScdTimeOverTmpPnt;

                if(EN_PASS != IgniteSecondTime(IgniteCheckTable2, GoToNextStepTempTable2, 3, 1)) {
                    APP_TRACE_INFO(("Ignite for the second time is failed...\n\r"));

                    if(GetSystemWorkStatu() != EN_WAITTING_COMMAND) {   //������Ϊ��λ��ֱ�ӽ�״̬��Ϊ�ر��£����µ��ʧ��
                        SetShutDownActionFlag(EN_DELAY_STOP_BOTH_PARTS);
                    }
                } else {
                    SetSystemWorkStatu(EN_RUNNING);
                }

                SetShutDownActionFlag(EN_DELAY_STOP_PART_ONE);
            } else {
                APP_TRACE_INFO(("Ignite for the second time has been leaped over ...\n\r"));
                SetShutDownActionFlag(EN_DELAY_STOP_PART_ONE);//���������ɹ���Ҳ������ʱ�رհ��һ������Ӧ��ʶ
            }
        } else {
            APP_TRACE_INFO(("Program's running here is not to start...\n\r"));
            SetShutDownActionFlag(EN_STOP_ALL_DIRECT);
        }

    }
    //����ģʽ
    else if(eWorkMode == EN_WORK_MODE_FUEL_CELL) {
        APP_TRACE_INFO(("Only stack of the machine is excepted to work in the current work mode...\n\r"));
        SetSystemWorkStatu(EN_RUNNING);
        SetShutDownActionFlag(EN_DELAY_STOP_PART_TWO);
    } else {
        APP_TRACE_INFO(("Fault: The program should not come here in the malfunciton mode...\n\r"));
        SetSystemWorkStatu(EN_KEEPING_WARM);
        SetShutDownActionFlag(EN_STOP_ALL_DIRECT);
    }
}

/*
***************************************************************************************************
*                                          KeepingWarm()
*
* Description : The use of the funciton is to keep the system warm.
*
* Arguments   : none.
*
* Returns     : none.
*
* Notes       : none.
***************************************************************************************************
*/
void    KeepingWarm(void)
{
    ShutDown();
    APP_TRACE_INFO(("Keeping warm...\n\r"));
    SetSystemWorkStatu(EN_WAITTING_COMMAND);

}

/*
***************************************************************************************************
*                                          Running()
*
* Description : The use of the funciton is to keep the system run.
*
* Arguments   : none.
*
* Returns     : none.
*
* Notes       : none.
***************************************************************************************************
*/
void Running()
{
    OS_ERR      err;
    SYSTEM_WORK_MODE_Typedef eWorkMode;

    if(GetSystemWorkStatu() == EN_RUNNING) {

        APP_TRACE_INFO(("Run: System enter the cruise process...\n\r"));
        eWorkMode = GetWorkMode();

        switch((u8)eWorkMode) {
            case(u8)EN_WORK_MODE_HYDROGEN_PRODUCER_AND_FUEL_CELL:
                OSTaskResume(&HydrgProducerManagerTaskTCB,
                             &err);
                OSTaskResume(&StackManagerTaskTCB,
                             &err);
                SetShutDownActionFlag(EN_DELAY_STOP_BOTH_PARTS);//���е���˵��һ���ģʽ������������ǰ�������׶�δ�ϸ�����һ�������ģʽ�����ڴ��ٴ�����
                OSTaskSuspend(&AppTaskStartTCB, //�������������������������͵�ѹ��������������
                              &err);
                SetSystemWorkStatu(EN_SHUTTING_DOWN);
                break;

            case(u8)EN_WORK_MODE_HYDROGEN_PRODUCER:
                OSTaskResume(&HydrgProducerManagerTaskTCB,
                             &err);
//                  SetShutDownActionFlag(EN_DELAY_STOP_PART_ONE);//������ģʽ�µĹػ���־�Ѿ��������׶����úã����ڸ�ģʽ�²���Ҫ��������
                OSTaskSuspend(&AppTaskStartTCB,
                              &err);
                SetSystemWorkStatu(EN_SHUTTING_DOWN);
                break;

            case(u8)EN_WORK_MODE_FUEL_CELL:
                OSTaskResume(&StackManagerTaskTCB,
                             &err);
//                  SetShutDownActionFlag(EN_DELAY_STOP_PART_TWO);//�򷢵�ģʽ�µĹػ���־�Ѿ��������׶����úã����ڸ�ģʽ�²���Ҫ��������
                OSTaskSuspend(&AppTaskStartTCB,
                              &err);
                SetSystemWorkStatu(EN_SHUTTING_DOWN);
                break;

            default:
                APP_TRACE_INFO(("Fault: System run at the malfunction mode...\n\r"));
                //�ػ�״̬��ʶ�Ѿ��������׶����úã������ڴ������趨
                SetSystemWorkStatu(EN_KEEPING_WARM);
                break;
        }
    } else {
        APP_TRACE_INFO(("Ignite befor the run process is failed or the run process is leaped over...\n\r"));
    }
}

/*
***************************************************************************************************
*                                          SetShutDownActionFlag()
*
* Description : The use of the funciton is to select the shut down action flag.
*
* Arguments   : none.
*
* Returns     : none.
*
* Notes       : none.
***************************************************************************************************
*/
void SetShutDownActionFlag(SYSTEM_SHUT_DOWN_ACTION_FLAG_Typedef i_eNewActionStatu)
{
    g_eShutDownActionFlag = i_eNewActionStatu;
}

/*
***************************************************************************************************
*                                                   ShutDown()
*
* Description : The use of the funciton is to shut down the system.
*
* Arguments   : none.
*
* Returns     : none.
*
* Notes       : none.
***************************************************************************************************
*/
void ShutDown()
{
    OS_ERR  err;
    uint8_t i;
    SYSTEM_WORK_STATU_Typedef eWorkStatu;

    eWorkStatu = GetSystemWorkStatu();
    ResetAllAlarms();//������о���

    if(eWorkStatu == EN_SHUTTING_DOWN) {
        APP_TRACE_INFO(("Exit the cruise process to shut down...\r\n"));//��ʾϵͳ�˳�Ѳ�����н׶�

        switch((u8)g_eShutDownActionFlag) {
            case EN_STOP_ALL_DIRECT:

                APP_TRACE_INFO(("Shutdown all direcetly!!!\r\n"));
                break;

            case EN_DELAY_STOP_PART_ONE:
                OSTaskResume(&HydrgProducerManagerDlyStopTaskTCB,   //�ָ��������ʱ�ر�����
                             &err);
                break;

            case EN_DELAY_STOP_PART_TWO:
//                if(EN_IN_WORK == GetStackWorkStatu()) { //������ڹ����Żָ���ʱ�ر�����
                OSTaskResume(&StackManagerDlyStopTaskTCB,
                             &err);
//                }

                break;

            case(u8)EN_DELAY_STOP_BOTH_PARTS:
                OSTaskResume(&HydrgProducerManagerDlyStopTaskTCB,   //�ָ�������������ʱ�ر�����
                             &err);

                if(EN_IN_WORK == GetStackWorkStatu()) { //������ڹ����Żָ���ʱ�ر�����
                    OSTaskResume(&StackManagerDlyStopTaskTCB,
                                 &err);
                }

                break;

            default:
                break;
        }

        BSP_BuzzerOff();//��ֹ�ػ�������ʾʱ�����ʼ״̬ƫ����ִ���

        for(i = 0; i < 4; i++) {
            BSP_BuzzerTurnover();

            OSTimeDlyHMSM(0, 0, 1, 0,
                          OS_OPT_TIME_HMSM_STRICT,
                          &err);
        }

        switch((u8)g_eShutDownActionFlag) {
            case(u8)EN_STOP_ALL_DIRECT:
                APP_TRACE_INFO(("--> The system has not start...\r\n"));  //ֱ�ӹػ�������Ҫ�ػ���ʱ
                break;

            case(u8)EN_DELAY_STOP_PART_ONE:
                APP_TRACE_INFO(("-->Delay stop part one...\r\n"));
                //�˴������ȴ��������ʱ�ر������е������ź���
                OSTaskSemPend(OS_CFG_TICK_RATE_HZ * 60 * 3,   //3���Ӻ�ر������
                              OS_OPT_PEND_BLOCKING,
                              NULL,
                              &err);

                if(err == OS_ERR_NONE) {
                    break;
                }

            case(u8)EN_DELAY_STOP_PART_TWO:
                APP_TRACE_INFO(("-->Delay stop part two...\r\n"));
                //�˴������ȴ������ʱ�ر������е������ź���
                OSTaskSemPend(OS_CFG_TICK_RATE_HZ * 30,         //30s��رյ��
                              OS_OPT_PEND_BLOCKING,
                              NULL,
                              &err);

                if(err == OS_ERR_NONE) {
                    break;
                }

            case(u8)EN_DELAY_STOP_BOTH_PARTS:
                OSTaskSemPend(OS_CFG_TICK_RATE_HZ * 30, //stop stack after 30 seconds
                              OS_OPT_PEND_BLOCKING,
                              NULL,
                              &err);

                OSTaskSemPend(OS_CFG_TICK_RATE_HZ * 160,//�����30+150��->3���Ӻ�ر�
                              OS_OPT_PEND_BLOCKING,
                              NULL,
                              &err);

                if(err == OS_ERR_NONE) {
                    APP_TRACE_INFO(("The hydrogen producer and stack delay stop finished...\r\n"));
                    break;
                } else {
                    APP_TRACE_INFO(("The hydrogen producer and stack delay stop over time...\r\n"));
                }

            default:
                break;
        }

        OSTaskSemSet(&AppTaskStartTCB, 0, &err); //��������ź�������ֹ�������ź����ۼ������һ�������쳣
    } else {
        APP_TRACE_INFO(("The program don't need to start the shut down process...\r\n"));
    }
}

/*
***************************************************************************************************
*                                         UpdateBuzzerStatuInCruise()
*
* Description : The use of the funciton is to update the buzzer statu when running.
*
* Arguments   : none.
*
* Returns     : none.
*
* Notes       : none.
***************************************************************************************************
*/
void UpdateBuzzerStatuInCruise(void)
{
    u32 u32AlarmCode;
    u32AlarmCode = GetRunAlarmCode();

    if(u32AlarmCode != 0) {
        BSP_BuzzerOn();
    } else {
        BSP_BuzzerOff();
    }
}
/******************* (C) COPYRIGHT 2015 Guangdong Hydrogen *****END OF FILE****/