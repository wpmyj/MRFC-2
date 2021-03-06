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
* Programmer(s) : JasonFan
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
    AnaSensorSelfCheck();   //模拟信号传感器自检

    stSelfCheckCode = GetSysSelfCheckCode();
    SendRealTimeAssistInfo();  //发送实时辅助信息：自检信息

    eWorkMode = GetWorkMode();

    switch((u8)eWorkMode) {
        case EN_WORK_MODE_HYDROGEN_PRODUCER_AND_FUEL_CELL:
            APP_TRACE_INFO(("Both parts of the machine are expected to work...\r\n"));

            if(stSelfCheckCode.DevSelfCheckSensorStatusCode == 0) { //设备自检传感器正常
                if((stSelfCheckCode.MachinePartASelfCheckCode == 0) && (stSelfCheckCode.MachinePartBSelfCheckCode == 0)) {  //制氢组设备正常,只进行模拟输入组设备自检
                    APP_TRACE_INFO(("Device of hydrogen producer group and stack group is at right statu...\r\n"));
                    MachineSelfCheckResult = EN_THROUGH;
                } else {
                    if(stSelfCheckCode.MachinePartASelfCheckCode != 0) { //发电组设备正常
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

            if(stSelfCheckCode.DevSelfCheckSensorStatusCode == 0) { //设备自检传感器正常
                if(stSelfCheckCode.MachinePartASelfCheckCode == 0) { //制氢组设备正常,只进行模拟输入组设备自检
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

            if(stSelfCheckCode.DevSelfCheckSensorStatusCode == 0) { //设备自检传感器正常
                if(stSelfCheckCode.MachinePartBSelfCheckCode == 0) { //发电组设备正常
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

        case EN_WORK_MODE_MALFUNCTION:  //故障模式
            APP_TRACE_INFO(("Device of machine is not at right statu,can not work...\r\n"));

        default:
            APP_TRACE_INFO(("Fault: System expected to work on the malfunction mode...\r\n"));
            MachineSelfCheckResult = EN_NOT_THROUGH;
            break;
    }

//  return MachineSelfCheckResult;
    return EN_THROUGH;//暂时关闭自检
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
 
//    SetWorkMode(EN_WORK_MODE_HYDROGEN_PRODUCER_AND_FUEL_CELL);  
//    APP_TRACE_INFO(("---HYDROGEN_PRODUCER_AND_FUEL_CELL_MODE---\r\n"));
	
	SetWorkMode(EN_WORK_MODE_FUEL_CELL);  
    APP_TRACE_INFO(("---FUEL_CELL_MODE---\r\n"));
	
    while(DEF_TRUE) {
        if(EN_THROUGH == DeviceSelfCheck()) {
            APP_TRACE_INFO(("Self-check success...\n\r"));

            OSTaskSemPend(OS_CFG_TICK_RATE_HZ * 60 * 5, //5分钟自检一次
                          OS_OPT_PEND_BLOCKING,
                          NULL,
                          &err);

            if(err == OS_ERR_NONE) {  
                APP_TRACE_INFO(("Receive the run command...\n\r"));
                WaitCmdStatu = EN_THROUGH;
                SetSystemWorkStatu(EN_START_PRGM_ONE_FRONT);
                break;
            } else if(err == OS_ERR_TIMEOUT) { 
                APP_TRACE_INFO(("Not receive the run command, restart the self-check...\n\r"));
            } else {
                APP_TRACE_INFO(("Waitting run command failed, err code %d...\n\r", err));
                WaitCmdStatu = EN_NOT_THROUGH;
                break;
            }

            SendRealTimeAssistInfo();   //向上位机发送一次自检信息
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
  
    ResetSystemIsolatedGeneratedEnergyThisTime();//复位系统单次发电量
    ResetHydrgProduceTimeThisTime();
    ResetStackProductTimeThisTime();

    SystemWorkTimesInc();
    eWorkMode = GetWorkMode();

    BSP_BuzzerOn();
    OSTimeDlyHMSM(0, 0, 0, 500, OS_OPT_TIME_HMSM_STRICT, &err);
    BSP_BuzzerOff();

    //一体机模式或者制氢模式
    if((eWorkMode == EN_WORK_MODE_HYDROGEN_PRODUCER_AND_FUEL_CELL) || (eWorkMode == EN_WORK_MODE_HYDROGEN_PRODUCER)) {
        if(EN_START_PRGM_ONE_FRONT == GetSystemWorkStatu()) {
			
            HydrgProducerWorkTimesInc();
            if(EN_WORK_MODE_FUEL_CELL != GetWorkMode()){
                OSTaskResume(&MembraneTubeProtectTaskTCB, &err); //重复启动时恢复抽真空任务，若是首次启动则无效
            }
            //第一次点火
            if((EN_PASS == IgniteFirstTime(IgniteCheckTable1, GoToNextStepTempTable1, 3, 1))) {
                SetSystemWorkStatu(EN_START_PRGM_TWO);
            } else {
                APP_TRACE_INFO(("Ignite for the first time is failed...\n\r"));

                if(GetSystemWorkStatu() != EN_WAIT_CMD) {   //上位机操作关机，导致冷启动提前结束
                    SetShutDownActionFlag(EN_DELAY_STOP_PART_ONE);
                }
            }

            //第二次点火
            if(GetSystemWorkStatu() == EN_START_PRGM_TWO) {
                IgniteCheckTable2 = g_stReformerTempCmpTbl.IgScdTimeFrtToBhdTmpPnt;
                GoToNextStepTempTable2 = g_stReformerTempCmpTbl.IgScdTimeOverTmpPnt;

                if(EN_PASS != IgniteSecondTime(IgniteCheckTable2, GoToNextStepTempTable2, 3, 1)) {
                    APP_TRACE_INFO(("Ignite for the second time is failed...\n\r"));

                    if(GetSystemWorkStatu() != EN_WAIT_CMD) {   //可能因为上位机直接将状态改为关保温，导致点火失败
                        SetShutDownActionFlag(EN_DELAY_STOP_BOTH_PARTS);
                    }
                } else {
                    SetSystemWorkStatu(EN_RUNNING);
                }

                SetShutDownActionFlag(EN_DELAY_STOP_PART_ONE);
            } else {
                APP_TRACE_INFO(("Ignite for the second time has been leaped over ...\n\r"));
                SetShutDownActionFlag(EN_DELAY_STOP_PART_ONE);//冷启动不成功，也设置延时关闭半机一动作响应标识
            }
        } else {
            APP_TRACE_INFO(("Program's running here is not to start...\n\r"));
            SetShutDownActionFlag(EN_STOP_ALL_DIRECT);
        }

    }
    //发电模式
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
void KeepingWarm(void)
{
    ShutDown();
    APP_TRACE_INFO(("Keeping warm...\n\r"));
    SetSystemWorkStatu(EN_WAIT_CMD);

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
                SetShutDownActionFlag(EN_DELAY_STOP_BOTH_PARTS);//运行到此说明一体机模式正常启动，先前在启动阶段未严格区分一体和制氢模式，故在此再次设置
                OSTaskSuspend(&AppTaskStartTCB, //阻塞主任务，由制氢机管理任务和电堆管理任务管理机器
                              &err);
                SetSystemWorkStatu(EN_SHUTTING_DOWN);
                break;

            case(u8)EN_WORK_MODE_HYDROGEN_PRODUCER:
                OSTaskResume(&HydrgProducerManagerTaskTCB,
                             &err);
//                  SetShutDownActionFlag(EN_DELAY_STOP_PART_ONE);//因制氢模式下的关机标志已经在启动阶段设置好，故在该模式下不需要另行设置
                OSTaskSuspend(&AppTaskStartTCB,
                              &err);
                SetSystemWorkStatu(EN_SHUTTING_DOWN);
                break;

            case(u8)EN_WORK_MODE_FUEL_CELL:
                OSTaskResume(&StackManagerTaskTCB,
                             &err);
//                  SetShutDownActionFlag(EN_DELAY_STOP_PART_TWO);//因发电模式下的关机标志已经在启动阶段设置好，故在该模式下不需要另行设置
                OSTaskSuspend(&AppTaskStartTCB,
                              &err);
                SetSystemWorkStatu(EN_SHUTTING_DOWN);
                break;

            default:
                APP_TRACE_INFO(("Fault: System run at the malfunction mode...\n\r"));
                //关机状态标识已经在启动阶段设置好，不需在此另行设定
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
    SYS_WORK_STATU_Typedef eWorkStatu;

    eWorkStatu = GetSystemWorkStatu();
    ResetAllAlarms();//清空所有警报

    if(eWorkStatu == EN_SHUTTING_DOWN) {
        APP_TRACE_INFO(("Exit the cruise process to shut down...\r\n"));//提示系统退出巡航运行阶段

        switch((u8)g_eShutDownActionFlag) {
            case EN_STOP_ALL_DIRECT:

                APP_TRACE_INFO(("Shutdown all direcetly!!!\r\n"));
                break;

            case EN_DELAY_STOP_PART_ONE:
				
                OSTaskResume(&HydrgProducerManagerDlyStopTaskTCB,&err);   //恢复制氢机延时关闭任务。
                             
                break;

            case EN_DELAY_STOP_PART_TWO:

                OSTaskResume(&StackManagerDlyStopTaskTCB,&err);
                             
                break;

            case(u8)EN_DELAY_STOP_BOTH_PARTS:
				
                OSTaskResume(&HydrgProducerManagerDlyStopTaskTCB,&err);   //恢复制氢机、电堆延时关闭任务 
                OSTaskResume(&StackManagerDlyStopTaskTCB,&err);                                 

                break;

            default:
                break;
        }

        BSP_BuzzerOff();//防止关机声音提示时，因初始状态偏差，出现错误

        for(i = 0; i < 4; i++) {
            BSP_BuzzerTurnover();

            OSTimeDlyHMSM(0, 0, 1, 0,OS_OPT_TIME_HMSM_STRICT,&err);
        }

        switch((u8)g_eShutDownActionFlag) {
            case(u8)EN_STOP_ALL_DIRECT:
                APP_TRACE_INFO(("--> The system has not start...\r\n"));  //直接关机，不需要关机延时
                break;

            case(u8)EN_DELAY_STOP_PART_ONE:
                APP_TRACE_INFO(("-->Delay stop part one...\r\n"));
                //此处阻塞等待制氢机延时关闭任务中的任务信号量
                OSTaskSemPend(OS_CFG_TICK_RATE_HZ * 60 * 3,   //3分钟后关闭制氢机
                              OS_OPT_PEND_BLOCKING,
                              NULL,
                              &err);

                if(err == OS_ERR_NONE) {
                    break;
                }

            case(u8)EN_DELAY_STOP_PART_TWO:
                APP_TRACE_INFO(("-->Delay stop part two...\r\n"));
                //此处阻塞等待电堆延时关闭任务中的任务信号量
                OSTaskSemPend(OS_CFG_TICK_RATE_HZ * 30,         //30s后关闭电堆
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

                OSTaskSemPend(OS_CFG_TICK_RATE_HZ * 160,//制氢机30+150秒->3分钟后关闭
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

        OSTaskSemSet(&AppTaskStartTCB, 0, &err); //清掉任务信号量，防止因任务信号量累计造成下一次启动异常
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
/******************* (C) COPYRIGHT 2016 Guangdong ENECO POWER *****END OF FILE****/
