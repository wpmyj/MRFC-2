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
* Filename      : app_top_task.c
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
#include "bsp_speed_adjust_device.h"
#include "app_top_task.h"
#include "app_system_run_cfg_parameters.h"
#include "app_system_real_time_parameters.h"
#include "app_hydrg_producer_manager.h"

/*
*********************************************************************************************************
*                                         OS-RELATED    VARIABLES
*********************************************************************************************************
*/
extern  OS_TCB   AppTaskStartTCB;
extern  OS_TCB   WirenessCommTaskTCB;

extern  OS_TCB   HydrgProducerManagerTaskTCB;
extern  OS_TCB   StackManagerTaskTCB;

extern  OS_TCB   HydrgProducerManagerDlyStopTaskTCB;
extern  OS_TCB   StackManagerDlyStopTaskTCB;

OS_SEM   IgniteFirstBehindWaitSem;

OS_SEM  IgniteSecondBehindWaitSem;
OS_SEM   MannualSelcetWorkModeSem;

/*
*********************************************************************************************************
*                                           LOCAL VARIABLES
*********************************************************************************************************
*/
extern  REFORMER_TEMP_CMP_LINES_Typedef             g_stReformerTempCmpTbl;

static  SWITCH_TYPE_VARIABLE_Typedef                g_eDeviceFaultAlarm = OFF;

static  SYSTEM_SHUT_DOWN_ACTION_FLAG_Typedef        g_eShutDownActionFlag = EN_DELAY_STOP_BOTH_PARTS;

/*
*********************************************************************************************************
*                                           GLOBAL VARIABLES
*********************************************************************************************************
*/
u16 g_u16StartRemainSecond = 15 * 60;



/*
*********************************************************************************************************
*                                         FUNCTION PROTOTYPES
*********************************************************************************************************
*/
static  void    ShutDown(void);

/*
*********************************************************************************************************
*                                          CheckAuthorization()
*
* Description : The use of this funciton is to check the authorization of the system.
*               检查权限
* Arguments   : none.
*
* Returns     : none
*
* Notes       : none.
*********************************************************************************************************
*/
VERIFY_RESULT_TYPE_VARIABLE_Typedef CheckAuthorization(void)
{
    return EN_THROUGH;
}

/*
*********************************************************************************************************
*                                          DeviceFaultAlarm()
*
* Description : The funciton is the device alarm process.
*               设备故障警报
* Arguments   : none.
*
* Returns     : none
*
* Notes       : none.
*********************************************************************************************************
*/
void DeviceFaultAlarm(void)
{
    OS_ERR      err;
    g_eDeviceFaultAlarm = ON;
    APP_TRACE_INFO(("Device fault alarming...\n\r"));
    while(DEF_TRUE)
    {
        BSP_BuzzerTurnover();

        if(g_eDeviceFaultAlarm == OFF)
        {
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
*********************************************************************************************************
*                                          ResetDeviceAlarmStatu()
*
* Description : The use of the funciton is to exit the device alarm process.
*
* Arguments   : none.
*
* Returns     : none
*
* Notes       : none.
*********************************************************************************************************
*/
void ResetDeviceAlarmStatu(void)
{
    g_eDeviceFaultAlarm = OFF;
}

/*
*********************************************************************************************************
*                                          DeviceSelfCheck()
*
* Description : The use of the funciton is to self-check the devices.
*
* Arguments   : none.
*
* Returns     : none.
*
* Notes       : none.
*********************************************************************************************************
*/
VERIFY_RESULT_TYPE_VARIABLE_Typedef DeviceSelfCheck(void)
{
    uint64_t TempSelfCheckCode;
    VERIFY_RESULT_TYPE_VARIABLE_Typedef MachineSelfCheckResult;
    SYSTEM_WORK_MODE_Typedef    eWorkMode;

    APP_TRACE_INFO(("Self-checking...\n\r"));
    ResetAllAlarms();
    AnaSensorSelfCheck();   //模拟信号传感器自检
    
    TempSelfCheckCode = GetSelfCheckCode();

    eWorkMode = GetWorkMode();

    switch((u8)eWorkMode)
    {
        case EN_WORK_MODE_HYDROGEN_PRODUCER_AND_FUEL_CELL:
            APP_TRACE_INFO(("Both parts of the machine are expected to work...\n\r"));

            if((TempSelfCheckCode & (0x201ll << 47)) == 0ll)//制氢组设备正常
            {
                APP_TRACE_INFO(("Device of hydrogen producer group is at right statu...\n\r"));

                if((TempSelfCheckCode & (0xFll << 18)) == 0ll)//发电组设备正常
                {
                    APP_TRACE_INFO(("Device of stack group is at right statu, the machine can work normal...\n\r"));
                    MachineSelfCheckResult = EN_THROUGH;
                }
                else
                {
                    APP_TRACE_INFO(("Device of stack group is not at right statu,can not work...\n\r"));
                    MachineSelfCheckResult = EN_NOT_THROUGH;
                }
            }
            else
            {
                APP_TRACE_INFO(("Device of hydrogen producer group is not at right statu,can not work...\n\r"));
                MachineSelfCheckResult = EN_NOT_THROUGH;
            }

            break;

        case EN_WORK_MODE_HYDROGEN_PRODUCER:
            APP_TRACE_INFO(("Hydrogen producer of the machine is expected to work...\n\r"));

            if((TempSelfCheckCode & (0x201ll << 47)) == 0ll)//制氢组设备正常
            {
                APP_TRACE_INFO(("Device of hydrogen producer group is at right statu,can work normal...\n\r"));
                MachineSelfCheckResult = EN_THROUGH;        //手动控制下，制氢模式自检通过
            }
            else
            {
                APP_TRACE_INFO(("Device of hydrogen producer group is not at right statu,can not work...\n\r"));
                MachineSelfCheckResult = EN_NOT_THROUGH;
            }

            break;

        case EN_WORK_MODE_FUEL_CELL:
            APP_TRACE_INFO(("Stack of the machine is expected to work...\n\r"));

            if((TempSelfCheckCode & (0xFll << 18)) == 0ll)//发电组设备正常
            {
                APP_TRACE_INFO(("Device of stack group is at right statu,can work normal...\n\r"));
                MachineSelfCheckResult = EN_THROUGH;        //手动控制下，发电模式自检通过
            }
            else
            {
                APP_TRACE_INFO(("Device of stack group is not at right statu,can not work...\n\r"));
                MachineSelfCheckResult = EN_NOT_THROUGH;
            }

            break;

        case EN_WORK_MODE_MALFUNCTION:

        default:
            APP_TRACE_INFO(("Fault: System expected to work on the malfunction mode...\n\r"));
            MachineSelfCheckResult = EN_NOT_THROUGH;
            break;
    }

    //  return MachineSelfCheckResult;
    return EN_THROUGH;//暂时关闭自检
}
/*
*********************************************************************************************************
*                                          WaittingCommand()
*
* Description : The use of the funciton is to wait the command and cyclic self-check.
*                               待机状态，等待上位机命令并周期自检
* Arguments   : none.
*
* Returns     : none.
*
* Notes       : none.
*********************************************************************************************************
*/
VERIFY_RESULT_TYPE_VARIABLE_Typedef WaittingCommand(void)
{
    OS_ERR      err;
    VERIFY_RESULT_TYPE_VARIABLE_Typedef WaitCmdStatu;
//    SYSTEM_WORK_MODE_Typedef    eWorkMode;

#if __INTERNAL_TEST_FLAG > 0u

    APP_TRACE_INFO(("Waitting for the command to select the work mode...\n\r"));
    SetWorkModeWaittingForSelectFlag();     //打开等待运行模式选择等待标志，提示上位机，需要立即指定工作模式
    OSTimeDlyResume(&WirenessCommTaskTCB,
                    &err);                  //立即发送数据，以便及时在上位机界面反映下位机正在等待指令的状态
    OSSemPend(&MannualSelcetWorkModeSem,
              0,
              OS_OPT_PEND_BLOCKING,
              NULL,
              &err);
    ResetWorkModeWaittingForSelectFlag();   //复位等待标志

    eWorkMode = GetWorkMode();
    APP_TRACE_INFO(("Receive the mannual work mode %d...\n\r", eWorkMode));
#else
    SetWorkMode(EN_WORK_MODE_FUEL_CELL);//发电半机模式
#endif

    while(DEF_TRUE)
    {
        if(EN_THROUGH == DeviceSelfCheck())
        {
            APP_TRACE_INFO(("Self-check success...\n\r"));

            OSTaskSemPend(OS_CFG_TICK_RATE_HZ * 60 * 1, //每分钟自检一次
                          OS_OPT_PEND_BLOCKING,
                          NULL,
                          &err);

            if(err == OS_ERR_NONE)    //正确得到信号量，开始运行
            {
                APP_TRACE_INFO(("Receive the run command...\n\r"));
                WaitCmdStatu = EN_THROUGH;
                SetSystemWorkStatu(EN_START_PRGM_ONE_FRONT);
                break;
            }
            else if(err == OS_ERR_TIMEOUT)   //等待超时，重新自检
            {
                APP_TRACE_INFO(("Not receive the run command, restart the self-check...\n\r"));
            }
            else
            {
                APP_TRACE_INFO(("Waitting run command failed, err code %d...\n\r", err));
                WaitCmdStatu = EN_NOT_THROUGH;
                break;
            }
        }
        else
        {
            APP_TRACE_INFO(("Self-check failed...\n\r"));
            WaitCmdStatu = EN_NOT_THROUGH;
            break;
        }
    }

    return WaitCmdStatu;
}

/*
*********************************************************************************************************
*                                          Starting()
*
* Description : The use of the funciton is to start the system.
*
* Arguments   : none.
*
* Returns     : none.
*
* Notes       : none.
*********************************************************************************************************
*/
void Starting(void)
{
    SYSTEM_WORK_MODE_Typedef eWorkMode;
    SYSTEM_TIME_Typedef m_stCURRENT_SYSTEM_TIME;
    SYSTEM_TIME_Typedef m_stSYSTEM_TIME_RECORD_FOR_START_RAMAIN;

    float IgniteCheckTable1, GoToNextStepTempTable1;
    float IgniteCheckTable2, GoToNextStepTempTable2;

    IgniteCheckTable1 = g_stReformerTempCmpTbl.IgFstTimeFrtToBhdTmpPnt;
    GoToNextStepTempTable1 = g_stReformerTempCmpTbl.IgFstTimeOverTmpPnt;

    //复位系统单次发电量
    ResetSystemIsolatedGeneratedEnergyThisTime();
    ResetHydrgProduceTimeThisTime();
    ResetStackProductTimeThisTime();

    SystemWorkTimesInc();
    //系统启动剩余时间计算
    m_stSYSTEM_TIME_RECORD_FOR_START_RAMAIN = GetSystemTime();
    m_stCURRENT_SYSTEM_TIME = GetSystemTime();

    if((((m_stCURRENT_SYSTEM_TIME.hour - m_stSYSTEM_TIME_RECORD_FOR_START_RAMAIN.hour) * 60
            + m_stCURRENT_SYSTEM_TIME.minute - m_stSYSTEM_TIME_RECORD_FOR_START_RAMAIN.minute) * 60
            + m_stCURRENT_SYSTEM_TIME.second - m_stSYSTEM_TIME_RECORD_FOR_START_RAMAIN.second)
            >= 1)
    {
        g_u16StartRemainSecond--;
        m_stSYSTEM_TIME_RECORD_FOR_START_RAMAIN = m_stCURRENT_SYSTEM_TIME;

        if( g_u16StartRemainSecond == 0 || g_u16StartRemainSecond > 900 )
        {
            /*越界处理*/
        }
    }

    eWorkMode = GetWorkMode();

    //一体机模式或者制氢模式
    if(( eWorkMode == EN_WORK_MODE_HYDROGEN_PRODUCER_AND_FUEL_CELL ) || ( eWorkMode == EN_WORK_MODE_HYDROGEN_PRODUCER ))
    {  
        OS_ERR err;
        HydrgProducerWorkTimesInc();                             //开始制氢计算时间      
        SetSystemWorkStatu(EN_START_PRGM_ONE_FRONT);  
        
        APP_TRACE_INFO(("Fast heat control...\n\r"));
        SetHydrgProducerDigSigIgniteFirstTimeBehindMonitorHookSwitch(DEF_ENABLED);
        BSP_FastHeaterPwrOn();                                //打开快速加热器
        OSSemPend(&IgniteFirstBehindWaitSem,                      //  因有多个之一的条件满足即可，且有多处等待点，故使用信号量传递信息，而非任务信号量。
                  ( OS_CFG_TICK_RATE_HZ * 60 * 3 ),                 //三分钟时间
                  OS_OPT_PEND_BLOCKING,
                  NULL,
                  &err);
                           
        if( err == OS_ERR_NONE )                             
        {
            APP_TRACE_INFO(("Fast heat control finish...\n\r"));
            SetSystemWorkStatu(EN_START_PRGM_ONE_FRONT);            //直接跳到准备运行前的第二次点火
            return;
        }
        else if( err == OS_ERR_TIMEOUT )
        {
            APP_TRACE_INFO(("Fast heat control : OS_ERR_TIMEOUT...\n\r"));
            SetSystemWorkStatu( EN_START_PRGM_ONE_FRONT );
        }
        
        else
        {
            APP_TRACE_INFO(("Fast heat control, error code : %d...\n\r", err));
        }
    

        if( GetSystemWorkStatu() == EN_START_PRGM_ONE_FRONT )
        {
  //        HydrgProducerWorkTimesInc();     

            //第一次点火
            if(( EN_PASS == IgniteFirstTime(IgniteCheckTable1, GoToNextStepTempTable1, 3, 1 )))
            {
                SetSystemWorkStatu( EN_START_PRGM_TWO );
            }
            else
            {
                APP_TRACE_INFO(("Ignite for the first time is failed...\n\r"));
                if( GetSystemWorkStatu() != EN_WAITTING_COMMAND )   //可能因为上位机直接将状态改为关保温，导致点火失败
                {
                    SetSystemWorkStatu( EN_KEEPING_WARM );
                }
            }

            //第二次点火
            if( GetSystemWorkStatu() == EN_START_PRGM_TWO )
            {
                IgniteCheckTable2 = g_stReformerTempCmpTbl.IgScdTimeFrtToBhdTmpPnt;
                GoToNextStepTempTable2 = g_stReformerTempCmpTbl.IgScdTimeOverTmpPnt;

                if( EN_PASS != IgniteSecondTime(IgniteCheckTable2, GoToNextStepTempTable2, 3, 1) )
                {
                    APP_TRACE_INFO(("Ignite for the second time is failed...\n\r"));

                    if( GetSystemWorkStatu() != EN_WAITTING_COMMAND )   //可能因为上位机直接将状态改为关保温，导致点火失败
                    {
                        SetSystemWorkStatu( EN_KEEPING_WARM );
                    }
                }
                else
                {
                    SetSystemWorkStatu( EN_RUNNING );
                }

                SetShutDownActionFlag( EN_DELAY_STOP_PART_ONE );
            }
            //第一次点火不成功，直接停止
            else
            {
                APP_TRACE_INFO( ("Ignite for the second time has been leaped over ...\n\r") );
                SetShutDownActionFlag(  EN_STOP_ALL_DIRECT );
            }
        }
        else
        {
            APP_TRACE_INFO( ("Program's running here is not to start...\n\r") );
            SetShutDownActionFlag( EN_STOP_ALL_DIRECT );
        }
        
    }
    //发电模式
    else if( eWorkMode == EN_WORK_MODE_FUEL_CELL )
    {
        APP_TRACE_INFO(("Only stack of the machine is excepted to work in the current work mode...\n\r"));
        SetSystemWorkStatu( EN_RUNNING );
        SetShutDownActionFlag( EN_DELAY_STOP_PART_TWO );
    }
    else
    {
        APP_TRACE_INFO(("Fault: The program should not come here in the malfunciton mode...\n\r"));
        SetSystemWorkStatu( EN_KEEPING_WARM );
        SetShutDownActionFlag( EN_STOP_ALL_DIRECT );
    }
}

/*
*********************************************************************************************************
*                                          KeepingWarm()
*
* Description : The use of the funciton is to keep the system warm.
*
* Arguments   : none.
*
* Returns     : none.
*
* Notes       : none.
*********************************************************************************************************
*/
void    KeepingWarm(void)
{
    ShutDown();
    APP_TRACE_INFO(("Keeping warm...\n\r"));
    SetSystemWorkStatu( EN_WAITTING_COMMAND );
   
}

/*
*********************************************************************************************************
*                                          Running()
*
* Description : The use of the funciton is to keep the system run.
*
* Arguments   : none.
*
* Returns     : none.
*
* Notes       : none.
*********************************************************************************************************
*/
void Running()
{
    OS_ERR      err;
    SYSTEM_WORK_MODE_Typedef eWorkMode;

    if( GetSystemWorkStatu() == EN_RUNNING )
    {
         
        APP_TRACE_INFO(("Run: System enter the cruise process...\n\r"));    //提示系统进入巡航运行阶段
        eWorkMode = GetWorkMode();

        switch((u8)eWorkMode)
        {
            case(u8)EN_WORK_MODE_HYDROGEN_PRODUCER_AND_FUEL_CELL:
                OSTaskResume(&HydrgProducerManagerTaskTCB,  //制氢机管理任务开始运行
                             &err);
                OSTaskResume(&StackManagerTaskTCB,          //电堆管理任务开始运行
                             &err);
                SetShutDownActionFlag(EN_DELAY_STOP_BOTH_PARTS);//运行到此说明一体机模式正常启动，先前在启动阶段未严格区分一体和制氢模式，故在此再次设置
                OSTaskSuspend(&AppTaskStartTCB,             //阻塞主任务，由制氢机管理任务和电堆管理任务管理机器
                              &err);
                SetSystemWorkStatu(EN_SHUTTING_DOWN);
                break;

            case(u8)EN_WORK_MODE_HYDROGEN_PRODUCER:
                OSTaskResume(&HydrgProducerManagerTaskTCB,  //制氢机管理任务开始运行
                             &err);
//                  SetShutDownActionFlag(EN_DELAY_STOP_PART_ONE);//因制氢模式下的关机标志已经在启动阶段设置好，故在该模式下不需要另行设置
                OSTaskSuspend(&AppTaskStartTCB,             //阻塞主任务，由制氢机管理任务和电堆管理任务管理机器
                              &err);
                SetSystemWorkStatu(EN_SHUTTING_DOWN);
                break;

            case(u8)EN_WORK_MODE_FUEL_CELL:
                OSTaskResume(&StackManagerTaskTCB,          //电堆管理任务开始运行
                             &err);
//                  SetShutDownActionFlag(EN_DELAY_STOP_PART_TWO);//因发电模式下的关机标志已经在启动阶段设置好，故在该模式下不需要另行设置
                OSTaskSuspend(&AppTaskStartTCB,             //阻塞主任务，由制氢机管理任务和电堆管理任务管理机器
                              &err);
                SetSystemWorkStatu( EN_SHUTTING_DOWN );
                break;

            default:
                APP_TRACE_INFO(("Fault: System run at the malfunction mode...\n\r"));
                //关机状态标识已经在启动阶段设置好，不需在此另行设定
                SetSystemWorkStatu( EN_KEEPING_WARM );
                break;
        }
    }
    else
    {
        APP_TRACE_INFO(("Ignite befor the run process is failed or the run process is leaped over...\n\r"));
    }
}

/*
*********************************************************************************************************
*                                          SetShutDownActionFlag()
*
* Description : The use of the funciton is to select the shut down action flag.
*               设置停机模式标志
* Arguments   : none.
*
* Returns     : none.
*
* Notes       : none.
*********************************************************************************************************
*/
void SetShutDownActionFlag(SYSTEM_SHUT_DOWN_ACTION_FLAG_Typedef i_eNewActionStatu)
{
    g_eShutDownActionFlag = i_eNewActionStatu;
}

/*
*********************************************************************************************************
*                                                   ShutDown()
*
* Description : The use of the funciton is to shut down the system.
*
* Arguments   : none.
*
* Returns     : none.
*
* Notes       : none.
*********************************************************************************************************
*/
void ShutDown()
{
    OS_ERR  err;
    uint8_t i;
    SYSTEM_WORK_STATU_Typedef eWorkStatu;
   
    eWorkStatu = GetSystemWorkStatu();
    if(eWorkStatu == EN_SHUTTING_DOWN) {
        APP_TRACE_DEBUG(("Exit the cruise process to shut down...\r\n"));//提示系统退出巡航运行阶段

        switch((u8)g_eShutDownActionFlag) {
            case EN_STOP_ALL_DIRECT:
                
                APP_TRACE_DEBUG(("Shutdown all direcetly!!!\r\n"));//不需要延时，直接关机
                break;

            case EN_DELAY_STOP_PART_ONE:
                OSTaskResume(&HydrgProducerManagerDlyStopTaskTCB,   //恢复制氢机延时关闭任务。
                             &err);
                break;

            case EN_DELAY_STOP_PART_TWO:
                OSTaskResume(&StackManagerDlyStopTaskTCB,           //恢复电堆延时关闭任务。
                                 &err);

                break;

            case(u8)EN_DELAY_STOP_BOTH_PARTS:
                OSTaskResume(&HydrgProducerManagerDlyStopTaskTCB,   //恢复制氢机、电堆延时关闭任务
                             &err);
                OSTaskResume(&StackManagerDlyStopTaskTCB,
                             &err);
                break;
            default:
                break;
        }

        BSP_BuzzerOff();//防止关机声音提示时，因初始状态偏差，出现错误

        for(i = 0; i < 4; i++) {
            BSP_BuzzerTurnover();
            
            OSTimeDlyHMSM(0, 0, 1, 0,
                          OS_OPT_TIME_HMSM_STRICT,
                          &err);
        }

        //下面的任务信号量由制氢机和电堆延时关闭任务中发送
        switch((u8)g_eShutDownActionFlag) {
            case(u8)EN_STOP_ALL_DIRECT:
                APP_TRACE_DEBUG(("--> The system has not start...\r\n"));  //直接关机，不需要关机延时
                break;

            case(u8)EN_DELAY_STOP_PART_ONE:
                APP_TRACE_DEBUG(("-->Delay stop part one...\r\n"));
                //此处阻塞等待制氢机延时关闭任务中的任务信号量
                OSTaskSemPend(OS_CFG_TICK_RATE_HZ * 60 * 5,         //5分钟后关闭制氢机
                              OS_OPT_PEND_BLOCKING,
                              NULL,
                              &err);

                if(err == OS_ERR_NONE) { //收到任务任务信号量
                    break;
                }

            case(u8)EN_DELAY_STOP_PART_TWO:
                APP_TRACE_DEBUG(("-->Delay stop part two...\r\n"));
                //此处阻塞等待电堆延时关闭任务中的任务信号量
                OSTaskSemPend(OS_CFG_TICK_RATE_HZ * 2 *60,         //2分钟后关闭电堆
                              OS_OPT_PEND_BLOCKING,
                              NULL,
                              &err);

                if(err == OS_ERR_NONE) { //收到任务任务信号量
                    break;
                }

            case(u8)EN_DELAY_STOP_BOTH_PARTS:
                APP_TRACE_DEBUG(("--> Delay stop both part...\r\n"));
                OSTaskSemPend(OS_CFG_TICK_RATE_HZ * 2, //2分钟后关闭电堆
                              OS_OPT_PEND_BLOCKING,
                              NULL,
                              &err);

                if(err == OS_ERR_NONE) { //收到电堆延时停止任务信号量，等待制氢机延时停止任务信号量
                    APP_TRACE_DEBUG(("The stack delay stop task finished,wait hdygren delay stop task...\r\n"));
                    OSTaskSemPend(OS_CFG_TICK_RATE_HZ * 3 * 61, //制氢机2+3分钟后关闭(可比5分钟稍长，嵌套Pend可能导致时间变长)
                                  OS_OPT_PEND_BLOCKING,
                                  NULL,
                                  &err);

                    if(err == OS_ERR_NONE) {
                        APP_TRACE_DEBUG(("The hydrogen producer manager delay stop task finished...\r\n"));
                        break;
                    } else {
                        //出现超时等其它错误，清零任务信号量计数，防止任务信号量累积
                        OSTaskSemSet(&HydrgProducerManagerDlyStopTaskTCB,
                                     0,
                                     &err);
                        break;
                    }
                } else {
                    APP_TRACE_DEBUG(("Fault: wait for the sem of the stack delay stop failed...\r\n"));
                }

            default:
                break;
        }
    } else {
        APP_TRACE_DEBUG(("The program don't need to start the shut down process...\r\n"));
    }
}

/*
*********************************************************************************************************
*                                         UpdateBuzzerStatuInCruise()
*
* Description : The use of the funciton is to update the buzzer statu when running.
*
* Arguments   : none.
*
* Returns     : none.
*
* Notes       : none.
*********************************************************************************************************
*/
void UpdateBuzzerStatuInCruise(void)
{
    u32 u32AlarmCode;
    u32AlarmCode = GetRunAlarmCode();

    if(u32AlarmCode != 0)
    {
        BSP_BuzzerOn();
    }
    else
    {
        BSP_BuzzerOff();
    }
}


/*
*********************************************************************************************************
*                                         u16 GetStartRemainSencond(void)
*
* Description : The use of the funciton is to get the start remain sencond.
*
* Arguments   : none.
*
* Returns     : none.
*
* Notes       : none.
*********************************************************************************************************
*/
u16 GetStartRemainSencond(void)
{
    return g_u16StartRemainSecond;
}







/******************* (C) COPYRIGHT 2015 Guangdong Hydrogen *****END OF FILE****/

