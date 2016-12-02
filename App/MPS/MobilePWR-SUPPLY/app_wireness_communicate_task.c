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
* Filename      : app_wireness_communicate_task.c
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
#include "app_wireness_communicate_task.h"
#include "app_top_task.h"
#include "bsp_speed_adjust_device.h"
#include "app_hydrg_producer_manager.h"
#include "app_stack_manager.h"

/*
*********************************************************************************************************
*                                           MACRO DEFINITIONS
*********************************************************************************************************
*/
#define WIRENESS_COMMUNICATE_TASK_STK_SIZE      200

#define PRGM_TX_BUFF_SIZE       146
#define PRGM_RX_BUFF_SIZE       10
/*
*********************************************************************************************************
*                                         OS-RELATED    VARIABLES
*********************************************************************************************************
*/
extern      OS_SEM      g_stWirenessCommSem;
extern      OS_SEM      IgniteFirstBehindWaitSem;

extern      OS_SEM      IgniteSecondBehindWaitSem;

extern      OS_SEM      MannualSelcetWorkModeSem;
extern      OS_TCB      AppTaskStartTCB;
extern      OS_TCB      HydrgProducerManagerTaskTCB;
extern      OS_TCB      StackManagerTaskTCB;



OS_TCB      WirenessCommTaskTCB;
static      CPU_STK     WirenessCommTaskStk[WIRENESS_COMMUNICATE_TASK_STK_SIZE];

/*
*********************************************************************************************************
*                                           LOCAL VARIABLES
*********************************************************************************************************
*/
extern      RUNNING_ALARM_STATUS_Typedef            g_stSystemAlarmsInf;
WHETHER_TYPE_VARIABLE_Typedef           g_eWirenessCommandReceived = NO;

static              uint8_t                         g_PrgmTxMsgBuff[PRGM_TX_BUFF_SIZE];
static              uint8_t                         g_PrgmRxMsgBuff[PRGM_RX_BUFF_SIZE];

/*
*********************************************************************************************************
*                                         FUNCTION PROTOTYPES
*********************************************************************************************************
*/

static      void        APP_PrgmMsgInit(void);
static      void        WirenessCommTask(void *p_arg);
static      void        AnwserPrgmCommand(void);

static      void        UpdatePrgmTxMsgUintByte(PRGM_TX_MSG_DATA_ADDR_Typedef, uint8_t);
static      void        UpdatePrgmTxMsgIntByte(PRGM_TX_MSG_DATA_ADDR_Typedef, int8_t);

static      void        UpdatePrgmTxMsg(void);
static      void        SendPrgmMsg(void);

/*
*********************************************************************************************************
*                                      APP_PrgmMsgInit()
*
* Description:  This function init head of the message that will send out.
*               初始化串口发送数据的包头
* Arguments  :  none
*
* Returns    :  none
*********************************************************************************************************
*/
void APP_PrgmMsgInit()
{
    g_PrgmTxMsgBuff[0] = 0xF1;
    g_PrgmTxMsgBuff[1] = 0xF2;
}

/*
*********************************************************************************************************
*                                          WirenessCommTaskCreate()
*
* Description : The use of the the funciton is to create the task that manager the communication.
*
* Arguments   : none.
*
* Returns     : none
*
* Notes       : none
*********************************************************************************************************
*/
void  WirenessCommTaskCreate(void)
{
    OS_ERR  err;

    APP_PrgmMsgInit();      //初始化串口发送数据包头
    BSP_SerToWIFI_Init();       //(WIFI)串口初始化

    BSP_SerToWIFI_TxMsgInit(g_PrgmTxMsgBuff, PRGM_TX_BUFF_SIZE);
    BSP_SerToWIFI_RxMsgInit(g_PrgmRxMsgBuff, PRGM_RX_BUFF_SIZE);

    OSTaskCreate((OS_TCB *)&WirenessCommTaskTCB,                    // Create the start task
                 (CPU_CHAR *)"Wireness Communicate Task Start",
                 (OS_TASK_PTR) WirenessCommTask,
                 (void *) 0,
                 (OS_PRIO) WIRENESS_COMMUNICATE_TASK_PRIO,
                 (CPU_STK *)&WirenessCommTaskStk[0],
                 (CPU_STK_SIZE) WIRENESS_COMMUNICATE_TASK_STK_SIZE / 10,
                 (CPU_STK_SIZE) WIRENESS_COMMUNICATE_TASK_STK_SIZE,
                 (OS_MSG_QTY) 5u,
                 (OS_TICK) 0u,
                 (void *) 0,
                 (OS_OPT)(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR *)&err);
    APP_TRACE_INFO(("Created wireness communication Task, and err code is %d...\n\r", err));
}

/*
*********************************************************************************************************
*                                      WirenessCommTask()
*
* Description:  This is a task that manage the communication with other device.
*
* Arguments  :  none
*
* Returns    :  none
*********************************************************************************************************
*/
void  WirenessCommTask(void *p_arg)
{
    OS_ERR      err;

    while(DEF_TRUE)
    {
        OSTimeDlyHMSM(0, 0, 1, 000,
                      OS_OPT_TIME_HMSM_STRICT,
                      &err);

        if(g_eWirenessCommandReceived == YES)//是否因收到指令而提前结束延时
        {
            AnwserPrgmCommand();    //响应上位机命令
            g_eWirenessCommandReceived = NO;
        }

        UpdatePrgmTxMsg();  //发送数据包更新
        SendPrgmMsg();      //DMA发送数据
    }
}

/*
*********************************************************************************************************
*                                      UpdatePrgmTxMsg()
*
* Description:  The use of the funciton update the message that will sendout.
*               发送数据包更新
* Arguments  :  none
*
* Returns    :  none
*********************************************************************************************************
*/
void UpdatePrgmTxMsg(void)
{
    u16     u16CurrentTemp;
    SYSTEM_TIME_Typedef     stHydrgProduceTimeThisTime, stHydrgProduceTimeTotal;
    SYSTEM_TIME_Typedef     stStackWorkTimeThisTime, stStackWorkTimeTotal;
    u16     u16HydrgWorkTimes;
    u8      m_u8PumpCtlSpd , m_u8PumpFeedBackSpd;
    u16     u16HydrgFanCtlSpd , u16HydrgFanFeedBackSpd;
    u16     u16StackFanCtlSpd, u16StackFan1SpdFeedBack, u16StackFan2SpdFeedBack;
    float   fStackCurrentMul100, fStackVoltageMul100, fStackTemp, fHydrgPressMul100, fStackPower;
    double dIsolatedGenratedEnergyThisTime;
    uint16_t m_u16LiquidPress;
    uint64_t u64SelfCheckCode;
    u32 u32AlarmCode;
    u32 u32SystemWorkStatuCode;
    //获取自检码
    u64SelfCheckCode = GetSelfCheckCode();
    g_PrgmTxMsgBuff[SELF_CHECK_CODE_BYTE_8] = (u8)(u64SelfCheckCode >> 56);
    g_PrgmTxMsgBuff[SELF_CHECK_CODE_BYTE_7] = (u8)((u64SelfCheckCode & 0xFF000000000000) >> 48);
    g_PrgmTxMsgBuff[SELF_CHECK_CODE_BYTE_6] = (u8)((u64SelfCheckCode & 0xFF0000000000) >> 40);
    g_PrgmTxMsgBuff[SELF_CHECK_CODE_BYTE_5] = (u8)((u64SelfCheckCode & 0xFF00000000) >> 32);
    g_PrgmTxMsgBuff[SELF_CHECK_CODE_BYTE_4] = (u8)((u64SelfCheckCode & 0xFF000000) >> 24);
    g_PrgmTxMsgBuff[SELF_CHECK_CODE_BYTE_3] = (u8)((u64SelfCheckCode & 0xFF0000) >> 16);
    g_PrgmTxMsgBuff[SELF_CHECK_CODE_BYTE_2] = (u8)((u64SelfCheckCode & 0xFF00) >> 8);
    g_PrgmTxMsgBuff[SELF_CHECK_CODE_BYTE_1] = (u8)(u64SelfCheckCode & 0xFF);

    //运行警报信息
    u32AlarmCode = GetRunAlarmCode();
    g_PrgmTxMsgBuff[RUN_ALARM_CODE_BYTE_4] = (u8)(u32AlarmCode >> 24);
    g_PrgmTxMsgBuff[RUN_ALARM_CODE_BYTE_3] = (u8)((u32AlarmCode & 0xFF0000) >> 16);
    g_PrgmTxMsgBuff[RUN_ALARM_CODE_BYTE_2] = (u8)((u32AlarmCode & 0xFF00) >> 8);
    g_PrgmTxMsgBuff[RUN_ALARM_CODE_BYTE_1] = (u8)(u32AlarmCode & 0xFF);
    //系统运行状态码
    u32SystemWorkStatuCode = GetSystemRunningStatuCode();
    g_PrgmTxMsgBuff[RUNNING_STATU_CODE_BYDE_4] = (u8)(u32SystemWorkStatuCode >> 24);
    g_PrgmTxMsgBuff[RUNNING_STATU_CODE_BYDE_3] = (u8)((u32SystemWorkStatuCode & 0xFF0000) >> 16);
    g_PrgmTxMsgBuff[RUNNING_STATU_CODE_BYDE_2] = (u8)((u32SystemWorkStatuCode & 0xFF00) >> 8);
    g_PrgmTxMsgBuff[RUNNING_STATU_CODE_BYDE_1] = (u8)(u32SystemWorkStatuCode & 0xFF);

    //在热电偶读取中更新
    //重整室温度
    u16CurrentTemp = (u16)GetReformerTemp();
    g_PrgmTxMsgBuff[REFORMER_TEMP_HIGH] = (u8)((u16CurrentTemp & 0xFF00) >> 8);
    g_PrgmTxMsgBuff[REFORMER_TEMP_LOW] = (u8)((u16CurrentTemp & 0xFF));
    //火焰温度
    u16CurrentTemp = (u16)GetFireOrRodTemp();
    g_PrgmTxMsgBuff[FIRE_OR_ROD_TEMP_HIGH] = (u8)((u16CurrentTemp & 0xFF00) >> 8);
    g_PrgmTxMsgBuff[FIRE_OR_ROD_TEMP_LOW] = (u8)((u16CurrentTemp & 0xFF));
    //液压
    m_u16LiquidPress = (uint16_t)(GetSrcAnaSig(LIQUID_PRESS) * 100);
    g_PrgmTxMsgBuff[LIQUID_PRESS_INTEGER_PART] = (u8)(m_u16LiquidPress / 100);
    g_PrgmTxMsgBuff[LIQUID_PRESS_DECIMAL_PART] = (u8)(m_u16LiquidPress % 100);


    //制氢机风机控制速度
    u16HydrgFanCtlSpd = (uint16_t) GetHydrgFanCtlSpd();
    g_PrgmTxMsgBuff[HYDROGEN_FAN_SPD_CONTROL_HIGH] = (u8)(u16HydrgFanCtlSpd >> 8);
    g_PrgmTxMsgBuff[HYDROGEN_FAN_SPD_CONTROL_LOW] = (u8)((u16HydrgFanCtlSpd & 0xFF));
    //制氢机风机反馈速度
    u16HydrgFanFeedBackSpd = (uint16_t)GetHydrgFanFeedBackSpd();
    g_PrgmTxMsgBuff[HYDROGEN_FAN_SPD_FEEDBACK_HIGH] = (u8)(u16HydrgFanFeedBackSpd >> 8);
    g_PrgmTxMsgBuff[HYDROGEN_FAN_SPD_FEEDBACK_LOW] = (u8)((u16HydrgFanFeedBackSpd & 0xFF));

    //水泵控制转速
    m_u8PumpCtlSpd = (uint8_t)GetPumpCtlSpd();
//  UpdatePrgmTxMsgUintByte(PUMP_SPD_CONTROL_HIGH, (m_u8PumpCtlSpd & 0xFF00) >> 8));
    g_PrgmTxMsgBuff[PUMP_SPD_CONTROL_LOW] = (u8)m_u8PumpCtlSpd;
    //水泵反馈转速
    m_u8PumpFeedBackSpd = (uint8_t)GetPumpFeedBackSpd();
    g_PrgmTxMsgBuff[PUMP_SPD_FEEDBACK_LOW] = (u8)m_u8PumpFeedBackSpd;

    //电堆电流*100
    fStackCurrentMul100 = GetSrcAnaSig(STACK_CURRENT) * 100;
    g_PrgmTxMsgBuff[STACK_CURRENT_INTEGER_PART] = (u8)((u16)fStackCurrentMul100 / 100);
    g_PrgmTxMsgBuff[STACK_CURRENT_DECIMAL_PART] = (u8)((u16)fStackCurrentMul100 % 100);
   // APP_TRACE_INFO(("The Ivalue is %f...\n\r", fStackCurrentMul100));
    //电堆电压*100
    fStackVoltageMul100 = GetSrcAnaSig(STACK_VOLTAGE) * 100;
    g_PrgmTxMsgBuff[STACK_VOLTAGE_INTEGER_PART] = (u8)((uint16_t) fStackVoltageMul100 / 100);
    g_PrgmTxMsgBuff[STACK_VOLTAGE_DECIMAL_PART] = (u8)((uint16_t) fStackVoltageMul100 % 100);
  //  APP_TRACE_INFO(("The Vvalue is %f...\n\r", fStackVoltageMul100));
    //电堆温度
    fStackTemp = GetSrcAnaSig(STACK_TEMP);
    UpdatePrgmTxMsgIntByte(STACK_TEMP_INTEGER_PART, (int)fStackTemp);

    //制氢机压力*100
    fHydrgPressMul100 = GetSrcAnaSig(HYDROGEN_PRESS_1) * 100;
    g_PrgmTxMsgBuff[HYDROGEN_PRESS_INTEGER_PART] = (u8)((uint16_t)fHydrgPressMul100 / 100);
    g_PrgmTxMsgBuff[HYDROGEN_PRESS_DECIMAL_PART] = (u8)((uint16_t)fHydrgPressMul100 % 100);


    //电堆风扇控制速度
    u16StackFanCtlSpd = (uint16_t)GetStackFanCtlSpd();
    UpdatePrgmTxMsgUintByte(STACK_FAN_SPD_CONTROL_HIGH, (uint8_t)((u16StackFanCtlSpd & 0xFF00) >> 8));
    UpdatePrgmTxMsgUintByte(STACK_FAN_SPD_CONTROL_LOW, (uint8_t)(u16StackFanCtlSpd & 0xFF));
    //电堆风扇1反馈速度
    u16StackFan1SpdFeedBack = (uint16_t)GetStackFan1SpdFeedBack();
    UpdatePrgmTxMsgUintByte(STACK_FAN_PART_A_SPD_FEEDBACK_HIGH, (uint8_t)((u16StackFan1SpdFeedBack & 0xFF00) >> 8));
    UpdatePrgmTxMsgUintByte(STACK_FAN_PART_A_SPD_FEEDBACK_LOW, (uint8_t)(u16StackFan1SpdFeedBack & 0xFF));

    //电堆风扇2反馈速度
    u16StackFan2SpdFeedBack = (uint16_t)GetStackFan2SpdFeedBack();
    UpdatePrgmTxMsgUintByte(STACK_FAN_PART_B_SPD_FEEDBACK_HIGH, (uint8_t)((u16StackFan2SpdFeedBack & 0xFF00) >> 8));
    UpdatePrgmTxMsgUintByte(STACK_FAN_PART_B_SPD_FEEDBACK_LOW, (uint8_t)(u16StackFan2SpdFeedBack & 0xFF));


    //本次制氢时间
    stHydrgProduceTimeThisTime = GetHydrgProduceTimeThisTime();
    g_PrgmTxMsgBuff[HYDROGEN_PRODUCT_TIME_THIS_TIME_HOUR_HIGH] = (u8)(stHydrgProduceTimeThisTime.hour >> 8);
    g_PrgmTxMsgBuff[HYDROGEN_PRODUCT_TIME_THIS_TIME_HOUR_LOW] = (u8)(stHydrgProduceTimeThisTime.hour & 0xFF);
    g_PrgmTxMsgBuff[HYDROGEN_PRODUCT_TIME_THIS_TIME_MINUTE] = stHydrgProduceTimeThisTime.minute;
    g_PrgmTxMsgBuff[HYDROGEN_PRODUCT_TIME_THIS_TIME_SECOND] = stHydrgProduceTimeThisTime.second;

    //累计制氢时间
    stHydrgProduceTimeTotal = GetHydrgProduceTimeThisTime();//暂以本次制氢时间代替
    g_PrgmTxMsgBuff[HYDROGEN_PRODUCT_TIME_TOTAL_HOUR_HIGH] = (u8)(stHydrgProduceTimeTotal.hour >> 8);
    g_PrgmTxMsgBuff[HYDROGEN_PRODUCT_TIME_TOTAL_HOUR_LOW] = (u8)(stHydrgProduceTimeTotal.hour & 0xFF);
    g_PrgmTxMsgBuff[HYDROGEN_PRODUCT_TIME_TOTAL_MINUTE] = stHydrgProduceTimeTotal.minute; //
    g_PrgmTxMsgBuff[HYDROGEN_PRODUCT_TIME_TOTAL_SECOND] = stHydrgProduceTimeTotal.second;

    //累计制氢次数
    u16HydrgWorkTimes = GetHydrgProducerWorkTimes();
    g_PrgmTxMsgBuff[HYDROGEN_PRODUCT_TOTAL_TIMES_HIGH] = (u8)(u16HydrgWorkTimes >> 8);
    g_PrgmTxMsgBuff[HYDROGEN_PRODUCT_TOTAL_TIMES_LOW] = (u8)(u16HydrgWorkTimes & 0xFF);

    //本次发电时间
    stStackWorkTimeThisTime = GetStackProductTimeThisTime();
    g_PrgmTxMsgBuff[STACK_WORK_TIME_THIS_TIME_HOUR_HIGH] = (u8)(stStackWorkTimeThisTime.hour >> 8);
    g_PrgmTxMsgBuff[STACK_WORK_TIME_THIS_TIME_HOUR_LOW] = (u8)(stStackWorkTimeThisTime.hour & 0xFF);
    g_PrgmTxMsgBuff[STACK_WORK_TIME_THIS_TIME_MINUTE] = stStackWorkTimeThisTime.minute;
    g_PrgmTxMsgBuff[STACK_WORK_TIME_THIS_TIME_SECOND] = stStackWorkTimeThisTime.second;

    //累计发电时间
    stStackWorkTimeTotal = GetStackProductTimeThisTime();//暂以本次发电时间代替
    g_PrgmTxMsgBuff[STACK_WORK_TIME_TOTAL_HOUR_HIGH] = (u8)(stStackWorkTimeTotal.hour >> 8);
    g_PrgmTxMsgBuff[STACK_WORK_TIME_TOTAL_HOUR_LOW] = (u8)(stStackWorkTimeTotal.hour & 0xFF);
    g_PrgmTxMsgBuff[STACK_WORK_TIME_TOTAL_MINUTE] = stStackWorkTimeTotal.minute;
    g_PrgmTxMsgBuff[STACK_WORK_TIME_TOTAL_SECOND] = stStackWorkTimeTotal.second;

    //当前输出功率
    fStackPower = GetCurrentPower();
    g_PrgmTxMsgBuff[CURRENT_ISOLATED_POWER_INTEGER_PART_HIGH] = (u8)(((uint16_t) fStackPower & 0xFF00) >> 8);
    g_PrgmTxMsgBuff[CURRENT_ISOLATED_POWER_INTEGER_PART_LOW] = (u8)((uint16_t) fStackPower & 0xFF);
    g_PrgmTxMsgBuff[CURRENT_ISOLATED_POWER_DECIMAL_PART] = (u8)((fStackPower - (uint16_t) fStackPower) * 100);

    //本次单机发电量
    dIsolatedGenratedEnergyThisTime = GetIsolatedGenratedEnergyThisTime();
    g_PrgmTxMsgBuff[ISOLATED_GENERATED_ENERGY_THIE_TIME_INTEGER_PART_HIGH] = (u8)(((uint16_t) dIsolatedGenratedEnergyThisTime) >> 8);
    g_PrgmTxMsgBuff[ISOLATED_GENERATED_ENERGY_THIE_TIME_INTEGER_PART_LOW] = (u8)(((uint16_t) dIsolatedGenratedEnergyThisTime) & 0xFF);
    g_PrgmTxMsgBuff[ISOLATED_GENERATED_ENERGY_THIE_TIME_DECIMAL_PART] = (u8)((dIsolatedGenratedEnergyThisTime - (uint16_t) dIsolatedGenratedEnergyThisTime) * 100);
}

/*
*********************************************************************************************************
*                                      SendPrgmMsg()
*
* Description:  The use of the funciton start the DMA send the message that sendout.
*
* Arguments  :  none
*
* Returns    :  none
*********************************************************************************************************
*/
static void SendPrgmMsg(void)
{
    BSP_PrgmDataDMASend(NULL, PRGM_TX_BUFF_SIZE);
}

/*
*********************************************************************************************************
*                                      GetPramTxBuffLen()
*
* Description:  get the length of the message that sendout.
*
* Arguments  :  none
*
* Returns    :  length
*********************************************************************************************************
*/
uint8_t GetPramTxBuffLen(void)
{
    return PRGM_TX_BUFF_SIZE;
}

/*
*********************************************************************************************************
*                                      GetPrgmRxBuffAddr()
*
* Description:  get the address of the message that receive in.
*
* Arguments  :  none
*
* Returns    :  address
*********************************************************************************************************
*/
uint8_t *GetPrgmRxBuffAddr(void)
{
    return g_PrgmRxMsgBuff;
}

/*
*********************************************************************************************************
*                                      GetPrgmRxBuffLen()
*
* Description:  get the length of the message that receive in.
*
* Arguments  :  none
*
* Returns    :  length
*********************************************************************************************************
*/
uint8_t GetPrgmRxBuffLen(void)
{
    return PRGM_RX_BUFF_SIZE;
}

/*
*********************************************************************************************************
*                                      AnwserPrgmCommand()
*
* Description:  anwser the command that receive from the control side.
*               应答上位机给出的控制命令
* Arguments  :  none
*
* Returns    :  length
*********************************************************************************************************
*/
static void AnwserPrgmCommand(void)
{
    OS_ERR      err;

    if((g_PrgmRxMsgBuff[0] == 0xFD) && (g_PrgmRxMsgBuff[1] == 0xFE))
    {
        switch(g_PrgmRxMsgBuff[2])
        {
            case 0x02:
                if(g_PrgmRxMsgBuff[4] <= 2)
                {
                    if(GetWorkModeWaittingForSelectFlag() == DEF_ENABLED)
                    {
                        SetWorkMode((SYSTEM_WORK_MODE_Typedef)g_PrgmRxMsgBuff[4]);
                        OSSemPost(&MannualSelcetWorkModeSem,        //结束手动控制模式下工作模式的选择等待
                                  OS_OPT_POST_1,
                                  &err);
                    }
                    else
                    {}
                }
                else
                {}

                break;

            case 0x03:
                ControlModeTurnOver();
                break;

            case 0x04:
                CmdStart();
                break;

            case 0x05:

                //提前启动命令
                if(GetSystemWorkStatu() == EN_START_PRGM_ONE_BEHIND)
                {
                    SetAheadRunningFlag(YES);
                    OSSemPost(&IgniteFirstBehindWaitSem,
                              OS_OPT_POST_1,
                              &err);
                }

                break;

            case 0x06:
                CmdShutDown();
                break;

            case 0x07:
                if(GetSystemWorkStatu() == EN_KEEPING_WARM)
                {
                    SetSystemWorkStatu(EN_WAITTING_COMMAND);
                }

                break;

            case 0x20:
                PumpSpdInc();
                break;

            case 0x21:
                PumpSpdDec();
                break;

            case 0x22:
                SetPumpCtlSpd(g_PrgmRxMsgBuff[4]);
                break;

            case 0x23:
                HydrgFanSpdInc();
                break;

            case 0x24:
                HydrgFanSpdDec();
                break;

            case 0x25:
                SetHydrgFanCtlSpd(g_PrgmRxMsgBuff[4]);
                break;

            case 0x30:
                IgniterWorkForSeconds(10);
                break;

            case 0x31:
                IgniterWorkForSeconds(0);
                break;

            case 0x50:
                StackFanSpdInc();
                break;

            case 0x51:
                StackFanSpdDec();
                break;

            case 0x52:
                SetStackFanCtlSpd(g_PrgmRxMsgBuff[4]);
                break;

            default:
                break;
        }
    }
}

/*
*********************************************************************************************************
*                                      UpdatePrgmTxMsgUintByte()
*
* Description:  update the message byte that the real type is unsigned char.
*
* Arguments  :  DataAddr - the offset address of the byte,
*               Value - the excepted value of the byte
*
* Returns    :  none
*********************************************************************************************************
*/
void UpdatePrgmTxMsgUintByte(PRGM_TX_MSG_DATA_ADDR_Typedef DataAddr, uint8_t Value)
{
    g_PrgmTxMsgBuff[DataAddr] = Value;
}

/*
*********************************************************************************************************
*                                      UpdatePrgmTxMsgIntByte()
*
* Description:  update the message byte that the real type is signed char.
*
* Arguments  :  DataAddr - the offset address of the byte,
*               Value - the excepted value of the byte
*
* Returns    :  none
*********************************************************************************************************
*/
void UpdatePrgmTxMsgIntByte(PRGM_TX_MSG_DATA_ADDR_Typedef DataAddr, int8_t Value)
{
    g_PrgmTxMsgBuff[DataAddr] = Value;
}

/*
*********************************************************************************************************
*                                      CmdStart()
*
* Description:  execute the "start" command accord to the statu of the system.
*
* Arguments  :  none
*
* Returns    :  none
*********************************************************************************************************
*/
void CmdStart(void)
{
    OS_ERR      err;
    SYSTEM_WORK_STATU_Typedef eSysRunningStatu;
    SYSTEM_WORK_MODE_Typedef    eWorkMode;

    eSysRunningStatu = GetSystemWorkStatu();

    if(DEF_OFF == GetHydrgProducerStopDlyStatu() && (DEF_OFF == GetStackStopDlyStatu()))
    {
        if(eSysRunningStatu == EN_WAITTING_COMMAND)//等待指令，循环设备自检中
        {
            eWorkMode = GetWorkMode();

            if(eWorkMode != EN_WORK_MODE_FUEL_CELL)
            {
                OSSchedLock(&err);
                SetSystemWorkStatu(EN_START_PRGM_ONE_FRONT);
            }
            else
            {
                OSSchedLock(&err);
                SetSystemWorkStatu(EN_RUNNING);
            }

            OSTaskSemPost(&AppTaskStartTCB,
                          OS_OPT_POST_NO_SCHED,
                          &err);
            OSSchedUnlock(&err);
        }
        else    if(eSysRunningStatu == EN_ALARMING)
        {
            SetSystemWorkStatu(EN_WAITTING_COMMAND);
            ResetDeviceAlarmStatu();
        }
    }
    else
    {
        APP_TRACE_INFO(("The last run cycle is not finish, please wait...\n\r"));
    }
}

/*
*********************************************************************************************************
*                                      CmdShutDown()
*
* Description:  execute the "shut down" command accord to the statu of the system.
*               根据系统当前工作状态执行关机命令
* Arguments  :  none
*
* Returns    :  none
*********************************************************************************************************
*/
void CmdShutDown(void)
{
    OS_ERR      err;
    SYSTEM_WORK_STATU_Typedef eSysRunningStatu;

    eSysRunningStatu = GetSystemWorkStatu();

    if((eSysRunningStatu == EN_START_PRGM_ONE_FRONT)
    || (eSysRunningStatu == EN_START_PRGM_ONE_BEHIND)
    || (eSysRunningStatu == EN_START_PRGM_TWO)
    || (eSysRunningStatu == EN_RUNNING))
    {
        OSSchedLock(&err);      //锁任务调度器
        SetSystemWorkStatu(EN_SHUTTING_DOWN);

        switch(eSysRunningStatu)    //因switch不支持枚举型变量，故将其转为uint8_t型
        {
            case EN_START_PRGM_ONE_FRONT:

                break;

            case EN_START_PRGM_ONE_BEHIND:
                OSSemPost(&IgniteFirstBehindWaitSem,
                          OS_OPT_POST_1,
                          &err);
                break;

            case EN_START_PRGM_TWO:
                break;

            case EN_RUNNING:
                OSTaskResume(&AppTaskStartTCB,      //恢复开始任务
                             &err);
                break;

            default:
                break;
        }

        OSSchedUnlock(&err);
    }

}
/******************* (C) COPYRIGHT 2015 Guangdong Hydrogen *****END OF FILE****/

