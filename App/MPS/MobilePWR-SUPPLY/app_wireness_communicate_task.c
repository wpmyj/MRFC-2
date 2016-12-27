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
* Filename      : app_wireness_communicate_task.c
* Version       : V1.00
* Programmer(s) : Fanjun
***************************************************************************************************
*/

/*
***************************************************************************************************
*                                             INCLUDE FILES
***************************************************************************************************
*/

#include <includes.h>
#include "app_system_real_time_parameters.h"
#include "app_wireness_communicate_task.h"
#include "app_top_task.h"
#include "bsp_speed_adjust_device.h"
#include "app_hydrg_producer_manager.h"
#include "app_stack_manager.h"
#include "app_system_run_cfg_parameters.h"

/*
***************************************************************************************************
*                                       MACRO DEFINITIONS
***************************************************************************************************
*/
#define COMMUNICATE_TASK_STK_SIZE       100
#define COMMUNICATA_DATA_SEND_TASK_STK_SIZE     128
#define COMMUNICATE_REQUEST_SEND_TASK_STK_SIZE 100


#define RUNNING_CONFIG_INTERFACE    0       //Running config interface reserved
/*
***************************************************************************************************
*                                         OS-RELATED    VARIABLES
***************************************************************************************************
*/
OS_FLAG_GRP     ConfigParametersChangeState;
OS_SEM          g_stCommunicateDataSendSem;
OS_SEM          g_stCommunicateDataSendResponseSem;

OS_TCB          CommunicateTaskTCB;
OS_TCB          CommunicateDataSendTaskTCB;
OS_TCB          CommunicateRequsetInfSendTaskTCB;

OS_MUTEX        TxMsgSendBuffWriteMutex;

__attribute__((aligned(8)))  //任务堆栈8字节对齐，确保浮点数显示正常
static      CPU_STK     CommunicateTaskStk[COMMUNICATE_TASK_STK_SIZE];
static      CPU_STK     CommunicateDataSendTaskStk[COMMUNICATA_DATA_SEND_TASK_STK_SIZE];
static      CPU_STK     CommunicateRequsetSendTaskStk[COMMUNICATE_REQUEST_SEND_TASK_STK_SIZE];
/*
***************************************************************************************************
*                                           LOCAL VARIABLES
***************************************************************************************************
*/
uint8_t             g_u8SerRxMsgBuff[PRGM_RX_BUFF_SIZE];
//static    uint32_t    g_u32TxMsgDataTagNumber[EN_SEND_DATA_TYPE_MAX] = {0};   //数据身份标签码

static      TX_MSG_SEND_BUFF_Typedef        g_stTxMsgDataSendBuff = {0};

/*
***************************************************************************************************
*                                         FUNCTION PROTOTYPES
***************************************************************************************************
*/
//          void        SetWaitShutDownRequestResponseFlag(SWITCH_TYPE_VARIABLE_Typedef i_eNewStatu);

static      void        CommunicateTask(void *p_arg);
static      void        CommunicateDataSendTask(void *p_arg);
static      void        CommunicateRequsetInfSendTask(void *p_arg);
static      void        ResponsePrgmCommand(uint8_t *);

static      void        LoadNonRealTimeWorkInfo(uint8_t , uint8_t , uint8_t , uint8_t *p_uint8_tParas, uint8_t *);
static      void        AddRealTimeWorkInfoDataToSendBuff(uint8_t , uint8_t);
static      void        LoadHydrogenProducerRealTimeWorkInfo(uint8_t, uint8_t *);
static      void        LoadFuelCellRealTimeWorkInfoPartA(uint8_t, uint8_t *);
//static      void        SendReferenceAndConfigrationInfo(void);
static      void        InsertNonRealTimeWorkInfoDataToSendBuff(uint8_t, uint8_t , uint8_t , uint8_t *);
static      void        SendAPrgmMsgFrame(uint8_t i_uint8_tTxMsgLen, uint8_t *i_pTxMsg);


/*
***************************************************************************************************
*                                          CommunicateTaskCreate()
*
* Description : The use of the the funciton is to create the task that manager the communication.
*
* Arguments   : none.
*
* Returns     : none
*
* Notes       : none
***************************************************************************************************
*/
void  CommunicateTaskCreate(void)
{
    OS_ERR  err;

    BSP_SerToWIFI_Init();       //(WIFI)串口初始化

    BSP_SerToWIFI_RxMsgInit(g_u8SerRxMsgBuff, PRGM_RX_BUFF_SIZE);   //接收数据地址初始化

    OSTaskCreate((OS_TCB *)&CommunicateTaskTCB,
                 (CPU_CHAR *)"Communicate Task Start",
                 (OS_TASK_PTR) CommunicateTask,
                 (void *) 0,
                 (OS_PRIO) COMMUNICATE_TASK_PRIO,
                 (CPU_STK *)&CommunicateTaskStk[0],
                 (CPU_STK_SIZE) COMMUNICATE_TASK_STK_SIZE / 10,
                 (CPU_STK_SIZE) COMMUNICATE_TASK_STK_SIZE,
                 (OS_MSG_QTY) 5u,
                 (OS_TICK) 0u,
                 (void *) 0,
                 (OS_OPT)(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR *)&err);
    APP_TRACE_INFO(("Created communicate Task, and err code is %d...\r\n", err));

    OSSemCreate(&g_stCommunicateDataSendSem,
                "Communicate data send sem",
                0,
                &err);
    OSMutexCreate(&TxMsgSendBuffWriteMutex,
                  "Tx msg send buff write mutex",
                  &err);

    OSTaskCreate((OS_TCB *)&CommunicateDataSendTaskTCB,
                 (CPU_CHAR *)"Communicate Data Send Task Start",
                 (OS_TASK_PTR) CommunicateDataSendTask,
                 (void *) 0,
                 (OS_PRIO) COMMUNICATE_DATA_SEND_TASK_PRIO,
                 (CPU_STK *)&CommunicateDataSendTaskStk[0],
                 (CPU_STK_SIZE) COMMUNICATA_DATA_SEND_TASK_STK_SIZE / 10,
                 (CPU_STK_SIZE) COMMUNICATA_DATA_SEND_TASK_STK_SIZE,
                 (OS_MSG_QTY) 5u,
                 (OS_TICK) 0u,
                 (void *) 0,
                 (OS_OPT)(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR *)&err);
    APP_TRACE_INFO(("Created communicate Data Send Task, and err code is %d...\r\n", err));

    OSTaskCreate((OS_TCB *)&CommunicateRequsetInfSendTaskTCB,
                 (CPU_CHAR *)"Requset information send Task Start",
                 (OS_TASK_PTR) CommunicateRequsetInfSendTask,
                 (void *) 0,
                 (OS_PRIO) COMMUNICATE_REQUEST_SEND_TASK_PRIO,
                 (CPU_STK *)&CommunicateRequsetSendTaskStk[0],
                 (CPU_STK_SIZE) COMMUNICATE_REQUEST_SEND_TASK_STK_SIZE / 10,
                 (CPU_STK_SIZE) COMMUNICATE_REQUEST_SEND_TASK_STK_SIZE,
                 (OS_MSG_QTY) 5u,
                 (OS_TICK) 0u,
                 (void *) 0,
                 (OS_OPT)(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR *)&err);
    APP_TRACE_INFO(("Created real time requset information send Task, and err code is %d...\r\n", err));
}

/*
***************************************************************************************************
*                               CommunicateTask()
*
* Description :This is a task that manage the communication with other device.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Note(s)     : none.
***************************************************************************************************
*/
void  CommunicateTask(void *p_arg)
{
    OS_ERR      err;
    SYSTEM_WORK_MODE_Typedef    eWorkMode;

    while(DEF_TRUE) {
        OSTimeDlyHMSM(0, 0, 1, 000,
                      OS_OPT_TIME_HMSM_STRICT,
                      &err);

        if(g_eWirenessCommandReceived == YES) { //收到串口指令而提前结束延时，响应上位机指令
            ResponsePrgmCommand(g_u8SerRxMsgBuff);
            g_eWirenessCommandReceived = NO;
        } else {
        }//正常延时

        if(DEF_DISABLED == GetWorkModeWaittingForSelectFlag()) {
            eWorkMode = GetWorkMode();      //根据工作模式选择性发送实时信息

            switch((u8)eWorkMode) {
                case EN_WORK_MODE_HYDROGEN_PRODUCER:
                    AddRealTimeWorkInfoDataToSendBuff(EN_LATEST, REAL_TIME_RUNNING_INFORMATION_A);  //在发送数据缓冲区尾增加一帧最新数据
                    break;

                case EN_WORK_MODE_FUEL_CELL:
                    AddRealTimeWorkInfoDataToSendBuff(EN_LATEST, REAL_TIME_RUNNING_INFORMATION_B);
                    break;

                case EN_WORK_MODE_HYDROGEN_PRODUCER_AND_FUEL_CELL:
                    AddRealTimeWorkInfoDataToSendBuff(EN_LATEST, REAL_TIME_RUNNING_INFORMATION_A);
                    AddRealTimeWorkInfoDataToSendBuff(EN_LATEST, REAL_TIME_RUNNING_INFORMATION_B);
                    break;

                default:
                    break;
            }
        }

#if RUNNING_CONFIG_INTERFACE

        if(ON == GetRunningParaCfgSwitchStatus()) {
            OSFlagPend(&ConfigParametersChangeState,
                       CFG_IGNITE_FIRST_STEP_PUMP_SPD + CFG_IGNITE_FIRST_STEP_FAN_SPD + CFG_IGNITE_FIRST_SUCCESSED_FAN_SPD + \
                       CFG_IGNITE_SECOND_STEP_PUMP_SPD + CFG_IGNITE_SECOND_STEP_FAN_SPD + CFG_IGNITE_SECOND_SUCCESSED_FAN_SPD + \
                       CFG_LIQUID_PRESS_EXCEED_4KG_PUMP_SPD,
                       0,
                       OS_OPT_PEND_FLAG_SET_ALL + OS_OPT_PEND_NON_BLOCKING,
                       (CPU_TS *)0,  //时间戳
                       &err);

            if(OS_ERR_NONE == err) {
                /*所有配置参数修改完成后发送确认参数给上位机*/
                SendReferenceAndConfigrationInfo();
                OSFlagDel(&ConfigParametersChangeState,
                          OS_OPT_DEL_ALWAYS,
                          &err);
                SetRunningParaCfgSwitch(OFF);  //配置完成关闭
            }
        }

#endif
        //发送信号量，启动一次数据传输
        OSSemPost(&g_stCommunicateDataSendSem,
                  OS_OPT_POST_1,
                  &err);
    }
}
/*
***************************************************************************************************
*                               CommunicateRequsetInfSendTask()
*
* Description :This is a task that manage the communication with other device.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Note(s)     : none.
***************************************************************************************************
*/
void  CommunicateRequsetInfSendTask(void *p_arg)
{
    OS_ERR      err;

    while(DEF_TRUE) {
        OSTaskSuspend(NULL,
                      &err);

        while(DEF_TRUE) {
            OSTimeDlyHMSM(0, 0, 0, 200,
                          OS_OPT_TIME_HMSM_STRICT,
                          &err);

            if(DEF_ENABLED == GetWorkModeWaittingForSelectFlag()) { //发送选择工作模式请求，直到上位机选定工作模式
                SendChooseWorkModeRequest();
                //启动一次数据传输
                OSSemPost(&g_stCommunicateDataSendSem,
                          OS_OPT_POST_1,
                          &err);
            } else { //停止发送请求
                SendRealTimeAssistInfo();   //向上位机发送一次自检信息,防止上位机没收到主任务中第一次发送的信息。
                break;
            }
        }
    }
}

/*
***************************************************************************************************
*                               AddRealTimeWorkInfoDataToSendBuff()
*
* Description :数据发送任务
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Note(s)     : none.
***************************************************************************************************
*/
void CommunicateDataSendTask(void *p_arg)
{
    OS_ERR      err;

    OSSemCreate(&g_stCommunicateDataSendResponseSem,
                "Communicate data send response sem",
                0,
                &err);

    while(DEF_TRUE) {
        OSSemPend(&g_stCommunicateDataSendSem,
                  OS_CFG_TICK_RATE_HZ,
                  OS_OPT_PEND_BLOCKING,
                  NULL,
                  &err);

        while(g_stTxMsgDataSendBuff.Q_length != 0) {
            g_stTxMsgDataSendBuff.Queue[g_stTxMsgDataSendBuff.Q_Qhead].OccupyStatu = EN_SENDING;
            //一帧一帧的发送数据,暂时不能灵活改变帧的长度，接收方的长度是固定的，如要变则需要在帧数据包中加上一个帧类型
            SendAPrgmMsgFrame(PRGM_TX_BUFF_SIZE, g_stTxMsgDataSendBuff.Queue[g_stTxMsgDataSendBuff.Q_Qhead].Data);

            OSSemPend(&g_stCommunicateDataSendResponseSem,
                      OS_CFG_TICK_RATE_HZ / 5,//五分之一秒
                      OS_OPT_PEND_BLOCKING,
                      NULL,
                      &err);

            if(err == OS_ERR_NONE) { //正常发送
//              APP_TRACE_INFO(("Send statu normal...\r\n"));
            } else { //存储到历史记录存储区中
//              APP_TRACE_INFO(("Send statu err...\r\n"));
            }

            OSMutexPend(&TxMsgSendBuffWriteMutex,
                        OS_CFG_TICK_RATE_HZ / 10,
                        OS_OPT_PEND_BLOCKING,
                        NULL,
                        &err);
            g_stTxMsgDataSendBuff.Q_length--;
            g_stTxMsgDataSendBuff.Queue[g_stTxMsgDataSendBuff.Q_Qhead].OccupyStatu = EN_FREE;
            //原来的第二节点成为新的头节点
            g_stTxMsgDataSendBuff.Q_Qhead = g_stTxMsgDataSendBuff.Queue[g_stTxMsgDataSendBuff.Q_Qhead].D_next;
            //原来的头节点后向指针指向自己—即新的头节点的前一节点
            g_stTxMsgDataSendBuff.Queue[g_stTxMsgDataSendBuff.Queue[g_stTxMsgDataSendBuff.Q_Qhead].D_last].D_next = g_stTxMsgDataSendBuff.Queue[g_stTxMsgDataSendBuff.Q_Qhead].D_last;
            g_stTxMsgDataSendBuff.Queue[g_stTxMsgDataSendBuff.Q_Qhead].D_last = g_stTxMsgDataSendBuff.Q_Qhead;//新头节点的前向标记指向自己
            OSMutexPost(&TxMsgSendBuffWriteMutex,
                        OS_OPT_POST_NONE,
                        &err);
        }
    }
}

/*
***************************************************************************************************
*                               AddRealTimeWorkInfoDataToSendBuff()
*
* Description :追加实时运行数据至发送缓冲区
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Note(s)     : none.
***************************************************************************************************
*/
void AddRealTimeWorkInfoDataToSendBuff(uint8_t i_uint8_tIsLatestData, uint8_t i_uint8_RealTimeWorkInfoType)
{
    OS_ERR      err;
    uint8_t i;

    OSMutexPend(&TxMsgSendBuffWriteMutex,
                OS_CFG_TICK_RATE_HZ / 10,
                OS_OPT_PEND_BLOCKING,
                NULL,
                &err);

    if(g_stTxMsgDataSendBuff.Q_length == 0) { //队列中没有信息待发送
        g_stTxMsgDataSendBuff.Q_length++;
        g_stTxMsgDataSendBuff.Q_Qhead = 0;
        g_stTxMsgDataSendBuff.Q_Qrear = 0;
        g_stTxMsgDataSendBuff.Queue[g_stTxMsgDataSendBuff.Q_Qrear].D_last = g_stTxMsgDataSendBuff.Q_Qhead;
        g_stTxMsgDataSendBuff.Queue[g_stTxMsgDataSendBuff.Q_Qrear].D_next = g_stTxMsgDataSendBuff.Q_Qrear;
        g_stTxMsgDataSendBuff.Queue[g_stTxMsgDataSendBuff.Q_Qrear].OccupyStatu = EN_WAITTING;

        if(REAL_TIME_RUNNING_INFORMATION_A == i_uint8_RealTimeWorkInfoType) {
            LoadHydrogenProducerRealTimeWorkInfo(i_uint8_tIsLatestData, g_stTxMsgDataSendBuff.Queue[g_stTxMsgDataSendBuff.Q_Qrear].Data);
        } else if(REAL_TIME_RUNNING_INFORMATION_B == i_uint8_RealTimeWorkInfoType) {
            LoadFuelCellRealTimeWorkInfoPartA(i_uint8_tIsLatestData, g_stTxMsgDataSendBuff.Queue[g_stTxMsgDataSendBuff.Q_Qrear].Data);
        } else {}

    } else if(++g_stTxMsgDataSendBuff.Q_length <= TX_MSG_SEND_QUEUE_SIZE) {
        //从缓冲区头搜寻空闲节点
        for(i = 0; i < TX_MSG_SEND_QUEUE_SIZE; i++) {
            if(g_stTxMsgDataSendBuff.Queue[i].OccupyStatu == EN_FREE) {
                g_stTxMsgDataSendBuff.Queue[g_stTxMsgDataSendBuff.Q_Qrear].D_next = i;//先前的尾节点后向标记指向搜寻到的空节点
                g_stTxMsgDataSendBuff.Queue[i].D_last = g_stTxMsgDataSendBuff.Q_Qrear;//开辟的新节点的前向标记指向原来的尾节点
                g_stTxMsgDataSendBuff.Q_Qrear = i;//尾节点更新到新的节点
                g_stTxMsgDataSendBuff.Queue[i].OccupyStatu = EN_WAITTING;//新节点的状态更新为等待发送

                if(REAL_TIME_RUNNING_INFORMATION_A == i_uint8_RealTimeWorkInfoType) {
                    LoadHydrogenProducerRealTimeWorkInfo(i_uint8_tIsLatestData, g_stTxMsgDataSendBuff.Queue[g_stTxMsgDataSendBuff.Q_Qrear].Data);
                } else if(REAL_TIME_RUNNING_INFORMATION_B == i_uint8_RealTimeWorkInfoType) {
                    LoadFuelCellRealTimeWorkInfoPartA(i_uint8_tIsLatestData, g_stTxMsgDataSendBuff.Queue[g_stTxMsgDataSendBuff.Q_Qrear].Data);
                } else {}

                break;
            }
        }
    } else {
        --g_stTxMsgDataSendBuff.Q_length;
        APP_TRACE_INFO(("The Sends message buffer is overflow...\r\n"));
    }

    OSMutexPost(&TxMsgSendBuffWriteMutex,       //释放写发送缓冲区互斥锁
                OS_OPT_POST_NONE,
                &err);
}

/*
***************************************************************************************************
*                                      LoadHydrogenProducerRealTimeWorkInfo()
*
* Description:  载入制氢机实时运行数据,或当前的,或提取自历史记录.
*
* Arguments  :  none
*
* Returns    :  none
***************************************************************************************************
*/
static void LoadHydrogenProducerRealTimeWorkInfo(uint8_t i_uint8_tIsHistoryData, uint8_t *i_pRealTimeWorkInfo)
{
    uint16_t    u16CurrentTemp = 0;
    uint16_t    u16HydrgWorkTimes = 0;
//    uint16_t    u16HydrogenGasConcentration = 0;
    uint16_t    u16PumpFeedbackSpeed = 0, u16PumpCtlSpeed = 0;
    uint16_t    u16HydrgFanCtlSpd = 0, u16HydrgFeedbackFanSpd = 0;
    uint32_t    u16FluidWeightPerMinuteMul100 = 0;
    uint16_t    m_u16LiquidPress = 0;
    uint32_t    u32AlarmCode = 0;
    uint32_t    u32SystemRunningStatuCode = 0;
    SYSTEM_TIME_Typedef     stHydrgProduceTimeThisTime = {0}, stHydrgProduceTimeTotal = {0};

    if(i_uint8_tIsHistoryData == EN_LATEST) { //最新的数据
//      APP_TRACE_INFO(("Load ENECO Producer real time work info...\r\n"));
        //数据报头段
        *(i_pRealTimeWorkInfo + HEAD_BYTE_ONE) = 0xF1;
        *(i_pRealTimeWorkInfo + HEAD_BYTE_TWO) = 0xF2;
        *(i_pRealTimeWorkInfo + HEAD_BYTE_THREE) = 0xF3;
        *(i_pRealTimeWorkInfo + PRODUCTS_TYPE_ID_HIGH) = (uint8_t)((PRODUCT_MODEL_CODE & 0xFF00) >>  8);
        *(i_pRealTimeWorkInfo + PRODUCTS_TYPE_ID_LOW) = (uint8_t)(PRODUCT_MODEL_CODE & 0xFF);
        *(i_pRealTimeWorkInfo + LOCAL_NETWORK_ID_CODE) = LOCAL_NETWORK_ID;
        *(i_pRealTimeWorkInfo + INFORMATION_TYPE_CONTROL_CODE) = (uint8_t)REAL_TIME_RUNNING_INFORMATION_A;

//      //数据标签码码值增加
//      g_u32TxMsgDataTagNumber[REAL_TIME_RUNNING_INFORMATION]++;
//      *(i_pRealTimeWorkInfo + DATA_IDENTIFY_TAG_INF_CODE_1) = (uint8_t)(g_u32TxMsgDataTagNumber[REAL_TIME_RUNNING_INFORMATION] >> 24);
//      *(i_pRealTimeWorkInfo + DATA_IDENTIFY_TAG_INF_CODE_2) = (uint8_t)((g_u32TxMsgDataTagNumber[REAL_TIME_RUNNING_INFORMATION] & 0xFF0000) >> 16);
//      *(i_pRealTimeWorkInfo + DATA_IDENTIFY_TAG_INF_CODE_3) = (uint8_t)((g_u32TxMsgDataTagNumber[REAL_TIME_RUNNING_INFORMATION] & 0xFF00) >> 8);
//      *(i_pRealTimeWorkInfo + DATA_IDENTIFY_TAG_INF_CODE_4) = (uint8_t)(g_u32TxMsgDataTagNumber[REAL_TIME_RUNNING_INFORMATION] & 0xFF);
        *(i_pRealTimeWorkInfo + VALID_INFORMATION_LENGTH_CONTROL_CODE) = REAL_TIME_RUNNING_INFO_A_LENGTH;
        //运行警报码
        u32AlarmCode = GetRunAlarmCode();
        *(i_pRealTimeWorkInfo + RUN_ALARM_CODE_BYTE_4) = (uint8_t)(u32AlarmCode >> 24);
        *(i_pRealTimeWorkInfo + RUN_ALARM_CODE_BYTE_3) = (uint8_t)((u32AlarmCode & 0xFF0000) >> 16);
        *(i_pRealTimeWorkInfo + RUN_ALARM_CODE_BYTE_2) = (uint8_t)((u32AlarmCode & 0xFF00) >> 8);
        *(i_pRealTimeWorkInfo + RUN_ALARM_CODE_BYTE_1) = (uint8_t)(u32AlarmCode & 0xFF);
        //运行状态码
        u32SystemRunningStatuCode = GetSystemRunningStatuCode();
        *(i_pRealTimeWorkInfo + RUNNING_STATU_CODE_BYTE_4) = (uint8_t)(u32SystemRunningStatuCode >> 24);
        *(i_pRealTimeWorkInfo + RUNNING_STATU_CODE_BYTE_3) = (uint8_t)((u32SystemRunningStatuCode & 0xFF0000) >> 16);
        *(i_pRealTimeWorkInfo + RUNNING_STATU_CODE_BYTE_2) = (uint8_t)((u32SystemRunningStatuCode & 0xFF00) >> 8);
        *(i_pRealTimeWorkInfo + RUNNING_STATU_CODE_BYTE_1) = (uint8_t)(u32SystemRunningStatuCode & 0xFF);
        //重整温度
        u16CurrentTemp = (uint16_t)GetReformerTemp();
        *(i_pRealTimeWorkInfo + REFORMER_TEMP_HIGH) = (uint8_t)((u16CurrentTemp & 0xFF00) >> 8);
        *(i_pRealTimeWorkInfo + REFORMER_TEMP_LOW) = (uint8_t)((u16CurrentTemp & 0xFF));
        //火焰温度
        u16CurrentTemp = (uint16_t)GetFireOrRodTemp();
        *(i_pRealTimeWorkInfo + FIRE_OR_ROD_TEMP_HIGH) = (uint8_t)((u16CurrentTemp & 0xFF00) >> 8);
        *(i_pRealTimeWorkInfo + FIRE_OR_ROD_TEMP_LOW) = (uint8_t)((u16CurrentTemp & 0xFF));
        //液压
        m_u16LiquidPress = (uint16_t)(GetSrcAnaSig(LIQUID_PRESS) * 100);
        *(i_pRealTimeWorkInfo + LIQUID_PRESS_INTEGER_PART) = (uint8_t)(m_u16LiquidPress / 100);
        *(i_pRealTimeWorkInfo + LIQUID_PRESS_DECIMAL_PART) = (uint8_t)(m_u16LiquidPress % 100);
        //风机控制速度
        u16HydrgFanCtlSpd = GetHydrgFanCurrentCtlSpd();
        *(i_pRealTimeWorkInfo + HYDROGEN_FAN_SPD_CONTROL_HIGH) = (uint8_t)((u16HydrgFanCtlSpd & 0xFF00) >> 8);
        *(i_pRealTimeWorkInfo + HYDROGEN_FAN_SPD_CONTROL_LOW) = (uint8_t)((u16HydrgFanCtlSpd & 0xFF));
        //风机反馈速度
        u16HydrgFeedbackFanSpd = GetHydrgFanFeedBackSpd();
        *(i_pRealTimeWorkInfo + HYDROGEN_FAN_SPD_FEEDBACK_HIGH) = (uint8_t)((u16HydrgFeedbackFanSpd & 0xFF00) >> 8);
        *(i_pRealTimeWorkInfo + HYDROGEN_FAN_SPD_FEEDBACK_LOW) = (uint8_t)((u16HydrgFeedbackFanSpd & 0xFF));
        //水泵控制速度
        u16PumpCtlSpeed = GetPumpCtlSpd();
        *(i_pRealTimeWorkInfo + PUMP_SPD_CONTROL_HIGH) = (uint8_t)((u16PumpCtlSpeed & 0xFF00) >> 8);
        *(i_pRealTimeWorkInfo + PUMP_SPD_CONTROL_LOW) = (uint8_t)(u16PumpCtlSpeed & 0xFF);
        //水泵反馈速度
        u16PumpFeedbackSpeed = GetPumpFeedBackSpd();
        *(i_pRealTimeWorkInfo + PUMP_SPD_FEEDBACK_HIGH) = (uint8_t)((u16PumpFeedbackSpeed & 0xFF00) >> 8);
        *(i_pRealTimeWorkInfo + PUMP_SPD_FEEDBACK_LOW) = (uint8_t)(u16PumpFeedbackSpeed & 0xFF);
        //本次制氢时间,APP上主界面显示的是这个的值
        stHydrgProduceTimeThisTime = GetHydrgProduceTimeThisTime();
        //  APP_TRACE_INFO(("AAAAA:%d%d\r\nMinut:%d\r\nSecond:%d\r\n",(uint8_t)((stHydrgProduceTimeThisTime.hour & 0xFF00) >> 8),(uint8_t)((stHydrgProduceTimeThisTime.hour & 0xFF)), stHydrgProduceTimeThisTime.minute,stHydrgProduceTimeThisTime.second));
        *(i_pRealTimeWorkInfo + HYDROGEN_PRODUCT_TIME_THIS_TIME_HOUR_HIGH) = (uint8_t)((stHydrgProduceTimeThisTime.hour & 0xFF00) >> 8);
        *(i_pRealTimeWorkInfo + HYDROGEN_PRODUCT_TIME_THIS_TIME_HOUR_LOW) = (uint8_t)(stHydrgProduceTimeThisTime.hour & 0xFF);
        *(i_pRealTimeWorkInfo + HYDROGEN_PRODUCT_TIME_THIS_TIME_MINUTE) = stHydrgProduceTimeThisTime.minute;
        *(i_pRealTimeWorkInfo + HYDROGEN_PRODUCT_TIME_THIS_TIME_SECOND) = stHydrgProduceTimeThisTime.second;
        //累计制氢时间
        stHydrgProduceTimeTotal = GetHydrgProduceTimeThisTime();//暂以本次制氢时间代替
        *(i_pRealTimeWorkInfo + HYDROGEN_PRODUCT_TIME_TOTAL_HOUR_HIGH) = (uint8_t)((stHydrgProduceTimeTotal.hour & 0xFF00) >> 8);
        *(i_pRealTimeWorkInfo + HYDROGEN_PRODUCT_TIME_TOTAL_HOUR_LOW) = (uint8_t)(stHydrgProduceTimeTotal.hour & 0xFF);
        *(i_pRealTimeWorkInfo + HYDROGEN_PRODUCT_TIME_TOTAL_MINUTE) = stHydrgProduceTimeTotal.minute;
        *(i_pRealTimeWorkInfo + HYDROGEN_PRODUCT_TIME_TOTAL_SECOND) = stHydrgProduceTimeTotal.second;
        //累计制氢次数
        u16HydrgWorkTimes = GetHydrgProducerWorkTimes();
        *(i_pRealTimeWorkInfo + HYDROGEN_PRODUCT_TOTAL_TIMES_HIGH) = (uint8_t)((u16HydrgWorkTimes & 0xFF00) >> 8);
        *(i_pRealTimeWorkInfo + HYDROGEN_PRODUCT_TOTAL_TIMES_LOW) = (uint8_t)(u16HydrgWorkTimes & 0xFF);
        //液位数据
//      *(i_pRealTimeWorkInfo + LIQUID_LEVEL_INTEGER_PART) = (uint8_t)(u16HydrgWorkTimes >> 8);
//      *(i_pRealTimeWorkInfo + LIQUID_LEVEL_DECIMAL_PART) =(uint8_t)(u16HydrgWorkTimes & 0xFF);
        //每分钟进液量
//        u16FluidWeightPerMinuteMul100 = (uint16_t)(GetHydrgProducerFluidWeightPerMinute() * 10000);
//      APP_TRACE_DEBUG(("FluidWeight:%f\r\n",fFluidWeightPerMinuteMul100));
//        u16FluidWeightPerMinuteMul100 = 56326;
        *(i_pRealTimeWorkInfo + FUEL_WEIGHT_INTEGER_PART_HIGH) = (uint8_t)(((u16FluidWeightPerMinuteMul100 / 100) & 0xFF0000) >> 16);
        *(i_pRealTimeWorkInfo + FUEL_WEIGHT_INTEGER_PART_MID) = (uint8_t)((u16FluidWeightPerMinuteMul100 / 100 & 0xFF00) >> 8);
        *(i_pRealTimeWorkInfo + FUEL_WEIGHT_INTEGER_PART_LOW) = (uint8_t)((u16FluidWeightPerMinuteMul100 / 100 & 0xFF));
        *(i_pRealTimeWorkInfo + FUEL_WEIGHT_DECIMAL_PART) = (uint8_t)(u16FluidWeightPerMinuteMul100 % 100);
        
//        //氢气浓度
//        u16HydrogenGasConcentration = (uint16_t)(GetSrcAnaSig(HYDROGEN_CONCENTRATION) * 100);
//        *(i_pRealTimeWorkInfo + HYDROGEN_PRODUCT_GAS_CONCENTRATION_INTEGER_PART) = (uint8_t)(u16HydrogenGasConcentration / 100);
//        *(i_pRealTimeWorkInfo + HYDROGEN_PRODUCT_GAS_CONCENTRATION_DECIMAL_PART) = (uint8_t)(u16HydrogenGasConcentration % 100);

        //子模块ID号
//        *(i_pRealTimeWorkInfo + SUB_MODULE_ID_OF_THE_MULTI_MODULE_TYPE) = SUB_MODULE_ID;
        //数据报尾段
//        *(i_pRealTimeWorkInfo + END_BYTE_ONE) = 0x5F;
//        *(i_pRealTimeWorkInfo + END_BYTE_TWO) = 0x6F;

    } else { //载入历史数据

    }
}
/*
***************************************************************************************************
*                                      LoadFuelCellRealTimeWorkInfoPartA()
*
* Description:  Load fuel cell real time work Info.
*
* Arguments  :  none
*
* Returns    :  none
***************************************************************************************************
*/
static void LoadFuelCellRealTimeWorkInfoPartA(uint8_t i_uint8_tIsHistoryData, uint8_t *i_pRealTimeWorkInfo)
{
    uint16_t  u16StackTemp = 0;
    uint16_t  u16StackFanSpdFeedBack = 0;
    uint16_t  u16StackFanCtlSpd = 0;
    uint32_t  u32StackPower = 0;
    uint32_t  u32AlarmCode = 0;
    uint16_t  m_u16StackCurrentMul100 = 0;
    uint16_t  m_u16StackVoltageMul100 = 0;
    uint16_t  m_u16StackPressMul100 = 0;
    uint32_t  u32IsolatedGenratedEnergyThisTime = 0;
    int16_t   i16HydrogYieldMatchOffsetValue = 0;
    uint32_t  u32SystemWorkStatuCode = 0;
    SYSTEM_TIME_Typedef     stStackWorkTimeThisTime = {0}, stStackWorkTimeTotal = {0};

    if(i_uint8_tIsHistoryData == EN_LATEST) { //最新的数据
//      APP_TRACE_INFO(("Load Fuel cell real time work info...\r\n"));
        //数据报头段
        *(i_pRealTimeWorkInfo + HEAD_BYTE_ONE) = 0xF1;
        *(i_pRealTimeWorkInfo + HEAD_BYTE_TWO) = 0xF2;
        *(i_pRealTimeWorkInfo + HEAD_BYTE_THREE) = 0xF3;
        *(i_pRealTimeWorkInfo + PRODUCTS_TYPE_ID_HIGH) = (uint8_t)(PRODUCT_MODEL_CODE >> 8);
        *(i_pRealTimeWorkInfo + PRODUCTS_TYPE_ID_LOW) = (uint8_t)(PRODUCT_MODEL_CODE & 0xFF);
        *(i_pRealTimeWorkInfo + LOCAL_NETWORK_ID_CODE) = LOCAL_NETWORK_ID;
        *(i_pRealTimeWorkInfo + INFORMATION_TYPE_CONTROL_CODE) = (uint8_t)REAL_TIME_RUNNING_INFORMATION_B;

//      //数据标签码码值增加
//      g_u32TxMsgDataTagNumber[REAL_TIME_RUNNING_INFORMATION]++;
//      *(i_pRealTimeWorkInfo + DATA_IDENTIFY_TAG_INF_CODE_1) = (uint8_t)(g_u32TxMsgDataTagNumber[REAL_TIME_RUNNING_INFORMATION] >> 24);
//      *(i_pRealTimeWorkInfo + DATA_IDENTIFY_TAG_INF_CODE_2) = (uint8_t)((g_u32TxMsgDataTagNumber[REAL_TIME_RUNNING_INFORMATION] & 0xFF0000) >> 16);
//      *(i_pRealTimeWorkInfo + DATA_IDENTIFY_TAG_INF_CODE_3) = (uint8_t)((g_u32TxMsgDataTagNumber[REAL_TIME_RUNNING_INFORMATION] & 0xFF00) >> 8);
//      *(i_pRealTimeWorkInfo + DATA_IDENTIFY_TAG_INF_CODE_4) = (uint8_t)(g_u32TxMsgDataTagNumber[REAL_TIME_RUNNING_INFORMATION] & 0xFF);
        *(i_pRealTimeWorkInfo + VALID_INFORMATION_LENGTH_CONTROL_CODE) = REAL_TIME_RUNNING_INFO_B_LENGTH;

        //运行警报码
        u32AlarmCode = GetRunAlarmCode();
        *(i_pRealTimeWorkInfo + RUN_ALARM_CODE_BYTE_4) = (uint8_t)(u32AlarmCode >> 24);
        *(i_pRealTimeWorkInfo + RUN_ALARM_CODE_BYTE_3) = (uint8_t)((u32AlarmCode & 0xFF0000) >> 16);
        *(i_pRealTimeWorkInfo + RUN_ALARM_CODE_BYTE_2) = (uint8_t)((u32AlarmCode & 0xFF00) >> 8);
        *(i_pRealTimeWorkInfo + RUN_ALARM_CODE_BYTE_1) = (uint8_t)(u32AlarmCode & 0xFF);

        //运行状态码
        u32SystemWorkStatuCode = GetSystemRunningStatuCode();
        *(i_pRealTimeWorkInfo + RUNNING_STATU_CODE_BYTE_4) = (uint8_t)(u32SystemWorkStatuCode >> 24);
        *(i_pRealTimeWorkInfo + RUNNING_STATU_CODE_BYTE_3) = (uint8_t)((u32SystemWorkStatuCode & 0xFF0000) >> 16);
        *(i_pRealTimeWorkInfo + RUNNING_STATU_CODE_BYTE_2) = (uint8_t)((u32SystemWorkStatuCode & 0xFF00) >> 8);
        *(i_pRealTimeWorkInfo + RUNNING_STATU_CODE_BYTE_1) = (uint8_t)(u32SystemWorkStatuCode & 0xFF);

        //电堆电流*100
        m_u16StackCurrentMul100 = (uint16_t)(GetSrcAnaSig(STACK_CURRENT) * 100);
        *(i_pRealTimeWorkInfo + STACK_CURRENT_INTEGER_PART) = (uint8_t)(m_u16StackCurrentMul100 / 100);
        *(i_pRealTimeWorkInfo + STACK_CURRENT_DECIMAL_PART) = (uint8_t)(m_u16StackCurrentMul100 % 100);

        //电堆电压*100
        m_u16StackVoltageMul100 = (uint16_t)(GetSrcAnaSig(STACK_VOLTAGE) * 100);
        *(i_pRealTimeWorkInfo + STACK_VOLTAGE_INTEGER_PART) = (uint8_t)(m_u16StackVoltageMul100 / 100);
        *(i_pRealTimeWorkInfo + STACK_VOLTAGE_DECIMAL_PART) = (uint8_t)(m_u16StackVoltageMul100 % 100);

        //电堆温度,暂时只用整数部分
        u16StackTemp = (uint16_t)GetSrcAnaSig(STACK_TEMP);
        *(i_pRealTimeWorkInfo + STACK_TEMP_INTEGER_PART) = (uint8_t)(u16StackTemp & 0xFF);
//      *(i_pRealTimeWorkInfo + STACK_TEMP_INTEGER_PART) = (uint8_t)((u32SystemWorkStatuCode & 0xFF00) >> 8);
//      *(i_pRealTimeWorkInfo + STACK_TEMP_DECIMAL_PART) = (uint8_t)(u32SystemWorkStatuCode & 0xFF);

        //电堆氢气压力*100
        m_u16StackPressMul100 = (uint16_t)(GetSrcAnaSig(HYDROGEN_PRESS_1) * 100);
        *(i_pRealTimeWorkInfo + STACK_HYDROGEN_PRESS_INTEGER_PART) = (uint8_t)(m_u16StackPressMul100 / 100);
        *(i_pRealTimeWorkInfo + STACK_HYDROGEN_PRESS_DECIMAL_PART) = (uint8_t)(m_u16StackPressMul100 % 100);

        //电堆风扇控制速度
        u16StackFanCtlSpd = GetStackFanCtlSpd();
        *(i_pRealTimeWorkInfo + STACK_FAN_SPD_CONTROL_HIGH) = (uint8_t)((u16StackFanCtlSpd & 0xFF00) >> 8);
        *(i_pRealTimeWorkInfo + STACK_FAN_SPD_CONTROL_LOW) = (uint8_t)(u16StackFanCtlSpd & 0xFF);

        //电堆风扇反馈速度
        u16StackFanSpdFeedBack = GetStackFanSpdFeedBack();
        *(i_pRealTimeWorkInfo + STACK_FAN_PART_A_SPD_FEEDBACK_HIGH) = (uint8_t)((u16StackFanSpdFeedBack & 0xFF00) >> 8);
        *(i_pRealTimeWorkInfo + STACK_FAN_PART_A_SPD_FEEDBACK_LOW) = (uint8_t)(u16StackFanSpdFeedBack & 0xFF);

        //本次发电时间,APP上发电界面显示的是这个值
        stStackWorkTimeThisTime = GetStackProductTimeThisTime();
        *(i_pRealTimeWorkInfo + STACK_WORK_TIME_THIS_TIME_HOUR_HIGH) = (uint8_t)((stStackWorkTimeThisTime.hour & 0xFF00) >> 8);
        *(i_pRealTimeWorkInfo + STACK_WORK_TIME_THIS_TIME_HOUR_LOW) = (uint8_t)(stStackWorkTimeThisTime.hour & 0xFF);
        *(i_pRealTimeWorkInfo + STACK_WORK_TIME_THIS_TIME_MINUTE) = (uint8_t)(stStackWorkTimeThisTime.minute);
        *(i_pRealTimeWorkInfo + STACK_WORK_TIME_THIS_TIME_SECOND) = (uint8_t)(stStackWorkTimeThisTime.second);

        //累计发电时间
        stStackWorkTimeTotal = GetStackProductTimeTotal();
        *(i_pRealTimeWorkInfo + STACK_WORK_TIME_TOTAL_HOUR_HIGH) = (uint8_t)((stStackWorkTimeTotal.hour & 0xFF00) >> 8);
        *(i_pRealTimeWorkInfo + STACK_WORK_TIME_TOTAL_HOUR_LOW) = (uint8_t)(stStackWorkTimeTotal.hour & 0xFF);
        *(i_pRealTimeWorkInfo + STACK_WORK_TIME_TOTAL_MINUTE) = (uint8_t)(stStackWorkTimeTotal.minute);
        *(i_pRealTimeWorkInfo + STACK_WORK_TIME_TOTAL_SECOND) = (uint8_t)(stStackWorkTimeTotal.second);

        //当前输出功率
        u32StackPower = (uint32_t)(GetCurrentPower() * 100);
        *(i_pRealTimeWorkInfo + CURRENT_ISOLATED_POWER_INTEGER_PART_HIGH) = (uint8_t)(((uint16_t)(u32StackPower / 100) & 0xFF00) >> 8);
        *(i_pRealTimeWorkInfo + CURRENT_ISOLATED_POWER_INTEGER_PART_LOW) = (uint8_t)((uint16_t)(u32StackPower / 100) & 0xFF);
        *(i_pRealTimeWorkInfo + CURRENT_ISOLATED_POWER_DECIMAL_PART) = (uint8_t)(u32StackPower % 100);

        //本次单机发电量
        u32IsolatedGenratedEnergyThisTime = (uint32_t)(GetIsolatedGenratedEnergyThisTime() * 1000);
        *(i_pRealTimeWorkInfo + ISOLATED_GENERATED_ENERGY_THIE_TIME_INTEGER_PART_HIGH) = (uint8_t)(((uint16_t)(u32IsolatedGenratedEnergyThisTime / 1000) & 0xFF00) >> 8);
        *(i_pRealTimeWorkInfo + ISOLATED_GENERATED_ENERGY_THIE_TIME_INTEGER_PART_LOW) = (uint8_t)((uint16_t)(u32IsolatedGenratedEnergyThisTime / 1000) & 0xFF);
        *(i_pRealTimeWorkInfo + ISOLATED_GENERATED_ENERGY_THIE_TIME_DECIMAL_PART_HIGH) = (uint8_t)(((uint16_t)(u32IsolatedGenratedEnergyThisTime % 1000) & 0xFF00) >> 8);
        *(i_pRealTimeWorkInfo + ISOLATED_GENERATED_ENERGY_THIE_TIME_DECIMAL_PART_LOW) = (uint8_t)((uint16_t)(u32IsolatedGenratedEnergyThisTime % 1000) & 0xFF);

        //匹氢偏移值
        i16HydrogYieldMatchOffsetValue = (int16_t)(GetStackHydrogenYieldMatchOffsetValue() * 100);

        if(i16HydrogYieldMatchOffsetValue > 0 || i16HydrogYieldMatchOffsetValue < -100) { //整数位带符号位
            *(i_pRealTimeWorkInfo + HYDROGEN_YIELD_MATCHING_OFFSET_VALUE_INTEGER_PART_MUL100) = (int8_t)(i16HydrogYieldMatchOffsetValue / 100);
            *(i_pRealTimeWorkInfo + HYDROGEN_YIELD_MATCHING_OFFSET_VALUE_DECIMAL_PART_MUL100_HIGH) = (uint8_t)(((i16HydrogYieldMatchOffsetValue % 100)) & 0xFF);
        } else { //小数位带符号位
            *(i_pRealTimeWorkInfo + HYDROGEN_YIELD_MATCHING_OFFSET_VALUE_INTEGER_PART_MUL100) = (uint8_t)(i16HydrogYieldMatchOffsetValue / 100);
            *(i_pRealTimeWorkInfo + HYDROGEN_YIELD_MATCHING_OFFSET_VALUE_DECIMAL_PART_MUL100_HIGH) = (int8_t)((i16HydrogYieldMatchOffsetValue % 100) & 0xFF);
        }

        //子模块ID号
//        *(i_pRealTimeWorkInfo + SUB_MODULE_ID_OF_THE_MULTI_MODULE_TYPE) = SUB_MODULE_ID;

        //数据报尾段
//        *(i_pRealTimeWorkInfo + END_BYTE_ONE) = 0x5F;
//        *(i_pRealTimeWorkInfo + END_BYTE_TWO) = 0x6F;

    } else { //载入历史数据

    }
}

/*
***************************************************************************************************
*                                      LoadFuelCellRealTimeWorkInfoPartA()
*
* Description:  Load fuel cell real time work Info.
*
* Arguments  :  none
*
* Returns    :  none
***************************************************************************************************
*/
static void LoadFuelCellRealTimeWorkInfoPartB(uint8_t i_uint8_tIsHistoryData, uint8_t *i_pRealTimeWorkInfo)
{
    uint16_t   u16DecompressCountPerMin = 0;
    uint16_t   u16VacuumNetativePressure = 0.0;
    uint16_t   u16BatteryVoltage = 0.0;
    uint16_t   u16BatteryCurrent = 0.0;

    if(i_uint8_tIsHistoryData == EN_LATEST) { //最新的数据
        
        //数据报头段
        *(i_pRealTimeWorkInfo + HEAD_BYTE_ONE) = 0xF1;
        *(i_pRealTimeWorkInfo + HEAD_BYTE_TWO) = 0xF2;
        *(i_pRealTimeWorkInfo + HEAD_BYTE_THREE) = 0xF3;
        *(i_pRealTimeWorkInfo + PRODUCTS_TYPE_ID_HIGH) = (uint8_t)(PRODUCT_MODEL_CODE >> 8);
        *(i_pRealTimeWorkInfo + PRODUCTS_TYPE_ID_LOW) = (uint8_t)(PRODUCT_MODEL_CODE & 0xFF);
        *(i_pRealTimeWorkInfo + LOCAL_NETWORK_ID_CODE) = LOCAL_NETWORK_ID;
        *(i_pRealTimeWorkInfo + INFORMATION_TYPE_CONTROL_CODE) = (uint8_t)REAL_TIME_RUNNING_INFORMATION_B;

//      //数据标签码码值增加
//      g_u32TxMsgDataTagNumber[REAL_TIME_RUNNING_INFORMATION]++;
//      *(i_pRealTimeWorkInfo + DATA_IDENTIFY_TAG_INF_CODE_1) = (uint8_t)(g_u32TxMsgDataTagNumber[REAL_TIME_RUNNING_INFORMATION] >> 24);
//      *(i_pRealTimeWorkInfo + DATA_IDENTIFY_TAG_INF_CODE_2) = (uint8_t)((g_u32TxMsgDataTagNumber[REAL_TIME_RUNNING_INFORMATION] & 0xFF0000) >> 16);
//      *(i_pRealTimeWorkInfo + DATA_IDENTIFY_TAG_INF_CODE_3) = (uint8_t)((g_u32TxMsgDataTagNumber[REAL_TIME_RUNNING_INFORMATION] & 0xFF00) >> 8);
//      *(i_pRealTimeWorkInfo + DATA_IDENTIFY_TAG_INF_CODE_4) = (uint8_t)(g_u32TxMsgDataTagNumber[REAL_TIME_RUNNING_INFORMATION] & 0xFF);
        *(i_pRealTimeWorkInfo + VALID_INFORMATION_LENGTH_CONTROL_CODE) = REAL_TIME_RUNNING_INFO_B_LENGTH;

        //电堆每分钟泄压排气次数
        u16DecompressCountPerMin = GetPassiveDecompressCountPerMinutes();
        *(i_pRealTimeWorkInfo + STACK_DECOMPRESS_COUNT_PER_MINUTES_HIGH) = (uint8_t)((u16DecompressCountPerMin & 0xFF00) >> 8);
        *(i_pRealTimeWorkInfo + STACK_DECOMPRESS_COUNT_PER_MINUTES_LOW) = (uint8_t)(u16DecompressCountPerMin & 0xFF);

        //电池电压
        u16BatteryVoltage = (uint16_t)(GetSrcAnaSig(BATTERY_VOLTAGE) * 100);
        *(i_pRealTimeWorkInfo + BATTERY_VOLTAGE_INTEGER_PART) = (uint8_t)(u16BatteryVoltage / 100);
        *(i_pRealTimeWorkInfo + BATTERY_VOLTAGE_IDECIMAL_PART) = (uint8_t)(u16BatteryVoltage % 100);

        //电池电流
        u16BatteryCurrent = (uint16_t)(GetSrcAnaSig(BATTERY_CURRENT) * 100);
        *(i_pRealTimeWorkInfo + BATTERY_CURRENT_INTEGER_PART) = (uint8_t)(u16BatteryCurrent / 100);
        *(i_pRealTimeWorkInfo + BATTERY_CURRENT_IDECIMAL_PART) = (uint8_t)(u16BatteryCurrent % 100);
        
        //真空负压
        u16VacuumNetativePressure = (uint16_t)(GetSrcAnaSig(NEGATIVE_PRESSURE) * 100);
        *(i_pRealTimeWorkInfo + VACUUM_NEGATIVE_PRESSURE_HIGH) = (uint8_t)(u16VacuumNetativePressure / 100);
        *(i_pRealTimeWorkInfo + VACUUM_NEGATIVE_PRESSURE_LOW) = (uint8_t)(u16VacuumNetativePressure % 100);
        
        //子模块ID号
//        *(i_pRealTimeWorkInfo + SUB_MODULE_ID_OF_THE_MULTI_MODULE_TYPE) = SUB_MODULE_ID;

        //数据报尾段
        *(i_pRealTimeWorkInfo + END_BYTE_ONE) = 0x5F;
        *(i_pRealTimeWorkInfo + END_BYTE_TWO) = 0x6F;

    } else { //载入历史数据

    }
}
/*
***************************************************************************************************
*                                      InsertNonRealTimeWorkInfoDataToSendBuff()
*
* Description:  插入非实时工作信息类数据到发送缓存区，出于实时性考虑，将其插入到第一个待发送的数据的位置.
*
* Arguments  :  none.
*
* Returns    :  none.
*
* Note(s)    :  The flag is included in the message that will send to the control side.
***************************************************************************************************
*/
static void InsertNonRealTimeWorkInfoDataToSendBuff(uint8_t i_eSendDataType, uint8_t i_uint8_tCmdCode, uint8_t i_uint8_tParaLen, uint8_t *p_uint8_tParas)
{
    OS_ERR  err;
    uint8_t i;
    uint8_t uint8_tInsertCursor;

//  APP_TRACE_INFO(("Insert the non-real time info...\r\n"));
    OSMutexPend(&TxMsgSendBuffWriteMutex,
                OS_CFG_TICK_RATE_HZ / 10,
                OS_OPT_PEND_BLOCKING,
                NULL,
                &err);

    if(g_stTxMsgDataSendBuff.Q_length == 0) { //直接放在第一个位置即可
//      APP_TRACE_INFO(("Insert the non-real time in the head...\r\n"));
        g_stTxMsgDataSendBuff.Q_length++;
        g_stTxMsgDataSendBuff.Q_Qhead = 0;
        g_stTxMsgDataSendBuff.Q_Qrear = 0;
        g_stTxMsgDataSendBuff.Queue[g_stTxMsgDataSendBuff.Q_Qrear].D_last = g_stTxMsgDataSendBuff.Q_Qhead;
        g_stTxMsgDataSendBuff.Queue[g_stTxMsgDataSendBuff.Q_Qrear].D_next = g_stTxMsgDataSendBuff.Q_Qrear;
        g_stTxMsgDataSendBuff.Queue[g_stTxMsgDataSendBuff.Q_Qrear].OccupyStatu = EN_WAITTING;
        LoadNonRealTimeWorkInfo(i_eSendDataType, i_uint8_tCmdCode, i_uint8_tParaLen, p_uint8_tParas, g_stTxMsgDataSendBuff.Queue[g_stTxMsgDataSendBuff.Q_Qhead].Data);
    } else if(++g_stTxMsgDataSendBuff.Q_length <= TX_MSG_SEND_QUEUE_SIZE) {
//      APP_TRACE_INFO(("Q_length normal...\r\n"));
        for(i = 0; i < TX_MSG_SEND_QUEUE_SIZE; i++) {
            if(g_stTxMsgDataSendBuff.Queue[i].OccupyStatu == EN_FREE) {
                uint8_tInsertCursor = i;
                g_stTxMsgDataSendBuff.Queue[uint8_tInsertCursor].OccupyStatu = EN_WAITTING;
                LoadNonRealTimeWorkInfo(i_eSendDataType, i_uint8_tCmdCode, i_uint8_tParaLen, p_uint8_tParas, g_stTxMsgDataSendBuff.Queue[uint8_tInsertCursor].Data);
                break;
            }
        }

        if(g_stTxMsgDataSendBuff.Q_length == 2) {
            if(g_stTxMsgDataSendBuff.Queue[g_stTxMsgDataSendBuff.Q_Qhead].OccupyStatu == EN_WAITTING) { //头节点也在等待中
                //直接替换头节点
                g_stTxMsgDataSendBuff.Queue[uint8_tInsertCursor].D_last = uint8_tInsertCursor;
                g_stTxMsgDataSendBuff.Queue[uint8_tInsertCursor].D_next = g_stTxMsgDataSendBuff.Q_Qhead;
                g_stTxMsgDataSendBuff.Queue[g_stTxMsgDataSendBuff.Q_Qhead].D_last = uint8_tInsertCursor;
                g_stTxMsgDataSendBuff.Q_Qhead = uint8_tInsertCursor;
            } else { //新节点增加到头节点后面
                g_stTxMsgDataSendBuff.Queue[uint8_tInsertCursor].D_last = g_stTxMsgDataSendBuff.Q_Qhead;
                g_stTxMsgDataSendBuff.Queue[uint8_tInsertCursor].D_next = uint8_tInsertCursor;
                g_stTxMsgDataSendBuff.Queue[g_stTxMsgDataSendBuff.Q_Qhead].D_next = uint8_tInsertCursor;
                g_stTxMsgDataSendBuff.Q_Qrear = uint8_tInsertCursor;
            }
        } else {
            for(i = g_stTxMsgDataSendBuff.Q_Qhead; ;) {
                if(g_stTxMsgDataSendBuff.Queue[i].OccupyStatu == EN_WAITTING) {
                    g_stTxMsgDataSendBuff.Queue[i].D_next = i;//先前从头节点开始第一个等待的节点的前向标记指向要插入新数据的节点
                    g_stTxMsgDataSendBuff.Queue[i].D_last = g_stTxMsgDataSendBuff.Q_Qrear;//开辟的新节点的前向标记指向原来的尾节点
                    g_stTxMsgDataSendBuff.Q_Qrear = i;//尾节点更新到新的节点
                    g_stTxMsgDataSendBuff.Queue[i].OccupyStatu = EN_WAITTING;//新节点的状态更新为等待发送
                    break;
                } else {
                    i = g_stTxMsgDataSendBuff.Queue[i].D_next;//搜索下一个节点
                }
            }
        }
    } else {
        --g_stTxMsgDataSendBuff.Q_length;
        APP_TRACE_INFO(("The Tx buf is overflow...\r\n"));
    }

    OSMutexPost(&TxMsgSendBuffWriteMutex,
                OS_OPT_POST_NONE,
                &err);
}
/*
***************************************************************************************************
*                                      LoadNonRealTimeWorkInfo()
*
* Description:  This is a task that manage the communication with other device.
*
* Arguments  :  none
*
* Returns    :  none
***************************************************************************************************
*/
void LoadNonRealTimeWorkInfo(uint8_t i_eSendDataType, uint8_t i_uint8_tCmdCode, uint8_t i_uint8_tParaLen, uint8_t *p_uint8_tParas, uint8_t *i_pNonRealTimeWorkInfo)
{
    uint8_t i;
    SELF_CHECK_CODE_Typedef stSelfCheckCode;
    uint16_t m_u16ConrolAndCommunicateStatuCode;

//  APP_TRACE_INFO(("load the non real time data...\r\n"));

    if((i_eSendDataType != REAL_TIME_RUNNING_INFORMATION_A) && (i_eSendDataType != REAL_TIME_RUNNING_INFORMATION_B) && (i_eSendDataType < EN_SEND_DATA_TYPE_MAX)) {
        //数据报头段
        *(i_pNonRealTimeWorkInfo + HEAD_BYTE_ONE) = 0xF1;
        *(i_pNonRealTimeWorkInfo + HEAD_BYTE_TWO) = 0xF2;
        *(i_pNonRealTimeWorkInfo + HEAD_BYTE_THREE) = 0xF3;
        *(i_pNonRealTimeWorkInfo + PRODUCTS_TYPE_ID_HIGH) = (uint8_t)((PRODUCT_MODEL_CODE & 0xFF00) >> 8);
        *(i_pNonRealTimeWorkInfo + PRODUCTS_TYPE_ID_LOW) = (uint8_t)(PRODUCT_MODEL_CODE & 0xFF);
        *(i_pNonRealTimeWorkInfo + LOCAL_NETWORK_ID_CODE) = LOCAL_NETWORK_ID;


//      g_u32TxMsgDataTagNumber[i_eSendDataType]++;
//      *(i_pNonRealTimeWorkInfo + DATA_IDENTIFY_TAG_INF_CODE_1) = (uint8_t)(g_u32TxMsgDataTagNumber[i_eSendDataType] >> 24);
//      *(i_pNonRealTimeWorkInfo + DATA_IDENTIFY_TAG_INF_CODE_2) = (uint8_t)((g_u32TxMsgDataTagNumber[i_eSendDataType] & 0xFF0000) >> 16);
//      *(i_pNonRealTimeWorkInfo + DATA_IDENTIFY_TAG_INF_CODE_3) = (uint8_t)((g_u32TxMsgDataTagNumber[i_eSendDataType] & 0xFF00) >> 8);
//      *(i_pNonRealTimeWorkInfo + DATA_IDENTIFY_TAG_INF_CODE_4) = (uint8_t)(g_u32TxMsgDataTagNumber[i_eSendDataType] & 0xFF);

        //数据类型控制码的高4位尚未定义，在此清零
        *(i_pNonRealTimeWorkInfo + INFORMATION_TYPE_CONTROL_CODE) = (uint8_t)(i_eSendDataType & 0x0F);

        switch((uint8_t)i_eSendDataType) {
            case REAL_TIME_REQUEST_INFORMATION:
                *(i_pNonRealTimeWorkInfo + VALID_INFORMATION_LENGTH_CONTROL_CODE) = REAL_TIME_REQUEST_INFO_LENGTH;
                *(i_pNonRealTimeWorkInfo + REQUEST_INFORMATION_TYPE) = i_uint8_tCmdCode;
                *(i_pNonRealTimeWorkInfo + LENGTH_OF_REQUEST_PARAMETERS) = i_uint8_tParaLen;
                i = 0;

                while(i < i_uint8_tParaLen) {
                    *(i_pNonRealTimeWorkInfo + REQUEST_PARAMETER_ONE + i) = *(p_uint8_tParas + i);
                    i++;
                }

                break;

            case REAL_TIME_ASSIST_INFORMATION:
                *(i_pNonRealTimeWorkInfo + VALID_INFORMATION_LENGTH_CONTROL_CODE) = REAL_TIME_ASSIST_INFO_LENGTH;
                *(i_pNonRealTimeWorkInfo + LEGAL_AUTHORIZATION_CODE) = 0;//暂未设权限检验码
                stSelfCheckCode = GetSysSelfCheckCode();
                *(i_pNonRealTimeWorkInfo + SELF_CHECK_SENSOR_STATUS_CODE) = stSelfCheckCode.DevSelfCheckSensorStatusCode;

                *(i_pNonRealTimeWorkInfo + SELF_CHECK_CODE_GROUP_HYDRGEN_BYTE_4) = (uint8_t)((stSelfCheckCode.MachinePartASelfCheckCode & 0xFF000000) >> 24);
                *(i_pNonRealTimeWorkInfo + SELF_CHECK_CODE_GROUP_HYDRGEN_BYTE_3) = (uint8_t)((stSelfCheckCode.MachinePartASelfCheckCode & 0xFF0000) >> 16);
                *(i_pNonRealTimeWorkInfo + SELF_CHECK_CODE_GROUP_HYDRGEN_BYTE_2) = (uint8_t)((stSelfCheckCode.MachinePartASelfCheckCode & 0xFF00) >> 8);
                *(i_pNonRealTimeWorkInfo + SELF_CHECK_CODE_GROUP_HYDRGEN_BYTE_1) = (uint8_t)(stSelfCheckCode.MachinePartASelfCheckCode & 0xFF);

                *(i_pNonRealTimeWorkInfo + SELF_CHECK_CODE_GROUP_FUEL_CELL_BYTE_4) = (uint8_t)((stSelfCheckCode.MachinePartBSelfCheckCode & 0xFF000000) >> 24);
                *(i_pNonRealTimeWorkInfo + SELF_CHECK_CODE_GROUP_FUEL_CELL_BYTE_3) = (uint8_t)((stSelfCheckCode.MachinePartBSelfCheckCode & 0xFF0000) >> 16);
                *(i_pNonRealTimeWorkInfo + SELF_CHECK_CODE_GROUP_FUEL_CELL_BYTE_2) = (uint8_t)((stSelfCheckCode.MachinePartBSelfCheckCode & 0xFF00) >> 8);
                *(i_pNonRealTimeWorkInfo + SELF_CHECK_CODE_GROUP_FUEL_CELL_BYTE_1) = (uint8_t)(stSelfCheckCode.MachinePartBSelfCheckCode & 0xFF);

                //通信状态控制码
                m_u16ConrolAndCommunicateStatuCode = GetConrolAndCommunicateStatuCode();
                *(i_pNonRealTimeWorkInfo + CONTROL_AND_COMMUNICATE_STATU_CODE_BYTE_H) = (uint8_t)((m_u16ConrolAndCommunicateStatuCode & 0xFF00) >> 8);
                *(i_pNonRealTimeWorkInfo + CONTROL_AND_COMMUNICATE_STATU_CODE_BYTE_L) = (uint8_t)(m_u16ConrolAndCommunicateStatuCode & 0xFF);

                break;

            case CONSTANT_ASSIST_INFORMATION:
                *(i_pNonRealTimeWorkInfo + VALID_INFORMATION_LENGTH_CONTROL_CODE) = CONSTANT_ASSIST_INFORMATION_LENGTH;
                *(i_pNonRealTimeWorkInfo + LEGAL_AUTHORIZATION_CODE) = 0;//暂未设权限检验码
                break;

#if RUNNING_CONFIG_INTERFACE

            case FOR_QUERY_AND_CONFIG_INFORMATION:
                *(i_pNonRealTimeWorkInfo + VALID_INFORMATION_LENGTH_CONTROL_CODE) = FOR_QUERY_AND_CONFIG_INFORMATION_LENGTH;

                /*获取修改后的Flash参数并发送到上位机确认*/
                GetStartHydrgPumpSpdParaFromFlash(&g_stStartHydrgPumpSpdPara);
                GetStartHydrgFanSpdParaFromFlash(&g_stStartHydrgFanSpdPara);

                *(i_pNonRealTimeWorkInfo + IGNITE_FIRST_STEP_PUMP_SPD_HIGH) = (uint8_t)((g_stStartHydrgPumpSpdPara.PumpSpdIgniterFirstTime & 0xFF00) >> 8);
                *(i_pNonRealTimeWorkInfo + IGNITE_FIRST_STEP_PUMP_SPD_LOW) = (uint8_t)(g_stStartHydrgPumpSpdPara.PumpSpdIgniterFirstTime & 0xFF);

                *(i_pNonRealTimeWorkInfo + IGNITE_FIRST_STEP_FAN_SPD_HIGH) = (uint8_t)((g_stStartHydrgFanSpdPara.FanSpdIgniterFirstTime & 0xFF00) >> 8);
                *(i_pNonRealTimeWorkInfo + IGNITE_FIRST_STEP_FAN_SPD_LOW) = (uint8_t)(g_stStartHydrgFanSpdPara.FanSpdIgniterFirstTime & 0xFF);

                *(i_pNonRealTimeWorkInfo + IGNITE_FIRST_SUCCESSED_FAN_SPD_HIGH) = (uint8_t)((g_stStartHydrgFanSpdPara.FanSpdAfterIgniterFirstSuccessd & 0xFF00) >> 8);
                *(i_pNonRealTimeWorkInfo + IGNITE_FIRST_SUCCESSED_FAN_SPD_LOW) = (uint8_t)(g_stStartHydrgFanSpdPara.FanSpdAfterIgniterFirstSuccessd & 0xFF);

                *(i_pNonRealTimeWorkInfo + IGNITE_SECOND_STEP_PUMP_SPD_HIGH) = (uint8_t)((g_stStartHydrgPumpSpdPara.PumpSpdIgniterSecondTime & 0xFF00) >> 8);
                *(i_pNonRealTimeWorkInfo + IGNITE_SECOND_STEP_PUMP_SPD_LOW) = (uint8_t)(g_stStartHydrgPumpSpdPara.PumpSpdIgniterSecondTime & 0xFF);

                *(i_pNonRealTimeWorkInfo + IGNITE_SECOND_STEP_FAN_SPD_HIGH) = (uint8_t)((g_stStartHydrgFanSpdPara.FanSpdIgniterSecondTime & 0xFF00) >> 8);
                *(i_pNonRealTimeWorkInfo + IGNITE_SECOND_STEP_FAN_SPD_LOW) = (uint8_t)(g_stStartHydrgFanSpdPara.FanSpdIgniterSecondTime & 0xFF);

                *(i_pNonRealTimeWorkInfo + IGNITE_SECOND_SUCCESSED_FAN_SPD_HIGH) = (uint8_t)((g_stStartHydrgFanSpdPara.FanSpdAfterIgniterSecondSuccessd & 0xFF00) >> 8);
                *(i_pNonRealTimeWorkInfo + IGNITE_SECOND_SUCCESSED_FAN_SPD_LOW) = (uint8_t)(g_stStartHydrgFanSpdPara.FanSpdAfterIgniterSecondSuccessd & 0xFF);

                *(i_pNonRealTimeWorkInfo + RUNNING_STATUS_LIQUID_PRESS_EXCEED_4KG_PUMP_SPD_HIGH) = (uint8_t)((g_stStartHydrgPumpSpdPara.PumpSpdAfterLiquidPressExceed4Kg & 0xFF00) >> 8);
                *(i_pNonRealTimeWorkInfo + RUNNING_STATUS_LIQUID_PRESS_EXCEED_4KG_PUMP_SPD_LOW) = (uint8_t)(g_stStartHydrgPumpSpdPara.PumpSpdAfterLiquidPressExceed4Kg & 0xFF);

                break;
#endif

            default:
                break;
        }

        //数据报尾段
        *(i_pNonRealTimeWorkInfo + END_BYTE_ONE) = 0x5F;
        *(i_pNonRealTimeWorkInfo + END_BYTE_TWO) = 0x6F;

    } else {
        APP_TRACE_INFO(("The load data type is illegal...\r\n"));
    }
}

/*
***************************************************************************************************
*                                      SendAPrgmMsgFrame()
*
* Description:  The use of the funciton start the DMA send the message that sendout.
*
* Arguments  :  none
*
* Returns    :  none
***************************************************************************************************
*/
static void SendAPrgmMsgFrame(uint8_t i_uint8_tTxMsgLen, uint8_t *i_pTxMsg)
{
    BSP_PrgmDataDMASend(i_uint8_tTxMsgLen, i_pTxMsg);
}

/*
***************************************************************************************************
*                                      ResponsePrgmCommand()
*
* Description:  anwser the command that receive from the control side under the Dubug mode.
*
* Arguments  :  none
*
* Returns    :  none.
***************************************************************************************************
*/
static void ResponsePrgmCommand(uint8_t *i_PrgmRxMsg)
{
    OS_ERR      err;
    uint32_t    u32ErrCode;
    uint8_t     i;

    if((*(i_PrgmRxMsg + RECEIVE_DATA_BYTE_HEAD_ONE)     == 0xFC)
            && (*(i_PrgmRxMsg + RECEIVE_DATA_BYTE_HEAD_TWO)     == 0xFD)
            && (*(i_PrgmRxMsg + RECEIVE_DATA_BYTE_HEAD_THREE)   == 0xFE)
//            && (*(i_PrgmRxMsg + REDEIVE_DATA_BYTE_CMD_CHILD_MODULE_ID)   == 0x01)//暂不判定子模块ID
//            && (*(i_PrgmRxMsg + REDEIVE_DATA_BYTE_END_OF_DATA)   == 0xAA)//报尾
//      && (i_PrgmRxMsg[RECEIVE_DATA_BYTE_HEAD_TARGET_LOCAL_NET_ID]     == 0xFF)//暂不判定接收ID
      ) {
        switch(*(i_PrgmRxMsg + RECEIVE_DATA_BYTE_CMD_TYPE) & 0x0F) { //指令类型码的高4位尚未定义——将其清零
            case COMMAND_TYPE_DBG://调试指令
                switch(*(i_PrgmRxMsg + REDEIVE_DATA_BYTE_CMD_CODE_VALUE)) {
                    case DBG_SELECT_WORK_MODE:
                        SetWorkMode((SYSTEM_WORK_MODE_Typedef) * (i_PrgmRxMsg + REDEIVE_DATA_BYTE_CMD_PARAMETER_SECTION_ONE_1));
                        OSSemPost(&MannualSelcetWorkModeSem,
                                  OS_OPT_POST_1,
                                  &err);
                        break;

                    case DBG_SWITCH_OVER_CONTROL_MODE:
//                        APP_TRACE_INFO(("Cmd ->Control_mode turn over ...\r\n"));
//                        ControlModeTurnOver();
//                        SendRealTimeAssistInfo();
                        APP_TRACE_INFO(("Ignite for 60 seconds...\r\n"));
                        IgniterWorkForSeconds(60);
                        break;

                    case DBG_START_THE_MACHINE:
                        APP_TRACE_INFO(("Cmd ->Start ...\r\n"));
                        CmdStart();
                        break;

                    case DBG_AHEAD_RUNNING:
                        APP_TRACE_INFO(("Cmd ->Ahead Running ...\r\n"));

                        if(EN_START_PRGM_ONE_BEHIND == GetSystemWorkStatu()) {
                            SetAheadRunningFlag(YES);
                            OSSemPost(&IgniteFirstBehindWaitSem,
                                      OS_OPT_POST_1,
                                      &err);
                        }

                        break;

                    case DBG_SHUT_DOWN_THE_MACHINE:
                        APP_TRACE_INFO(("Cmd -> Shutdown ...\r\n"));
                        CmdShutDown();
                        break;

                    case DBG_CHANGE_THE_WORK_STATU_TO_STANDING_BY:
                        if(EN_KEEPING_WARM == GetSystemWorkStatu()) {
                            SetSystemWorkStatu(EN_WAITTING_COMMAND);
                        }

                        break;

                    case DBG_PUMP_SPEED_INC:
                        PumpSpdInc();
                        break;

                    case DBG_PUMP_SPEED_DEC:
                        PumpSpdDec();
                        break;

                    case DBG_PUMP_SPEED_SET_WITH_PARAMETERS:
                        SetPumpCtlSpd((uint16_t)(*(i_PrgmRxMsg + REDEIVE_DATA_BYTE_CMD_PARAMETER_SECTION_ONE_1) << 8) + * (i_PrgmRxMsg + REDEIVE_DATA_BYTE_CMD_PARAMETER_SECTION_ONE_2));
                        break;

                    case DBG_HYDRG_SPEED_INC:
                        HydrgFanSpdInc();
                        break;

                    case DBG_HYDRG_SPEED_DEC:
                        HydrgFanSpdDec();
                        break;

                    case DBG_HYDRG_SPEED_SET_WHTI_PARAMETERS:
//                        SetHydrgFanCtlSpd((uint16_t)((*(i_PrgmRxMsg + REDEIVE_DATA_BYTE_CMD_PARAMETER_SECTION_ONE_1) << 8) + * (i_PrgmRxMsg + REDEIVE_DATA_BYTE_CMD_PARAMETER_SECTION_ONE_2)));
                        break;

                    case DBG_STACK_FAN_SPEED_INC:
                        StackFanSpdInc();
                        break;

                    case DBG_STACK_FAN_SPEED_DEC:
                        StackFanSpdDec();
                        break;

                    case DBG_OPEN_IGNITER:
                        IgniterWorkForSeconds(60);
                        break;

                    case DBG_CLOSE_IGNITER:
                        IgniterWorkForSeconds(0);
                        break;

                    default:
                        break;
                }

                break;
#if RUNNING_CONFIG_INTERFACE

            case COMMAND_TYPE_CONFIGURATION: //配置类指令
                if(EN_WAITTING_COMMAND == GetSystemWorkStatu()) { //待机状态下才能修改参数
                    SetRunningParaCfgSwitch(ON);  //打开运行配置参数配置开关

                    switch(*(i_PrgmRxMsg + REDEIVE_DATA_BYTE_CMD_CODE_VALUE)) {
                        case CONFIG_HYDROGEN_GROUP_RUNNING_PARA:
                            switch(*(i_PrgmRxMsg + REDEIVE_DATA_BYTE_CMD_PARAMETER_SECTION_ONE_1)) {
                                case CONFIG_IGNITE_FIRST_STEP_PARA:
                                    switch(*(i_PrgmRxMsg + REDEIVE_DATA_BYTE_CMD_PARAMETER_SECTION_ONE_2)) {
                                        case CONFIG_IGNITE_FIRST_STEP_PUMP_SPD:
                                            g_stStartHydrgPumpSpdPara.PumpSpdIgniterFirstTime = (uint16_t)((*(i_PrgmRxMsg + REDEIVE_DATA_BYTE_CMD_PARAMETER_SECTION_ONE_3) << 8) + * (i_PrgmRxMsg + REDEIVE_DATA_BYTE_CMD_PARAMETER_SECTION_ONE_4));
                                            OS_FlagPost(&ConfigParametersChangeState, CFG_IGNITE_FIRST_STEP_PUMP_SPD, OS_OPT_POST_FLAG_SET, 0, &err);
                                            break;

                                        case CONFIG_IGNITE_FIRST_STEP_FAN_SPD:
                                            g_stStartHydrgFanSpdPara.FanSpdIgniterFirstTime = (uint16_t)((*(i_PrgmRxMsg + REDEIVE_DATA_BYTE_CMD_PARAMETER_SECTION_ONE_3) << 8) + * (i_PrgmRxMsg + REDEIVE_DATA_BYTE_CMD_PARAMETER_SECTION_ONE_4));
                                            OS_FlagPost(&ConfigParametersChangeState, CFG_IGNITE_FIRST_STEP_FAN_SPD, OS_OPT_POST_FLAG_SET, 0, &err);
                                            break;

                                        case CONFIG_IGNITE_FIRST_SUCCESSED_FAN_SPD:
                                            g_stStartHydrgFanSpdPara.FanSpdAfterIgniterFirstSuccessd = (uint16_t)((*(i_PrgmRxMsg + REDEIVE_DATA_BYTE_CMD_PARAMETER_SECTION_ONE_3) << 8) + * (i_PrgmRxMsg + REDEIVE_DATA_BYTE_CMD_PARAMETER_SECTION_ONE_4));
                                            OS_FlagPost(&ConfigParametersChangeState, CFG_IGNITE_FIRST_SUCCESSED_FAN_SPD, OS_OPT_POST_FLAG_SET, 0, &err);
                                            break;

                                        default:
                                            break;
                                    }

                                    break;

                                case CONFIG_IGNITE_SECOND_STEP_PARA:
                                    switch(*(i_PrgmRxMsg + REDEIVE_DATA_BYTE_CMD_PARAMETER_SECTION_ONE_2)) {
                                        case CONFIG_IGNITE_SECOND_STEP_PUMP_SPD:
                                            g_stStartHydrgPumpSpdPara.PumpSpdIgniterSecondTime = (uint16_t)((*(i_PrgmRxMsg + REDEIVE_DATA_BYTE_CMD_PARAMETER_SECTION_ONE_3) << 8) + * (i_PrgmRxMsg + REDEIVE_DATA_BYTE_CMD_PARAMETER_SECTION_ONE_4));
                                            OS_FlagPost(&ConfigParametersChangeState, CFG_IGNITE_SECOND_STEP_PUMP_SPD, OS_OPT_POST_FLAG_SET, 0, &err);
                                            break;

                                        case CONFIG_IGNITE_SECOND_STEP_FAN_SPD:
                                            g_stStartHydrgFanSpdPara.FanSpdIgniterSecondTime = (uint16_t)((*(i_PrgmRxMsg + REDEIVE_DATA_BYTE_CMD_PARAMETER_SECTION_ONE_3) << 8) + * (i_PrgmRxMsg + REDEIVE_DATA_BYTE_CMD_PARAMETER_SECTION_ONE_4));
                                            OS_FlagPost(&ConfigParametersChangeState, CFG_IGNITE_SECOND_STEP_FAN_SPD, OS_OPT_POST_FLAG_SET, 0, &err);
                                            break;

                                        case CONFIG_IGNITE_SECOND_SUCCESSED_FAN_SPD:
                                            g_stStartHydrgFanSpdPara.FanSpdAfterIgniterSecondSuccessd = (uint16_t)((*(i_PrgmRxMsg + REDEIVE_DATA_BYTE_CMD_PARAMETER_SECTION_ONE_3) << 8) + * (i_PrgmRxMsg + REDEIVE_DATA_BYTE_CMD_PARAMETER_SECTION_ONE_4));
                                            OS_FlagPost(&ConfigParametersChangeState, CFG_IGNITE_SECOND_SUCCESSED_FAN_SPD, OS_OPT_POST_FLAG_SET, 0, &err);
                                            break;

                                        default:
                                            break;
                                    }

                                    break;

                                case CONFIG_RUNNING_STATUS_PARA:
                                    switch(*(i_PrgmRxMsg + REDEIVE_DATA_BYTE_CMD_PARAMETER_SECTION_ONE_2)) {
                                        case CONFIG_RUNNING_STATUS_LIQUID_PRESS_EXCEED_4KG_PUMP_SPD:
                                            g_stStartHydrgPumpSpdPara.PumpSpdAfterLiquidPressExceed4Kg = (uint16_t)((*(i_PrgmRxMsg + REDEIVE_DATA_BYTE_CMD_PARAMETER_SECTION_ONE_3) << 8) + * (i_PrgmRxMsg + REDEIVE_DATA_BYTE_CMD_PARAMETER_SECTION_ONE_4));
                                            OS_FlagPost(&ConfigParametersChangeState, CFG_LIQUID_PRESS_EXCEED_4KG_PUMP_SPD, OS_OPT_POST_FLAG_SET, 0, &err);
                                            break;

                                        default:
                                            break;
                                    }

                                    break;

                                case CONFIG_SHUTING_DOWN_STEP_PARA:
                                    break;

                                default:
                                    break;
                            }

                            break;

                        case CONFIG_FUEL_CELL_GROUP_RUNNING_PARA:
                            break;

                        default:
                            break;
                    }

                    //保存参数到flash
                    StoreStartHydrgPumpSpdPara(&g_stStartHydrgPumpSpdPara);
                    StoreStartHydrgFanSpdPara(&g_stStartHydrgFanSpdPara);
                } else {
                    /*机器处于非待机状态*/
                    APP_TRACE_INFO(("Machine not in waiting command status...\r\n"));
                }

                break;
#endif

            case COMMAND_TYPE_INQUIRE_REQUEST://查询、请求指令
                switch(*(i_PrgmRxMsg + REDEIVE_DATA_BYTE_CMD_CODE_VALUE)) {
                    case QUERY_REQUEST_DATA_RETRANSMIT:

                        break;

                    default:
                        break;
                }

                break;

            case COMMAND_TYPE_RESPONSE_CONFIRM://应答、确认指令
                switch(*(i_PrgmRxMsg + REDEIVE_DATA_BYTE_CMD_CODE_VALUE)) {
                    case RESPONSE_ALLOCATE_ID_NMB_WITH_PARAMETERS:
                        break;

                    case RESPONSE_SLAVE_SHUT_DOWN_CMD:
                        u32ErrCode = ((uint32_t) * (i_PrgmRxMsg + REDEIVE_DATA_BYTE_CMD_PARAMETER_SECTION_ONE_1) << 24)
                                     | ((uint32_t) * (i_PrgmRxMsg + REDEIVE_DATA_BYTE_CMD_PARAMETER_SECTION_ONE_2) << 16)
                                     | ((uint32_t) * (i_PrgmRxMsg + REDEIVE_DATA_BYTE_CMD_PARAMETER_SECTION_ONE_3) << 8)
                                     | ((uint32_t) * (i_PrgmRxMsg + REDEIVE_DATA_BYTE_CMD_PARAMETER_SECTION_ONE_4));

                        switch(*(i_PrgmRxMsg + REDEIVE_DATA_BYTE_CMD_PARAMETER_SECTION_ONE_5)) {
                            case RESPONSE_SLAVE_SHUT_DOWN_CMD_RIGHT_NOW:
                                i = 0;

                                while(u32ErrCode) {
                                    if((u32ErrCode % 2) == 1) {
                                        SetShutDownRequestMaskStatu((SYSTEM_ALARM_ADDR_Typedef)i, EN_UN_MASK, 0);//对应的错误屏蔽位清零
                                    }

                                    i++;
                                    u32ErrCode >>= 1;
                                }

//                              BSP_BuzzerOn();
                                CmdShutDown();
                                break;

                            case RESPONSE_SLAVE_SHUT_DOWN_CMD_DELAY:
                                i = 0;

                                while(u32ErrCode) {
                                    if((u32ErrCode % 2) == 1) {
                                        SetShutDownRequestMaskStatu((SYSTEM_ALARM_ADDR_Typedef)i, EN_DELAY, (uint16_t)(*(i_PrgmRxMsg + REDEIVE_DATA_BYTE_CMD_PARAMETER_SECTION_ONE_6) * 60));//对应的错误屏蔽位清零
                                    }

                                    i++;
                                    u32ErrCode >>= 1;
                                }

//                              APP_TRACE_INFO(("Err bit %d, %d...\r\n",i, *(i_PrgmRxMsg + REDEIVE_DATA_BYTE_CMD_PARAMETER_SECTION_ONE_6)));
//                              BSP_BuzzerOn();
                                break;

                            default:
                                break;
                        }

                        break;

                    default:
                        break;
                }

                break;

            default:
                break;
        }
    } else { //否则通信出错
        //接收到位未定义数据
        APP_TRACE_INFO(("Undefined data is received...\r\n"));
    }
}

/*
***************************************************************************************************
*                                      CmdStart()
*
* Description:  execute the "start" command accord to the statu of the system.
*
* Arguments  :  none
*
* Returns    :  none
***************************************************************************************************
*/
void CmdStart(void)
{
    OS_ERR      err;
    SYSTEM_WORK_STATU_Typedef eSysRunningStatu;
    SYSTEM_WORK_MODE_Typedef    eWorkMode;

    eSysRunningStatu = GetSystemWorkStatu();

    if(DEF_OFF == GetHydrgProducerStopDlyStatu() && (DEF_OFF == GetStackStopDlyStatu())) {  //防止关机过程未结束再次开机
        if(eSysRunningStatu == EN_WAITTING_COMMAND) { //等待指令，循环设备自检中
            eWorkMode = GetWorkMode();

            if(eWorkMode != EN_WORK_MODE_FUEL_CELL) {   //制氢或者一体机模式
                OSSchedLock(&err);
                SetSystemWorkStatu(EN_START_PRGM_ONE_FRONT);
            } else {
                OSSchedLock(&err);                                      //发电模式直接为运行状态
                SetSystemWorkStatu(EN_RUNNING);
            }

            OSTaskSemPost(&AppTaskStartTCB,   //send to WaittingCommand函数中的任务信号量请求，用于启动
                          OS_OPT_POST_NO_SCHED,
                          &err);
            OSSchedUnlock(&err);
        } else if(eSysRunningStatu == EN_ALARMING) {

            SetSystemWorkStatu(EN_WAITTING_COMMAND);
            ResetDeviceAlarmStatu();
        } else {

        }
    } else {
        APP_TRACE_INFO(("The last run cycle is not finish, please wait...\r\n"));
    }
}

/*
***************************************************************************************************
*                                      CmdShutDown()
*
* Description:  execute the "shut down" command accord to the statu of the system.
*
* Arguments  :  none
*
* Returns    :  don't printf in this function.
***************************************************************************************************
*/
void CmdShutDown()
{
    OS_ERR      err;
    SYSTEM_WORK_STATU_Typedef eSysRunningStatu;

    eSysRunningStatu = GetSystemWorkStatu();

    if((eSysRunningStatu == EN_START_PRGM_ONE_FRONT)
            || (eSysRunningStatu == EN_START_PRGM_ONE_BEHIND)
            || (eSysRunningStatu == EN_START_PRGM_TWO)
            || (eSysRunningStatu == EN_RUNNING)) {
        OSSchedLock(&err);      //锁任务调度器
        SetSystemWorkStatu(EN_SHUTTING_DOWN);

        switch((uint8_t)eSysRunningStatu) { //switch不支持枚举型变量，将其转为uint8_t型
            case(uint8_t)EN_START_PRGM_ONE_FRONT:

                break;

            case(uint8_t)EN_START_PRGM_ONE_BEHIND:
                OSSemPost(&IgniteFirstBehindWaitSem,    //结束点火
                          OS_OPT_POST_1,
                          &err);
                break;

            case(uint8_t)EN_START_PRGM_TWO:
                break;

            case(uint8_t)EN_RUNNING:
                OSTaskResume(&AppTaskStartTCB,      //恢复开始任务，进入shutdown程序
                             &err);
                break;

            default:
                break;
        }

        OSSchedUnlock(&err);
    }

}

/*
***************************************************************************************************
*                                      SendShutDownRequest()
*
* Description:  send the "shut down" command accord to the statu of the system.
*
* Arguments  :  none
*
* Returns    :  none
***************************************************************************************************
*/
void SendShutDownRequest()
{
    uint32_t u32ErrCode;
    uint8_t  uint8_tCodePara[4];
    u32ErrCode = GetSysErrCode();
    uint8_tCodePara[0] = (uint8_t)(u32ErrCode >> 24);
    uint8_tCodePara[1] = (uint8_t)((u32ErrCode & 0xFF0000) >> 16);
    uint8_tCodePara[2] = (uint8_t)((u32ErrCode & 0xFF00) >> 8);
    uint8_tCodePara[3] = (uint8_t)(u32ErrCode & 0xFF);
    InsertNonRealTimeWorkInfoDataToSendBuff(REAL_TIME_REQUEST_INFORMATION, REAL_TIME_REQUEST_INFO_REQUEST_SHUT_DOWN, 4, uint8_tCodePara);
}

/*
***************************************************************************************************
*                             SendChooseWorkModeRequest()
*
* Description: send choose the work mode request.
*
* Arguments  :  none
*
* Returns    :  none
***************************************************************************************************
*/
void SendChooseWorkModeRequest()
{
    InsertNonRealTimeWorkInfoDataToSendBuff(REAL_TIME_REQUEST_INFORMATION, REAL_TIME_REQUEST_INFO_REQUEST_CHOOSE_WORK_MODE, NULL, NULL);
}

/*
***************************************************************************************************
*                                      SendRealTimeAssistInfo()
*
* Description:  Send real time assist information.
*
* Arguments  :  none
*
* Returns    :  none
***************************************************************************************************
*/
void SendRealTimeAssistInfo()
{
    InsertNonRealTimeWorkInfoDataToSendBuff(REAL_TIME_ASSIST_INFORMATION, NULL, NULL, NULL);
}

/*
***************************************************************************************************
*                                      SendReferenceAndConfigrationInfo()
*
* Description:  Send reference and configration information.
*
* Arguments  :  none
*
* Returns    :  none
***************************************************************************************************
*/
//void SendReferenceAndConfigrationInfo()
//{
//    InsertNonRealTimeWorkInfoDataToSendBuff(FOR_QUERY_AND_CONFIG_INFORMATION, NULL, NULL, NULL);
//}
/*
***************************************************************************************************
*                                      GetPramTxBuffLen()
*
* Description:  get the length of the message that sendout.
*
* Arguments  :  none
*
* Returns    :  length
***************************************************************************************************
*/
uint8_t GetPramTxBuffLen(void)
{
    return PRGM_TX_BUFF_SIZE;
}

/*
***************************************************************************************************
*                                      GetPrgmRxBuffAddr()
*
* Description:  get the address of the message that receive in.
*
* Arguments  :  none
*
* Returns    :  address
***************************************************************************************************
*/
uint8_t *GetPrgmRxBuffAddr(void)
{
    return g_u8SerRxMsgBuff;
}

/*
***************************************************************************************************
*                                      GetPrgmRxBuffLen()
*
* Description:  get the length of the message that receive in.
*
* Arguments  :  none
*
* Returns    :  length
***************************************************************************************************
*/
uint8_t GetPrgmRxBuffLen(void)
{
    return PRGM_RX_BUFF_SIZE;
}



/******************* (C) COPYRIGHT 2016 Guangdong ENECO *****END OF FILE****/
