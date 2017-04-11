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
#include "app_system_run_cfg_parameters.h"
#include "app_wireness_communicate_task.h"
#include "app_top_task.h"
#include "bsp_speed_adjust_device.h"
#include "app_hydrg_producer_manager.h"
#include "app_stack_manager.h"
#include "app_system_run_cfg_parameters.h"
#include "bsp_scale_data_read.h"
#include "bsp_can.h"
/*
***************************************************************************************************
*                                       MACRO DEFINITIONS
***************************************************************************************************
*/
#define COMMUNICATE_TASK_STK_SIZE               128
#define COMMUNICATA_DATA_SEND_TASK_STK_SIZE     128

/*
***************************************************************************************************
*                                         OS-RELATED    VARIABLES
***************************************************************************************************
*/
OS_SEM          g_stCommunicateDataSendSem;
OS_SEM          g_stCommunicateDataSendResponseSem;

OS_TCB          CommunicateTaskTCB;
OS_TCB          CommunicateDataSendTaskTCB;

OS_MUTEX        TxMsgSendBuffWriteMutex;

__attribute__((aligned(8)))  //�����ջ8�ֽڶ��룬ȷ����������ʾ����
static      CPU_STK     CommunicateTaskStk[COMMUNICATE_TASK_STK_SIZE];
static      CPU_STK     CommunicateDataSendTaskStk[COMMUNICATA_DATA_SEND_TASK_STK_SIZE];
/*
***************************************************************************************************
*                                           LOCAL VARIABLES
***************************************************************************************************
*/
uint8_t             g_u8SerRxMsgBuff[PRGM_RX_BUFF_SIZE];
static    uint32_t    g_u32TxMsgDataTagNumber[EN_SEND_DATA_TYPE_MAX] = {0};   //�������ݱ�ǩ��

static      TX_MSG_SEND_BUFF_Typedef        g_stTxMsgDataSendBuff = {0};

/*
***************************************************************************************************
*                                         FUNCTION PROTOTYPES
***************************************************************************************************
*/
static      void        CommunicateTask(void *p_arg);
static      void        CommunicateDataSendTask(void *p_arg);
static      void        ResponsePrgmCommand(uint8_t *);

static      void        LoadNonRealTimeWorkInfo(uint8_t , uint8_t , uint8_t , uint8_t *p_uint8_tParas, uint8_t *);
static      void        AddRealTimeWorkInfoDataToSendBuff(uint8_t , uint8_t);
static      void        LoadHydrogenProducerRealTimeWorkInfo(uint8_t, uint8_t *);
static      void        LoadFuelCellRealTimeWorkInfoPartA(uint8_t, uint8_t *);
static      void        LoadFuelCellRealTimeWorkInfoPartB(uint8_t , uint8_t *);
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

    BSP_SerToWIFI_Init();       //(WIFI)���ڳ�ʼ��

    BSP_SerToWIFI_RxMsgInit(g_u8SerRxMsgBuff, PRGM_RX_BUFF_SIZE);   //�������ݵ�ַ��ʼ��

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
    uint8_t     i = 0;
    static      uint8_t st_SendMegGroupNum = 0;

    while(DEF_TRUE) {

        OSTimeDlyHMSM(0, 0, 0, 300,
                      OS_OPT_TIME_HMSM_STRICT,
                      &err);

        if(EN_WORK_MODE_HYDROGEN_PRODUCER_AND_FUEL_CELL == GetWorkMode()) {

            st_SendMegGroupNum ++;//���ݷֿ�3�鷢��,0.3S��һ��

            if(st_SendMegGroupNum == GROUP_ONE) {
                AddRealTimeWorkInfoDataToSendBuff(EN_LATEST, REAL_TIME_RUNNING_INFORMATION_A);//������
            } else if(st_SendMegGroupNum == GROUP_TWO) {
                AddRealTimeWorkInfoDataToSendBuff(EN_LATEST, REAL_TIME_RUNNING_INFORMATION_B_1);
            } else if(st_SendMegGroupNum == GROUP_THERE) {
                AddRealTimeWorkInfoDataToSendBuff(EN_LATEST, REAL_TIME_RUNNING_INFORMATION_B_2);
                st_SendMegGroupNum = 0;//����ˢ�������
            } else {}
        }

        if(g_u8WifiCommandReceived == YES) { //�յ�����ָ�����ǰ������ʱ����Ӧ��λ��ָ��
            ResponsePrgmCommand(g_u8SerRxMsgBuff);
            g_u8WifiCommandReceived = NO;
        } else if(g_eCanMsgRxStatu == YES) { //�Ƿ����յ�CAN�ӿ�ָ�����ǰ������ʱ
            ResponsePrgmCommand(g_u8CanRxMsg);
            g_eCanMsgRxStatu = NO;
            APP_TRACE_INFO(("Can Rx data:"));

            for(i = 0; i < 16; i++) {
                APP_TRACE_INFO(("%X ", g_u8CanRxMsg[i]));
            }

            APP_TRACE_INFO(("...\n\r"));
        } else {
            //������ʱ
        }

        //�����ź���������һ�����ݴ���
        OSSemPost(&g_stCommunicateDataSendSem,
                  OS_OPT_POST_1,
                  &err);
    }
}

/*
***************************************************************************************************
*                               AddRealTimeWorkInfoDataToSendBuff()
*
* Description :���ݷ�������
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
            //һ֡һ֡�ķ�������,��ʱ�������ı�֡�ĳ��ȣ����շ��ĳ����ǹ̶��ģ���Ҫ������Ҫ��֡���ݰ��м���һ��֡����
            SendAPrgmMsgFrame(PRGM_TX_BUFF_SIZE, g_stTxMsgDataSendBuff.Queue[g_stTxMsgDataSendBuff.Q_Qhead].Data);

            OSSemPend(&g_stCommunicateDataSendResponseSem,
                      OS_CFG_TICK_RATE_HZ / 5,//���֮һ��
                      OS_OPT_PEND_BLOCKING,
                      NULL,
                      &err);

            if(err == OS_ERR_NONE) { //��������
//              APP_TRACE_INFO(("Send statu normal...\r\n"));
            } else { //�洢����ʷ��¼�洢����
                APP_TRACE_INFO(("Send statu err...\r\n"));
            }

            OSMutexPend(&TxMsgSendBuffWriteMutex,
                        OS_CFG_TICK_RATE_HZ / 10,
                        OS_OPT_PEND_BLOCKING,
                        NULL,
                        &err);
            g_stTxMsgDataSendBuff.Q_length--;
            g_stTxMsgDataSendBuff.Queue[g_stTxMsgDataSendBuff.Q_Qhead].OccupyStatu = EN_FREE;
            //ԭ���ĵڶ��ڵ��Ϊ�µ�ͷ�ڵ�
            g_stTxMsgDataSendBuff.Q_Qhead = g_stTxMsgDataSendBuff.Queue[g_stTxMsgDataSendBuff.Q_Qhead].D_next;
            //ԭ����ͷ�ڵ����ָ��ָ���Լ������µ�ͷ�ڵ��ǰһ�ڵ�
            g_stTxMsgDataSendBuff.Queue[g_stTxMsgDataSendBuff.Queue[g_stTxMsgDataSendBuff.Q_Qhead].D_last].D_next = g_stTxMsgDataSendBuff.Queue[g_stTxMsgDataSendBuff.Q_Qhead].D_last;
            g_stTxMsgDataSendBuff.Queue[g_stTxMsgDataSendBuff.Q_Qhead].D_last = g_stTxMsgDataSendBuff.Q_Qhead;//��ͷ�ڵ��ǰ����ָ���Լ�
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
* Description :׷��ʵʱ�������������ͻ�����
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

    if(g_stTxMsgDataSendBuff.Q_length == 0) { //������û����Ϣ������
        g_stTxMsgDataSendBuff.Q_length++;
        g_stTxMsgDataSendBuff.Q_Qhead = 0;
        g_stTxMsgDataSendBuff.Q_Qrear = 0;
        g_stTxMsgDataSendBuff.Queue[g_stTxMsgDataSendBuff.Q_Qrear].D_last = g_stTxMsgDataSendBuff.Q_Qhead;
        g_stTxMsgDataSendBuff.Queue[g_stTxMsgDataSendBuff.Q_Qrear].D_next = g_stTxMsgDataSendBuff.Q_Qrear;
        g_stTxMsgDataSendBuff.Queue[g_stTxMsgDataSendBuff.Q_Qrear].OccupyStatu = EN_WAITTING;

        if(REAL_TIME_RUNNING_INFORMATION_A == i_uint8_RealTimeWorkInfoType) {
            LoadHydrogenProducerRealTimeWorkInfo(i_uint8_tIsLatestData, g_stTxMsgDataSendBuff.Queue[g_stTxMsgDataSendBuff.Q_Qrear].Data);
        } else if(REAL_TIME_RUNNING_INFORMATION_B_1 == i_uint8_RealTimeWorkInfoType) {
            LoadFuelCellRealTimeWorkInfoPartA(i_uint8_tIsLatestData, g_stTxMsgDataSendBuff.Queue[g_stTxMsgDataSendBuff.Q_Qrear].Data);
        } else if(REAL_TIME_RUNNING_INFORMATION_B_2 == i_uint8_RealTimeWorkInfoType) {
            LoadFuelCellRealTimeWorkInfoPartB(i_uint8_tIsLatestData, g_stTxMsgDataSendBuff.Queue[g_stTxMsgDataSendBuff.Q_Qrear].Data);
        } else {}

    } else if(++g_stTxMsgDataSendBuff.Q_length <= TX_MSG_SEND_QUEUE_SIZE) {
        //�ӻ�����ͷ��Ѱ���нڵ�
        for(i = 0; i < TX_MSG_SEND_QUEUE_SIZE; i++) {
            if(g_stTxMsgDataSendBuff.Queue[i].OccupyStatu == EN_FREE) {
                g_stTxMsgDataSendBuff.Queue[g_stTxMsgDataSendBuff.Q_Qrear].D_next = i;//��ǰ��β�ڵ������ָ����Ѱ���Ŀսڵ�
                g_stTxMsgDataSendBuff.Queue[i].D_last = g_stTxMsgDataSendBuff.Q_Qrear;//���ٵ��½ڵ��ǰ����ָ��ԭ����β�ڵ�
                g_stTxMsgDataSendBuff.Q_Qrear = i;//β�ڵ���µ��µĽڵ�
                g_stTxMsgDataSendBuff.Queue[i].OccupyStatu = EN_WAITTING;//�½ڵ��״̬����Ϊ�ȴ�����

                if(REAL_TIME_RUNNING_INFORMATION_A == i_uint8_RealTimeWorkInfoType) {
                    LoadHydrogenProducerRealTimeWorkInfo(i_uint8_tIsLatestData, g_stTxMsgDataSendBuff.Queue[g_stTxMsgDataSendBuff.Q_Qrear].Data);
                } else if(REAL_TIME_RUNNING_INFORMATION_B_1 == i_uint8_RealTimeWorkInfoType) {
                    LoadFuelCellRealTimeWorkInfoPartA(i_uint8_tIsLatestData, g_stTxMsgDataSendBuff.Queue[g_stTxMsgDataSendBuff.Q_Qrear].Data);
                } else if(REAL_TIME_RUNNING_INFORMATION_B_2 == i_uint8_RealTimeWorkInfoType) {
                    LoadFuelCellRealTimeWorkInfoPartB(i_uint8_tIsLatestData, g_stTxMsgDataSendBuff.Queue[g_stTxMsgDataSendBuff.Q_Qrear].Data);
                } else {}

                break;
            }
        }
    } else {
        --g_stTxMsgDataSendBuff.Q_length;
        APP_TRACE_INFO(("The real time sends message buffer is overflow...\r\n"));
    }

    OSMutexPost(&TxMsgSendBuffWriteMutex,       //�ͷ�д���ͻ�����������
                OS_OPT_POST_NONE,
                &err);
}

/*
***************************************************************************************************
*                                      LoadHydrogenProducerRealTimeWorkInfo()
*
* Description:  ���������ʵʱ��������,��ǰ��,����ȡ����ʷ��¼.
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
    uint16_t    u16HydrogenGasConcentration = 0;
    uint16_t    u16VacuumNetativePressure = 0;
    uint16_t    u16LiquidLevel = 0;
    uint16_t    u16LiquidFeedPerMinute = 0;
    uint16_t    u16PumpFeedbackSpeed = 0, u16PumpCtlSpeed = 0;
    uint16_t    u16HydrgFanCtlSpd = 0, u16HydrgFeedbackFanSpd = 0;
//    uint16_t    u16FluidWeightPerMinuteMul100 = 0;
    uint16_t    m_u16LiquidPress = 0;
    uint32_t    u32AlarmCode = 0;
    uint32_t    u32SystemRunningStatuCode = 0;
    SYSTEM_TIME_Typedef     stHydrgProduceTimeThisTime = {0}, stHydrgProduceTimeTotal = {0};

    if(i_uint8_tIsHistoryData == EN_LATEST) { //���µ�����
//        APP_TRACE_INFO(("Load Hydrogen Producer real time work info...\r\n"));
        //���ݱ�ͷ��
        *(i_pRealTimeWorkInfo + HEAD_BYTE_ONE) = 0xF1;
        *(i_pRealTimeWorkInfo + HEAD_BYTE_TWO) = 0xF2;
        *(i_pRealTimeWorkInfo + HEAD_BYTE_THREE) = 0xF3;
        *(i_pRealTimeWorkInfo + PRODUCTS_TYPE_ID_HIGH) = (uint8_t)((PRODUCT_MODEL_CODE & 0xFF00) >>  8);
        *(i_pRealTimeWorkInfo + PRODUCTS_TYPE_ID_LOW) = (uint8_t)(PRODUCT_MODEL_CODE & 0xFF);
        *(i_pRealTimeWorkInfo + LOCAL_NETWORK_ID_CODE) = g_u16GlobalNetWorkId;

        *(i_pRealTimeWorkInfo + INFORMATION_TYPE_CONTROL_CODE) = (uint8_t)REAL_TIME_RUNNING_INFORMATION_A;
        *(i_pRealTimeWorkInfo + VALID_INFORMATION_LENGTH_CONTROL_CODE) = REAL_TIME_RUNNING_INFO_A_LENGTH;

        //���ݱ�ǩ����ֵ����
        g_u32TxMsgDataTagNumber[REAL_TIME_RUNNING_INFORMATION_A]++;
        *(i_pRealTimeWorkInfo + DATA_IDENTIFY_TAG_INF_CODE_1) = (uint8_t)(g_u32TxMsgDataTagNumber[REAL_TIME_RUNNING_INFORMATION_A] >> 24);
        *(i_pRealTimeWorkInfo + DATA_IDENTIFY_TAG_INF_CODE_2) = (uint8_t)((g_u32TxMsgDataTagNumber[REAL_TIME_RUNNING_INFORMATION_A] & 0xFF0000) >> 16);
        *(i_pRealTimeWorkInfo + DATA_IDENTIFY_TAG_INF_CODE_3) = (uint8_t)((g_u32TxMsgDataTagNumber[REAL_TIME_RUNNING_INFORMATION_A] & 0xFF00) >> 8);
        *(i_pRealTimeWorkInfo + DATA_IDENTIFY_TAG_INF_CODE_4) = (uint8_t)(g_u32TxMsgDataTagNumber[REAL_TIME_RUNNING_INFORMATION_A] & 0xFF);

        //���о�����
        u32AlarmCode = GetRunAlarmCode();
        *(i_pRealTimeWorkInfo + RUN_ALARM_CODE_BYTE_4) = (uint8_t)(u32AlarmCode >> 24);
        *(i_pRealTimeWorkInfo + RUN_ALARM_CODE_BYTE_3) = (uint8_t)((u32AlarmCode & 0xFF0000) >> 16);
        *(i_pRealTimeWorkInfo + RUN_ALARM_CODE_BYTE_2) = (uint8_t)((u32AlarmCode & 0xFF00) >> 8);
        *(i_pRealTimeWorkInfo + RUN_ALARM_CODE_BYTE_1) = (uint8_t)(u32AlarmCode & 0xFF);
        //����״̬��
        u32SystemRunningStatuCode = GetSystemRunningStatuCode();
        *(i_pRealTimeWorkInfo + RUNNING_STATU_CODE_BYTE_4) = (uint8_t)(u32SystemRunningStatuCode >> 24);
        *(i_pRealTimeWorkInfo + RUNNING_STATU_CODE_BYTE_3) = (uint8_t)((u32SystemRunningStatuCode & 0xFF0000) >> 16);
        *(i_pRealTimeWorkInfo + RUNNING_STATU_CODE_BYTE_2) = (uint8_t)((u32SystemRunningStatuCode & 0xFF00) >> 8);
        *(i_pRealTimeWorkInfo + RUNNING_STATU_CODE_BYTE_1) = (uint8_t)(u32SystemRunningStatuCode & 0xFF);
        //�����¶�
        u16CurrentTemp = (uint16_t)GetReformerTemp();
        *(i_pRealTimeWorkInfo + REFORMER_TEMP_HIGH) = (uint8_t)((u16CurrentTemp & 0xFF00) >> 8);
        *(i_pRealTimeWorkInfo + REFORMER_TEMP_LOW) = (uint8_t)((u16CurrentTemp & 0xFF));
        //�����¶�
        u16CurrentTemp = (uint16_t)GetFireOrRodTemp();
        *(i_pRealTimeWorkInfo + FIRE_OR_ROD_TEMP_HIGH) = (uint8_t)((u16CurrentTemp & 0xFF00) >> 8);
        *(i_pRealTimeWorkInfo + FIRE_OR_ROD_TEMP_LOW) = (uint8_t)((u16CurrentTemp & 0xFF));
        //Һѹ
        m_u16LiquidPress = (uint16_t)(GetSrcAnaSig(LIQUID_PRESS) * 100);
        *(i_pRealTimeWorkInfo + LIQUID_PRESS_INTEGER_PART) = (uint8_t)(m_u16LiquidPress / 100);
        *(i_pRealTimeWorkInfo + LIQUID_PRESS_DECIMAL_PART) = (uint8_t)(m_u16LiquidPress % 100);
        //��������ٶ�
        u16HydrgFanCtlSpd = GetHydrgFanCurrentCtlSpd();
        *(i_pRealTimeWorkInfo + HYDROGEN_FAN_SPD_CONTROL_HIGH) = (uint8_t)((u16HydrgFanCtlSpd & 0xFF00) >> 8);
        *(i_pRealTimeWorkInfo + HYDROGEN_FAN_SPD_CONTROL_LOW) = (uint8_t)((u16HydrgFanCtlSpd & 0xFF));
        //��������ٶ�
        u16HydrgFeedbackFanSpd = GetHydrgFanFeedBackSpd();
        *(i_pRealTimeWorkInfo + HYDROGEN_FAN_SPD_FEEDBACK_HIGH) = (uint8_t)((u16HydrgFeedbackFanSpd & 0xFF00) >> 8);
        *(i_pRealTimeWorkInfo + HYDROGEN_FAN_SPD_FEEDBACK_LOW) = (uint8_t)((u16HydrgFeedbackFanSpd & 0xFF));
        //ˮ�ÿ����ٶ�
        u16PumpCtlSpeed = GetPumpCtlSpd();
        *(i_pRealTimeWorkInfo + PUMP_SPD_CONTROL_HIGH) = (uint8_t)((u16PumpCtlSpeed & 0xFF00) >> 8);
        *(i_pRealTimeWorkInfo + PUMP_SPD_CONTROL_LOW) = (uint8_t)(u16PumpCtlSpeed & 0xFF);
        //ˮ�÷����ٶ�
        u16PumpFeedbackSpeed = GetPumpFeedBackSpd();
        *(i_pRealTimeWorkInfo + PUMP_SPD_FEEDBACK_HIGH) = (uint8_t)((u16PumpFeedbackSpeed & 0xFF00) >> 8);
        *(i_pRealTimeWorkInfo + PUMP_SPD_FEEDBACK_LOW) = (uint8_t)(u16PumpFeedbackSpeed & 0xFF);

        //��������ʱ��,APP����������ʾ���������ֵ
        stHydrgProduceTimeThisTime = GetHydrgProduceTimeThisTime();
        //  APP_TRACE_INFO(("AAAAA:%d%d\r\nMinut:%d\r\nSecond:%d\r\n",(uint8_t)((stHydrgProduceTimeThisTime.hour & 0xFF00) >> 8),(uint8_t)((stHydrgProduceTimeThisTime.hour & 0xFF)), stHydrgProduceTimeThisTime.minute,stHydrgProduceTimeThisTime.second));
        *(i_pRealTimeWorkInfo + HYDROGEN_PRODUCT_TIME_THIS_TIME_HOUR_HIGH) = (uint8_t)((stHydrgProduceTimeThisTime.hour & 0xFF00) >> 8);
        *(i_pRealTimeWorkInfo + HYDROGEN_PRODUCT_TIME_THIS_TIME_HOUR_LOW) = (uint8_t)(stHydrgProduceTimeThisTime.hour & 0xFF);
        *(i_pRealTimeWorkInfo + HYDROGEN_PRODUCT_TIME_THIS_TIME_MINUTE) = stHydrgProduceTimeThisTime.minute;
        *(i_pRealTimeWorkInfo + HYDROGEN_PRODUCT_TIME_THIS_TIME_SECOND) = stHydrgProduceTimeThisTime.second;
        //�ۼ�����ʱ��
        stHydrgProduceTimeTotal = GetHydrgProduceTimeThisTime();//���Ա�������ʱ�����
        *(i_pRealTimeWorkInfo + HYDROGEN_PRODUCT_TIME_TOTAL_HOUR_HIGH) = (uint8_t)((stHydrgProduceTimeTotal.hour & 0xFF00) >> 8);
        *(i_pRealTimeWorkInfo + HYDROGEN_PRODUCT_TIME_TOTAL_HOUR_LOW) = (uint8_t)(stHydrgProduceTimeTotal.hour & 0xFF);
        *(i_pRealTimeWorkInfo + HYDROGEN_PRODUCT_TIME_TOTAL_MINUTE) = stHydrgProduceTimeTotal.minute;
        *(i_pRealTimeWorkInfo + HYDROGEN_PRODUCT_TIME_TOTAL_SECOND) = stHydrgProduceTimeTotal.second;
        //�ۼ��������
        u16HydrgWorkTimes = GetHydrgProducerWorkTimes();
        *(i_pRealTimeWorkInfo + HYDROGEN_PRODUCT_TOTAL_TIMES_HIGH) = (uint8_t)((u16HydrgWorkTimes & 0xFF00) >> 8);
        *(i_pRealTimeWorkInfo + HYDROGEN_PRODUCT_TOTAL_TIMES_LOW) = (uint8_t)(u16HydrgWorkTimes & 0xFF);

        //Һλ����
        u16LiquidLevel = (uint16_t)GetSrcAnaSig(LIQUID_LEVEL);
//        u16LiquidLevel = 235;
        *(i_pRealTimeWorkInfo + LIQUID_LEVEL_INTEGER_PART) = (uint8_t)((u16LiquidLevel & 0xFF00) >> 8);
        *(i_pRealTimeWorkInfo + LIQUID_LEVEL_DECIMAL_PART) = (uint8_t)(u16LiquidLevel & 0xFF);
        //ÿ���ӽ�Һ��
//        u16LiquidFeedPerMinute = (uint16_t)(ReadLiquidFlowRate() * 100);
        u16LiquidFeedPerMinute = 0;
        *(i_pRealTimeWorkInfo + LIQUID_FEED_PER_MINUTE_INTEGER_PART) = (uint8_t)(u16LiquidFeedPerMinute / 100);
        *(i_pRealTimeWorkInfo + LIQUID_FEED_PER_MINUTE_DECIMAL_PART) = (uint8_t)(u16LiquidFeedPerMinute % 100);

        //��ո�ѹ
//        u16VacuumNetativePressure = (uint16_t)(GetSrcAnaSig(NEGATIVE_PRESSURE) * 100);
        u16VacuumNetativePressure = 0;
        *(i_pRealTimeWorkInfo + VACUUM_NEGATIVE_PRESSURE_HIGH) = (uint8_t)(u16VacuumNetativePressure / 100);
        *(i_pRealTimeWorkInfo + VACUUM_NEGATIVE_PRESSURE_LOW) = (uint8_t)(u16VacuumNetativePressure % 100);

        //����Ũ��
//        u16HydrogenGasConcentration = (uint16_t)(GetSrcAnaSig(HYDROGEN_CONCENTRATION) * 100);
        u16HydrogenGasConcentration = 0;
        *(i_pRealTimeWorkInfo + HYDROGEN_PRODUCT_GAS_CONCENTRATION_INTEGER_PART) = (uint8_t)(u16HydrogenGasConcentration / 100);
        *(i_pRealTimeWorkInfo + HYDROGEN_PRODUCT_GAS_CONCENTRATION_DECIMAL_PART) = (uint8_t)(u16HydrogenGasConcentration % 100);

        //��ģ��ID��
//        *(i_pRealTimeWorkInfo + SUB_MODULE_ID_OF_THE_MULTI_MODULE_TYPE) = SUB_MODULE_ID;
        //���ݱ�β��
        *(i_pRealTimeWorkInfo + END_BYTE_ONE) = 0x5F;
        *(i_pRealTimeWorkInfo + END_BYTE_TWO) = 0x6F;

    } else { //������ʷ����

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
    uint16_t  u16StackFanSpdFeedBackB = 0;
    uint16_t  u16StackFanCtlSpd = 0;
    uint32_t  u32StackPower = 0;
    uint32_t  u32AlarmCode = 0;
    uint16_t  m_u16StackCurrentMul100 = 0;
    uint16_t  m_u16StackVoltageMul100 = 0;
    uint16_t  m_u16StackPressMul100 = 0;
    uint32_t  u32IsolatedGenratedEnergyThisTime = 0;
//    uint32_t  u32IsolatedGenratedEnergyCount = 0;
    int16_t   i16HydrogYieldMatchOffsetValue = 0;
    uint32_t  u32SystemWorkStatuCode = 0;
    SYSTEM_TIME_Typedef     stStackWorkTimeThisTime = {0}, stStackWorkTimeTotal = {0};

    if(i_uint8_tIsHistoryData == EN_LATEST) { //���µ�����
//        APP_TRACE_INFO(("Load Fuel cell real time part A work info...\r\n"));
        //���ݱ�ͷ��
        *(i_pRealTimeWorkInfo + HEAD_BYTE_ONE) = 0xF1;
        *(i_pRealTimeWorkInfo + HEAD_BYTE_TWO) = 0xF2;
        *(i_pRealTimeWorkInfo + HEAD_BYTE_THREE) = 0xF3;
        *(i_pRealTimeWorkInfo + PRODUCTS_TYPE_ID_HIGH) = (uint8_t)(PRODUCT_MODEL_CODE >> 8);
        *(i_pRealTimeWorkInfo + PRODUCTS_TYPE_ID_LOW) = (uint8_t)(PRODUCT_MODEL_CODE & 0xFF);
        *(i_pRealTimeWorkInfo + LOCAL_NETWORK_ID_CODE) = g_u16GlobalNetWorkId;

        *(i_pRealTimeWorkInfo + INFORMATION_TYPE_CONTROL_CODE) = (uint8_t)REAL_TIME_RUNNING_INFORMATION_B_1;
        *(i_pRealTimeWorkInfo + VALID_INFORMATION_LENGTH_CONTROL_CODE) = REAL_TIME_RUNNING_INFO_B_1_LENGTH;

        //���ݱ�ǩ����ֵ����
        g_u32TxMsgDataTagNumber[REAL_TIME_RUNNING_INFORMATION_B_1]++;
        *(i_pRealTimeWorkInfo + DATA_IDENTIFY_TAG_INF_CODE_1) = (uint8_t)(g_u32TxMsgDataTagNumber[REAL_TIME_RUNNING_INFORMATION_B_1] >> 24);
        *(i_pRealTimeWorkInfo + DATA_IDENTIFY_TAG_INF_CODE_2) = (uint8_t)((g_u32TxMsgDataTagNumber[REAL_TIME_RUNNING_INFORMATION_B_1] & 0xFF0000) >> 16);
        *(i_pRealTimeWorkInfo + DATA_IDENTIFY_TAG_INF_CODE_3) = (uint8_t)((g_u32TxMsgDataTagNumber[REAL_TIME_RUNNING_INFORMATION_B_1] & 0xFF00) >> 8);
        *(i_pRealTimeWorkInfo + DATA_IDENTIFY_TAG_INF_CODE_4) = (uint8_t)(g_u32TxMsgDataTagNumber[REAL_TIME_RUNNING_INFORMATION_B_1] & 0xFF);

        //���о�����
        u32AlarmCode = GetRunAlarmCode();
        *(i_pRealTimeWorkInfo + RUN_ALARM_CODE_BYTE_4) = (uint8_t)(u32AlarmCode >> 24);
        *(i_pRealTimeWorkInfo + RUN_ALARM_CODE_BYTE_3) = (uint8_t)((u32AlarmCode & 0xFF0000) >> 16);
        *(i_pRealTimeWorkInfo + RUN_ALARM_CODE_BYTE_2) = (uint8_t)((u32AlarmCode & 0xFF00) >> 8);
        *(i_pRealTimeWorkInfo + RUN_ALARM_CODE_BYTE_1) = (uint8_t)(u32AlarmCode & 0xFF);

        //����״̬��
        u32SystemWorkStatuCode = GetSystemRunningStatuCode();
        *(i_pRealTimeWorkInfo + RUNNING_STATU_CODE_BYTE_4) = (uint8_t)(u32SystemWorkStatuCode >> 24);
        *(i_pRealTimeWorkInfo + RUNNING_STATU_CODE_BYTE_3) = (uint8_t)((u32SystemWorkStatuCode & 0xFF0000) >> 16);
        *(i_pRealTimeWorkInfo + RUNNING_STATU_CODE_BYTE_2) = (uint8_t)((u32SystemWorkStatuCode & 0xFF00) >> 8);
        *(i_pRealTimeWorkInfo + RUNNING_STATU_CODE_BYTE_1) = (uint8_t)(u32SystemWorkStatuCode & 0xFF);

        //��ѵ���*100
        m_u16StackCurrentMul100 = (uint16_t)(GetSrcAnaSig(STACK_CURRENT) * 100);
        *(i_pRealTimeWorkInfo + STACK_CURRENT_INTEGER_PART) = (uint8_t)(m_u16StackCurrentMul100 / 100);
        *(i_pRealTimeWorkInfo + STACK_CURRENT_DECIMAL_PART) = (uint8_t)(m_u16StackCurrentMul100 % 100);

        //��ѵ�ѹ*100
        m_u16StackVoltageMul100 = (uint16_t)(GetSrcAnaSig(STACK_VOLTAGE) * 100);
        *(i_pRealTimeWorkInfo + STACK_VOLTAGE_INTEGER_PART) = (uint8_t)(m_u16StackVoltageMul100 / 100);
        *(i_pRealTimeWorkInfo + STACK_VOLTAGE_DECIMAL_PART) = (uint8_t)(m_u16StackVoltageMul100 % 100);

        //����¶�,��ʱֻ����������
        u16StackTemp = (uint16_t)GetSrcAnaSig(STACK_TEMP);
        *(i_pRealTimeWorkInfo + STACK_TEMP_INTEGER_PART) = (uint8_t)(u16StackTemp & 0xFF);
//      *(i_pRealTimeWorkInfo + STACK_TEMP_INTEGER_PART) = (uint8_t)((u32SystemWorkStatuCode & 0xFF00) >> 8);
//      *(i_pRealTimeWorkInfo + STACK_TEMP_DECIMAL_PART) = (uint8_t)(u32SystemWorkStatuCode & 0xFF);

        //�������ѹ��*100
        m_u16StackPressMul100 = (uint16_t)(GetSrcAnaSig(HYDROGEN_PRESS_1) * 100);
        *(i_pRealTimeWorkInfo + STACK_HYDROGEN_PRESS_INTEGER_PART) = (uint8_t)(m_u16StackPressMul100 / 100);
        *(i_pRealTimeWorkInfo + STACK_HYDROGEN_PRESS_DECIMAL_PART) = (uint8_t)(m_u16StackPressMul100 % 100);

        //��ѷ��ȿ����ٶ�
        u16StackFanCtlSpd = GetStackFanCtlSpd();
        *(i_pRealTimeWorkInfo + STACK_FAN_SPD_CONTROL_HIGH) = (uint8_t)((u16StackFanCtlSpd & 0xFF00) >> 8);
        *(i_pRealTimeWorkInfo + STACK_FAN_SPD_CONTROL_LOW) = (uint8_t)(u16StackFanCtlSpd & 0xFF);

        //��ѷ��ȷ����ٶ�
        u16StackFanSpdFeedBack = GetStackFanSpdFeedBack();
        *(i_pRealTimeWorkInfo + STACK_FAN_PART_A_SPD_FEEDBACK_HIGH) = (uint8_t)((u16StackFanSpdFeedBack & 0xFF00) >> 8);
        *(i_pRealTimeWorkInfo + STACK_FAN_PART_A_SPD_FEEDBACK_LOW) = (uint8_t)(u16StackFanSpdFeedBack & 0xFF);

        u16StackFanSpdFeedBackB = 0;
        *(i_pRealTimeWorkInfo + STACK_FAN_PART_B_SPD_FEEDBACK_HIGH) = (uint8_t)((u16StackFanSpdFeedBackB & 0xFF00) >> 8);
        *(i_pRealTimeWorkInfo + STACK_FAN_PART_B_SPD_FEEDBACK_LOW) = (uint8_t)(u16StackFanSpdFeedBackB & 0xFF);

        //���η���ʱ��,APP�Ϸ��������ʾ�������ֵ
        stStackWorkTimeThisTime = GetStackProductTimeThisTime();
        *(i_pRealTimeWorkInfo + STACK_WORK_TIME_THIS_TIME_HOUR_HIGH) = (uint8_t)((stStackWorkTimeThisTime.hour & 0xFF00) >> 8);
        *(i_pRealTimeWorkInfo + STACK_WORK_TIME_THIS_TIME_HOUR_LOW) = (uint8_t)(stStackWorkTimeThisTime.hour & 0xFF);
        *(i_pRealTimeWorkInfo + STACK_WORK_TIME_THIS_TIME_MINUTE) = (uint8_t)(stStackWorkTimeThisTime.minute);
        *(i_pRealTimeWorkInfo + STACK_WORK_TIME_THIS_TIME_SECOND) = (uint8_t)(stStackWorkTimeThisTime.second);

        //�ۼƷ���ʱ��
        stStackWorkTimeTotal = GetStackProductTimeTotal();
        *(i_pRealTimeWorkInfo + STACK_WORK_TIME_TOTAL_HOUR_HIGH) = (uint8_t)((stStackWorkTimeTotal.hour & 0xFF00) >> 8);
        *(i_pRealTimeWorkInfo + STACK_WORK_TIME_TOTAL_HOUR_LOW) = (uint8_t)(stStackWorkTimeTotal.hour & 0xFF);
        *(i_pRealTimeWorkInfo + STACK_WORK_TIME_TOTAL_MINUTE) = (uint8_t)(stStackWorkTimeTotal.minute);
        *(i_pRealTimeWorkInfo + STACK_WORK_TIME_TOTAL_SECOND) = (uint8_t)(stStackWorkTimeTotal.second);

        //��ǰ�������
        u32StackPower = (uint32_t)(GetCurrentPower() * 100);
        *(i_pRealTimeWorkInfo + CURRENT_ISOLATED_POWER_INTEGER_PART_HIGH) = (uint8_t)(((uint16_t)(u32StackPower / 100) & 0xFF00) >> 8);
        *(i_pRealTimeWorkInfo + CURRENT_ISOLATED_POWER_INTEGER_PART_LOW) = (uint8_t)((uint16_t)(u32StackPower / 100) & 0xFF);
        *(i_pRealTimeWorkInfo + CURRENT_ISOLATED_POWER_DECIMAL_PART) = (uint8_t)(u32StackPower % 100);

        //���ε���������
        u32IsolatedGenratedEnergyThisTime = (uint32_t)(GetIsolatedGenratedEnergyThisTime() * 1000);
//        u32IsolatedGenratedEnergyThisTime = 635423;
        *(i_pRealTimeWorkInfo + ISOLATED_GENERATED_ENERGY_THIE_TIME_INTEGER_PART_HIGH) = (uint8_t)(((uint16_t)(u32IsolatedGenratedEnergyThisTime / 1000) & 0xFF00) >> 8);
        *(i_pRealTimeWorkInfo + ISOLATED_GENERATED_ENERGY_THIE_TIME_INTEGER_PART_LOW) = (uint8_t)((uint16_t)(u32IsolatedGenratedEnergyThisTime / 1000) & 0xFF);
        *(i_pRealTimeWorkInfo + ISOLATED_GENERATED_ENERGY_THIE_TIME_DECIMAL_PART_HIGH) = (uint8_t)(((uint16_t)(u32IsolatedGenratedEnergyThisTime % 1000) & 0xFF00) >> 8);
        *(i_pRealTimeWorkInfo + ISOLATED_GENERATED_ENERGY_THIE_TIME_DECIMAL_PART_LOW) = (uint8_t)((uint16_t)(u32IsolatedGenratedEnergyThisTime % 1000) & 0xFF);

//        u32IsolatedGenratedEnergyCount = 552341;
//        *(i_pRealTimeWorkInfo + GENERATED_ENERGY_TOTAL_INTEGER_PART_HIGH) = (uint8_t)(((uint16_t)(u32IsolatedGenratedEnergyCount / 1000) & 0xFF00) >> 8);
//        *(i_pRealTimeWorkInfo + GENERATED_ENERGY_TOTAL_INTEGER_PART_LOW) = (uint8_t)((uint16_t)(u32IsolatedGenratedEnergyCount / 1000) & 0xFF);
//        *(i_pRealTimeWorkInfo + GENERATED_ENERGY_TOTAL_DECIMAL_PART_HIGH) = (uint8_t)(((uint16_t)(u32IsolatedGenratedEnergyCount % 1000) & 0xFF00) >> 8);
//        *(i_pRealTimeWorkInfo + GENERATED_ENERGY_TOTAL_DECIMAL_PART_LOW) = (uint8_t)((uint16_t)(u32IsolatedGenratedEnergyCount % 1000) & 0xFF);

        //ƥ��ƫ��ֵ
        i16HydrogYieldMatchOffsetValue = (int16_t)(GetStackHydrogenYieldMatchOffsetValue() * 100);

        if(i16HydrogYieldMatchOffsetValue > 0 || i16HydrogYieldMatchOffsetValue < -100) { //����λ������λ
            *(i_pRealTimeWorkInfo + HYDROGEN_YIELD_MATCHING_OFFSET_VALUE_INTEGER_PART_MUL100) = (int8_t)(i16HydrogYieldMatchOffsetValue / 100);
            *(i_pRealTimeWorkInfo + HYDROGEN_YIELD_MATCHING_OFFSET_VALUE_DECIMAL_PART_MUL100_HIGH) = (uint8_t)(((i16HydrogYieldMatchOffsetValue % 100)) & 0xFF);
        } else { //С��λ������λ
            *(i_pRealTimeWorkInfo + HYDROGEN_YIELD_MATCHING_OFFSET_VALUE_INTEGER_PART_MUL100) = (uint8_t)(i16HydrogYieldMatchOffsetValue / 100);
            *(i_pRealTimeWorkInfo + HYDROGEN_YIELD_MATCHING_OFFSET_VALUE_DECIMAL_PART_MUL100_HIGH) = (int8_t)((i16HydrogYieldMatchOffsetValue % 100) & 0xFF);
        }

        //��ģ��ID��
//        *(i_pRealTimeWorkInfo + SUB_MODULE_ID_OF_THE_MULTI_MODULE_TYPE) = SUB_MODULE_ID;

        //���ݱ�β��
        *(i_pRealTimeWorkInfo + END_BYTE_ONE) = 0x5F;
        *(i_pRealTimeWorkInfo + END_BYTE_TWO) = 0x6F;

    } else { //������ʷ����

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
    uint16_t   u8DecompressCountPerMin = 0;
    uint16_t   u16BatteryVoltage = 0.0;
    uint16_t   u16BatteryCurrent = 0.0;

    if(i_uint8_tIsHistoryData == EN_LATEST) { //���µ�����

//        APP_TRACE_INFO(("Load Fuel cell real time part B work info...\r\n"));
        //���ݱ�ͷ��
        *(i_pRealTimeWorkInfo + HEAD_BYTE_ONE) = 0xF1;
        *(i_pRealTimeWorkInfo + HEAD_BYTE_TWO) = 0xF2;
        *(i_pRealTimeWorkInfo + HEAD_BYTE_THREE) = 0xF3;
        *(i_pRealTimeWorkInfo + PRODUCTS_TYPE_ID_HIGH) = (uint8_t)(PRODUCT_MODEL_CODE >> 8);
        *(i_pRealTimeWorkInfo + PRODUCTS_TYPE_ID_LOW) = (uint8_t)(PRODUCT_MODEL_CODE & 0xFF);
        *(i_pRealTimeWorkInfo + LOCAL_NETWORK_ID_CODE) = g_u16GlobalNetWorkId;
        *(i_pRealTimeWorkInfo + INFORMATION_TYPE_CONTROL_CODE) = (uint8_t)REAL_TIME_RUNNING_INFORMATION_B_2;
        *(i_pRealTimeWorkInfo + VALID_INFORMATION_LENGTH_CONTROL_CODE) = REAL_TIME_RUNNING_INFO_B_2_LENGTH;

        //���ݱ�ǩ����ֵ����
//        g_u32TxMsgDataTagNumber[REAL_TIME_RUNNING_INFORMATION_B_1]++;
//        *(i_pRealTimeWorkInfo + DATA_IDENTIFY_TAG_INF_CODE_1) = (uint8_t)(g_u32TxMsgDataTagNumber[REAL_TIME_RUNNING_INFORMATION_B_1] >> 24);
//        *(i_pRealTimeWorkInfo + DATA_IDENTIFY_TAG_INF_CODE_2) = (uint8_t)((g_u32TxMsgDataTagNumber[REAL_TIME_RUNNING_INFORMATION_B_1] & 0xFF0000) >> 16);
//        *(i_pRealTimeWorkInfo + DATA_IDENTIFY_TAG_INF_CODE_3) = (uint8_t)((g_u32TxMsgDataTagNumber[REAL_TIME_RUNNING_INFORMATION_B_1] & 0xFF00) >> 8);
//        *(i_pRealTimeWorkInfo + DATA_IDENTIFY_TAG_INF_CODE_4) = (uint8_t)(g_u32TxMsgDataTagNumber[REAL_TIME_RUNNING_INFORMATION_B_1] & 0xFF);

        //���һ������ʵʱ��йѹ��������
        u8DecompressCountPerMin = GetPassiveDecompressCountPerMinutes();
        *(i_pRealTimeWorkInfo + STACK_DECOMPRESS_COUNT_PER_MINUTES_HIGH) = 0;
        *(i_pRealTimeWorkInfo + STACK_DECOMPRESS_COUNT_PER_MINUTES_LOW) = (uint8_t)(u8DecompressCountPerMin & 0xFF);

        //��ص�ѹ
//        u16BatteryVoltage = (uint16_t)(GetSrcAnaSig(BATTERY_VOLTAGE) * 100);
        u16BatteryVoltage = 0;
        *(i_pRealTimeWorkInfo + BATTERY_VOLTAGE_INTEGER_PART) = (uint8_t)(u16BatteryVoltage / 100);
        *(i_pRealTimeWorkInfo + BATTERY_VOLTAGE_IDECIMAL_PART) = (uint8_t)(u16BatteryVoltage % 100);

        //��ص���
//        u16BatteryCurrent = (uint16_t)(GetSrcAnaSig(BATTERY_CURRENT) * 100);
        u16BatteryCurrent = 0;
        *(i_pRealTimeWorkInfo + BATTERY_CURRENT_INTEGER_PART) = (uint8_t)(u16BatteryCurrent / 100);
        *(i_pRealTimeWorkInfo + BATTERY_CURRENT_IDECIMAL_PART) = (uint8_t)(u16BatteryCurrent % 100);

        //���ݱ�β��
        *(i_pRealTimeWorkInfo + END_BYTE_ONE) = 0x5F;
        *(i_pRealTimeWorkInfo + END_BYTE_TWO) = 0x6F;

    } else { //������ʷ����

    }
}
/*
***************************************************************************************************
*                                      InsertNonRealTimeWorkInfoDataToSendBuff()
*
* Description:  �����ʵʱ������Ϣ�����ݵ����ͻ�����������ʵʱ�Կ��ǣ�������뵽��һ�������͵����ݵ�λ��.
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

    OSMutexPend(&TxMsgSendBuffWriteMutex,
                OS_CFG_TICK_RATE_HZ / 10,
                OS_OPT_PEND_BLOCKING,
                NULL,
                &err);

    if(g_stTxMsgDataSendBuff.Q_length == 0) { //ֱ�ӷ��ڵ�һ��λ�ü���
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
            if(g_stTxMsgDataSendBuff.Queue[g_stTxMsgDataSendBuff.Q_Qhead].OccupyStatu == EN_WAITTING) { //ͷ�ڵ�Ҳ�ڵȴ���
                //ֱ���滻ͷ�ڵ�
                g_stTxMsgDataSendBuff.Queue[uint8_tInsertCursor].D_last = uint8_tInsertCursor;
                g_stTxMsgDataSendBuff.Queue[uint8_tInsertCursor].D_next = g_stTxMsgDataSendBuff.Q_Qhead;
                g_stTxMsgDataSendBuff.Queue[g_stTxMsgDataSendBuff.Q_Qhead].D_last = uint8_tInsertCursor;
                g_stTxMsgDataSendBuff.Q_Qhead = uint8_tInsertCursor;
            } else { //�½ڵ����ӵ�ͷ�ڵ����
                g_stTxMsgDataSendBuff.Queue[uint8_tInsertCursor].D_last = g_stTxMsgDataSendBuff.Q_Qhead;
                g_stTxMsgDataSendBuff.Queue[uint8_tInsertCursor].D_next = uint8_tInsertCursor;
                g_stTxMsgDataSendBuff.Queue[g_stTxMsgDataSendBuff.Q_Qhead].D_next = uint8_tInsertCursor;
                g_stTxMsgDataSendBuff.Q_Qrear = uint8_tInsertCursor;
            }
        } else {
            for(i = g_stTxMsgDataSendBuff.Q_Qhead; ;) {
                if(g_stTxMsgDataSendBuff.Queue[i].OccupyStatu == EN_WAITTING) {
                    g_stTxMsgDataSendBuff.Queue[i].D_next = i;//��ǰ��ͷ�ڵ㿪ʼ��һ���ȴ��Ľڵ��ǰ����ָ��Ҫ���������ݵĽڵ�
                    g_stTxMsgDataSendBuff.Queue[i].D_last = g_stTxMsgDataSendBuff.Q_Qrear;//���ٵ��½ڵ��ǰ����ָ��ԭ����β�ڵ�
                    g_stTxMsgDataSendBuff.Q_Qrear = i;//β�ڵ���µ��µĽڵ�
                    g_stTxMsgDataSendBuff.Queue[i].OccupyStatu = EN_WAITTING;//�½ڵ��״̬����Ϊ�ȴ�����
                    break;
                } else {
                    i = g_stTxMsgDataSendBuff.Queue[i].D_next;//������һ���ڵ�
                }
            }
        }
    } else {
        --g_stTxMsgDataSendBuff.Q_length;
        APP_TRACE_INFO(("The non real time inf add faild,the Tx queue is overflow...\r\n"));
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

//    APP_TRACE_INFO(("load the non real time data...\r\n"));

    if((i_eSendDataType != REAL_TIME_RUNNING_INFORMATION_A) && (i_eSendDataType != REAL_TIME_RUNNING_INFORMATION_B_1) && (i_eSendDataType != REAL_TIME_RUNNING_INFORMATION_B_2) && (i_eSendDataType < EN_SEND_DATA_TYPE_MAX)) {
        //���ݱ�ͷ��
        *(i_pNonRealTimeWorkInfo + HEAD_BYTE_ONE) = 0xF1;
        *(i_pNonRealTimeWorkInfo + HEAD_BYTE_TWO) = 0xF2;
        *(i_pNonRealTimeWorkInfo + HEAD_BYTE_THREE) = 0xF3;
        *(i_pNonRealTimeWorkInfo + PRODUCTS_TYPE_ID_HIGH) = (uint8_t)((PRODUCT_MODEL_CODE & 0xFF00) >> 8);
        *(i_pNonRealTimeWorkInfo + PRODUCTS_TYPE_ID_LOW) = (uint8_t)(PRODUCT_MODEL_CODE & 0xFF);
        *(i_pNonRealTimeWorkInfo + LOCAL_NETWORK_ID_CODE) = g_u16GlobalNetWorkId;

        g_u32TxMsgDataTagNumber[i_eSendDataType]++;
        *(i_pNonRealTimeWorkInfo + DATA_IDENTIFY_TAG_INF_CODE_1) = (uint8_t)(g_u32TxMsgDataTagNumber[i_eSendDataType] >> 24);
        *(i_pNonRealTimeWorkInfo + DATA_IDENTIFY_TAG_INF_CODE_2) = (uint8_t)((g_u32TxMsgDataTagNumber[i_eSendDataType] & 0xFF0000) >> 16);
        *(i_pNonRealTimeWorkInfo + DATA_IDENTIFY_TAG_INF_CODE_3) = (uint8_t)((g_u32TxMsgDataTagNumber[i_eSendDataType] & 0xFF00) >> 8);
        *(i_pNonRealTimeWorkInfo + DATA_IDENTIFY_TAG_INF_CODE_4) = (uint8_t)(g_u32TxMsgDataTagNumber[i_eSendDataType] & 0xFF);

        //�������Ϳ�����ĸ�4λ��δ���壬�ڴ�����
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

                *(i_pNonRealTimeWorkInfo + LEGAL_AUTHORIZATION_CODE) = EN_LEGAL_AUTHORIZATION;//��δ����Ȩ�޼�����

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

                //ͨ��״̬������
                m_u16ConrolAndCommunicateStatuCode = GetConrolAndCommunicateStatuCode();
                *(i_pNonRealTimeWorkInfo + CONTROL_AND_COMMUNICATE_STATU_CODE_BYTE_H) = (uint8_t)((m_u16ConrolAndCommunicateStatuCode & 0xFF00) >> 8);
                *(i_pNonRealTimeWorkInfo + CONTROL_AND_COMMUNICATE_STATU_CODE_BYTE_L) = (uint8_t)(m_u16ConrolAndCommunicateStatuCode & 0xFF);

                break;

            case CONSTANT_ASSIST_INFORMATION:
                *(i_pNonRealTimeWorkInfo + VALID_INFORMATION_LENGTH_CONTROL_CODE) = CONSTANT_ASSIST_INFORMATION_LENGTH;
                *(i_pNonRealTimeWorkInfo + LEGAL_AUTHORIZATION_CODE) = 0;//��δ��Ȩ�޼�����
                break;

            case FOR_QUERY_AND_CONFIG_INFORMATION:
                *(i_pNonRealTimeWorkInfo + VALID_INFORMATION_LENGTH_CONTROL_CODE) = FOR_QUERY_AND_CONFIG_INFORMATION_LENGTH;

                APP_TRACE_INFO(("Report config information...\r\n"));
                /*��ȡ�޸ĺ��Flash���������͵���λ��ȷ��*/
                *(i_pNonRealTimeWorkInfo + HEAT_STATUS_PUMP_CONTROL_SPD_HIGH) = (uint8_t)((g_stStartHydrgPumpSpdPara.PumpSpdIgniterFirstTime & 0xFF00) >> 8);
                *(i_pNonRealTimeWorkInfo + HEAT_STATUS_PUMP_CONTROL_SPD_LOW) = (uint8_t)(g_stStartHydrgPumpSpdPara.PumpSpdIgniterFirstTime & 0xFF);

                *(i_pNonRealTimeWorkInfo + HEAT_STATUS_HYDROGEN_FAN_CONTROL_SPD_HIGH) = (uint8_t)((g_stStartHydrgFanSpdPara.FanSpdIgniterFirstTime & 0xFF00) >> 8);
                *(i_pNonRealTimeWorkInfo + HEAT_STATUS_HYDROGEN_FAN_CONTROL_SPD_LOW) = (uint8_t)(g_stStartHydrgFanSpdPara.FanSpdIgniterFirstTime & 0xFF);

                *(i_pNonRealTimeWorkInfo + RUNNING_STATUS_PUMP_SPD_CONTROL_HIGH) = (uint8_t)((g_stStartHydrgPumpSpdPara.PumpSpdIgniterSecondTime & 0xFF00) >> 8);
                *(i_pNonRealTimeWorkInfo + RUNNING_STATUS_PUMP_SPD_CONTROL_LOW) = (uint8_t)(g_stStartHydrgPumpSpdPara.PumpSpdIgniterSecondTime & 0xFF);

                *(i_pNonRealTimeWorkInfo + RUNNING_STATUS_HYDROGEN_FAN_SPD_CONTROL_HIGH) = (uint8_t)((g_stStartHydrgFanSpdPara.FanSpdIgniterSecondTime & 0xFF00) >> 8);
                *(i_pNonRealTimeWorkInfo + RUNNING_STATUS_HYDROGEN_FAN_SPD_CONTROL_LOW) = (uint8_t)(g_stStartHydrgFanSpdPara.FanSpdIgniterSecondTime & 0xFF);

                *(i_pNonRealTimeWorkInfo + RUNNING_STATUS_FIRST_DELAY_TIME) = (uint8_t)(g_stRunningStatusDelayAdjustSpdPara.FirstDelayTimeByMin & 0xFF);

                *(i_pNonRealTimeWorkInfo + RUNNING_STATUS_FIRST_TIME_ADJUST_PUMP_FLAG) = (uint8_t)(g_stRunningStatusDelayAdjustSpdPara.FirstTimeAdjustPumpFlag & 0xFF);
                *(i_pNonRealTimeWorkInfo + RUNNING_STATUS_FIRST_TIME_ADJUST_PUMP_VALUE) = (uint8_t)(g_stRunningStatusDelayAdjustSpdPara.FirstTimeAdjustPumpValue & 0xFF);

                *(i_pNonRealTimeWorkInfo + RUNNING_STATUS_FIRST_TIME_ADJUST_FUN_FLAG) = (uint8_t)(g_stRunningStatusDelayAdjustSpdPara.FirstTimeAdjustFanFlag & 0xFF);
                *(i_pNonRealTimeWorkInfo + RUNNING_STATUS_FIRST_TIME_ADJUST_FUN_VALUE) = (uint8_t)(g_stRunningStatusDelayAdjustSpdPara.FirstTimeAdjustFanValue & 0xFF);

                *(i_pNonRealTimeWorkInfo + RUNNING_STATUS_SECOND_DELAY_TIME) = (uint8_t)(g_stRunningStatusDelayAdjustSpdPara.SecondDelayTimeByMin & 0xFF);

                *(i_pNonRealTimeWorkInfo + RUNNING_STATUS_SECOND_TIME_ADJUST_PUMP_FLAG) = (uint8_t)(g_stRunningStatusDelayAdjustSpdPara.SecondTimeAdjustPumpFlag & 0xFF);
                *(i_pNonRealTimeWorkInfo + RUNNING_STATUS_SECOND_TIME_ADJUST_PUMP_VALUE) = (uint8_t)(g_stRunningStatusDelayAdjustSpdPara.SecondTimeAdjustPumpValue & 0xFF);

                *(i_pNonRealTimeWorkInfo + RUNNING_STATUS_SECOND_TIME_ADJUST_FUN_FLAG) = (uint8_t)(g_stRunningStatusDelayAdjustSpdPara.SecondTimeAdjustFanFlag & 0xFF);
                *(i_pNonRealTimeWorkInfo + RUNNING_STATUS_SECOND_TIME_ADJUST_FUN_VALUE) = (uint8_t)(g_stRunningStatusDelayAdjustSpdPara.SecondTimeAdjustFanValue & 0xFF);
                break;

            default:
                break;
        }

        //���ݱ�β��
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
    static uint8_t i;
    
    BSP_PrgmDataDMASend(i_uint8_tTxMsgLen, i_pTxMsg);

    if(g_eCAN_BusOnLineFlag == YES)//CAN��������
    {
        SendCanMsgContainNodeId(PRGM_TX_BUFF_SIZE, i_pTxMsg, g_u16GlobalNetWorkId);
        APP_TRACE_INFO(("Can Tx data:"));
        for(i = 0; i < 60; i++) {
            APP_TRACE_INFO(("%X ", i_pTxMsg[i]));
        }
        APP_TRACE_INFO(("...\n\r"));
    }

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
            && (*(i_PrgmRxMsg + REDEIVE_DATA_BYTE_END_OF_DATA)   == 0xAA)//��β
            && (*(i_PrgmRxMsg + RECEIVE_DATA_BYTE_HEAD_TARGET_LOCAL_NET_ID)   == g_u16GlobalNetWorkId)//����ID
//            && (*(i_PrgmRxMsg + RECEIVE_DATA_BYTE_HEAD_TARGET_LOCAL_NET_ID)   == 0x01)//�ݲ��ж���ģ��ID
      ) {
        switch(*(i_PrgmRxMsg + RECEIVE_DATA_BYTE_CMD_TYPE) & 0x0F) { //ָ��������ĸ�4λ��δ���塪����������
            case COMMAND_TYPE_DBG://����ָ��
                switch(*(i_PrgmRxMsg + REDEIVE_DATA_BYTE_CMD_CODE_VALUE)) {
                    case DBG_SELECT_WORK_MODE:
                        break;

                    case DBG_SWITCH_OVER_CONTROL_MODE:
                        ControlModeTurnOver();
                        SendRealTimeAssistInfo();
                        break;

                    case DBG_START_THE_MACHINE:
                        APP_TRACE_INFO(("Cmd ->Start ...\r\n"));
                        CmdStart();
                        break;

                    case DBG_AHEAD_RUNNING:
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
                        SetPumpCtlSpd((uint16_t)(*(i_PrgmRxMsg + REDEIVE_DATA_BYTE_CMD_PARAMETER_SECTION_1) << 8) + * (i_PrgmRxMsg + REDEIVE_DATA_BYTE_CMD_PARAMETER_SECTION_2));
                        break;

                    case DBG_HYDRG_SPEED_INC:
                        HydrgFanSpdInc();
                        break;

                    case DBG_HYDRG_SPEED_DEC:
                        HydrgFanSpdDec();
                        break;

                    case DBG_HYDRG_SPEED_SET_WHTI_PARAMETERS:
                        SetHydrgFanCtlSpd((uint16_t)((*(i_PrgmRxMsg + REDEIVE_DATA_BYTE_CMD_PARAMETER_SECTION_1) << 8) + * (i_PrgmRxMsg + REDEIVE_DATA_BYTE_CMD_PARAMETER_SECTION_2)));
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

            case COMMAND_TYPE_CONFIGURATION: //������ָ��
                switch(*(i_PrgmRxMsg + REDEIVE_DATA_BYTE_CMD_CODE_VALUE)) {

                    case CONFIG_HYDROGEN_GROUP_RUNNING_PARA:
                        if((EN_WAITTING_COMMAND == GetSystemWorkStatu() || EN_ALARMING == GetSystemWorkStatu())) {
                            if(*(i_PrgmRxMsg + REDEIVE_DATA_BYTE_CMD_PARAMETERS_LENGTH) == 6u) {

                                switch((uint8_t)(*(i_PrgmRxMsg + REDEIVE_DATA_BYTE_CMD_PARAMETER_SECTION_1))) {
                                    case CONFIG_IGNITE_FIRST_STEP_PARA:
                                        switch((uint8_t)(*(i_PrgmRxMsg + REDEIVE_DATA_BYTE_CMD_PARAMETER_SECTION_2))) {
                                            case CONFIG_IGNITE_FIRST_STEP_PUMP_SPD:
                                                OSSchedLock(&err);
                                                g_stStartHydrgPumpSpdPara.PumpSpdIgniterFirstTime  = (*(i_PrgmRxMsg + REDEIVE_DATA_BYTE_CMD_PARAMETER_SECTION_3) << 8) | *(i_PrgmRxMsg + REDEIVE_DATA_BYTE_CMD_PARAMETER_SECTION_4);
                                                StoreStartHydrgPumpSpdParaBySingle(&g_stStartHydrgPumpSpdPara.PumpSpdIgniterFirstTime, 0);
                                                SendInquireOrConfigurationInfo();//�������ò���
                                                OSSchedUnlock(&err);
                                                APP_TRACE_INFO(("PumpSpdIgniterFirstTime:%d...\n\r", g_stStartHydrgPumpSpdPara.PumpSpdIgniterFirstTime));
                                                break;

                                            case CONFIG_IGNITE_FIRST_STEP_FAN_SPD:
                                                OSSchedLock(&err);
                                                g_stStartHydrgFanSpdPara.FanSpdIgniterFirstTime = (*(i_PrgmRxMsg + REDEIVE_DATA_BYTE_CMD_PARAMETER_SECTION_3) << 8) | *(i_PrgmRxMsg + REDEIVE_DATA_BYTE_CMD_PARAMETER_SECTION_4);
                                                StoreStartHydrgFanSpdParaBySingle(&g_stStartHydrgFanSpdPara.FanSpdIgniterFirstTime, 0);
                                                SendInquireOrConfigurationInfo();
                                                OSSchedUnlock(&err);
                                                APP_TRACE_INFO(("FanSpdIgniterFirstTime:%d...\n\r", g_stStartHydrgFanSpdPara.FanSpdIgniterFirstTime));
                                                break;

                                            case CONFIG_FIRST_TIME_HEAT_HOLD_TIME_BY_SEC:
                                                OSSchedLock(&err);
                                                g_u16FirstTimeHeatHoldSeconds = (*(i_PrgmRxMsg + REDEIVE_DATA_BYTE_CMD_PARAMETER_SECTION_3) << 8) | *(i_PrgmRxMsg + REDEIVE_DATA_BYTE_CMD_PARAMETER_SECTION_4);
                                                StoreFirstTimeHeatHoldSeconds(&g_u16FirstTimeHeatHoldSeconds);
                                                SendInquireOrConfigurationInfo();
                                                OSSchedUnlock(&err);
                                                APP_TRACE_INFO(("g_u16FirstTimeHeatHoldSeconds:%d...\n\r", g_u16FirstTimeHeatHoldSeconds));
                                                break;

                                            default:
                                                break;
                                        }

                                        break;

                                    case CONFIG_RUNNING_STATUS_PARA:
                                        switch((uint8_t)(*(i_PrgmRxMsg + REDEIVE_DATA_BYTE_CMD_PARAMETER_SECTION_2))) {
                                            case CONFIG_IGNITE_SECOND_STEP_PUMP_SPD:
                                                OSSchedLock(&err);
                                                g_stStartHydrgPumpSpdPara.PumpSpdIgniterSecondTime = (*(i_PrgmRxMsg + REDEIVE_DATA_BYTE_CMD_PARAMETER_SECTION_3) << 8) | *(i_PrgmRxMsg + REDEIVE_DATA_BYTE_CMD_PARAMETER_SECTION_4);
                                                StoreStartHydrgPumpSpdParaBySingle(&g_stStartHydrgPumpSpdPara.PumpSpdIgniterSecondTime, 1);
                                                SendInquireOrConfigurationInfo();
                                                OSSchedUnlock(&err);
                                                APP_TRACE_INFO(("PumpSpdIgniterSecondTime:%d...\n\r", g_stStartHydrgPumpSpdPara.PumpSpdIgniterSecondTime));
                                                break;

                                            case CONFIG_IGNITE_SECOND_STEP_FAN_SPD:
                                                OSSchedLock(&err);
                                                g_stStartHydrgFanSpdPara.FanSpdIgniterSecondTime = (*(i_PrgmRxMsg + REDEIVE_DATA_BYTE_CMD_PARAMETER_SECTION_3) << 8) | *(i_PrgmRxMsg + REDEIVE_DATA_BYTE_CMD_PARAMETER_SECTION_4);
                                                StoreStartHydrgFanSpdParaBySingle(&g_stStartHydrgFanSpdPara.FanSpdIgniterSecondTime, 2);
                                                SendInquireOrConfigurationInfo();
                                                OSSchedUnlock(&err);
                                                APP_TRACE_INFO(("FanSpdIgniterSecondTime:%d...\n\r", g_stStartHydrgFanSpdPara.FanSpdIgniterSecondTime));
                                                break;

                                            case CONFIG_FIRST_TIME_DELAY_ADJUST_TIME_BY_MINUTE:
                                                OSSchedLock(&err);
                                                g_stRunningStatusDelayAdjustSpdPara.FirstDelayTimeByMin = *(i_PrgmRxMsg + REDEIVE_DATA_BYTE_CMD_PARAMETER_SECTION_4);
                                                StoreRunningStatusDelayAdjustSpdParaBySingle(&g_stRunningStatusDelayAdjustSpdPara.FirstDelayTimeByMin, 0);
                                                SendInquireOrConfigurationInfo();
                                                OSSchedUnlock(&err);
                                                APP_TRACE_INFO(("FirstDelayTimeByMin:%d...\n\r", g_stRunningStatusDelayAdjustSpdPara.FirstDelayTimeByMin));
                                                break;

                                            case CONFIG_FIRST_TIME_DELAY_ADJUST_PUMP_SPEED_VALUE:
                                                OSSchedLock(&err);
                                                g_stRunningStatusDelayAdjustSpdPara.FirstTimeAdjustPumpFlag = *(i_PrgmRxMsg + REDEIVE_DATA_BYTE_CMD_PARAMETER_SECTION_3);
                                                g_stRunningStatusDelayAdjustSpdPara.FirstTimeAdjustPumpValue = *(i_PrgmRxMsg + REDEIVE_DATA_BYTE_CMD_PARAMETER_SECTION_4);
                                                StoreRunningStatusDelayAdjustSpdParaBySingle((u16 *)&g_stRunningStatusDelayAdjustSpdPara.FirstTimeAdjustPumpValue, 1);
                                                SendInquireOrConfigurationInfo();
                                                APP_TRACE_INFO(("FirstTimeAdjustPumpValue:%d...\n\r", g_stRunningStatusDelayAdjustSpdPara.FirstTimeAdjustPumpValue));
                                                OSSchedUnlock(&err);
                                                break;

                                            case CONFIG_FIRST_TIME_DELAY_ADJUST_FANS_SPEED_VALUE:
                                                OSSchedLock(&err);
                                                g_stRunningStatusDelayAdjustSpdPara.FirstTimeAdjustFanFlag = *(i_PrgmRxMsg + REDEIVE_DATA_BYTE_CMD_PARAMETER_SECTION_3);
                                                g_stRunningStatusDelayAdjustSpdPara.FirstTimeAdjustFanValue = *(i_PrgmRxMsg + REDEIVE_DATA_BYTE_CMD_PARAMETER_SECTION_4);
                                                StoreRunningStatusDelayAdjustSpdParaBySingle((u16 *)&g_stRunningStatusDelayAdjustSpdPara.FirstTimeAdjustFanValue, 2);
                                                SendInquireOrConfigurationInfo();
                                                OSSchedUnlock(&err);
                                                APP_TRACE_INFO(("FirstTimeAdjustFanValue:%d...\n\r", g_stRunningStatusDelayAdjustSpdPara.FirstTimeAdjustFanValue));
                                                break;

                                            case CONFIG_SECOND_TIME_DELAY_ADJUST_TIME_BY_MINUTE:
                                                OSSchedLock(&err);
                                                g_stRunningStatusDelayAdjustSpdPara.SecondDelayTimeByMin = *(i_PrgmRxMsg + REDEIVE_DATA_BYTE_CMD_PARAMETER_SECTION_4);
                                                StoreRunningStatusDelayAdjustSpdParaBySingle(&g_stRunningStatusDelayAdjustSpdPara.SecondDelayTimeByMin, 3);
                                                SendInquireOrConfigurationInfo();
                                                OSSchedUnlock(&err);
                                                APP_TRACE_INFO(("SecondDelayTimeByMin:%d...\n\r", g_stRunningStatusDelayAdjustSpdPara.SecondDelayTimeByMin));
                                                break;

                                            case CONFIG_SECOND_TIME_DELAY_ADJUST_PUMP_SPEED_VALUE:
                                                OSSchedLock(&err);
                                                g_stRunningStatusDelayAdjustSpdPara.SecondTimeAdjustPumpFlag = *(i_PrgmRxMsg + REDEIVE_DATA_BYTE_CMD_PARAMETER_SECTION_3);
                                                g_stRunningStatusDelayAdjustSpdPara.SecondTimeAdjustPumpValue = *(i_PrgmRxMsg + REDEIVE_DATA_BYTE_CMD_PARAMETER_SECTION_4);
                                                StoreRunningStatusDelayAdjustSpdParaBySingle((u16 *)&g_stRunningStatusDelayAdjustSpdPara.SecondTimeAdjustPumpValue, 4);
                                                SendInquireOrConfigurationInfo();
                                                OSSchedUnlock(&err);
                                                APP_TRACE_INFO(("SecondTimeAdjustPumpValue:%d...\n\r", g_stRunningStatusDelayAdjustSpdPara.SecondTimeAdjustPumpValue));
                                                break;

                                            case CONFIG_SECOND_TIME_DELAY_ADJUST_FANS_SPEED_VALUE:
                                                OSSchedLock(&err);
                                                g_stRunningStatusDelayAdjustSpdPara.SecondTimeAdjustFanFlag = *(i_PrgmRxMsg + REDEIVE_DATA_BYTE_CMD_PARAMETER_SECTION_3);
                                                g_stRunningStatusDelayAdjustSpdPara.SecondTimeAdjustFanValue = *(i_PrgmRxMsg + REDEIVE_DATA_BYTE_CMD_PARAMETER_SECTION_4);
                                                StoreRunningStatusDelayAdjustSpdParaBySingle((u16 *)&g_stRunningStatusDelayAdjustSpdPara.SecondTimeAdjustFanValue, 5);
                                                SendInquireOrConfigurationInfo();
                                                OSSchedUnlock(&err);
                                                APP_TRACE_INFO(("SecondTimeAdjustFanValue:%d...\n\r", g_stRunningStatusDelayAdjustSpdPara.SecondTimeAdjustFanValue));
                                                break;

                                            default:
                                                break;
                                        }

                                        break;

                                    default:
                                        break;
                                }
                            }
                        }

                        break;

                    default:
                        break;
                }

                break;

            case COMMAND_TYPE_INQUIRE_REQUEST://��ѯ������ָ��
                switch(*(i_PrgmRxMsg + REDEIVE_DATA_BYTE_CMD_CODE_VALUE)) {
                    case REQUEST_DATA_RETRANSMIT:
                        break;

                    case INQUIRE_HYDROGEN_RUNNING_PARAMETERS:
                        SendInquireOrConfigurationInfo();//�ϱ����в���
                        break;

                    default:
                        break;
                }

            case COMMAND_TYPE_RESPONSE_CONFIRM://Ӧ��ȷ��ָ��
                switch(*(i_PrgmRxMsg + REDEIVE_DATA_BYTE_CMD_CODE_VALUE)) {
                    case RESPONSE_ALLOCATE_ID_NMB_WITH_PARAMETERS:
                        if((EN_WAITTING_COMMAND == GetSystemWorkStatu() || EN_ALARMING == GetSystemWorkStatu()) && (1 == *(i_PrgmRxMsg + REDEIVE_DATA_BYTE_CMD_PARAMETERS_LENGTH))) { //���Ʋ�������Ϊ1
                            OSSchedLock(&err);

                            if((*(i_PrgmRxMsg + REDEIVE_DATA_BYTE_CMD_PARAMETERS_LENGTH) <= 255)) {
                                g_u16GlobalNetWorkId = *(i_PrgmRxMsg + REDEIVE_DATA_BYTE_CMD_PARAMETER_SECTION_1);
                                StoreGlobalNetWorkID(&g_u16GlobalNetWorkId);
                                SendInquireOrConfigurationInfo();//�������ò���
                                CAN1_Init();//֡ID�仯�������³�ʼ��CAN������,�Խ����µ�֡ID����
                            }

                            OSSchedUnlock(&err);
                            APP_TRACE_INFO(("Id set,ID:%d...\n\r", (u8)g_u16GlobalNetWorkId));
                        }

                        break;

                    case RESPONSE_SLAVE_SHUT_DOWN_CMD:
                        u32ErrCode = ((uint32_t) * (i_PrgmRxMsg + REDEIVE_DATA_BYTE_CMD_PARAMETER_SECTION_1) << 24)
                                     | ((uint32_t) * (i_PrgmRxMsg + REDEIVE_DATA_BYTE_CMD_PARAMETER_SECTION_2) << 16)
                                     | ((uint32_t) * (i_PrgmRxMsg + REDEIVE_DATA_BYTE_CMD_PARAMETER_SECTION_3) << 8)
                                     | ((uint32_t) * (i_PrgmRxMsg + REDEIVE_DATA_BYTE_CMD_PARAMETER_SECTION_4));

                        switch(*(i_PrgmRxMsg + REDEIVE_DATA_BYTE_CMD_PARAMETER_SECTION_5)) {
                            case RESPONSE_SLAVE_SHUT_DOWN_CMD_RIGHT_NOW:
                                i = 0;

                                while(u32ErrCode) {
                                    if((u32ErrCode % 2) == 1) {
                                        SetShutDownRequestMaskStatu((SYSTEM_ALARM_ADDR_Typedef)i, EN_UN_MASK, 0);//��Ӧ�Ĵ�������λ����
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
                                        SetShutDownRequestMaskStatu((SYSTEM_ALARM_ADDR_Typedef)i, EN_DELAY, (uint16_t)(*(i_PrgmRxMsg + REDEIVE_DATA_BYTE_CMD_PARAMETER_SECTION_6) * 60));//��Ӧ�Ĵ�������λ����
                                    }

                                    i++;
                                    u32ErrCode >>= 1;
                                }

//                              APP_TRACE_INFO(("Err bit %d, %d...\r\n",i, *(i_PrgmRxMsg + REDEIVE_DATA_BYTE_CMD_PARAMETER_SECTION_6)));
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
    } else { //����ͨ�ų���
        //���յ�λδ��������
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

    if(DEF_OFF == GetHydrgProducerStopDlyStatu() && (DEF_OFF == GetStackStopDlyStatu())) {  //��ֹ�ػ�����δ�����ٴο���
        if(eSysRunningStatu == EN_WAITTING_COMMAND) { //�ȴ�ָ�ѭ���豸�Լ���
            eWorkMode = GetWorkMode();

            if(eWorkMode != EN_WORK_MODE_FUEL_CELL) {   //�������һ���ģʽ
                OSSchedLock(&err);
                SetSystemWorkStatu(EN_START_PRGM_ONE_FRONT);
            } else {
                OSSchedLock(&err);                   //����ģʽֱ��Ϊ����״̬
                SetSystemWorkStatu(EN_RUNNING);
            }

            OSTaskSemPost(&AppTaskStartTCB,   //send to WaittingCommand�����е������ź���������������
                          OS_OPT_POST_NO_SCHED,
                          &err);
            OSSchedUnlock(&err);
        } else if(eSysRunningStatu == EN_ALARMING) {

            SetSystemWorkStatu(EN_WAITTING_COMMAND);//����״̬�£���������ų�,�ٴΰ������������ų�����
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
        OSSchedLock(&err);
        SetSystemWorkStatu(EN_SHUTTING_DOWN);

        switch((uint8_t)eSysRunningStatu) { //switch��֧��ö���ͱ���������תΪuint8_t��
            case(uint8_t)EN_START_PRGM_ONE_FRONT:
                OSTaskSemPost(&AppTaskStartTCB,//�������ټ���
                              OS_OPT_POST_NO_SCHED,
                              &err);
                break;

            case(uint8_t)EN_START_PRGM_ONE_BEHIND:
                OSSemPost(&IgniteFirstBehindWaitSem,    //�������
                          OS_OPT_POST_1,
                          &err);
                break;

            case(uint8_t)EN_START_PRGM_TWO:
                break;

            case(uint8_t)EN_RUNNING:
                OSTaskResume(&AppTaskStartTCB,      //�ָ���ʼ���񣬽���shutdown����
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

void SendInquireOrConfigurationInfo()
{
    InsertNonRealTimeWorkInfoDataToSendBuff(FOR_QUERY_AND_CONFIG_INFORMATION, NULL, NULL, NULL);
}
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