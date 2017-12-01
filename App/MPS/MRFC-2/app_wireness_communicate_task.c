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
#define COMM_TASK_STK_SIZE               200
#define COMM_DATA_SEND_TASK_STK_SIZE     200

#define  TX_MSG_MEM_BLK_NUM             30    //内存分区的内存块数量
#define  TX_MSG_MEM_BLK_SIZE            60    //内存分区的内存块大小
#define  TX_MSG_QUEUE_SIZE              30    //发送消息队列长度
/*
***************************************************************************************************
*                                         OS-RELATED    VARIABLES
***************************************************************************************************
*/
OS_SEM          g_stCommDataSendSem;

OS_TCB          CommTaskTCB;
OS_TCB          CommDataSendTaskTCB;

OS_Q            CommInfoQueue;
OS_MEM          CommInfoMem;

static          CPU_STK_8BYTE_ALIGNED     CommTaskStk[COMM_TASK_STK_SIZE];
static          CPU_STK_8BYTE_ALIGNED     CommDataSendTaskStk[COMM_DATA_SEND_TASK_STK_SIZE];

/*
***************************************************************************************************
*                                           LOCAL VARIABLES
***************************************************************************************************
*/
		 CPU_INT08U  				 CommInfoMemStorage[TX_MSG_MEM_BLK_NUM][TX_MSG_MEM_BLK_SIZE];
         uint8_t                     g_u8SerRxMsgBuff[PRGM_RX_BUFF_SIZE];
static   uint32_t                    g_u32TxMsgDataTagNumber[EN_SEND_DATA_TYPE_MAX] = {0};   //数据身份标签码

/*
***************************************************************************************************
*                                         FUNCTION PROTOTYPES
***************************************************************************************************
*/
static      void        CommTask(void *p_arg);
static      void        CommDataSendTask(void *p_arg);
static      void        ResponsePrgmCmd(uint8_t *);
static      void        SendAPrgmMsgFrame(uint8_t i_uint8_tTxMsgLen, uint8_t *i_pTxMsg);

static      void        AddRealTimeWorkInfoPartA1ToSendQueue(void);
static      void        AddRealTimeWorkInfoPartB1ToSendQueue(void);
static      void        AddRealTimeWorkInfoPartB2ToSendQueue(void);
static      void        AddNonRealTimeWorkInfoToSendQueue(uint8_t , uint8_t, uint8_t, uint8_t *);

/*
***************************************************************************************************
*                                       MACRO DEFINITIONS
***************************************************************************************************
*/


/*
***************************************************************************************************
*                                         OS-RELATED    VARIABLES
***************************************************************************************************
*/


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

void  CommTaskCreate(void)
{
    OS_ERR  err;

	BSP_SerToWIFI_Init();       //(WIFI)串口初始化
	
    OSTaskCreate((OS_TCB *)&CommTaskTCB,
                 (CPU_CHAR *)"Comm Task Start",
                 (OS_TASK_PTR) CommTask,
                 (void *) 0,
                 (OS_PRIO) COMM_TASK_PRIO,
                 (CPU_STK *)&CommTaskStk[0],
                 (CPU_STK_SIZE) COMM_TASK_STK_SIZE / 10,
                 (CPU_STK_SIZE) COMM_TASK_STK_SIZE,
                 (OS_MSG_QTY) 5u,
                 (OS_TICK) 0u,
                 (void *) 0,
                 (OS_OPT)(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR *)&err);
    APP_TRACE_INFO(("Created communicate Task, and err code is %d...\r\n", err));
                   
    OSSemCreate(&g_stCommDataSendSem,
                "Comm data send sem",
                0,
                &err);
                                
    OSQCreate((OS_Q *)&CommInfoQueue,
              (CPU_CHAR *)"Comm Info Queue",
              (OS_MSG_QTY)TX_MSG_QUEUE_SIZE,
              (OS_ERR *)&err);
                
    OSMemCreate((OS_MEM     *)&CommInfoMem,
               (CPU_CHAR    *)"Comm Info Mem",
               (void        *)&CommInfoMemStorage[0][0],//内存分区起始地址
               (OS_MEM_QTY   )TX_MSG_MEM_BLK_NUM,//内存分区里的内存块数量
               (OS_MEM_SIZE  )TX_MSG_MEM_BLK_SIZE,//每个内存块的大小(字节)
               (OS_ERR      *)&err);
}

/*
***************************************************************************************************
*                                     CommDataSendTaskCreate()
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
void  CommDataSendTaskCreate(void)
{
    OS_ERR  err;
    OSTaskCreate((OS_TCB *)&CommDataSendTaskTCB,
                 (CPU_CHAR *)"Comm Data Send Task Start",
                 (OS_TASK_PTR) CommDataSendTask,
                 (void *) 0,
                 (OS_PRIO) COMM_DATA_SEND_TASK_PRIO,
                 (CPU_STK *)&CommDataSendTaskStk[0],
                 (CPU_STK_SIZE) COMM_DATA_SEND_TASK_STK_SIZE / 10,
                 (CPU_STK_SIZE) COMM_DATA_SEND_TASK_STK_SIZE,
                 (OS_MSG_QTY) 5u,
                 (OS_TICK) 0u,
                 (void *) 0,
                 (OS_OPT)(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR *)&err);
    APP_TRACE_INFO(("Created communicate Data Send Task, and err code is %d...\r\n", err));
}

/*
***************************************************************************************************
*                               CommTask()
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
void  CommTask(void *p_arg)
{
    OS_ERR      err;

    while(DEF_TRUE) {
        
        OSTimeDlyHMSM(0, 0, 1, 000,
                      OS_OPT_TIME_HMSM_STRICT,
                      &err);
       
		if(g_u8WifiCmdRec == DEF_YES) { //收到串口指令而提前结束延时，响应上位机指令
            ResponsePrgmCmd(g_u8SerRxMsgBuff);
            g_u8WifiCmdRec = DEF_NO;
		
//WIFI_RX_DATA_TEST
#if     0  
            APP_TRACE_INFO(("WIFI Rx data:"));
            for(uint8_t i = 0;i < 16;i++){
                APP_TRACE_INFO(("%X ",g_u8SerRxMsgBuff[i]));
            }
            APP_TRACE_INFO(("...\n\r"));
#endif
		}
			
        if(g_eCanMsgRxStatu == DEF_YES){//是否因收到CAN接口指令而提前结束延时
		
			ResponsePrgmCmd(g_u8CanRxMsg);
			g_eCanMsgRxStatu = DEF_NO;
		}
		//添加实时信息到发送队列
		AddRealTimeWorkInfoPartA1ToSendQueue();
        AddRealTimeWorkInfoPartB1ToSendQueue();
        AddRealTimeWorkInfoPartB2ToSendQueue();

        //发送信号量，启动一次数据传输
        OSSemPost(&g_stCommDataSendSem,
                  OS_OPT_POST_1,
                  &err);
    }
}


/*
***************************************************************************************************
*                               CommDataSendTask()
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
static void CommDataSendTask(void *p_arg)
{
    OS_ERR       err;
    OS_MSG_SIZE  msg_size;
    CPU_INT08U   *pTxMsgData = NULL;
 
    while(DEF_TRUE) {
        
          pTxMsgData = OSQPend((OS_Q        *)&CommInfoQueue,
                               (OS_TICK      )OS_CFG_TICK_RATE_HZ / 5,
                               (OS_OPT       )OS_OPT_PEND_BLOCKING,
                               (OS_MSG_SIZE *)&msg_size,
                               (CPU_TS      *)NULL,
                               (OS_ERR      *)&err);
        
          if(err == OS_ERR_NONE){
              if(pTxMsgData != NULL){
				  
                  SendAPrgmMsgFrame(msg_size, pTxMsgData);
                  OSMemPut(&CommInfoMem,pTxMsgData,&err);    
              }                   
          }   
    }
}


/*
***************************************************************************************************
*                                      AddRealTimeWorkInfoPartA1ToSendQueue()
*
* Description:  载入制氢机实时运行数据,或当前的,或提取自历史记录.
*
* Arguments  :  none
*
* Returns    :  none
***************************************************************************************************
*/
static void AddRealTimeWorkInfoPartA1ToSendQueue(void)
{
	OS_ERR        err;
	CPU_INT08U   *pTxBlkPtr; 
    uint16_t    u16CurrentTemp = 0;
    uint16_t    u16HydrgWorkTimes = 0;
//    uint16_t    u16HydrogenGasConcentration = 0;
    uint16_t    u16VacuumNetativePressure = 0;
    uint16_t    u16LiquidLevel = 0;
//    uint16_t    u16LiquidFeedPerMinute = 0;
    uint16_t    u16PumpFeedbackSpeed = 0, u16PumpCtlSpeed = 0;
    uint16_t    u16HydrgFanCtlSpd = 0, u16HydrgFeedbackFanSpd = 0;
//    uint16_t    u16FluidWeightPerMinuteMul100 = 0;
    uint16_t    m_u16LiquidPress = 0;
    uint32_t    u32AlarmCode = 0;
    uint32_t    u32SystemRunningStatuCode = 0;
    SYSTEM_TIME_Typedef     stHydrgProduceTimeThisTime = {0}, stHydrgProduceTimeTotal = {0};

	pTxBlkPtr = OSMemGet(&CommInfoMem,&err);
	
    if(err == OS_ERR_NONE){
//        APP_TRACE_INFO(("Load Hydrogen Producer real time work info...\r\n"));
        //数据报头段
        *(pTxBlkPtr + HEAD_BYTE_ONE) = 0xF1;
        *(pTxBlkPtr + HEAD_BYTE_TWO) = 0xF2;
        *(pTxBlkPtr + HEAD_BYTE_THREE) = 0xF3;
        *(pTxBlkPtr + PRODUCTS_TYPE_ID_HIGH) = (uint8_t)((g_ProductsType & 0xFF00) >>  8);
        *(pTxBlkPtr + PRODUCTS_TYPE_ID_LOW) = (uint8_t)(g_ProductsType & 0xFF);
        *(pTxBlkPtr + LOCAL_NETWORK_ID_CODE) = g_u16GlobalNetWorkId;

        *(pTxBlkPtr + INFORMATION_TYPE_CODE) = (uint8_t)RT_RUNNING_INFO_A_PART_A;
        *(pTxBlkPtr + VALID_INFO_LEN_CTRL_CODE) = RT_RUNNING_INFO_A_1_LEN;

        //数据标签码码值增加
        g_u32TxMsgDataTagNumber[RT_RUNNING_INFO_A_PART_A]++;
        *(pTxBlkPtr + DATA_IDENTIFY_TAG_INF_CODE_1) = (uint8_t)(g_u32TxMsgDataTagNumber[RT_RUNNING_INFO_A_PART_A] >> 24);
        *(pTxBlkPtr + DATA_IDENTIFY_TAG_INF_CODE_2) = (uint8_t)((g_u32TxMsgDataTagNumber[RT_RUNNING_INFO_A_PART_A] & 0xFF0000) >> 16);
        *(pTxBlkPtr + DATA_IDENTIFY_TAG_INF_CODE_3) = (uint8_t)((g_u32TxMsgDataTagNumber[RT_RUNNING_INFO_A_PART_A] & 0xFF00) >> 8);
        *(pTxBlkPtr + DATA_IDENTIFY_TAG_INF_CODE_4) = (uint8_t)(g_u32TxMsgDataTagNumber[RT_RUNNING_INFO_A_PART_A] & 0xFF);

        //运行警报码
        u32AlarmCode = GetRunAlarmCode();
        *(pTxBlkPtr + RUN_ALARM_CODE_BYTE_4) = (uint8_t)(u32AlarmCode >> 24);
        *(pTxBlkPtr + RUN_ALARM_CODE_BYTE_3) = (uint8_t)((u32AlarmCode & 0xFF0000) >> 16);
        *(pTxBlkPtr + RUN_ALARM_CODE_BYTE_2) = (uint8_t)((u32AlarmCode & 0xFF00) >> 8);
        *(pTxBlkPtr + RUN_ALARM_CODE_BYTE_1) = (uint8_t)(u32AlarmCode & 0xFF);
        //运行状态码
        u32SystemRunningStatuCode = GetSystemRunningStatuCode();
        *(pTxBlkPtr + RUNNING_STATU_CODE_BYTE_4) = (uint8_t)(u32SystemRunningStatuCode >> 24);
        *(pTxBlkPtr + RUNNING_STATU_CODE_BYTE_3) = (uint8_t)((u32SystemRunningStatuCode & 0xFF0000) >> 16);
        *(pTxBlkPtr + RUNNING_STATU_CODE_BYTE_2) = (uint8_t)((u32SystemRunningStatuCode & 0xFF00) >> 8);
        *(pTxBlkPtr + RUNNING_STATU_CODE_BYTE_1) = (uint8_t)(u32SystemRunningStatuCode & 0xFF);
        //重整温度
        u16CurrentTemp = (uint16_t)GetReformerTemp();
        *(pTxBlkPtr + REFORMER_TEMP_HIGH) = (uint8_t)((u16CurrentTemp & 0xFF00) >> 8);
        *(pTxBlkPtr + REFORMER_TEMP_LOW) = (uint8_t)((u16CurrentTemp & 0xFF));
        //火焰温度
        u16CurrentTemp = (uint16_t)GetFireOrRodTemp();
        *(pTxBlkPtr + FIRE_OR_ROD_TEMP_HIGH) = (uint8_t)((u16CurrentTemp & 0xFF00) >> 8);
        *(pTxBlkPtr + FIRE_OR_ROD_TEMP_LOW) = (uint8_t)((u16CurrentTemp & 0xFF));
        //液压
        m_u16LiquidPress = (uint16_t)(GetSrcAnaSig(LIQUID_PRESS) * 100);
        *(pTxBlkPtr + LIQUID_PRESS_INTEGER_PART) = (uint8_t)(m_u16LiquidPress / 100);
        *(pTxBlkPtr + LIQUID_PRESS_DECIMAL_PART) = (uint8_t)(m_u16LiquidPress % 100);
        //风机控制速度
        u16HydrgFanCtlSpd = GetHydrgFanCurrentCtlSpd();
        *(pTxBlkPtr + HYDROGEN_FAN_SPD_CONTROL_HIGH) = (uint8_t)((u16HydrgFanCtlSpd & 0xFF00) >> 8);
        *(pTxBlkPtr + HYDROGEN_FAN_SPD_CONTROL_LOW) = (uint8_t)((u16HydrgFanCtlSpd & 0xFF));
        //风机反馈速度
        u16HydrgFeedbackFanSpd = GetHydrgFanFeedBackSpd();
        *(pTxBlkPtr + HYDROGEN_FAN_SPD_FEEDBACK_HIGH) = (uint8_t)((u16HydrgFeedbackFanSpd & 0xFF00) >> 8);
        *(pTxBlkPtr + HYDROGEN_FAN_SPD_FEEDBACK_LOW) = (uint8_t)((u16HydrgFeedbackFanSpd & 0xFF));
        //水泵控制速度
        u16PumpCtlSpeed = GetPumpCtlSpd();
        *(pTxBlkPtr + PUMP_SPD_CONTROL_HIGH) = (uint8_t)((u16PumpCtlSpeed & 0xFF00) >> 8);
        *(pTxBlkPtr + PUMP_SPD_CONTROL_LOW) = (uint8_t)(u16PumpCtlSpeed & 0xFF);
        //水泵反馈速度
        u16PumpFeedbackSpeed = GetPumpFeedBackSpd();
        *(pTxBlkPtr + PUMP_SPD_FEEDBACK_HIGH) = (uint8_t)((u16PumpFeedbackSpeed & 0xFF00) >> 8);
        *(pTxBlkPtr + PUMP_SPD_FEEDBACK_LOW) = (uint8_t)(u16PumpFeedbackSpeed & 0xFF);

        //本次制氢时间,APP上主界面显示的是这个的值
        stHydrgProduceTimeThisTime = GetHydrgProduceTimeThisTime();
        //  APP_TRACE_INFO(("AAAAA:%d%d\r\nMinut:%d\r\nSecond:%d\r\n",(uint8_t)((stHydrgProduceTimeThisTime.hour & 0xFF00) >> 8),(uint8_t)((stHydrgProduceTimeThisTime.hour & 0xFF)), stHydrgProduceTimeThisTime.minute,stHydrgProduceTimeThisTime.second));
        *(pTxBlkPtr + HYDROGEN_PRODUCT_TIME_THIS_TIME_HOUR_HIGH) = (uint8_t)((stHydrgProduceTimeThisTime.hour & 0xFF00) >> 8);
        *(pTxBlkPtr + HYDROGEN_PRODUCT_TIME_THIS_TIME_HOUR_LOW) = (uint8_t)(stHydrgProduceTimeThisTime.hour & 0xFF);
        *(pTxBlkPtr + HYDROGEN_PRODUCT_TIME_THIS_TIME_MINUTE) = stHydrgProduceTimeThisTime.minute;
        *(pTxBlkPtr + HYDROGEN_PRODUCT_TIME_THIS_TIME_SECOND) = stHydrgProduceTimeThisTime.second;
        //累计制氢时间
        stHydrgProduceTimeTotal = GetHydrgProduceTimeThisTime();//暂以本次制氢时间代替
        *(pTxBlkPtr + HYDROGEN_PRODUCT_TIME_TOTAL_HOUR_HIGH) = (uint8_t)((stHydrgProduceTimeTotal.hour & 0xFF00) >> 8);
        *(pTxBlkPtr + HYDROGEN_PRODUCT_TIME_TOTAL_HOUR_LOW) = (uint8_t)(stHydrgProduceTimeTotal.hour & 0xFF);
        *(pTxBlkPtr + HYDROGEN_PRODUCT_TIME_TOTAL_MINUTE) = stHydrgProduceTimeTotal.minute;
        *(pTxBlkPtr + HYDROGEN_PRODUCT_TIME_TOTAL_SECOND) = stHydrgProduceTimeTotal.second;
        //累计制氢次数
        u16HydrgWorkTimes = GetHydrgProducerWorkTimes();
        *(pTxBlkPtr + HYDROGEN_PRODUCT_TOTAL_TIMES_HIGH) = (uint8_t)((u16HydrgWorkTimes & 0xFF00) >> 8);
        *(pTxBlkPtr + HYDROGEN_PRODUCT_TOTAL_TIMES_LOW) = (uint8_t)(u16HydrgWorkTimes & 0xFF);

        //液位数据
        u16LiquidLevel = (uint16_t)GetSrcAnaSig(LIQUID_LEVEL);
        *(pTxBlkPtr + LIQUID_LEVEL_INTEGER_PART) = (uint8_t)((u16LiquidLevel & 0xFF00) >> 8);
        *(pTxBlkPtr + LIQUID_LEVEL_DECIMAL_PART) = (uint8_t)(u16LiquidLevel & 0xFF);
		
        //每分钟进液量
//        u16LiquidFeedPerMinute = (uint16_t)(ReadLiquidFlowRate() * 100);
//        u16LiquidFeedPerMinute = 0;
//        *(pTxBlkPtr + LIQUID_FEED_PER_MINUTE_INTEGER_PART) = (uint8_t)(u16LiquidFeedPerMinute / 100);
//        *(pTxBlkPtr + LIQUID_FEED_PER_MINUTE_DECIMAL_PART) = (uint8_t)(u16LiquidFeedPerMinute % 100);

        //真空负压
        u16VacuumNetativePressure = (uint16_t)(GetSrcAnaSig(NEGATIVE_PRESSURE) * 100);
        *(pTxBlkPtr + VACUUM_NEGATIVE_PRESSURE_HIGH) = (uint8_t)(u16VacuumNetativePressure / 100);
        *(pTxBlkPtr + VACUUM_NEGATIVE_PRESSURE_LOW) = (uint8_t)(u16VacuumNetativePressure % 100);

        //氢气浓度
//        u16HydrogenGasConcentration = (uint16_t)(GetSrcAnaSig(HYDROGEN_CONCENTRATION) * 100);
//        u16HydrogenGasConcentration = 0;
//        *(pTxBlkPtr + HYDROGEN_PRODUCT_GAS_CONCENTRATION_INTEGER_PART) = (uint8_t)(u16HydrogenGasConcentration / 100);
//        *(pTxBlkPtr + HYDROGEN_PRODUCT_GAS_CONCENTRATION_DECIMAL_PART) = (uint8_t)(u16HydrogenGasConcentration % 100);

        //子模块ID号
//        *(pTxBlkPtr + SUB_MODULE_ID_OF_THE_MULTI_MODULE_TYPE) = SUB_MODULE_ID;
        //数据报尾段
        *(pTxBlkPtr + END_BYTE_ONE) = 0x5F;
        *(pTxBlkPtr + END_BYTE_TWO) = 0x6F;

		OSQPost(&CommInfoQueue,
				pTxBlkPtr,
				TX_MSG_MEM_BLK_SIZE,
				OS_OPT_POST_FIFO,
				&err);
		if(err != OS_ERR_NONE){
			APP_TRACE_INFO(("- ->RT work info part_a1 msg post err,err code is %d\n\r",err));
		}
	}else{
		APP_TRACE_INFO(("- ->RT work info part_a1 mem blk get err,err code is %d\n\r",err));
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
static void AddRealTimeWorkInfoPartB1ToSendQueue(void)
{
	OS_ERR        err;
	CPU_INT08U   *pTxBlkPtr; 
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
    uint32_t  u32IsolatedGenratedEnergyCount = 0;
    int16_t   i16HydrogYieldMatchOffsetValue = 0;
    uint32_t  u32SystemWorkStatuCode = 0;
    SYSTEM_TIME_Typedef     stStackWorkTimeThisTime = {0}, stStackWorkTimeTotal = {0};

    pTxBlkPtr = OSMemGet(&CommInfoMem,&err);
    
    if(err == OS_ERR_NONE){
//        APP_TRACE_INFO(("Load Fuel cell real time part A work info...\r\n"));
        //数据报头段
        *(pTxBlkPtr + HEAD_BYTE_ONE) = 0xF1;
        *(pTxBlkPtr + HEAD_BYTE_TWO) = 0xF2;
        *(pTxBlkPtr + HEAD_BYTE_THREE) = 0xF3;
        *(pTxBlkPtr + PRODUCTS_TYPE_ID_HIGH) = (uint8_t)(g_ProductsType >> 8);
        *(pTxBlkPtr + PRODUCTS_TYPE_ID_LOW) = (uint8_t)(g_ProductsType & 0xFF);
        *(pTxBlkPtr + LOCAL_NETWORK_ID_CODE) = g_u16GlobalNetWorkId;

        *(pTxBlkPtr + INFORMATION_TYPE_CODE) = (uint8_t)RT_RUNNING_INFO_B_PART_A;
        *(pTxBlkPtr + VALID_INFO_LEN_CTRL_CODE) = RT_RUNNING_INFO_B_1_LENGTH;

        //数据标签码码值增加
        g_u32TxMsgDataTagNumber[RT_RUNNING_INFO_B_PART_A]++;
        *(pTxBlkPtr + DATA_IDENTIFY_TAG_INF_CODE_1) = (uint8_t)(g_u32TxMsgDataTagNumber[RT_RUNNING_INFO_B_PART_A] >> 24);
        *(pTxBlkPtr + DATA_IDENTIFY_TAG_INF_CODE_2) = (uint8_t)((g_u32TxMsgDataTagNumber[RT_RUNNING_INFO_B_PART_A] & 0xFF0000) >> 16);
        *(pTxBlkPtr + DATA_IDENTIFY_TAG_INF_CODE_3) = (uint8_t)((g_u32TxMsgDataTagNumber[RT_RUNNING_INFO_B_PART_A] & 0xFF00) >> 8);
        *(pTxBlkPtr + DATA_IDENTIFY_TAG_INF_CODE_4) = (uint8_t)(g_u32TxMsgDataTagNumber[RT_RUNNING_INFO_B_PART_A] & 0xFF);

        //运行警报码
        u32AlarmCode = GetRunAlarmCode();
        *(pTxBlkPtr + RUN_ALARM_CODE_BYTE_4) = (uint8_t)(u32AlarmCode >> 24);
        *(pTxBlkPtr + RUN_ALARM_CODE_BYTE_3) = (uint8_t)((u32AlarmCode & 0xFF0000) >> 16);
        *(pTxBlkPtr + RUN_ALARM_CODE_BYTE_2) = (uint8_t)((u32AlarmCode & 0xFF00) >> 8);
        *(pTxBlkPtr + RUN_ALARM_CODE_BYTE_1) = (uint8_t)(u32AlarmCode & 0xFF);

        //运行状态码
        u32SystemWorkStatuCode = GetSystemRunningStatuCode();
        *(pTxBlkPtr + RUNNING_STATU_CODE_BYTE_4) = (uint8_t)(u32SystemWorkStatuCode >> 24);
        *(pTxBlkPtr + RUNNING_STATU_CODE_BYTE_3) = (uint8_t)((u32SystemWorkStatuCode & 0xFF0000) >> 16);
        *(pTxBlkPtr + RUNNING_STATU_CODE_BYTE_2) = (uint8_t)((u32SystemWorkStatuCode & 0xFF00) >> 8);
        *(pTxBlkPtr + RUNNING_STATU_CODE_BYTE_1) = (uint8_t)(u32SystemWorkStatuCode & 0xFF);

        //电堆电流*100
        m_u16StackCurrentMul100 = (uint16_t)(GetSrcAnaSig(STACK_CURRENT) * 100);
        *(pTxBlkPtr + STACK_CURRENT_INTEGER_PART) = (uint8_t)(m_u16StackCurrentMul100 / 100);
        *(pTxBlkPtr + STACK_CURRENT_DECIMAL_PART) = (uint8_t)(m_u16StackCurrentMul100 % 100);

        //电堆电压*100
        m_u16StackVoltageMul100 = (uint16_t)(GetSrcAnaSig(STACK_VOLTAGE) * 100);
        *(pTxBlkPtr + STACK_VOLTAGE_INTEGER_PART) = (uint8_t)(m_u16StackVoltageMul100 / 100);
        *(pTxBlkPtr + STACK_VOLTAGE_DECIMAL_PART) = (uint8_t)(m_u16StackVoltageMul100 % 100);

        //电堆温度,暂时只用整数部分
        u16StackTemp = (uint16_t)GetSrcAnaSig(STACK_TEMP);
        *(pTxBlkPtr + STACK_TEMP_INTEGER_PART) = (uint8_t)(u16StackTemp & 0xFF);
//      *(pTxBlkPtr + STACK_TEMP_INTEGER_PART) = (uint8_t)((u32SystemWorkStatuCode & 0xFF00) >> 8);
//      *(pTxBlkPtr + STACK_TEMP_DECIMAL_PART) = (uint8_t)(u32SystemWorkStatuCode & 0xFF);

        //电堆氢气压力*100
        m_u16StackPressMul100 = (uint16_t)(GetSrcAnaSig(HYDROGEN_PRESS_1) * 100);
        *(pTxBlkPtr + STACK_HYDROGEN_PRESS_INTEGER_PART) = (uint8_t)(m_u16StackPressMul100 / 100);
        *(pTxBlkPtr + STACK_HYDROGEN_PRESS_DECIMAL_PART) = (uint8_t)(m_u16StackPressMul100 % 100);

        //电堆风扇控制速度
        u16StackFanCtlSpd = GetStackFanCtlSpd();
        *(pTxBlkPtr + STACK_FAN_SPD_CONTROL_HIGH) = (uint8_t)((u16StackFanCtlSpd & 0xFF00) >> 8);
        *(pTxBlkPtr + STACK_FAN_SPD_CONTROL_LOW) = (uint8_t)(u16StackFanCtlSpd & 0xFF);

        //电堆风扇反馈速度
        u16StackFanSpdFeedBack = GetStackFanSpdFeedBack();
        *(pTxBlkPtr + STACK_FAN_PART_A_SPD_FEEDBACK_HIGH) = (uint8_t)((u16StackFanSpdFeedBack & 0xFF00) >> 8);
        *(pTxBlkPtr + STACK_FAN_PART_A_SPD_FEEDBACK_LOW) = (uint8_t)(u16StackFanSpdFeedBack & 0xFF);

        u16StackFanSpdFeedBackB = 0;
        *(pTxBlkPtr + STACK_FAN_PART_B_SPD_FEEDBACK_HIGH) = (uint8_t)((u16StackFanSpdFeedBackB & 0xFF00) >> 8);
        *(pTxBlkPtr + STACK_FAN_PART_B_SPD_FEEDBACK_LOW) = (uint8_t)(u16StackFanSpdFeedBackB & 0xFF);

        //本次发电时间,APP上发电界面显示的是这个值
        stStackWorkTimeThisTime = GetStackProductTimeThisTime();
        *(pTxBlkPtr + STACK_WORK_TIME_THIS_TIME_HOUR_HIGH) = (uint8_t)((stStackWorkTimeThisTime.hour & 0xFF00) >> 8);
        *(pTxBlkPtr + STACK_WORK_TIME_THIS_TIME_HOUR_LOW) = (uint8_t)(stStackWorkTimeThisTime.hour & 0xFF);
        *(pTxBlkPtr + STACK_WORK_TIME_THIS_TIME_MINUTE) = (uint8_t)(stStackWorkTimeThisTime.minute);
        *(pTxBlkPtr + STACK_WORK_TIME_THIS_TIME_SECOND) = (uint8_t)(stStackWorkTimeThisTime.second);

        //累计发电时间
        stStackWorkTimeTotal = GetStackProductTimeTotal();
        *(pTxBlkPtr + STACK_WORK_TIME_TOTAL_HOUR_HIGH) = (uint8_t)((stStackWorkTimeTotal.hour & 0xFF00) >> 8);
        *(pTxBlkPtr + STACK_WORK_TIME_TOTAL_HOUR_LOW) = (uint8_t)(stStackWorkTimeTotal.hour & 0xFF);
        *(pTxBlkPtr + STACK_WORK_TIME_TOTAL_MINUTE) = (uint8_t)(stStackWorkTimeTotal.minute);
        *(pTxBlkPtr + STACK_WORK_TIME_TOTAL_SECOND) = (uint8_t)(stStackWorkTimeTotal.second);

        //当前输出功率
        u32StackPower = (uint32_t)(GetCurrentPower() * 100);
        *(pTxBlkPtr + CURRENT_ISOLATED_POWER_INTEGER_PART_HIGH) = (uint8_t)(((uint16_t)(u32StackPower / 100) & 0xFF00) >> 8);
        *(pTxBlkPtr + CURRENT_ISOLATED_POWER_INTEGER_PART_LOW) = (uint8_t)((uint16_t)(u32StackPower / 100) & 0xFF);
        *(pTxBlkPtr + CURRENT_ISOLATED_POWER_DECIMAL_PART) = (uint8_t)(u32StackPower % 100);

        //本次单机发电量
        u32IsolatedGenratedEnergyThisTime = (uint32_t)(GetIsolatedGenratedEnergyThisTime() * 1000);
        *(pTxBlkPtr + ISOLATED_GENERATED_ENERGY_THIE_TIME_INTEGER_PART_HIGH) = (uint8_t)(((uint16_t)(u32IsolatedGenratedEnergyThisTime / 1000) & 0xFF00) >> 8);
        *(pTxBlkPtr + ISOLATED_GENERATED_ENERGY_THIE_TIME_INTEGER_PART_LOW) = (uint8_t)((uint16_t)(u32IsolatedGenratedEnergyThisTime / 1000) & 0xFF);
        *(pTxBlkPtr + ISOLATED_GENERATED_ENERGY_THIE_TIME_DECIMAL_PART_HIGH) = (uint8_t)(((uint16_t)(u32IsolatedGenratedEnergyThisTime % 1000) & 0xFF00) >> 8);
        *(pTxBlkPtr + ISOLATED_GENERATED_ENERGY_THIE_TIME_DECIMAL_PART_LOW) = (uint8_t)((uint16_t)(u32IsolatedGenratedEnergyThisTime % 1000) & 0xFF);

        u32IsolatedGenratedEnergyCount = 0;
        *(pTxBlkPtr + GENERATED_ENERGY_TOTAL_INTEGER_PART_HIGH) = (uint8_t)(((uint16_t)(u32IsolatedGenratedEnergyCount / 1000) & 0xFF00) >> 8);
        *(pTxBlkPtr + GENERATED_ENERGY_TOTAL_INTEGER_PART_LOW) = (uint8_t)((uint16_t)(u32IsolatedGenratedEnergyCount / 1000) & 0xFF);
        *(pTxBlkPtr + GENERATED_ENERGY_TOTAL_DECIMAL_PART_HIGH) = (uint8_t)(((uint16_t)(u32IsolatedGenratedEnergyCount % 1000) & 0xFF00) >> 8);
        *(pTxBlkPtr + GENERATED_ENERGY_TOTAL_DECIMAL_PART_LOW) = (uint8_t)((uint16_t)(u32IsolatedGenratedEnergyCount % 1000) & 0xFF);

        //匹氢偏移值
		i16HydrogYieldMatchOffsetValue = 0;
//        i16HydrogYieldMatchOffsetValue = (int16_t)(GetStackHydrogenYieldMatchOffsetValue() * 100);

//        if(i16HydrogYieldMatchOffsetValue > 0 || i16HydrogYieldMatchOffsetValue < -100) { //整数位带符号位
//            *(pTxBlkPtr + HYDROGEN_YIELD_MATCHING_OFFSET_VALUE_INTEGER_PART_MUL100) = (int8_t)(i16HydrogYieldMatchOffsetValue / 100);
//            *(pTxBlkPtr + HYDROGEN_YIELD_MATCHING_OFFSET_VALUE_DECIMAL_PART_MUL100_HIGH) = (uint8_t)(((i16HydrogYieldMatchOffsetValue % 100)) & 0xFF);
//        } else { //小数位带符号位
//            *(pTxBlkPtr + HYDROGEN_YIELD_MATCHING_OFFSET_VALUE_INTEGER_PART_MUL100) = (uint8_t)(i16HydrogYieldMatchOffsetValue / 100);
//            *(pTxBlkPtr + HYDROGEN_YIELD_MATCHING_OFFSET_VALUE_DECIMAL_PART_MUL100_HIGH) = (int8_t)((i16HydrogYieldMatchOffsetValue % 100) & 0xFF);
//        }

        //子模块ID号
//        *(pTxBlkPtr + SUB_MODULE_ID_OF_THE_MULTI_MODULE_TYPE) = SUB_MODULE_ID;

        //数据报尾段
        *(pTxBlkPtr + END_BYTE_ONE) = 0x5F;
        *(pTxBlkPtr + END_BYTE_TWO) = 0x6F;
		
		OSQPost(&CommInfoQueue,
                pTxBlkPtr,
                TX_MSG_MEM_BLK_SIZE,
                OS_OPT_POST_FIFO,
                &err);
        if(err != OS_ERR_NONE){
            APP_TRACE_INFO(("- ->RT work info part_b1 msg post err,err code is %d\n\r",err));
        }
    }else{
        APP_TRACE_INFO(("- ->RT work info part_b1 mem blk get err,err code is %d\n\r",err));
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
static void AddRealTimeWorkInfoPartB2ToSendQueue(void)
{
	OS_ERR       err;
    CPU_INT08U   *pTxBlkPtr;
    uint16_t   u8DecompressCountPerMin = 0;
    uint16_t   u16BatteryVoltage = 0.0;
    uint16_t   u16BatteryCurrent = 0.0;

    pTxBlkPtr = OSMemGet(&CommInfoMem,&err);
    
    if(err == OS_ERR_NONE){

//        APP_TRACE_INFO(("Load Fuel cell real time part B work info...\r\n"));
        //数据报头段
        *(pTxBlkPtr + HEAD_BYTE_ONE) = 0xF1;
        *(pTxBlkPtr + HEAD_BYTE_TWO) = 0xF2;
        *(pTxBlkPtr + HEAD_BYTE_THREE) = 0xF3;
        *(pTxBlkPtr + PRODUCTS_TYPE_ID_HIGH) = (uint8_t)(g_ProductsType >> 8);
        *(pTxBlkPtr + PRODUCTS_TYPE_ID_LOW) = (uint8_t)(g_ProductsType & 0xFF);
        *(pTxBlkPtr + LOCAL_NETWORK_ID_CODE) = g_u16GlobalNetWorkId;
        *(pTxBlkPtr + INFORMATION_TYPE_CODE) = (uint8_t)RT_RUNNING_INFO_B_PART_B;
        *(pTxBlkPtr + VALID_INFO_LEN_CTRL_CODE) = RT_RUNNING_INFO_B_2_LENGTH;

        //数据标签码码值增加
        g_u32TxMsgDataTagNumber[RT_RUNNING_INFO_B_PART_A]++;
        *(pTxBlkPtr + DATA_IDENTIFY_TAG_INF_CODE_1) = (uint8_t)(g_u32TxMsgDataTagNumber[RT_RUNNING_INFO_B_PART_A] >> 24);
        *(pTxBlkPtr + DATA_IDENTIFY_TAG_INF_CODE_2) = (uint8_t)((g_u32TxMsgDataTagNumber[RT_RUNNING_INFO_B_PART_A] & 0xFF0000) >> 16);
        *(pTxBlkPtr + DATA_IDENTIFY_TAG_INF_CODE_3) = (uint8_t)((g_u32TxMsgDataTagNumber[RT_RUNNING_INFO_B_PART_A] & 0xFF00) >> 8);
        *(pTxBlkPtr + DATA_IDENTIFY_TAG_INF_CODE_4) = (uint8_t)(g_u32TxMsgDataTagNumber[RT_RUNNING_INFO_B_PART_A] & 0xFF);

        //电堆一分钟内实时的泄压排气次数
        u8DecompressCountPerMin = GetPassiveDecompressCountPerMinutes();
        *(pTxBlkPtr + STACK_DECOMPRESS_COUNT_PER_MINUTES_HIGH) = 0;
        *(pTxBlkPtr + STACK_DECOMPRESS_COUNT_PER_MINUTES_LOW) = (uint8_t)(u8DecompressCountPerMin & 0xFF);

        //电池电压
        u16BatteryVoltage = (uint16_t)(GetSrcAnaSig(BATTERY_VOLTAGE) * 100);
        *(pTxBlkPtr + BATTERY_VOLTAGE_INTEGER_PART) = (uint8_t)(u16BatteryVoltage / 100);
        *(pTxBlkPtr + BATTERY_VOLTAGE_IDECIMAL_PART) = (uint8_t)(u16BatteryVoltage % 100);

        //电池电流
//        u16BatteryCurrent = (uint16_t)(GetSrcAnaSig(BATTERY_CURRENT) * 100);
        u16BatteryCurrent = 0;
        *(pTxBlkPtr + BATTERY_CURRENT_INTEGER_PART) = (uint8_t)(u16BatteryCurrent / 100);
        *(pTxBlkPtr + BATTERY_CURRENT_IDECIMAL_PART) = (uint8_t)(u16BatteryCurrent % 100);

        //数据报尾段
        *(pTxBlkPtr + END_BYTE_ONE) = 0x5F;
        *(pTxBlkPtr + END_BYTE_TWO) = 0x6F;
		
		OSQPost(&CommInfoQueue,
                pTxBlkPtr,
                TX_MSG_MEM_BLK_SIZE,
                OS_OPT_POST_FIFO,
                &err);
        if(err != OS_ERR_NONE){
            APP_TRACE_INFO(("- ->RT work info part_b2 msg post err,err code is %d\n\r",err));
        }
    }else{
        APP_TRACE_INFO(("- ->RT work info part_b2 mem blk get err,err code is %d\n\r",err));
    }
}
/*
***************************************************************************************************
*                                      AddNonRealTimeWorkInfoToSendQueue()
*
* Description:  This is a task that manage the communication with other device.
*
* Arguments  :  none
*
* Returns    :  none
***************************************************************************************************
*/
static void AddNonRealTimeWorkInfoToSendQueue(uint8_t i_eSendDataType, uint8_t i_uint8_tCmdCode, uint8_t i_uint8_tParaLen, uint8_t *p_uint8_tParas)
{
    OS_ERR       err;
    uint8_t i;
    CPU_INT08U   *pTxBlkPtr;
    SELF_CHECK_CODE_Typedef     stSelfCheckCode;
    uint16_t   m_u16ConrolAndCommStatusCode;

    pTxBlkPtr = OSMemGet(&CommInfoMem,&err);
    
    if(err == OS_ERR_NONE){
        if((i_eSendDataType != RT_RUNNING_INFO_A_PART_A) && (i_eSendDataType != RT_RUNNING_INFO_B_PART_A) && (i_eSendDataType != RT_RUNNING_INFO_B_PART_B) && (i_eSendDataType < EN_SEND_DATA_TYPE_MAX)) {
            //数据报头段
            *(pTxBlkPtr + HEAD_BYTE_ONE) = 0xF1;
            *(pTxBlkPtr + HEAD_BYTE_TWO) = 0xF2;
            *(pTxBlkPtr + HEAD_BYTE_THREE) = 0xF3;
            *(pTxBlkPtr + PRODUCTS_TYPE_ID_HIGH) = (uint8_t)((g_ProductsType & 0xFF00) >> 8);
            *(pTxBlkPtr + PRODUCTS_TYPE_ID_LOW) = (uint8_t)(g_ProductsType & 0xFF);
            *(pTxBlkPtr + LOCAL_NETWORK_ID_CODE) = g_u16GlobalNetWorkId;

            g_u32TxMsgDataTagNumber[i_eSendDataType]++;
            *(pTxBlkPtr + DATA_IDENTIFY_TAG_INF_CODE_1) = (uint8_t)(g_u32TxMsgDataTagNumber[i_eSendDataType] >> 24);
            *(pTxBlkPtr + DATA_IDENTIFY_TAG_INF_CODE_2) = (uint8_t)((g_u32TxMsgDataTagNumber[i_eSendDataType] & 0xFF0000) >> 16);
            *(pTxBlkPtr + DATA_IDENTIFY_TAG_INF_CODE_3) = (uint8_t)((g_u32TxMsgDataTagNumber[i_eSendDataType] & 0xFF00) >> 8);
            *(pTxBlkPtr + DATA_IDENTIFY_TAG_INF_CODE_4) = (uint8_t)(g_u32TxMsgDataTagNumber[i_eSendDataType] & 0xFF);

            //数据类型控制码的高4位尚未定义，在此清零
            *(pTxBlkPtr + INFORMATION_TYPE_CODE) = (uint8_t)(i_eSendDataType & 0x0F);

            switch((uint8_t)i_eSendDataType) {
                case RT_REQ_INFO:
                    *(pTxBlkPtr + VALID_INFO_LEN_CTRL_CODE) = RT_REQUEST_INFO_LENGTH;
                    *(pTxBlkPtr + REQUEST_INFORMATION_TYPE) = i_uint8_tCmdCode;
                    *(pTxBlkPtr + LENGTH_OF_REQUEST_PARAMETERS) = i_uint8_tParaLen;
                    i = 0;

                    while(i < i_uint8_tParaLen) {
                        *(pTxBlkPtr + REQUEST_PARA_ONE + i) = *(p_uint8_tParas + i);
                        i++;
                    }

                    break;

                case RT_ASSIST_INFO:
                    *(pTxBlkPtr + VALID_INFO_LEN_CTRL_CODE) = RT_ASSIST_INFO_LENGTH;
                    
                    *(pTxBlkPtr + LEGAL_AUTHORIZATION_CODE) = EN_LEGAL_AUTHORIZATION;//暂未检验权限检验码
                
                    stSelfCheckCode = GetSysSelfCheckCode();
                    *(pTxBlkPtr + SELF_CHECK_SENSOR_STATUS_CODE) = stSelfCheckCode.DevSelfCheckSensorStatusCode;

                    *(pTxBlkPtr + SELF_CHECK_CODE_GROUP_HYDRGEN_BYTE_4) = (uint8_t)((stSelfCheckCode.MachinePartASelfCheckCode & 0xFF000000) >> 24);
                    *(pTxBlkPtr + SELF_CHECK_CODE_GROUP_HYDRGEN_BYTE_3) = (uint8_t)((stSelfCheckCode.MachinePartASelfCheckCode & 0xFF0000) >> 16);
                    *(pTxBlkPtr + SELF_CHECK_CODE_GROUP_HYDRGEN_BYTE_2) = (uint8_t)((stSelfCheckCode.MachinePartASelfCheckCode & 0xFF00) >> 8);
                    *(pTxBlkPtr + SELF_CHECK_CODE_GROUP_HYDRGEN_BYTE_1) = (uint8_t)(stSelfCheckCode.MachinePartASelfCheckCode & 0xFF);

                    *(pTxBlkPtr + SELF_CHECK_CODE_GROUP_FUEL_CELL_BYTE_4) = (uint8_t)((stSelfCheckCode.MachinePartBSelfCheckCode & 0xFF000000) >> 24);
                    *(pTxBlkPtr + SELF_CHECK_CODE_GROUP_FUEL_CELL_BYTE_3) = (uint8_t)((stSelfCheckCode.MachinePartBSelfCheckCode & 0xFF0000) >> 16);
                    *(pTxBlkPtr + SELF_CHECK_CODE_GROUP_FUEL_CELL_BYTE_2) = (uint8_t)((stSelfCheckCode.MachinePartBSelfCheckCode & 0xFF00) >> 8);
                    *(pTxBlkPtr + SELF_CHECK_CODE_GROUP_FUEL_CELL_BYTE_1) = (uint8_t)(stSelfCheckCode.MachinePartBSelfCheckCode & 0xFF);

                    //通信状态控制码
                    m_u16ConrolAndCommStatusCode = GetConrolAndCommStatuCode();
                    *(pTxBlkPtr + CTRL_AND_COMM_STATU_CODE_BYTE_H) = (uint8_t)((m_u16ConrolAndCommStatusCode & 0xFF00) >> 8);
                    *(pTxBlkPtr + CTRL_AND_COMM_STATU_CODE_BYTE_L) = (uint8_t)(m_u16ConrolAndCommStatusCode & 0xFF);

                    break;

                case CONSTANT_ASSIST_INFO:
                    *(pTxBlkPtr + VALID_INFO_LEN_CTRL_CODE) = CONSTANT_ASSIST_INFO_LEN;
                    *(pTxBlkPtr + LEGAL_AUTHORIZATION_CODE) = 0;//暂未设权限检验码
                    break;
				
				case FOR_QUERY_AND_CFG_INFO:
                *(pTxBlkPtr + VALID_INFO_LEN_CTRL_CODE) = FOR_QUERY_AND_CFG_INFO_LEN;

				/*获取修改后的Flash参数并发送到上位机确认*/
				*(pTxBlkPtr + HEAT_STATUS_PUMP_CONTROL_SPD_HIGH) = (uint8_t)((g_stStartHydrgPumpSpdPara.PumpSpdIgniterFirstTime & 0xFF00) >> 8);
				*(pTxBlkPtr + HEAT_STATUS_PUMP_CONTROL_SPD_LOW) = (uint8_t)(g_stStartHydrgPumpSpdPara.PumpSpdIgniterFirstTime & 0xFF);

				*(pTxBlkPtr + HEAT_STATUS_HYDROGEN_FAN_CONTROL_SPD_HIGH) = (uint8_t)((g_stStartHydrogenFanSpdPara.FanSpdIgniterFirstTime & 0xFF00) >> 8);
				*(pTxBlkPtr + HEAT_STATUS_HYDROGEN_FAN_CONTROL_SPD_LOW) = (uint8_t)(g_stStartHydrogenFanSpdPara.FanSpdIgniterFirstTime & 0xFF);

				*(pTxBlkPtr + RUNNING_STATUS_PUMP_SPD_CONTROL_HIGH) = (uint8_t)((g_stStartHydrgPumpSpdPara.PumpSpdIgniterSecondTime & 0xFF00) >> 8);
				*(pTxBlkPtr + RUNNING_STATUS_PUMP_SPD_CONTROL_LOW) = (uint8_t)(g_stStartHydrgPumpSpdPara.PumpSpdIgniterSecondTime & 0xFF);

				*(pTxBlkPtr + RUNNING_STATUS_HYDROGEN_FAN_SPD_CONTROL_HIGH) = (uint8_t)((g_stStartHydrogenFanSpdPara.FanSpdIgniterSecondTime & 0xFF00) >> 8);
				*(pTxBlkPtr + RUNNING_STATUS_HYDROGEN_FAN_SPD_CONTROL_LOW) = (uint8_t)(g_stStartHydrogenFanSpdPara.FanSpdIgniterSecondTime & 0xFF);

				*(pTxBlkPtr + FAST_HEAT_HOLD_SECONDS_VALUE_HIGH) = (uint8_t)((g_u16FirstTimeHeatHoldSeconds & 0xFF00) >> 8);
				*(pTxBlkPtr + FAST_HEAT_HOLD_SECONDS_VALUE_LOW) = (uint8_t)(g_u16FirstTimeHeatHoldSeconds & 0xFF);

				*(pTxBlkPtr + HYDROGEN_ACTIVATED_STEP1_PUMP_SPD_HIGH) = (uint8_t)((g_stRichHydrogenModePara.ActiveStep1PumpSpd & 0xFF00) >> 8);
				*(pTxBlkPtr + HYDROGEN_ACTIVATED_STEP1_PUMP_SPD_LOW) = (uint8_t)(g_stRichHydrogenModePara.ActiveStep1PumpSpd & 0xFF);
				*(pTxBlkPtr + HYDROGEN_ACTIVATED_STEP1_HOLD_TIME_HOUR) = (uint8_t)(g_stRichHydrogenModePara.ActiveStep1HoldHour & 0xFF);
				*(pTxBlkPtr + HYDROGEN_ACTIVATED_STEP1_HOLD_TIME_MIN) = (uint8_t)(g_stRichHydrogenModePara.ActiveStep1HoldMin & 0xFF);

				*(pTxBlkPtr + HYDROGEN_ACTIVATED_STEP2_PUMP_SPD_HIGH) = (uint8_t)((g_stRichHydrogenModePara.ActiveStep2PumpSpd & 0xFF00) >> 8);
				*(pTxBlkPtr + HYDROGEN_ACTIVATED_STEP2_PUMP_SPD_LOW) = (uint8_t)(g_stRichHydrogenModePara.ActiveStep2PumpSpd & 0xFF);
				*(pTxBlkPtr + HYDROGEN_ACTIVATED_STEP2_HOLD_TIME_HOUR) = (uint8_t)(g_stRichHydrogenModePara.ActiveStep2HoldHour & 0xFF);
				*(pTxBlkPtr + HYDROGEN_ACTIVATED_STEP2_HOLD_TIME_MIN) = (uint8_t)(g_stRichHydrogenModePara.ActiveStep2HoldMin & 0xFF);
				
				*(pTxBlkPtr + HYDROGEN_ACTIVATED_STEP3_PUMP_SPD_HIGH) = (uint8_t)((g_stRichHydrogenModePara.ActiveStep3PumpSpd & 0xFF00) >> 8);
				*(pTxBlkPtr + HYDROGEN_ACTIVATED_STEP3_PUMP_SPD_LOW) = (uint8_t)(g_stRichHydrogenModePara.ActiveStep3PumpSpd & 0xFF);
				*(pTxBlkPtr + HYDROGEN_ACTIVATED_STEP3_HOLD_TIME_HOUR) = (uint8_t)(g_stRichHydrogenModePara.ActiveStep3HoldHour & 0xFF);
				*(pTxBlkPtr + HYDROGEN_ACTIVATED_STEP3_HOLD_TIME_MIN) = (uint8_t)(g_stRichHydrogenModePara.ActiveStep3HoldMin & 0xFF);
				
				*(pTxBlkPtr + HYDROGEN_ACTIVATED_STEP4_PUMP_SPD_HIGH) = (uint8_t)((g_stRichHydrogenModePara.ActiveStep4PumpSpd & 0xFF00) >> 8);
				*(pTxBlkPtr + HYDROGEN_ACTIVATED_STEP4_PUMP_SPD_LOW) = (uint8_t)(g_stRichHydrogenModePara.ActiveStep4PumpSpd & 0xFF);
				*(pTxBlkPtr + HYDROGEN_ACTIVATED_STEP4_HOLD_TIME_HOUR) = (uint8_t)(g_stRichHydrogenModePara.ActiveStep4HoldHour & 0xFF);
				*(pTxBlkPtr + HYDROGEN_ACTIVATED_STEP4_HOLD_TIME_MIN) = (uint8_t)(g_stRichHydrogenModePara.ActiveStep4HoldMin & 0xFF);
				break;

                default:
                    break;
            }

            //数据报尾段
            *(pTxBlkPtr + END_BYTE_ONE) = 0x5F;
            *(pTxBlkPtr + END_BYTE_TWO) = 0x6F;
            
            OSQPost(&CommInfoQueue,
                    pTxBlkPtr,
                    TX_MSG_MEM_BLK_SIZE,
                    OS_OPT_POST_LIFO, //提高非实时信息实时性
                    &err);
            if(err != OS_ERR_NONE){
                APP_TRACE_INFO(("- ->NRT work info msg post err,err code is %d\n\r",err));
            }              
        } else {
            APP_TRACE_INFO(("The load data type is illegal...\r\n"));
        }
    }else{
        APP_TRACE_INFO(("- ->NRT work info mem blk get err,err code is %d\n\r",err));
    } 
}


/*
***************************************************************************************************
*                                      StoreCfgParaBySingleAndReport()
*
* Description:  存储参数然后上报.
*
* Arguments  :  fp_StoreType:调用的存储函数类型
*
* Returns    :  none.
***************************************************************************************************
*/
void StoreCfgParaBySingleAndReport(uint8_t *i_PrgmRxMsg,uint16_t *i_StoreData,StoreParaBySingleType fp_StoreType,uint8_t i_StoreAddr)
{
    OS_ERR      err;  
    OSSchedLock(&err);

    if(((*(i_PrgmRxMsg + REC_DATA_BYTE_CMD_PARA_SECTION_3) << 8) | (*(i_PrgmRxMsg + REC_DATA_BYTE_CMD_PARA_SECTION_4))) >= 2000) {
        * i_StoreData = 2000;
    } else {
        * i_StoreData  = (*(i_PrgmRxMsg + REC_DATA_BYTE_CMD_PARA_SECTION_3) << 8) | (*(i_PrgmRxMsg + REC_DATA_BYTE_CMD_PARA_SECTION_4));
    }
    
    fp_StoreType(i_StoreData,i_StoreAddr);//调用函数指针指向的专用函数存储
    SendInquireOrConfigurationInfo();
    
    OSSchedUnlock(&err);
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

	if(g_eCAN_BusOnLineFlag == DEF_YES)//CAN总线在线
	{
		SendCanMsgContainNodeId(PRGM_TX_BUFF_SIZE, i_pTxMsg, g_u16GlobalNetWorkId);
//		APP_TRACE_INFO(("Can Tx data:"));
//		for(uint8_t i = 0; i < 60; i++) {
//			APP_TRACE_INFO(("%X ", i_pTxMsg[i]));
//		}
//		APP_TRACE_INFO(("...\n\r"));
	}
	

}

/*
***************************************************************************************************
*                                      ResponsePrgmCmd()
*
* Description:  anwser the command that receive from the control side under the Dubug mode.
*
* Arguments  :  none
*
* Returns    :  none.
***************************************************************************************************
*/
static void ResponsePrgmCmd(uint8_t *i_PrgmRxMsg)
{
    OS_ERR      err;
    uint32_t    u32ErrCode;
    uint8_t     i;

    if((*(i_PrgmRxMsg + REC_DATA_BYTE_HEAD_ONE)     == 0xFC)
            && (*(i_PrgmRxMsg + REC_DATA_BYTE_HEAD_TWO)     == 0xFD)
            && (*(i_PrgmRxMsg + REC_DATA_BYTE_HEAD_THREE)   == 0xFE)
            && (*(i_PrgmRxMsg + REC_DATA_BYTE_END_OF_DATA)   == 0xAA)//报尾
            && (*(i_PrgmRxMsg + REC_DATA_BYTE_HEAD_TARGET_LOCAL_NET_ID)   == g_u16GlobalNetWorkId)//组网ID
//            && (*(i_PrgmRxMsg + RECEIVE_DATA_BYTE_HEAD_TARGET_LOCAL_NET_ID)   == 0x01)//暂不判定子模块ID
      ) {
        switch(*(i_PrgmRxMsg + REC_DATA_BYTE_CMD_TYPE) & 0x0F) { //指令类型码的高4位尚未定义——将其清零
            case CMD_TYPE_DBG://调试指令
                switch(*(i_PrgmRxMsg + REC_DATA_BYTE_CMD_CODE_VALUE)) {
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
                            SetSystemWorkStatu(EN_WAIT_CMD);
                        }

                        break;

                    case DBG_PUMP1_SPEED_INC:
                        PumpSpdInc();
                        break;

                    case DBG_PUMP1_SPEED_DEC:
                        PumpSpdDec();
                        break;

                    case DBG_PUMP1_SPEED_SET_WITH_PARAMETERS:
                        SetPumpCtlSpd((uint16_t)(*(i_PrgmRxMsg + REC_DATA_BYTE_CMD_PARA_SECTION_1) << 8) + * (i_PrgmRxMsg + REC_DATA_BYTE_CMD_PARA_SECTION_2));
                        break;

                    case DBG_HYDRG_SPEED_INC:
                        HydrgFanSpdInc();
                        break;

                    case DBG_HYDRG_SPEED_DEC:
                        HydrgFanSpdDec();
                        break;

                    case DBG_HYDRG_SPEED_SET_WHTI_PARAMETERS:
                        SetHydrgFanCtlSpd((uint16_t)((*(i_PrgmRxMsg + REC_DATA_BYTE_CMD_PARA_SECTION_1) << 8) + * (i_PrgmRxMsg + REC_DATA_BYTE_CMD_PARA_SECTION_2)));
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

            case CMD_TYPE_CFG: //配置类指令
                switch(*(i_PrgmRxMsg + REC_DATA_BYTE_CMD_CODE_VALUE)) {

                    case CONFIG_HYDROGEN_GROUP_RUNNING_PARA:
                        if((EN_WAIT_CMD == GetSystemWorkStatu() || EN_ALARMING == GetSystemWorkStatu())) {
                            if(*(i_PrgmRxMsg + REC_DATA_BYTE_CMD_PARAMETERS_LENGTH) == 6u) {

                                switch((uint8_t)(*(i_PrgmRxMsg + REC_DATA_BYTE_CMD_PARA_SECTION_1))) {
                                    case CONFIG_HEAT_STEP_PARA:
                                        switch((uint8_t)(*(i_PrgmRxMsg + REC_DATA_BYTE_CMD_PARA_SECTION_2))) {
                                            case CONFIG_HEAT_STEP_PUMP_SPD:
                                                StoreCfgParaBySingleAndReport(i_PrgmRxMsg,&g_stStartHydrgPumpSpdPara.PumpSpdIgniterFirstTime,StoreStartHydrgPumpSpdParaBySingle,0);              
                                                break;

                                            case CONFIG_HEAT_STEP_FAN_SPD:
                                                StoreCfgParaBySingleAndReport(i_PrgmRxMsg,&g_stStartHydrogenFanSpdPara.FanSpdIgniterFirstTime,StoreStartHydrgFanSpdParaBySingle,1);              
                                                break;

                                            case CONFIG_HEAT_HOLD_TIME_BY_SEC:
                                                OSSchedLock(&err);
                                                g_u16FirstTimeHeatHoldSeconds = (*(i_PrgmRxMsg + REC_DATA_BYTE_CMD_PARA_SECTION_3) << 8) | (*(i_PrgmRxMsg + REC_DATA_BYTE_CMD_PARA_SECTION_4));
                                                StoreFirstTimeHeatHoldSeconds(&g_u16FirstTimeHeatHoldSeconds);
                                                SendInquireOrConfigurationInfo();
                                                OSSchedUnlock(&err);
                                                break;

                                            default:
                                                break;
                                        }

                                        break;

                                    case CONFIG_RUNNING_STATUS_PARA:
                                        switch((uint8_t)(*(i_PrgmRxMsg + REC_DATA_BYTE_CMD_PARA_SECTION_2))) {
                                            case CONFIG_RUNNING_STEP_PUMP_SPD:
                                                StoreCfgParaBySingleAndReport(i_PrgmRxMsg,&g_stStartHydrgPumpSpdPara.PumpSpdIgniterSecondTime,StoreStartHydrgPumpSpdParaBySingle,1);              
                                                break;

                                            case CONFIG_RUNNING_STEP_FAN_SPD:
                                                StoreCfgParaBySingleAndReport(i_PrgmRxMsg,&g_stStartHydrogenFanSpdPara.FanSpdIgniterSecondTime,StoreStartHydrgFanSpdParaBySingle,2);              
                                                break;

                                            default:
                                                break;
                                        }

                                        break;
                                    case CONFIG_RICH_HYDROG_ACTIVE_PARA:
                                        switch((uint8_t)(*(i_PrgmRxMsg + REC_DATA_BYTE_CMD_PARA_SECTION_2))) {
                                            case CONFIG_RICH_HYDROG_ACTIVE_STEP:                         
                                                g_stRichHydrogenModePara.ActiveStep = (uint8_t)(*(i_PrgmRxMsg + REC_DATA_BYTE_CMD_PARA_SECTION_4));
                                                LoadCurrentStepRemainTimePara(g_stRichHydrogenModePara.ActiveStep);
//                                                APP_TRACE_INFO(("ActiveStep %d...\r\n",*(i_PrgmRxMsg + REC_DATA_BYTE_CMD_PARA_SECTION_4)));
                                                break;

                                            case CONFIG_RICH_HYDROG_ACTIVE_STEP_PUMP_SPD:
                                                StoreRichHydrogenModePumpPara(i_PrgmRxMsg,(uint8_t)(*(i_PrgmRxMsg + REC_DATA_BYTE_CMD_PARA_SECTION_4)));
                                                break;
                                            
                                            case CONFIG_RICH_HYDROG_ACTIVE_STEP_FAN_SPD:
//                                                StoreRichHydrogenModeFanPara(i_PrgmRxMsg,(uint8_t)(*(i_PrgmRxMsg + REC_DATA_BYTE_CMD_PARA_SECTION_4)));
                                                break;
                                            
                                            case CONFIG_RICH_HYDROG_ACTIVE_STEP_HOLD_TIME:
                                                StoreRichHydrogenModeHoldTimePara(i_PrgmRxMsg,(uint8_t)(*(i_PrgmRxMsg + REC_DATA_BYTE_CMD_PARA_SECTION_4)));
                                                break;
                                            
                                            case CONFIG_HYDROG_PRODUCER_WORK_MODE_NEXT_TIME:
//                                                HydrogenWorkModeTurnOver();
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

            case CMD_TYPE_REQ://查询、请求指令
                switch(*(i_PrgmRxMsg + REC_DATA_BYTE_CMD_CODE_VALUE)) {
                    case REQUEST_DATA_RETRANSMIT:
                        break;

                    case INQUIRE_HYDROGEN_RUNNING_PARAMETERS:
                        SendInquireOrConfigurationInfo();//上报运行参数
                        break;

                    default:
                        break;
                }

            case CMD_TYPE_CONFIRM://应答、确认指令
                switch(*(i_PrgmRxMsg + REC_DATA_BYTE_CMD_CODE_VALUE)) {
                    case RESPONSE_ALLOCATE_ID_NMB_WITH_PARAMETERS:
                        if((EN_WAIT_CMD == GetSystemWorkStatu() || EN_ALARMING == GetSystemWorkStatu()) && (1 == *(i_PrgmRxMsg + REC_DATA_BYTE_CMD_PARAMETERS_LENGTH))) { //限制参数长度为1
                            OSSchedLock(&err);

                            if((*(i_PrgmRxMsg + REC_DATA_BYTE_CMD_PARAMETERS_LENGTH) <= 255)) {
                                g_u16GlobalNetWorkId = *(i_PrgmRxMsg + REC_DATA_BYTE_CMD_PARA_SECTION_1);
                                StoreGlobalNetWorkID(&g_u16GlobalNetWorkId);
                                SendInquireOrConfigurationInfo();//返回设置参数
                                CAN1_Init();//帧ID变化后需重新初始化CAN过滤器,以接收新的帧ID数据
                            }

                            OSSchedUnlock(&err);
                            APP_TRACE_INFO(("Id set,ID:%d...\n\r", (u8)g_u16GlobalNetWorkId));
                        }

                        break;

                    case RESPONSE_SLAVE_SHUT_DOWN_CMD:
                        u32ErrCode = ((uint32_t) * (i_PrgmRxMsg + REC_DATA_BYTE_CMD_PARA_SECTION_1) << 24)
                                     | ((uint32_t) * (i_PrgmRxMsg + REC_DATA_BYTE_CMD_PARA_SECTION_2) << 16)
                                     | ((uint32_t) * (i_PrgmRxMsg + REC_DATA_BYTE_CMD_PARA_SECTION_3) << 8)
                                     | ((uint32_t) * (i_PrgmRxMsg + REC_DATA_BYTE_CMD_PARA_SECTION_4));

                        switch(*(i_PrgmRxMsg + REC_DATA_BYTE_CMD_PARA_SECTION_5)) {
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
                                        SetShutDownRequestMaskStatu((SYSTEM_ALARM_ADDR_Typedef)i, EN_DELAY, (uint16_t)(*(i_PrgmRxMsg + REC_DATA_BYTE_CMD_PARA_SECTION_6) * 60));//对应的错误屏蔽位清零
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
    SYS_WORK_STATU_Typedef eSysRunningStatu;
    SYSTEM_WORK_MODE_Typedef    eWorkMode;

    eSysRunningStatu = GetSystemWorkStatu();

    if(DEF_OFF == GetHydrgProducerStopDlyStatu() && (DEF_OFF == GetStackStopDlyStatu())) {  //防止关机过程未结束再次开机
        if(eSysRunningStatu == EN_WAIT_CMD) { //等待指令，循环设备自检中
            eWorkMode = GetWorkMode();

            if(eWorkMode != EN_WORK_MODE_FUEL_CELL) {   //制氢或者一体机模式
                OSSchedLock(&err);
                SetSystemWorkStatu(EN_START_PRGM_ONE_FRONT);
            } else {
                OSSchedLock(&err);                   //发电模式直接为运行状态
                SetSystemWorkStatu(EN_RUNNING);
            }

            OSTaskSemPost(&AppTaskStartTCB,   //send to WaittingCommand函数中的任务信号量请求，用于启动
                          OS_OPT_POST_NO_SCHED,
                          &err);
            OSSchedUnlock(&err);
        } else if(eSysRunningStatu == EN_ALARMING) {

            SetSystemWorkStatu(EN_WAIT_CMD);//故障状态下，如果故障排除,再次按下启动键可排除故障
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
    SYS_WORK_STATU_Typedef eSysRunningStatu;

    eSysRunningStatu = GetSystemWorkStatu();

    if((eSysRunningStatu == EN_START_PRGM_ONE_FRONT)
            || (eSysRunningStatu == EN_START_PRGM_ONE_BEHIND)
            || (eSysRunningStatu == EN_START_PRGM_TWO)
            || (eSysRunningStatu == EN_RUNNING)) {
        OSSchedLock(&err);
        SetSystemWorkStatu(EN_SHUTTING_DOWN);

        switch((uint8_t)eSysRunningStatu) { //switch不支持枚举型变量，将其转为uint8_t型
            case(uint8_t)EN_START_PRGM_ONE_FRONT:
                OSTaskSemPost(&AppTaskStartTCB,//结束快速加热
                              OS_OPT_POST_NO_SCHED,
                              &err);
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
*                                      SendRealTimeAssistInfo()
*
* Description:  Send real time assist information.
*
* Arguments  :  none
*
* Returns    :  none
***************************************************************************************************
*/
void SendRealTimeAssistInfo(void)
{
    AddNonRealTimeWorkInfoToSendQueue(RT_ASSIST_INFO, NULL, NULL, NULL);
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
    AddNonRealTimeWorkInfoToSendQueue(FOR_QUERY_AND_CFG_INFO, NULL, NULL, NULL);
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
