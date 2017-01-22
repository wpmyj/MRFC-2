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
* Filename      : app_mf210_communicate_task.c
* Version       : V1.00
* Programmer(s) : JiaCai.He
***************************************************************************************************
*/
/*
***************************************************************************************************
*                                             INCLUDE FILES
***************************************************************************************************
*/
#include "app_mf210_communicate_task.h"
#include <includes.h>
#include <bsp.h>
#include "bsp_MF210.h"
#include "app_wireness_communicate_task.h"
#include "bsp_speed_adjust_device.h"
#include <string.h>
#include "bsp_ser.h"
#include "app_stack_manager.h"
/*
***************************************************************************************************
*                                       MICRO DEFINE
***************************************************************************************************
*/
#define MF210_TASK_STK_SIZE 100

#define PRGM_TX_BUFF_SIZE       60
#define PRGM_RX_BUFF_SIZE       16
/*
***************************************************************************************************
*                                         OS-RELATED    VARIABLES
***************************************************************************************************
*/
OS_TCB     MF210_CommunicateTaskTCB;

static CPU_STK    MF210_TASK_STK[MF210_TASK_STK_SIZE];

/*
***************************************************************************************************
*                                        GLOBAL VARIABLES
***************************************************************************************************
*/

u8  Uatr_TxBuf[32]; //制氢半机实时数据
u8  Full_TxBuf[32]; //发电半机实时数据
u8  Uatr_RxBuf[PRGM_3G_RX_BUFF_SIZE];//注意变量数组不要溢出

/*
***************************************************************************************************
*                                         FUNCTION PROTOTYPES
***************************************************************************************************
*/
static void  MF210_Communicate_Task(void *p_arg);


/*
***************************************************************************************************
*                                LoadHydrogenProducerRealTimeInfo()
*
* Description:  Send stack real time info.
*
* Arguments  :  none
*
* Returns    :  none
***************************************************************************************************
*/
static void LoadHydrogenProducerRealTimeInfo(void)
{
    uint32_t  u32IsolatedGenratedEnergyThisTime = 0;
    SYSTEM_TIME_Typedef     stStackWorkTimeThisTime = {0};
        
    Full_TxBuf[0] = 0XF1;  //报头
    Full_TxBuf[1] = 0xF2;
    Full_TxBuf[2] = (u8)((uint16_t)(GetSrcAnaSig(STACK_CURRENT) * 10)%100);                                             //(u8)((uint16_t)(GetSrcAnaSig(STACK_CURRENT))%10);           
    Full_TxBuf[3] = (u8)((uint16_t)(GetSrcAnaSig(STACK_CURRENT) * 10)/100);                                             //* 100))/100;//Ivalue / 100;
    Full_TxBuf[4] = (u8)((uint16_t)(GetSrcAnaSig(STACK_VOLTAGE) * 10)%100);                                             //Vvalue % 100;
    Full_TxBuf[5] = (u8)((uint16_t)(GetSrcAnaSig(STACK_VOLTAGE) * 10)/100);                                             //Vvalue / 100;
    Full_TxBuf[6] = (u8)((uint16_t)((GetSrcAnaSig(STACK_CURRENT) )* (GetSrcAnaSig(STACK_VOLTAGE))) % 100);      //Power / 10000;
    Full_TxBuf[7] = (u8)((uint16_t)((GetSrcAnaSig(STACK_CURRENT) )* (GetSrcAnaSig(STACK_VOLTAGE)))  / 100);      //Power ;    
    Full_TxBuf[8] =  ((uint16_t)GetSrcAnaSig(STACK_TEMP)& 0xFF);                    //Temperature1;
    Full_TxBuf[9] =   GetSrcAnaSig(HYDROGEN_PRESS_1);                               //H2Press;
    
    stStackWorkTimeThisTime = GetStackProductTimeThisTime();
    Full_TxBuf[10] = (uint8_t)(stStackWorkTimeThisTime.second);                    //Full_Timer_Buffer[0];
    Full_TxBuf[11] = (uint8_t)(stStackWorkTimeThisTime.minute);                    //Full_Timer_Buffer[1];
    Full_TxBuf[12] = (uint8_t)(stStackWorkTimeThisTime.hour & 0xFF);               //Full_Timer_Buffer[2];
    Full_TxBuf[13] = (uint8_t)((stStackWorkTimeThisTime.hour & 0xFF00) >> 8);      //Full_Timer_Buffer[3];
    Full_TxBuf[14] = 0;
    
    u32IsolatedGenratedEnergyThisTime = (uint32_t)(GetIsolatedGenratedEnergyThisTime() * 10);
    Full_TxBuf[15] = (uint8_t)((u32IsolatedGenratedEnergyThisTime) % 100);                     //Energy_Array[0];
    Full_TxBuf[16] = (uint8_t)(u32IsolatedGenratedEnergyThisTime  % 10000 / 100);             //Energy_Array[1];
    Full_TxBuf[17] = (uint8_t)((u32IsolatedGenratedEnergyThisTime % 1000000 / 10000));       //Energy_Array[2];
    Full_TxBuf[18] = (uint8_t)(u32IsolatedGenratedEnergyThisTime  / 1000000);                 //Energy_Array[3];
    
    Full_TxBuf[19] = 0;                                                     //H2Towork;
    Full_TxBuf[20] = 0;                                                     //H2ToStop;
    Full_TxBuf[21] = 0;                                                     //LowGes_press_err;
    Full_TxBuf[22] = 0;                                                     //HighGes_press_err;
    Full_TxBuf[23] = 0;                                                     //High_Temper_err;
    Full_TxBuf[24] = 0;                                                     //Vvalue_err;
    Full_TxBuf[25] = 0;
    Full_TxBuf[26] = 0x4E;
    Full_TxBuf[27] = 0x54;
    Full_TxBuf[28] = 0x30;
    Full_TxBuf[29] = 0x30;
    Full_TxBuf[30] = 0x39;
    Full_TxBuf[31] = 0x39;

}
/*
***************************************************************************************************
*                                LoadFuelCellRealTimeInfo()
*
* Description:  Send hydrogen producer real time info.
*
* Arguments  :  none
*
* Returns    :  none
***************************************************************************************************
*/
static void LoadFuelCellRealTimeInfo(void)
{
    SYSTEM_TIME_Typedef     stHydrgProduceTimeThisTime = {0,0,0}, stHydrgProduceTimeTotal = {0,0,0};
    u8 u8DecompressCountPerMin = 0;
    uint16_t u16PumpFeedbackSpeed = 0;
    uint16_t flqdHeight = 0;
    uint16_t u16ReformerTemp  = 0;
    uint16_t u16FireOrRodTemp  = 0;
    
    Uatr_TxBuf[0] = 0xF7; //信道
    Uatr_TxBuf[1] = 0xF8;
    
    u16ReformerTemp = (uint16_t) GetReformerTemp();
    Uatr_TxBuf[2] =   (uint8_t)(u16ReformerTemp  % 100);                                //Date_Temperature1_Low;   //温度1低两位 十位个位
    Uatr_TxBuf[3] =   (uint8_t)(u16ReformerTemp / 100);                                //Date_Temperature1_High;  //温度1高两位 千位百位
    
    u16FireOrRodTemp = (uint16_t)GetFireOrRodTemp();
    Uatr_TxBuf[4] =   (uint8_t)(u16FireOrRodTemp  % 100);                                // Date_Temperature2_Low;   //温度2低两位 十位个位
    Uatr_TxBuf[5] =   (uint8_t)(u16FireOrRodTemp / 100);                               //Date_Temperature2_High;  //温度2高两位 千位百位
    Uatr_TxBuf[6] =  0;                                                                 //HOT;    //加热指示灯
    Uatr_TxBuf[7] =  0;                                                                 //RUN;    //运行指示灯
    Uatr_TxBuf[8] =  0;                                                                 //FIRE;   //点火指示灯
    Uatr_TxBuf[9] =  0;                                                                 //BLOWER; //鼓风指示灯
    Uatr_TxBuf[10] = 0;                                                                 //WATER; //液位指示灯
    
    stHydrgProduceTimeThisTime = GetHydrgProduceTimeThisTime();
    Uatr_TxBuf[11] = stHydrgProduceTimeThisTime.second;                                 //Device_Timer_Buffer[0]; //秒
    Uatr_TxBuf[12] = stHydrgProduceTimeThisTime.minute;                                 //Device_Timer_Buffer[1]; //分
    Uatr_TxBuf[13] = (uint8_t)(stHydrgProduceTimeThisTime.hour & 0xFF);                 //Device_Timer_Buffer[2]; //时 十位个位
    Uatr_TxBuf[14] = (uint8_t)((stHydrgProduceTimeThisTime.hour & 0xFF00) >> 8);        //Device_Timer_Buffer[3]; //时 千位百位
    Uatr_TxBuf[15] = 0;                                                                 //Device_Timer_Buffer[4]; //时 十万位万位
    Uatr_TxBuf[16] = stHydrgProduceTimeTotal.minute;                                    //Device_Time_Buffer[0]; //累计使用次数 十位个位
    Uatr_TxBuf[17] = (uint8_t)(stHydrgProduceTimeTotal.hour & 0xFF);                    //Device_Time_Buffer[1];  //累计使用次数  千位百位
    Uatr_TxBuf[18] = (uint8_t)((stHydrgProduceTimeTotal.hour & 0xFF00) >> 8);           //Device_Time_Buffer[2];  //累计使用次数  十万位万位           
    
    u16PumpFeedbackSpeed = GetPumpFeedBackSpd();                                        //泵速反馈
    Uatr_TxBuf[19] = (uint8_t)(u16PumpFeedbackSpeed / 100);                         //水泵转速千位百位
    Uatr_TxBuf[20] = (uint8_t)(u16PumpFeedbackSpeed  % 100);                        //水泵转速十位个位
    Uatr_TxBuf[21] = GetPumpCtlSpd() / 10 ;                                             //Pump_CountValue;
    Uatr_TxBuf[22] = 0;                                                                 //Get_H2Concentration1;   //氢气泄露报警值0-50PP
    
    flqdHeight = GetSrcAnaSig(LIQUID_LEVEL);
    Uatr_TxBuf[23] = flqdHeight / 2;                                                    //Get_WaterValue1 / 2;  //液位高度: 0-500MM
    Uatr_TxBuf[24] = (uint8_t)((uint16_t)(GetSrcAnaSig(LIQUID_PRESS)*10));
    Uatr_TxBuf[25] = 0;                                                                 //Pump_Blower_Number[0]; //制氢机启动状态反馈信号0x01有效，其他数据无效
    Uatr_TxBuf[26] = 0;                                                                 //Pump_Blower_Number[1]; //制氢机待机状态反馈信号0x02有效，其他数据无效
    Uatr_TxBuf[27] = 0;                                                                 //Pump_Blower_Number[2]; //制氢机运行状态反馈信号0x03有效，其他数据无效
    Uatr_TxBuf[28] = 0;                                                                   //制氢机停机状态反馈信号0x04有效，其他数据无效
    Uatr_TxBuf[29] = 0;                                                                   //制氢机保养提示！1有效
    Uatr_TxBuf[30] = (uint8_t)(GetHydrgFanCurrentCtlSpd() / 100) ;                             //鼓风机值
    
    u8DecompressCountPerMin = GetPassiveDecompressCountPerMinutes();
    Uatr_TxBuf[31] = (uint8_t)(u8DecompressCountPerMin & 0xFF);
}


/*
***************************************************************************************************
*                              RespondRemoteControlCmd()
*
* Description:  Respone promote control cmd.
*
* Arguments  :  none
*
* Returns    :  none
***************************************************************************************************
*/
void RespondRemoteControlCmd(void)
{
    OS_ERR  err;
    if((Uatr_RxBuf[0] == 0xAA) && (Uatr_RxBuf[1] == 0x78))
    {
        if(Uatr_RxBuf[3] == 0)
        { }                                         //优化程序
        else if(Uatr_RxBuf[3] == 0x04)              //启动
        {
            Uatr_RxBuf[3] = 0;
            APP_TRACE_INFO(("RemoteCmd ->Start ...\r\n"));
            SetSystemWorkStatu(EN_START_PRGM_ONE_FRONT);
            OSTaskSemPost(&AppTaskStartTCB,   //send to WaittingCommand函数中的任务信号量请求，用于启动
                          OS_OPT_POST_NO_SCHED,
                          &err);
        }
        else if(Uatr_RxBuf[3] == 0x06)              // 关机
        {
            Uatr_RxBuf[3] = 0;
            APP_TRACE_INFO(("RemoteCmd -> Shutdown ...\r\n"));
            SetSystemWorkStatu(EN_SHUTTING_DOWN);
            OSTaskResume(&AppTaskStartTCB,      //恢复开始任务，进入shutdown程序
             &err);
        }
        else if(Uatr_RxBuf[3] == 0x12)
        {
            Uatr_RxBuf[3] = 0;
            PumpSpdDec();
        }
        else if(Uatr_RxBuf[3] == 0x13)
        {
            Uatr_RxBuf[3] = 0;
            PumpSpdInc();
        }
        else{}
 
    }
}

/*
***************************************************************************************************
*                                SendConsolidatedRealTimeInfo()
*
* Description:  Load fuel cell real time work Info.
*
* Arguments  :  none
*
* Returns    :  none
***************************************************************************************************
*/
void SendConsolidatedRealTimeInfo(void)
{
    uint8_t send_buff[64] = {0};
    LoadHydrogenProducerRealTimeInfo(); //更新发电半机数据
    LoadFuelCellRealTimeInfo();      //更新制氢半机数据
    memcpy(&send_buff[0], Full_TxBuf, 32);
    memcpy(&send_buff[32], Uatr_TxBuf, 32);
    Uart_Send_array1(send_buff, 64);
}

/*
***************************************************************************************************
*                                LoadHydrogenProducerRealTimeInfo()
*
* Description:  Load fuel cell real time work Info.
*
* Arguments  :  none
*
* Returns    :  none
***************************************************************************************************
*/
void MF210_CommunicateTaskCreate(void)
{
    OS_ERR  err;
    
    BSP_Ser_To_3G_Init(115200);//3G模块使用的串口2初始化
    OSTaskCreate((OS_TCB *)&MF210_CommunicateTaskTCB,
                 (CPU_CHAR *)"MF210 Function Task Start",
                 (OS_TASK_PTR)MF210_Communicate_Task,
                 (void *) 0,
                 (OS_PRIO) MF210_COMMUNICATE_TASK_PRIO,
                 (CPU_STK *)&MF210_TASK_STK[0],
                 (CPU_STK_SIZE) MF210_TASK_STK_SIZE / 10,
                 (CPU_STK_SIZE) MF210_TASK_STK_SIZE,
                 (OS_MSG_QTY) 5u,
                 (OS_TICK) 0u,
                 (void *) 0,
                 (OS_OPT)(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR *)&err);
    APP_TRACE_INFO(("Created MF210 communicate Task, and err code is %d...\n\r", err));
}

/*
***************************************************************************************************
*                                MF210_Communicate_Task()
*
* Description:  MF210 module communicate task.
*
* Arguments  :  none
*
* Returns    :  none
***************************************************************************************************
*/
static void  MF210_Communicate_Task(void *p_arg)
{
    OS_ERR  err;
    
    while(DEF_TRUE) {
        
        OSTaskSemPend(OS_CFG_TICK_RATE_HZ,   
              OS_OPT_PEND_BLOCKING,
              NULL,
              &err);
        if(err == OS_ERR_TIMEOUT){
            
            SendConsolidatedRealTimeInfo();//通过3G模块发送整合后的数据给服务器
        }
    }
}










