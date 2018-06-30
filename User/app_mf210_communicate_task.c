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
#include "bsp_MF210v2.h"
#include "app_wireness_communicate_task.h"
#include "bsp_speed_adjust_device.h"
#include <string.h>
#include "bsp_ser.h"
#include "app_stack_manager.h"
#include "app_system_run_cfg_parameters.h"
/*
***************************************************************************************************
*                                       MICRO DEFINE
***************************************************************************************************
*/
#define MF210_TASK_STK_SIZE 	  200

#define PRGM_TX_BUFF_SIZE       	60
#define PRGM_RX_BUFF_SIZE       	16
#define REMOTE_TX_BUFF_SIZE			64			//远程数据包大小
#define REMOTE_FC_TX_BUFF_SIZE		32
#define REMOTE_HP_TX_BUFF_SIZE		32
/*
***************************************************************************************************
*                                         OS-RELATED    VARIABLES
***************************************************************************************************
*/
		OS_TCB    MF210_CommunicateTaskTCB;

static CPU_STK    MF210_TASK_STK[MF210_TASK_STK_SIZE];

/*
***************************************************************************************************
*                                        GLOBAL VARIABLES
***************************************************************************************************
*/

u8  HydrogenProducerTxBuf[REMOTE_HP_TX_BUFF_SIZE] = {0}; //制氢半机实时数据
u8  FuelCellTxBuf[REMOTE_FC_TX_BUFF_SIZE] = {0}; //发电半机实时数据
u8  Uart2RxBuf[PRGM_3G_RX_BUFF_SIZE] = {0};//注意变量数组不要溢出

/*
***************************************************************************************************
*                                         FUNCTION PROTOTYPES
***************************************************************************************************
*/
static void  MF210_Communicate_Task(void *p_arg);


/*
***************************************************************************************************
*                                LoadFuelCellRealTimeInfo()
*
* Description:  Send stack real time info.
*
* Arguments  :  none
*
* Returns    :  none
***************************************************************************************************
*/
static void LoadFuelCellRealTimeInfo(void)
{
    uint32_t  u32IsolatedGenratedEnergyThisTime = 0;
    SYSTEM_TIME_Typedef     stStackWorkTimeThisTime = {0};
    uint16_t  u16StackCurrent = 0;
    uint16_t  u16StackVoltage = 0;

    FuelCellTxBuf[0] = 0XF1;  //报头
    FuelCellTxBuf[1] = 0xF2;

    u16StackCurrent = (uint16_t)(GetSrcAnaSig(STACK_CURRENT) * 10);
    FuelCellTxBuf[2] = (u8)(u16StackCurrent % 100);        //(u8)((uint16_t)(GetSrcAnaSig(STACK_CURRENT))%10);
    FuelCellTxBuf[3] = (u8)(u16StackCurrent / 100);        //* 100))/100;//Ivalue / 100;

    u16StackVoltage = (uint16_t)(GetSrcAnaSig(STACK_VOLTAGE) * 10);
    FuelCellTxBuf[4] = (u8)(u16StackVoltage % 100);                                             //Vvalue % 100;
    FuelCellTxBuf[5] = (u8)(u16StackVoltage / 100);                                             //Vvalue / 100;

    FuelCellTxBuf[6] = (u8)((uint16_t)((GetSrcAnaSig(STACK_CURRENT)) * (GetSrcAnaSig(STACK_VOLTAGE))) % 100);      //Power / 10000;
    FuelCellTxBuf[7] = (u8)((uint16_t)((GetSrcAnaSig(STACK_CURRENT)) * (GetSrcAnaSig(STACK_VOLTAGE)))  / 100);      //Power ;
    FuelCellTxBuf[8] = ((uint16_t)GetSrcAnaSig(STACK_TEMP) & 0xFF);                    //Temperature1;
    FuelCellTxBuf[9] =   GetSrcAnaSig(HYDROGEN_PRESS_1);                               //H2Press;

    stStackWorkTimeThisTime = GetStackProductTimeThisTime();
    FuelCellTxBuf[10] = (uint8_t)(stStackWorkTimeThisTime.second);                    //Full_Timer_Buffer[0];
    FuelCellTxBuf[11] = (uint8_t)(stStackWorkTimeThisTime.minute);                    //Full_Timer_Buffer[1];
    FuelCellTxBuf[12] = (uint8_t)(stStackWorkTimeThisTime.hour & 0xFF);               //Full_Timer_Buffer[2];
    FuelCellTxBuf[13] = (uint8_t)((stStackWorkTimeThisTime.hour & 0xFF00) >> 8);      //Full_Timer_Buffer[3];
    FuelCellTxBuf[14] = 0;

    u32IsolatedGenratedEnergyThisTime = (uint32_t)(GetIsolatedGenratedEnergyThisTime() * 10);
    FuelCellTxBuf[15] = (uint8_t)((u32IsolatedGenratedEnergyThisTime) % 100);                     //Energy_Array[0];
    FuelCellTxBuf[16] = (uint8_t)(u32IsolatedGenratedEnergyThisTime  % 10000 / 100);             //Energy_Array[1];
    FuelCellTxBuf[17] = (uint8_t)((u32IsolatedGenratedEnergyThisTime % 1000000 / 10000));       //Energy_Array[2];
    FuelCellTxBuf[18] = (uint8_t)(u32IsolatedGenratedEnergyThisTime  / 1000000);                 //Energy_Array[3];

    FuelCellTxBuf[19] = 0;                                                     //H2Towork;
    FuelCellTxBuf[20] = 0;                                                     //H2ToStop;
    FuelCellTxBuf[21] = 0;                                                     //LowGes_press_err;
    FuelCellTxBuf[22] = 0;                                                     //HighGes_press_err;
    FuelCellTxBuf[23] = 0;                                                     //High_Temper_err;
    FuelCellTxBuf[24] = 0;                                                     //Vvalue_err;
    FuelCellTxBuf[25] = 0;
	//载入从Flash中读取的机器ID
    FuelCellTxBuf[26] = g_stMachineIdPara.byte1;//0x4E
    FuelCellTxBuf[27] = g_stMachineIdPara.byte2;//0x54
    FuelCellTxBuf[28] = g_stMachineIdPara.byte3;//0x30
    FuelCellTxBuf[29] = g_stMachineIdPara.byte4;//0x30
    FuelCellTxBuf[30] = g_stMachineIdPara.byte5;//0x39
    FuelCellTxBuf[31] = g_stMachineIdPara.byte6;//0x39

}
/*
***************************************************************************************************
*                                LoadHydrogenProducerRealTimeInfo()
*
* Description:  Send hydrogen producer real time info.
*
* Arguments  :  none
*
* Returns    :  none
***************************************************************************************************
*/
static void LoadHydrogenProducerRealTimeInfo(void)
{
    SYSTEM_TIME_Typedef     stHydrgProduceTimeThisTime = {0, 0, 0}, stHydrgProduceTimeTotal = {0, 0, 0};
    u8 u8DecompressCountPerMin = 0;
    uint8_t  u8LiquifPress = 0;
    uint16_t u16PumpFeedbackSpeed = 0;
    uint16_t flqdHeight = 0;
    uint16_t u16ReformerTemp  = 0;
    uint16_t u16FireOrRodTemp  = 0;
	uint16_t u16BatteryVoltage = 0;
	uint16_t u16LiquidLevel1 = 0;

    HydrogenProducerTxBuf[0] = 0xF7; //信道
    HydrogenProducerTxBuf[1] = 0xF8;

    u16ReformerTemp = (uint16_t) GetReformerTemp();
    HydrogenProducerTxBuf[2] = (uint8_t)(u16ReformerTemp  % 100);                                  //Date_Temperature1_Low;   //温度1低两位 十位个位
    HydrogenProducerTxBuf[3] = (uint8_t)(u16ReformerTemp / 100);                                  //Date_Temperature1_High;  //温度1高两位 千位百位

    u16FireOrRodTemp = (uint16_t)GetFireOrRodTemp();
    HydrogenProducerTxBuf[4] = (uint8_t)(u16FireOrRodTemp  % 100);                                  // Date_Temperature2_Low;   //温度2低两位 十位个位
    HydrogenProducerTxBuf[5] = (uint8_t)(u16FireOrRodTemp / 100);                                 //Date_Temperature2_High;  //温度2高两位 千位百位
    HydrogenProducerTxBuf[6] =  0;                                                                 //HOT;    //加热指示灯
    HydrogenProducerTxBuf[7] =  0;                                                                 //RUN;    //运行指示灯
    HydrogenProducerTxBuf[8] =  0;                                                                 //FIRE;   //点火指示灯
    HydrogenProducerTxBuf[9] =  0;                                                                 //BLOWER; //鼓风指示灯
    HydrogenProducerTxBuf[10] = 0;                                                                 //WATER; //液位指示灯

    stHydrgProduceTimeThisTime = GetHydrgProduceTimeThisTime();
    HydrogenProducerTxBuf[11] = stHydrgProduceTimeThisTime.second;                                 //Device_Timer_Buffer[0]; //秒
    HydrogenProducerTxBuf[12] = stHydrgProduceTimeThisTime.minute;                                 //Device_Timer_Buffer[1]; //分
    HydrogenProducerTxBuf[13] = (uint8_t)(stHydrgProduceTimeThisTime.hour & 0xFF);                 //Device_Timer_Buffer[2]; //时 十位个位
    HydrogenProducerTxBuf[14] = (uint8_t)((stHydrgProduceTimeThisTime.hour & 0xFF00) >> 8);        //Device_Timer_Buffer[3]; //时 千位百位
    HydrogenProducerTxBuf[15] = 0;                                                                 //Device_Timer_Buffer[4]; //时 十万位万位
    HydrogenProducerTxBuf[16] = stHydrgProduceTimeTotal.minute;                                    //Device_Time_Buffer[0]; //累计使用次数 十位个位
    HydrogenProducerTxBuf[17] = (uint8_t)(stHydrgProduceTimeTotal.hour & 0xFF);                    //Device_Time_Buffer[1];  //累计使用次数  千位百位
    HydrogenProducerTxBuf[18] = (uint8_t)((stHydrgProduceTimeTotal.hour & 0xFF00) >> 8);           //Device_Time_Buffer[2];  //累计使用次数  十万位万位

    u16PumpFeedbackSpeed = GetPumpFeedBackSpd();                                        		//泵速反馈
    HydrogenProducerTxBuf[19] = (uint8_t)(u16PumpFeedbackSpeed / 100);                         //水泵转速千位百位
    HydrogenProducerTxBuf[20] = (uint8_t)(u16PumpFeedbackSpeed  % 100);                        //水泵转速十位个位
    HydrogenProducerTxBuf[21] = GetPumpCurrentCtrlSpd() / 10 ;                                             //Pump_CountValue;
    HydrogenProducerTxBuf[22] = 0;                                                                 //Get_H2Concentration1;   //氢气泄露报警值0-50PP

    flqdHeight = GetSrcAnaSig(LIQUID_LEVEL1);
    HydrogenProducerTxBuf[23] = flqdHeight / 2;                                                    //Get_WaterValue1 / 2;  //液位高度: 0-500MM

    u8LiquifPress = (uint8_t)((uint16_t)(GetSrcAnaSig(LIQUID_PRESS) * 10));
    HydrogenProducerTxBuf[24] = u8LiquifPress;
    HydrogenProducerTxBuf[25] = 20;                                                                 //预留电池电流
    HydrogenProducerTxBuf[26] = 20;                                                                 //预留电池电流
	
	u16BatteryVoltage = (uint16_t)(GetSrcAnaSig(BATTERY_VOLTAGE) * 100);
    HydrogenProducerTxBuf[27] = u16BatteryVoltage % 100;                                            //电池电压
    HydrogenProducerTxBuf[28] = u16BatteryVoltage / 100;                                            //电池电压
	
	u16LiquidLevel1 = (uint16_t)(GetSrcAnaSig(LIQUID_LEVEL2) * 0.5);				//大水箱液位要做/2处理
    HydrogenProducerTxBuf[29] = u16LiquidLevel1;                                                                 //大水箱液位高度
    HydrogenProducerTxBuf[30] = (uint8_t)(GetHydrgFanCurrentCtlSpd() / 100) ;                             //鼓风机值

    u8DecompressCountPerMin = GetPassiveDecompressCntPerMin();
    HydrogenProducerTxBuf[31] = (uint8_t)(u8DecompressCountPerMin & 0xFF);
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

    if((Uart2RxBuf[0] == 0xAA) && (Uart2RxBuf[1] == 0x78)) {
        if(Uart2RxBuf[3] == 0) {//优化程序
			
        }else if(Uart2RxBuf[3] == 0x04) {            //启动
            Uart2RxBuf[3] = 0;
            APP_TRACE_INFO(("RemoteCmd ->Start ...\r\n"));
            SetSystemWorkStatu(EN_START_PRGM_ONE_FRONT);
            OSTaskSemPost(&AppTaskStartTCB,   //send to WaittingCommand函数中的任务信号量请求，用于启动
                          OS_OPT_POST_NO_SCHED,
                          &err);
        } else if(Uart2RxBuf[3] == 0x06) {          // 关机
            Uart2RxBuf[3] = 0;
            APP_TRACE_INFO(("RemoteCmd -> Shutdown ...\r\n"));
            SetSystemWorkStatu(EN_SHUTTING_DOWN);
            OSTaskResume(&AppTaskStartTCB,      //恢复开始任务，进入shutdown程序
                         &err);
        } else if(Uart2RxBuf[3] == 0x12) {
            Uart2RxBuf[3] = 0;
            PumpSpdDec();
        } else if(Uart2RxBuf[3] == 0x13) {
            Uart2RxBuf[3] = 0;
            PumpSpdInc();
        } else {}

    }
}

/*
***************************************************************************************************
*                                MF210_CommunicateTaskCreate()
*
* Description:  MF210 （3G/4G） module communicate task create.
*
* Arguments  :  none
*
* Returns    :  none
***************************************************************************************************
*/
void MF210_CommunicateTaskCreate(void)
{
    OS_ERR  err;

    BSP_Ser_To_3G_Init(115200);//uart2 init
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
	uint8_t send_buff[64] = {0};

    while(DEF_TRUE) {
		
		OSTimeDlyHMSM(0, 0, 1, 000,OS_OPT_TIME_HMSM_STRICT,&err);
		
		if(SocketRecv(SOCKET_ID, Uart2RxBuf)) {
            RespondRemoteControlCmd();    //接收服务器控制指令
        }
				
		LoadFuelCellRealTimeInfo(); 				
		LoadHydrogenProducerRealTimeInfo();      	
		memcpy(&send_buff[0], FuelCellTxBuf, 32);
		memcpy(&send_buff[32], HydrogenProducerTxBuf, 32);
		Uart2SendBuffer(send_buff, REMOTE_TX_BUFF_SIZE);
    }
}










