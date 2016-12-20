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

/********************************************************************************************************
* Filename      :  bsp_ans_senor.h
* Programmer(s) :  Fanjun
* Version       :  V1.0
* data          :  2016.4.4
* brief         :  This file contains all the functions prototypes for the analog sensor
*                  firmware library.
*********************************************************************************************************/
#ifndef __SYSTEM_REAL_TIME_PARAMETERS_H
#define __SYSTEM_REAL_TIME_PARAMETERS_H
/*
***************************************************************************************************
*                                                 MODULE
*
* Note(s) : (1) This header file is protected from multiple pre-processor inclusion through use of the
*               BSP present pre-processor macro definition.
***************************************************************************************************
*/

/*
***************************************************************************************************
*                                           INCLUDE FILES
***************************************************************************************************
*/
#include <os.h>
#include <os_cfg_app.h>
#include <bsp_ana_sensor.h>
/*
***************************************************************************************************
*                                           MACRO DEFINITIONS
***************************************************************************************************
*/
/*
***************************************************************************************************
*                                           EXPORTED TYPE
***************************************************************************************************
*/
typedef enum {
    EN_LEGAL_AUTHORIZATION = 0,            //合法授权
    EN_ILLEGAL_AUTHORIZATION,              //非法授权
    EN_ON_PROBATION,                       //试用期
    EN_OVERDUE_AUTHORIZATION,              //授权过期

} AUTHORIZATION_CODE_Typedef;


typedef struct {
    uint8_t     DevSelfCheckSensorStatusCode;   //自检传感器状态码
    uint32_t    MachinePartASelfCheckCode;      //半机1自检码
    uint32_t    MachinePartBSelfCheckCode;      //半机2自检码

} SELF_CHECK_CODE_Typedef;

typedef struct {
    uint16_t    hour;
    uint8_t     minute;
    uint8_t     second;
} SYSTEM_TIME_Typedef;


//实时请求信息类型定义
typedef enum {
    NULL_INVALID_REQUEST = 0,
    REQUEST_DISTRIBUTION_NETWORK_ID,
    REQUEST_SHUT_DONE,
    REQUEST_CHOOSE_WORK_MODE,

} REAL_TIME_REQUEST_INF_Typedef;


typedef enum {
    EN_UN_MASK,
    EN_MASK,
    EN_DELAY,
} REAL_TIME_REQUEST_MASK_Typedef;

typedef struct {
    REAL_TIME_REQUEST_MASK_Typedef      MaskStatu;
    uint16_t                            DelaySecond;
    SYSTEM_TIME_Typedef                 RecordStartTime;
} SHUT_DOWN_REQUEST_RESPONSE_WAIT_MASK_Typedef;

//系统运行状态状态
typedef enum {

    EN_WAITTING_COMMAND = 0,            //待机状态
    EN_START_PRGM_ONE_FRONT,        //加热准备(快速启动阶段即电加热阶段)
    EN_START_PRGM_ONE_BEHIND ,       //加热中(第一次点火后采用火焰加热阶段)
    EN_START_PRGM_TWO ,          //第二次点火
    EN_RUNNING,                 //巡航运行阶段
    EN_KEEPING_WARM,                //保温状态
    EN_ALARMING,                     //故障状态
    EN_SHUTTING_DOWN = 7,               //关机状态
} SYSTEM_WORK_STATU_Typedef;

typedef enum {
    EN_NOT_IN_WORK = 0,
    EN_IN_WORK = !EN_NOT_IN_WORK,
} STACK_WORK_STATU_Typedef;

typedef enum {
    EN_WORK_MODE_HYDROGEN_PRODUCER_AND_FUEL_CELL,   //制氢和发电模式(一体机模式)
    EN_WORK_MODE_HYDROGEN_PRODUCER,                 //制氢模式
    EN_WORK_MODE_FUEL_CELL,                         //发电模式
    EN_WORK_MODE_MALFUNCTION,                       //故障模式
} SYSTEM_WORK_MODE_Typedef;


typedef enum {
    EN_CONTROL_MODE_AUTO = 0,
    EN_CONTROL_MODE_MANNUAL = !EN_CONTROL_MODE_AUTO,
} SYSTEM_CONTROL_MODE_Typedef;

typedef enum {
    OFF = 0,
    ON = !OFF,
} SWITCH_TYPE_VARIABLE_Typedef;

typedef enum {
    NO = 0,
    YES = !NO,
} WHETHER_TYPE_VARIABLE_Typedef;

typedef enum {
    STACK_TEMP_LOW_ALARM = 0,           //电堆低温警报
    STACK_TEMP_HIGH_ALARM,              //电堆高温警报

    STACK_VOLTAGE_LOW_ALARM = 2,               //电堆低压警报

    HYDROGEN_PRESS_LOW_ALARM = 8,     //低气压警报
    HYDROGEN_PRESS_HIGH_ALARM,       //高气压警报
    HYDORGEN_LEAKAGE_ALARM,             //气体泄漏报警

    LIQUID_PRESS_HIGH_ALARM = 16,
    FUEL_SHORTAGE_ALARM,           //低液位警报
    REFORMER_TEMP_LOW_ALARM,      //重整室低温警报
    REFORMER_TEMP_HIGH_ALARM,    //重整室高温警报
    IGNITE_FAILED_ALARM,

} SYSTEM_ALARM_ADDR_Typedef;


typedef struct {
    uint64_t u32_TimeRecordNum; //计时基数,加1为加0.001s
    float  fVentAirTimeIntervalValue;//排气间隔
    float  fDecompressVentTimeValue;//泄压时间

} STACK_VENTING_TIME_PARAMETER_Typedef;

typedef struct {
    uint32_t    AlarmCode;
    SYSTEM_TIME_Typedef         HoldTime[32];
} RUNNING_ALARM_STATUS_Typedef;
/*
***************************************************************************************************
*                                           EXPORTED GLOBAL VARIABLE DECLARATIONS
***************************************************************************************************
*/

/*
***************************************************************************************************
*                                           EXPORTED MACRO
***************************************************************************************************
*/
//设备自检码的位定义
//自检码1-自检传感器状态码
#define SelfCheckCodeSensorStatusRsvdBit_7                      7
#define SelfCheckCodeSensorStatusRsvdBit_6                      6
#define SelfCheckCodeSensorStatusRsvdBit_5                      5
#define SelfCheckCodeSensorStatusRsvdBit_4                      4
#define SelfCheckCodeSensorStatusRsvdBit_3                      3
#define SelfCheckCodeSensorStatusRsvdBit_2                      2
#define SelfCheckCodeSensorStatusRsvdBit_1                      1
#define SelfCheckCodeCurrentSensorStatusBit                     0

//自检码2--半机1设备自检状态
#define SelfCheckCodeGrpHydrgFireThermocoupleBit            31
#define SelfCheckCodeGrpHydrgReformerThermocoupleBit    30
#define SelfCheckCodeGrpHydrgAnaRsvdBit_8                       29
#define SelfCheckCodeGrpHydrgAnaRsvdBit_7                       28
#define SelfCheckCodeGrpHydrgAnaRsvdBit_6                       27
#define SelfCheckCodeGrpHydrgAnaRsvdBit_5                       26
#define SelfCheckCodeGrpHydrgAnaRsvdBit_4                       25
#define SelfCheckCodeGrpHydrgAnaRsvdBit_3                       24
#define SelfCheckCodeGrpHydrgAnaRsvdBit_2                       23
#define SelfCheckCodeGrpHydrgAnaRsvdBit_1                       22
#define SelfCheckCodeGrpHydrgAnaRsvdBit_0                       21

#define SelfCheckCodeGrpHydrgLqdPressBit                            20

#define SelfCheckCodeGrpHydrgPwmRsvdBit_1                       19
#define SelfCheckCodeGrpHydrgPwmRsvdBit_0                       18
#define SelfCheckCodeGrpHydrgPwmSRVBit                              17
#define SelfCheckCodeGrpHydrgPwmLqdFluxBit                      16

#define SelfCheckCodeGrpHydrgSpdRsvdBit_1                           15
#define SelfCheckCodeGrpHydrgSpdRsvdBit_0                           14
#define SelfCheckCodeGrpHydrgSpdFanBit                              13
#define SelfCheckCodeGrpHydrgSpdPumpBit                             12

#define SelfCheckCodeGrpHydrgSwOutRsvdBit_6                     11
#define SelfCheckCodeGrpHydrgSwOutRsvdBit_5                     10
#define SelfCheckCodeGrpHydrgSwOutRsvdBit_4                     9
#define SelfCheckCodeGrpHydrgSwOutRsvdBit_3                     8
#define SelfCheckCodeGrpHydrgSwOutRsvdBit_2                     7
#define SelfCheckCodeGrpHydrgSwOutRsvdBit_1                     6
#define SelfCheckCodeGrpHydrgSwOutRsvdBit_0                     5

#define SelfCheckCodeGrpHydrgSwOutKeepWarmBit               4
#define SelfCheckCodeGrpHydrgSwOutLqdInValve1_Bit       3
#define SelfCheckCodeGrpHydrgSwOutLqdInValve2_Bit       2
#define SelfCheckCodeGrpHydrgSwOutIgniterBit                1
#define SelfCheckCodeGrpHydrgSwOutHeaterBit                 0

//自检码3--半机2(Fuel cell)设备自检状态
#define SelfCheckCodeGrpFCAnaRsvdBit_7                      31
#define SelfCheckCodeGrpFCAnaRsvdBit_6                      30
#define SelfCheckCodeGrpFCAnaRsvdBit_5                      29
#define SelfCheckCodeGrpFCAnaRsvdBit_4                      28
#define SelfCheckCodeGrpFCAnaRsvdBit_3                      27
#define SelfCheckCodeGrpFCAnaRsvdBit_2                      26
#define SelfCheckCodeGrpFCAnaRsvdBit_1                      25
#define SelfCheckCodeGrpFCAnaRsvdBit_0                      24
#define SelfCheckCodeGrpFCAnaCurrentBit                     23
#define SelfCheckCodeGrpFCAnaVoltageBit                     22
#define SelfCheckCodeGrpFCAnaTempBit                        21  //电堆温度
#define SelfCheckCodeGrpFCAnaHydrgPressBit                  20  //气压  
#define SelfCheckCodeGrpFCPwmRsvdBit_3                      19
#define SelfCheckCodeGrpFCPwmRsvdBit_2                      18
#define SelfCheckCodeGrpFCPwmRsvdBit_1                      17
#define SelfCheckCodeGrpFCPwmRsvdBit_0                      16
#define SelfCheckCodeGrpFCSpdRsvdBit_1                      15
#define SelfCheckCodeGrpFCSpdRsvdBit_0                      14
#define SelfCheckCodeGrpFCSpdFanPartB_Bit                   13
#define SelfCheckCodeGrpFCSpdFanPartA_Bit                   12
#define SelfCheckCodeGrpFCSwOutRsvdBit_8                    11
#define SelfCheckCodeGrpFCSwOutRsvdBit_7                    10
#define SelfCheckCodeGrpFCSwOutRsvdBit_6                    9
#define SelfCheckCodeGrpFCSwOutRsvdBit_5                    8
#define SelfCheckCodeGrpFCSwOutRsvdBit_4                    7
#define SelfCheckCodeGrpFCSwOutRsvdBit_3                    6
#define SelfCheckCodeGrpFCSwOutRsvdBit_2                    5
#define SelfCheckCodeGrpFCSwOutRsvdBit_1                    4
#define SelfCheckCodeGrpFCSwOutRsvdBit_0                    3
#define SelfCheckCodeGrpFCSwOutDCConnectBit             2
#define SelfCheckCodeGrpFCSwOutHydrgOutBit              1
#define SelfCheckCodeGrpFCSwOutHydrgInBit               0


//控制/通信状态码位定义
#define ConrtolStatusCodeCtrlMode                                                       15
#define ConrtolStatusCodeNetworkmModes_H                                        14
#define ConrtolStatusCodeNetworkmModes_L                                        13
#define CommunicationStatusCodeWorkModeBit_H                                12
#define CommunicationStatusCodeWorkModeBit_L                          11
#define CommunicationStatusCodeRsvdBit_3                                        10
#define CommunicationStatusCodeRsvdBit_2                                         9
#define CommunicationStatusCodeRsvdBit_1                                         8
#define CommunicationStatusCode3G_ConnectStatuBit                    7
#define CommunicationStatusCodeUSB_Slave_ConnectStatuBit         6
#define CommunicationStatusCodeUSB_Host_ConnectStatuBit          5
#define CommunicationStatusCodeEtherNetConnectStatuBit           4
#define CommunicationStatusCode485_BusConnectStatuBit              3
#define CommunicationStatusCodeCAN_BusConnectStatuBit              2
#define CommunicationStatusCodeWIFI_ClientConnectStatuBit        1
#define CommunicationStatusCodeWIFI_ServerConnectStatuBit        0

//运行警报码位定义
#define AlarmCodeGrpComRsvdBit_3                            31
#define AlarmCodeGrpComRsvdBit_2                            30
#define AlarmCodeGrpComRsvdBit_1                            29
#define AlarmCodeGrpComRsvdBit_0                            28

#define AlarmCodeGrpHydrgRsvdBit_6                      27
#define AlarmCodeGrpHydrgRsvdBit_5                      26
#define AlarmCodeGrpHydrgRsvdBit_4                      25
#define AlarmCodeGrpHydrgRsvdBit_3                      24
#define AlarmCodeGrpHydrgRsvdBit_2                      23
#define AlarmCodeGrpHydrgRsvdBit_1                      22
#define AlarmCodeGrpHydrgRsvdBit_0                      21

#define AlarmCodeGrpHydrgIgniteFailedBit                20
#define AlarmCodeGrpHydrgReformerTempHigBit             19
#define AlarmCodeGrpHydrgReformerTempLowBit             18
#define AlarmCodeGrpHydrgFuelShortageBit                17
#define AlarmCodeGrpHydrgLqdPressHigBit                 16

#define AlarmCodeGrpFCHydrgRsvdBit_4                    15
#define AlarmCodeGrpFCHydrgRsvdBit_3                    14
#define AlarmCodeGrpFCHydrgRsvdBit_2                    13
#define AlarmCodeGrpFCHydrgRsvdBit_1                    12
#define AlarmCodeGrpFCHydrgRsvdBit_0                    11

#define AlarmCodeGrpFCHydrgLeakageBit                   10
#define AlarmCodeGrpFCHydrgPressHigBit                  9
#define AlarmCodeGrpFCHydrgPressLowBit                  8

#define AlarmCodeGrpFCStackRsvdBit_4                    7
#define AlarmCodeGrpFCStackRsvdBit_3                    6
#define AlarmCodeGrpFCStackRsvdBit_2                    5
#define AlarmCodeGrpFCStackRsvdBit_1                    4
#define AlarmCodeGrpFCStackRsvdBit_0                    3

#define AlarmCodeGrpFCStackVoltageLowBit                2
#define AlarmCodeGrpFCStackTempHigBit                   1
#define AlarmCodeGrpFCStackTempLowBit                   0

//运行状态码
#define RuningStatuCodeHydrgenProducerRuningStatuBit_4      31
#define RuningStatuCodeHydrgenProducerRuningStatuBit_3      30
#define RuningStatuCodeHydrgenProducerRuningStatuBit_2      29
#define RuningStatuCodeHydrgenProducerRuningStatuBit_1      28
#define RuningStatuCodeFuelCellRuningStatuBit_4             27
#define RuningStatuCodeFuelCellRuningStatuBit_3             26
#define RuningStatuCodeFuelCellRuningStatuBit_2             25
#define RuningStatuCodeFuelCellRuningStatuBit_1             24

#define RuningStatuCodeGrpHydrgSwitchOutRsvdBit_6           23
#define RuningStatuCodeGrpHydrgSwitchOutRsvdBit_5           22
#define RuningStatuCodeGrpHydrgSwitchOutRsvdBit_4           21
#define RuningStatuCodeGrpHydrgSwitchOutRsvdBit_3           20
#define RuningStatuCodeGrpHydrgSwitchOutRsvdBit_2           19
#define RuningStatuCodeGrpHydrgSwitchOutRsvdBit_1           18
#define RuningStatuCodeGrpHydrgSwitchOutRsvdBit_0           17
#define RuningStatuCodeGrpHydrgOut5_Bit                             16  //保温电磁阀
#define RuningStatuCodeGrpHydrgOut4_Bit                             15  //进液阀2
#define RuningStatuCodeGrpHydrgOut3_Bit                             14  //进液阀1
#define RuningStatuCodeGrpHydrgOut2_Bit                             13  //点火器
#define RuningStatuCodeGrpHydrgOut1_Bit                             12  //加热器

#define RuningStatuCodeGrpStackSwitchOutRsvdBit_8           11
#define RuningStatuCodeGrpStackSwitchOutRsvdBit_7           10
#define RuningStatuCodeGrpStackSwitchOutRsvdBit_6           9
#define RuningStatuCodeGrpStackSwitchOutRsvdBit_5           8
#define RuningStatuCodeGrpStackSwitchOutRsvdBit_4           7
#define RuningStatuCodeGrpStackSwitchOutRsvdBit_3           6
#define RuningStatuCodeGrpStackSwitchOutRsvdBit_2           5
#define RuningStatuCodeGrpStackSwitchOutRsvdBit_1           4
#define RuningStatuCodeGrpStackSwitchOutRsvdBit_0           3

#define RuningStatuCodeGrpStackSwitchOut3_Bit                   2   //直流接触器
#define RuningStatuCodeGrpStackSwitchOut2_Bit                   1   //出气阀
#define RuningStatuCodeGrpStackSwitchOut1_Bit                   0   //进气阀
/*
***************************************************************************************************
*                                           EXPORTED FUNCTION
***************************************************************************************************
*/
//机器权限码相关管理函数
AUTHORIZATION_CODE_Typedef GetAuthorizationCode(void);


//控制通信状态码相关管理函数
uint16_t GetConrolAndCommunicateStatuCode(void);

//工作模式相关函数
SYSTEM_WORK_MODE_Typedef        GetWorkMode(void);
void                    SetWorkMode(SYSTEM_WORK_MODE_Typedef);
void                    SetWorkModeWaittingForSelectFlag(void);
void                    ResetWorkModeWaittingForSelectFlag(void);
uint8_t                 GetWorkModeWaittingForSelectFlag(void);

//自动手动模式相关函数
void                    ControlModeTurnOver(void);
SYSTEM_CONTROL_MODE_Typedef     GetControlMode(void);

//系统时间相关函数
SYSTEM_TIME_Typedef             GetSystemTime(void);
void                     LoadTotalWorkTimeToPrgm(SYSTEM_TIME_Typedef i_stTotalTime);
//系统制氢时间相关函数
void                    ResetHydrgProduceTimeThisTime(void);
SYSTEM_TIME_Typedef             GetHydrgProduceTimeThisTime(void);
SYSTEM_TIME_Typedef             GetHydrgProduceTimeTotal(void);

//系统发电时间相关函数
void                    ResetStackProductTimeThisTime(void);
SYSTEM_TIME_Typedef             GetStackProductTimeThisTime(void);
SYSTEM_TIME_Typedef             GetStackProductTimeTotal(void);

//系统工作次数管理相关函数
void                    ResetSystemWorkTimes(void);
void                    LoadSystemWorkTimesToPrgm(u16 i_u16WorkTimes);
void                    SystemWorkTimesInc(void);
uint16_t                GetSystemWorkTimes(void);

//系统制氢次数管理相关函数
void                    ResetHydrgProducerWorkTimes(void);
void                    LoadHydrgProducerWorkTimes(u16);
void                    HydrgProducerWorkTimesInc(void);
uint16_t                GetHydrgProducerWorkTimes(void);


//系统发电次数管理相关函数
void                    ResetStackWorkTimes(void);
void                    LoadStackWorkTimes(u16);
void                    StackWorkTimesInc(void);
uint16_t                GetStackWorkTimes(void);

//系统工作状态管理相关函数
void                    SetSystemWorkStatu(SYSTEM_WORK_STATU_Typedef);
SYSTEM_WORK_STATU_Typedef       GetSystemWorkStatu(void);

//串口显示屏工作状态管理函数
void SetExternalScreenUpdateStatu(WHETHER_TYPE_VARIABLE_Typedef i_NewStatu);
WHETHER_TYPE_VARIABLE_Typedef GetExternalScreenUpdateStatu(void);


//电堆工作状态管理相关函数
void                    SetStackWorkStatu(STACK_WORK_STATU_Typedef);
STACK_WORK_STATU_Typedef        GetStackWorkStatu(void);

//系统警报管理相关函数
void                    ResetAllAlarms(void);
uint32_t                GetRunAlarmCode(void);
void                    AlarmCmd(SYSTEM_ALARM_ADDR_Typedef, SWITCH_TYPE_VARIABLE_Typedef);
SWITCH_TYPE_VARIABLE_Typedef    GetAlarmStatu(SYSTEM_ALARM_ADDR_Typedef m_enSystemAlarmKind);
SYSTEM_TIME_Typedef             GetAlarmHoldTime(SYSTEM_ALARM_ADDR_Typedef m_enSystemAlarmKind);

//系统发电量管理相关函数
void    ResetSystemIsolatedGeneratedEnergyThisTime(void);
void    UpdateSystemIsolatedGeneratedEnergyThisTime(void);
float   GetIsolatedGenratedEnergyThisTime(void);

//自检码管理相关函数
SELF_CHECK_CODE_Typedef GetSysSelfCheckCode(void);
void SetDevSelfCheckSensorStatusCodeBit(uint8_t i_u8BitNmb);
void SetMachinePartASelfCheckCodeBit(uint8_t i_u8BitNmb);
void SetMachinePartBSelfCheckCodeBit(uint8_t i_u8BitNmb);

void    ResetDevSelfCheckSensorStatusCodeBit(uint8_t i_u8BitNmb);
void    ResetMachinePartASelfCheckCodeBit(uint8_t i_u8BitNmb);
void    ResetMachinePartBSelfCheckCodeBit(uint8_t i_u8BitNmb);


//系统运行状态码管理相关函数
u32     GetSystemRunningStatuCode(void);
void    SetSystemRunningStatuCodeBit(uint8_t);
void    ResetSystemRunningStatuCodeBit(uint8_t);
void    ResetSystemRunningStatuCode(void);
void    SetSystemRunningStatuCodeSysWorkStatuSection(SYSTEM_WORK_STATU_Typedef);

//实时请求信息相关管理函数
void SendRealTimeRequestInf(REAL_TIME_REQUEST_INF_Typedef i_ReqInfType);
uint8_t GetRealTimeRequestInf(void);

//控制通信状态码相关管理函数
void SetConrolAndCommunicateStatuCodeBit(uint8_t i_u8BitNmb);
void ResetConrolAndCommunicateStatuCodeBit(uint8_t i_u8BitNmb);


//获取系统功率
float                   GetCurrentPower(void);

//时间统计任务
void                    SystemTimeStatTaskCreate(void);

//运行指示灯管理函数
void UpMachineLED_Status(void);

//氢气裕度相关函数
uint8_t RecordStackPurifyVentingTimeInterval(void);
uint8_t GetStackPurifyVentingTime(void);

//关机错误码相关函数
void                    SetShutDownRequestMaskStatu(SYSTEM_ALARM_ADDR_Typedef, REAL_TIME_REQUEST_MASK_Typedef, uint16_t);
REAL_TIME_REQUEST_MASK_Typedef  GetShutDownRequestMaskStatu(SYSTEM_ALARM_ADDR_Typedef);
uint32_t                GetSysErrCode(void);


//制氢机延时调节时间函数
u16 GetPrestartHydroFanDlyAdTime(void);
u16 GetRunningHydroFanDlyAdTime(void);



//电堆温度计算相关函数
uint8_t CalcStackOptimumTemperatureByCurrent(void);
uint8_t CalcStackMaximumTemperatureByCurrent(void);
uint8_t CalcStackMinimumTemperatureByCurrent(void);


void    SetHydrogenPressArrivedWaitSwitch(uint8_t i_WaitStatus);
#endif

