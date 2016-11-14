#ifndef __SYSTEM_REAL_TIME_PARAMETERS_H
#define __SYSTEM_REAL_TIME_PARAMETERS_H

#include <os.h>
#include <os_cfg_app.h>
#include <bsp_ana_sensor.h>
//�豸�Լ����ÿһλ�Ķ���
#define SelfCheckCodeWorkModeBit_H                      63
#define SelfCheckCodeWorkModeBit_L                      62
#define SelfCheckCodeLegalAuthorizationBit              61
#define SelfCheckCodeGrpComRsvdBit_H                    60
#define SelfCheckCodeGrpComRsvdBit_L                    59
#define SelfCheckCodeGrpComCurrentSensorBit             58

#define SelfCheckCodeGrpHydrgFireThermocoupleBit        57
#define SelfCheckCodeGrpHydrgReformerThermocoupleBit    56
#define SelfCheckCodeGrpHydrgAnaRsvdBit_7               55
#define SelfCheckCodeGrpHydrgAnaRsvdBit_6               54
#define SelfCheckCodeGrpHydrgAnaRsvdBit_5               53
#define SelfCheckCodeGrpHydrgAnaRsvdBit_4               52
#define SelfCheckCodeGrpHydrgAnaRsvdBit_3               51
#define SelfCheckCodeGrpHydrgAnaRsvdBit_2               50
#define SelfCheckCodeGrpHydrgAnaRsvdBit_1               49
#define SelfCheckCodeGrpHydrgAnaRsvdBit_0               48

#define SelfCheckCodeGrpHydrgLqdPressBit                47

#define SelfCheckCodeGrpHydrgPwmRsvdBit_1               46
#define SelfCheckCodeGrpHydrgPwmRsvdBit_0               45

#define SelfCheckCodeGrpHydrgPwmSRVBit                  44
#define SelfCheckCodeGrpHydrgPwmLqdFluxBit              43

#define SelfCheckCodeGrpHydrgSpdRsvdBit_1               42
#define SelfCheckCodeGrpHydrgSpdRsvdBit_0               41

#define SelfCheckCodeGrpHydrgSpdFanBit                  40
#define SelfCheckCodeGrpHydrgSpdPumpBit                 39

#define SelfCheckCodeGrpHydrgSwOutRsvdBit_5             38
#define SelfCheckCodeGrpHydrgSwOutRsvdBit_4             37
#define SelfCheckCodeGrpHydrgSwOutRsvdBit_3             36
#define SelfCheckCodeGrpHydrgSwOutRsvdBit_2             35
#define SelfCheckCodeGrpHydrgSwOutRsvdBit_1             34
#define SelfCheckCodeGrpHydrgSwOutRsvdBit_0             33

#define SelfCheckCodeGrpHydrgSwOutKeepWarmBit           32
#define SelfCheckCodeGrpHydrgSwOutLqdInValve1_Bit       31
#define SelfCheckCodeGrpHydrgSwOutLqdInValve2_Bit       30
#define SelfCheckCodeGrpHydrgSwOutIgniterBit            29
#define SelfCheckCodeGrpHydrgSwOutHeaterBit             28

#define SelfCheckCodeGrpFCAnaRsvdBit_5                  27
#define SelfCheckCodeGrpFCAnaRsvdBit_4                  26
#define SelfCheckCodeGrpFCAnaRsvdBit_3                  25
#define SelfCheckCodeGrpFCAnaRsvdBit_2                  24
#define SelfCheckCodeGrpFCAnaRsvdBit_1                  23
#define SelfCheckCodeGrpFCAnaRsvdBit_0                  22

#define SelfCheckCodeGrpFCAnaCurrentBit                 21
#define SelfCheckCodeGrpFCAnaVoltageBit                 20
#define SelfCheckCodeGrpFCAnaTempBit                    19
#define SelfCheckCodeGrpFCAnaHydrgPressBit              18

#define SelfCheckCodeGrpFCPwmRsvdBit_3                  17
#define SelfCheckCodeGrpFCPwmRsvdBit_2                  16
#define SelfCheckCodeGrpFCPwmRsvdBit_1                  15
#define SelfCheckCodeGrpFCPwmRsvdBit_0                  14

#define SelfCheckCodeGrpFCSpdRsvdBit_1                  13
#define SelfCheckCodeGrpFCSpdRsvdBit_0                  12
#define SelfCheckCodeGrpFCSpdFanPartB_Bit               11
#define SelfCheckCodeGrpFCSpdFanPartA_Bit               10

#define SelfCheckCodeGrpFCSwOutRsvdBit_6                9
#define SelfCheckCodeGrpFCSwOutRsvdBit_5                8
#define SelfCheckCodeGrpFCSwOutRsvdBit_4                7
#define SelfCheckCodeGrpFCSwOutRsvdBit_3                6
#define SelfCheckCodeGrpFCSwOutRsvdBit_2                5
#define SelfCheckCodeGrpFCSwOutRsvdBit_1                4
#define SelfCheckCodeGrpFCSwOutRsvdBit_0                3

#define SelfCheckCodeGrpFCSwOutDCConnectBit             2
#define SelfCheckCodeGrpFCSwOutHydrgOutBit              1
#define SelfCheckCodeGrpFCSwOutHydrgInBit               0

//���о�����λ����
#define AlarmCodeGrpComRsvdBit_3                        31
#define AlarmCodeGrpComRsvdBit_2                        30
#define AlarmCodeGrpComRsvdBit_1                        29
#define AlarmCodeGrpComRsvdBit_0                        28

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


//����״̬��λ����
#define RuningStatuCodeCtrlMode                         31

#define RuningStatuCodeHydrgProducerStatuBit_2          30
#define RuningStatuCodeHydrgProducerStatuBit_1          29
#define RuningStatuCodeHydrgProducerStatuBit_0          28

#define RuningStatuCodeStackStatuBit                    27
#define RuningStatuCodeControlModeSelectWaitFlagBit     26

#define RuningStatuCodeNeighborsNetWorkStatuBit_1       25
#define RuningStatuCodeNeighborsNetWorkStatuBit_0       24

#define RuningStatuCode4G_ConnectStatuBit               23
#define RuningStatuCodeUSB_Slave_ConnectStatuBit        22
#define RuningStatuCodeUSB_Host_ConnectStatuBit         21
#define RuningStatuCodeEtherNetConnectStatuBit          20
#define RuningStatuCode485_BusConnectStatuBit           19
#define RuningStatuCodeCAN_BusConnectStatuBit           18
#define RuningStatuCodeWIFI_ClientConnectStatuBit       17
#define RuningStatuCodeWIFI_ServerConnectStatuBit       16

#define RuningStatuCodeGrpHydrgSwitchOutRsvdBit_2       15
#define RuningStatuCodeGrpHydrgSwitchOutRsvdBit_1       14
#define RuningStatuCodeGrpHydrgSwitchOutRsvdBit_0       13

#define RuningStatuCodeGrpHydrgOut5_Bit                 12
#define RuningStatuCodeGrpHydrgOut4_Bit                 11
#define RuningStatuCodeGrpHydrgOut3_Bit                 10
#define RuningStatuCodeGrpHydrgOut2_Bit                 9
#define RuningStatuCodeGrpHydrgOut1_Bit                 8

#define RuningStatuCodeGrpStackSwitchOutRsvdBit_4       7
#define RuningStatuCodeGrpStackSwitchOutRsvdBit_3       6
#define RuningStatuCodeGrpStackSwitchOutRsvdBit_2       5
#define RuningStatuCodeGrpStackSwitchOutRsvdBit_1       4
#define RuningStatuCodeGrpStackSwitchOutRsvdBit_0       3

#define RuningStatuCodeGrpStackSwitchOut3_Bit           2
#define RuningStatuCodeGrpStackSwitchOut2_Bit           1
#define RuningStatuCodeGrpStackSwitchOut1_Bit           0

typedef struct
{
    uint16_t    hour;
    uint8_t     minute;
    uint8_t     second;
} SYSTEM_TIME_Typedef;

//ϵͳ����״̬״̬
typedef enum
{
    EN_WAITTING_COMMAND = 0,            //����״̬
    EN_START_PRGM_ONE_FRONT = 1,        //���������׶μ�����Ƚ׶�
    EN_START_PRGM_ONE_BEHIND = 2,       //��һ�ε�����û�����Ƚ׶�
    EN_START_PRGM_TWO = 3,              //�ڶ��ε��
    EN_RUNNING = 4,                     //Ѳ�����н׶�
    EN_KEEPING_WARM = 5,                //����״̬
    EN_ALARMING = 6,                    //����״̬
    EN_SHUTTING_DOWN = 7,               //�ػ�״̬
} SYSTEM_WORK_STATU_Typedef;

typedef enum
{
    EN_NOT_IN_WORK = 0,
    EN_IN_WORK = !EN_NOT_IN_WORK,
} STACK_WORK_STATU_Typedef;

typedef enum
{
    EN_WORK_MODE_HYDROGEN_PRODUCER_AND_FUEL_CELL,   //����ͷ���ģʽ(һ���ģʽ)
    EN_WORK_MODE_HYDROGEN_PRODUCER,                 //����ģʽ
    EN_WORK_MODE_FUEL_CELL,                         //����ģʽ
    EN_WORK_MODE_MALFUNCTION,                       //����ģʽ
} SYSTEM_WORK_MODE_Typedef;

typedef enum
{
    EN_CONTROL_MODE_AUTO = 0,
    EN_CONTROL_MODE_MANNUAL = !EN_CONTROL_MODE_AUTO,
} SYSTEM_CONTROL_MODE_Typedef;

typedef enum
{
    OFF = 0,
    ON = !OFF,
} SWITCH_TYPE_VARIABLE_Typedef;

typedef enum
{
    NO = 0,
    YES = !NO,
} WHETHER_TYPE_VARIABLE_Typedef;

typedef enum
{
    STACK_TEMP_LOW_ALARM = 0,           //��ѵ��¾���
    STACK_TEMP_HIGH_ALARM,              //��Ѹ��¾���

    STACK_VOLTAGE_LOW_ALARM = 2,        //��ѵ�ѹ����

    HYDROGEN_PRESS_LOW_ALARM = 8,    //����ѹ����
    HYDROGEN_PRESS_HIGH_ALARM,       //����ѹ����
    HYDORGEN_LEAKAGE_ALARM,

    LIQUID_PRESS_HIGH_ALARM = 16,
    FUEL_SHORTAGE_ALARM,            //��Һλ����
    REFORMER_TEMP_LOW_ALARM,        //�����ҵ��¾���
    REFORMER_TEMP_HIGH_ALARM,       //�����Ҹ��¾���
    IGNITE_FAILED_ALARM,

} SYSTEM_ALARM_ADDR_Typedef;

typedef struct
{
    u8  WorkMode: 2;
    u8  GrpComAuthority: 1;
    u8  GrpComRsvd: 2;
    u8  GrpComCurrentSensor: 1;
    u8  GrpHydrgThermocoupleFire: 1;
    u8  GrpHydrgThermocoupleReformer: 1;

    u8  GrpHydrgAnaSigRsvd: 8;

    u8  GrpHydrgAnaSigLqdPress: 1;
    u8  GrpHydrgPwmInputRsvd: 2;
    u8  GrpHydrgPwmDecompressValve: 1;
    u8  GrpHydrgPwmLqdFlowRateSensor: 1;
    u8  GrpHydrgSpdCtlRsvd: 2;
    u8  GrpHydrgSpdCtlFan: 1;

    u8  GrpHydrgSpdCtlPump: 1;
    u8  GrpHydrgSwTypOutDevRsvd: 6;
    u8  GrpHydrgSwTypOutDevKeepWarm: 1;

    u8  GrpHydrgSwTypOutDevLqdInValve2: 1;
    u8  GrpHydrgSwTypOutDevLqdInValve1: 1;
    u8  GrpHydrgSwTypOutDevIgniter: 1;
    u8  GrpHydrgSwTypOutDevHeater: 1;
    u8  GrpFcAnaSigRsvdHig4: 4;

    u8  GrpFcAnaSigRsvdLow2: 2;
    u8  GrpFcAnaSigStackCurrent: 1;
    u8  GrpFcAnaSigStackVoltage: 1;
    u8  GrpFcAnaSigStackTemperature: 1;
    u8  GrpFcAnaSigStackHydrgPress: 1;
    u8  GrpFcPwmInputRsvdHig2: 2;

    u8  GrpFcPwmInputRsvdLow2: 2;
    u8  GrpFcSpdCtlDevRsvd3: 3;
    u8  GrpFcSpdCtlDevStackFan: 1;
    u8  GrpFcSwTypOutDevRsvdHig2: 2;

    u8  GrpFcSwTypOutDevRsvdLow5: 5;
    u8  GrpFcSwTypOutDevDCConnect: 1;
    u8  GrpFcSwTypOutDevHydrgOutValve: 1;
    u8  GrpFcSwTypOutDevHydrgInValve: 1;
} SELF_CHECK_CODE_Typedef;

typedef struct
{
    u8  ComAlarmRsvd: 4;
    u8  GrpHydrgAlarmRsvdHig4: 4;

    u8  GrpHydrgAlarmRsvdLow3: 3;
    u8  GrpHydrgAlarmIgniterFail: 1;
    u8  GrpHydrgAlarmReformTempHig: 1;
    u8  GrpHydrgAlarmReformTempLow: 1;
    u8  GrpHydrgAlarmFuelShortage: 1;
    u8  GrpHydrgAlarmLqdPressHig: 1;

    u8  GrpFcAlarmSrcHydrgRsvd: 5;
    u8  GrpFcAlarmSrcHydrgLeak: 1;
    u8  GrpFcAlarmSrcHydrgPressHig: 1;
    u8  GrpFcAlarmSrcHydrgPressLow: 1;

    u8  GrpFcAlarmRsvd5: 5;
    u8  GrpFcAlarmVoltageLow: 1;
    u8  GrpFcAlarmTemperatureHig: 1;
    u8  GrpFcAlarmTemperatureLow: 1;

} RUNNING_ALARM_CODE_Typedef;

typedef struct
{
    uint32_t    AlarmCode;
    SYSTEM_TIME_Typedef         HoldTime[32];
} RUNNING_ALARM_STATUS_Typedef;

//����ģʽ��غ���
SYSTEM_WORK_MODE_Typedef        GetWorkMode(void);
void                    SetWorkMode(SYSTEM_WORK_MODE_Typedef);
void                    SetWorkModeWaittingForSelectFlag(void);
void                    ResetWorkModeWaittingForSelectFlag(void);
uint8_t                 GetWorkModeWaittingForSelectFlag(void);

//�Զ��ֶ���غ���
void                    ControlModeTurnOver(void);
SYSTEM_CONTROL_MODE_Typedef     GetControlMode(void);

//ϵͳʱ����غ���
SYSTEM_TIME_Typedef             GetSystemTime(void);

//ϵͳ����ʱ����غ���
void                    ResetHydrgProduceTimeThisTime(void);
SYSTEM_TIME_Typedef             GetHydrgProduceTimeThisTime(void);
SYSTEM_TIME_Typedef             GetHydrgProduceTimeTotal(void);

//ϵͳ����ʱ����غ���
void                    ResetStackProductTimeThisTime(void);
SYSTEM_TIME_Typedef             GetStackProductTimeThisTime(void);
SYSTEM_TIME_Typedef             GetStackProductTimeTotal(void);

//ϵͳ��������������غ���
void                    ResetSystemWorkTimes(void);
void                    LoadSystemWorkTimes(u16);
void                    SystemWorkTimesInc(void);
uint16_t                GetSystemWorkTimes(void);

//ϵͳ�������������غ���
void                    ResetHydrgProducerWorkTimes(void);
void                    LoadHydrgProducerWorkTimes(u16);
void                    HydrgProducerWorkTimesInc(void);
uint16_t                GetHydrgProducerWorkTimes(void);


//ϵͳ�������������غ���
void                    ResetStackWorkTimes(void);
void                    LoadStackWorkTimes(u16);
void                    StackWorkTimesInc(void);
uint16_t                GetStackWorkTimes(void);

//ϵͳ����״̬������غ���
void                    SetSystemWorkStatu(SYSTEM_WORK_STATU_Typedef);
SYSTEM_WORK_STATU_Typedef       GetSystemWorkStatu(void);

//������ʾ������״̬������
void SetExternalScreenUpdateStatu(WHETHER_TYPE_VARIABLE_Typedef i_NewStatu);
WHETHER_TYPE_VARIABLE_Typedef GetExternalScreenUpdateStatu(void);


//��ѹ���״̬������غ���
void                    SetStackWorkStatu(STACK_WORK_STATU_Typedef);
STACK_WORK_STATU_Typedef        GetStackWorkStatu(void);

//ϵͳ����������غ���
void                    ResetAllAlarms(void);
uint32_t                GetRunAlarmCode(void);
void                    AlarmCmd(SYSTEM_ALARM_ADDR_Typedef, SWITCH_TYPE_VARIABLE_Typedef);
SWITCH_TYPE_VARIABLE_Typedef    GetAlarmStatu(SYSTEM_ALARM_ADDR_Typedef m_enSystemAlarmKind);
SYSTEM_TIME_Typedef             GetAlarmHoldTime(SYSTEM_ALARM_ADDR_Typedef m_enSystemAlarmKind);

//ϵͳ������������غ���
void                    ResetSystemIsolatedGeneratedEnergyThisTime(void);
void                    UpdateSystemIsolatedGeneratedEnergyThisTime(void);
double                  GetIsolatedGenratedEnergyThisTime(void);

//�Լ��������غ���
void                    SetSelfCheckCodeBit(uint8_t);
void                    ResetSelfCheckCodeBit(uint8_t);
uint64_t                GetSelfCheckCode(void);

//ϵͳ����״̬�������غ���
u32                     GetSystemRunningStatuCode(void);
void                    SetSystemRunningStatuCodeBit(uint8_t);
void                    ResetSystemRunningStatuCodeBit(uint8_t);
void                    SetSystemRunningStatuCodeSysWorkStatuSection(SYSTEM_WORK_STATU_Typedef);

//��ȡϵͳ����
float                   GetCurrentPower(void);

//ʱ��ͳ������
void                    SystemTimeStatTaskCreate(void);

#endif

