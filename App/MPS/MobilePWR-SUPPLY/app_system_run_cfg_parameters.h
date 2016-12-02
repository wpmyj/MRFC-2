#ifndef __APP_SYSTEM_RUN_CFG_PARAMETERS_H__
#define __APP_SYSTEM_RUN_CFG_PARAMETERS_H__

#ifndef __INTERNAL_TEST_FLAG
    #define __INTERNAL_TEST_FLAG    0u
#endif

//重整室参数表
typedef struct
{
    u16 IgFstTimeFrtToBhdTmpPnt;
    u16 IgFstTimeOverTmpPnt;        //第一次点火完成温度
    u8  IgFstTimeWatiTimeMax1;
    u8  IgFstTimeWatiTimeMax2;

    u16 IgScdTimeFrtToBhdTmpPnt;
    u16 IgScdTimeOverTmpPnt;
    u8  IgScdTimeWatiTimeMax1;
    u8  IgScdTimeWatiTimeMax2;

    u16 AlarmLowerLimit;
    u16 AlarmUpperLimit;

    u16 ShutDownLowerLimit;
    u16 ShutDownUpperLimit;

} REFORMER_TEMP_CMP_LINES_Typedef;

//鼓风机和泵参数表
typedef struct
{
    u16 FanSpdIgniterFirstTime;
    u16 FanSpdIgniterSecondTime;

    u16 PumpSpdIgniterFirstTime;
    u16 PumpSpdIgniterSecondTime;
} HYDROGEN_FAN_AND_PUMP_SPEED_LINES_Typedef;

//液压参数表
typedef struct
{
    float AlarmLowerLimit;      //警报下限1Kpa
    float AlarmUpperLimit;      //警报上限18Kpa
    float ShutDownLowerLimit;   //下限0KPa
    float ShutDownUpperLimit;   //过高20KPa
} LIQUID_PRESSURE_CMP_LINES_Typedef;

// 液位高度检测
typedef struct
{
    float AlarmlowerLiquidLevellimit;


} LIQUID_HEIGHT_CMP_LINES_Typedef;

void LoadParameters(void);
void SaveSystemWorkTimes(void);
#endif


