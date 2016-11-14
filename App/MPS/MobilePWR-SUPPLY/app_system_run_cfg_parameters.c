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
* Filename      : app_system_real_time_parameters.c
* Version       : V1.00
* Programmer(s) : SunKing.Yun
*********************************************************************************************************
*/
/*
*********************************************************************************************************
*                                           INCLUDE FILES
*********************************************************************************************************
*/
#include "includes.h"
#include "app_system_run_cfg_parameters.h"
#include "app_system_real_time_parameters.h"

/*
*********************************************************************************************************
*                                           MACRO DEFINITIONS
*********************************************************************************************************
*/
#define STM32_FLASH_SIZE            128             //所用STM32的FLASH容量大小(单位为K)
#define STM32_FLASH_WREN            1               //使能FLASH写入(0:失能;1:使能)

#define STM32_FLASH_BASE            0x08000000      //STM32 FLASH的起始地址

#define PARAMETER_SAVE_ADDR         0x0801FC00      //地址頂端為0x08020000 设置FLASH 保存地址為最後的1K字節(必须为偶数)

//各留出了一个字的空间
#define PARAMETERS_STORE_AREA_SIZE                                      512 //参数保存区的大小（字节）
#define RUN_REAL_PARAMETERS_STORE_OFFSET_ADDR                           0   //起始地址，存储运行次数
#define REAL_TIME_PARAMETERS_STORE_OFFSET_ADDR                          1   //  4个时间，各4个半字位置
#define RUN_REFORMER_CMP_TBL_STORE_OFFSET_ADDR                          20  //12个数
#define RUN_LIQUIDPRESS_CMP_TBL_STORE_OFFSET_ADDR                       35  // 4个数
#define RUN_START_HYDROGEN_FAN_AND_IGNITER_SPEED_STORE_OFFSET_ADDR      40
#define RUN_PURIFY_AMP_INTEGRAL_VALUE                                   45  //电堆提纯膜间电荷量
#define ANA_SENSOR_PARAMETERS_STORE_OFFSET_ADDR                         50
#define RUN_LIQUID_HEIGHT_CMP_TBL_STORE_OFFSET_ADDR                     122

/*
*********************************************************************************************************
*                                           LOCAL VARIABLES
*********************************************************************************************************
*/
REFORMER_TEMP_CMP_LINES_Typedef             g_stReformerTempCmpTbl;
LIQUID_PRESSURE_CMP_LINES_Typedef           g_stLqdPressCmpTbl;
LIQUID_HEIGHT_CMP_LINES_Typedef             g_stLqdHeightCmpTbl;


HYDROGEN_FAN_AND_PUMP_SPEED_LINES_Typedef   g_stStartHydrgFanIgniterSpd;
u16                             g_u16RunPurifyAmpIntegralValue;

static                  uint16_t                        RW_ParametersBuffer[PARAMETERS_STORE_AREA_SIZE / 2];

/*
*********************************************************************************************************
*                                         FUNCTION PROTOTYPES
*********************************************************************************************************
*/
static          u16         STMFLASH_ReadHalfWord(u32 faddr);         //读出半字
static          void        STMFLASH_Write_NoCheck(u32 WriteAddr, u16 *pBuffer, u16 NumToWrite);

static          void        STMFLASH_Write(u32 WriteAddr, u16 *pBuffer, u16 NumToWrite);    //从指定地址开始写入指定长度的数据
static          void        STMFLASH_Read(u32 ReadAddr, u16 *pBuffer, u16 NumToRead);       //从指定地址开始读出指定长度的数据

static          void        GetSavedSystemWorkTimes(u16 *);

static          void        LoadReformerTempCmpTbl(REFORMER_TEMP_CMP_LINES_Typedef *);
static          void        LoadDefaultReformerTempCmpTbl(REFORMER_TEMP_CMP_LINES_Typedef *);
static          void        SaveReformerTempCmpTbl(REFORMER_TEMP_CMP_LINES_Typedef *);

static void LoadLqdHeightCmpTbl(LIQUID_HEIGHT_CMP_LINES_Typedef *i_LqdHeightCmpTbl);
static void LoadDefaultLqdHeightCmpTbl(LIQUID_HEIGHT_CMP_LINES_Typedef *i_LqdHeightCmpTbl);
static void SaveLqdHeightCmpTbl(LIQUID_HEIGHT_CMP_LINES_Typedef *i_LqdHeightCmpTbl);

static          void        LoadLqdPressCmpTbl(LIQUID_PRESSURE_CMP_LINES_Typedef *);
static          void        LoadDefaultLqdPressCmpTbl(LIQUID_PRESSURE_CMP_LINES_Typedef *);
static          void        SaveLqdPressCmpTbl(LIQUID_PRESSURE_CMP_LINES_Typedef *);

static          void        LoadStartHydrgFanAndPumpSpd(HYDROGEN_FAN_AND_PUMP_SPEED_LINES_Typedef *);
static          void        LoadDefaultStartHydrgFanAndPumpSpd(HYDROGEN_FAN_AND_PUMP_SPEED_LINES_Typedef *);
static          void        SaveStartHydrgFanAndPumpSpd(HYDROGEN_FAN_AND_PUMP_SPEED_LINES_Typedef *);

static          void        LoadRunPurifyAmpIntegralValue(u16 *);
static          void        LoadDefaultRunPurifyAmpIntegralValue(u16 *);
static          void        SaveRunPurifyAmpIntegralValue(u16 *);

#if 1

/*
*********************************************************************************************************
*                                      LoadParameters()
*
* Description:  load the paramters to the system variables, run config prameters, memorability real time parameters
*
* Arguments  :  none.
*
* Returns    :  none.
*
* Note(s)    :  none.
*********************************************************************************************************
*/
void LoadParameters()
{
    u16 u16WorkTimes;

    GetSavedSystemWorkTimes(&RW_ParametersBuffer[0]);

    //重载系统运行次数
    if(RW_ParametersBuffer[0] != 0xFFFF)
    {
        LoadSystemWorkTimes(RW_ParametersBuffer[0]);
    }
    else
    {
        LoadSystemWorkTimes(0);
    }

    u16WorkTimes = GetSystemWorkTimes();

    if(u16WorkTimes != 0)
    {
        APP_TRACE_INFO(("This is the %d time for the system to boot,loading the parameters..\n\r", u16WorkTimes + 1));
        STMFLASH_Read(PARAMETER_SAVE_ADDR + 2, &RW_ParametersBuffer[1], PARAMETERS_STORE_AREA_SIZE - 1);
        LoadReformerTempCmpTbl(&g_stReformerTempCmpTbl);
        LoadLqdPressCmpTbl(&g_stLqdPressCmpTbl);
        LoadLqdHeightCmpTbl(&g_stLqdHeightCmpTbl);
        LoadStartHydrgFanAndPumpSpd(&g_stStartHydrgFanIgniterSpd);
        LoadRunPurifyAmpIntegralValue(&g_u16RunPurifyAmpIntegralValue);

        BSP_LoadAnaSensorParameters((float *)&RW_ParametersBuffer[ANA_SENSOR_PARAMETERS_STORE_OFFSET_ADDR]);
    }
    else//载入默认参数
    {
        APP_TRACE_INFO(("This is the first time for the system to boot,capturing the parameters...\n\r"));

        LoadDefaultReformerTempCmpTbl(&g_stReformerTempCmpTbl);
        LoadDefaultLqdPressCmpTbl(&g_stLqdPressCmpTbl);
        LoadDefaultLqdHeightCmpTbl(&g_stLqdHeightCmpTbl);
        LoadDefaultStartHydrgFanAndPumpSpd(&g_stStartHydrgFanIgniterSpd);
        LoadDefaultRunPurifyAmpIntegralValue(&g_u16RunPurifyAmpIntegralValue);

        //载入标准的模拟量传感器的参数
        BSP_LoadCalibratedAnaSensorParameters();

        SaveReformerTempCmpTbl(&g_stReformerTempCmpTbl);
        SaveLqdPressCmpTbl(&g_stLqdPressCmpTbl);
        SaveLqdHeightCmpTbl(&g_stLqdHeightCmpTbl);
        SaveStartHydrgFanAndPumpSpd(&g_stStartHydrgFanIgniterSpd);
        SaveRunPurifyAmpIntegralValue(&g_u16RunPurifyAmpIntegralValue);
        //保存标准的模拟量传感器的参数
        BSP_SaveAnaSensorParameters((float *)&RW_ParametersBuffer[ANA_SENSOR_PARAMETERS_STORE_OFFSET_ADDR]);

        STMFLASH_Write(PARAMETER_SAVE_ADDR, RW_ParametersBuffer, PARAMETERS_STORE_AREA_SIZE);
    }
}

/*
*********************************************************************************************************
*                                      SaveSystemWorkTimes()
*
* Description:  save the system work times to the flash
*
* Arguments  :  none.
*
* Returns    :  none.
*
* Note(s)    :  none.
*********************************************************************************************************
*/
void SaveSystemWorkTimes()
{
    u16 WorkTimes;
    WorkTimes = GetSystemWorkTimes();
    STMFLASH_Write(PARAMETER_SAVE_ADDR + RUN_REAL_PARAMETERS_STORE_OFFSET_ADDR, &WorkTimes, 1);
}

/*
*********************************************************************************************************
*                                      GetSavedSystemWorkTimes()
*
* Description:  Get the system work times that has stored in the flash
*
* Arguments  :  the avriable that store the work times.
*
* Returns    :  none.
*
* Note(s)    :  none.
*********************************************************************************************************
*/
static void GetSavedSystemWorkTimes(u16 *i_StoreAddr)
{
    STMFLASH_Read(PARAMETER_SAVE_ADDR + RUN_REAL_PARAMETERS_STORE_OFFSET_ADDR, i_StoreAddr, 1);//获取运行次数
}

/*
*********************************************************************************************************
*                                      LoadReformerTempCmpTbl()
*
* Description:  load the paramters, that related to the reformer, that has stored in the flash.
*
* Arguments  :  the avriable that store the parameters.
*
* Returns    :  none.
*
* Note(s)    :  none.
*********************************************************************************************************
*/
static void LoadReformerTempCmpTbl(REFORMER_TEMP_CMP_LINES_Typedef *i_ReformerTemCmpTbl)
{
    i_ReformerTemCmpTbl->IgFstTimeFrtToBhdTmpPnt    = RW_ParametersBuffer[RUN_REFORMER_CMP_TBL_STORE_OFFSET_ADDR];
    i_ReformerTemCmpTbl->IgFstTimeOverTmpPnt        = RW_ParametersBuffer[RUN_REFORMER_CMP_TBL_STORE_OFFSET_ADDR + 1];
    i_ReformerTemCmpTbl->IgFstTimeWatiTimeMax1      = RW_ParametersBuffer[RUN_REFORMER_CMP_TBL_STORE_OFFSET_ADDR + 2];
    i_ReformerTemCmpTbl->IgFstTimeWatiTimeMax2      = RW_ParametersBuffer[RUN_REFORMER_CMP_TBL_STORE_OFFSET_ADDR + 3];
    i_ReformerTemCmpTbl->IgScdTimeFrtToBhdTmpPnt    = RW_ParametersBuffer[RUN_REFORMER_CMP_TBL_STORE_OFFSET_ADDR + 4];
    i_ReformerTemCmpTbl->IgScdTimeOverTmpPnt        = RW_ParametersBuffer[RUN_REFORMER_CMP_TBL_STORE_OFFSET_ADDR + 5];
    i_ReformerTemCmpTbl->IgScdTimeWatiTimeMax1      = RW_ParametersBuffer[RUN_REFORMER_CMP_TBL_STORE_OFFSET_ADDR + 6];
    i_ReformerTemCmpTbl->IgScdTimeWatiTimeMax2      = RW_ParametersBuffer[RUN_REFORMER_CMP_TBL_STORE_OFFSET_ADDR + 7];

    i_ReformerTemCmpTbl->AlarmUpperLimit            = RW_ParametersBuffer[RUN_REFORMER_CMP_TBL_STORE_OFFSET_ADDR + 8];
    i_ReformerTemCmpTbl->AlarmLowerLimit            = RW_ParametersBuffer[RUN_REFORMER_CMP_TBL_STORE_OFFSET_ADDR + 9];

    i_ReformerTemCmpTbl->ShutDownLowerLimit         = RW_ParametersBuffer[RUN_REFORMER_CMP_TBL_STORE_OFFSET_ADDR + 10];
    i_ReformerTemCmpTbl->ShutDownUpperLimit         = RW_ParametersBuffer[RUN_REFORMER_CMP_TBL_STORE_OFFSET_ADDR + 11];
}

/*
*********************************************************************************************************
*                                      LoadDefaultReformerTempCmpTbl()
*
* Description:  load the default paramters that related to the reformer.
*
* Arguments  :  the avriable that store the parameters.
*
* Returns    :  none.
*
* Note(s)    :  none.
*********************************************************************************************************
*/
static void LoadDefaultReformerTempCmpTbl(REFORMER_TEMP_CMP_LINES_Typedef *i_ReformerTemCmpTbl)
{
    i_ReformerTemCmpTbl->IgFstTimeFrtToBhdTmpPnt    = NULL;
    i_ReformerTemCmpTbl->IgFstTimeOverTmpPnt        = 50;
    i_ReformerTemCmpTbl->IgFstTimeWatiTimeMax1      = NULL;
    i_ReformerTemCmpTbl->IgFstTimeWatiTimeMax2      = NULL;
    i_ReformerTemCmpTbl->IgScdTimeFrtToBhdTmpPnt    = NULL;
    i_ReformerTemCmpTbl->IgScdTimeOverTmpPnt        = NULL;
    i_ReformerTemCmpTbl->IgScdTimeWatiTimeMax1      = NULL;
    i_ReformerTemCmpTbl->IgScdTimeWatiTimeMax2      = NULL;

    i_ReformerTemCmpTbl->AlarmLowerLimit            = 0;
    i_ReformerTemCmpTbl->AlarmUpperLimit            = 550;
    i_ReformerTemCmpTbl->ShutDownLowerLimit         = 0;
    i_ReformerTemCmpTbl->ShutDownUpperLimit         = 10;
}

/*
*********************************************************************************************************
*                                      SaveReformerTempCmpTbl()
*
* Description:  save the paramters that related to the reformer.
*
* Arguments  :  the buff that store the parameters.
*
* Returns    :  none.
*
* Note(s)    :  none.
*********************************************************************************************************
*/
static void SaveReformerTempCmpTbl(REFORMER_TEMP_CMP_LINES_Typedef *i_ReformerTemCmpTbl)
{
    RW_ParametersBuffer[RUN_REFORMER_CMP_TBL_STORE_OFFSET_ADDR]      = i_ReformerTemCmpTbl->IgFstTimeFrtToBhdTmpPnt;
    RW_ParametersBuffer[RUN_REFORMER_CMP_TBL_STORE_OFFSET_ADDR + 1]  = i_ReformerTemCmpTbl->IgFstTimeOverTmpPnt ;
    RW_ParametersBuffer[RUN_REFORMER_CMP_TBL_STORE_OFFSET_ADDR + 2]  = i_ReformerTemCmpTbl->IgFstTimeWatiTimeMax1;
    RW_ParametersBuffer[RUN_REFORMER_CMP_TBL_STORE_OFFSET_ADDR + 3]  = i_ReformerTemCmpTbl->IgFstTimeWatiTimeMax2;
    RW_ParametersBuffer[RUN_REFORMER_CMP_TBL_STORE_OFFSET_ADDR + 4]  = i_ReformerTemCmpTbl->IgScdTimeFrtToBhdTmpPnt;
    RW_ParametersBuffer[RUN_REFORMER_CMP_TBL_STORE_OFFSET_ADDR + 5]  = i_ReformerTemCmpTbl->IgScdTimeOverTmpPnt;
    RW_ParametersBuffer[RUN_REFORMER_CMP_TBL_STORE_OFFSET_ADDR + 6]  = i_ReformerTemCmpTbl->IgScdTimeWatiTimeMax1;
    RW_ParametersBuffer[RUN_REFORMER_CMP_TBL_STORE_OFFSET_ADDR + 7]  = i_ReformerTemCmpTbl->IgScdTimeWatiTimeMax2;

    RW_ParametersBuffer[RUN_REFORMER_CMP_TBL_STORE_OFFSET_ADDR + 8]  = i_ReformerTemCmpTbl->AlarmUpperLimit;
    RW_ParametersBuffer[RUN_REFORMER_CMP_TBL_STORE_OFFSET_ADDR + 9]  = i_ReformerTemCmpTbl->AlarmLowerLimit;

    RW_ParametersBuffer[RUN_REFORMER_CMP_TBL_STORE_OFFSET_ADDR + 10] = i_ReformerTemCmpTbl->ShutDownLowerLimit;
    RW_ParametersBuffer[RUN_REFORMER_CMP_TBL_STORE_OFFSET_ADDR + 11] = i_ReformerTemCmpTbl->ShutDownUpperLimit;
}

/*
*********************************************************************************************************
*                                      LoadLqdPressCmpTbl()
*
* Description:  load the paramters, that related to the liquid press, that has stored in the flash.
*
* Arguments  :  the avriable that store the parameters.
*
* Returns    :  none.
*
* Note(s)    :  none.
*********************************************************************************************************
*/
static void LoadLqdPressCmpTbl(LIQUID_PRESSURE_CMP_LINES_Typedef *i_LqdPressCmpTbl)
{
    i_LqdPressCmpTbl->AlarmLowerLimit               = RW_ParametersBuffer[RUN_LIQUIDPRESS_CMP_TBL_STORE_OFFSET_ADDR];
    i_LqdPressCmpTbl->AlarmUpperLimit               = RW_ParametersBuffer[RUN_LIQUIDPRESS_CMP_TBL_STORE_OFFSET_ADDR + 1];
    i_LqdPressCmpTbl->ShutDownLowerLimit            = RW_ParametersBuffer[RUN_LIQUIDPRESS_CMP_TBL_STORE_OFFSET_ADDR + 2];
    i_LqdPressCmpTbl->ShutDownUpperLimit            = RW_ParametersBuffer[RUN_LIQUIDPRESS_CMP_TBL_STORE_OFFSET_ADDR + 3];
}

/*
*********************************************************************************************************
*                                      LoadLqdHeightCmpTbl()
*
* Description:  load the paramters, that related to the liquid press, that has stored in the flash.
*
* Arguments  :  the avriable that store the parameters.
*
* Returns    :  none.
*
* Note(s)    :  none.
*********************************************************************************************************
*/
static void LoadLqdHeightCmpTbl(LIQUID_HEIGHT_CMP_LINES_Typedef *i_LqdHeightCmpTbl)
{
    i_LqdHeightCmpTbl->AlarmlowerLiquidLevellimit = RW_ParametersBuffer[RUN_LIQUID_HEIGHT_CMP_TBL_STORE_OFFSET_ADDR];
}
/*
*********************************************************************************************************
*                                      LoadDefaultLqdPressCmpTbl()
*
* Description:  load the default paramters that related to the liquid press.
*
* Arguments  :  the avriable that store the parameters.
*
* Returns    :  none.
*
* Note(s)    :  none.
*********************************************************************************************************
*/
static void LoadDefaultLqdPressCmpTbl(LIQUID_PRESSURE_CMP_LINES_Typedef *i_LqdPressCmpTbl)
{
    i_LqdPressCmpTbl->AlarmLowerLimit               = 1;
    i_LqdPressCmpTbl->AlarmUpperLimit               = 18;
    i_LqdPressCmpTbl->ShutDownLowerLimit            = 0;
    i_LqdPressCmpTbl->ShutDownUpperLimit            = 20;
}


/*
*********************************************************************************************************
*                                      LoadDefaultLqdHeightCmpTbl()
*
* Description:  load the default paramters that related to the liquid press.
*
* Arguments  :  the avriable that store the parameters.
*
* Returns    :  none.
*
* Note(s)    :  none.
*********************************************************************************************************
*/
static void LoadDefaultLqdHeightCmpTbl(LIQUID_HEIGHT_CMP_LINES_Typedef *i_LqdHeightCmpTbl)
{
    i_LqdHeightCmpTbl->AlarmlowerLiquidLevellimit = 70;
}


/*
*********************************************************************************************************
*                                      SaveLqdPressCmpTbl()
*
* Description:  save the paramters that related to the liquid press.
*
* Arguments  :  the avriable that store the parameters.
*
* Returns    :  none.
*
* Note(s)    :  none.
*********************************************************************************************************
*/
static void SaveLqdPressCmpTbl(LIQUID_PRESSURE_CMP_LINES_Typedef *i_LqdPressCmpTbl)
{
    RW_ParametersBuffer[RUN_LIQUIDPRESS_CMP_TBL_STORE_OFFSET_ADDR]      =   i_LqdPressCmpTbl->AlarmLowerLimit;
    RW_ParametersBuffer[RUN_LIQUIDPRESS_CMP_TBL_STORE_OFFSET_ADDR + 1]  =   i_LqdPressCmpTbl->AlarmUpperLimit;
    RW_ParametersBuffer[RUN_LIQUIDPRESS_CMP_TBL_STORE_OFFSET_ADDR + 2]  =   i_LqdPressCmpTbl->ShutDownLowerLimit;
    RW_ParametersBuffer[RUN_LIQUIDPRESS_CMP_TBL_STORE_OFFSET_ADDR + 3]  =   i_LqdPressCmpTbl->ShutDownUpperLimit;
}

/*
*********************************************************************************************************
*                                      SaveLqdHeightCmpTbl()
*
* Description:  save the paramters that related to the liquid press.
*
* Arguments  :  the avriable that store the parameters.
*
* Returns    :  none.
*
* Note(s)    :  none.
*********************************************************************************************************
*/
static void SaveLqdHeightCmpTbl(LIQUID_HEIGHT_CMP_LINES_Typedef *i_LqdHeightCmpTbl)
{
    RW_ParametersBuffer[RUN_LIQUID_HEIGHT_CMP_TBL_STORE_OFFSET_ADDR] = i_LqdHeightCmpTbl->AlarmlowerLiquidLevellimit;
}

/*
*********************************************************************************************************
*                                      LoadStartHydrgFanAndPumpSpd()
*
* Description:  load the paramters, that related to the speed adjust device, that has stored in the flash.
*
* Arguments  :  the avriable that store the parameters.
*
* Returns    :  none.
*
* Note(s)    :  none.
*********************************************************************************************************
*/
static void LoadStartHydrgFanAndPumpSpd(HYDROGEN_FAN_AND_PUMP_SPEED_LINES_Typedef *i_StartHydrgFanPumpSpd)
{
    i_StartHydrgFanPumpSpd->FanSpdIgniterFirstTime
        = RW_ParametersBuffer[RUN_START_HYDROGEN_FAN_AND_IGNITER_SPEED_STORE_OFFSET_ADDR];
    i_StartHydrgFanPumpSpd->FanSpdIgniterSecondTime
        = RW_ParametersBuffer[RUN_START_HYDROGEN_FAN_AND_IGNITER_SPEED_STORE_OFFSET_ADDR + 1];
    i_StartHydrgFanPumpSpd->PumpSpdIgniterFirstTime
        = RW_ParametersBuffer[RUN_START_HYDROGEN_FAN_AND_IGNITER_SPEED_STORE_OFFSET_ADDR + 2];
    i_StartHydrgFanPumpSpd->PumpSpdIgniterSecondTime
        = RW_ParametersBuffer[RUN_START_HYDROGEN_FAN_AND_IGNITER_SPEED_STORE_OFFSET_ADDR + 3];
}

/*
*********************************************************************************************************
*                                      LoadDefaultStartHydrgFanAndPumpSpd()
*
* Description:  load the default paramters that related to the speed adjust device.
*
* Arguments  :  the avriable that store the parameters.
*
* Returns    :  none.
*
* Note(s)    :  none.
*********************************************************************************************************
*/
static void LoadDefaultStartHydrgFanAndPumpSpd(HYDROGEN_FAN_AND_PUMP_SPEED_LINES_Typedef *i_StartHydrgFanPumpSpd)
{
    i_StartHydrgFanPumpSpd->FanSpdIgniterFirstTime   = 200;
    i_StartHydrgFanPumpSpd->FanSpdIgniterSecondTime   = 200;

    i_StartHydrgFanPumpSpd->PumpSpdIgniterFirstTime   = 19;
    i_StartHydrgFanPumpSpd->PumpSpdIgniterSecondTime  = 40;
}

/*
*********************************************************************************************************
*                                      SaveStartHydrgFanAndPumpSpd()
*
* Description:  save the paramters that related to the speed adjust device.
*
* Arguments  :  the buff that store the parameters.
*
* Returns    :  none.
*
* Note(s)    :  none.
*********************************************************************************************************
*/
static void SaveStartHydrgFanAndPumpSpd(HYDROGEN_FAN_AND_PUMP_SPEED_LINES_Typedef *i_StartHydrgFanPumpSpd)
{
    RW_ParametersBuffer[RUN_START_HYDROGEN_FAN_AND_IGNITER_SPEED_STORE_OFFSET_ADDR]
        = i_StartHydrgFanPumpSpd->FanSpdIgniterFirstTime;

    RW_ParametersBuffer[RUN_START_HYDROGEN_FAN_AND_IGNITER_SPEED_STORE_OFFSET_ADDR + 1]
        = i_StartHydrgFanPumpSpd->FanSpdIgniterSecondTime;

    RW_ParametersBuffer[RUN_START_HYDROGEN_FAN_AND_IGNITER_SPEED_STORE_OFFSET_ADDR + 2]
        = i_StartHydrgFanPumpSpd->PumpSpdIgniterFirstTime;

    RW_ParametersBuffer[RUN_START_HYDROGEN_FAN_AND_IGNITER_SPEED_STORE_OFFSET_ADDR + 3]
        = i_StartHydrgFanPumpSpd->PumpSpdIgniterSecondTime;
}

/*
*********************************************************************************************************
*                                      LoadRunPurifyAmpIntegralValue()
*
* Description:  load the paramters that related to the hydrogen in and out cmp level.
*
* Arguments  :  the avriable that store the parameters.
*
* Returns    :  none.
*
* Note(s)    :  none.
*********************************************************************************************************
*/
static void LoadRunPurifyAmpIntegralValue(u16 *i_RunPurifyAmpIntegralValue)
{
    *i_RunPurifyAmpIntegralValue = RW_ParametersBuffer[RUN_PURIFY_AMP_INTEGRAL_VALUE];
}

/*
*********************************************************************************************************
*                                      LoadRunPurifyAmpIntegralValue()
*
* Description:  load the default paramters that related to the hydrogen in and out cmp level.
*
* Arguments  :  the avriable that store the parameters.
*
* Returns    :  none.
*
* Note(s)    :  none.
*********************************************************************************************************
*/
static void LoadDefaultRunPurifyAmpIntegralValue(u16 *i_RunPurifyAmpIntegralValue)
{
    *i_RunPurifyAmpIntegralValue = 2300;
}

/*
*********************************************************************************************************
*                                      LoadRunPurifyAmpIntegralValue()
*
* Description:  save the paramters that related to the hydrogen in and out cmp level.
*
* Arguments  :  the buff that store the parameters.
*
* Returns    :  none.
*
* Note(s)    :  none.
*********************************************************************************************************
*/
static void SaveRunPurifyAmpIntegralValue(u16 *i_RunPurifyAmpIntegralValue)
{
    RW_ParametersBuffer[RUN_PURIFY_AMP_INTEGRAL_VALUE] = *i_RunPurifyAmpIntegralValue;
}

/*
*********************************************************************************************************
*                                      STMFLASH_ReadHalfWord()
*
* Description:  读取指定地址的半字(16位数据)
*
* Arguments  :  读地址(此地址必须为2的倍数!!)
*
* Returns    :  对应数据.
*
* Note(s)    :  none.
*********************************************************************************************************
*/
u16 STMFLASH_ReadHalfWord(u32 faddr)
{
    return *(vu16 *)faddr;
}

#if STM32_FLASH_WREN    //如果使能了写功能    
/*
*********************************************************************************************************
*                                      STMFLASH_Write_NoCheck()
*
* Description:  不检查的写入半字(16位数据)到指定地址
*
* Arguments  :  写起始地址(此地址必须为2的倍数!!)
*               待写数据
*
* Returns    :  none.
*
* Note(s)    :  none.
*********************************************************************************************************
*/
static void STMFLASH_Write_NoCheck(u32 WriteAddr, u16 *pBuffer, u16 NumToWrite)
{
    u16 i;

    for(i = 0; i < NumToWrite; i++)
    {
        FLASH_ProgramHalfWord(WriteAddr, pBuffer[i]);
        WriteAddr += 2;//地址增加2.
    }
}

/*
*********************************************************************************************************
*                                      STMFLASH_Write()
*
* Description:  从指定地址开始写入指定长度的数据
*
* Arguments  :  WriteAddr   -  写起始地址(此地址必须为2的倍数!!)
*               pBuffer     -  待写数据指针
*               NumToWrite  -  待写数据指针的半字（16位）数
*
* Returns    :  none.
*
* Note(s)    :  none.
*********************************************************************************************************
*/
#if     STM32_FLASH_SIZE < 256
    #define STM_SECTOR_SIZE 1024    //字节
#else
    #define STM_SECTOR_SIZE 2048
#endif
static      u16     STMFLASH_BUF[PARAMETERS_STORE_AREA_SIZE];
static void STMFLASH_Write(u32 WriteAddr, u16 *pBuffer, u16 NumToWrite)
{
    u32 secpos;    //扇区地址
    u16 secoff;    //扇区内偏移地址(16位字计算)
    u16 secremain; //扇区内剩余地址(16位字计算)
    u16 i;
    u32 offaddr;   //去掉0X08000000后的地址

    if((WriteAddr >= STM32_FLASH_BASE) && (WriteAddr < (STM32_FLASH_BASE + 1024 * STM32_FLASH_SIZE)))
    {
        FLASH_Unlock();                                 //解锁
        offaddr = WriteAddr - STM32_FLASH_BASE;         //实际偏移地址.
        secpos = offaddr / STM_SECTOR_SIZE;             //扇区地址  0~127 共128个扇区，在STM32F105RB中，每个扇区1K
        secoff = (offaddr % STM_SECTOR_SIZE) / 2;       //在扇区内的偏移(2个字节为基本单位.)
        secremain = STM_SECTOR_SIZE / 2 - secoff;       //扇区剩余空间大小

        if(NumToWrite <= secremain)
        {
            secremain = NumToWrite;//不大于该扇区范围   //否则后续会启动下一个扇区的书写
        }

        while(1)
        {
            STMFLASH_Read(secpos * STM_SECTOR_SIZE + STM32_FLASH_BASE, STMFLASH_BUF, STM_SECTOR_SIZE / 2);//读出整个扇区的内容

            for(i = 0; i < secremain; i++)//校验数据
            {
                if(STMFLASH_BUF[secoff + i] != 0XFFFF)
                {
                    break;//需要擦除
                }
            }

            if(i < secremain)//需要擦除
            {
                FLASH_ErasePage(secpos * STM_SECTOR_SIZE + STM32_FLASH_BASE);//擦除这个扇区

                for(i = 0; i < secremain; i++)//复制
                {
                    STMFLASH_BUF[i + secoff] = pBuffer[i];
                }

                STMFLASH_Write_NoCheck(secpos * STM_SECTOR_SIZE + STM32_FLASH_BASE, STMFLASH_BUF, STM_SECTOR_SIZE / 2);//写入整个扇区
            }
            else
            {
                STMFLASH_Write_NoCheck(WriteAddr, pBuffer, secremain);//写已经擦除了的,直接写入扇区剩余区间.
            }

            if(NumToWrite == secremain)
            {
                break;//写入结束了
            }
            else//写入未结束
            {
                secpos ++;              //扇区地址增1
                secoff = 0;             //偏移位置为0
                pBuffer += secremain;   //指针偏移
                WriteAddr += secremain; //写地址偏移
                NumToWrite -= secremain;    //字节(16位)数递减

                if(NumToWrite > (STM_SECTOR_SIZE / 2))
                {
                    secremain = STM_SECTOR_SIZE / 2;//下一个扇区还是写不完
                }
                else
                {
                    secremain = NumToWrite;//下一个扇区可以写完了
                }
            }
        }

        FLASH_Lock();//上锁
    }
}
#endif

/*
*********************************************************************************************************
*                                      STMFLASH_Write()
*
* Description:  从指定地址开始读出指定长度的数据
*
* Arguments  :  ReadAddr   -  读起始地址(此地址必须为2的倍数!!)
*               pBuffer     -  承接数据指针
*               NumToWrite  -  待读数据指针的半字（16位）数
*
* Returns    :  none.
*
* Note(s)    :  none.
*********************************************************************************************************
*/
void STMFLASH_Read(u32 ReadAddr, u16 *pBuffer, u16 NumToRead)
{
    u16 i;

    for(i = 0; i < NumToRead; i++)
    {
        pBuffer[i] = STMFLASH_ReadHalfWord(ReadAddr);//读取2个字节.
        ReadAddr += 2;//偏移2个字节.
    }
}

#endif
/******************* (C) COPYRIGHT 2015 Guangdong Hydrogen *****END OF FILE****/

