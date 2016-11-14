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
#define STM32_FLASH_SIZE            128             //����STM32��FLASH������С(��λΪK)
#define STM32_FLASH_WREN            1               //ʹ��FLASHд��(0:ʧ��;1:ʹ��)

#define STM32_FLASH_BASE            0x08000000      //STM32 FLASH����ʼ��ַ

#define PARAMETER_SAVE_ADDR         0x0801FC00      //��ַ픶˞�0x08020000 ����FLASH �����ַ�������1K�ֹ�(����Ϊż��)

//��������һ���ֵĿռ�
#define PARAMETERS_STORE_AREA_SIZE                                      512 //�����������Ĵ�С���ֽڣ�
#define RUN_REAL_PARAMETERS_STORE_OFFSET_ADDR                           0   //��ʼ��ַ���洢���д���
#define REAL_TIME_PARAMETERS_STORE_OFFSET_ADDR                          1   //  4��ʱ�䣬��4������λ��
#define RUN_REFORMER_CMP_TBL_STORE_OFFSET_ADDR                          20  //12����
#define RUN_LIQUIDPRESS_CMP_TBL_STORE_OFFSET_ADDR                       35  // 4����
#define RUN_START_HYDROGEN_FAN_AND_IGNITER_SPEED_STORE_OFFSET_ADDR      40
#define RUN_PURIFY_AMP_INTEGRAL_VALUE                                   45  //����ᴿĤ������
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
static          u16         STMFLASH_ReadHalfWord(u32 faddr);         //��������
static          void        STMFLASH_Write_NoCheck(u32 WriteAddr, u16 *pBuffer, u16 NumToWrite);

static          void        STMFLASH_Write(u32 WriteAddr, u16 *pBuffer, u16 NumToWrite);    //��ָ����ַ��ʼд��ָ�����ȵ�����
static          void        STMFLASH_Read(u32 ReadAddr, u16 *pBuffer, u16 NumToRead);       //��ָ����ַ��ʼ����ָ�����ȵ�����

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

    //����ϵͳ���д���
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
    else//����Ĭ�ϲ���
    {
        APP_TRACE_INFO(("This is the first time for the system to boot,capturing the parameters...\n\r"));

        LoadDefaultReformerTempCmpTbl(&g_stReformerTempCmpTbl);
        LoadDefaultLqdPressCmpTbl(&g_stLqdPressCmpTbl);
        LoadDefaultLqdHeightCmpTbl(&g_stLqdHeightCmpTbl);
        LoadDefaultStartHydrgFanAndPumpSpd(&g_stStartHydrgFanIgniterSpd);
        LoadDefaultRunPurifyAmpIntegralValue(&g_u16RunPurifyAmpIntegralValue);

        //�����׼��ģ�����������Ĳ���
        BSP_LoadCalibratedAnaSensorParameters();

        SaveReformerTempCmpTbl(&g_stReformerTempCmpTbl);
        SaveLqdPressCmpTbl(&g_stLqdPressCmpTbl);
        SaveLqdHeightCmpTbl(&g_stLqdHeightCmpTbl);
        SaveStartHydrgFanAndPumpSpd(&g_stStartHydrgFanIgniterSpd);
        SaveRunPurifyAmpIntegralValue(&g_u16RunPurifyAmpIntegralValue);
        //�����׼��ģ�����������Ĳ���
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
    STMFLASH_Read(PARAMETER_SAVE_ADDR + RUN_REAL_PARAMETERS_STORE_OFFSET_ADDR, i_StoreAddr, 1);//��ȡ���д���
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
* Description:  ��ȡָ����ַ�İ���(16λ����)
*
* Arguments  :  ����ַ(�˵�ַ����Ϊ2�ı���!!)
*
* Returns    :  ��Ӧ����.
*
* Note(s)    :  none.
*********************************************************************************************************
*/
u16 STMFLASH_ReadHalfWord(u32 faddr)
{
    return *(vu16 *)faddr;
}

#if STM32_FLASH_WREN    //���ʹ����д����    
/*
*********************************************************************************************************
*                                      STMFLASH_Write_NoCheck()
*
* Description:  ������д�����(16λ����)��ָ����ַ
*
* Arguments  :  д��ʼ��ַ(�˵�ַ����Ϊ2�ı���!!)
*               ��д����
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
        WriteAddr += 2;//��ַ����2.
    }
}

/*
*********************************************************************************************************
*                                      STMFLASH_Write()
*
* Description:  ��ָ����ַ��ʼд��ָ�����ȵ�����
*
* Arguments  :  WriteAddr   -  д��ʼ��ַ(�˵�ַ����Ϊ2�ı���!!)
*               pBuffer     -  ��д����ָ��
*               NumToWrite  -  ��д����ָ��İ��֣�16λ����
*
* Returns    :  none.
*
* Note(s)    :  none.
*********************************************************************************************************
*/
#if     STM32_FLASH_SIZE < 256
    #define STM_SECTOR_SIZE 1024    //�ֽ�
#else
    #define STM_SECTOR_SIZE 2048
#endif
static      u16     STMFLASH_BUF[PARAMETERS_STORE_AREA_SIZE];
static void STMFLASH_Write(u32 WriteAddr, u16 *pBuffer, u16 NumToWrite)
{
    u32 secpos;    //������ַ
    u16 secoff;    //������ƫ�Ƶ�ַ(16λ�ּ���)
    u16 secremain; //������ʣ���ַ(16λ�ּ���)
    u16 i;
    u32 offaddr;   //ȥ��0X08000000��ĵ�ַ

    if((WriteAddr >= STM32_FLASH_BASE) && (WriteAddr < (STM32_FLASH_BASE + 1024 * STM32_FLASH_SIZE)))
    {
        FLASH_Unlock();                                 //����
        offaddr = WriteAddr - STM32_FLASH_BASE;         //ʵ��ƫ�Ƶ�ַ.
        secpos = offaddr / STM_SECTOR_SIZE;             //������ַ  0~127 ��128����������STM32F105RB�У�ÿ������1K
        secoff = (offaddr % STM_SECTOR_SIZE) / 2;       //�������ڵ�ƫ��(2���ֽ�Ϊ������λ.)
        secremain = STM_SECTOR_SIZE / 2 - secoff;       //����ʣ��ռ��С

        if(NumToWrite <= secremain)
        {
            secremain = NumToWrite;//�����ڸ�������Χ   //���������������һ����������д
        }

        while(1)
        {
            STMFLASH_Read(secpos * STM_SECTOR_SIZE + STM32_FLASH_BASE, STMFLASH_BUF, STM_SECTOR_SIZE / 2);//������������������

            for(i = 0; i < secremain; i++)//У������
            {
                if(STMFLASH_BUF[secoff + i] != 0XFFFF)
                {
                    break;//��Ҫ����
                }
            }

            if(i < secremain)//��Ҫ����
            {
                FLASH_ErasePage(secpos * STM_SECTOR_SIZE + STM32_FLASH_BASE);//�����������

                for(i = 0; i < secremain; i++)//����
                {
                    STMFLASH_BUF[i + secoff] = pBuffer[i];
                }

                STMFLASH_Write_NoCheck(secpos * STM_SECTOR_SIZE + STM32_FLASH_BASE, STMFLASH_BUF, STM_SECTOR_SIZE / 2);//д����������
            }
            else
            {
                STMFLASH_Write_NoCheck(WriteAddr, pBuffer, secremain);//д�Ѿ������˵�,ֱ��д������ʣ������.
            }

            if(NumToWrite == secremain)
            {
                break;//д�������
            }
            else//д��δ����
            {
                secpos ++;              //������ַ��1
                secoff = 0;             //ƫ��λ��Ϊ0
                pBuffer += secremain;   //ָ��ƫ��
                WriteAddr += secremain; //д��ַƫ��
                NumToWrite -= secremain;    //�ֽ�(16λ)���ݼ�

                if(NumToWrite > (STM_SECTOR_SIZE / 2))
                {
                    secremain = STM_SECTOR_SIZE / 2;//��һ����������д����
                }
                else
                {
                    secremain = NumToWrite;//��һ����������д����
                }
            }
        }

        FLASH_Lock();//����
    }
}
#endif

/*
*********************************************************************************************************
*                                      STMFLASH_Write()
*
* Description:  ��ָ����ַ��ʼ����ָ�����ȵ�����
*
* Arguments  :  ReadAddr   -  ����ʼ��ַ(�˵�ַ����Ϊ2�ı���!!)
*               pBuffer     -  �н�����ָ��
*               NumToWrite  -  ��������ָ��İ��֣�16λ����
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
        pBuffer[i] = STMFLASH_ReadHalfWord(ReadAddr);//��ȡ2���ֽ�.
        ReadAddr += 2;//ƫ��2���ֽ�.
    }
}

#endif
/******************* (C) COPYRIGHT 2015 Guangdong Hydrogen *****END OF FILE****/

