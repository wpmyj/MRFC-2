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
* Filename      : app_system_real_time_parameters.c
* Version       : V1.00
* Programmer(s) : FanJun
***************************************************************************************************
*/
/*
***************************************************************************************************
*                                           INCLUDE FILES
***************************************************************************************************
*/
#include "includes.h"
#include "app_system_run_cfg_parameters.h"
#include "app_system_real_time_parameters.h"
#include "bsp_ana_sensor.h"
/*
***************************************************************************************************
*                                           MACRO DEFINITIONS
***************************************************************************************************
*/
#define STM32_FLASH_SIZE            128             //所用STM32的FLASH容量大小(单位为K)
#define STM32_FLASH_WREN            1               //使能FLASH写入(0:失能;1:使能)

#define STM32_FLASH_BASE            0x08000000      //STM32 FLASH的起始地址

#define SYSTEM_PARAMETER_STORE_SEGMENT_ADDR         0x0801FC00      //地址頂端為0x08020000 设置FLASH 保存地址為最後的1K字節(必须为偶数)

#define PARAMETERS_STORE_AREA_SIZE                      512 //参数保存区的大小（字节）

//下面宏定义表示开机运行的驱动层参数（传感器）和应用层工作参数是采用设定值还是默认值的标志值的偏移地址
//无符号16位数，擦除后为全0xFFFF
//bit[0]表示应用层是采用默认值还是配置值--0:配置值  1:默认值
//bit[1]表示驱动层参数采用默认值还是配置--0:配置值  1:默认值
#define APP_PARAMETERS_SELECT_FLAG_CHECK                                0x0001
#define APP_PARAMETERS_SELECT_CFG_VALUE                                 0x0000
#define APP_PARAMETERS_SELECT_DEFAULT_VALUE                             0x0001

#define DRIVER_LAYER_PARAMETERS_SELECT_FLAG_CHECK                       0x0002
#define DRIVER_LAYER_PARAMETERS_SELECT_CFG_VALUE                        0x0000  //使用配置过的参数
#define DRIVER_LAYER_PARAMETERS_SELECT_DEFAULT_VALUE                    0x0002  //使用默认参数

/*以下宏定义表示参数存储基地址*/
#define PARAMETERS_SELECT_FLAG_OFFSET_ADDR                              0   //参数选择标志
#define SYSTEM_RUN_TIME_STORE_OFFSET_ADDR                               2   //系统运行次数
#define SYSTEM_TOTAL_WORK_TIME_STORE_OFFSET_ADDR                        4   //系统时间时间4个，各4个字节
#define RUN_REFORMER_CMP_TBL_STORE_OFFSET_ADDR                          20  //12个数
#define RUN_LIQUIDPRESS_CMP_TBL_STORE_OFFSET_ADDR                       40  // 液压4个数
#define RUN_LIQUID_HEIGHT_CMP_TBL_STORE_OFFSET_ADDR                     50  //液位参数4个
#define RUN_HYDROGEN_PUMP_SPEED_PARA_STORE_OFFSET_ADDR                  60  //泵参数2个
#define RUN_HYDROGEN_FAN_SPEED_PARA_STORE_OFFSET_ADDR                   70  //制氢风机参数4个
#define RUN_PURIFY_AMP_INTEGRAL_VALUE                                   80  //电堆提纯膜间电荷量1个
#define DRIVER_LAYER_CALIBRATED_LINEAR_ANA_SENSOR_PARAMETERS_OFFSET_ADDR  82  //驱动层线性传感器校准参数,28个数


/*
***************************************************************************************************
*                                     GLOBAL  VARIABLES
***************************************************************************************************
*/
REFORMER_TEMP_CMP_LINES_Typedef             g_stReformerTempCmpTbl;
LIQUID_PRESSURE_CMP_LINES_Typedef           g_stLqdPressCmpTbl;
LIQUID_HEIGHT_CMP_LINES_Typedef             g_stLqdHeightCmpTbl;
HYDROGEN_PUMP_SPEED_PARA_Typedef            g_stStartHydrgPumpSpdPara;
HYDROGEN_FAN_SPEED_PARA_Typedef             g_stStartHydrgFanSpdPara;

uint16_t                                    g_u16RunPurifyAmpIntegralValue;
/*
***************************************************************************************************
*                                     LOCAL VARIABLES
***************************************************************************************************
*/
//static     uint16_t      RW_ParametersBuffer[PARAMETERS_STORE_AREA_SIZE / 2];

/*
***************************************************************************************************
*                                         FUNCTION PROTOTYPES
***************************************************************************************************
*/
static          uint16_t    STMFLASH_ReadHalfWord(u32 faddr);         //读出半字
static          void        STMFLASH_Write_NoCheck(u32 WriteAddr, u16 *pBuffer, u16 NumToWrite);

static          void        STMFLASH_Write(u32 WriteAddr, u16 *pBuffer, u16 NumToWrite);    //从指定地址写指定长度的数据
static          void        STMFLASH_Read(u32 ReadAddr, u16 *pBuffer, u16 NumToRead);       //从指定地址读指定长度的数据

static          void        GetParametersSelectFlag(uint16_t *o_pParametersSelectFlagAddr);
static          void        SetParametersSelectFlag(uint16_t *i_pParametersSelectFlagAddr);

static          void        BSP_GetStoredAnaSensorsParaFromFlash(ANALOG_SIGNAL_SERSOR_PARAMETERS_Typedef *, uint8_t);
static          void        BSP_StoreAnaSensorParaToFlash(ANALOG_SIGNAL_SERSOR_PARAMETERS_Typedef *, uint8_t);

static          void        GetSystemWorkTimesFromFlash(u16 *o_StoreAddr);
static          void        GetTotalWorkTimeFromFlash(SYSTEM_TIME_Typedef *o_StoreAddr);
static          void        StoreTotalWorkTime(SYSTEM_TIME_Typedef *i_stTotalTime);

static          void        GetReformerTempCmpTblFromFlash(REFORMER_TEMP_CMP_LINES_Typedef *);
static          void        GetDefaultReformerTempCmpTbl(REFORMER_TEMP_CMP_LINES_Typedef *);
static          void        StoreReformerTempCmpTbl(REFORMER_TEMP_CMP_LINES_Typedef *);

static          void        GetLqdPressCmpTblFromFlash(LIQUID_PRESSURE_CMP_LINES_Typedef *);
static          void        GetDefaultLqdPressCmpTbl(LIQUID_PRESSURE_CMP_LINES_Typedef *);
static          void        StoreLqdPressCmpTbl(LIQUID_PRESSURE_CMP_LINES_Typedef *);

static          void        GetLqdHeightCmpTblFromFlash(LIQUID_HEIGHT_CMP_LINES_Typedef *i_LqdHeightCmpTbl);
static          void        GetDefaultLqdHeightCmpTbl(LIQUID_HEIGHT_CMP_LINES_Typedef *i_LqdHeightCmpTbl);
static          void        StoreLqdHeightCmpTbl(LIQUID_HEIGHT_CMP_LINES_Typedef *i_LqdHeightCmpTbl);

static          void        GetDefaultStartHydrgPumpSpdPara(HYDROGEN_PUMP_SPEED_PARA_Typedef *i_StartPumpSpdPara);
static          void        GetDefaultStartHydrgFanSpdPara(HYDROGEN_FAN_SPEED_PARA_Typedef *i_StartHydrgFanSpdPara);

static          void        GetRunPurifyAmpIntegralValueFromFlash(u16 *);
static          void        GetDefaultRunPurifyAmpIntegralValue(u16 *);
static          void        StoreRunPurifyAmpIntegralValue(u16 *);



#if 1
/*
***************************************************************************************************
*                               LoadDriverLayerParameters()
*
* Description:  load the driver Layer sensors and others devices prameters.
*
* Arguments  :  none.
*
* Returns    :  none.
*
* Note(s)    :  none.
***************************************************************************************************
*/
void LoadDriverLayerParameters()
{
    uint16_t u16ParametersSelectFlag;
    ErrorStatus eCalibratedFlag;
    ANALOG_SIGNAL_SERSOR_PARAMETERS_Typedef stAnaSensorParameters[BSP_ANA_SENSORS_NMB];
    uint8_t u8DataLength = BSP_ANA_SENSORS_NMB;

    GetParametersSelectFlag(&u16ParametersSelectFlag);

    /* 使用已经配置好的参数 */
    if(DRIVER_LAYER_PARAMETERS_SELECT_CFG_VALUE == (DRIVER_LAYER_PARAMETERS_SELECT_FLAG_CHECK & u16ParametersSelectFlag)) {
        BSP_GetStoredAnaSensorsParaFromFlash(stAnaSensorParameters, u8DataLength);//从flash中读取参数
//      APP_TRACE_INFO(("%f , %f...\r\n", stAnaSensorParameters[0].AnaToDigRatio, stAnaSensorParameters[0].BaseDigValue));
        BSP_LoadAnaSensorParaToPrgm(stAnaSensorParameters, u8DataLength);//将参数传给程序
    } else { //首次校准或者校准失败时重新校准
        eCalibratedFlag = BSP_GetCalibratedAnaSensorPara(stAnaSensorParameters, u8DataLength);//从校准程序获得校零后的参数

//      APP_TRACE_INFO(("%f , %f...\r\n", stAnaSensorParameters[0].AnaToDigRatio, stAnaSensorParameters[0].BaseDigValue));
        if(eCalibratedFlag == SUCCESS) {
            /*校准成功则下次启动采用配置值即可*/
            u16ParametersSelectFlag &= ~DRIVER_LAYER_PARAMETERS_SELECT_DEFAULT_VALUE;//清除选择默认参数标志位
            SetParametersSelectFlag(&u16ParametersSelectFlag);
            BSP_StoreAnaSensorParaToFlash(stAnaSensorParameters, u8DataLength);//将获得的参数保存到flash
        } else {
            //校准失败，程序使用默认值
            //对启动驱动参数选择标志不做修改，下次启动仍然需要校准
        }
    }
}

/*
***************************************************************************************************
*                                      LoadApplicationLayerParameters()
*
* Description:  load the application layer paramters .
*
* Arguments  :  none.
*
* Returns    :  none.
*
* Note(s)    :  none.
***************************************************************************************************
*/
void LoadApplicationLayerParameters()
{
    uint16_t u16WorkTimes;
    uint16_t u16ParametersSelectFlag;
    SYSTEM_TIME_Typedef     stTotalWorkTime;

    GetParametersSelectFlag(&u16ParametersSelectFlag);

    /*使用配置好的应用参数*/
    if(APP_PARAMETERS_SELECT_CFG_VALUE == (APP_PARAMETERS_SELECT_FLAG_CHECK & u16ParametersSelectFlag)) {
        APP_TRACE_INFO(("The machine will work with the configured parameters!...\r\n"));
        GetSystemWorkTimesFromFlash(&u16WorkTimes);

        if(u16WorkTimes != 0xFFFF) {
            LoadSystemWorkTimesToPrgm(u16WorkTimes);//使用记录的参数
        } else {
            LoadSystemWorkTimesToPrgm(0);//历史运行次数自动归零，但暂不保存，机器启动后才增加一次
        }

        APP_TRACE_INFO(("The %d time for the system to boot,loading the parameters..\r\n", u16WorkTimes + 1));

        GetTotalWorkTimeFromFlash(&stTotalWorkTime);
        LoadTotalWorkTimeToPrgm(stTotalWorkTime);   //传到实时运行参数所在文件对应的程序中去
        
        GetReformerTempCmpTblFromFlash(&g_stReformerTempCmpTbl);
        GetLqdPressCmpTblFromFlash(&g_stLqdPressCmpTbl);
        GetLqdHeightCmpTblFromFlash(&g_stLqdHeightCmpTbl);
        GetStartHydrgPumpSpdParaFromFlash(&g_stStartHydrgPumpSpdPara);
        GetStartHydrgFanSpdParaFromFlash(&g_stStartHydrgFanSpdPara);
        GetRunPurifyAmpIntegralValueFromFlash(&g_u16RunPurifyAmpIntegralValue);
        
    } else { //载入默认参数
        APP_TRACE_INFO(("First time run,the machine will work with the default parameters!...\r\n"));

        u16ParametersSelectFlag &= ~APP_PARAMETERS_SELECT_DEFAULT_VALUE;//下次启动采用配置好的参数
        SetParametersSelectFlag(&u16ParametersSelectFlag);//保存参数

        u16WorkTimes = 0;//系统运行次数为0
        LoadSystemWorkTimesToPrgm(u16WorkTimes);
        StoreSystemWorkTimes(u16WorkTimes);

        GetTotalWorkTimeFromFlash(&stTotalWorkTime);

        if((stTotalWorkTime.hour == 0xFFFF) && (stTotalWorkTime.minute == 0xFF) && (stTotalWorkTime.second == 0xFF)) {
            stTotalWorkTime.hour = 0;
            stTotalWorkTime.minute = 0;
            stTotalWorkTime.second = 0;
            StoreTotalWorkTime(&stTotalWorkTime);//累计运行时间归零并保存
        }

        LoadTotalWorkTimeToPrgm(stTotalWorkTime);

        APP_TRACE_INFO(("This is the first time for the system to boot,capturing the parameters...\r\n"));

        /*获取默认参数并且存到Flash中*/
        GetDefaultReformerTempCmpTbl(&g_stReformerTempCmpTbl);
        StoreReformerTempCmpTbl(&g_stReformerTempCmpTbl);
        
        GetDefaultLqdPressCmpTbl(&g_stLqdPressCmpTbl);
        StoreLqdPressCmpTbl(&g_stLqdPressCmpTbl);

        GetDefaultLqdHeightCmpTbl(&g_stLqdHeightCmpTbl);
        StoreLqdHeightCmpTbl(&g_stLqdHeightCmpTbl);

        GetDefaultStartHydrgPumpSpdPara(&g_stStartHydrgPumpSpdPara);
        StoreStartHydrgPumpSpdPara(&g_stStartHydrgPumpSpdPara);

        GetDefaultStartHydrgFanSpdPara(&g_stStartHydrgFanSpdPara);
        StoreStartHydrgFanSpdPara(&g_stStartHydrgFanSpdPara);

        GetDefaultRunPurifyAmpIntegralValue(&g_u16RunPurifyAmpIntegralValue);
        StoreRunPurifyAmpIntegralValue(&g_u16RunPurifyAmpIntegralValue);
    }
}

/*
***************************************************************************************************
*                           GetSystemWorkTimesFromFlash()
*
* Description:  Get the system work times that has stored in the flash
*
* Arguments  :  the avriable that store the work times.
*
* Returns    :  none.
*
* Note(s)    :  none.
***************************************************************************************************
*/
static void GetParametersSelectFlag(uint16_t *o_pParametersSelectFlagAddr)
{
    STMFLASH_Read(SYSTEM_PARAMETER_STORE_SEGMENT_ADDR + PARAMETERS_SELECT_FLAG_OFFSET_ADDR, o_pParametersSelectFlagAddr, 1);//获取运行参数选择标志
}

static void SetParametersSelectFlag(uint16_t *i_pParametersSelectFlagAddr)
{
    STMFLASH_Write(SYSTEM_PARAMETER_STORE_SEGMENT_ADDR + PARAMETERS_SELECT_FLAG_OFFSET_ADDR, i_pParametersSelectFlagAddr, 1);
}
/*
***************************************************************************************************
*                           StoreSystemWorkTimes()
*
* Description:  Get the system work times that has stored in the flash
*
* Arguments  :  the avriable that store the work times.
*
* Returns    :  none.
*
* Note(s)    :  none.
***************************************************************************************************
*/
void StoreSystemWorkTimes(uint16_t i_u16WorkTimes)
{
    STMFLASH_Write(SYSTEM_PARAMETER_STORE_SEGMENT_ADDR + SYSTEM_RUN_TIME_STORE_OFFSET_ADDR, &i_u16WorkTimes, 2);
}

static void GetSystemWorkTimesFromFlash(u16 *o_StoreAddr)
{
    STMFLASH_Read(SYSTEM_PARAMETER_STORE_SEGMENT_ADDR + SYSTEM_RUN_TIME_STORE_OFFSET_ADDR, o_StoreAddr, 2);//获取运行次数
}


/*
***************************************************************************************************
*                                   BSP_GetStoredAnaSensorsParaFromFlash()
*
* Description:  Get the Analog sensors prameters from Flash.
*
* Arguments  :  none.
*
* Returns    :  none.
*
* Note(s)    :  none.
***************************************************************************************************
*/
void BSP_GetStoredAnaSensorsParaFromFlash(ANALOG_SIGNAL_SERSOR_PARAMETERS_Typedef *o_stAnaSensorParameters, uint8_t i_u8Length)
{
    STMFLASH_Read(SYSTEM_PARAMETER_STORE_SEGMENT_ADDR + DRIVER_LAYER_CALIBRATED_LINEAR_ANA_SENSOR_PARAMETERS_OFFSET_ADDR,
                  (uint16_t *)o_stAnaSensorParameters, i_u8Length * 4); //结构体的长度是u16类型数据长度的4倍
}

void BSP_StoreAnaSensorParaToFlash(ANALOG_SIGNAL_SERSOR_PARAMETERS_Typedef *i_stAnaSensorParameters, uint8_t i_u8Length)
{
    STMFLASH_Write(SYSTEM_PARAMETER_STORE_SEGMENT_ADDR + DRIVER_LAYER_CALIBRATED_LINEAR_ANA_SENSOR_PARAMETERS_OFFSET_ADDR,
                   (uint16_t *)i_stAnaSensorParameters, i_u8Length * 4); //结构体的长度是u16类型数据长度的4倍
}

/*
***************************************************************************************************
*                                      GetTotalWorkTimeFromFlash()
*
* Description:  load the paramters, that related to the reformer, that has stored in the flash.
*
* Arguments  :  the avriable that store the parameters.
*
* Returns    :  none.
*
* Note(s)    :  none.
***************************************************************************************************
*/
static void GetTotalWorkTimeFromFlash(SYSTEM_TIME_Typedef *o_StoreAddr)
{
    STMFLASH_Read(SYSTEM_PARAMETER_STORE_SEGMENT_ADDR + SYSTEM_TOTAL_WORK_TIME_STORE_OFFSET_ADDR, (uint16_t *)o_StoreAddr, 16);
}

void StoreTotalWorkTime(SYSTEM_TIME_Typedef *i_stTotalTime)
{
    STMFLASH_Write(SYSTEM_PARAMETER_STORE_SEGMENT_ADDR + SYSTEM_TOTAL_WORK_TIME_STORE_OFFSET_ADDR, (uint16_t *)i_stTotalTime, 16);
}

/*
***************************************************************************************************
*                                      GetReformerTempCmpTblFromFlash()
*
* Description:  load the paramters, that related to the reformer, that has stored in the flash.
*
* Arguments  :  the avriable that store the parameters.
*
* Returns    :  none.
*
* Note(s)    :  none.
***************************************************************************************************
*/
static void GetReformerTempCmpTblFromFlash(REFORMER_TEMP_CMP_LINES_Typedef *i_ReformerTemCmpTbl)
{    
    STMFLASH_Read(SYSTEM_PARAMETER_STORE_SEGMENT_ADDR + RUN_REFORMER_CMP_TBL_STORE_OFFSET_ADDR, (uint16_t*)i_ReformerTemCmpTbl, 12);
}

static void GetDefaultReformerTempCmpTbl(REFORMER_TEMP_CMP_LINES_Typedef *i_ReformerTemCmpTbl)
{
    i_ReformerTemCmpTbl->IgFstTimeFrtToBhdTmpPnt    = NULL;
    i_ReformerTemCmpTbl->IgFstTimeOverTmpPnt        = 230;
    i_ReformerTemCmpTbl->IgFstTimeWatiTimeMax1      = NULL;
    i_ReformerTemCmpTbl->IgFstTimeWatiTimeMax2      = NULL;
    i_ReformerTemCmpTbl->IgScdTimeFrtToBhdTmpPnt    = NULL;
    i_ReformerTemCmpTbl->IgScdTimeOverTmpPnt        = NULL;
    i_ReformerTemCmpTbl->IgScdTimeWatiTimeMax1      = NULL;
    i_ReformerTemCmpTbl->IgScdTimeWatiTimeMax2      = NULL;

    i_ReformerTemCmpTbl->AlarmLowerLimit            = 0;
    i_ReformerTemCmpTbl->AlarmUpperLimit            = 550;
    i_ReformerTemCmpTbl->ShutDownLowerLimit         = 0;
    i_ReformerTemCmpTbl->ShutDownUpperLimit         = 100;
}

static void StoreReformerTempCmpTbl(REFORMER_TEMP_CMP_LINES_Typedef *i_ReformerTemCmpTbl)
{
    STMFLASH_Write(SYSTEM_PARAMETER_STORE_SEGMENT_ADDR + RUN_REFORMER_CMP_TBL_STORE_OFFSET_ADDR, (uint16_t*)i_ReformerTemCmpTbl, 12);
}

/*
***************************************************************************************************
*                                      GetLqdPressCmpTblFromFlash()
*
* Description:  load the paramters, that related to the liquid press, that has stored in the flash.
*
* Arguments  :  the avriable that store the parameters.
*
* Returns    :  none.
*
* Note(s)    :  none.
***************************************************************************************************
*/
static void GetLqdPressCmpTblFromFlash(LIQUID_PRESSURE_CMP_LINES_Typedef *i_LqdPressCmpTbl)
{
    STMFLASH_Read(SYSTEM_PARAMETER_STORE_SEGMENT_ADDR + RUN_LIQUIDPRESS_CMP_TBL_STORE_OFFSET_ADDR, (uint16_t*)i_LqdPressCmpTbl, 4);
}

static void GetDefaultLqdPressCmpTbl(LIQUID_PRESSURE_CMP_LINES_Typedef *i_LqdPressCmpTbl)
{
    i_LqdPressCmpTbl->AlarmLowerLimit               = 1;
    i_LqdPressCmpTbl->AlarmUpperLimit               = 18;
    i_LqdPressCmpTbl->ShutDownLowerLimit            = 0;
    i_LqdPressCmpTbl->ShutDownUpperLimit            = 20;
}

static void StoreLqdPressCmpTbl(LIQUID_PRESSURE_CMP_LINES_Typedef *i_LqdPressCmpTbl)
{
    STMFLASH_Write(SYSTEM_PARAMETER_STORE_SEGMENT_ADDR + RUN_LIQUIDPRESS_CMP_TBL_STORE_OFFSET_ADDR, (uint16_t*)i_LqdPressCmpTbl, 4);
}
/*
***************************************************************************************************
*                                      GetLqdHeightCmpTblFromFlash()
*
* Description:  load the paramters, that related to the liquid press, that has stored in the flash.
*
* Arguments  :  the avriable that store the parameters.
*
* Returns    :  none.
*
* Note(s)    :  none.
***************************************************************************************************
*/
static void GetLqdHeightCmpTblFromFlash(LIQUID_HEIGHT_CMP_LINES_Typedef *i_LqdHeightCmpTbl)
{
    STMFLASH_Read(SYSTEM_PARAMETER_STORE_SEGMENT_ADDR + RUN_LIQUID_HEIGHT_CMP_TBL_STORE_OFFSET_ADDR, (uint16_t*)i_LqdHeightCmpTbl, 4);
}

static void GetDefaultLqdHeightCmpTbl(LIQUID_HEIGHT_CMP_LINES_Typedef *i_LqdHeightCmpTbl)
{
    i_LqdHeightCmpTbl->AlarmlowerLiquidLevellimit = 70;
    i_LqdHeightCmpTbl->OpenAutomaticliquidValue = 100;
    i_LqdHeightCmpTbl->CloseAutomaticliquidValue = 200;
    i_LqdHeightCmpTbl->AlarmUpperLiquidLevellimit = 230;//暂定
}

static void StoreLqdHeightCmpTbl(LIQUID_HEIGHT_CMP_LINES_Typedef *i_LqdHeightCmpTbl)
{
    STMFLASH_Write(SYSTEM_PARAMETER_STORE_SEGMENT_ADDR + RUN_LIQUID_HEIGHT_CMP_TBL_STORE_OFFSET_ADDR, (uint16_t*)i_LqdHeightCmpTbl, 4);
}

/*
***************************************************************************************************
*                                      GetStartHydrgPumpSpdParaFromFlash()
*
* Description:  load the paramters, that related to the speed adjust device, that has stored in the flash.
*
* Arguments  :  the avriable that store the parameters.
*
* Returns    :  none.
*
* Note(s)    :  none.
***************************************************************************************************
*/
void GetStartHydrgPumpSpdParaFromFlash(HYDROGEN_PUMP_SPEED_PARA_Typedef *i_StartPumpSpdPara)
{
    STMFLASH_Read(SYSTEM_PARAMETER_STORE_SEGMENT_ADDR + RUN_HYDROGEN_PUMP_SPEED_PARA_STORE_OFFSET_ADDR, (uint16_t*)i_StartPumpSpdPara, 3);
}

static void GetDefaultStartHydrgPumpSpdPara(HYDROGEN_PUMP_SPEED_PARA_Typedef *i_StartPumpSpdPara)
{
    i_StartPumpSpdPara->PumpSpdIgniterFirstTime   = 190;
    i_StartPumpSpdPara->PumpSpdIgniterSecondTime  = 250;
    i_StartPumpSpdPara->PumpSpdAfterLiquidPressExceed4Kg  = 320;
}

void StoreStartHydrgPumpSpdPara(HYDROGEN_PUMP_SPEED_PARA_Typedef *i_StartPumpSpdPara)
{
    STMFLASH_Write(SYSTEM_PARAMETER_STORE_SEGMENT_ADDR + RUN_HYDROGEN_PUMP_SPEED_PARA_STORE_OFFSET_ADDR, (uint16_t*)i_StartPumpSpdPara, 3);
}

/*
***************************************************************************************************
*                                      GetStartHydrgFanSpdParaFromFlash()
*
* Description:  load the default paramters that related to the speed adjust device.
*
* Arguments  :  the avriable that store the parameters.
*
* Returns    :  none.
*
* Note(s)    :  none.
***************************************************************************************************
*/
void GetStartHydrgFanSpdParaFromFlash(HYDROGEN_FAN_SPEED_PARA_Typedef *i_StartHydrgFanSpdPara)
{
    STMFLASH_Read(SYSTEM_PARAMETER_STORE_SEGMENT_ADDR + RUN_HYDROGEN_FAN_SPEED_PARA_STORE_OFFSET_ADDR, (uint16_t*)i_StartHydrgFanSpdPara, 4);
}

static void GetDefaultStartHydrgFanSpdPara(HYDROGEN_FAN_SPEED_PARA_Typedef *i_StartHydrgFanSpdPara)
{
    i_StartHydrgFanSpdPara->FanSpdIgniterFirstTime   = 1000;
    i_StartHydrgFanSpdPara->FanSpdAfterIgniterFirstSuccessd = 2000;
    i_StartHydrgFanSpdPara->FanSpdIgniterSecondTime   = 1500;
    i_StartHydrgFanSpdPara->FanSpdAfterIgniterSecondSuccessd = 2000;
}

void StoreStartHydrgFanSpdPara(HYDROGEN_FAN_SPEED_PARA_Typedef *i_StartHydrgFanSpdPara)
{
    STMFLASH_Write(SYSTEM_PARAMETER_STORE_SEGMENT_ADDR + RUN_HYDROGEN_FAN_SPEED_PARA_STORE_OFFSET_ADDR, (uint16_t*)i_StartHydrgFanSpdPara, 4);
}
/*
***************************************************************************************************
*                                GetRunPurifyAmpIntegralValueFromFlash()
*
* Description:  load the paramters that related to the hydrogen in and out cmp level.
*
* Arguments  :  the avriable that store the parameters.
*
* Returns    :  none.
*
* Note(s)    :  none.
***************************************************************************************************
*/
static void GetRunPurifyAmpIntegralValueFromFlash(u16 *i_RunPurifyAmpIntegralValue)
{
    STMFLASH_Read(SYSTEM_PARAMETER_STORE_SEGMENT_ADDR + RUN_PURIFY_AMP_INTEGRAL_VALUE, (uint16_t*)i_RunPurifyAmpIntegralValue, 1);
}

static void GetDefaultRunPurifyAmpIntegralValue(u16 *i_RunPurifyAmpIntegralValue)
{
    *i_RunPurifyAmpIntegralValue = 800;
}

static void StoreRunPurifyAmpIntegralValue(u16 *i_RunPurifyAmpIntegralValue)
{
    STMFLASH_Write(SYSTEM_PARAMETER_STORE_SEGMENT_ADDR + RUN_PURIFY_AMP_INTEGRAL_VALUE, (uint16_t*)i_RunPurifyAmpIntegralValue, 1);
}
/*
***************************************************************************************************
*                                      STMFLASH_ReadHalfWord()
*
* Description:  读取指定地址的半字(16位数据)
*
* Arguments  :  读地址(此地址必须为2的倍数!!)
*
* Returns    :  对应数据.
*
* Note(s)    :  none.
***************************************************************************************************
*/
u16 STMFLASH_ReadHalfWord(u32 faddr)
{
    return *(vu16 *)faddr;
}

#if STM32_FLASH_WREN    //如果使能了写功能    
/*
***************************************************************************************************
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
***************************************************************************************************
*/
static void STMFLASH_Write_NoCheck(u32 WriteAddr, u16 *pBuffer, u16 NumToWrite)
{
    u16 i;

    for(i = 0; i < NumToWrite; i++) {
        FLASH_ProgramHalfWord(WriteAddr, pBuffer[i]);
        WriteAddr += 2;//地址增加2.
    }
}

/*
***************************************************************************************************
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
***************************************************************************************************
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

    if((WriteAddr >= STM32_FLASH_BASE) && (WriteAddr < (STM32_FLASH_BASE + 1024 * STM32_FLASH_SIZE))) {
        FLASH_Unlock();                                 //解锁
        offaddr = WriteAddr - STM32_FLASH_BASE;         //实际偏移地址.
        secpos = offaddr / STM_SECTOR_SIZE;             //扇区地址  0~127 共128个扇区，在STM32F105RB中，每个扇区1K
        secoff = (offaddr % STM_SECTOR_SIZE) / 2;       //在扇区内的偏移(2个字节为基本单位.)
        secremain = STM_SECTOR_SIZE / 2 - secoff;       //扇区剩余空间大小

        if(NumToWrite <= secremain) {
            secremain = NumToWrite;//不大于该扇区范围   //否则后续会启动下一个扇区的书写
        }

        while(1) {
            STMFLASH_Read(secpos * STM_SECTOR_SIZE + STM32_FLASH_BASE, STMFLASH_BUF, STM_SECTOR_SIZE / 2);//读出整个扇区的内容

            for(i = 0; i < secremain; i++) { //校验数据
                if(STMFLASH_BUF[secoff + i] != 0XFFFF) {
                    break;//需要擦除
                }
            }

            if(i < secremain) { //需要擦除
                FLASH_ErasePage(secpos * STM_SECTOR_SIZE + STM32_FLASH_BASE);//擦除这个扇区

                for(i = 0; i < secremain; i++) { //复制
                    STMFLASH_BUF[i + secoff] = pBuffer[i];
                }

                STMFLASH_Write_NoCheck(secpos * STM_SECTOR_SIZE + STM32_FLASH_BASE, STMFLASH_BUF, STM_SECTOR_SIZE / 2);//写入整个扇区
            } else {
                STMFLASH_Write_NoCheck(WriteAddr, pBuffer, secremain);//写已经擦除了的,直接写入扇区剩余区间.
            }

            if(NumToWrite == secremain) {
                break;//写入结束了
            } else { //写入未结束
                secpos ++;              //扇区地址增1
                secoff = 0;             //偏移位置为0
                pBuffer += secremain;   //指针偏移
                WriteAddr += secremain; //写地址偏移
                NumToWrite -= secremain;    //字节(16位)数递减

                if(NumToWrite > (STM_SECTOR_SIZE / 2)) {
                    secremain = STM_SECTOR_SIZE / 2;//下一个扇区还是写不完
                } else {
                    secremain = NumToWrite;//下一个扇区可以写完了
                }
            }
        }

        FLASH_Lock();//上锁
    }
}
#endif

/*
***************************************************************************************************
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
***************************************************************************************************
*/
void STMFLASH_Read(u32 ReadAddr, u16 *pBuffer, u16 NumToRead)
{
    u16 i;

    for(i = 0; i < NumToRead; i++) {
        pBuffer[i] = STMFLASH_ReadHalfWord(ReadAddr);//读取2个字节.
        ReadAddr += 2;//偏移2个字节.
    }
}

#endif
/******************* (C) COPYRIGHT 2015 Guangdong Hydrogen *****END OF FILE****/

