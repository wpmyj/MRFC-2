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
* Programmer(s) : JasonFan
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
#include "app_wireness_communicate_task.h"
/*
***************************************************************************************************
*                                           MACRO DEFINITIONS
***************************************************************************************************
*/
#define STM32_FLASH_SIZE            256             //所用STM32的FLASH容量大小(单位为K)
#define STM32_FLASH_WREN            1               //使能FLASH写入(0:失能;1:使能)

#define STM32_FLASH_BASE            0x08000000      //STM32 FLASH的起始地址

//0x0801FC00
#define SYSTEM_PARA_STORE_SEGMENT_ADDR         0x0803F800      //地址頂端為0x0803FFFF 设置FLASH 保存地址為最後的2K字節(必须为偶数)

#define PARAMETERS_STORE_AREA_SIZE                      256 //参数保存区的大小（字节）

//下面宏定义表示开机运行的驱动层参数（传感器）和应用层工作参数是采用设定值还是默认值的标志值的偏移地址
//无符号16位数，擦除后为全0xFFFF
//bit[0]表示应用层是采用默认值还是配置值--0:配置值  1:默认值
//bit[1]表示驱动层参数采用默认值还是配置--0:配置值  1:默认值
#define APP_PARAMETERS_SELECT_FLAG_CHECK                                        0x0001
#define APP_PARAMETERS_SELECT_CFG_VALUE                                         0x0000
#define APP_PARAMETERS_SELECT_DEFAULT_VALUE                                     0x0001

#define DRIVER_LAYER_PARAMETERS_SELECT_FLAG_CHECK                               0x0002
#define DRIVER_LAYER_PARAMETERS_SELECT_CFG_VALUE                                0x0000  //使用配置过的参数
#define DRIVER_LAYER_PARAMETERS_SELECT_DEFAULT_VALUE                            0x0002  //使用默认参数

/*以下宏定义表示参数存储基地址*/
#define PARAMETERS_SELECT_FLAG_OFFSET_ADDR                                      0   //参数选择标志
#define SYSTEM_RUN_TIME_STORE_OFFSET_ADDR                                       2   //系统运行次数
#define GLOBAL_NET_WORK_ID_ADDR                                                 4   //本地组网ID
#define SYSTEM_TOTAL_WORK_TIME_STORE_OFFSET_ADDR                                6   //系统时间时间4个，各4个字节
#define RUN_REFORMER_CMP_TBL_STORE_OFFSET_ADDR                                  30  //12个数
#define RUN_LIQUIDPRESS_CMP_TBL_STORE_OFFSET_ADDR                               50  // 液压4个数
#define RUN_LIQUID_HEIGHT_CMP_TBL_STORE_OFFSET_ADDR                             60  //液位参数4个
#define RUN_HYDROGEN_PUMP_SPEED_PARA_STORE_OFFSET_ADDR                          70  //泵参数3个
#define RUN_HYDROGEN_FAN_SPEED_PARA_STORE_OFFSET_ADDR                           80  //制氢风机参数4个
#define RUN_PURIFY_AMP_INTEGRAL_VALUE                                           90  //电堆提纯膜间电荷量1个
#define DRIVER_LAYER_CALIBRATED_LINEAR_ANA_SENSOR_PARAMETERS_OFFSET_ADDR        92  //驱动层线性传感器校准参数,28个数
#define RUNNING_STATUS_DELAY_ADJUST_SPEED_PARAMETERS_ADDR                       130 //运行调速设备延时调节参数 
#define FIRST_TIME_HEAT_HOLD_SECONDS_PARA_ADDR                                  142 //加热持续时间
#define RICH_HYDROGEN_MODE_PARA_STORE_OFFSET_ADDR                           	144 //富氢参数18个，全当uint16

/*
***************************************************************************************************
*                                     GLOBAL  VARIABLES
***************************************************************************************************
*/
REFORMER_TEMP_CMP_LINES_Typedef                         g_stReformerTempCmpTbl;
LIQUID_PRESSURE_CMP_LINES_Typedef                       g_stLqdPressCmpTbl;
LIQUID_HEIGHT_CMP_LINES_Typedef                         g_stLqdHeightCmpTbl;
HYDROGEN_PUMP_SPEED_PARA_Typedef                        g_stStartHydrgPumpSpdPara;
HYDROGEN_FAN_SPEED_PARA_Typedef                         g_stStartHydrogenFanSpdPara;
RUNNING_STATUS_DELAY_ADJUST_SPEED_PARAMETERS_Typedef    g_stRunningStatusDelayAdjustSpdPara;
RICH_HYDROGEN_ACTIVE_PARA_Typedef                       g_stRichHydrogenModePara;
uint16_t                                                g_u16FirstTimeHeatHoldSeconds;
uint16_t                                                g_u16RunPurifyAmpIntegralValue;

//在CAN总线中的节点ID/本地组网ID,默认为255号,独立运行的机器设置为0
uint16_t                                    g_u16GlobalNetWorkId = 0x20;//0xFF
//PS3:0x1100,MRFC-2:0x1101,发电模块0100，制氢机0x1000、1001
uint16_t                                   g_ProductsType = 0x1000;  
/*
***************************************************************************************************
*                                     LOCAL VARIABLES
***************************************************************************************************
*/

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

static          void        GetDefaultRunningStatusDelayAdjustSpdPara(RUNNING_STATUS_DELAY_ADJUST_SPEED_PARAMETERS_Typedef *i_DelayAndAdjustSpd);
static          void        StoreRunningStatusDelayAdjustSpdPara(RUNNING_STATUS_DELAY_ADJUST_SPEED_PARAMETERS_Typedef *i_DelayAndAdjustSpdPara);
static          void        GetRunningStageDelayAndAdjustSpdFromFlash(RUNNING_STATUS_DELAY_ADJUST_SPEED_PARAMETERS_Typedef *i_DelayAndAdjustSpd);

static          void        GetGlobalNetWorkIDFromFlash(u16 *i_GlobalNetWorkID);
static          void        GetDefaultFirstTimeHeatHoldSeconds(u16 *i_FirstTimeHeatHoldSeconds);
static          void        GetFirstTimeHeatHoldSecondsFromFlash(u16 *i_FirstTimeHeatHoldSeconds);

static          void        GetRichHydrogenModeParaFromFlash(RICH_HYDROGEN_ACTIVE_PARA_Typedef *i_RichModePara);
static          void        GetDefaultRichHydrogenModePara(RICH_HYDROGEN_ACTIVE_PARA_Typedef *i_RichModePara);
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

        LoadSystemWorkTimesToPrgm(u16WorkTimes);//使用记录的参数,运行次数暂不清零
        APP_TRACE_INFO(("The %d time for the system to boot,loading the parameters..\r\n", u16WorkTimes + 1));

        GetTotalWorkTimeFromFlash(&stTotalWorkTime);
		APP_TRACE_INFO(("stTotalWorkTime:%d...\n\r", stTotalWorkTime.hour));
        LoadTotalWorkTimeToPrgm(stTotalWorkTime);   //传到实时运行参数所在文件对应的程序中去
	
        GetGlobalNetWorkIDFromFlash(&g_u16GlobalNetWorkId);
        APP_TRACE_INFO(("Load g_u16GlobalNetWorkId:%d...\n\r", g_u16GlobalNetWorkId));
        GetReformerTempCmpTblFromFlash(&g_stReformerTempCmpTbl);
        GetLqdPressCmpTblFromFlash(&g_stLqdPressCmpTbl);
		APP_TRACE_INFO(("g_stLqdPressAlarmLower:%d...\n\r", g_stLqdPressCmpTbl.AlarmUpperLimit));
        GetLqdHeightCmpTblFromFlash(&g_stLqdHeightCmpTbl);
		APP_TRACE_INFO(("stTotalWorkTime:%d...\n\r", stTotalWorkTime.hour));
        GetStartHydrgPumpSpdParaFromFlash(&g_stStartHydrgPumpSpdPara);
        GetStartHydrgFanSpdParaFromFlash(&g_stStartHydrogenFanSpdPara);
        GetRunPurifyAmpIntegralValueFromFlash(&g_u16RunPurifyAmpIntegralValue);
        GetRunningStageDelayAndAdjustSpdFromFlash(&g_stRunningStatusDelayAdjustSpdPara);
        GetFirstTimeHeatHoldSecondsFromFlash(&g_u16FirstTimeHeatHoldSeconds);
		APP_TRACE_INFO(("g_u16FirstTimeHeatHoldSeconds:%d...\n\r", g_u16FirstTimeHeatHoldSeconds));
		GetRunningStageDelayAndAdjustSpdFromFlash(&g_stRunningStatusDelayAdjustSpdPara);
        GetRichHydrogenModeParaFromFlash(&g_stRichHydrogenModePara);
		APP_TRACE_INFO(("g_stRichHydrogenModePara:%d...\n\r", g_stRichHydrogenModePara.ActiveStep1FanSpd));

    } else { //载入默认参数
        APP_TRACE_INFO(("First time run,the machine will work with the default parameters!...\r\n"));

        u16ParametersSelectFlag &= ~APP_PARAMETERS_SELECT_DEFAULT_VALUE;//下次启动采用配置好的参数
        SetParametersSelectFlag(&u16ParametersSelectFlag);//保存参数设置标志

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
        StoreGlobalNetWorkID(&g_u16GlobalNetWorkId);
        APP_TRACE_INFO(("Default GlobalNetWorkId:%d...\n\r", g_u16GlobalNetWorkId));

        GetDefaultReformerTempCmpTbl(&g_stReformerTempCmpTbl);
        StoreReformerTempCmpTbl(&g_stReformerTempCmpTbl);

        GetDefaultLqdPressCmpTbl(&g_stLqdPressCmpTbl);
        StoreLqdPressCmpTbl(&g_stLqdPressCmpTbl);

        GetDefaultLqdHeightCmpTbl(&g_stLqdHeightCmpTbl);
        StoreLqdHeightCmpTbl(&g_stLqdHeightCmpTbl);

        GetDefaultStartHydrgPumpSpdPara(&g_stStartHydrgPumpSpdPara);
        StoreStartHydrgPumpSpdPara(&g_stStartHydrgPumpSpdPara);

        GetDefaultStartHydrgFanSpdPara(&g_stStartHydrogenFanSpdPara);
        StoreStartHydrgFanSpdPara(&g_stStartHydrogenFanSpdPara);

        GetDefaultRunPurifyAmpIntegralValue(&g_u16RunPurifyAmpIntegralValue);
        StoreRunPurifyAmpIntegralValue(&g_u16RunPurifyAmpIntegralValue);

        GetDefaultFirstTimeHeatHoldSeconds(&g_u16FirstTimeHeatHoldSeconds);
        StoreFirstTimeHeatHoldSeconds(&g_u16FirstTimeHeatHoldSeconds);

        GetDefaultRunningStatusDelayAdjustSpdPara(&g_stRunningStatusDelayAdjustSpdPara);
        StoreRunningStatusDelayAdjustSpdPara(&g_stRunningStatusDelayAdjustSpdPara);
		
		GetDefaultFirstTimeHeatHoldSeconds(&g_u16FirstTimeHeatHoldSeconds);
        StoreFirstTimeHeatHoldSeconds(&g_u16FirstTimeHeatHoldSeconds);
        
        GetDefaultRichHydrogenModePara(&g_stRichHydrogenModePara);
        StoreRichHydrogenModePara(&g_stRichHydrogenModePara);
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
    STMFLASH_Read(SYSTEM_PARA_STORE_SEGMENT_ADDR + PARAMETERS_SELECT_FLAG_OFFSET_ADDR, o_pParametersSelectFlagAddr, 1);//获取运行参数选择标志
}

static void SetParametersSelectFlag(uint16_t *i_pParametersSelectFlagAddr)
{
    STMFLASH_Write(SYSTEM_PARA_STORE_SEGMENT_ADDR + PARAMETERS_SELECT_FLAG_OFFSET_ADDR, i_pParametersSelectFlagAddr, 1);
}

/*
*********************************************************************************************************
*                                      StoreGlobalNetWorkID()
*
* Description:  save the paramters that related to the network ID.
*
* Arguments  :  the buff that store the parameters.
*
* Returns    :  none.
*
* Note(s)    :  none.
*********************************************************************************************************
*/
static void GetGlobalNetWorkIDFromFlash(u16 *i_GlobalNetWorkID)
{
    STMFLASH_Read(SYSTEM_PARA_STORE_SEGMENT_ADDR + GLOBAL_NET_WORK_ID_ADDR, i_GlobalNetWorkID, 1);
}

void StoreGlobalNetWorkID(u16 *i_GlobalNetWorkID)
{
    STMFLASH_Write(SYSTEM_PARA_STORE_SEGMENT_ADDR + GLOBAL_NET_WORK_ID_ADDR, i_GlobalNetWorkID, 1);
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
    STMFLASH_Write(SYSTEM_PARA_STORE_SEGMENT_ADDR + SYSTEM_RUN_TIME_STORE_OFFSET_ADDR, &i_u16WorkTimes, 1);
}

static void GetSystemWorkTimesFromFlash(u16 *o_StoreAddr)
{
    STMFLASH_Read(SYSTEM_PARA_STORE_SEGMENT_ADDR + SYSTEM_RUN_TIME_STORE_OFFSET_ADDR, o_StoreAddr, 1);//获取运行次数
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
    STMFLASH_Read(SYSTEM_PARA_STORE_SEGMENT_ADDR + DRIVER_LAYER_CALIBRATED_LINEAR_ANA_SENSOR_PARAMETERS_OFFSET_ADDR,
                  (uint16_t *)o_stAnaSensorParameters, i_u8Length * 4); //结构体的长度是u16类型数据长度的4倍
}

void BSP_StoreAnaSensorParaToFlash(ANALOG_SIGNAL_SERSOR_PARAMETERS_Typedef *i_stAnaSensorParameters, uint8_t i_u8Length)
{
    STMFLASH_Write(SYSTEM_PARA_STORE_SEGMENT_ADDR + DRIVER_LAYER_CALIBRATED_LINEAR_ANA_SENSOR_PARAMETERS_OFFSET_ADDR,
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
    STMFLASH_Read(SYSTEM_PARA_STORE_SEGMENT_ADDR + SYSTEM_TOTAL_WORK_TIME_STORE_OFFSET_ADDR, (uint16_t *)o_StoreAddr, 16);
}

void StoreTotalWorkTime(SYSTEM_TIME_Typedef *i_stTotalTime)
{
    STMFLASH_Write(SYSTEM_PARA_STORE_SEGMENT_ADDR + SYSTEM_TOTAL_WORK_TIME_STORE_OFFSET_ADDR, (uint16_t *)i_stTotalTime, 16);
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
    STMFLASH_Read(SYSTEM_PARA_STORE_SEGMENT_ADDR + RUN_REFORMER_CMP_TBL_STORE_OFFSET_ADDR, (uint16_t *)i_ReformerTemCmpTbl, 12);
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
    STMFLASH_Write(SYSTEM_PARA_STORE_SEGMENT_ADDR + RUN_REFORMER_CMP_TBL_STORE_OFFSET_ADDR, (uint16_t *)i_ReformerTemCmpTbl, 12);
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
    STMFLASH_Read(SYSTEM_PARA_STORE_SEGMENT_ADDR + RUN_LIQUIDPRESS_CMP_TBL_STORE_OFFSET_ADDR, (uint16_t *)i_LqdPressCmpTbl, 4);
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
    STMFLASH_Write(SYSTEM_PARA_STORE_SEGMENT_ADDR + RUN_LIQUIDPRESS_CMP_TBL_STORE_OFFSET_ADDR, (uint16_t *)i_LqdPressCmpTbl, 4);
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
    STMFLASH_Read(SYSTEM_PARA_STORE_SEGMENT_ADDR + RUN_LIQUID_HEIGHT_CMP_TBL_STORE_OFFSET_ADDR, (uint16_t *)i_LqdHeightCmpTbl, 3);
}

static void GetDefaultLqdHeightCmpTbl(LIQUID_HEIGHT_CMP_LINES_Typedef *i_LqdHeightCmpTbl)
{
    i_LqdHeightCmpTbl->AlarmlowerLiquidLevellimit = 70;
    i_LqdHeightCmpTbl->OpenAutomaticliquidValue = 100;
    i_LqdHeightCmpTbl->CloseAutomaticliquidValue = 200;
}

static void StoreLqdHeightCmpTbl(LIQUID_HEIGHT_CMP_LINES_Typedef *i_LqdHeightCmpTbl)
{
    STMFLASH_Write(SYSTEM_PARA_STORE_SEGMENT_ADDR + RUN_LIQUID_HEIGHT_CMP_TBL_STORE_OFFSET_ADDR, (uint16_t *)i_LqdHeightCmpTbl, 3);
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
    STMFLASH_Read(SYSTEM_PARA_STORE_SEGMENT_ADDR + RUN_HYDROGEN_PUMP_SPEED_PARA_STORE_OFFSET_ADDR, (uint16_t *)i_StartPumpSpdPara, 3);
}

static void GetDefaultStartHydrgPumpSpdPara(HYDROGEN_PUMP_SPEED_PARA_Typedef *i_StartPumpSpdPara)
{
    i_StartPumpSpdPara->PumpSpdIgniterFirstTime   = 190;
    i_StartPumpSpdPara->PumpSpdIgniterSecondTime  = 360;
    i_StartPumpSpdPara->PumpSpdAfterLiquidPressExceed4Kg  = 320;//稳定运行的泵速
}

void StoreStartHydrgPumpSpdPara(HYDROGEN_PUMP_SPEED_PARA_Typedef *i_StartPumpSpdPara)
{
    STMFLASH_Write(SYSTEM_PARA_STORE_SEGMENT_ADDR + RUN_HYDROGEN_PUMP_SPEED_PARA_STORE_OFFSET_ADDR, (uint16_t *)i_StartPumpSpdPara, 3);
}

//单独一个水泵的参数保存
void StoreStartHydrgPumpSpdParaBySingle(uint16_t *i_StartPumpSpdPara, u8 DataAddr)
{
    STMFLASH_Write(SYSTEM_PARA_STORE_SEGMENT_ADDR + RUN_HYDROGEN_PUMP_SPEED_PARA_STORE_OFFSET_ADDR + 2 * DataAddr, i_StartPumpSpdPara, 1);
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
    STMFLASH_Read(SYSTEM_PARA_STORE_SEGMENT_ADDR + RUN_HYDROGEN_FAN_SPEED_PARA_STORE_OFFSET_ADDR, (uint16_t *)i_StartHydrgFanSpdPara, 4);
}

static void GetDefaultStartHydrgFanSpdPara(HYDROGEN_FAN_SPEED_PARA_Typedef *i_StartHydrgFanSpdPara)
{
    i_StartHydrgFanSpdPara->FanSpdIgniterFirstTime   = 1000;
    i_StartHydrgFanSpdPara->FanSpdAfterIgniterFirstSuccessd = 2000;
    i_StartHydrgFanSpdPara->FanSpdIgniterSecondTime   = 1300;
    i_StartHydrgFanSpdPara->FanSpdAfterIgniterSecondSuccessd = 2000;
}

void StoreStartHydrgFanSpdPara(HYDROGEN_FAN_SPEED_PARA_Typedef *i_StartHydrgFanSpdPara)
{
    STMFLASH_Write(SYSTEM_PARA_STORE_SEGMENT_ADDR + RUN_HYDROGEN_FAN_SPEED_PARA_STORE_OFFSET_ADDR, (uint16_t *)i_StartHydrgFanSpdPara, 4);
}

//单独一个风机的参数保存
void StoreStartHydrgFanSpdParaBySingle(uint16_t *i_StartHydrgFanSpdPara, u8 DataAddr)
{
    STMFLASH_Write(SYSTEM_PARA_STORE_SEGMENT_ADDR + RUN_HYDROGEN_FAN_SPEED_PARA_STORE_OFFSET_ADDR + 2 * DataAddr, i_StartHydrgFanSpdPara, 1);
}

/*
*********************************************************************************************************
*                                      GetRunningStageDelayAndAdjustSpdFromFlash()
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
static void GetRunningStageDelayAndAdjustSpdFromFlash(RUNNING_STATUS_DELAY_ADJUST_SPEED_PARAMETERS_Typedef *i_DelayAndAdjustSpd)
{
    STMFLASH_Read(SYSTEM_PARA_STORE_SEGMENT_ADDR + RUNNING_STATUS_DELAY_ADJUST_SPEED_PARAMETERS_ADDR, (uint16_t *)i_DelayAndAdjustSpd, 6);
}

static void GetDefaultRunningStatusDelayAdjustSpdPara(RUNNING_STATUS_DELAY_ADJUST_SPEED_PARAMETERS_Typedef *i_DelayAndAdjustSpd)
{
    i_DelayAndAdjustSpd->FirstDelayTimeByMin       = 2;
    i_DelayAndAdjustSpd->FirstTimeAdjustPumpValue  = 20;
    i_DelayAndAdjustSpd->FirstTimeAdjustFanValue   = 20;

    i_DelayAndAdjustSpd->SecondDelayTimeByMin      = 2;
    i_DelayAndAdjustSpd->SecondTimeAdjustPumpValue = 20;
    i_DelayAndAdjustSpd->SecondTimeAdjustFanValue  = 20;
}

static void StoreRunningStatusDelayAdjustSpdPara(RUNNING_STATUS_DELAY_ADJUST_SPEED_PARAMETERS_Typedef *i_DelayAndAdjustSpdPara)
{
    STMFLASH_Write(SYSTEM_PARA_STORE_SEGMENT_ADDR + RUNNING_STATUS_DELAY_ADJUST_SPEED_PARAMETERS_ADDR, (u16 *)i_DelayAndAdjustSpdPara, 6);
}

//延时调速单独两个字节的数据保存
void StoreRunningStatusDelayAdjustSpdParaBySingle(u16 *i_DelayAndAdjustSpdPara, u8 DataAddr)
{
    STMFLASH_Write(SYSTEM_PARA_STORE_SEGMENT_ADDR + RUNNING_STATUS_DELAY_ADJUST_SPEED_PARAMETERS_ADDR + 2 * DataAddr, i_DelayAndAdjustSpdPara, 1);
}

/*
*********************************************************************************************************
*                                      GetFirstTimeHeatHoldSecondsFromFlash()
*
* Description:  load the paramters, that related to the fire heat time by the first ignite, that has stored in the flash.
*
* Arguments  :  the avriable that store the parameters.
*
* Returns    :  none.
*
* Note(s)    :  none.
*********************************************************************************************************
*/
void GetFirstTimeHeatHoldSecondsFromFlash(u16 *i_FirstTimeHeatHoldSeconds)
{
    STMFLASH_Read(SYSTEM_PARA_STORE_SEGMENT_ADDR + FIRST_TIME_HEAT_HOLD_SECONDS_PARA_ADDR, i_FirstTimeHeatHoldSeconds, 1);
}

void StoreFirstTimeHeatHoldSeconds(u16 *i_FirstTimeHeatHoldSeconds)
{
    STMFLASH_Write(SYSTEM_PARA_STORE_SEGMENT_ADDR + FIRST_TIME_HEAT_HOLD_SECONDS_PARA_ADDR, i_FirstTimeHeatHoldSeconds, 1);
}

static void GetDefaultFirstTimeHeatHoldSeconds(u16 *i_FirstTimeHeatHoldSeconds)
{
    *i_FirstTimeHeatHoldSeconds = 180;
}



//存储富氢模式的一些参数
void StoreRichHydrogenModeFanPara(uint8_t * RecBuf,uint8_t i_eActiveStep)
{
    switch((uint8_t)i_eActiveStep){
        case ACTIVE_STEP_ONE:
            g_stRichHydrogenModePara.ActiveStep1FanSpd = (uint8_t)(*(RecBuf + REC_DATA_BYTE_CMD_PARA_SECTION_5)) + (uint8_t)(*(RecBuf + REC_DATA_BYTE_CMD_PARA_SECTION_6));
            break;
        case ACTIVE_STEP_TWO:
            g_stRichHydrogenModePara.ActiveStep2FanSpd = (uint8_t)(*(RecBuf + REC_DATA_BYTE_CMD_PARA_SECTION_5)) + (uint8_t)(*(RecBuf + REC_DATA_BYTE_CMD_PARA_SECTION_6));
            break;
        case ACTIVE_STEP_THREE:
            g_stRichHydrogenModePara.ActiveStep3FanSpd = (uint8_t)(*(RecBuf + REC_DATA_BYTE_CMD_PARA_SECTION_5)) + (uint8_t)(*(RecBuf + REC_DATA_BYTE_CMD_PARA_SECTION_6));
            break;
        case ACTIVE_STEP_FOUR:
            g_stRichHydrogenModePara.ActiveStep4FanSpd = (uint8_t)(*(RecBuf + REC_DATA_BYTE_CMD_PARA_SECTION_5)) + (uint8_t)(*(RecBuf + REC_DATA_BYTE_CMD_PARA_SECTION_6));
            break;
        default:
            break;
        }
}

void StoreRichHydrogenModePumpPara(uint8_t * RecBuf,uint8_t i_eActiveStep)
{
    switch((uint8_t)i_eActiveStep){
        case ACTIVE_STEP_ONE:
            g_stRichHydrogenModePara.ActiveStep1PumpSpd = (uint8_t)(*(RecBuf + REC_DATA_BYTE_CMD_PARA_SECTION_5)) + (uint8_t)(*(RecBuf + REC_DATA_BYTE_CMD_PARA_SECTION_6));
            break;
        case ACTIVE_STEP_TWO:
            g_stRichHydrogenModePara.ActiveStep2PumpSpd = (uint8_t)(*(RecBuf + REC_DATA_BYTE_CMD_PARA_SECTION_5)) + (uint8_t)(*(RecBuf + REC_DATA_BYTE_CMD_PARA_SECTION_6));
            break;
        case ACTIVE_STEP_THREE:
            g_stRichHydrogenModePara.ActiveStep3PumpSpd = (uint8_t)(*(RecBuf + REC_DATA_BYTE_CMD_PARA_SECTION_5)) + (uint8_t)(*(RecBuf + REC_DATA_BYTE_CMD_PARA_SECTION_6));
            break;
        case ACTIVE_STEP_FOUR:
            g_stRichHydrogenModePara.ActiveStep4PumpSpd = (uint8_t)(*(RecBuf + REC_DATA_BYTE_CMD_PARA_SECTION_5)) + (uint8_t)(*(RecBuf + REC_DATA_BYTE_CMD_PARA_SECTION_6));
            break;
        default:
            break;
   }
}

void StoreRichHydrogenModeHoldTimePara(uint8_t * RecBuf,uint8_t i_eActiveStep)
{
    switch((uint8_t)i_eActiveStep){
        case ACTIVE_STEP_ONE:
            g_stRichHydrogenModePara.ActiveStep1HoldHour = (uint8_t)(*(RecBuf + REC_DATA_BYTE_CMD_PARA_SECTION_5));
            g_stRichHydrogenModePara.ActiveStep1HoldMin =  (uint8_t)(*(RecBuf + REC_DATA_BYTE_CMD_PARA_SECTION_6));
            break;
        case ACTIVE_STEP_TWO:
            g_stRichHydrogenModePara.ActiveStep2HoldHour = (uint8_t)(*(RecBuf + REC_DATA_BYTE_CMD_PARA_SECTION_5));
            g_stRichHydrogenModePara.ActiveStep2HoldMin =  (uint8_t)(*(RecBuf + REC_DATA_BYTE_CMD_PARA_SECTION_6));
            break;
        case ACTIVE_STEP_THREE:
            g_stRichHydrogenModePara.ActiveStep3HoldHour = (uint8_t)(*(RecBuf + REC_DATA_BYTE_CMD_PARA_SECTION_5));
            g_stRichHydrogenModePara.ActiveStep3HoldMin =  (uint8_t)(*(RecBuf + REC_DATA_BYTE_CMD_PARA_SECTION_6));
            break;
        case ACTIVE_STEP_FOUR:
            g_stRichHydrogenModePara.ActiveStep4HoldHour = (uint8_t)(*(RecBuf + REC_DATA_BYTE_CMD_PARA_SECTION_5));
            g_stRichHydrogenModePara.ActiveStep4HoldMin =  (uint8_t)(*(RecBuf + REC_DATA_BYTE_CMD_PARA_SECTION_6));
            break;
        default:
            break;
        }
}

void  LoadCurrentStepRemainTimePara(uint8_t i_CurrentActiveStep)
{
    switch((uint8_t)i_CurrentActiveStep){
        case ACTIVE_STEP_ONE:
            g_stRichHydrogenModePara.CurrentActiveStepHoldHour = g_stRichHydrogenModePara.ActiveStep1HoldHour;
            g_stRichHydrogenModePara.CurrentActiveStepHoldMin = g_stRichHydrogenModePara.ActiveStep1HoldMin;
            break;
        case ACTIVE_STEP_TWO:
            g_stRichHydrogenModePara.CurrentActiveStepHoldHour = g_stRichHydrogenModePara.ActiveStep2HoldHour;
            g_stRichHydrogenModePara.CurrentActiveStepHoldMin = g_stRichHydrogenModePara.ActiveStep2HoldMin;
            break;
        case ACTIVE_STEP_THREE:
            g_stRichHydrogenModePara.CurrentActiveStepHoldHour = g_stRichHydrogenModePara.ActiveStep3HoldHour;
            g_stRichHydrogenModePara.CurrentActiveStepHoldMin = g_stRichHydrogenModePara.ActiveStep3HoldMin;
            break;
        case ACTIVE_STEP_FOUR:
            g_stRichHydrogenModePara.CurrentActiveStepHoldHour = g_stRichHydrogenModePara.ActiveStep4HoldHour;
            g_stRichHydrogenModePara.CurrentActiveStepHoldMin = g_stRichHydrogenModePara.ActiveStep4HoldMin;
            break;
        default:
            break;
    }

}


static void GetRichHydrogenModeParaFromFlash(RICH_HYDROGEN_ACTIVE_PARA_Typedef *i_RichModePara)
{
    STMFLASH_Read(SYSTEM_PARA_STORE_SEGMENT_ADDR + RICH_HYDROGEN_MODE_PARA_STORE_OFFSET_ADDR, (uint16_t *)i_RichModePara, 18);
}


static void GetDefaultRichHydrogenModePara(RICH_HYDROGEN_ACTIVE_PARA_Typedef *i_RichModePara)
{
    i_RichModePara->ActiveStep = 1;
    i_RichModePara->ActiveStep1FanSpd = 100;
    i_RichModePara->ActiveStep1PumpSpd = 200;
    i_RichModePara->ActiveStep1HoldHour = 2;
    i_RichModePara->ActiveStep1HoldMin = 30;
    
    i_RichModePara->ActiveStep2FanSpd = 200;
    i_RichModePara->ActiveStep2PumpSpd = 210;
    i_RichModePara->ActiveStep2HoldHour = 3;
    i_RichModePara->ActiveStep2HoldMin = 30;
    
    i_RichModePara->ActiveStep3FanSpd = 400;
    i_RichModePara->ActiveStep3PumpSpd = 250;
    i_RichModePara->ActiveStep3HoldHour = 1;
    i_RichModePara->ActiveStep3HoldMin = 50;
    
    i_RichModePara->ActiveStep4FanSpd = 700;
    i_RichModePara->ActiveStep4PumpSpd = 270;
    i_RichModePara->ActiveStep4HoldHour = 1;
    i_RichModePara->ActiveStep4HoldMin = 20;
    
    LoadCurrentStepRemainTimePara(i_RichModePara->ActiveStep);
}

static void StoreRichHydrogenModePara(RICH_HYDROGEN_ACTIVE_PARA_Typedef *i_RichModePara)
{
    STMFLASH_Write(SYSTEM_PARA_STORE_SEGMENT_ADDR + RICH_HYDROGEN_MODE_PARA_STORE_OFFSET_ADDR, (u16 *)i_RichModePara, 18);
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
    STMFLASH_Read(SYSTEM_PARA_STORE_SEGMENT_ADDR + RUN_PURIFY_AMP_INTEGRAL_VALUE, (uint16_t *)i_RunPurifyAmpIntegralValue, 1);
}

static void GetDefaultRunPurifyAmpIntegralValue(u16 *i_RunPurifyAmpIntegralValue)
{
    *i_RunPurifyAmpIntegralValue = 800;
}

static void StoreRunPurifyAmpIntegralValue(u16 *i_RunPurifyAmpIntegralValue)
{
    STMFLASH_Write(SYSTEM_PARA_STORE_SEGMENT_ADDR + RUN_PURIFY_AMP_INTEGRAL_VALUE, (uint16_t *)i_RunPurifyAmpIntegralValue, 1);
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
uint16_t STMFLASH_ReadHalfWord(uint32_t faddr)
{
    return *(vu16 *)faddr;
}

uint8_t STMFLASH_ReadByte(uint32_t faddr)
{
    return *(vu8 *)faddr;
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
static      u16     STMFLASH_BUF[STM_SECTOR_SIZE / 2];
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
        secpos = offaddr / STM_SECTOR_SIZE;             //扇区地址  0~127 共128个扇区，每个扇区2K
        secoff = (offaddr % STM_SECTOR_SIZE) / 2;       //在扇区内的偏移(2个字节为基本单位.)
        secremain = STM_SECTOR_SIZE / 2 - secoff;       //扇区剩余空间大小

        if(NumToWrite <= secremain) {
            secremain = NumToWrite;//不大于该扇区范围   //否则后续会启动下一个扇区的书写
        }

        while(1) {
            STMFLASH_Read(secpos * STM_SECTOR_SIZE + STM32_FLASH_BASE, STMFLASH_BUF, STM_SECTOR_SIZE / 2);//读出整个扇区的内容

            for(i = 0; i < secremain; i++) { //校验数据是否擦除过
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
/******************* (C) COPYRIGHT 2016 Guangdong ENECO POWER *****END OF FILE****/

