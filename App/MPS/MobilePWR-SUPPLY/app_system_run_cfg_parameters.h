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
* Filename      :  app_system_run_cfg_parameters.h
* Programmer(s) :  Fanjun
* Version       :  V1.0
* data          :  2016.12.10
* brief         :This file contains all the functions prototypes for the system run
*                config parameters firmware library.
*********************************************************************************************************/
#ifndef __APP_SYSTEM_RUN_CFG_PARAMETERS_H__
#define __APP_SYSTEM_RUN_CFG_PARAMETERS_H__
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
#include "includes.h"
/*
***************************************************************************************************
*                                           MACRO DEFINITIONS
***************************************************************************************************
*/
#ifndef __INTERNAL_TEST_FLAG
    #define __INTERNAL_TEST_FLAG    0u      //Internal debug mode
    #define __HYDROGEN_GENERATOR_MODULE 0u
    #define __FUEL_CELL_MODULE    0u
    #define __ONE_MACHINE_MODULE    1u
#endif

#define     SYSTEM_STACK_USAGE_CHECK     0  //系统任务堆栈监测使能开关   

/*
***************************************************************************************************
*                                           EXPORTED DATA TYPE
***************************************************************************************************
*/
typedef struct {

    uint16_t IgFstTimeFrtToBhdTmpPnt;
    uint16_t IgFstTimeOverTmpPnt;
    uint8_t IgFstTimeWatiTimeMax1;
    uint8_t IgFstTimeWatiTimeMax2;

    uint16_t IgScdTimeFrtToBhdTmpPnt;
    uint16_t IgScdTimeOverTmpPnt;
    uint8_t IgScdTimeWatiTimeMax1;
    uint8_t IgScdTimeWatiTimeMax2;

    uint16_t AlarmLowerLimit;
    uint16_t AlarmUpperLimit;

    uint16_t ShutDownLowerLimit;
    uint16_t ShutDownUpperLimit;

} REFORMER_TEMP_CMP_LINES_Typedef;

typedef struct {
    uint16_t    PumpSpdIgniterFirstTime;
    uint16_t    PumpSpdIgniterSecondTime;
    uint16_t    PumpSpdAfterLiquidPressExceed4Kg;
    
} HYDROGEN_PUMP_SPEED_PARA_Typedef;


typedef struct {
    uint16_t FanSpdIgniterFirstTime;
    uint16_t FanSpdAfterIgniterFirstSuccessd;
    uint16_t FanSpdIgniterSecondTime;
    uint16_t FanSpdAfterIgniterSecondSuccessd;

} HYDROGEN_FAN_SPEED_PARA_Typedef;

typedef struct {
    uint16_t AlarmLowerLimit;      //警报下限1Kpa
    uint16_t AlarmUpperLimit;      //警报上限18Kpa
    uint16_t ShutDownLowerLimit;   //下限0KPa
    uint16_t ShutDownUpperLimit;   //过高20KPa

} LIQUID_PRESSURE_CMP_LINES_Typedef;


typedef struct {
    uint16_t AlarmlowerLiquidLevellimit;//液位低下限
    uint16_t OpenAutomaticliquidValue; //开自动加液液位值
    uint16_t CloseAutomaticliquidValue; //关自动加液液位值
    uint16_t AlarmUpperLiquidLevellimit;//液位高上限

} LIQUID_HEIGHT_CMP_LINES_Typedef;

/*
***************************************************************************************************
*                              EXPORTED GLOBAL VARIABLE DECLARATIONS
***************************************************************************************************
*/
extern HYDROGEN_PUMP_SPEED_PARA_Typedef            g_stStartHydrgPumpSpdPara;
extern HYDROGEN_FAN_SPEED_PARA_Typedef             g_stStartHydrgFanSpdPara;
extern REFORMER_TEMP_CMP_LINES_Typedef             g_stReformerTempCmpTbl;
extern LIQUID_PRESSURE_CMP_LINES_Typedef           g_stLqdPressCmpTbl;
extern LIQUID_HEIGHT_CMP_LINES_Typedef             g_stLqdHeightCmpTbl;

extern uint16_t                                    g_u16RunPurifyAmpIntegralValue;
/*
***************************************************************************************************
*                                           EXPORTED MACRO
***************************************************************************************************
*/
#define LOCAL_NETWORK_ID        0       //本地组网ID，独立运行的机器默认为0
#define PRODUCTS_TYPE_ID        0x1100  //产品型号ID：小型发电机
/*
***************************************************************************************************
*                                           EXPORTED FUNCTION
***************************************************************************************************
*/
//原有的
void        LoadParameters(void);
void        SaveSystemWorkTimes(void);

void        LoadDriverLayerParameters(void);
void        LoadApplicationLayerParameters(void);

void        StoreSystemWorkTimes(uint16_t);

void        GetStartHydrgPumpSpdParaFromFlash(HYDROGEN_PUMP_SPEED_PARA_Typedef *);
void        StoreStartHydrgPumpSpdPara(HYDROGEN_PUMP_SPEED_PARA_Typedef *);

void        GetStartHydrgFanSpdParaFromFlash(HYDROGEN_FAN_SPEED_PARA_Typedef *);
void        StoreStartHydrgFanSpdPara(HYDROGEN_FAN_SPEED_PARA_Typedef *);

void        SetRunningParaCfgSwitch(uint8_t);
uint8_t     GetRunningParaCfgSwitchStatus(void);



/*
***************************************************************************************************
*                                             MODULE END
***************************************************************************************************
*/


#endif                                                          /* End of module include */

