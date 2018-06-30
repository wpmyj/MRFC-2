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
* Programmer(s) :  JasonFan
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

} LIQUID_HEIGHT_CMP_LINES_Typedef;


typedef struct {
    uint16_t FirstDelayTimeByMin;
    uint8_t  FirstTimeAdjustPumpFlag;
    uint8_t  FirstTimeAdjustPumpValue;
    uint8_t  FirstTimeAdjustFanFlag;
    uint8_t  FirstTimeAdjustFanValue;
    uint16_t SecondDelayTimeByMin;
    uint8_t  SecondTimeAdjustPumpFlag;
    uint8_t  SecondTimeAdjustPumpValue;
    uint8_t  SecondTimeAdjustFanFlag;
    uint8_t  SecondTimeAdjustFanValue;

} RUNNING_STATUS_DELAY_ADJUST_SPEED_PARAMETERS_Typedef;

typedef struct {
    
    uint8_t  ActiveStep;
    uint16_t ActiveStep1PumpSpd;
    uint16_t ActiveStep1FanSpd;
    uint8_t  ActiveStep1HoldHour;
    uint8_t  ActiveStep1HoldMin;
    
    uint16_t ActiveStep2PumpSpd;
    uint16_t ActiveStep2FanSpd;
    uint8_t  ActiveStep2HoldHour;
    uint8_t  ActiveStep2HoldMin;
    
    uint16_t ActiveStep3PumpSpd;
    uint16_t ActiveStep3FanSpd;
    uint8_t  ActiveStep3HoldHour;
    uint8_t  ActiveStep3HoldMin;
    
    uint16_t ActiveStep4PumpSpd;
    uint16_t ActiveStep4FanSpd;
    uint8_t  ActiveStep4HoldHour;
    uint8_t  ActiveStep4HoldMin;
    
    uint8_t  CurrentActiveStepHoldHour;
    uint8_t  CurrentActiveStepHoldMin;

} RICH_HYDROGEN_ACTIVE_PARA_Typedef;

typedef struct {
	
	uint16_t current_max;
    uint16_t voltage_max;
	
} MAX_VI_VALUE_PARA_Typedef;

typedef struct {
	
	uint8_t byte1;
	uint8_t byte2;
	uint8_t byte3;
	uint8_t byte4;
	uint8_t byte5;
	uint8_t byte6;
}MACHINE_ID_PARA_Typedef;


typedef enum {
    
    ACTIVE_STEP_ONE = 1,
    ACTIVE_STEP_TWO,
    ACTIVE_STEP_THREE,
    ACTIVE_STEP_FOUR,
    
}RICH_HYDROGEN_ACTIVE_STEP_Typedef;

/*
***************************************************************************************************
*                              EXPORTED GLOBAL VARIABLE DECLARATIONS
***************************************************************************************************
*/
extern HYDROGEN_PUMP_SPEED_PARA_Typedef                         g_stStartHydrgPumpSpdPara;
extern HYDROGEN_FAN_SPEED_PARA_Typedef                          g_stStartHydrogenFanSpdPara;
extern REFORMER_TEMP_CMP_LINES_Typedef                          g_stReformerTempCmpTbl;
extern LIQUID_PRESSURE_CMP_LINES_Typedef                        g_stLqdPressCmpTbl;
extern LIQUID_HEIGHT_CMP_LINES_Typedef                          g_stLqdHeightCmpTbl;
extern RUNNING_STATUS_DELAY_ADJUST_SPEED_PARAMETERS_Typedef     g_stRunningStatusDelayAdjustSpdPara;
extern uint16_t                                                 g_u16RunPurifyAmpIntegralValue;
extern RICH_HYDROGEN_ACTIVE_PARA_Typedef                        g_stRichHydrogenModePara;
extern MAX_VI_VALUE_PARA_Typedef								g_stMaxVILimitPara;
extern MACHINE_ID_PARA_Typedef									g_stMachineIdPara;
extern uint16_t                                                 g_u16GlobalNetWorkId;
extern uint16_t                                                 g_u16FirstTimeHeatHoldSeconds;
extern uint16_t                                   				g_ProductsType;
/*
***************************************************************************************************
*                                           EXPORTED MACRO
***************************************************************************************************
*/

/*
***************************************************************************************************
*                                           EXPORTED FUNCTION
***************************************************************************************************
*/
void        LoadParameters(void);
void        SaveSystemWorkTimes(void);

void        LoadDriverLayerParameters(void);
void        LoadApplicationLayerParameters(void);

void        StoreSystemWorkTimes(uint16_t);

void        GetStartHydrgPumpSpdParaFromFlash(HYDROGEN_PUMP_SPEED_PARA_Typedef *);
void        StoreStartHydrgPumpSpdPara(HYDROGEN_PUMP_SPEED_PARA_Typedef *);
void        StoreStartHydrgPumpSpdParaBySingle(uint16_t *i_StartPumpSpdPara, u8 DataAddr);

void        GetStartHydrgFanSpdParaFromFlash(HYDROGEN_FAN_SPEED_PARA_Typedef *);
void        StoreStartHydrgFanSpdPara(HYDROGEN_FAN_SPEED_PARA_Typedef *);
void        StoreStartHydrgFanSpdParaBySingle(uint16_t *i_StartHydrgFanSpdPara, u8 DataAddr);

void        StoreGlobalNetWorkID(u16 *i_GlobalNetWorkID);
void        StoreFirstTimeHeatHoldSeconds(uint16_t *i_FirstTimeHeatHoldSeconds);
void        StoreRunningStatusDelayAdjustSpdParaBySingle(uint16_t *i_DelayAndAdjustSpdPara, uint8_t DataAddr);

void        StoreRichHydrogenModeFanPara(uint8_t * RecBuf,uint8_t i_eActiveStep);
void        StoreRichHydrogenModePumpPara(uint8_t * RecBuf,uint8_t i_eActiveStep);
void        StoreRichHydrogenModeHoldTimePara(uint8_t * RecBuf,uint8_t i_eActiveStep);
void        LoadCurrentStepRemainTimePara(uint8_t i_CurrentActiveStep);
void        LoadDefaultRichHydrogenModeHoldTimePara(void);


void 		StoreLimitVIMaxValue(MAX_VI_VALUE_PARA_Typedef *i_LimitVIMaxValue);
void 		GetLimitVIMaxValueFromFlash(MAX_VI_VALUE_PARA_Typedef *i_LimitVIMaxValue);

void 		GetRemoteMachineIdFromFlash(MACHINE_ID_PARA_Typedef *i_MachineIdPara);
void 		StoreRemoteMachineIdPara(MACHINE_ID_PARA_Typedef *i_MachineIdPara);
void 		SetRemoteMachineIdPara(uint8_t * RecBuf);
	
void        StoreRichHydrogenModePara(RICH_HYDROGEN_ACTIVE_PARA_Typedef *i_RichModePara);
typedef     void    (*StoreParaBySingleType_t)(uint16_t *,uint8_t);//存储参数类函数指针类型
/*
***************************************************************************************************
*                                             MODULE END
***************************************************************************************************
*/


#endif                                                          /* End of module include */

