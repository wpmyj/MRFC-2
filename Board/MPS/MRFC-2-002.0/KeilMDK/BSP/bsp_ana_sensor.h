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

/********************************************************************************
  * @file    bsp_ans_senor.h
  * @author  JasonFan
  * @version V1.0
  * @date    12-December-2016
  * @brief   This file contains all the functions prototypes for the analog sensor
  *          firmware library.
*********************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef  BSP_ANA_SENSOR_PRESENT
#define  BSP_ANA_SENSOR_PRESENT
/*
***************************************************************************************************
*                                           INCLUDE FILES
***************************************************************************************************
*/
#include "stm32f10x.h"
/*
***************************************************************************************************
*                                           MACRO DEFINITIONS
***************************************************************************************************
*/

#ifdef   BSP_ANA_SENSOR_MODULE
    #define  BSP_ANA_SENSOR_EXT
#else
    #define  BSP_ANA_SENSOR  extern
#endif

#define BSP_ANA_SENSORS_NMB                      14u     //模拟信号采样通道数-全开为14
/*
***************************************************************************************************
*                                           EXPORTED DATA TYPE
***************************************************************************************************
*/
typedef struct {
    float    BaseDigValue;      //原始数字量
    float    AnaToDigRatio; //模数转换比率
} ANALOG_SIGNAL_SERSOR_PARAMETERS_Typedef;
/*
***************************************************************************************************
*                                               DEFINES
***************************************************************************************************
*/
typedef enum {

    STACK_TEMP = 0,//对应ADC规则组通道
    STACK_VOLTAGE,
    STACK_CURRENT,

    LIQUID_PRESS,
    HYDROGEN_PRESS_1,
    HYDROGEN_PRESS_2,

    LIQUID_LEVEL1 = 6,//小水箱液位
    BATTERY_VOLTAGE,
    LIQUID_LEVEL2,//LIQUID_LEVEL2
    BATTERY_CURRENT,
    RAPID_HEATER_CURRETN,
    NEGATIVE_PRESSURE,
    RESERVED_1 = 12,//预留
    RESERVED_2,

} ANALOG_SIGNAL_KIND_Typedef;





/*
***************************************************************************************************
*                                          GLOBAL VARIABLES
***************************************************************************************************
*/


/*
***************************************************************************************************
*                                               MACRO'S
***************************************************************************************************
*/


/*
***************************************************************************************************
*                                              DATA TYPES
***************************************************************************************************
*/

void    AnaSensorSelfCheck(void);

void    BSP_StoreAnaSigParameters(void);
void    AnaSigSampleStart(void);
void    UpdateAnaSigDigValue(void);
float   GetSrcAnaSig(ANALOG_SIGNAL_KIND_Typedef);

void    BSP_LoadAnaSensorParaToPrgm(ANALOG_SIGNAL_SERSOR_PARAMETERS_Typedef *i_ParaGrps, uint8_t i_u8GrpLenth);
void    BSP_SaveAnaSensorParameters(float *i_NewParameters);

ErrorStatus BSP_GetCalibratedAnaSensorPara(ANALOG_SIGNAL_SERSOR_PARAMETERS_Typedef *o_stAnaSensorPara, uint8_t i_u8Length);

/*
***************************************************************************************************
*                                            FUNCTION PROTOTYPES
***************************************************************************************************
*/


/*
***************************************************************************************************
*                                              MODULE END
***************************************************************************************************
*/


#endif
