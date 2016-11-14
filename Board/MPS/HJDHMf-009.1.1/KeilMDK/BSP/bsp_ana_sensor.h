/*
*********************************************************************************************************
*                                     MICRIUM BOARD SUPPORT SUPPORT
*
*                          (c) Copyright 2003-2009; Micrium, Inc.; Weston, FL
*
*               All rights reserved.  Protected by international copyright laws.
*               Knowledge of the source code may NOT be used to develop a similar product.
*               Please help us continue to provide the Embedded community with the finest
*               software available.  Your honesty is greatly appreciated
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*
*                                     MICIUM BOARD SUPPORT PACKAGE
*                                 STLM75 CMOS TEMPERATURE SENSOR DRIVER
*
*
* Filename      : bsp_stlm75.h
* Version       : V1.00
* Programmer(s) : FT
*********************************************************************************************************
* Note(s)       :
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                                 MODULE
*
* Note(s) : (1) This header file is protected from multiple pre-processor inclusion through use of the
*               BSP_STLM75 present pre-processor macro definition.
*********************************************************************************************************
*/

#ifndef  BSP_ANA_SENSOR_PRESENT
#define  BSP_ANA_SENSOR_PRESENT


/*
*********************************************************************************************************
*                                              EXTERNS
*********************************************************************************************************
*/

#ifdef   BSP_ANA_SENSOR_MODULE
    #define  BSP_ANA_SENSOR_EXT
#else
    #define  BSP_ANA_SENSOR  extern
#endif


/*
*********************************************************************************************************
*                                               DEFINES
*********************************************************************************************************
*/
typedef enum
{
    STACK_TEMP = 0,
    STACK_VOLTAGE,
    STACK_CURRENT,

    LIQUID_PRESS,
    HYDROGEN_PRESS_1,
    HYDROGEN_PRESS_2,
    //Ô¤Áô²ÎÊý
    LIQUID_LEVEL,
    RESERVED_2,
    RESERVED_3
} ANALOG_SIGNAL_KIND_Typedef;





/*
*********************************************************************************************************
*                                          GLOBAL VARIABLES
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                               MACRO'S
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                              DATA TYPES
*********************************************************************************************************
*/

void    AnaSensorSelfCheck(void);

void    BSP_LoadAnaSensorParameters(float *);
void    BSP_SaveAnaSensorParameters(float *);
void    BSP_LoadCalibratedAnaSensorParameters(void);

void    BSP_StoreAnaSigParameters(void);
void    AnaSigSampleStart(void);
void    UpdateAnaSigDigValue(void);
float   GetSrcAnaSig(ANALOG_SIGNAL_KIND_Typedef);
/*
*********************************************************************************************************
*                                            FUNCTION PROTOTYPES
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                              MODULE END
*********************************************************************************************************
*/


#endif
