/*
***************************************************************************************************
*                                      APPLICATION CONFIGURATION
*
*                                     ST Microelectronics STM32
*                                              on the
*
*                                     Micrium uC-Eval-STM32F107
*                                         Evaluation Board
*
* Filename      : app_cfg.h
* Version       : V1.00
* Programmer(s) : JJL
*                 EHS
*                 DC
***************************************************************************************************
*/

#ifndef  __APP_CFG_H__
#define  __APP_CFG_H__


/*
***************************************************************************************************
*                                       MODULE ENABLE / DISABLE
***************************************************************************************************
*/

#define  APP_CFG_SERIAL_EN                          DEF_ENABLED


/*
***************************************************************************************************
*                                    BSP CONFIGURATION: RS-232
***************************************************************************************************
*/

#define  BSP_CFG_SER_COMM_SEL                       BSP_SER_COMM_UART_01
#define  BSP_CFG_TS_TMR_SEL                         1


/*
***************************************************************************************************
*                                     TRACE / DEBUG CONFIGURATION
***************************************************************************************************
*/
#if 0
    #define  TRACE_LEVEL_DEF_OFF                            0
    #define  TRACE_LEVEL_INFO                           1
    #define  TRACE_LEVEL_DEBUG                          2
#endif

#define  APP_TRACE_LEVEL                            TRACE_LEVEL_INFO
#define  APP_TRACE                                  BSP_Ser_Printf

#define  APP_TRACE_INFO(x)            ((APP_TRACE_LEVEL >= TRACE_LEVEL_INFO)  ? (void)(APP_TRACE x) : (void)0)
#define  APP_TRACE_DEBUG(x)           ((APP_TRACE_LEVEL >= TRACE_LEVEL_DBG) ? (void)(APP_TRACE x) : (void)0)


#endif
