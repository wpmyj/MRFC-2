/*
***************************************************************************************************
*                                              EXAMPLE CODE
*
*                          (c) Copyright 2003-2013; Micrium, Inc.; Weston, FL
*
*               All rights reserved.  Protected by international copyright laws.
*               Knowledge of the source code may NOT be used to develop a similar product.
*               Please help us continue to provide the Embedded community with the finest
*               software available.  Your honesty is greatly appreciated.
***************************************************************************************************
*/

/*
***************************************************************************************************
*
*                                           MASTER INCLUDES
*
*                                     ST Microelectronics STM32
*                                              on the
*
*                                     Micrium uC-Eval-STM32F107
*                                         Evaluation Board
*
* Filename      : includes.h
* Version       : V1.00
* Programmer(s) : EHS
*                 DC
***************************************************************************************************
*/

#ifndef  INCLUDES_PRESENT
#define  INCLUDES_PRESENT


/*
***************************************************************************************************
*                                         STANDARD LIBRARIES
***************************************************************************************************
*/

#include  <stdarg.h>
#include  <stdio.h>
#include  <stdlib.h>
#include  <stdint.h>
#include  <math.h>


/*
***************************************************************************************************
*                                              LIBRARIES
***************************************************************************************************
*/

#include  "cpu.h"
#include  <lib_def.h>
#include  <lib_ascii.h>
#include  <lib_math.h>
#include  <lib_mem.h>
#include  <lib_str.h>


/*
***************************************************************************************************
*                                              APP / BSP
***************************************************************************************************
*/

#include "task_prio_defines.h"
#include  "app.h"
#include  "bsp.h"
#include  "app_system_real_time_parameters.h"
#include  "bsp_ser.h"


/*
***************************************************************************************************
*                                                 OS
***************************************************************************************************
*/

#include  <os.h>


/*
***************************************************************************************************
*                                                 ST
***************************************************************************************************
*/

#include  <stm32f10x_conf.h>
#include  "stm32f10x.h"

void BSP_BuzzerInit(void);
/*
***************************************************************************************************
*                                            INCLUDES END
***************************************************************************************************
*/


#endif

