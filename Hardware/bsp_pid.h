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
  * @file    bsp_pid.h
  * @author  JasonFan
  * @version V1.0
  * @date    3-May-2016
  * @brief   This file contains all the functions prototypes for the pid control
  *          firmware library.
*********************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BSP_PID_H__
#define __BSP_PID_H__
/*
***************************************************************************************************
*                                           INCLUDE FILES
***************************************************************************************************
*/
/*
***************************************************************************************************
*                                           MACRO DEFINITIONS
***************************************************************************************************
*/
/*
***************************************************************************************************
*                                           EXPORTED TYPE
***************************************************************************************************
*/

typedef struct {
    uint8_t Sv;   //设定温度
    uint8_t Pv;  //当前温度

    int8_t Err;//本次偏差
    int8_t Err_Next;//上次偏差
    int8_t Err_Last;//上上次偏差


    float Kp, Ki, Kd;//定义比例propotion、积分Integral、微分Differential系数

    uint8_t Tsam; //采样周期---控制周期，每隔Tsam控制器输出一次PID运算结果

    uint16_t OutValueMax;                   //控制量最大值
    uint16_t OutValueMin;                   //控制量最小值
    uint16_t OutValue;

    uint16_t CalcCycleCnt; //PID计算周期

} INC_TYPE_PID_PARA_Typedef;



extern INC_TYPE_PID_PARA_Typedef IPID;

/*
***************************************************************************************************
*                                           EXPORTED CONSTANTS
***************************************************************************************************
*/

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

void IncrementType_PID_Init(void);

void ResetPidErr(INC_TYPE_PID_PARA_Typedef * i_PidStruct);
uint16_t IncrementType_PID_Process(uint8_t i_OptimumTemperature);

#endif

