/*
***************************************************************************************************
*                                              APPLICATION CODE
*
*                          (c) Copyright 2016; Guangdong ENECO Energy Science And Technology Co.,Ltd
*
*               All rights reserved.  Protected by international copyright laws.
*               Knowledge of the source code may NOT be used without authorization.
***************************************************************************************************
*/
/*
***************************************************************************************************
* Filename      : bsp_pid.c
* Version       : V1.00
* Programmer(s) : JasonFan
*
***************************************************************************************************
*/
/*
***************************************************************************************************
*                                             INCLUDE FILES
***************************************************************************************************
*/
#include "includes.h"
#include "bsp_pid.h"
#include "math.h"
#include "bsp_speed_adjust_device.h"
/*
***************************************************************************************************
*                                           MACRO DEFINITIONS
***************************************************************************************************
*/

#define PID_TEST_INCREMENTAL    1
/*
***************************************************************************************************
*                                           LOCAL VARIABLES
***************************************************************************************************
*/


/*
***************************************************************************************************
*                                           GLOBAL VARIABLES
***************************************************************************************************
*/
INC_TYPE_PID_PARA_Typedef IPID;

/*
***************************************************************************************************
*                             IncrementType_PID_Init()
*
* Description : Initialize the PID struct.
*
* Arguments   : none.
*
* Returns     : none.
*
* Notes       : none.
***************************************************************************************************
*/
void IncrementType_PID_Init(void)
{
    IPID.Sv = 0;
    IPID.Pv = 0;

    IPID.Err = 0;
    IPID.Err_Next = 0;
    IPID.Err_Last = 0;

    IPID.OutValueMax = 2000;
    IPID.OutValueMin = 200;//MRFC-2板上已有3.2V电压输出了，实际为30%转速了
    IPID.OutValue = 200;

    IPID.CalcCycleCnt = 0;//计算/控制周期

    IPID.Kp = 35;//13.5
    IPID.Ki = 16.2;
    IPID.Kd = 12.5;
    IPID.Tsam = 6.0;//10

}


void ResetPidErr(INC_TYPE_PID_PARA_Typedef * i_PidStruct)
{
	i_PidStruct->Err = 0;
	i_PidStruct->Err_Last = 0;
	i_PidStruct->Err_Next = 0;
}

/*
***************************************************************************************************
*                                          PID_Process()
*
* Description : 增量式数字PID算法.
*
* Arguments   : i_OptimumTemperature:the best temperature in that current.
*
* Returns     : 实际输出控制量.
*
* Notes       : 结果只与三次采样值有关，减小误差,大大减小了计算量.
*
***************************************************************************************************
*/
uint16_t IncrementType_PID_Process(uint8_t i_OptimumTemperature)
{
    float  IncrementSpeed = 0.0;
    float  fPout = 0.0 , fIout = 0.0 , fDout = 0.0;

    IPID.Sv = i_OptimumTemperature;
    IPID.Pv = GetSrcAnaSig(STACK_TEMP);

    //Err > 0，控制量要增大;Err < 0,控制量要减小
    IPID.Err = IPID.Pv - IPID.Sv;

    fPout = IPID.Kp * (IPID.Err - IPID.Err_Last);
    fIout = IPID.Ki * IPID.Err;

    if(IPID.Err > 0) {
        fDout = IPID.Kd * (IPID.Err - 2 * IPID.Err_Last + IPID.Err_Next);
    } else {    //当前值比设置值低,采用PI调节
        fDout = 0;
    }

    IncrementSpeed = fPout + fIout + fDout; //本次得到的增量

    if((IPID.OutValue <= IPID.OutValueMin) && (fIout < 0)) { //防止累计误差积累导致减溢出
        IncrementSpeed = 0.0;
    }

    IPID.OutValue += (int16_t)IncrementSpeed;//本次应该输出的实际控制量

    if(IPID.OutValue <= IPID.OutValueMin) {
        IPID.OutValue = IPID.OutValueMin;
    }

    if(IPID.OutValue >= IPID.OutValueMax) {
        IPID.OutValue = IPID.OutValueMax;
    }

    IPID.Err_Last = IPID.Err_Next;    //更新偏差
    IPID.Err_Next = IPID.Err;

    return IPID.OutValue;
}


/******************* (C) COPYRIGHT 2016 Guangdong ENECO POWER *****END OF FILE****/
