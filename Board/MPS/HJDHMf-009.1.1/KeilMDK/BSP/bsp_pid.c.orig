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
* Programmer(s) : Fanjun
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
#define POSITIONTYPE_PID        0
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
INCREMENT_TYPE_PID_PARAMETER_Typedef IPID;

/*
***************************************************************************************************
*                             IncrementType2_PID_Init()
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

    IPID.OutValue = 500;
    IPID.OutValueMax = 1900;
    IPID.OutValueMin = 500;
    
    IPID.Pout = 0.0,
    IPID.Iout = 0.0,
    IPID.Dout = 0.0,
    
    IPID.CalcCycleCount = 0;//计算/控制周期

//    IPID.Td = 200.0;
//    IPID.Ti = 400.0;
    IPID.Kp = 13.5;
    IPID.Ki = 16.2;
    IPID.Kd = 12.5;
    IPID.Tsam = 10.0;

}


/*
***************************************************************************************************
*                                          PID_Process()
*
* Description : 增量式数字PID算法.
*
* Arguments   : i_OptimumTemperature:the best temperature in that current.
*
* Returns     : none.
*
* Notes       : 结果只与三次采样值有关，减小误差,大大减小了计算量.
*
***************************************************************************************************
*/
void IncrementType_PID_Process(uint8_t i_OptimumTemperature)  //增量式PID计算
{
    float IncrementSpeed = 0.0;//PID应该输出的增量值
//    int16_t dk1 = 0,dk2 = 0;
//    float Pout = 0.0 ,Iout = 0.0 ,Dout = 0.0; //调试完成后可采用局部变量

    if(IPID.CalcCycleCount < IPID.Tsam) {
        return ;    //控制周期未到
    }

    IPID.Sv = i_OptimumTemperature;
    IPID.Pv = GetSrcAnaSig(STACK_TEMP);//也可在模拟量检测任务中更新
    
    //本次误差(偏差为正，控制量要增大(风机速度);偏差为负,控制量要减小)
    IPID.Err = IPID.Pv - IPID.Sv; 
   
    IPID.Pout = IPID.Kp * (IPID.Err - IPID.Err_Last);//dk1
    IPID.Iout = IPID.Ki * IPID.Err;
    if(IPID.Err > 0){ //当前值比设置值高，要减控制量,采用PI调节 
        IPID.Dout = IPID.Kd * (IPID.Err - 2 * IPID.Err_Last + IPID.Err_Next);//dk2
    } else {
        IPID.Dout = 0;
    }
//    IncrementSpeed = IPID.Kp * (IPID.Err - IPID.Err_Last) + IPID.Ki * IPID.Err + IPID.Kd * (IPID.Err - 2 * IPID.Err_Last + IPID.Err_Next);
 
    IncrementSpeed = IPID.Pout + IPID.Iout + IPID.Dout; //本次得到的增量
    IPID.OutValue += (int16_t)IncrementSpeed;           //本次应该输出的实际控制量

    if(IPID.OutValue < IPID.OutValueMin) {
        IPID.OutValue = IPID.OutValueMin;
    }
    if(IPID.OutValue > IPID.OutValueMax) {
        IPID.OutValue = IPID.OutValueMax;
    } 

    SetStackFanCtlSpd(IPID.OutValue);//根据计算出的输出量给到控制对象
    
    IPID.Err_Last = IPID.Err_Next;    //更新偏差
    IPID.Err_Next = IPID.Err;

    IPID.CalcCycleCount = 0;
}


/******************* (C) COPYRIGHT 2016 Guangdong ENECO *****END OF FILE****/
