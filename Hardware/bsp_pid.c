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
    IPID.OutValueMin = 200;//MRFC-2��������3.2V��ѹ����ˣ�ʵ��Ϊ30%ת����
    IPID.OutValue = 200;

    IPID.CalcCycleCnt = 0;//����/��������

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
* Description : ����ʽ����PID�㷨.
*
* Arguments   : i_OptimumTemperature:the best temperature in that current.
*
* Returns     : ʵ�����������.
*
* Notes       : ���ֻ�����β���ֵ�йأ���С���,����С�˼�����.
*
***************************************************************************************************
*/
uint16_t IncrementType_PID_Process(uint8_t i_OptimumTemperature)
{
    float  IncrementSpeed = 0.0;
    float  fPout = 0.0 , fIout = 0.0 , fDout = 0.0;

    IPID.Sv = i_OptimumTemperature;
    IPID.Pv = GetSrcAnaSig(STACK_TEMP);

    //Err > 0��������Ҫ����;Err < 0,������Ҫ��С
    IPID.Err = IPID.Pv - IPID.Sv;

    fPout = IPID.Kp * (IPID.Err - IPID.Err_Last);
    fIout = IPID.Ki * IPID.Err;

    if(IPID.Err > 0) {
        fDout = IPID.Kd * (IPID.Err - 2 * IPID.Err_Last + IPID.Err_Next);
    } else {    //��ǰֵ������ֵ��,����PI����
        fDout = 0;
    }

    IncrementSpeed = fPout + fIout + fDout; //���εõ�������

    if((IPID.OutValue <= IPID.OutValueMin) && (fIout < 0)) { //��ֹ�ۼ������۵��¼����
        IncrementSpeed = 0.0;
    }

    IPID.OutValue += (int16_t)IncrementSpeed;//����Ӧ�������ʵ�ʿ�����

    if(IPID.OutValue <= IPID.OutValueMin) {
        IPID.OutValue = IPID.OutValueMin;
    }

    if(IPID.OutValue >= IPID.OutValueMax) {
        IPID.OutValue = IPID.OutValueMax;
    }

    IPID.Err_Last = IPID.Err_Next;    //����ƫ��
    IPID.Err_Next = IPID.Err;

    return IPID.OutValue;
}


/******************* (C) COPYRIGHT 2016 Guangdong ENECO POWER *****END OF FILE****/