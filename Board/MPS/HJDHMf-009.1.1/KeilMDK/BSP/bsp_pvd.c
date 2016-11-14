/*
*********************************************************************************************************
*                                              APPLICATION CODE
*
*                          (c) Copyright 2015; Guangdong Hydrogen Energy Science And Technology Co.,Ltd
*
*               All rights reserved.  Protected by international copyright laws.
*               Knowledge of the source code may NOT be used without authorization.
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                           INCLUDE FILES
*********************************************************************************************************
*/
#include "includes.h"
#include "bsp_pvd.h"



/*
*********************************************************************************************************
*                                      Bsp_PVD_Configuration()
*
* Description:  initialize the brownout protect
*
* Arguments  :  none.
*
* Returns    :  none.
*
* Note(s)    :  none.
*********************************************************************************************************
*/
void Bsp_PVD_Configuration(void)
{
    EXTI_InitTypeDef EXTI_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);

    EXTI_ClearITPendingBit(EXTI_Line16);
    EXTI_InitStructure.EXTI_Line = EXTI_Line16;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    Bsp_PVD_ProtectStatuCmd(ON);            //开掉电保护

    PWR_PVDLevelConfig(PWR_PVDLevel_2V9);   //PVD探测电压阀值2.9V
    PWR_PVDCmd(ENABLE);                     //使能可编程的电压探测器
}


/*
*********************************************************************************************************
*                                      Bsp_PVD_ProtectStatuCmd()
*
* Description:  set the new status of brownout protect
*
* Arguments  :  none.
*
* Returns    :  none.
*
* Note(s)    :  none.
*********************************************************************************************************
*/
void Bsp_PVD_ProtectStatuCmd(SWITCH_TYPE_VARIABLE_Typedef i_NewStatu)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_InitStructure.NVIC_IRQChannel = PVD_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;

    if(i_NewStatu == ON)
    {
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    }
    else
    {
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//DISABLE;
    }

    NVIC_Init(&NVIC_InitStructure);
}


