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
* Filename      : bsp_speed_adjust_device.c
* Version       : V1.00
* Programmer(s) : SunKing.Yun
*********************************************************************************************************
*/
/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/
#define  BSP_ANA_SENSOR_MODULE
#include <bsp.h>
#include "os_cfg_app.h"
#include <bsp_ana_sensor.h>
#include "app_system_real_time_parameters.h"
#include "bsp_speed_adjust_device.h"
/*
*********************************************************************************************************
*                                       LOCAL GLOBAL VARIABLES
*********************************************************************************************************
*/
u16         g_u16HydrgPumpCtlSpd = 0;
u16         g_u16HydrgFanCtlSpd = 0;
u16         g_u16StackFanCtlSpd = 0;

/*
*********************************************************************************************************
*                                         PumpSpdInc()
*
* Description : increase the pump speed a grade.
*
* Arguments   : none.
*
* Returns     : none
*
* Notes       : the speed grade whole number is 200.
*********************************************************************************************************
*/
void PumpSpdInc()
{
    uint16_t u16PumpCtlSpd;
    u16PumpCtlSpd = GetPumpCtlSpd();

    if(u16PumpCtlSpd >= 199)
    {
        u16PumpCtlSpd = 200;
    }
    else
    {
        u16PumpCtlSpd += 1;
    }

    SetPumpCtlSpd(u16PumpCtlSpd);
}

/*
*********************************************************************************************************
*                                         PumpSpdDec()
*
* Description : decrease the pump speed a grade.
*                               降低泵速
* Arguments   : none.
*
* Returns     : none
*
* Notes       : the speed grade whole number is 200.
*********************************************************************************************************
*/
void PumpSpdDec()
{
    uint16_t u16PumpCtlSpd;
    u16PumpCtlSpd = GetPumpCtlSpd();

    if(u16PumpCtlSpd < 1)
    {
        u16PumpCtlSpd = 0;
    }
    else
    {
        u16PumpCtlSpd -= 1;
    }

    SetPumpCtlSpd(u16PumpCtlSpd);
}

/*
*********************************************************************************************************
*                                         GetPumpCtlSpd()
*
* Description : get the pump speed grade number.
*
* Arguments   : none.
*
* Returns     : the pump speed grade number.
*
* Notes       : the speed grade whole number is 200.
*********************************************************************************************************
*/
uint16_t GetPumpCtlSpd(void)
{
    return g_u16HydrgPumpCtlSpd;
}



/*
*********************************************************************************************************
*                                         SetPumpCtlSpd()
*
* Description : set the pump speed grade.
*
* Arguments   : the expected pump speed.
*
* Returns     : none.
*
* Notes       : the speed grade whole number is 200.
*********************************************************************************************************
*/
void SetPumpCtlSpd(uint16_t i_u16NewSpd)
{
    CPU_SR_ALLOC();
    CPU_CRITICAL_ENTER();
    g_u16HydrgPumpCtlSpd = i_u16NewSpd;
    BSP_SetPumpSpd(i_u16NewSpd);
    CPU_CRITICAL_EXIT();
}

/*
*********************************************************************************************************
*                                         HydrgFanSpdInc()
*
* Description : increase the hydrogen fan speed a grade.
*
* Arguments   : none.
*
* Returns     : none
*
* Notes       : the speed grade whole number is 200.
*********************************************************************************************************
*/
void HydrgFanSpdInc()
{
    uint16_t u16HydrgFanCtlSpd;
    u16HydrgFanCtlSpd = GetHydrgFanCtlSpd();

    if(u16HydrgFanCtlSpd >= 190)
    {
        u16HydrgFanCtlSpd = 200;
    }
    else
    {
        if(u16HydrgFanCtlSpd >= 80)
        {
            u16HydrgFanCtlSpd += 10;
        }
        else
        {
            u16HydrgFanCtlSpd += 20;
        }
    }

    SetHydrgFanCtlSpd(u16HydrgFanCtlSpd);
}

/*
*********************************************************************************************************
*                                         HydrgFanSpdDec()
*
* Description : decrease the hydrogen fan speed a grade.
*
* Arguments   : none.
*
* Returns     : none
*
* Notes       : the speed grade whole number is 200.
*********************************************************************************************************
*/
void HydrgFanSpdDec()
{
    uint16_t u16HydrgFanCtlSpd;
    u16HydrgFanCtlSpd = GetHydrgFanCtlSpd();

    if(u16HydrgFanCtlSpd < 10)
    {
        u16HydrgFanCtlSpd = 0;
    }
    else
    {
        if(u16HydrgFanCtlSpd > 80)
        {
            u16HydrgFanCtlSpd -= 10;
        }
        else
        {
            u16HydrgFanCtlSpd -= 20;
        }
    }

    SetHydrgFanCtlSpd(u16HydrgFanCtlSpd);
}

/*
*********************************************************************************************************
*                                         GetHydrgFanCtlSpd()
*
* Description : get the hydrogen fan speed grade number.
*
* Arguments   : none.
*
* Returns     : the hydrogen fan speed grade number.
*
* Notes       : the speed grade whole number is 200.
*********************************************************************************************************
*/
uint16_t GetHydrgFanCtlSpd(void)
{
    return g_u16HydrgFanCtlSpd;
}



/*
*********************************************************************************************************
*                                         SetHydrgFanCtlSpd()
*
* Description : set the hydrogen fan speed grade.
*               设置制氢风机速度
* Arguments   : the expected hydrogen fan speed.
*
* Returns     : none.
*
* Notes       : the speed grade whole number is 200.
*********************************************************************************************************
*/
void SetHydrgFanCtlSpd(uint16_t i_u16NewSpd)
{
    CPU_SR_ALLOC();
    CPU_CRITICAL_ENTER();
    g_u16HydrgFanCtlSpd = i_u16NewSpd;
    BSP_SetHydrgFanSpd(i_u16NewSpd);
    CPU_CRITICAL_EXIT();
}

/*
*********************************************************************************************************
*                                         StackFanSpdInc()
*
* Description : increase the stack fan1 and fan2 speed a grade.
*
* Arguments   : none.
*
* Returns     : none
*
* Notes       : the speed grade whole number is 200.
*********************************************************************************************************
*/
void StackFanSpdInc()
{
    uint16_t u16StackFanCtlSpd;
    u16StackFanCtlSpd = GetStackFanCtlSpd();

    if(u16StackFanCtlSpd >= 190)
    {
        u16StackFanCtlSpd = 200;
    }
    else
    {
        u16StackFanCtlSpd += 10;
    }

    SetStackFanCtlSpd(200-u16StackFanCtlSpd);
}

/*
*********************************************************************************************************
*                                         StackFanSpdDec()
*
* Description : decrease the stack fan speed a grade.
*
* Arguments   : none.
*
* Returns     : none
*
* Notes       : the speed grade whole number is 200.
*********************************************************************************************************
*/
void StackFanSpdDec()
{
    uint16_t u16StackFanCtlSpd;
    u16StackFanCtlSpd = GetStackFanCtlSpd();

    if(u16StackFanCtlSpd < 10)
    {
        u16StackFanCtlSpd = 0;      //确保泵速输出不会反转至200
    }
    else
    {
        u16StackFanCtlSpd -= 10;
    }

    SetStackFanCtlSpd(200-u16StackFanCtlSpd);
}


/*
*********************************************************************************************************
*                                         GetStackFanCtlSpd()
*
* Description : get the stack fan speed grade number.
*
* Arguments   : none.
*
* Returns     : the stack fan speed grade number.
*
* Notes       : the speed grade whole number is 200.
*********************************************************************************************************
*/
uint16_t GetStackFanCtlSpd(void)
{
    return g_u16StackFanCtlSpd;
}



/*
*********************************************************************************************************
*                                         SetStackFanCtlSpd()
*
* Description : set the stack fan speed grade.
*
* Arguments   : the expected stack fan speed.
*
* Returns     : none.
*
* Notes       : the speed grade whole number is 200.
*********************************************************************************************************
*/
void SetStackFanCtlSpd(uint16_t i_u16NewSpd)
{
    CPU_SR_ALLOC();
    CPU_CRITICAL_ENTER();
    g_u16StackFanCtlSpd = i_u16NewSpd;
    Set_PWM(0, g_u16StackFanCtlSpd);
//    BSP_SetStackFanSpd(i_u16NewSpd);
    CPU_CRITICAL_EXIT();

}
