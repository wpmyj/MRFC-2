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
/*
***************************************************************************************************
* Filename      : bsp_speed_adjust_device.c
* Version       : V1.00
* Programmer(s) : Fanjun
***************************************************************************************************
*/
/*
***************************************************************************************************
*                                             INCLUDE FILES
***************************************************************************************************
*/
#define  BSP_ANA_SENSOR_MODULE
#include <bsp.h>
#include "math.h"
#include <includes.h>
#include "os_cfg_app.h"
#include <bsp_ana_sensor.h>
#include "app_system_real_time_parameters.h"
#include "app_analog_signal_monitor_task.h"
#include "bsp_speed_adjust_device.h"

/*
***************************************************************************************************
*                                       MICRO DEFINE
***************************************************************************************************
*/
#define SPEED_SAMPLE_CHANNEL          4u//速度反馈信号采样通道数

#define FRONT_EDGE      0 //捕获脉冲前沿
#define LAST_EDGE       1 //捕获脉冲后沿
/*
***************************************************************************************************
*                                         OS-RELATED    VARIABLES
***************************************************************************************************
*/
static      OS_TMR      HydrgFanSpdDlyAdjTmr;//延时调节定时器
static      OS_TMR      HydrgFanSpdCycleDlyTimeAdjustTmr;//周期延时调节定时器
/*
***************************************************************************************************
*                                       LOCAL GLOBAL VARIABLES
***************************************************************************************************
*/
static  uint16_t    g_u16HydrgFanExpectCtlSpd = 0;
static  uint16_t    g_u16HydrgFanCurrentCtlSpd = 0;
static  uint16_t    g_u16HydrgPumpCtlSpd = 0;
static  uint16_t    g_u16StackFanCtlSpd = 0;

static  SWITCH_TYPE_VARIABLE_Typedef g_eSpdCaptureWorkSwitch[3] = {OFF, OFF, OFF}; //3路通道监测开关
static  float g_fSpdCaptureFrequency[SPEED_SAMPLE_CHANNEL] = {0.0, 0.0, 0.0, 0.0}; //对应检测通道的脉冲频率数

static  uint16_t g_u16SpdCaptureEdgeNum[3] = {0, 0, 0}; //三个通道进入中断次数
static  uint16_t g_u16SpeedCaptureValue[3][2] = {0}; //三个通道周期内第一次进入中断和最后一次进入中断的计数值

/*
***************************************************************************************************
*                                         FUNCTION PROTOTYPES
***************************************************************************************************
*/
static void HydrgFanSpdDlyAdjCallBack(OS_TMR *p_tmr, void *p_arg);
static void HydrgFanSpdCycleDlyAdjCallBack(OS_TMR *p_tmr, void *p_arg);

/*
***************************************************************************************************
*                                         PumpSpdInc()
*
* Description : increase the pump speed a grade.
*
* Arguments   : none.
*
* Returns     : none
*
* Notes       : the speed grade whole number is 2000.
***************************************************************************************************
*/
void PumpSpdInc()
{
    uint16_t u16PumpCtlSpd;
    u16PumpCtlSpd = GetPumpCtlSpd();

    if(u16PumpCtlSpd >= 1990) {
        u16PumpCtlSpd = 2000;
    } else {
        u16PumpCtlSpd += 10;
    }

    SetPumpCtlSpd(u16PumpCtlSpd);
}

/*
***************************************************************************************************
*                                         PumpSpdDec()
*
* Description : decrease the pump speed a grade.
*
* Arguments   : none.
*
* Returns     : none
*
* Notes       : the speed grade whole number is 2000.
***************************************************************************************************
*/
void PumpSpdDec()
{
    uint16_t u16PumpCtlSpd;
    u16PumpCtlSpd = GetPumpCtlSpd();

    if(u16PumpCtlSpd < 10) {
        u16PumpCtlSpd = 0;
    } else {
        u16PumpCtlSpd -= 10;
    }

    SetPumpCtlSpd(u16PumpCtlSpd);
}

/*
***************************************************************************************************
*                                         GetPumpCtlSpd()
*
* Description : get the pump speed grade number.
*
* Arguments   : none.
*
* Returns     : the pump speed grade number.
*
* Notes       : the speed grade whole number is 2000.
***************************************************************************************************
*/
uint16_t GetPumpCtlSpd(void)
{
    return g_u16HydrgPumpCtlSpd;
}
/*
***************************************************************************************************
*                                         SetPumpCtlSpd()
*
* Description : set the pump speed grade.
*
* Arguments   : the expected pump speed.
*
* Returns     : none.
*
* Notes       : the speed grade whole number is 2000.
***************************************************************************************************
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
***************************************************************************************************
*                                         HydrgFanSpdInc()
*
* Description : increase the hydrogen fan speed a grade.
*
* Arguments   : none.
*
* Returns     : none
*
* Notes       : the speed grade whole number is 2000.
***************************************************************************************************
*/
void HydrgFanSpdInc()
{
    uint16_t u16HydrgFanCtlSpd;
    u16HydrgFanCtlSpd = GetHydrgFanCurrentCtlSpd();

    if(u16HydrgFanCtlSpd >= 1900) {
        u16HydrgFanCtlSpd = 2000;
    } else {
        u16HydrgFanCtlSpd += 100;
    }

    SetHydrgFanCtlSpd(u16HydrgFanCtlSpd);
}

/*
***************************************************************************************************
*                                         HydrgFanSpdDec()
*
* Description : decrease the hydrogen fan speed a grade.
*
* Arguments   : none.
*
* Returns     : none
*
* Notes       : the speed grade whole number is 2000.
***************************************************************************************************
*/
void HydrgFanSpdDec()
{
    uint16_t u16HydrgFanCtlSpd;
    u16HydrgFanCtlSpd = GetHydrgFanCurrentCtlSpd();

    if(u16HydrgFanCtlSpd < 100) {
        u16HydrgFanCtlSpd = 0;
    } else {
        u16HydrgFanCtlSpd -= 100;
    }

    SetHydrgFanCtlSpd(u16HydrgFanCtlSpd);
}

/*
***************************************************************************************************
*                               GetHydrgFanCurrentCtlSpd()
*
* Description : get the hydrogen fan speed grade number.
*
* Arguments   : none.
*
* Returns     : the hydrogen fan speed grade number.
*
* Notes       : the speed grade whole number is 2000.
***************************************************************************************************
*/
uint16_t GetHydrgFanCurrentCtlSpd(void)
{
    return g_u16HydrgFanCurrentCtlSpd;
}

uint16_t GetHydrgFanExpectCtlSpd(void)
{
    return g_u16HydrgFanExpectCtlSpd;
}

/*
***************************************************************************************************
*                                         SetHydrgFanCtlSpd()
*
* Description : set the hydrogen fan speed grade.
*
* Arguments   : the expected hydrogen fan speed.
*
* Returns     : none.
*
* Notes       : the speed grade whole number is 2000.
***************************************************************************************************
*/
void SetHydrgFanCtlSpd(uint16_t i_u16NewSpd)
{
    CPU_SR_ALLOC();
    CPU_CRITICAL_ENTER();
    g_u16HydrgFanCurrentCtlSpd = i_u16NewSpd;
    BSP_SetHydrgFanSpd(i_u16NewSpd);
    CPU_CRITICAL_EXIT();
}

/*
***************************************************************************************************
*                                         SetHydrgFanCtlSpdSmoothly()
*
* Description : Delay for a period of time to set the hydrogen fan speed.
*
* Arguments   : i_u8StartDlyAdjTime:start adjust delay time.
*               i_u8DelayTime:Adjust the cycle time delay(s),if i_u8DelayTime =0,set immediately.
*               i_u16ExpectSpdValue:the expected hydrogen fan speed.
*               
* Returns     : none.
*
* Notes       : the actual speed grade whole number is 2000.
***************************************************************************************************
*/
void SetHydrgFanCtlSpdSmoothly(uint16_t i_u16CurrentSpdValue,uint8_t i_u8StartDlyAdjTime, uint8_t i_u8CycleDlyAdjTime,uint16_t i_u16ExpSpdValue)
{
    OS_ERR err;
    
    g_u16HydrgFanExpectCtlSpd = i_u16ExpSpdValue ;

    if(i_u8CycleDlyAdjTime > 0) {//周期延时调节定时器   
         OSTmrCreate((OS_TMR *)&HydrgFanSpdCycleDlyTimeAdjustTmr,
                     (CPU_CHAR *)"Hydrg fan Speed cycle Delay Adjust Timer",
                     (OS_TICK)0,
                     (OS_TICK)i_u8CycleDlyAdjTime * OS_CFG_TMR_TASK_RATE_HZ,
                     (OS_OPT)OS_OPT_TMR_PERIODIC,
                     (OS_TMR_CALLBACK_PTR)HydrgFanSpdCycleDlyAdjCallBack,
                     (void *)0,
                     (OS_ERR *)&err);
    }
                    
    if(i_u8StartDlyAdjTime > 0) {//单次延时调节定时器   
        OSTmrCreate((OS_TMR *)&HydrgFanSpdDlyAdjTmr,
                    (CPU_CHAR *)"Hydrg fan Speed Delay Adjust Timer",
                    (OS_TICK)i_u8StartDlyAdjTime * OS_CFG_TMR_TASK_RATE_HZ,//dly
                    (OS_TICK)0,                                             //period
                    (OS_OPT)OS_OPT_TMR_ONE_SHOT,    
                    (OS_TMR_CALLBACK_PTR)HydrgFanSpdDlyAdjCallBack,
                    (void *)0,
                    (OS_ERR *)&err);

        if(err == OS_ERR_NONE) {
            OSTmrStart(&HydrgFanSpdDlyAdjTmr, &err);
        }
    } else {
        if(i_u8CycleDlyAdjTime > 0){
            OSTmrStart(&HydrgFanSpdCycleDlyTimeAdjustTmr, &err);
        }
    }
    SetHydrgFanCtlSpd(i_u16CurrentSpdValue);    
}
/*
***************************************************************************************************
*                            HydrgFanSpdAdjustDlyCallBack()
*
* Description : when HydrgFanSpdCycleDlyTimeAdjustTmr time out that call back function.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : Application.
*
* Note(s)     : none.
***************************************************************************************************
*/
static void HydrgFanSpdDlyAdjCallBack(OS_TMR *p_tmr, void *p_arg)
{
    OS_ERR err;
    
    OSTmrStart(&HydrgFanSpdCycleDlyTimeAdjustTmr, &err);
    OSTmrDel(&HydrgFanSpdDlyAdjTmr, &err);//删除单次定时器
}

static void HydrgFanSpdCycleDlyAdjCallBack(OS_TMR *p_tmr, void *p_arg)
{
    OS_ERR err;

    if(g_u16HydrgFanCurrentCtlSpd != g_u16HydrgFanExpectCtlSpd) {
        if(g_u16HydrgFanCurrentCtlSpd < g_u16HydrgFanExpectCtlSpd) {
            HydrgFanSpdInc();
        } else {
            HydrgFanSpdDec();
        }
    } else {
        OSTmrDel(&HydrgFanSpdCycleDlyTimeAdjustTmr, &err);
    }
}
/*
***************************************************************************************************
*                                         StackFanSpdInc()
*
* Description : increase the stack fan1 and fan2 speed a grade.
*
* Arguments   : none.
*
* Returns     : none
*
* Notes       : the speed grade whole number is 200.
***************************************************************************************************
*/
void StackFanSpdInc()
{
    uint16_t u16StackFanCtlSpd;
    u16StackFanCtlSpd = GetStackFanCtlSpd();

    if(u16StackFanCtlSpd >= 1900) {
        u16StackFanCtlSpd = 2000;
    } else {
        u16StackFanCtlSpd += 100;
    }

    SetStackFanCtlSpd(u16StackFanCtlSpd);
}

/*
***************************************************************************************************
*                                         StackFanSpdDec()
*
* Description : decrease the stack fan speed a grade.
*
* Arguments   : none.
*
* Returns     : none
*
* Notes       : the speed grade whole number is 200.
***************************************************************************************************
*/
void StackFanSpdDec()
{
    uint16_t u16StackFanCtlSpd;
    u16StackFanCtlSpd = GetStackFanCtlSpd();

    if(u16StackFanCtlSpd < 100) {
        u16StackFanCtlSpd = 0;
    } else {
        u16StackFanCtlSpd -= 100;
    }

    SetStackFanCtlSpd(u16StackFanCtlSpd);
}


/*
***************************************************************************************************
*                                         GetStackFanCtlSpd()
*
* Description : get the stack fan speed grade number.
*
* Arguments   : none.
*
* Returns     : the stack fan speed grade number.
*
* Notes       : the speed grade whole number is 200.
***************************************************************************************************
*/
uint16_t GetStackFanCtlSpd(void)
{
    return g_u16StackFanCtlSpd;
}
/*
***************************************************************************************************
*                                         SetStackFanCtlSpd()
*
* Description : set the stack fan speed grade.
*
* Arguments   : the expected stack fan speed.
*
* Returns     : none.
*
* Notes       : the speed grade whole number is 200.
***************************************************************************************************
*/
void SetStackFanCtlSpd(uint16_t i_u16NewSpd)
{
    CPU_SR_ALLOC();
    CPU_CRITICAL_ENTER();
    g_u16StackFanCtlSpd = i_u16NewSpd;
    BSP_SetStackFanSpd(i_u16NewSpd);
    CPU_CRITICAL_EXIT();

}


/*
********************************************************************************************************
*                                     BSP_DevSpdCaptureFinishedHandler()
*
* Description : 速度控制设备速度输入捕获完成中断服务函数.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : none.
*
* Note(s)     : 捕获脉宽+次数.
********************************************************************************************************
*/
void BSP_DevSpdCaptureFinishedHandler(void)
{
    /*输入捕获状态:bit15-捕获使能位，bit14-捕获中状态位,,其余位做捕获边沿计数位*/
    static uint16_t TIM1CHX_CAPTURE_STA[3] = {0, 0,0}; //确保有一个通道被使能,才能轮流
    static uint8_t  stEnableChannelNum = 0;//通道切换计数,每0.5秒切换一次
    static uint16_t u16SpeedCaptureValue[3][2];  //临时保存三个通道捕获周期内第一次和最后一次的计数值

    if(TIM_GetITStatus(TIM1, TIM_IT_CC1) != RESET) { //捕获到水泵转速脉冲
        if(g_eSpdCaptureWorkSwitch[PUMP_SPD_MONITOR] == ON) {
            if(TIM1CHX_CAPTURE_STA[PUMP_SPD_MONITOR] & 0x8000) {

                TIM1CHX_CAPTURE_STA[PUMP_SPD_MONITOR]++;

                if(TIM1CHX_CAPTURE_STA[PUMP_SPD_MONITOR] & 0x4000) { //不是第一次进入中断
                    u16SpeedCaptureValue[PUMP_SPD_MONITOR][LAST_EDGE] = TIM_GetCapture1(TIM1);//不断更新该计数值,直到记录到捕获周期内最后一次进入中断的计数值
                } else {
                    TIM1CHX_CAPTURE_STA[PUMP_SPD_MONITOR] |= 0x4000;//标记首次捕获到水泵转速脉冲
                    u16SpeedCaptureValue[PUMP_SPD_MONITOR][FRONT_EDGE] = TIM_GetCapture1(TIM1);//记录第一次的计数值
                }
            }
        }

        TIM_ClearITPendingBit(TIM1, TIM_IT_CC1);
    } else if(TIM_GetITStatus(TIM1, TIM_IT_CC2) != RESET) { //捕获到水泵转速脉冲
        if(g_eSpdCaptureWorkSwitch[HYDROGEN_FAN_SPD_MONITOR] == ON) {
            if(TIM1CHX_CAPTURE_STA[HYDROGEN_FAN_SPD_MONITOR] & 0x8000) {

                TIM1CHX_CAPTURE_STA[HYDROGEN_FAN_SPD_MONITOR]++;

                if(TIM1CHX_CAPTURE_STA[HYDROGEN_FAN_SPD_MONITOR] & 0x4000) { //不是第一次进入中断
                    u16SpeedCaptureValue[HYDROGEN_FAN_SPD_MONITOR][LAST_EDGE] = TIM_GetCapture2(TIM1);
                } else {
                    TIM1CHX_CAPTURE_STA[HYDROGEN_FAN_SPD_MONITOR] |= 0x4000;
                    u16SpeedCaptureValue[HYDROGEN_FAN_SPD_MONITOR][FRONT_EDGE] = TIM_GetCapture2(TIM1);
                }
            }
        }

        TIM_ClearITPendingBit(TIM1, TIM_IT_CC2);
    } else if(TIM_GetITStatus(TIM1, TIM_IT_CC3) != RESET) { //捕获到电堆风机1转速脉冲
        if(g_eSpdCaptureWorkSwitch[STACK_FAN_SPD_MONITOR] == ON) {
            if(TIM1CHX_CAPTURE_STA[STACK_FAN_SPD_MONITOR] & 0x8000) {

                TIM1CHX_CAPTURE_STA[STACK_FAN_SPD_MONITOR]++;

                if(TIM1CHX_CAPTURE_STA[STACK_FAN_SPD_MONITOR] & 0x4000) {
                    u16SpeedCaptureValue[STACK_FAN_SPD_MONITOR][LAST_EDGE] = TIM_GetCapture3(TIM1);
                } else {
                    TIM1CHX_CAPTURE_STA[STACK_FAN_SPD_MONITOR] |= 0x4000;
                    u16SpeedCaptureValue[STACK_FAN_SPD_MONITOR][FRONT_EDGE] = TIM_GetCapture3(TIM1);
                }
            }
        }

        TIM_ClearITPendingBit(TIM1, TIM_IT_CC3);
    } else {
    }

    /*更新中断中每0.5s切换一次捕获通道*/
    if(TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET) {
        stEnableChannelNum++;

        if(stEnableChannelNum == 1) {
            if((TIM1CHX_CAPTURE_STA[PUMP_SPD_MONITOR] & 0x3FFF) != 0) {
                g_u16SpdCaptureEdgeNum[PUMP_SPD_MONITOR] = (TIM1CHX_CAPTURE_STA[PUMP_SPD_MONITOR] & 0x3FFF);//保存捕获边沿次数
                g_u16SpeedCaptureValue[PUMP_SPD_MONITOR][FRONT_EDGE] = u16SpeedCaptureValue[PUMP_SPD_MONITOR][FRONT_EDGE];
                g_u16SpeedCaptureValue[PUMP_SPD_MONITOR][LAST_EDGE] = u16SpeedCaptureValue[PUMP_SPD_MONITOR][LAST_EDGE];
            } else {
                g_fSpdCaptureFrequency[PUMP_SPD_MONITOR] = 0;
            }

            TIM1CHX_CAPTURE_STA[PUMP_SPD_MONITOR] = 0;  //关闭本捕获通道捕获并清空捕获边沿计数
            TIM_ITConfig(TIM1, TIM_IT_CC1, DISABLE);
            TIM1CHX_CAPTURE_STA[HYDROGEN_FAN_SPD_MONITOR] = 0x8000;   //开启下一通道捕获
            TIM_ITConfig(TIM1, TIM_IT_CC2, ENABLE);
        } else if(stEnableChannelNum == 2) {
            if((TIM1CHX_CAPTURE_STA[HYDROGEN_FAN_SPD_MONITOR] & 0x3FFF) != 0) {
                g_u16SpdCaptureEdgeNum[HYDROGEN_FAN_SPD_MONITOR] = (TIM1CHX_CAPTURE_STA[HYDROGEN_FAN_SPD_MONITOR] & 0x3FFF);//保存捕获边沿次数
                g_u16SpeedCaptureValue[HYDROGEN_FAN_SPD_MONITOR][FRONT_EDGE] = u16SpeedCaptureValue[HYDROGEN_FAN_SPD_MONITOR][FRONT_EDGE];
                g_u16SpeedCaptureValue[HYDROGEN_FAN_SPD_MONITOR][LAST_EDGE] = u16SpeedCaptureValue[HYDROGEN_FAN_SPD_MONITOR][LAST_EDGE];
            } else {
                g_fSpdCaptureFrequency[HYDROGEN_FAN_SPD_MONITOR] = 0;
            }

            TIM1CHX_CAPTURE_STA[HYDROGEN_FAN_SPD_MONITOR] = 0;  //关闭本捕获通道捕获并清空捕获边沿计数
            TIM_ITConfig(TIM1, TIM_IT_CC2, DISABLE);
            TIM1CHX_CAPTURE_STA[STACK_FAN_SPD_MONITOR] = 0x8000;   //开启下一通道捕获
            TIM_ITConfig(TIM1, TIM_IT_CC3, ENABLE);
        } else if(stEnableChannelNum == 3) {
            if((TIM1CHX_CAPTURE_STA[STACK_FAN_SPD_MONITOR] & 0x3FFF) != 0) {
                g_u16SpdCaptureEdgeNum[STACK_FAN_SPD_MONITOR] = (TIM1CHX_CAPTURE_STA[STACK_FAN_SPD_MONITOR] & 0x3FFF);//保存捕获边沿次数
                g_u16SpeedCaptureValue[STACK_FAN_SPD_MONITOR][FRONT_EDGE] = u16SpeedCaptureValue[STACK_FAN_SPD_MONITOR][FRONT_EDGE];
                g_u16SpeedCaptureValue[STACK_FAN_SPD_MONITOR][LAST_EDGE] = u16SpeedCaptureValue[STACK_FAN_SPD_MONITOR][LAST_EDGE];
            } else {
                g_fSpdCaptureFrequency[STACK_FAN_SPD_MONITOR] = 0;
            }

            TIM1CHX_CAPTURE_STA[STACK_FAN_SPD_MONITOR] = 0;  //关闭本捕获通道捕获并清空捕获边沿计数
            TIM_ITConfig(TIM1, TIM_IT_CC3, DISABLE);
            TIM1CHX_CAPTURE_STA[PUMP_SPD_MONITOR] = 0x8000;   //开启下一通道捕获
            TIM_ITConfig(TIM1, TIM_IT_CC1, ENABLE);
            stEnableChannelNum = 0; //恢复初始通道

        } else {}

        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
    }
}

/*
***************************************************************************************************
*                          GetHydrgFanFeedBackSpd()
*
* Description : get the hydrogen fan feed back speed grade number.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : Application.
*
* Note(s)     : 两脉冲表示转了一圈.
***************************************************************************************************
*/
uint16_t    GetPumpFeedBackSpd(void)
{
    /*2脉冲每转、一分钟的脉冲计数次数->0.5 * 60 * 2 = 60 ,5000为定时器溢出计数值*/
    g_fSpdCaptureFrequency[PUMP_SPD_MONITOR] = 60 * (((5000.0 - g_u16SpeedCaptureValue[PUMP_SPD_MONITOR][FRONT_EDGE] \
            + g_u16SpeedCaptureValue[PUMP_SPD_MONITOR][LAST_EDGE]) / 5000.0) + (g_u16SpdCaptureEdgeNum[PUMP_SPD_MONITOR] - 1));

    return (uint16_t)(g_fSpdCaptureFrequency[PUMP_SPD_MONITOR] * 0.33);//2000/6000（上位机数据显示比例）

}
/*
***************************************************************************************************
*                         GetHydrgFanFeedBackSpd()
*
* Description : get the hydrogen fan feed back speed grade number.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : Application.
*
* Note(s)     : none.
***************************************************************************************************
*/
uint16_t GetHydrgFanFeedBackSpd(void)
{
    g_fSpdCaptureFrequency[HYDROGEN_FAN_SPD_MONITOR] = 60 * (((5000.0 - g_u16SpeedCaptureValue[HYDROGEN_FAN_SPD_MONITOR][FRONT_EDGE] \
            + g_u16SpeedCaptureValue[HYDROGEN_FAN_SPD_MONITOR][LAST_EDGE]) / 5000.0) + (g_u16SpdCaptureEdgeNum[HYDROGEN_FAN_SPD_MONITOR] - 1));
    return (uint16_t)(g_fSpdCaptureFrequency[HYDROGEN_FAN_SPD_MONITOR] * 0.083);// 2000 / 24000
}

/*
***************************************************************************************************
*                        GetStackFanSpdFeedBack()
*
* Description : get the stack fan feed back speed grade number.
*
* Arguments   : none.
*
* Returns     : the stack fan speed grade number.
*
* Notes       : the speed grade whole number is 2000.
***************************************************************************************************
*/
uint16_t GetStackFanSpdFeedBack(void)
{
    g_fSpdCaptureFrequency[STACK_FAN_SPD_MONITOR] = 60 * (((5000.0 - g_u16SpeedCaptureValue[STACK_FAN_SPD_MONITOR][FRONT_EDGE] \
            + g_u16SpeedCaptureValue[STACK_FAN_SPD_MONITOR][LAST_EDGE]) / 5000.0) + (g_u16SpdCaptureEdgeNum[STACK_FAN_SPD_MONITOR] - 1));

    return (uint16_t)(g_fSpdCaptureFrequency[STACK_FAN_SPD_MONITOR]);
}

/*
***************************************************************************************************
*                     SetSpdMonitorSwitch()
*
* Description : Turn on the running speed control device monitor.
*
* Arguments   : none.
*
* Returns     : none.
*
* Notes       : none.
***************************************************************************************************
*/
void SetSpdMonitorSwitch(SPEED_MONITOR_CHANNEL_Typedef i_SpdMonitorChannel)
{
    g_eSpdCaptureWorkSwitch[i_SpdMonitorChannel] = ON;

}
/*
***************************************************************************************************
*                     ResetSpdMonitorSwitch()
*
* Description : Turn off the running speed control device monitor.
*
* Arguments   : none.
*
* Returns     : none.
*
* Notes       : none.
***************************************************************************************************
*/
void ResetSpdMonitorSwitch(SPEED_MONITOR_CHANNEL_Typedef i_SpdMonitorChannel)
{
    g_eSpdCaptureWorkSwitch[i_SpdMonitorChannel] = OFF;
}
