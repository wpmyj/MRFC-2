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
* Programmer(s) : JasonFan
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
#define SPEED_CONTROL_DEVICE_MANAGE_TASK_STK_SIZE       100

#define SPEED_SAMPLE_CHANNEL          3u//速度反馈信号采样通道数

#define FRONT_EDGE      0 //捕获脉冲前沿
#define LAST_EDGE       1 //捕获脉冲后沿
/*
***************************************************************************************************
*                                         OS-RELATED    VARIABLES
***************************************************************************************************
*/

OS_TCB      SpeedControlDevManageTaskTCB;
static      CPU_STK     SpdCtrlDevManageTaskStk[SPEED_CONTROL_DEVICE_MANAGE_TASK_STK_SIZE];

static      OS_TMR      PumpSpdAdjustTmr;//泵速平滑调节定时器

static      OS_TMR      HydrogenFanSpdCycleDlyTimeAdjustTmr;//制氢风机延时调节定时器
/*
***************************************************************************************************
*                                       LOCAL GLOBAL VARIABLES
***************************************************************************************************
*/
static  uint16_t    g_u16HydrogenFanExpectCtrlSpd = 0;
static  uint16_t    g_u16HydrogenFanCurrentCtrlSpd = 0;

static  uint16_t    g_u16PumpExpectCtrlSpd = 0;
static  uint16_t    g_u16PumpCurrentCtrlSpd = 0;
static  uint16_t    g_u16PumpAdjAcceleratedSpdVal = 0;

static  uint16_t    g_u16StackFanCtlSpd = 0;

static  SWITCH_TYPE_VARIABLE_Typedef g_eSpdCaptureWorkSwitch[3] = {OFF, OFF, OFF}; //3路通道监测开关
static  float 		g_fSpdCaptureFrequency[SPEED_SAMPLE_CHANNEL] = {0.0, 0.0, 0.0}; //对应检测通道的脉冲频率数
static  uint16_t    g_u16SpdCaptureStatu[SPEED_SAMPLE_CHANNEL] = {0, 0, 0};//捕获状态:0-第一个上升沿，1:第二个上升沿
static  uint16_t    g_SpeedCaptureValueBuff[SPEED_SAMPLE_CHANNEL][2] = {0}; //捕获边沿计数值
static  uint16_t    g_u16SpeedCaptureDeltValue[SPEED_SAMPLE_CHANNEL] = {0, 0, 0};//计数器差值

/*
***************************************************************************************************
*                                         FUNCTION PROTOTYPES
***************************************************************************************************
*/
static void HydrogenFanSpdCycleDlyAdjCallBack(OS_TMR *p_tmr, void *p_arg);

static void PumpAdjustCallBack(OS_TMR *p_tmr, void *p_arg);

static void SpeedControlDevManageTask(void);
/*
***************************************************************************************************
*                              SpeedControlDevManageTaskCreate()
*
* Description : increase the pump speed a grade.
*
* Arguments   : none.
*
* Returns     : none
*
* Notes       : the actual speed grade whole number is 2000.
***************************************************************************************************
*/
void SpeedControlDevManageTaskCreate(void)
{
    OS_ERR  err;

    OSTaskCreate((OS_TCB *)&SpeedControlDevManageTaskTCB,
                 (CPU_CHAR *)"Speed Monitor task",
                 (OS_TASK_PTR) SpeedControlDevManageTask,
                 (void *) 0,
                 (OS_PRIO) SPEED_CONTROL_DEVICE_MANAGE_TASK_PRIO,
                 (CPU_STK *)&SpdCtrlDevManageTaskStk[0],
                 (CPU_STK_SIZE) SPEED_CONTROL_DEVICE_MANAGE_TASK_STK_SIZE / 10,
                 (CPU_STK_SIZE) SPEED_CONTROL_DEVICE_MANAGE_TASK_STK_SIZE,
                 (OS_MSG_QTY) 5u,
                 (OS_TICK) 0u,
                 (void *) 0,
                 (OS_OPT)(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR *)&err);
    APP_TRACE_DEBUG(("Created Speed Monitor task, and err code is %d...\r\n", err));
}

/*
***************************************************************************************************
*                                         SpeedControlDevManageTask()
*
* Description : increase the pump speed a grade.
*
* Arguments   : none.
*
* Returns     : none
*
* Notes       : the actual speed grade whole number is 2000.
***************************************************************************************************
*/
static void SpeedControlDevManageTask(void)
{
    OS_ERR      err;

    OSTaskSuspend(NULL, &err);

    while(DEF_TRUE) {
        OSTimeDlyHMSM(0, 0, 0, 500,
                      OS_OPT_TIME_HMSM_STRICT,
                      &err);
//        APP_TRACE_INFO(("g_EnableChannelNum: %d...\n\r\n\r", g_EnableChannelNum));
//        APP_TRACE_INFO(("TIM1CHX_CAPTURE_STA[PUMP_SPD_MONITOR]: %X...\n\r", TIM1CHX_CAPTURE_STA[PUMP_SPD_MONITOR]));
//        APP_TRACE_INFO(("TIM1CHX_CAPTURE_STA[HYDROGEN_FAN_SPD_MONITOR]: %X...\n\r", TIM1CHX_CAPTURE_STA[HYDROGEN_FAN_SPD_MONITOR]));
//        APP_TRACE_INFO(("TIM1CHX_CAPTURE_STA[STACK_FAN_SPD_MONITOR]: %X...\n\r\n\r", TIM1CHX_CAPTURE_STA[STACK_FAN_SPD_MONITOR]));
    }
}
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
    u16PumpCtlSpd = GetPumpCurrentCtrlSpd();

    if(u16PumpCtlSpd >= 1990) {
        u16PumpCtlSpd = 2000;
    } else {
        u16PumpCtlSpd += 10;
    }

    SetPumpCurrentCtrlSpd(u16PumpCtlSpd);
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
    u16PumpCtlSpd = GetPumpCurrentCtrlSpd();

    if(u16PumpCtlSpd < 10) {
        u16PumpCtlSpd = 0;
    } else {
        u16PumpCtlSpd -= 10;
    }

    SetPumpCurrentCtrlSpd(u16PumpCtlSpd);
}

/*
***************************************************************************************************
*                                         GetPumpCurrentCtrlSpd()
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
uint16_t GetPumpCurrentCtrlSpd(void)
{
    return g_u16PumpCurrentCtrlSpd;
}

uint16_t GetPumpExpectCtrlSpd(void)
{
    return g_u16PumpExpectCtrlSpd;
}
/*
***************************************************************************************************
*                                         SetPumpCurrentCtrlSpd()
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
void SetPumpCurrentCtrlSpd(uint16_t i_u16NewSpd)
{
    CPU_SR_ALLOC();
    CPU_CRITICAL_ENTER();
    g_u16PumpCurrentCtrlSpd = i_u16NewSpd;
    BSP_SetPumpSpd(i_u16NewSpd);
    CPU_CRITICAL_EXIT();
}

/*
***************************************************************************************************
*                            SetPumpExpectSpdSmoothly()
*
* Description : Control the speed of the pump smoothly.
*
* Argument(s) : i_u16ExpectPumpSpdValue:Expect Speed.
*               i_u8SmoothlyControlTimeDly:Adjust pump Delay Time >= 0;if i_u8AdjustTimeDly = 0,set immediately.
*
* Return(s)   : none.
*
* Caller(s)   : none.
*
* Note(s)     : the actual speed grade whole number is 2000.
***************************************************************************************************
*/
void SetPumpExpectSpdSmoothly(u16 i_u16ExpectPumpSpdValue, u8 i_u8SmoothlyControlTimeDly)
{
    OS_ERR err;
    uint16_t m_u16PumpSpdDiffValue = 0;

    OSTmrDel(&PumpSpdAdjustTmr, &err);//重置定时器状态

    if(i_u8SmoothlyControlTimeDly > 0) { //需要速度平滑处理
        if(i_u16ExpectPumpSpdValue != g_u16PumpExpectCtrlSpd){//产生新的期望速度
            g_u16PumpExpectCtrlSpd = i_u16ExpectPumpSpdValue;
        }
        
        if(g_u16PumpCurrentCtrlSpd != g_u16PumpExpectCtrlSpd) {
            m_u16PumpSpdDiffValue = abs(g_u16PumpExpectCtrlSpd - g_u16PumpCurrentCtrlSpd);
        }
        
        /*加速度计算*/
        g_u16PumpAdjAcceleratedSpdVal = m_u16PumpSpdDiffValue / 2 / i_u8SmoothlyControlTimeDly;
        
        OSTmrCreate((OS_TMR *)&PumpSpdAdjustTmr,
                    (CPU_CHAR *)"Pump Speed Adjust Timer",
                    (OS_TICK)0,
                    (OS_TICK) OS_CFG_TMR_TASK_RATE_HZ / 2,      /* control cycle 0.5s */
                    (OS_OPT)OS_OPT_TMR_PERIODIC,
                    (OS_TMR_CALLBACK_PTR)PumpAdjustCallBack,
                    (void *)0,
                    (OS_ERR *)&err);

        if(err == OS_ERR_NONE) {
            OSTmrStart(&PumpSpdAdjustTmr, &err);
        }
    } else {
        SetPumpCurrentCtrlSpd(i_u16ExpectPumpSpdValue);
    }
}
/*
***************************************************************************************************
*                            PumpAdjustCallBack()
*
* Description : when PumpSpdAdjustTmr time out that call back function.
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
static void PumpAdjustCallBack(OS_TMR *p_tmr, void *p_arg)
{	
	OS_ERR err;
    uint16_t u16PumpCurrentCtlSpd = 0;

    u16PumpCurrentCtlSpd = GetPumpCurrentCtrlSpd();

    if(u16PumpCurrentCtlSpd != GetPumpExpectCtrlSpd()) {
        if(u16PumpCurrentCtlSpd > GetPumpExpectCtrlSpd()) {
            if(u16PumpCurrentCtlSpd > g_u16PumpAdjAcceleratedSpdVal){//防止减溢出
                u16PumpCurrentCtlSpd -= g_u16PumpAdjAcceleratedSpdVal;
            }else{
                u16PumpCurrentCtlSpd = 0;
                BSP_LqdValve2_PwrOff();//延时关闭完后才关阀
            }
        } else {
            u16PumpCurrentCtlSpd += g_u16PumpAdjAcceleratedSpdVal;
            if(u16PumpCurrentCtlSpd >= g_u16PumpExpectCtrlSpd){
                u16PumpCurrentCtlSpd = g_u16PumpExpectCtrlSpd;
            }
        }
        SetPumpCurrentCtrlSpd(u16PumpCurrentCtlSpd);
    } else {
        OSTmrDel(&PumpSpdAdjustTmr, &err);
    }
}
/*
***************************************************************************************************
*                                         HydrogenFanCtrSpdInc()
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
void HydrogenFanCtrSpdInc()
{
    uint16_t u16HydrgFanCtlSpd;
    u16HydrgFanCtlSpd = GetHydrgFanCurrentCtlSpd();

    if(u16HydrgFanCtlSpd >= 1900) {
        u16HydrgFanCtlSpd = 2000;
    } else {
        u16HydrgFanCtlSpd += 100;
    }

    SetHydrogenFanCtrlSpd(u16HydrgFanCtlSpd);
}

/*
***************************************************************************************************
*                                         HydrogenFanCtrSpdDec()
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
void HydrogenFanCtrSpdDec()
{
    uint16_t u16HydrgFanCtlSpd;
    u16HydrgFanCtlSpd = GetHydrgFanCurrentCtlSpd();

    if(u16HydrgFanCtlSpd < 100) {
        u16HydrgFanCtlSpd = 0;
    } else {
        u16HydrgFanCtlSpd -= 100;
    }

    SetHydrogenFanCtrlSpd(u16HydrgFanCtlSpd);
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
    return g_u16HydrogenFanCurrentCtrlSpd;
}

uint16_t GetHydrgFanExpectCtlSpd(void)
{
    return g_u16HydrogenFanExpectCtrlSpd;
}

/*
***************************************************************************************************
*                                         SetHydrogenFanCtrlSpd()
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
void SetHydrogenFanCtrlSpd(uint16_t i_u16NewSpd)
{
    CPU_SR_ALLOC();
    CPU_CRITICAL_ENTER();
    g_u16HydrogenFanCurrentCtrlSpd = i_u16NewSpd;
    BSP_SetHydrgFanSpd(i_u16NewSpd);
    CPU_CRITICAL_EXIT();
}

/*
***************************************************************************************************
*                                         SetHydrogenFanCtrlSpdSmoothly()
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

void SetHydrogenFanCtrlSpdSmoothly(uint16_t i_u16CurrentSpdVal, uint8_t i_u8FirstDlyAdjTime, uint8_t i_u8CycleDlyAdjTime, uint16_t i_u16ExpSpdVal)
{
    OS_ERR err;

    OSTmrDel(&HydrogenFanSpdCycleDlyTimeAdjustTmr, &err);//重置定时器状态
    g_u16HydrogenFanExpectCtrlSpd = i_u16ExpSpdVal ;

    if((i_u8FirstDlyAdjTime > 0) || (i_u8CycleDlyAdjTime > 0)) {
        OSTmrCreate((OS_TMR *)&HydrogenFanSpdCycleDlyTimeAdjustTmr,
                    (CPU_CHAR *)"Hydrg fan Speed cycle Delay Adjust Timer",
                    (OS_TICK)i_u8FirstDlyAdjTime * OS_CFG_TMR_TASK_RATE_HZ,//首次延时dly(s)
                    (OS_TICK)i_u8CycleDlyAdjTime * OS_CFG_TMR_TASK_RATE_HZ,//周期延时cycle(s)
                    (OS_OPT)OS_OPT_TMR_PERIODIC,
                    (OS_TMR_CALLBACK_PTR)HydrogenFanSpdCycleDlyAdjCallBack,
                    (void *)0,
                    (OS_ERR *)&err);
                    
         if(err == OS_ERR_NONE) {
            OSTmrStart(&HydrogenFanSpdCycleDlyTimeAdjustTmr, &err);   
        }          
    } else {
        SetHydrogenFanCtrlSpd(g_u16HydrogenFanExpectCtrlSpd);
    }  
    SetHydrogenFanCtrlSpd(i_u16CurrentSpdVal);
}
/*
***************************************************************************************************
*                            HydrogenFanSpdCycleDlyAdjCallBack()
*
* Description : when HydrogenFanSpdCycleDlyTimeAdjustTmr time out that call back function.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Note(s)     : none.
***************************************************************************************************
*/
static void HydrogenFanSpdCycleDlyAdjCallBack(OS_TMR *p_tmr, void *p_arg)
{
    OS_ERR err;

    if(g_u16HydrogenFanCurrentCtrlSpd != g_u16HydrogenFanExpectCtrlSpd) {
        if(g_u16HydrogenFanCurrentCtrlSpd < g_u16HydrogenFanExpectCtrlSpd) {
            HydrogenFanCtrSpdInc();
        } else {
            HydrogenFanCtrSpdDec();
        }
    } else {
        OSTmrDel(&HydrogenFanSpdCycleDlyTimeAdjustTmr, &err);
    }
}
/*
***************************************************************************************************
*                                         StackFanSpdInc()
*
* Description : increase the stack fan speed a grade.
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

    SetStackFanCtrlSpd(u16StackFanCtlSpd);
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

    SetStackFanCtrlSpd(u16StackFanCtlSpd);
}


/*
***************************************************************************************************
*                           GetStackFanCtlSpd()
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
*                                         SetStackFanCtrlSpd()
*
* Description : set the stack fan speed grade.
*
* Arguments   : the expected stack fan speed.
*
* Returns     : none.
*
* Notes       : the speed grade whole number is 2000.
***************************************************************************************************
*/
void SetStackFanCtrlSpd(uint16_t i_u16NewSpd)
{
    CPU_SR_ALLOC();
    CPU_CRITICAL_ENTER();
    g_u16StackFanCtlSpd = i_u16NewSpd;
    BSP_SetStackFanSpd(i_u16NewSpd);
    CPU_CRITICAL_EXIT();
}

/*
***************************************************************************************************
*                                         ResetSpdCaptureValue()
*
* Description : set the stack fan speed grade.
*
* Arguments   : the expected stack fan speed.
*
* Returns     : none.
*
* Notes       : the speed grade whole number is 2000.
***************************************************************************************************
*/
void ResetSpdCaptureValue(uint8_t i_CaptureChannel)
{
    g_fSpdCaptureFrequency[i_CaptureChannel] = 0;
    g_SpeedCaptureValueBuff[i_CaptureChannel][LAST_EDGE] = 0;
    g_SpeedCaptureValueBuff[i_CaptureChannel][FRONT_EDGE] = 0;
	g_u16SpeedCaptureDeltValue[i_CaptureChannel] = 0;
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

	if(TIM_GetITStatus(TIM1,TIM_IT_CC1)!=RESET){
		
		if(g_eSpdCaptureWorkSwitch[PUMP_SPD_MONITOR] == DEF_ON){      
		           
           if(g_u16SpdCaptureStatu[PUMP_SPD_MONITOR] == FRONT_EDGE){//捕获第一个上升沿
               
               g_u16SpdCaptureStatu[PUMP_SPD_MONITOR] = 1;
               g_SpeedCaptureValueBuff[PUMP_SPD_MONITOR][FRONT_EDGE] = TIM_GetCapture2(TIM1);//记录第一次上升沿的CNT值
               
           }else if(g_u16SpdCaptureStatu[PUMP_SPD_MONITOR] == LAST_EDGE){//捕获第二个上升沿
               
               g_u16SpdCaptureStatu[PUMP_SPD_MONITOR] = 0;
               g_SpeedCaptureValueBuff[PUMP_SPD_MONITOR][LAST_EDGE] = TIM_GetCapture2(TIM1);//记录第二次上升沿的CNT值
               
               if(g_SpeedCaptureValueBuff[PUMP_SPD_MONITOR][FRONT_EDGE] < g_SpeedCaptureValueBuff[PUMP_SPD_MONITOR][LAST_EDGE]){
                    g_u16SpeedCaptureDeltValue[PUMP_SPD_MONITOR] = g_SpeedCaptureValueBuff[PUMP_SPD_MONITOR][LAST_EDGE] - g_SpeedCaptureValueBuff[PUMP_SPD_MONITOR][FRONT_EDGE];           //两次上升沿的差值
               }else if(g_SpeedCaptureValueBuff[PUMP_SPD_MONITOR][FRONT_EDGE] > g_SpeedCaptureValueBuff[PUMP_SPD_MONITOR][LAST_EDGE]){
                    g_u16SpeedCaptureDeltValue[PUMP_SPD_MONITOR] = (2000 - g_SpeedCaptureValueBuff[PUMP_SPD_MONITOR][FRONT_EDGE]) + g_SpeedCaptureValueBuff[PUMP_SPD_MONITOR][LAST_EDGE];  //两次上升沿的差值
               }else{
                 g_u16SpeedCaptureDeltValue[PUMP_SPD_MONITOR]=0;
               }
           }
       }else{
		   ResetSpdCaptureValue(PUMP_SPD_MONITOR);
	   }
	   TIM_ClearITPendingBit(TIM1, TIM_IT_CC1);    //清除中断标志位
   }
   
   if(TIM_GetITStatus(TIM1,TIM_IT_CC2)!=RESET){
		
		if(g_eSpdCaptureWorkSwitch[HYDROGEN_FAN_SPD_MONITOR] == DEF_ON){      
		           
           if(g_u16SpdCaptureStatu[HYDROGEN_FAN_SPD_MONITOR] == FRONT_EDGE){//捕获第一个上升沿
               
               g_u16SpdCaptureStatu[HYDROGEN_FAN_SPD_MONITOR] = 1;
               g_SpeedCaptureValueBuff[HYDROGEN_FAN_SPD_MONITOR][FRONT_EDGE] = TIM_GetCapture2(TIM1);//记录第一次上升沿的CNT值
               
           }else if(g_u16SpdCaptureStatu[HYDROGEN_FAN_SPD_MONITOR] == LAST_EDGE){//捕获第二个上升沿
               
               g_u16SpdCaptureStatu[HYDROGEN_FAN_SPD_MONITOR] = 0;
               g_SpeedCaptureValueBuff[HYDROGEN_FAN_SPD_MONITOR][LAST_EDGE] = TIM_GetCapture2(TIM1);//记录第二次上升沿的CNT值
               
               if(g_SpeedCaptureValueBuff[HYDROGEN_FAN_SPD_MONITOR][FRONT_EDGE] < g_SpeedCaptureValueBuff[HYDROGEN_FAN_SPD_MONITOR][LAST_EDGE]){
                    g_u16SpeedCaptureDeltValue[HYDROGEN_FAN_SPD_MONITOR] = g_SpeedCaptureValueBuff[HYDROGEN_FAN_SPD_MONITOR][LAST_EDGE] - g_SpeedCaptureValueBuff[HYDROGEN_FAN_SPD_MONITOR][FRONT_EDGE];           //两次上升沿的差值
               }else if(g_SpeedCaptureValueBuff[HYDROGEN_FAN_SPD_MONITOR][FRONT_EDGE] > g_SpeedCaptureValueBuff[HYDROGEN_FAN_SPD_MONITOR][LAST_EDGE]){
                    g_u16SpeedCaptureDeltValue[HYDROGEN_FAN_SPD_MONITOR] = (2000 - g_SpeedCaptureValueBuff[HYDROGEN_FAN_SPD_MONITOR][FRONT_EDGE]) + g_SpeedCaptureValueBuff[HYDROGEN_FAN_SPD_MONITOR][LAST_EDGE];  //两次上升沿的差值
               }else{
                 g_u16SpeedCaptureDeltValue[HYDROGEN_FAN_SPD_MONITOR]=0;
               }
           }
       }else{
		   ResetSpdCaptureValue(HYDROGEN_FAN_SPD_MONITOR);
	   }
	   TIM_ClearITPendingBit(TIM1, TIM_IT_CC2);    //清除中断标志位
   }
   
   if(TIM_GetITStatus(TIM1,TIM_IT_CC3)!=RESET){
		
		if(g_eSpdCaptureWorkSwitch[STACK_FAN_SPD_MONITOR] == DEF_ON){      
		           
           if(g_u16SpdCaptureStatu[STACK_FAN_SPD_MONITOR] == FRONT_EDGE){//捕获第一个上升沿
               
               g_u16SpdCaptureStatu[STACK_FAN_SPD_MONITOR] = 1;
               g_SpeedCaptureValueBuff[STACK_FAN_SPD_MONITOR][FRONT_EDGE] = TIM_GetCapture3(TIM1);//记录第一次上升沿的CNT值
               
           }else if(g_u16SpdCaptureStatu[STACK_FAN_SPD_MONITOR] == LAST_EDGE){//捕获第二个上升沿
               
               g_u16SpdCaptureStatu[STACK_FAN_SPD_MONITOR] = 0;
               g_SpeedCaptureValueBuff[STACK_FAN_SPD_MONITOR][LAST_EDGE] = TIM_GetCapture3(TIM1);//记录第二次上升沿的CNT值
               
               if(g_SpeedCaptureValueBuff[STACK_FAN_SPD_MONITOR][FRONT_EDGE] < g_SpeedCaptureValueBuff[STACK_FAN_SPD_MONITOR][LAST_EDGE]){
                    g_u16SpeedCaptureDeltValue[STACK_FAN_SPD_MONITOR] = g_SpeedCaptureValueBuff[STACK_FAN_SPD_MONITOR][LAST_EDGE] - g_SpeedCaptureValueBuff[STACK_FAN_SPD_MONITOR][FRONT_EDGE];           //两次上升沿的差值
               }else if(g_SpeedCaptureValueBuff[STACK_FAN_SPD_MONITOR][FRONT_EDGE] > g_SpeedCaptureValueBuff[STACK_FAN_SPD_MONITOR][LAST_EDGE]){
                    g_u16SpeedCaptureDeltValue[STACK_FAN_SPD_MONITOR] = (2000 - g_SpeedCaptureValueBuff[STACK_FAN_SPD_MONITOR][FRONT_EDGE]) + g_SpeedCaptureValueBuff[STACK_FAN_SPD_MONITOR][LAST_EDGE];  //两次上升沿的差值
               }else{
                 g_u16SpeedCaptureDeltValue[STACK_FAN_SPD_MONITOR]=0;
               }
           }
       }else{
		   ResetSpdCaptureValue(STACK_FAN_SPD_MONITOR);
	   }
	   TIM_ClearITPendingBit(TIM1, TIM_IT_CC3);    //清除中断标志位
   }
}

/*
***************************************************************************************************
*                          GetPumpFeedBackSpd()
*
* Description : get the pump feed back speed grade number.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : Application.
*
* Note(s)     : (系统时钟 / 分频系数)  / 周期差值,两脉冲表示转了一圈.
***************************************************************************************************
*/
uint16_t    GetPumpFeedBackSpd(void)
{
    if(g_u16SpeedCaptureDeltValue[PUMP_SPD_MONITOR] > 0){
        g_fSpdCaptureFrequency[PUMP_SPD_MONITOR] = 10000 /(float)(g_u16SpeedCaptureDeltValue[PUMP_SPD_MONITOR] + 1);
    }else{
        g_fSpdCaptureFrequency[PUMP_SPD_MONITOR] = 0;
    }
    return (uint16_t)(g_fSpdCaptureFrequency[PUMP_SPD_MONITOR] * 30);//2脉冲每转 60 * 1/2

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
	if(g_u16SpeedCaptureDeltValue[HYDROGEN_FAN_SPD_MONITOR] > 0){
        g_fSpdCaptureFrequency[HYDROGEN_FAN_SPD_MONITOR] = 1000000 /(float)(g_u16SpeedCaptureDeltValue[HYDROGEN_FAN_SPD_MONITOR] + 1);
    }else{
        g_fSpdCaptureFrequency[HYDROGEN_FAN_SPD_MONITOR] = 0;
    }
    return (uint16_t)(g_fSpdCaptureFrequency[HYDROGEN_FAN_SPD_MONITOR] * 30);
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
	if(g_u16SpeedCaptureDeltValue[STACK_FAN_SPD_MONITOR] > 0){
        g_fSpdCaptureFrequency[STACK_FAN_SPD_MONITOR] = 1000000 /(float)(g_u16SpeedCaptureDeltValue[STACK_FAN_SPD_MONITOR] + 1);
    }else{
        g_fSpdCaptureFrequency[STACK_FAN_SPD_MONITOR] = 0;
    }
    return (uint16_t)(g_fSpdCaptureFrequency[HYDROGEN_FAN_SPD_MONITOR] * 30);
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

/******************* (C) COPYRIGHT 2016 Guangdong ENECO POWER *****END OF FILE****/
