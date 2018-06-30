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
* Filename      : app_dc_module_485_communicate_task.c
* Version       : V1.00
* Programmer(s) : JasonFan
***************************************************************************************************
*/
/*
***************************************************************************************************
*                                             INCLUDE FILES
***************************************************************************************************
*/
#include <includes.h>
#include "app_dc_module_communicate_task.h"
#include "bsp_dc_module_adjust.h"
#include "app_analog_signal_monitor_task.h"
#include "app_stack_short_circuit_task.h"
#include "app_system_run_cfg_parameters.h"
#include "bsp_can.h"
/*
***************************************************************************************************
*                                           MACRO DEFINITIONS
***************************************************************************************************
*/

#define CAN_TX_LEN_OF_SHORT_ACT					16
#define CURRENT_LIMIT_DELAY      				15 //平滑限流延时

#define DC_lIMIT_CURRENT_SMOOTHLY_TASK_SIZE    	200
/*
***************************************************************************************************
*                                         OS-RELATED    VARIABLES
***************************************************************************************************
*/

OS_TCB      DCLimitCurrentSmoothlyTaskTCB ;

static      CPU_STK     DC_LIMIT_CURRENT_SMOOTHLY_TASK_STK[DC_lIMIT_CURRENT_SMOOTHLY_TASK_SIZE];
/*
***************************************************************************************************
*                                           LOCAL VARIABLES
***************************************************************************************************
*/
static  	uint8_t     g_u8DCModuleLimitCurrentSmoothlyTaskSw = DEF_DISABLED;
/*
***************************************************************************************************
*                                           GLOBAL VARIABLES
***************************************************************************************************
*/
float   g_fIvalueMax;
float   g_fVvalueMax;
float   g_fIvalueNow = (float)CURRENT_LIMIT_MIN; //初始值

/*
***************************************************************************************************
*                                         FUNCTION PROTOTYPES
***************************************************************************************************
*/

static void DcModuleLimitCurrentSmoothlyTask(void *p_arg);
/*
***************************************************************************************************
*                                      CurrentSmoothlyLimitTaskCreate()
*
* Description:  Create DC Module Adjust Task .
*
* Arguments  :  none.
*
* Returns    :  none.
*
* Note(s)    :  none.
***************************************************************************************************
*/

void CurrentSmoothlyLimitTaskCreate(void)
{
    OS_ERR  err;
	Bsp_DcModuleConmunicateInit();//485初始化
	//初始化最大V、I限制值
	UpdateVIParaToPrgm();
	
    OSTaskCreate((OS_TCB *)&DCLimitCurrentSmoothlyTaskTCB,
                 (CPU_CHAR *)"DC Limit Current Smoothly Task Create",
                 (OS_TASK_PTR) DcModuleLimitCurrentSmoothlyTask,
                 (void *) 0,
                 (OS_PRIO) DC_MODULE_ADJUST_TASK_PRIO,
                 (CPU_STK *)&DC_LIMIT_CURRENT_SMOOTHLY_TASK_STK[0],
                 (CPU_STK_SIZE) DC_lIMIT_CURRENT_SMOOTHLY_TASK_SIZE / 10,
                 (CPU_STK_SIZE) DC_lIMIT_CURRENT_SMOOTHLY_TASK_SIZE,
                 (OS_MSG_QTY) 5u,
                 (OS_TICK) 0u,
                 (void *) 0,
                 (OS_OPT)(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR *)&err);
    APP_TRACE_INFO(("Created DC Limit Current Smoothly Task, and err code is %d...\n\r", err));
}

/*
***************************************************************************************************
*                                      UpdateVIParaToPrgm()
*
* Description:  Uptade the Max voletage and current value by flash data .
*
* Arguments  :  none.
*
* Returns    :  none.
*
* Note(s)    :  none.
***************************************************************************************************
*/
void UpdateVIParaToPrgm(void)
{
	g_fIvalueMax = (float)g_stMaxVILimitPara.current_max / 100;
	g_fVvalueMax = (float)g_stMaxVILimitPara.voltage_max / 100;
}
/*
***************************************************************************************************
*                                   DcModuleLimitCurrentSmoothlyTask()
*
* Description:  DC Module limit current smoothly task.
*
* Arguments  :  none.
*
* Returns    :  none.
*
* Note(s)    :  none.
***************************************************************************************************
*/
static void DcModuleLimitCurrentSmoothlyTask(void *p_arg)
{
    OS_ERR  err;
    static uint8_t u8CurrentLimitDelayCount = 0;

		
	while(DEF_TRUE) {
		
		APP_TRACE_INFO(("Smoothly current limit suspend...\n\r"));
		OSTaskSuspend(NULL, &err);
		APP_TRACE_INFO(("Resume Limit Current Smoothly Task...\n\r"));
		SetDcOutPutCurrentLimitPoint(HOLD_LIMIT_POINT);

		while(DEF_TRUE) {

			if(g_u8DCModuleLimitCurrentSmoothlyTaskSw == DEF_DISABLED) {
				u8CurrentLimitDelayCount = 0;	
				APP_TRACE_INFO(("Limit Current Smoothly Task break ...\n\r"));
				break;
			}

			OSTimeDlyHMSM(0, 0, 1, 0, OS_OPT_TIME_HMSM_STRICT, &err);

			u8CurrentLimitDelayCount ++;
			if(u8CurrentLimitDelayCount >= CURRENT_LIMIT_DELAY) {
				if((g_fIvalueNow < g_fIvalueMax)) {
					g_fIvalueNow += 5.0;

					if(g_fIvalueNow >= g_fIvalueMax) {
						g_fIvalueNow = g_fIvalueMax;
					}

					SetDcOutPutCurrentLimitPoint(g_fIvalueNow);
					APP_TRACE_INFO(("Smoothly current limit point increase,the IvalueNow is %.2f ...\n\r", g_fIvalueNow));

					if(g_fIvalueNow >= g_fIvalueMax) {
						
#ifdef  STACK_SHORT_CTRL_EN
						OSTaskResume(&StackRunningShortTaskTCB, &err);//恢复短路活化任务
#endif
						u8CurrentLimitDelayCount = 0;
						SetRestartLimitCurrentFlagStatus(DEF_CLR);//清重新限流标志
						APP_TRACE_INFO(("Smoothly current limit finished and break...\n\r"));
						break;//平滑限流完成
					}
				}
				u8CurrentLimitDelayCount = 0;
			}
		}
	}
}

/*
***************************************************************************************************
*                            HoldSlaveDCOutCurrentCmdSendThroughCAN()
*
* Description:  Enable or Disable the dc module limit current task switch.
*
* Arguments  :  none
*
* Returns    :  none
***************************************************************************************************
*/
void HoldSlaveDCOutCurrentCmdSendThroughCAN(void)
{	
	uint8_t TxBuf[CAN_TX_LEN_OF_SHORT_ACT] = {0xFC,0xFD,0xFE,0x20,g_u16GlobalNetWorkId,0x04,0x05,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xAA};
												/*		 							请求code  保持  */
	SendCanMsgContainNodeId(CAN_TX_LEN_OF_SHORT_ACT, TxBuf, g_u16GlobalNetWorkId);
	APP_TRACE_INFO(("Send Cmd to hold slave dc out current limit point...\n\r"));
}

void ResumeSlaveDCOutCurrentCmdSendThroughCAN(void)
{	
	uint8_t TxBuf[CAN_TX_LEN_OF_SHORT_ACT] = {0xFC,0xFD,0xFE,0x20,g_u16GlobalNetWorkId,0x04,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xAA};
												/*		 							请求code  恢复  */
	SendCanMsgContainNodeId(CAN_TX_LEN_OF_SHORT_ACT,TxBuf,g_u16GlobalNetWorkId);
	APP_TRACE_INFO(("Send Cmd to resume slave dc out max current limit point...\n\r"));
}
/*
***************************************************************************************************
*                            SetDCModuleLimitCurrentSmoothlyTaskSwitch()
*
* Description:  Enable or Disable the dc module limit current task switch.
*
* Arguments  :  none
*
* Returns    :  none
***************************************************************************************************
*/
void SetDCModuleLimitCurrentSmoothlyTaskSwitch(uint8_t i_NewStatu)
{
    g_u8DCModuleLimitCurrentSmoothlyTaskSw = i_NewStatu;
}

/*
***************************************************************************************************
*                            IncCurrentLimitPoint()
*
* Description:  Increase the current limit point.
*
* Arguments  :  none
*
* Returns    :  g_fIvalueNow:The current limit point this time.
***************************************************************************************************
*/
float IncCurrentLimitPoint(void)
{
	if((g_fIvalueNow < g_fIvalueMax)) {
		g_fIvalueNow += 10.0;

		if(g_fIvalueNow >= g_fIvalueMax) {
			g_fIvalueNow = g_fIvalueMax;
		}
	}
	
	return g_fIvalueNow;
}


float DecCurrentLimitPoint(void)
{
	if((g_fIvalueNow > CURRENT_LIMIT_MIN)) {
		g_fIvalueNow -= 10.0;

		if(g_fIvalueNow <= CURRENT_LIMIT_MIN) {
			g_fIvalueNow = CURRENT_LIMIT_MIN;
		}
	}
	
	return g_fIvalueNow;
}
/******************* (C) COPYRIGHT 2016 Guangdong ENECO POWER *****END OF FILE****/
