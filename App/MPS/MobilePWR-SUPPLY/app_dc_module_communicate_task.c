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
* Programmer(s) : FanJun
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
/*
***************************************************************************************************
*                                           MACRO DEFINITIONS
***************************************************************************************************
*/
#define HUA_WEI_MODULE_ADJUST_TASK_SIZE 200
/*
***************************************************************************************************
*                                         OS-RELATED    VARIABLES
***************************************************************************************************
*/
OS_TCB      DcModuleAdjustTaskTCB ;
static      CPU_STK     HUA_WEI_MODULE_ADJUST_TASK_STK[HUA_WEI_MODULE_ADJUST_TASK_SIZE];
/*
***************************************************************************************************
*                                           LOCAL VARIABLES
***************************************************************************************************
*/
static  uint8_t             g_u8HuaWeiModuleCurrentLimitingPointImproveFlag = DEF_CLR;
static  uint8_t             g_u8HuaWeiModuleCurrentLimitingPointReduceFlag = DEF_CLR;
static  uint8_t             g_u8HuaWeiModuleAutoAdjustTaskSw = DEF_DISABLED;
/*
***************************************************************************************************
*                                         FUNCTION PROTOTYPES
***************************************************************************************************
*/

static void HuaWeiModuleAdjustTask(void *p_arg);

/*
***************************************************************************************************
*                                      DcModuleAdjustTaskCreate()
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
void DcModuleAdjustTaskCreate(void)
{
    OS_ERR  err;
    uint8_t i = 0;
    uint8_t u8RxLength = 0;
    uint16_t u16Rs485RxBuf[2] = {0, 0};
    Bsp_DcModuleConmunicateInit();//485初始化
    OSTaskCreate((OS_TCB *)&DcModuleAdjustTaskTCB,                    // Create the start task
                 (CPU_CHAR *)"DC Module Adjust Task Create",
                 (OS_TASK_PTR) HuaWeiModuleAdjustTask,
                 (void *) 0,
                 (OS_PRIO) DC_MODULE_ADJUST_TASK_PRIO,
                 (CPU_STK *)&HUA_WEI_MODULE_ADJUST_TASK_STK[0],
                 (CPU_STK_SIZE) HUA_WEI_MODULE_ADJUST_TASK_SIZE / 10,
                 (CPU_STK_SIZE) HUA_WEI_MODULE_ADJUST_TASK_SIZE,
                 (OS_MSG_QTY) 5u,
                 (OS_TICK) 0u,
                 (void *) 0,
                 (OS_OPT)(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR *)&err);
    APP_TRACE_INFO(("Created DC Module Adjust Task, and err code is %d...\n\r", err));

    //485通信成功验证
    Bsp_SendAddressByDifferentCmdType(TRANSPOND_COMMAND);
    Bsp_RS485_Receive_Data(u16Rs485RxBuf, &u8RxLength);

    if(u16Rs485RxBuf[0] == 0x07) { //返回地址验证成功
        APP_TRACE_INFO(("Rs485 communicate successed...\n\r"));
        Bsp_SetDcModuleOutPutVIvalue(VOLTAGE_LIMIT_MAX, CURRENT_LIMIT_MIN); //上电预设值，电流设置为最小值
    } else {
        APP_TRACE_INFO(("Rs485 communicate failed...\n\r"));
    }

}


/*
***************************************************************************************************
*                                   HuaWeiModuleAdjustTask()
*
* Description:  DC Module adjust Voltage and current task.
*
* Arguments  :  none.
*
* Returns    :  none.
*
* Note(s)    :  none.
***************************************************************************************************
*/

static void HuaWeiModuleAdjustTask(void *p_arg)
{
    OS_ERR  err;
    static float   fIvalueNow = (float)CURRENT_LIMIT_MIN;
    static float   fVvalueNow = (float)VOLTAGE_LIMIT_MAX;

    OSTaskSuspend(NULL, &err);

    while(DEF_TRUE) {
        OSTaskSemPend(OS_CFG_TICK_RATE_HZ * 8,//隔8s请求匹氢偏移监测任务的限流调节任务信号量
                      OS_OPT_PEND_BLOCKING,
                      NULL,
                      &err);

        if(OS_ERR_NONE == err) {
            if(DEF_SET == g_u8HuaWeiModuleCurrentLimitingPointImproveFlag) {
                if(fIvalueNow < CURRENT_LIMIT_MAX) { //提高限流点
                    fIvalueNow += 2.0;
                    Bsp_SendAddressByDifferentCmdType(TRANSPOND_COMMAND);
                    Bsp_SetDcModuleOutPutVIvalue(fVvalueNow, fIvalueNow);
                    SetDcModuleCurrentLimitingPointImproveFlag(DEF_CLR);
                    APP_TRACE_INFO(("DC outPut current limit point increase,the IvalueNow is %.2f ...\n\r", fIvalueNow));
                }
            } else if(DEF_SET == g_u8HuaWeiModuleCurrentLimitingPointReduceFlag) {
                if(fIvalueNow >= CURRENT_LIMIT_MIN) { //降低限流点
                    fIvalueNow -= 2.0;
                    Bsp_SendAddressByDifferentCmdType(TRANSPOND_COMMAND);
                    Bsp_SetDcModuleOutPutVIvalue(fVvalueNow, fIvalueNow);
                    SetDcModuleCurrentLimitingPointReduceFlag(DEF_CLR);
                    APP_TRACE_INFO(("DC outPut current limit point decrease,the IvalueNow is %.2f ...\n\r", fIvalueNow));
                }
            } else {}
        } else if(OS_ERR_TIMEOUT) { //超时说明不需要调节，保持原来设置,
            
            if(DEF_SET == GetStackNeedRestartLimitCurrentFlag()){
                fIvalueNow = (float)CURRENT_LIMIT_MIN;
                APP_TRACE_INFO(("Restart current limit,the IvalueNow is %.2f ...\n\r", fIvalueNow));
            }else{
                APP_TRACE_INFO(("Current limit stay the same,the IvalueNow is %.2f ...\n\r", fIvalueNow));
            }
            Bsp_SendAddressByDifferentCmdType(TRANSPOND_COMMAND);
            Bsp_SetDcModuleOutPutVIvalue(fVvalueNow, fIvalueNow);            
        } else {
            APP_TRACE_INFO(("DC module Task Sem Pend err code is %d...\n\r", err));
        }
        
        if(g_u8HuaWeiModuleAutoAdjustTaskSw == DEF_DISABLED){
            break;
        }
    }
}

/*
*********************************************************************************************************
*                           SetHuaWeiModuleImproveCurrentLimitingPointFlag()
*
* Description:  华为限流点提高、降低标志相关函数
*
* Arguments  :  none.
*
* Returns    :  none.
*
* Note(s)    :  none.
*********************************************************************************************************
*/
void SetDcModuleCurrentLimitingPointImproveFlag(uint8_t i_NewStatu)
{
    g_u8HuaWeiModuleCurrentLimitingPointImproveFlag = i_NewStatu;
}

uint8_t GetDcModuleCurrentLimitingPointImproveFlagStatus(void)
{
    return g_u8HuaWeiModuleCurrentLimitingPointImproveFlag;
}

void SetDcModuleCurrentLimitingPointReduceFlag(uint8_t i_NewStatu)
{
    g_u8HuaWeiModuleCurrentLimitingPointReduceFlag = i_NewStatu;
}

uint8_t GetDcModuleCurrentLimitingPointReduceFlagStatus(void)
{
    return g_u8HuaWeiModuleCurrentLimitingPointReduceFlag;
}

/*
***************************************************************************************************
*                            SetHuaWeiModuleAutoAdjustTaskSwitch()
*
* Description:  Enable or Disable the hua wei auto adjust task switch.
*
* Arguments  :  none
*
* Returns    :  none
***************************************************************************************************
*/
void SetHuaWeiModuleAutoAdjustTaskSwitch(uint8_t i_NewStatu)
{
    g_u8HuaWeiModuleAutoAdjustTaskSw = i_NewStatu;
}
