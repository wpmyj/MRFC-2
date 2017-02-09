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
#include "app_stack_short_circuit_task.h"
/*
***************************************************************************************************
*                                           MACRO DEFINITIONS
***************************************************************************************************
*/
#define REPORT_ADRESS       0x07  //上报地址信息

#define CURRENT_LIMIT_DELAY      30 //平滑限流延时

#define DC_lIMIT_CURRENT_SMOOTHLY_TASK_SIZE    100
#define DC_MODULE_ADJUST_TASK_SIZE             100
/*
***************************************************************************************************
*                                         OS-RELATED    VARIABLES
***************************************************************************************************
*/

OS_TCB      DCLimitCurrentSmoothlyTaskTCB ;
OS_TCB      DCModuleDynamicAdjustTaskTCB ;

static      CPU_STK     DC_LIMIT_CURRENT_SMOOTHLY_TASK_STK[DC_lIMIT_CURRENT_SMOOTHLY_TASK_SIZE];
static      CPU_STK     DC_MODULE_ADJUST_TASK_STK[DC_MODULE_ADJUST_TASK_SIZE];
/*
***************************************************************************************************
*                                           LOCAL VARIABLES
***************************************************************************************************
*/
static  uint8_t             g_u8DCModuleCurrentLimitingPointImproveFlag = DEF_CLR;
static  uint8_t             g_u8DCModuleCurrentLimitingPointReduceFlag = DEF_CLR;
static  uint8_t             g_u8DCModuleDynamicAdjustTaskSw = DEF_DISABLED;
static  uint8_t             g_u8DCModuleLimitCurrentSmoothlyTaskSw = DEF_DISABLED;

/*
***************************************************************************************************
*                                           GLOBAL VARIABLES
***************************************************************************************************
*/
float   fIvalueNow = (float)CURRENT_LIMIT_MIN;
float   fVvalueNow = (float)VOLTAGE_LIMIT_MAX;
/*
***************************************************************************************************
*                                         FUNCTION PROTOTYPES
***************************************************************************************************
*/

static void DcModuleDynamicAdjustTask(void *p_arg);
static void DcModuleLimitCurrentSmoothlyTask(void *p_arg);
/*
***************************************************************************************************
*                                      DcModuleDynamicAdjustTaskCreate()
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
void DcModuleDynamicAdjustTaskCreate(void)
{
    OS_ERR  err;
    uint8_t u8RxLength = 0;
    uint16_t u16Rs485RxBuf[2] = {0, 0};
    
    Bsp_DcModuleConmunicateInit();//485初始化
    //DC动态限流任务
    OSTaskCreate((OS_TCB *)&DCModuleDynamicAdjustTaskTCB,                    // Create the start task
                 (CPU_CHAR *)"DC Module Adjust Task Create",
                 (OS_TASK_PTR) DcModuleDynamicAdjustTask,
                 (void *) 0,
                 (OS_PRIO) DC_MODULE_ADJUST_TASK_PRIO,
                 (CPU_STK *)&DC_MODULE_ADJUST_TASK_STK[0],
                 (CPU_STK_SIZE) DC_MODULE_ADJUST_TASK_SIZE / 10,
                 (CPU_STK_SIZE) DC_MODULE_ADJUST_TASK_SIZE,
                 (OS_MSG_QTY) 5u,
                 (OS_TICK) 0u,
                 (void *) 0,
                 (OS_OPT)(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR *)&err);
    APP_TRACE_INFO(("Created DC Module Adjust Task, and err code is %d...\n\r", err));
                 
    //DC平滑限流任务
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
                 
    //485通信成功验证
    Bsp_SendAddressByDifferentCmdType(TRANSPOND_COMMAND);
    Bsp_RS485_Receive_Data(u16Rs485RxBuf, &u8RxLength);

    if(u16Rs485RxBuf[0] == REPORT_ADRESS) { //上报地址验证成功
        APP_TRACE_INFO(("Rs485 communicate successed...\n\r"));
        Bsp_SendAddressByDifferentCmdType(TRANSPOND_COMMAND);
        Bsp_SetDcModuleOutPutVIvalue(VOLTAGE_LIMIT_MAX, CURRENT_LIMIT_MIN); //上电预设值，电流设置为最小值
    } else {
        APP_TRACE_INFO(("Rs485 communicate failed...\n\r"));
    }

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
* Note(s)    :  重新开始限流的平滑限流任务.
***************************************************************************************************
*/
static void DcModuleLimitCurrentSmoothlyTask(void *p_arg)
{
    OS_ERR  err;
    float fStackVoltage = 0.0;
    static uint8_t u8CurrentLimitDelayCount = 0;
    static uint8_t u8CurrentLimitHoldCount = 0;        

    while(DEF_TRUE) { 
        
        OSTaskSuspend(NULL, &err);
        APP_TRACE_INFO(("Resume Limit Current Smoothly Task...\n\r"));
        
        fIvalueNow = 0.0;//重新开始限流   
        Bsp_SendAddressByDifferentCmdType(TRANSPOND_COMMAND);
        Bsp_SetDcModuleOutPutVIvalue(fVvalueNow, fIvalueNow);
        
        while(DEF_TRUE) {
                
            if(g_u8DCModuleLimitCurrentSmoothlyTaskSw == DEF_DISABLED){
                
                APP_TRACE_INFO(("Limit Current Smoothly Task break ...\n\r"));
                break;
            }
            
            OSTimeDlyHMSM(0, 0, 1, 0,OS_OPT_TIME_HMSM_STRICT,&err);
            
            fStackVoltage = GetSrcAnaSig(STACK_VOLTAGE);
            u8CurrentLimitDelayCount ++;
            if(u8CurrentLimitDelayCount >= CURRENT_LIMIT_DELAY){
                if(fIvalueNow <= CURRENT_LIMIT_MAX && fStackVoltage >= 41.0){
                    fIvalueNow += 5.0;
                    Bsp_SendAddressByDifferentCmdType(TRANSPOND_COMMAND);
                    Bsp_SetDcModuleOutPutVIvalue(fVvalueNow, fIvalueNow);
                    APP_TRACE_INFO(("Smoothly current limit point increase,the IvalueNow is %.2f ...\n\r", fIvalueNow));
                }else{
                    
                    APP_TRACE_INFO(("Smoothly current limit finished ...\n\r")); 
//                    OSTaskResume(&StackShortCircuitTaskTCB,&err);//恢复电堆短路活化任务
                    SetDCModuleAutoAdjustTaskSwitch(DEF_ENABLED);   //开动态限流开关
                    OSTaskResume(&DCModuleDynamicAdjustTaskTCB,&err);//恢复动态限流任务
                    break;//平滑限流完成  
                }
                u8CurrentLimitDelayCount = 0;
            }else{
                u8CurrentLimitHoldCount ++;
                if(u8CurrentLimitHoldCount >= 10){//保证限流不中断，隔10s发送一次
                    Bsp_SendAddressByDifferentCmdType(TRANSPOND_COMMAND);
                    Bsp_SetDcModuleOutPutVIvalue(fVvalueNow, fIvalueNow);
                    u8CurrentLimitHoldCount = 0;
                    APP_TRACE_INFO(("Smoothly current limit point stay the same ,the IvalueNow is %.2f ...\n\r", fIvalueNow));
                }
            }
                
        }
    }
}
/*
***************************************************************************************************
*                                   DcModuleDynamicAdjustTask()
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

static void DcModuleDynamicAdjustTask(void *p_arg)
{
    OS_ERR  err;

    OSTaskSuspend(NULL, &err);
    APP_TRACE_INFO(("Resume dynamic current limit task ...\n\r"));
    
    while(DEF_TRUE) {
        
        if(g_u8DCModuleDynamicAdjustTaskSw == DEF_DISABLED){
            APP_TRACE_INFO(("Dynamic current limit task break ...\n\r"));
            break;
        }
                  
        OSTaskSemPend(OS_CFG_TICK_RATE_HZ * 8,//隔8s请求匹氢偏移监测任务的限流调节任务信号量
                      OS_OPT_PEND_BLOCKING,
                      NULL,
                      &err);

        if(OS_ERR_NONE == err) {
            if(DEF_SET == g_u8DCModuleCurrentLimitingPointImproveFlag) {
                if(fIvalueNow < CURRENT_LIMIT_MAX) { //提高限流点
                    fIvalueNow += 2.0;
                    Bsp_SendAddressByDifferentCmdType(TRANSPOND_COMMAND);
                    Bsp_SetDcModuleOutPutVIvalue(fVvalueNow, fIvalueNow);
                    SetDCModuleCurrentLimitingPointImproveFlag(DEF_CLR);
                    APP_TRACE_INFO(("DC dynamic current limit point increase,the IvalueNow is %.2f ...\n\r", fIvalueNow));
                }
            } else if(DEF_SET == g_u8DCModuleCurrentLimitingPointReduceFlag) {
                if(fIvalueNow >= CURRENT_LIMIT_MIN) { //降低限流点
                    fIvalueNow -= 2.0;
                    Bsp_SendAddressByDifferentCmdType(TRANSPOND_COMMAND);
                    Bsp_SetDcModuleOutPutVIvalue(fVvalueNow, fIvalueNow);
                    SetDcModuleCurrentLimitingPointReduceFlag(DEF_CLR);
                    APP_TRACE_INFO(("DC dynamic current limit point decrease,the IvalueNow is %.2f ...\n\r", fIvalueNow));
                }
            } else {}
                
        } else if(OS_ERR_TIMEOUT== err) { //超时说明不需要调节，保持原来设置
            
            Bsp_SendAddressByDifferentCmdType(TRANSPOND_COMMAND);
            Bsp_SetDcModuleOutPutVIvalue(fVvalueNow, fIvalueNow);
            APP_TRACE_INFO(("DC dynamic current limit point keep the same,the IvalueNow is %.2f...\n\r",fIvalueNow));            
        } else {
            APP_TRACE_INFO(("DC dynamic Task Sem Pend err code is %d...\n\r", err));
        }
    }
}

/*
*********************************************************************************************************
*                           SetDCModuleCurrentLimitingPointImproveFlag()
*
* Description:  DC限流点提高、降低标志相关函数
*
* Arguments  :  none.
*
* Returns    :  none.
*
* Note(s)    :  none.
*********************************************************************************************************
*/
void SetDCModuleCurrentLimitingPointImproveFlag(uint8_t i_NewStatu)
{
    g_u8DCModuleCurrentLimitingPointImproveFlag = i_NewStatu;
}

uint8_t GetDcModuleCurrentLimitingPointImproveFlagStatus(void)
{
    return g_u8DCModuleCurrentLimitingPointImproveFlag;
}

void SetDcModuleCurrentLimitingPointReduceFlag(uint8_t i_NewStatu)
{
    g_u8DCModuleCurrentLimitingPointReduceFlag = i_NewStatu;
}

uint8_t GetDcModuleCurrentLimitingPointReduceFlagStatus(void)
{
    return g_u8DCModuleCurrentLimitingPointReduceFlag;
}

/*
***************************************************************************************************
*                            SetDCModuleAutoAdjustTaskSwitch()
*
* Description:  Enable or Disable the hua wei auto adjust task switch.
*
* Arguments  :  none
*
* Returns    :  none
***************************************************************************************************
*/
void SetDCModuleAutoAdjustTaskSwitch(uint8_t i_NewStatu)
{
    g_u8DCModuleDynamicAdjustTaskSw = i_NewStatu;
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
