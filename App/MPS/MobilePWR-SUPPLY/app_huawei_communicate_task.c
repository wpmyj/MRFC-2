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
* Filename      : app_huawei_communicate_task.c
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
#include "app_huawei_communicate_task.h"
#include "bsp_huawei_485_adjust.h"
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
            OS_TCB      HuaWeiModuleAdjustTaskTCB ;
static      CPU_STK     HUA_WEI_MODULE_ADJUST_TASK_STK[HUA_WEI_MODULE_ADJUST_TASK_SIZE];
/*
***************************************************************************************************
*                                           LOCAL VARIABLES
***************************************************************************************************
*/
static  uint8_t             g_u8HuaWeiModuleCurrentLimitingPointImproveFlag = DEF_CLR;
static  uint8_t             g_u8HuaWeiModuleCurrentLimitingPointReduceFlag = DEF_CLR;

/*
***************************************************************************************************
*                                         FUNCTION PROTOTYPES
***************************************************************************************************
*/

static void HuaWeiModuleAdjustTask(void *p_arg);


/*
*********************************************************************************************************
*                                      HuaWeiModuleAdjustTaskCreate()
*
* Description:  Create Hua Wei Module Adjust Task .
*
* Arguments  :  none.
*
* Returns    :  none.
*
* Note(s)    :  none.
*********************************************************************************************************
*/
void HuaWeiModuleAdjustTaskCreate(void)
{
    OS_ERR  err;
    uint8_t i=0;
    uint8_t u8RxLength = 0;
    uint16_t u16Rs485RxBuf[2] = {0,0};
    Bsp_HuaWeiDCConmunicateInit();//485��ʼ��   
    OSTaskCreate((OS_TCB *)&HuaWeiModuleAdjustTaskTCB,                    // Create the start task
                 (CPU_CHAR *)"Hua Wei Module Adjust Task Create",
                 (OS_TASK_PTR) HuaWeiModuleAdjustTask,
                 (void *) 0,
                 (OS_PRIO) HUA_WEI_MODULE_ADJUST_TASK_PRIO,
                 (CPU_STK *)&HUA_WEI_MODULE_ADJUST_TASK_STK[0],
                 (CPU_STK_SIZE) HUA_WEI_MODULE_ADJUST_TASK_SIZE / 10,
                 (CPU_STK_SIZE) HUA_WEI_MODULE_ADJUST_TASK_SIZE,
                 (OS_MSG_QTY) 5u,
                 (OS_TICK) 0u,
                 (void *) 0,
                 (OS_OPT)(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR *)&err);
    APP_TRACE_INFO(("Created Hua Wei Module Adjust Task, and err code is %d...\n\r", err));
    
    //485ͨ�ųɹ���֤
    Bsp_SendAddressByDifferentCmdType(TRANSPOND_COMMAND);
    Bsp_RS485_Receive_Data(u16Rs485RxBuf,&u8RxLength);                
    if(u16Rs485RxBuf[0] == 0x07){//���ص�ַ��֤�ɹ�
        APP_TRACE_INFO(("Rs485 communicate successed...\n\r"));
        Bsp_SetHWmoduleOutPutVIvalue(VOLTAGE_LIMIT_MAX,CURRENT_LIMIT_MIN);//�ϵ�Ԥ��ֵ����������Ϊ��Сֵ
    } else {
        APP_TRACE_INFO(("Rs485 communicate failed...\n\r"));
    }
                
}


/*
*********************************************************************************************************
*                                   HuaWeiModuleAdjustTask()
*
* Description:  Hua Wei Module adjust Voltage and current task.
*
* Arguments  :  none.
*
* Returns    :  none.
*
* Note(s)    :  none.
*********************************************************************************************************
*/

static void HuaWeiModuleAdjustTask(void *p_arg)
{
    OS_ERR  err;
    static float   fIvalueNow = CURRENT_LIMIT_MIN;
    static float   fVvalueNow = VOLTAGE_LIMIT_MAX;

    OSTaskSuspend(NULL, &err);
    APP_TRACE_INFO(("Resume hua wei module adjust task...\n\r"));   
    
    while(DEF_TRUE)
    {
        OSTaskSemPend(OS_CFG_TICK_RATE_HZ * 10,//��10s����ƥ��ƫ�Ƽ��������������������ź���
                      OS_OPT_PEND_BLOCKING,
                      NULL,
                      &err);
        if(OS_ERR_NONE == err){
            if(DEF_SET == GetHuaWeiModuleCurrentLimitingPointImproveFlagStatus()){
                if(fIvalueNow < CURRENT_LIMIT_MAX){//���������
                    fIvalueNow += 1.0;
                    Bsp_SendAddressByDifferentCmdType(TRANSPOND_COMMAND);
                    Bsp_SetHWmoduleOutPutVIvalue(fVvalueNow,fIvalueNow);
                    SetHuaWeiModuleCurrentLimitingPointImproveFlag(DEF_CLR);
                }
            }
            else if(DEF_SET == GetHuaWeiModuleCurrentLimitingPointImproveFlagStatus()){
                if(fIvalueNow >= CURRENT_LIMIT_MIN){//����������
                    fIvalueNow -= 1.0;
                    Bsp_SendAddressByDifferentCmdType(TRANSPOND_COMMAND);
                    Bsp_SetHWmoduleOutPutVIvalue(fVvalueNow,fIvalueNow);
                    SetHuaWeiModuleCurrentLimitingPointReduceFlag(DEF_CLR);
                }
            } else {}
        } else if(OS_ERR_TIMEOUT){//��ʱ˵������Ҫ���ڣ�����ԭ������
            Bsp_SendAddressByDifferentCmdType(TRANSPOND_COMMAND);
            Bsp_SetHWmoduleOutPutVIvalue(fVvalueNow,fIvalueNow);
            APP_TRACE_INFO(("Hua Wei module OutPut VIvalue stay the same...\n\r"));
        } else {
            APP_TRACE_INFO(("OS Task Sem Pend err code is %d...\n\r",err));   
        }
     }   
}

/*
*********************************************************************************************************
*                           SetHuaWeiModuleImproveCurrentLimitingPointFlag()
*
* Description:  ��Ϊ��������ߡ����ͱ�־��غ���
*
* Arguments  :  none.
*
* Returns    :  none.
*
* Note(s)    :  none.
*********************************************************************************************************
*/
void SetHuaWeiModuleCurrentLimitingPointImproveFlag(uint8_t i_NewStatu)
{
    g_u8HuaWeiModuleCurrentLimitingPointImproveFlag = i_NewStatu;
}

uint8_t GetHuaWeiModuleCurrentLimitingPointImproveFlagStatus(void)
{
    return g_u8HuaWeiModuleCurrentLimitingPointImproveFlag;
}

void SetHuaWeiModuleCurrentLimitingPointReduceFlag(uint8_t i_NewStatu)
{
    g_u8HuaWeiModuleCurrentLimitingPointReduceFlag = i_NewStatu;
}

uint8_t GetHuaWeiModuleCurrentLimitingPointReduceFlagStatus(void)
{
    return g_u8HuaWeiModuleCurrentLimitingPointReduceFlag;
}

