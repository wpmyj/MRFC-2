/*
************************************************************************************************************************
*                                              APPLICATION CODE
*
*                          (c) Copyright 2015; Guangdong Hydrogen Energy Science And Technology Co.,Ltd
*
*               All rights reserved.  Protected by international copyright laws.
*               Knowledge of the source code may NOT be used without authorization.
*
***************************************************************************************************
*/
/*
***************************************************************************************************
* Filename      : os_app_hooks.c
* Version       : V1.00
* Programmer(s) : EHS
************************************************************************************************************************
*/
/*
***************************************************************************************************
*                                             INCLUDE FILES
***************************************************************************************************
*/
#include <stdint.h>
#include <os.h>
#include <os_app_hooks.h>
#include <app_cfg.h>
#include <bsp_ser.h>
#include <os_cfg_app.h>
#include <includes.h>

/*$PAGE*/
/*
************************************************************************************************************************
*                                              SET ALL APPLICATION HOOKS
*
* Description: Set ALL application hooks.
*              设置与应用相关的介入函数
* Arguments  : none.
*
* Note(s)    : none
*
*   Description:    user task context switching
************************************************************************************************************************
*/

void  App_OS_SetAllHooks(void)
{
#if OS_CFG_APP_HOOKS_EN > 0u
    CPU_SR_ALLOC();

    CPU_CRITICAL_ENTER();
    OS_AppTaskCreateHookPtr = App_OS_TaskCreateHook;
    OS_AppTaskDelHookPtr    = App_OS_TaskDelHook;
    OS_AppTaskReturnHookPtr = App_OS_TaskReturnHook;

    OS_AppIdleTaskHookPtr   = App_OS_IdleTaskHook;      //空闲任务调用的介入函数
    OS_AppStatTaskHookPtr   = App_OS_StatTaskHook;      //统计任务调用的介入函数
    OS_AppTaskSwHookPtr     = App_OS_TaskSwHook;        //任务切换所调用的介入函数
    OS_AppTimeTickHookPtr   = App_OS_TimeTickHook;      //时基任务调用的介入函数

    CPU_CRITICAL_EXIT();
#endif
}

/*$PAGE*/
/*
************************************************************************************************************************
*                                             CLEAR ALL APPLICATION HOOKS
*
* Description: Clear ALL application hooks.
*
* Arguments  : none.
*
* Note(s)    : none
************************************************************************************************************************
*/

void  App_OS_ClrAllHooks(void)
{
#if OS_CFG_APP_HOOKS_EN > 0u
    CPU_SR_ALLOC();

    CPU_CRITICAL_ENTER();
    OS_AppTaskCreateHookPtr = (OS_APP_HOOK_TCB)0;
    OS_AppTaskDelHookPtr    = (OS_APP_HOOK_TCB)0;
    OS_AppTaskReturnHookPtr = (OS_APP_HOOK_TCB)0;

    OS_AppIdleTaskHookPtr   = (OS_APP_HOOK_VOID)0;
    OS_AppStatTaskHookPtr   = (OS_APP_HOOK_VOID)0;
    OS_AppTaskSwHookPtr     = (OS_APP_HOOK_VOID)0;
    OS_AppTimeTickHookPtr   = (OS_APP_HOOK_VOID)0;
    CPU_CRITICAL_EXIT();
#endif
}

/*$PAGE*/
/*
************************************************************************************************************************
*                                            APPLICATION TASK CREATION HOOK
*
* Description: This function is called when a task is created.
*
* Arguments  : p_tcb   is a pointer to the task control block of the task being created.
*
* Note(s)    : none
************************************************************************************************************************
*/

void  App_OS_TaskCreateHook(OS_TCB  *p_tcb)
{
    (void)&p_tcb;
}

/*$PAGE*/
/*
************************************************************************************************************************
*                                            APPLICATION TASK DELETION HOOK
*
* Description: This function is called when a task is deleted.
*
* Arguments  : p_tcb   is a pointer to the task control block of the task being deleted.
*
* Note(s)    : none
************************************************************************************************************************
*/

void  App_OS_TaskDelHook(OS_TCB  *p_tcb)
{
    (void)&p_tcb;
}

/*$PAGE*/
/*
************************************************************************************************************************
*                                             APPLICATION TASK RETURN HOOK
*
* Description: This function is called if a task accidentally returns.  In other words, a task should either be an
*              infinite loop or delete itself when done.
*
* Arguments  : p_tcb     is a pointer to the OS_TCB of the task that is returning.
*
* Note(s)    : none
************************************************************************************************************************
*/

void  App_OS_TaskReturnHook(OS_TCB  *p_tcb)
{
    (void)&p_tcb;
}

/*$PAGE*/
/*
************************************************************************************************************************
*                                              APPLICATION IDLE TASK HOOK
*
* Description: This function is called by the idle task.  This hook has been added to allow you to do such things as
*              STOP the CPU to conserve power.
*
* Arguments  : none
*
* Note(s)    : none
************************************************************************************************************************
*/

void  App_OS_IdleTaskHook(void)
{

}

/*$PAGE*/
/*
************************************************************************************************************************
*                                          APPLICATION OS INITIALIZATION HOOK
*
* Description: This function is called by OSInit() at the beginning of OSInit().
*
* Arguments  : none
*
* Note(s)    : none
************************************************************************************************************************
*/

void  App_OS_InitHook(void)
{

}

/*$PAGE*/
/*
************************************************************************************************************************
*                                           APPLICATION STATISTIC TASK HOOK
*
* Description: This function is called every second by uC/OS-III's statistics task.  This allows your application to add
*              functionality to the statistics task.
*
* Arguments  : none
*
* Note(s)    : none
************************************************************************************************************************
*/

void  App_OS_StatTaskHook(void)
{

}

/*$PAGE*/
/*
************************************************************************************************************************
*                                             APPLICATION TASK SWITCH HOOK
*
* Description: This function is called when a task switch is performed.  This allows you to perform other operations
*              during a context switch.
*
* Arguments  : none
*
* Note(s)    : 1) Interrupts are disabled during this call.
*              2) It is assumed that the global pointer 'OSTCBHighRdyPtr' points to the TCB of the task that will be
*                 'switched in' (i.e. the highest priority task) and, 'OSTCBCurPtr' points to the task being switched out
*                 (i.e. the preempted task).
************************************************************************************************************************
*/

void  App_OS_TaskSwHook(void)
{

}

/*$PAGE*/
/*
************************************************************************************************************************
*                                                APPLICATION TICK HOOK
*
* Description: This function is called every tick.
*
* Arguments  : none
*
* Note(s)    : 1) This function is assumed to be called from the Tick ISR.
************************************************************************************************************************
*/
extern          OS_SEM              g_stSystemTimeUpdateSem;

void  App_OS_TimeTickHook(void)
{
    static uint8_t i = 0;
    OS_ERR      err;

    ++i;

    if(i >= OS_CFG_TICK_RATE_HZ)
    {
        i = 0;
        OSSemPost(&g_stSystemTimeUpdateSem,
                  OS_OPT_POST_1,
                  &err);
    }
}

/******************* (C) COPYRIGHT 2015 Guangdong Hydrogen *****END OF FILE****/
