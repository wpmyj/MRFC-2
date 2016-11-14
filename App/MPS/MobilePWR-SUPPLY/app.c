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
* Filename      : app.c
* Version       : V1.00
* Programmer(s) : 
*********************************************************************************************************
*/
/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/
#include <includes.h>
#include <os_app_hooks.h>
#include "app_system_real_time_parameters.h"
#include "app_system_run_cfg_parameters.h"
#include "app_top_task.h"
#include "app_hydrg_producer_manager.h"
#include "app_stack_manager.h"
#include "app_wireness_communicate_task.h"
#include "app_analog_signal_monitor_task.h"
#include "app_digital_signal_monitor_task.h"
#include "app_speed_control_device_monitor_task.h"
#include "app_screen_display_task.h"
#include "RS485CommWithUart5TaskCreate.h"
#include "Make_Vacuum.h"
#include "Time.h"
/*
*********************************************************************************************************
*                                           MACRO DEFINITIONS
*********************************************************************************************************
*/
#define  APP_TASK_START_STK_SIZE                    1024         /*size of TASK STK*/

/*
*********************************************************************************************************
*                                         OS-RELATED    VARIABLES
*********************************************************************************************************
*/
extern      OS_SEM      g_stAnaSigConvertFinishSem;
extern      OS_SEM      IgniteFirstBehindWaitSem;


extern      OS_SEM      IgniteSecondBehindWaitSem;
extern      OS_SEM      MannualSelcetWorkModeSem;

OS_TCB      AppTaskStartTCB;

static CPU_STK_8BYTE_ALIGNED AppTaskStartStk[APP_TASK_START_STK_SIZE];
/*
*********************************************************************************************************
*                                         FUNCTION PROTOTYPES
*********************************************************************************************************
*/

static      void        AppTaskStart(void *p_arg);
static      void        USER_NVIC_Cfg(void);

/*
*********************************************************************************************************
*                                                main()
*
* Description : This is the standard entry point for C code.  It is assumed that your code will call
*               main() once you have performed all necessary initialization.
*
* Arguments   : none
*
* Returns     : none
*********************************************************************************************************
*/
int  main(void)
{
    OS_ERR  err;
    CPU_SR_ALLOC();
    BSP_IntDisAll();                                            /* Disable all interrupts.                              */

    OSInit(&err);                                               /* Init uC/OS-III.                                      */
    OS_CRITICAL_ENTER();//�����ٽ���

    OSTaskCreate((OS_TCB *)&AppTaskStartTCB,                    //������ƿ�
                 (CPU_CHAR *)"App Task Start",                                  //��������
                 (OS_TASK_PTR) AppTaskStart,                                        //������
                 (void *) 0,                                                                //���ݸ��������Ĳ���
                 (OS_PRIO) APP_TASK_START_PRIO,                             //�������ȼ�
                 (CPU_STK *)&AppTaskStartStk[0],                            //�����ջ����ַ
                 (CPU_STK_SIZE) APP_TASK_START_STK_SIZE / 10,       //�����ջ�����λ
                 (CPU_STK_SIZE) APP_TASK_START_STK_SIZE,            //�����ջ��С
                 (OS_MSG_QTY) 5u,                                   //�����ڲ���Ϣ�����ܹ����յ������Ϣ��Ŀ,Ϊ0ʱ��ֹ������Ϣ
                 (OS_TICK) 0u,                                      //��ʹ��ʱ��Ƭ��תʱ��ʱ��Ƭ���ȣ�Ϊ0ʱΪĬ�ϳ��ȣ�
                 (void *) 0,                                        //�û�����Ĵ洢��
                 (OS_OPT)(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),       //����ѡ��
                 (OS_ERR *)&err);                                   //��Ÿú�������ʱ�ķ���ֵ
                 
    OS_CRITICAL_EXIT(); //�˳��ٽ���
    OSStart(&err);                                              /* Start multitasking (i.e. give control to uC/OS-III). */
}

/*
*********************************************************************************************************
*                                          STARTUP TASK
*
* Description : This is the code of the startup task.
*
* Arguments   : p_arg   is the argument passed to 'AppTaskStart()' by 'OSTaskCreate()'.
*
* Returns     : none
*
* Notes       : 1) The first line of code is used to prevent a compiler warning because 'p_arg' is not
*                  used.  The compiler should not generate any code for this statement.
*********************************************************************************************************
*/
static  void  AppTaskStart(void *p_arg)
{
    CPU_INT32U  cpu_clk_freq;
    CPU_INT32U  cnts;
    OS_ERR      err;

    VERIFY_RESULT_TYPE_VARIABLE_Typedef eWaitCmdCheckStatu;

    (void)p_arg;

    BSP_Init();

    CPU_Init();

    cpu_clk_freq = BSP_CPU_ClkFreq();                           /* Determine SysTick reference freq.                    */
    cnts = cpu_clk_freq / (CPU_INT32U)OSCfg_TickRate_Hz;        /* Determine nbr SysTick increments                     */
    OS_CPU_SysTickInit(cnts);                                   /* Init uC/OS periodic time src (SysTick).              */

    USER_NVIC_Cfg();

    Mem_Init();                                                 /*��ʼ���ڴ����ģ��     */

#if OS_CFG_STAT_TASK_EN > 0u                                                                    /*ͳ������*/
    OSStatTaskCPUUsageInit(&err);                             /* Compute CPU capacity with no task running            */
#endif

#ifdef CPU_CFG_INT_DIS_MEAS_EN                                //���ʹ���˲����жϹر�ʱ��
    CPU_IntDisMeasMaxCurReset();                              //���õ�ǰ�������ж�ʱ��
#endif

    /*
    #if OS_CFG_SCHED_ROUND_ROBIN_EN                                                     //��ʹ��ʱ��Ƭ��ת��ʱ��
        //ʹ��ʱ��Ƭ��ת���ȹ���,ʱ��Ƭ����Ϊ1��ϵͳʱ�ӽ��ģ���1*5=5ms
        OSSchedRoundRobinCfg(DEF_ENABLED,1,&err);
    #endif
    */
    
#if (APP_CFG_SERIAL_EN == DEF_ENABLED)                                                    /*���ڳ�ʼ��*/
    BSP_Ser_Init(115200);                                       /* Enable Serial Interface                              */
#endif

    OSSemCreate(&g_stAnaSigConvertFinishSem, "Ana Signal convert finish sem", 0, &err);
    OSSemCreate(&IgniteFirstBehindWaitSem, "Fast heater finish sem", 0, &err);
    OSSemCreate(&IgniteSecondBehindWaitSem, "IgniteSecondBehindWaitSem...", 0, &err);
    
    LoadParameters();                   // ���в����ĳ�ʼ����������Ҫ�õ��Ĳ���������Ҫ����ǰ�档

    App_OS_SetAllHooks();               // �û�Ӧ����ؽ��뺯������

    SystemTimeStatTaskCreate();         // ϵͳʱ��ͳ�����񴴽�

    AnaSigMonitorTaskCreate();          // ģ���źż�����񴴽�

    DigSigMonitorTaskCreate();          // �����źż�����񴴽�

    SpdCtlDevMonitorTaskCreate();       // �����豸�������

    WirenessCommTaskCreate();           // ����ͨ�����񴴽�
    
    RS485CommWithUart5TaskCreate();                     //485�봮��ͨ�ŵ�����

    Make_Vacuum_FunctionTaskCreate();                   //����պ�������Ĵ
    
    SerToScreenDisplayTaskCreate();     // ������ʾ����ʾ���񴴽�

    IgniterWorkTaskCreate();            // ��������񴴽�

    HydrgProducerManagerTaskCreate();   // �������������

    StackManagerTaskCreate();           // ��ѹ�������

    HydrgProducerDlyStopTaskCreate();   // �������ʱ�ر�����

    StackManagerDlyStopTaskCreate();    // �����ʱ�ر�����

    while(DEF_TRUE)
    {  
        eWaitCmdCheckStatu = EN_THROUGH;                                        //EN_NOT_THROUGH          
        if( EN_THROUGH == CheckAuthorization() )
        {          
            if( EN_THROUGH == eWaitCmdCheckStatu ) 
            {
                SetWorkMode( EN_WORK_MODE_HYDROGEN_PRODUCER_AND_FUEL_CELL );    // ucosIII test code.
                Starting();            
                Running();
                KeepingWarm();                                                  
            }
            else
            {
                SetSystemWorkStatu( EN_ALARMING );
                DeviceFaultAlarm();
            }
        }
        else
        {
            SetSystemWorkStatu( EN_ALARMING );
            DeviceFaultAlarm();
        }

    }

}


/*
*********************************************************************************************************
*                                          USER NVICConfiguration
*
* Description : The use of this funciton is to set the interrupt group.
*               �û��ж����ȼ�����
* Arguments   : none.
*
* Returns     : none
*
* Notes       : none.
*********************************************************************************************************
*/
void USER_NVIC_Cfg( void )
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);     //����NVIC�жϷ���3λ��ռ���ȼ���1λ��ռ���ȼ�
}

/******************* (C) COPYRIGHT 2015 Guangdong Hydrogen *****END OF FILE****/
