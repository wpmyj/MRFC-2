/*********************************************************************************************************
*                                              APPLICATION CODE
*
*                          (c) Copyright 2016; Guangdong Hydrogen Energy Science And Technology Co.,Ltd
*
*               All rights reserved.  Protected by international copyright laws.
*               Knowledge of the source code may NOT be used without authorization.
*********************************************************************************************************
*/
/*
*********************************************************************************************************
* Filename      : app_speed_control_device_monitor_task.c
* Version       : V1.00
* Programmer(s) : SunKing.Yun
*********************************************************************************************************
*/


/*
**********************************************************************************************************
                                      INCLUDE FILES
**********************************************************************************************************
*/
#include "includes.h"
#include "app_screen_display_task.h"
#include "app_system_real_time_parameters.h"
#include "app_top_task.h"

/*
*********************************************************************************************************
*                                           MACRO DEFINITIONS
*********************************************************************************************************
*/
#define     SER_TO_SCREEN_DISPLAY_TASK_STK_SIZE         64
#define         SCREEN_PAGE_NMB_MAX     30


/*
*********************************************************************************************************
*                                         OS-RELATED    VARIABLES
*********************************************************************************************************
*/
OS_TCB      SerToScreenDisplayTaskTCB;

static      CPU_STK     SerToScreenDisplayTaskStk[SER_TO_SCREEN_DISPLAY_TASK_STK_SIZE];

/*
*********************************************************************************************************
*                                       LOCAL GLOBAL VARIABLES
*********************************************************************************************************
*/
//static uint8_t g_u8CurrentScreenPage = 0;
char CmdStrBuff[80];
/*
*********************************************************************************************************
*                                         FUNCTION PROTOTYPES
*********************************************************************************************************
*/
static void HMISendCmd(char *buf1);

static void   SerToScreenDisplayTask(void *p_arg);


/*
*********************************************************************************************************
*                                          SerToScreenDisplayTaskCreate()
*
* Description : create the task that serial screen display.
*
* Arguments   : none.
*
* Returns     : none.
*
* Notes       : none.
*********************************************************************************************************
*/
void  SerToScreenDisplayTaskCreate(void)
{
    OS_ERR      err;

    OSTaskCreate((OS_TCB *)&SerToScreenDisplayTaskTCB,
                 (CPU_CHAR *)"Serial Screen display Task Start",
                 (OS_TASK_PTR) SerToScreenDisplayTask,
                 (void *) 0,
                 (OS_PRIO) SPEED_CONTROL_DEVICE_MONITOR_TASK_PRIO,
                 (CPU_STK *)&SerToScreenDisplayTaskStk[0],
                 (CPU_STK_SIZE) SER_TO_SCREEN_DISPLAY_TASK_STK_SIZE / 10,
                 (CPU_STK_SIZE) SER_TO_SCREEN_DISPLAY_TASK_STK_SIZE,
                 (OS_MSG_QTY) 5u,
                 (OS_TICK) 0u,
                 (void *) 0,
                 (OS_OPT)(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR *)&err);
    APP_TRACE_INFO(("Created Serial to screen display Task, and err code is %d...\n\r", err));
}

/*
*********************************************************************************************************
*                                          SerToScreenDisplayTask()
*
* Description : Serial to screen diaplay task.
*
* Arguments   : none.
*
* Returns     : none.
*
* Notes       : none.
*********************************************************************************************************
*/
static void   SerToScreenDisplayTask(void *p_arg)
{
    OS_ERR      err;

    while(DEF_TRUE)
    {
        OSTimeDlyHMSM(0, 0, 0, 500,
                      OS_OPT_TIME_HMSM_STRICT,
                      &err);
        //0.5S更新一次屏幕
        UpdateScreen();
    }
}


/*
*********************************************************************************************************
*                                          ScreenInit()
*
* Description : none.
*
* Arguments   : none.
*
* Returns     : none.
*
* Notes       : none.
*********************************************************************************************************
*/
void ScreenInit(void)
{
    /*HMI串口屏不需要初始化*/
}


/*
*********************************************************************************************************
*                                          UpdateScreen()
*
* Description : Update the screen
*
* Arguments   : none.
*
* Returns     : none.
*
* Notes       : none.
*********************************************************************************************************
*/
void UpdateScreen(void)
{
    u16 StartRemainSecond = 900;

    switch((u8)GetSystemWorkStatu())
    {
        case(u8)EN_WAITTING_COMMAND:
            if(GetExternalScreenUpdateStatu() == YES)
            {
                sprintf(CmdStrBuff, "page WelcomePage");
                HMISendCmd(CmdStrBuff);//发送串口指令
                sprintf(CmdStrBuff, "t0.txt=\"主人，素电宝宝状态良好，随时为您服务\"");
                HMISendCmd(CmdStrBuff);//发送串口指令
                sprintf(CmdStrBuff, "UpdateTime.tim=300");
                HMISendCmd(CmdStrBuff);//发送串口指令
                SetExternalScreenUpdateStatu(NO);
            }

            break;

        case(u8)EN_START_PRGM_ONE_FRONT:
        case(u8)EN_START_PRGM_ONE_BEHIND:
        case(u8)EN_START_PRGM_TWO:
            if(GetExternalScreenUpdateStatu() == YES)
            {
                sprintf(CmdStrBuff, "page WelcomePage");
                HMISendCmd(CmdStrBuff);//发送串口指令
                sprintf(CmdStrBuff, "UpdateTime.tim=100");
                HMISendCmd(CmdStrBuff);//发送串口指令
                //获取启动剩余时间
                StartRemainSecond = GetStartRemainSencond();
                sprintf(CmdStrBuff, "t0.txt=\"素电宝宝正在拼命为您启动中，请耐心等候，还有%d分%d秒\"", StartRemainSecond / 60, StartRemainSecond % 60);
                HMISendCmd(CmdStrBuff);
                SetExternalScreenUpdateStatu(NO);
            }
            else
            {
                StartRemainSecond = GetStartRemainSencond();
                sprintf(CmdStrBuff, "t0.txt=\"素电宝宝正在拼命为您启动中，请耐心等候，还有%d分%d秒\"", StartRemainSecond / 60, StartRemainSecond % 60);
                HMISendCmd(CmdStrBuff);//发送串口指令
            }

            break;

        case(u8)EN_RUNNING:
            if(GetExternalScreenUpdateStatu() == YES)
            {
                sprintf(CmdStrBuff, "page CmpMachineDft");
                HMISendCmd(CmdStrBuff);//发送串口指令
                SetExternalScreenUpdateStatu(NO);
            }

            sprintf(CmdStrBuff, "WSValue.txt=\"运行\"");
            HMISendCmd(CmdStrBuff);//发送串口指令
            sprintf(CmdStrBuff, "VValue.txt=\"%.2f\"", GetSrcAnaSig(STACK_VOLTAGE));
            HMISendCmd(CmdStrBuff);//发送串口指令
            sprintf(CmdStrBuff, "IValue.txt=\"%.2f\"", GetSrcAnaSig(STACK_CURRENT));
            HMISendCmd(CmdStrBuff);//发送串口指令
            sprintf(CmdStrBuff, "PValue.txt=\"%.4f\"", GetCurrentPower());
            HMISendCmd(CmdStrBuff);//发送串口指令
            sprintf(CmdStrBuff, "FSValue.txt=\"100\"");
            HMISendCmd(CmdStrBuff);//发送串口指令
            break;
    }
}


/*
*********************************************************************************************************
*                                          void HMISendCmd(char *buf1)
*
* Description : 向人机界面发送字符串指令函数
*
* Arguments   : none.
*
* Returns     : none.
*
* Notes       : none.
*********************************************************************************************************
*/
void HMISendCmd(char *buf1)
{
//    u8 i = 0;
//    u8 j = 0;

    while(1)
    {
//        if(buf1[i] != 0)
//        {
////            USART_SendData(UART5, buf1[i]);  //发送一个字节

//            while(USART_GetFlagStatus(UART5, USART_FLAG_TXE) == RESET)
//            {};//等待发送结束

//            i++;
//        }
//        else
//        {
//            for(j = 0; j < 3; j++)
//            {
////                USART_SendData(UART5, 0xFF);  //连续发送三次0xFF，完成一个结束符的发送（HMI屏指令规定）

//                while(USART_GetFlagStatus(UART5, USART_FLAG_TXE) == RESET)
//                {};//等待发送结束
//            }

//            break;
//        }
    }
}


