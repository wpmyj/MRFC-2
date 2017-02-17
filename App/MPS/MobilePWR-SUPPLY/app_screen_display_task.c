/*********************************************************************************************************
*                                              APPLICATION CODE
*
*                          (c) Copyright 2016; Guangdong ENECO Science And Technology Co.,Ltd
*
*               All rights reserved.  Protected by international copyright laws.
*               Knowledge of the source code may NOT be used without authorization.
*********************************************************************************************************
*/
/*
*********************************************************************************************************
* Filename      : app_screen_display_task.c
* Version       : V1.00
* Programmer(s) : Fanjun
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
#include <string.h>
/*
*********************************************************************************************************
*                                           MACRO DEFINITIONS
*********************************************************************************************************
*/
#define     SER_TO_SCREEN_DISPLAY_TASK_STK_SIZE         200

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
char CmdStrBuff[80];
/*
*********************************************************************************************************
*                                         FUNCTION PROTOTYPES
*********************************************************************************************************
*/
static void   HMISendCmd(char *buf1);

static void   SerToScreenDisplayTask(void *p_arg);

static void   BSP_Ser3ToLCD_Init(void);
/*
*********************************************************************************************************
*                                         BSP_SerToLCD_Init()
*
* Description : USART3 Init.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   :
*
* Note(s)     : none.
*********************************************************************************************************
*/
void BSP_Ser3ToLCD_Init()
{
    USART_InitTypeDef USART_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);   //使能UART3时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD , ENABLE);  //使能GPIOD口时钟

    GPIO_PinRemapConfig(GPIO_FullRemap_USART3, ENABLE);//串口3完全重映射
    
    //UART3_TX   PD8
    GPIO_InitStructure.GPIO_Pin = BSP_GPIOD_PIN8_USART3_TX_PORT_NMB;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //复用推挽输出
    GPIO_Init(GPIOD, &GPIO_InitStructure); 

    //UART3_RX    PD9
    GPIO_InitStructure.GPIO_Pin = BSP_GPIOD_PIN9_USART3_RX_PORT_NMB;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	//浮空输入
    GPIO_Init(GPIOD, &GPIO_InitStructure);  
	
	USART_DeInit(USART3);  //复位串口3
		
    USART_InitStructure.USART_BaudRate = 9600;		//波特率：9600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
    USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
    USART_InitStructure.USART_Mode = /* USART_Mode_Rx | */USART_Mode_Tx; //暂时只发模式
    USART_Init(USART3, &USART_InitStructure); //初始化串口
		
	//禁用各种中断
    USART_ITConfig(USART3, USART_IT_RXNE, DISABLE);
    USART_ITConfig(USART3, USART_IT_IDLE, DISABLE);
    USART_ITConfig(USART3, USART_IT_TXE, DISABLE);
    USART_ITConfig(USART3, USART_IT_TC, DISABLE);

	USART_Cmd(USART3, ENABLE);
}
/*
*********************************************************************************************************
*                                          ScreenDisplayTaskCreate()
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
void  ScreenDisplayTaskCreate(void)
{
    OS_ERR      err;
    
    BSP_Ser3ToLCD_Init(); //串口3初始化
    
    OSTaskCreate((OS_TCB *)&SerToScreenDisplayTaskTCB,
                 (CPU_CHAR *)"Serial Screen display Task Start",
                 (OS_TASK_PTR) SerToScreenDisplayTask,
                 (void *) 0,
                 (OS_PRIO) SERIAL_SCREEN_DISPLAY_TASK_PRIO,
                 (CPU_STK *)&SerToScreenDisplayTaskStk[0],
                 (CPU_STK_SIZE) SER_TO_SCREEN_DISPLAY_TASK_STK_SIZE / 10,
                 (CPU_STK_SIZE) SER_TO_SCREEN_DISPLAY_TASK_STK_SIZE,
                 (OS_MSG_QTY) 5u,
                 (OS_TICK) 0u,
                 (void *) 0,
                 (OS_OPT)(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR *)&err);
    APP_TRACE_INFO(("Created Serial to screen display Task, and err code is %d...\r\n", err));
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
        OSTimeDlyHMSM(0, 0, 1, 000,
                      OS_OPT_TIME_HMSM_STRICT,
                      &err);
        //1S更新一次屏幕
        UpdateScreen();
    }
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
    u16 StartRemainSecond = 0;
    uint16_t u16AlarmCode = 0;
    uint8_t u8AlarmBit = 0;
    const char AlarmInfoBuf[][10] = {"电堆低温","、电堆高温","、低电压","","","","","","、气压过低","、气压过高","、气体泄漏","","","","",""};
    char  AlarmInfoDisplayBuf[200];
        
    switch((u8)GetSystemWorkStatu())
    {
        case(u8)EN_WAITTING_COMMAND:
            if(YES == GetExternalScreenUpdateStatu() )
            {
                sprintf(CmdStrBuff, "page WelcomePage");
                HMISendCmd(CmdStrBuff);//发送串口指令
//                SetExternalScreenUpdateStatu(NO);
            }
            break;
        case(u8)EN_START_PRGM_ONE_FRONT:
        case(u8)EN_START_PRGM_ONE_BEHIND:
        case(u8)EN_START_PRGM_TWO:
            break;

        case(u8)EN_RUNNING:
            if(GetExternalScreenUpdateStatu() == YES)
            {
                sprintf(CmdStrBuff, "page HalfMachine2");
                HMISendCmd(CmdStrBuff);//发送串口指令
                SetExternalScreenUpdateStatu(NO);
            }
            
            //报警信息显示
            u16AlarmCode = (uint16_t)(GetRunAlarmCode() & 0xFFFF);
            for(u8AlarmBit = 0;u8AlarmBit < 16;u8AlarmBit++){
                if(((u16AlarmCode < u8AlarmBit) & 1) != 0){
                    strcat(AlarmInfoDisplayBuf,AlarmInfoBuf[u8AlarmBit]);
                }
            }
            APP_TRACE_INFO(("%s...\n\r",AlarmInfoDisplayBuf));   
            sprintf(CmdStrBuff, "AlarmInfo.txt=\"%s\"",AlarmInfoDisplayBuf);
            HMISendCmd(CmdStrBuff);//发送串口指令
            
//            sprintf(CmdStrBuff, "WSValue.txt=\"运行\"");
//            HMISendCmd(CmdStrBuff);//发送串口指令
            
            sprintf(CmdStrBuff, "FTValue.txt=\"%.d℃\"",(uint8_t)GetSrcAnaSig(STACK_TEMP) );
            HMISendCmd(CmdStrBuff);//发送串口指令
            sprintf(CmdStrBuff, "VValue.txt=\"%.1fV\"",GetSrcAnaSig(STACK_VOLTAGE) );
            HMISendCmd(CmdStrBuff);//发送串口指令
            sprintf(CmdStrBuff, "IValue.txt=\"%.1fA\"", GetSrcAnaSig(STACK_CURRENT));
            HMISendCmd(CmdStrBuff);//发送串口指令
            sprintf(CmdStrBuff, "PValue.txt=\"%.2fW\"", GetCurrentPower());
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
    u8 i = 0;
    u8 j = 0;

    while(1)
    {
        if(buf1[i] != 0)
        {
            USART_SendData(USART3, buf1[i]);  //发送一个字节

            while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET)
            {};//等待发送结束

            i++;
        }
        else
        {
            for(j = 0; j < 3; j++)
            {
                USART_SendData(USART3, 0xFF);  //连续发送三次0xFF，完成一个结束符的发送（HMI屏指令规定）

                while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET)
                {};//等待发送结束
            }

            break;
        }
    }
}



