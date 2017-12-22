/*
***************************************************************************************************
*                                         APPLICATION CODE
*
*                      (c) Copyright 2017; Guangdong ENECO Science And Technology Co.,Ltd
*
*               All rights reserved.  Protected by international copyright laws.
*               Knowledge of the source code may NOT be used without authorization.
***************************************************************************************************
*/
/*
***************************************************************************************************
* Filename      : bsp_delay_task_timer.c
* Version       : V1.00
* Programmer(s) : JasonFan
***************************************************************************************************
*/
/*
***************************************************************************************************
*                                             INCLUDE FILES
***************************************************************************************************
*/
#include "bsp_delay_task_timer.h"
#include "bsp_can.h"
#include "app_stack_manager.h"
/*
***************************************************************************************************
*                                         OS-RELATED    VARIABLES
***************************************************************************************************
*/

/*
***************************************************************************************************
*                                           MACRO DEFINITIONS
***************************************************************************************************
*/
#define TASK_TIMER			 TIM6
#define RCC_PERIPH_TIMER     RCC_APB1Periph_TIM6
#define BSP_INT_ID           BSP_INT_ID_TIM6


/*
***************************************************************************************************
*                                           LOCAL VARIABLES
***************************************************************************************************
*/
static uint8_t g_u8DelayTaskNum = 0; //定时器当前正在进行和等待的任务数
static TIM6_DELAY_TASK_TYPE_AND_TIME_PARA_Typedef st_DelayTaskQueue[20] = {

    {TASK_MAX_NUM, 0xFFFF},
    {TASK_MAX_NUM, 0xFFFF},
    {TASK_MAX_NUM, 0xFFFF},
    {TASK_MAX_NUM, 0xFFFF},
    {TASK_MAX_NUM, 0xFFFF},
    {TASK_MAX_NUM, 0xFFFF},
    {TASK_MAX_NUM, 0xFFFF},
    {TASK_MAX_NUM, 0xFFFF},
    {TASK_MAX_NUM, 0xFFFF},
    {TASK_MAX_NUM, 0xFFFF},
    {TASK_MAX_NUM, 0xFFFF},
    {TASK_MAX_NUM, 0xFFFF},
    {TASK_MAX_NUM, 0xFFFF},
    {TASK_MAX_NUM, 0xFFFF},
    {TASK_MAX_NUM, 0xFFFF},
    {TASK_MAX_NUM, 0xFFFF},
    {TASK_MAX_NUM, 0xFFFF},
    {TASK_MAX_NUM, 0xFFFF},
    {TASK_MAX_NUM, 0xFFFF},
    {TASK_MAX_NUM, 0xFFFF}
};

static TIM_TimeBaseInitTypeDef     TimerBaseStructure;


/*
***************************************************************************************************
*                                           GLOBAL VARIABLES
***************************************************************************************************
*/


/*
***************************************************************************************************
*                                         FUNCTION PROTOTYPES
***************************************************************************************************
*/
static void ResponseTheDelayTaskAndExecuteCmd(TIMER_DELAY_TASK_TYPE_Typedef i_eTaskType);

static void DelayQueue_IRQHandler(void);

/*
***************************************************************************************************
*                                          DelayQueueTimerInit()
*
* Description : Init the timer 6 for delay task queue.
*
* Arguments   : none.
*
* Returns     : none
*
* Notes       : none.
***************************************************************************************************
*/
void DelayQueueTimerInit(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_PERIPH_TIMER, ENABLE);

    TIM_DeInit(TASK_TIMER);
    TimerBaseStructure.TIM_Period = 0; //设定计数器自动重装值,装入2为1ms
    TimerBaseStructure.TIM_Prescaler = 35999;   //预分频器，将72MHz降为2KHz,1000/2000 = 0.5
    TimerBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割，定时器6.7无效
    TimerBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
    TIM_TimeBaseInit(TASK_TIMER, &TimerBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位

    //中断分组初始化
    NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn;  //TIM6中断
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  //先占优先级2级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //从优先级0级
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;     //IRQ通道被使能
    NVIC_Init(&NVIC_InitStructure);  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器

    BSP_IntVectSet(BSP_INT_ID, DelayQueue_IRQHandler);
    BSP_IntEn(BSP_INT_ID);
}
/*
***************************************************************************************************
*                                          CheckAuthorization()
*
* Description : The use of this funciton is to check the authorization of the system.
*
* Arguments   : i_eTaskType:定义的延时任务类型.
*               i_u16TaskDelayMs:延时任务的延时时间,单位ms，最大不能超过32767
*
* Returns     : none
*
* Notes       : 用于开启一个延时任务，不要开启打印信息,因为在外部中断检测中有调用.
***************************************************************************************************
*/

void StartTimerDelayTask(TIMER_DELAY_TASK_TYPE_Typedef i_eTaskType, u16 i_u16TaskDelayMs)
{
    vu16  vu16TimerCaptureValue = 0;
    uint8_t i, j;
    TIM6_DELAY_TASK_TYPE_AND_TIME_PARA_Typedef m_NewDelayTask;

    g_u8DelayTaskNum ++;

    if(g_u8DelayTaskNum <= 1) {

//        APP_TRACE_INFO(("Start new task %d!<--- \n\r",i_eTaskType));
        TIM_ITConfig(TASK_TIMER, TIM_IT_Update, DISABLE);
        TIM_Cmd(TASK_TIMER, DISABLE); //失能定时器6

        st_DelayTaskQueue[0].DelayTask = i_eTaskType;
        st_DelayTaskQueue[0].DelayTime = i_u16TaskDelayMs * 2 - 1;//计算重载值(ms值转换为重载值)Prescaler value
        TimerBaseStructure.TIM_Period = st_DelayTaskQueue[0].DelayTime; //设定自动重载值
        TIM_TimeBaseInit(TASK_TIMER, &TimerBaseStructure); //初始化定时器时基参数

        TIM_GenerateEvent(TASK_TIMER, TIM_EventSource_Update);   // 产生软件更新事件，立即更新数据
        TIM_ClearFlag(TASK_TIMER, TIM_FLAG_Update);             //清除标志位。定时器一打开便产生更新事件，若不清除，将会进入中断
        TIM_ITConfig(TASK_TIMER, TIM_IT_Update, ENABLE);
        TIM_Cmd(TASK_TIMER, ENABLE);
    } else {

        vu16TimerCaptureValue = TIM_GetCounter(TASK_TIMER);

        if(i_u16TaskDelayMs * 2 > (TimerBaseStructure.TIM_Period - vu16TimerCaptureValue)) { //新加任务在当前任务后响应

            m_NewDelayTask.DelayTime = (i_u16TaskDelayMs * 2 - (TimerBaseStructure.TIM_Period - vu16TimerCaptureValue)) - 1;//新增任务在当前任务完成后需要延后的时间

            //新增任务等待时间最长，若是，则直接添加在队尾
            if((g_u8DelayTaskNum <= 2)
                    || (m_NewDelayTask.DelayTime >= st_DelayTaskQueue[g_u8DelayTaskNum - 2].DelayTime)) {
//                APP_TRACE_INFO(("Add new task %d to the end of the list!<--- \n\r"));
                st_DelayTaskQueue[g_u8DelayTaskNum - 1].DelayTask = i_eTaskType;
                st_DelayTaskQueue[g_u8DelayTaskNum - 1].DelayTime = m_NewDelayTask.DelayTime;
            } else { //新增任务可能比正在等待的任务等待时间短，需要将其插入至适当位置进行等待

//                APP_TRACE_INFO(("Insert new task %d to the waitting list!<---\n\r",i_eTaskType));
                for(i = 1; i < g_u8DelayTaskNum - 1 ; i ++) {
                    if(m_NewDelayTask.DelayTime <= st_DelayTaskQueue[i].DelayTime) {
                        for(j = g_u8DelayTaskNum - 1; j > i; j --) {

                            st_DelayTaskQueue[j].DelayTask = st_DelayTaskQueue[j - 1].DelayTask;
                            st_DelayTaskQueue[j].DelayTime = st_DelayTaskQueue[j - 1].DelayTime;
                        }

                        st_DelayTaskQueue[i].DelayTask = i_eTaskType;
                        st_DelayTaskQueue[i].DelayTime = m_NewDelayTask.DelayTime;
                        break;
                    }
                }
            }
        } else { //新加入的任务比当前任务响应还要早

//            APP_TRACE_INFO(("The new task response earlier of all task!<---\n\r"));
            for(i = g_u8DelayTaskNum - 1; i >= 1; i--) {
                st_DelayTaskQueue[i].DelayTask = st_DelayTaskQueue[i - 1].DelayTask;//任务队列整体后移
                //任务延时时间需要
                st_DelayTaskQueue[i].DelayTime = (TimerBaseStructure.TIM_Period - vu16TimerCaptureValue - i_u16TaskDelayMs * 2) + st_DelayTaskQueue[i - 1].DelayTime;
            }

            TIM_ITConfig(TASK_TIMER, TIM_IT_Update, DISABLE);
            TIM_Cmd(TASK_TIMER, DISABLE);

            st_DelayTaskQueue[0].DelayTask = i_eTaskType;
            st_DelayTaskQueue[0].DelayTime = i_u16TaskDelayMs * 2 - 1;

            TimerBaseStructure.TIM_Period = st_DelayTaskQueue[0].DelayTime; //设定自动重载值
            TIM_TimeBaseInit(TASK_TIMER, &TimerBaseStructure); //初始化定时器时基参数

            TIM_GenerateEvent(TASK_TIMER, TIM_EventSource_Update);   //产生软件更新事件，立即更新数据
            TIM_ClearFlag(TASK_TIMER, TIM_FLAG_Update);             //清除标志位。定时器一打开便产生更新事件，若不清除，将会进入中断
            TIM_ITConfig(TASK_TIMER, TIM_IT_Update, ENABLE);
            TIM_Cmd(TASK_TIMER, ENABLE);
        }
    }
}


/*
***************************************************************************************************
*                                          AddNewDelayTaskToTimerQueue()
*
* Description : Add the new delay task to the timer6 delay queue.
*
* Arguments   : i_eTaskType:定义的延时任务类型.
*               i_u16TaskDelayMs:添加的延时任务的延时时间,单位ms
*
* Returns     : none
*
* Notes       : 用于在已有的延时队列中继续添加延时任务.
***************************************************************************************************
*/
void AddNewDelayTaskToTimerQueue(TIMER_DELAY_TASK_TYPE_Typedef i_eTaskType, u16 i_u16TaskDelayMs)
{
    uint8_t i, j;
    TIM6_DELAY_TASK_TYPE_AND_TIME_PARA_Typedef m_NewDelayTask;

    g_u8DelayTaskNum ++;

    m_NewDelayTask.DelayTime = i_u16TaskDelayMs * 2 - 1;//新增任务需要延后的时间(ms时间转换为重载值)

    if(g_u8DelayTaskNum == 2) {
        st_DelayTaskQueue[1].DelayTask = i_eTaskType;
        st_DelayTaskQueue[1].DelayTime = m_NewDelayTask.DelayTime;
    } else if(g_u8DelayTaskNum >= 3) {//进行插入排序
        for(i = 1; i <= g_u8DelayTaskNum - 1 ; i ++) {
            if(m_NewDelayTask.DelayTime <= st_DelayTaskQueue[i].DelayTime) {
                for(j = g_u8DelayTaskNum - 1; j > i; j --) {
                    st_DelayTaskQueue[j].DelayTask = st_DelayTaskQueue[j - 1].DelayTask;
                    st_DelayTaskQueue[j].DelayTime = st_DelayTaskQueue[j - 1].DelayTime;
                }

                st_DelayTaskQueue[i].DelayTask = i_eTaskType;//将新任务添加到队列中
                st_DelayTaskQueue[i].DelayTime = m_NewDelayTask.DelayTime;
                break;
            }
        }
    }
}

/*
***************************************************************************************************
*                                          DelayQueue_IRQHandler()
*
* Description : The use of this funciton is to check the authorization of the system.
*               s
* Arguments   : none.
*
* Returns     : none
*
* Notes       : none.
***************************************************************************************************
*/
static void DelayQueue_IRQHandler(void)
{
    uint8_t i;

    TIM_ITConfig(TASK_TIMER, TIM_IT_Update, DISABLE);
    TIM_Cmd(TASK_TIMER, DISABLE);

    do {
        if(g_u8DelayTaskNum >= 1) {//对列中只存在一个任务了

            ResponseTheDelayTaskAndExecuteCmd(st_DelayTaskQueue[0].DelayTask);
            g_u8DelayTaskNum --;
            st_DelayTaskQueue[0].DelayTask = st_DelayTaskQueue[1].DelayTask;
            st_DelayTaskQueue[0].DelayTime = st_DelayTaskQueue[1].DelayTime;

            if(g_u8DelayTaskNum >= 2) {//队列中还有多个任务
                for(i = 1; i <= g_u8DelayTaskNum - 1; i++) {

                    st_DelayTaskQueue[i].DelayTask = st_DelayTaskQueue[i + 1].DelayTask;
                    //执行完前面任务，切换至新任务的等待后，后面等待任务的等待时间要以新任务为准
                    st_DelayTaskQueue[i].DelayTime = st_DelayTaskQueue[i + 1].DelayTime - st_DelayTaskQueue[0].DelayTime;
                }

                st_DelayTaskQueue[g_u8DelayTaskNum].DelayTask = TASK_MAX_NUM;//清空延时任务信息
                st_DelayTaskQueue[g_u8DelayTaskNum].DelayTime = 0xFFFF;
            } else {
                st_DelayTaskQueue[1].DelayTask = TASK_MAX_NUM;
                st_DelayTaskQueue[1].DelayTime = 0xFFFF;   //若当前任务只有一个，后面的则不需要减掉前面任务等待时间
            }
        } else {
            break;
        }
    } while(st_DelayTaskQueue[0].DelayTime == 0);//将所有与当前任务同时响应的任务都执行完

    TIM_ITConfig(TASK_TIMER, TIM_IT_Update, DISABLE);
    TIM_Cmd(TASK_TIMER, DISABLE); //失能定时器6

    //有任务时,
    if(g_u8DelayTaskNum >= 1) {
        TimerBaseStructure.TIM_Period = st_DelayTaskQueue[0].DelayTime; //设定计数器重装值
        TIM_TimeBaseInit(TASK_TIMER, &TimerBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位

        TIM_GenerateEvent(TASK_TIMER, TIM_EventSource_Update);   //产生软件更新事件，立即更新数据
        TIM_ClearFlag(TASK_TIMER, TIM_FLAG_Update);             //清除标志位。定时器一打开便产生更新事件，若不清除，将会进入中断
        TIM_ITConfig(TASK_TIMER, TIM_IT_Update, ENABLE);
        TIM_Cmd(TASK_TIMER, ENABLE);  //使能定时器6
    }

    TIM_ClearITPendingBit(TASK_TIMER, TIM_IT_Update); //清除中断标志位
}


/*
***************************************************************************************************
*                              ResponseTheDelayTaskAndExecuteCmd()
*
* Description : Respone the delay task and execute the cmd.
*
* Arguments   : none.
*
* Returns     : none
*
* Notes       : 在此函数中不能做打印信息操作.
***************************************************************************************************
*/
static void ResponseTheDelayTaskAndExecuteCmd(TIMER_DELAY_TASK_TYPE_Typedef i_eTaskType)
{

    switch((u8)i_eTaskType) {

        case CAN_BUS_AUTO_RECONNECT_AFTER_30_SEC:
            SetCanBusOnlineFlag(DEF_YES);//检测一下CAN是否在线
            CAN1_Init();
            break;

        case START_UP_SWITCH_CHECK_DELAY_1S:
            CmdButtonStatuCheck();
            break;

        case SHUT_DOWN_SWITCH_CHECK_DELAY_3S:
            CmdButtonStatuCheck();
            break;
		
		case UPDATE_DECOMPRESS_CNT_EVER_1MIN:
			UpdatePassiveDecompressCntPerMin();
            AddNewDelayTaskToTimerQueue(UPDATE_DECOMPRESS_CNT_EVER_1MIN,60000);
            break;
		

        default:
            break;
    }
}

/******************* (C) COPYRIGHT 2016 Guangdong ENECO POWER *****END OF FILE****/
