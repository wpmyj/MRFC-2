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
static uint8_t g_u8DelayTaskNum = 0; //��ʱ����ǰ���ڽ��к͵ȴ���������
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
    TimerBaseStructure.TIM_Period = 0; //�趨�������Զ���װֵ,װ��2Ϊ1ms
    TimerBaseStructure.TIM_Prescaler = 35999;   //Ԥ��Ƶ������72MHz��Ϊ2KHz,1000/2000 = 0.5
    TimerBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //����ʱ�ӷָ��ʱ��6.7��Ч
    TimerBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
    TIM_TimeBaseInit(TASK_TIMER, &TimerBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ

    //�жϷ����ʼ��
    NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn;  //TIM6�ж�
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  //��ռ���ȼ�2��
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //�����ȼ�0��
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;     //IRQͨ����ʹ��
    NVIC_Init(&NVIC_InitStructure);  //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���

    BSP_IntVectSet(BSP_INT_ID, DelayQueue_IRQHandler);
    BSP_IntEn(BSP_INT_ID);
}
/*
***************************************************************************************************
*                                          CheckAuthorization()
*
* Description : The use of this funciton is to check the authorization of the system.
*
* Arguments   : i_eTaskType:�������ʱ��������.
*               i_u16TaskDelayMs:��ʱ�������ʱʱ��,��λms������ܳ���32767
*
* Returns     : none
*
* Notes       : ���ڿ���һ����ʱ���񣬲�Ҫ������ӡ��Ϣ,��Ϊ���ⲿ�жϼ�����е���.
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
        TIM_Cmd(TASK_TIMER, DISABLE); //ʧ�ܶ�ʱ��6

        st_DelayTaskQueue[0].DelayTask = i_eTaskType;
        st_DelayTaskQueue[0].DelayTime = i_u16TaskDelayMs * 2 - 1;//��������ֵ(msֵת��Ϊ����ֵ)Prescaler value
        TimerBaseStructure.TIM_Period = st_DelayTaskQueue[0].DelayTime; //�趨�Զ�����ֵ
        TIM_TimeBaseInit(TASK_TIMER, &TimerBaseStructure); //��ʼ����ʱ��ʱ������

        TIM_GenerateEvent(TASK_TIMER, TIM_EventSource_Update);   // ������������¼���������������
        TIM_ClearFlag(TASK_TIMER, TIM_FLAG_Update);             //�����־λ����ʱ��һ�򿪱���������¼��������������������ж�
        TIM_ITConfig(TASK_TIMER, TIM_IT_Update, ENABLE);
        TIM_Cmd(TASK_TIMER, ENABLE);
    } else {

        vu16TimerCaptureValue = TIM_GetCounter(TASK_TIMER);

        if(i_u16TaskDelayMs * 2 > (TimerBaseStructure.TIM_Period - vu16TimerCaptureValue)) { //�¼������ڵ�ǰ�������Ӧ

            m_NewDelayTask.DelayTime = (i_u16TaskDelayMs * 2 - (TimerBaseStructure.TIM_Period - vu16TimerCaptureValue)) - 1;//���������ڵ�ǰ������ɺ���Ҫ�Ӻ��ʱ��

            //��������ȴ�ʱ��������ǣ���ֱ������ڶ�β
            if((g_u8DelayTaskNum <= 2)
                    || (m_NewDelayTask.DelayTime >= st_DelayTaskQueue[g_u8DelayTaskNum - 2].DelayTime)) {
//                APP_TRACE_INFO(("Add new task %d to the end of the list!<--- \n\r"));
                st_DelayTaskQueue[g_u8DelayTaskNum - 1].DelayTask = i_eTaskType;
                st_DelayTaskQueue[g_u8DelayTaskNum - 1].DelayTime = m_NewDelayTask.DelayTime;
            } else { //����������ܱ����ڵȴ�������ȴ�ʱ��̣���Ҫ����������ʵ�λ�ý��еȴ�

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
        } else { //�¼��������ȵ�ǰ������Ӧ��Ҫ��

//            APP_TRACE_INFO(("The new task response earlier of all task!<---\n\r"));
            for(i = g_u8DelayTaskNum - 1; i >= 1; i--) {
                st_DelayTaskQueue[i].DelayTask = st_DelayTaskQueue[i - 1].DelayTask;//��������������
                //������ʱʱ����Ҫ
                st_DelayTaskQueue[i].DelayTime = (TimerBaseStructure.TIM_Period - vu16TimerCaptureValue - i_u16TaskDelayMs * 2) + st_DelayTaskQueue[i - 1].DelayTime;
            }

            TIM_ITConfig(TASK_TIMER, TIM_IT_Update, DISABLE);
            TIM_Cmd(TASK_TIMER, DISABLE);

            st_DelayTaskQueue[0].DelayTask = i_eTaskType;
            st_DelayTaskQueue[0].DelayTime = i_u16TaskDelayMs * 2 - 1;

            TimerBaseStructure.TIM_Period = st_DelayTaskQueue[0].DelayTime; //�趨�Զ�����ֵ
            TIM_TimeBaseInit(TASK_TIMER, &TimerBaseStructure); //��ʼ����ʱ��ʱ������

            TIM_GenerateEvent(TASK_TIMER, TIM_EventSource_Update);   //������������¼���������������
            TIM_ClearFlag(TASK_TIMER, TIM_FLAG_Update);             //�����־λ����ʱ��һ�򿪱���������¼��������������������ж�
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
* Arguments   : i_eTaskType:�������ʱ��������.
*               i_u16TaskDelayMs:��ӵ���ʱ�������ʱʱ��,��λms
*
* Returns     : none
*
* Notes       : ���������е���ʱ�����м��������ʱ����.
***************************************************************************************************
*/
void AddNewDelayTaskToTimerQueue(TIMER_DELAY_TASK_TYPE_Typedef i_eTaskType, u16 i_u16TaskDelayMs)
{
    uint8_t i, j;
    TIM6_DELAY_TASK_TYPE_AND_TIME_PARA_Typedef m_NewDelayTask;

    g_u8DelayTaskNum ++;

    m_NewDelayTask.DelayTime = i_u16TaskDelayMs * 2 - 1;//����������Ҫ�Ӻ��ʱ��(msʱ��ת��Ϊ����ֵ)

    if(g_u8DelayTaskNum == 2) {
        st_DelayTaskQueue[1].DelayTask = i_eTaskType;
        st_DelayTaskQueue[1].DelayTime = m_NewDelayTask.DelayTime;
    } else if(g_u8DelayTaskNum >= 3) {//���в�������
        for(i = 1; i <= g_u8DelayTaskNum - 1 ; i ++) {
            if(m_NewDelayTask.DelayTime <= st_DelayTaskQueue[i].DelayTime) {
                for(j = g_u8DelayTaskNum - 1; j > i; j --) {
                    st_DelayTaskQueue[j].DelayTask = st_DelayTaskQueue[j - 1].DelayTask;
                    st_DelayTaskQueue[j].DelayTime = st_DelayTaskQueue[j - 1].DelayTime;
                }

                st_DelayTaskQueue[i].DelayTask = i_eTaskType;//����������ӵ�������
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
        if(g_u8DelayTaskNum >= 1) {//������ֻ����һ��������

            ResponseTheDelayTaskAndExecuteCmd(st_DelayTaskQueue[0].DelayTask);
            g_u8DelayTaskNum --;
            st_DelayTaskQueue[0].DelayTask = st_DelayTaskQueue[1].DelayTask;
            st_DelayTaskQueue[0].DelayTime = st_DelayTaskQueue[1].DelayTime;

            if(g_u8DelayTaskNum >= 2) {//�����л��ж������
                for(i = 1; i <= g_u8DelayTaskNum - 1; i++) {

                    st_DelayTaskQueue[i].DelayTask = st_DelayTaskQueue[i + 1].DelayTask;
                    //ִ����ǰ�������л���������ĵȴ��󣬺���ȴ�����ĵȴ�ʱ��Ҫ��������Ϊ׼
                    st_DelayTaskQueue[i].DelayTime = st_DelayTaskQueue[i + 1].DelayTime - st_DelayTaskQueue[0].DelayTime;
                }

                st_DelayTaskQueue[g_u8DelayTaskNum].DelayTask = TASK_MAX_NUM;//�����ʱ������Ϣ
                st_DelayTaskQueue[g_u8DelayTaskNum].DelayTime = 0xFFFF;
            } else {
                st_DelayTaskQueue[1].DelayTask = TASK_MAX_NUM;
                st_DelayTaskQueue[1].DelayTime = 0xFFFF;   //����ǰ����ֻ��һ�������������Ҫ����ǰ������ȴ�ʱ��
            }
        } else {
            break;
        }
    } while(st_DelayTaskQueue[0].DelayTime == 0);//�������뵱ǰ����ͬʱ��Ӧ������ִ����

    TIM_ITConfig(TASK_TIMER, TIM_IT_Update, DISABLE);
    TIM_Cmd(TASK_TIMER, DISABLE); //ʧ�ܶ�ʱ��6

    //������ʱ,
    if(g_u8DelayTaskNum >= 1) {
        TimerBaseStructure.TIM_Period = st_DelayTaskQueue[0].DelayTime; //�趨��������װֵ
        TIM_TimeBaseInit(TASK_TIMER, &TimerBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ

        TIM_GenerateEvent(TASK_TIMER, TIM_EventSource_Update);   //������������¼���������������
        TIM_ClearFlag(TASK_TIMER, TIM_FLAG_Update);             //�����־λ����ʱ��һ�򿪱���������¼��������������������ж�
        TIM_ITConfig(TASK_TIMER, TIM_IT_Update, ENABLE);
        TIM_Cmd(TASK_TIMER, ENABLE);  //ʹ�ܶ�ʱ��6
    }

    TIM_ClearITPendingBit(TASK_TIMER, TIM_IT_Update); //����жϱ�־λ
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
* Notes       : �ڴ˺����в�������ӡ��Ϣ����.
***************************************************************************************************
*/
static void ResponseTheDelayTaskAndExecuteCmd(TIMER_DELAY_TASK_TYPE_Typedef i_eTaskType)
{

    switch((u8)i_eTaskType) {

        case CAN_BUS_AUTO_RECONNECT_AFTER_30_SEC:
            SetCanBusOnlineFlag(DEF_YES);//���һ��CAN�Ƿ�����
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
