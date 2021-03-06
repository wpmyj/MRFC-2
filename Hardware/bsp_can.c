
/*
*********************************************************************************************************
*                                         APPLICATION CODE
*
*                      (c) Copyright 2016; Guangdong ENECO Science And Technology Co.,Ltd
*
*               All rights reserved.  Protected by international copyright laws.
*               Knowledge of the source code may NOT be used without authorization.
*********************************************************************************************************
*/
/*
*********************************************************************************************************
* Filename      : bsp_can.c
* Version       : V1.00
* Programmer(s) : JasonFan
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/
#include "bsp_can.h"
#include "app_system_run_cfg_parameters.h"
#include "bsp_delay_task_timer.h"
/*
***************************************************************************************************
*                                           MACRO DEFINITIONS
***************************************************************************************************
*/

//CAN总线数据发送状态
#define NOT_STARTED     0
#define COLLECTTING     1
#define READY           2
/*
***************************************************************************************************
*                                         FUNCTION PROTOTYPES
***************************************************************************************************
*/
static void CAN1_RX0_IRQHandler(void);
/*
***************************************************************************************************
*                                           LOCAL VARIABLES
***************************************************************************************************
*/
static uint8_t  g_u8CanMsgRxBuff[PRGM_RX_BUFF_SIZE];//从CAN接口接收到的待汇总的数据的缓冲区
static uint8_t  CanMsgRxStatu = NOT_STARTED;        //CAN总线接收状态

/*
***************************************************************************************************
*                                           GLOBAL VARIABLES
***************************************************************************************************
*/
uint8_t     g_eCAN_BusOnLineFlag = DEF_YES;
uint8_t     g_eCanMsgRxStatu = DEF_NO;//是否接收到信息

uint8_t     g_u8CanRxMsg[PRGM_RX_BUFF_SIZE];//汇总后CAN接收数据

/*
***************************************************************************************************
*                                          CAN1_Init()
*
* Description : The use of this funciton is to check the authorization of the system.
*
* Arguments   : none.
*
* Returns     : none.
*
* Notes       : none.
***************************************************************************************************
*/
void CAN1_Init(void)
{
    GPIO_InitTypeDef       GPIO_InitStructure;
    CAN_InitTypeDef        CAN_InitStructure;
    CAN_FilterInitTypeDef  CAN_FilterInitStructure;
#if CAN1_RX0_INT_ENABLE
    NVIC_InitTypeDef       NVIC_InitStructure;
#endif
    //时钟使能
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOD, ENABLE);

    GPIO_PinRemapConfig(GPIO_Remap2_CAN1, ENABLE);  //开重映射
    
    //CAN1 TX
    GPIO_InitStructure.GPIO_Pin = BSP_GPIOD_PIN1_CAN1_TX_PORT_NMB;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //复用推挽
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    //CAN1 RX
    GPIO_InitStructure.GPIO_Pin = BSP_GPIOD_PIN0_CAN1_RX_PORT_NMB;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//上拉输入
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    /* CAN1 单元设置 */
    CAN_InitStructure.CAN_TTCM = DISABLE; /* 时间触发禁止, 时间触发：CAN硬件的内部定时器被激活，并且被用于产生时间戳 */
    CAN_InitStructure.CAN_ABOM = DISABLE; /* 自动离线,一旦硬件监控到128次11个隐性位，就自动退出离线状态。在这里要软件设定后才能退出 */
    CAN_InitStructure.CAN_AWUM = ENABLE; /* 自动唤醒,有报文来的时候自动退出休眠   */
    CAN_InitStructure.CAN_NART = DISABLE; /* 报文重传, 如果错误一直传到成功止，否则只传一次 */
    CAN_InitStructure.CAN_RFLM = DISABLE; /* 接收FIFO锁定, 1--锁定后接收到新的报文摘不要，0--接收到新的报文则覆盖前一报文   */
    CAN_InitStructure.CAN_TXFP = ENABLE;  /* 发送优先级  0---由标识符决定  1---由发送请求顺序决定   */
    CAN_InitStructure.CAN_Mode = CAN_Mode_Normal; /*工作模式*/ //CAN_Mode_LoopBack、CAN_Mode_Normal
    
    CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;		//BTR-SJW 重新同步跳跃宽度 1个时间单元
    CAN_InitStructure.CAN_BS1 = CAN_BS1_3tq;		//BTR-TS1 时间段1 占用了12个时间单元
    CAN_InitStructure.CAN_BS2 = CAN_BS2_2tq;		//BTR-TS1 时间段2 占用了3个时间单元
    CAN_InitStructure.CAN_Prescaler = 120;		   	//BTR-BRP 波特率分频器  定义了时间单元的时间长度 36MHz/(1 + 3 + 2) / 120 = 50KHZ
    CAN_Init(CAN1, &CAN_InitStructure);	

    CAN_FilterInitStructure.CAN_FilterNumber = 0;     //过滤器组编号
    CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask; //工作在屏蔽位模式
    CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;                 
    
    CAN_FilterInitStructure.CAN_FilterIdHigh =((((u32)g_u16GlobalNetWorkId << 21) & 0xFFFF0000) >>16); //过滤掉不是发送给本机的数据帧
    CAN_FilterInitStructure.CAN_FilterIdLow = ((((u32)g_u16GlobalNetWorkId << 21) | CAN_ID_STD | CAN_RTR_DATA) & 0xFFFF); //确保收到的是标准数据帧
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0xFFFF;//所有的位全部必须匹配
    CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0xFFFF; 

    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_FIFO0;  //过滤器0关联到FIFO0
    CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;//激活过滤器0，激活后会自动退出初始化模式
    CAN_FilterInit(&CAN_FilterInitStructure);

#if CAN1_RX0_INT_ENABLE
    NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;     // 主优先级为1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;            // 次优先级为0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);/* 挂号中断, 进入中断后读FIFO的报文函数释放报文清中断标志 */
    BSP_IntVectSet(BSP_INT_ID_CAN1_RX0, CAN1_RX0_IRQHandler);
    BSP_IntEn(BSP_INT_ID_CAN1_RX0);
#endif
}

/*
***************************************************************************************************
*                                 CANx_Send_Msg()
*
* Description : can发送一帧数据(固定格式:标准ID + 扩展ID + 标准帧 + 数据帧)
*
* Arguments   : CANx:CAN接口,CAN1或CAN2;len:数据长度(最大为8);msg:数据指针,最大为8个字节;ID:标示符.
*
* Returns     : 返回值:0,成功;其他,失败;
*
* Caller(s)   : Application.
*
***************************************************************************************************
*/
uint8_t CANx_Send_Msg(CAN_TypeDef * CANx, Message *m)
{
    uint8_t u8SendStatu = 0;
    uint8_t i;
    uint16_t u16SendErrCount = 0;
    CanTxMsg TxMessage;

    TxMessage.StdId = (uint32_t)(m->cob_id);// 标准标识符11位,即发送优先级
    TxMessage.ExtId = 0x00;         // 设置扩展标示符
    TxMessage.RTR = m->rtr;         //消息类型为数据帧，一帧8位
    TxMessage.IDE = CAN_ID_STD;     // 使用标准标识符
    TxMessage.DLC = m->len;         // 要发送的数据长度

    for(i = 0; i < m->len; i++) {
        TxMessage.Data[i] = m->data[i];
    }

    while(CAN_TxStatus_NoMailBox == CAN_Transmit(CANx, &TxMessage))
	{	
		if(u16SendErrCount >= 0xFFF){
			u8SendStatu = 1;
//            APP_TRACE_INFO(("CAN send error..\r\n"));
			break;
		}else{
			u16SendErrCount ++;
		}
	}

    return u8SendStatu;
}

/*
***************************************************************************************************
*                                 SendCanMsgContainNodeId()
*
* Description : The use of this funciton is concordance the node id inf to the can message.
*
* Arguments   : none.
*
* Returns     : none
*
* Notes       : none.
***************************************************************************************************
*/
uint8_t SendCanMsgContainNodeId(uint32_t i_Msglen, uint8_t *msg, uint8_t i_NodeId)
{
    OS_ERR err;
    uint8_t ret;
    uint32_t i, j;
    uint8_t  u8SendErrCount = 0;
    uint8_t  LastTimeSendErrFlag = NO;
    Message ProcessedData;   

    ProcessedData.cob_id = i_NodeId ;//CAN ID
    ProcessedData.rtr = CAN_RTR_DATA;

    for(i = 0; i < (i_Msglen / 8 + ((i_Msglen % 8) ? 1 : 0)); i++) { //长消息拆包的分包数,8为CAN消息的标准包长度
        if(LastTimeSendErrFlag == NO) {
            ProcessedData.len = ((i_Msglen - i * 8) > 8) ? 8 : (i_Msglen - i * 8);//每个短包的数据长度，除最后一包需要另算外，其余均为8

            for(j = 0; j < ProcessedData.len; j++) {
                ProcessedData.data[j] = msg[i * 8 + j];
            }        
        } else {
            //上次发送出错，直接发送数组即可，不需要重新复制
        }

        ret = CANx_Send_Msg(CAN1, &ProcessedData);

        if(ret != 0) {
            LastTimeSendErrFlag = YES;
            i--;
            OSTimeDlyHMSM(0, 0, 0, 100,
                          OS_OPT_TIME_HMSM_STRICT,
                          &err);

            if(++u8SendErrCount >= 10) {
                g_eCAN_BusOnLineFlag = NO;
                APP_TRACE_INFO(("CAN bus has dropped,send err..\r\n"));
                StartTimerDelayTask(CAN_BUS_AUTO_RECONNECT_AFTER_30_SEC, 30000); //掉线后定时监测是否在线
                break;
            }
        } else {
            LastTimeSendErrFlag = NO;//发送成功，清零该标志，进入下一次循环，发送该数据
        }   
    }
    return ret;
}

/*
***************************************************************************************************
*                                   Can_Receive_Msg()
*
* Description : can口接收数据.
*
* Arguments   : CANx:CAN接口,CAN1或CAN2;i_u8RxBuf:数据缓存区;.
*
* Returns     : 返回值:0,无数据被收到;其他,接收的数据长度.
*
* Note        :
*
***************************************************************************************************
*/
uint8_t CANx_Receive_Msg(CAN_TypeDef *CANx, uint8_t *i_u8RxBuf)
{
    uint32_t i;
    CanRxMsg RxMessage;

    if(CAN_MessagePending(CANx, CAN_FIFO0) == 0) {
        return 0;    //没有接收到数据,直接退出
    }

    CAN_Receive(CANx, CAN_FIFO0, &RxMessage);//读取数据

    for(i = 0; i < 8; i++) {
        i_u8RxBuf[i] = RxMessage.Data[i];
    }

    return RxMessage.DLC;
}

/*
***************************************************************************************************
*                                          CAN1_RX1_IRQHandler()
*
* Description : The use of this funciton is to check the authorization of the system.
*
* Arguments   : none.
*
* Returns     : none
*
* Notes       : none.
***************************************************************************************************
*/
#if CAN1_RX0_INT_ENABLE

static void CAN1_RX0_IRQHandler(void)
{
    static   uint8_t i = 0;
    uint8_t  j = 0;
    CanRxMsg CAN1_Rx_Msg; //数据链路层的数据包定义
    OS_ERR err;
    CPU_SR_ALLOC();
    
    CAN_Receive(CAN1, CAN_FIFO0, &(CAN1_Rx_Msg));//从CAN1 FIFO0接收数据链路层的CAN数据,注意此处提取报文后会自动清除中断
    
    if((CAN1_Rx_Msg.DLC == 8) 
        && (CAN1_Rx_Msg.Data[0] == 0xFC) 
        && (CAN1_Rx_Msg.Data[1] == 0xFD) 
        && (CAN1_Rx_Msg.Data[2] == 0xFE)) { //收到的是报头帧数据
            
        for(i = 0; i < CAN1_Rx_Msg.DLC; i++) {
            g_u8CanMsgRxBuff[i] = CAN1_Rx_Msg.Data[i];
        }

        CanMsgRxStatu = COLLECTTING;
    } else { //收到的是非报头的后续帧数据
        if(CanMsgRxStatu == COLLECTTING) { //否则将后续的字节存入到CAN消息缓冲区
            for(j = 0; j < CAN1_Rx_Msg.DLC;) {
                g_u8CanMsgRxBuff[i] = CAN1_Rx_Msg.Data[j];
                j++;
                i++;

                if(i >= PRGM_RX_BUFF_SIZE) {
                    CanMsgRxStatu = READY;//数据收集完成
                    break;
                }
            }
        } else { //未收集数据时收到了未知数据
            CanMsgRxStatu = NOT_STARTED;
        }
    }

    if(CanMsgRxStatu == READY) {//汇总后的数据包接收完成后再发送到上位机
        if(g_eCanMsgRxStatu == NO) {
            CPU_CRITICAL_ENTER();
            for(j = 0; j < PRGM_RX_BUFF_SIZE; j++) {
                g_u8CanRxMsg[j] = g_u8CanMsgRxBuff[j];
            }

            OSTimeDlyResume(&CommTaskTCB,
                            &err);
            g_eCanMsgRxStatu = YES;
            CPU_CRITICAL_EXIT();
        }

        CanMsgRxStatu = NOT_STARTED;
        g_eCAN_BusOnLineFlag = YES;//原为注释状态
    } else {
        //否则未接收完，不予解析
    }
}

#endif

/*
***************************************************************************************************
*                                          SetCanBusOnlineFlag()
*
* Description : The use of this funciton is to check the authorization of the system.
*
* Arguments   : none.
*
* Returns     : none.
*
* Notes       : none.
***************************************************************************************************
*/
void SetCanBusOnlineFlag(uint8_t i_NewStatus)
{
    g_eCAN_BusOnLineFlag = i_NewStatus;
}

uint8_t GetCanBusOnlineFlag(void)
{
    return g_eCAN_BusOnLineFlag;
}
/******************* (C) COPYRIGHT 2016 Guangdong ENECO POWER *****END OF FILE****/
