
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
* Programmer(s) : FanJun
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/
#include "bsp_can.h"

/*
***************************************************************************************************
*                                           MACRO DEFINITIONS
***************************************************************************************************
*/
#define CAN_BAUD_NUM    20

//CAN总线数据发送状态
#define NOT_STARTED     0
#define COLLECTTING     1
#define READY           2
/*
***************************************************************************************************
*                                         FUNCTION PROTOTYPES
***************************************************************************************************
*/
static uint8_t  CAN1_Init(uint8_t i_u8CanMode);

static void     CAN_Baud_Process(unsigned int Baud, CAN_InitTypeDef *CAN_InitStructure);

static void     CAN1_RX0_IRQHandler(void);
/*
***************************************************************************************************
*                                           LOCAL VARIABLES
***************************************************************************************************
*/

//CAN总线的波特率 = PCLK1/((CAN_SJW +CAN_BS1 +  CAN_BS2) * CAN_Prescaler)
const unsigned int CAN_baud_table[20][5] = {
    /*波特率， CAN_SJW，    CAN_BS1，       CAN_BS2，  CAN_Prescaler */
    {5,     CAN_SJW_1tq,    CAN_BS1_13tq,   CAN_BS2_2tq,    450  },     //未通
    {10,    CAN_SJW_1tq,    CAN_BS1_6tq,    CAN_BS2_2tq,    400  },     //未通
    {15,    CAN_SJW_1tq,    CAN_BS1_13tq,   CAN_BS2_2tq,    150  },     //15K
    {20,    CAN_SJW_1tq,    CAN_BS1_6tq,    CAN_BS2_2tq,    200  },     //20k
    {25,    CAN_SJW_1tq,    CAN_BS1_13tq,   CAN_BS2_2tq,    90   },     //25k
    {40,    CAN_SJW_1tq,    CAN_BS1_6tq,    CAN_BS2_2tq,    100  },     //40k
    {50,    CAN_SJW_1tq,    CAN_BS1_3tq,    CAN_BS2_2tq,    120  },    //50k
    {62,    CAN_SJW_1tq,    CAN_BS1_13tq,   CAN_BS2_2tq,    36   },     //62.5k
    {80,    CAN_SJW_1tq,    CAN_BS1_6tq,    CAN_BS2_2tq,    50   },     //80k
    {100,   CAN_SJW_1tq,    CAN_BS1_5tq,    CAN_BS2_2tq,    45   },     //100K
    {125,   CAN_SJW_1tq,    CAN_BS1_13tq,   CAN_BS2_2tq,    18   },     //125K
    {200,   CAN_SJW_1tq,    CAN_BS1_6tq,    CAN_BS2_2tq,    20   },     //200K
    {250,   CAN_SJW_1tq,    CAN_BS1_13tq,   CAN_BS2_2tq,    9    },     //250k
    {400,   CAN_SJW_1tq,    CAN_BS1_6tq,    CAN_BS2_2tq,    10   },     //400K
    {500,   CAN_SJW_1tq,    CAN_BS1_5tq,    CAN_BS2_2tq,    9    },     //500K
    {666,   CAN_SJW_1tq,    CAN_BS1_5tq,    CAN_BS2_2tq,    8    },     //未通
    {800,   CAN_SJW_1tq,    CAN_BS1_6tq,    CAN_BS2_2tq,    5    },     //800K
    {1000,  CAN_SJW_1tq,    CAN_BS1_6tq,    CAN_BS2_2tq,    4    },     //1000K
};

static uint8_t  g_u8CanMsgRxBuff[PRGM_RX_BUFF_SIZE];//从CAN接口接收到的数据的汇总缓冲区
static uint8_t  CanMsgRxStatu = NOT_STARTED;       //CAN总线接收状态
/*
***************************************************************************************************
*                                           GLOBAL VARIABLES
***************************************************************************************************
*/
WHETHER_TYPE_VARIABLE_Typedef       g_eCAN_BusOnLineFlag = NO;
WHETHER_TYPE_VARIABLE_Typedef       g_eCanMsgRxStatu = NO;//是否接收到信息

uint8_t     g_u8CanRxMsg[PRGM_RX_BUFF_SIZE];//汇总后CAN接收数据
/*
***************************************************************************************************
*                               CAN_Configuration()
*
* Description : The use of this funciton is to Configuration the CAN.
*
* Arguments   : none.
*
* Returns     : none
*
* Notes       : none.
***************************************************************************************************
*/
void CAN_Configuration(void)
{
    uint8_t err_code;
    err_code = CAN1_Init(CAN_Mode_Normal);//CAN_Mode_LoopBack
    if( 0 ==err_code){
        APP_TRACE_INFO(("CAN 1 init successed......\n\r"));
    }else{
        APP_TRACE_INFO(("CAN 1 init failed......\n\r"));
    }
}

/*
***************************************************************************************************
*                                          CAN_Configuration()
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
static uint8_t CAN1_Init(uint8_t i_u8CanMode)
{
    GPIO_InitTypeDef       GPIO_InitStructure;
    CAN_InitTypeDef        CAN_InitStructure;
    CAN_FilterInitTypeDef  CAN_FilterInitStructure;
#if CAN1_RX0_INT_ENABLE
    NVIC_InitTypeDef  NVIC_InitStructure;
#endif
    //时钟使能
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOD, ENABLE);

    GPIO_PinRemapConfig(GPIO_Remap1_CAN1, ENABLE);//引脚重定义
//    GPIO_PinRemapConfig(GPIO_Remap2_CAN1, ENABLE);  
    
    //CAN1 TX
    GPIO_InitStructure.GPIO_Pin = BSP_GPIOD_PIN1_CAN1_TX_PORT_NMB;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //复用推挽
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    //CAN1 RX
    GPIO_InitStructure.GPIO_Pin = BSP_GPIOD_PIN0_CAN1_RX_PORT_NMB;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//上拉输入
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    /*CAN 1 初始化*/
    CAN_DeInit(CAN1);  //将寄存器重设为缺省值
    CAN_StructInit(&CAN_InitStructure);

    /* CAN1 单元设置 */
    CAN_InitStructure.CAN_TTCM = DISABLE; /* 时间触发禁止, 时间触发：CAN硬件的内部定时器被激活，并且被用于产生时间戳 */
    CAN_InitStructure.CAN_ABOM = ENABLE; /* 自动离线,一旦硬件监控到128次11个隐性位，就自动退出离线状态。在这里要软件设定后才能退出 */
    CAN_InitStructure.CAN_AWUM = ENABLE; /* 自动唤醒,有报文来的时候自动退出休眠   */
    CAN_InitStructure.CAN_NART = DISABLE; /* 报文重传, 如果错误一直传到成功止，否则只传一次 */
    CAN_InitStructure.CAN_RFLM = DISABLE; /* 接收FIFO锁定, 1--锁定后接收到新的报文摘不要，0--接收到新的报文则覆盖前一报文   */
    CAN_InitStructure.CAN_TXFP = ENABLE;  /* 发送优先级  0---由标识符决定  1---由发送请求顺序决定   */
    CAN_InitStructure.CAN_Mode = i_u8CanMode; /*工作模式*/

    CAN_Baud_Process(50, &CAN_InitStructure); //波特率50K

    if(CAN_Init(CAN1, &CAN_InitStructure) == CANINITFAILED) {
        /* 初始化时先设置CAN_MCR的初始化位 */
        /* 然后查看硬件是否真的设置了CAN_MSR的初始化位来确认是否进入了初始化模式  */
         return 1;
    }

    CAN_FilterInitStructure.CAN_FilterNumber = 0;     //过滤器组编号
    CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask; //工作在屏蔽位模式
    CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit; //
    
    CAN_FilterInitStructure.CAN_FilterIdHigh =  ((uint32_t)GLOBAL_NET_WORK_ID << 5) & 0xFFFF; //要过滤的ID高位，过滤掉不是发送给本机的数据帧
    CAN_FilterInitStructure.CAN_FilterIdLow = (( CAN_ID_STD | CAN_RTR_DATA) & 0xFFFF); //确保收到的是标准数据帧
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0xFFFF;//所有的位全部必须匹配(或只判定nodeID位)
    CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0xFFFF;                 

//    CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;     //优先级
//    CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
//    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;       //32位标示符高位
//    CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_FIFO0;  //过滤器0关联到FIFO0
    CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;//激活过滤器0，激活后会自动退出初始化模式
    CAN_FilterInit(&CAN_FilterInitStructure);

#if CAN1_RX0_INT_ENABLE
    NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;     // 主优先级为1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // 次优先级为0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);/* 挂号中断, 进入中断后读FIFO的报文函数释放报文清中断标志 */
    BSP_IntVectSet(BSP_INT_ID_CAN1_RX0, CAN1_RX0_IRQHandler);
    BSP_IntEn(BSP_INT_ID_CAN1_RX0);
#endif
    return 0;
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
#if 0
uint8_t CANx_Send_Msg(CAN_TypeDef *CANx, u16 ID, uint8_t *i_u8TxMsg, uint8_t i_u8LenOfFrame)
{
    uint8_t u8Mbox = 0;
    uint8_t u8SendStatus = 0;
    u16 i = 0;
    CanTxMsg TxMessage;
    
    TxMessage.StdId = ID;                   // 标准标识符11位,即发送优先级
    TxMessage.ExtId = 0x00;                 // 设置扩展标示符
    TxMessage.IDE = CAN_ID_STD;             // 使用标准标识符
    TxMessage.RTR = CAN_RTR_DATA;           //消息类型为数据帧，一帧8位
    TxMessage.DLC = i_u8LenOfFrame;         // 要发送的数据长度

//    TxMessage.StdId = (uint32_t)(m->cob_id);
//    TxMessage.ExtId = 0x00;
//    TxMessage.RTR = m->rtr;
//    TxMessage.IDE = CAN_ID_STD;
//    TxMessage.DLC = m->len;
    
    for(i = 0; i < i_u8LenOfFrame; i++) {
        TxMessage.Data[i] = i_u8TxMsg[i];
    }
    
    u8Mbox = CAN_Transmit(CANx, &TxMessage);
    if(u8Mbox != CAN_TxStatus_NoMailBox) 
	{
//		APP_TRACE_INFO(("MailBox Num: %d\r\n", ret));
		u8SendStatus = 0;
	}
	else
	{		
//		APP_TRACE_INFO(("MailBox has no empty space!\r\n"));
		u8SendStatus = 1;
	}

    return u8SendStatus;
}

#endif
/*
***************************************************************************************************
*                                          CANx_Send_Msg()
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
uint8_t CANx_Send_Msg(CAN_TypeDef * CANx, Message *m)
{
    uint8_t u8SendStatu = 0;
    uint8_t ret;
    uint8_t i;
    CanTxMsg TxMessage;

    TxMessage.StdId = (uint32_t)(m->cob_id);// 标准标识符11位,即发送优先级
    TxMessage.ExtId = 0x00;         // 设置扩展标示符
    TxMessage.RTR = m->rtr;         //消息类型为数据帧，一帧8位
    TxMessage.IDE = CAN_ID_STD;     // 使用标准标识符
    TxMessage.DLC = m->len;         // 要发送的数据长度

    for(i = 0; i < m->len; i++) {
        TxMessage.Data[i] = m->data[i];
    }

    ret = CAN_Transmit(CAN1, &TxMessage);

    if(ret != CAN_TxStatus_NoMailBox) {
        u8SendStatu = 0;
//		APP_TRACE_INFO(("MailBox Num: %d\r\n", ret));
    } else {
        u8SendStatu = 1;
//		APP_TRACE_INFO(("MailBox has no empty space!\r\n"));
    }

    return u8SendStatu;
}

/*
***************************************************************************************************
*                                 SendCanMsgContainNodeId()
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
uint8_t SendCanMsgContainNodeId(uint32_t i_Msglen, uint8_t *msg, uint8_t i_NodeId)
{
    OS_ERR err;
    uint8_t ret;
    uint32_t i, j;
    uint8_t  u8SendErrCount = 0;
    uint8_t  LastTimeSendErrFlag = NO;
    Message ProcessedData;   

    ProcessedData.cob_id = i_NodeId | CAN_MSG_TX_FLAG;
    ProcessedData.rtr = CAN_RTR_DATA;

    for(i = 0; i < (i_Msglen / 8 + ((i_Msglen % 8) ? 1 : 0)); i++) { //长消息拆包的分包数,8为CAN消息的标准包长度
        if(LastTimeSendErrFlag == NO) {
            ProcessedData.len = ((i_Msglen - i * 8) > 8) ? 8 : (i_Msglen - i * 8);//每个短包的数据长度，除最后一包需要另算外，其余均为8

            for(j = 0; j < ProcessedData.len; j++) {
                ProcessedData.data[j] = msg[i * 8 + j];
//                APP_TRACE_INFO(("ProcessedData.data %x:..\r\n",ProcessedData.data[j]));
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
//              g_eCAN_BusOnLineFlag = OFF;
//                APP_TRACE_INFO(("CAN bus has dropped,send err..\r\n"));
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
*                                          CAN_Baud_Process()
*
* Description : 计算CAN的波特率.
*
* Arguments   : CAN_SJW : CAN_SJW_1tq - CAN_SJW_4tq   不能比任何一相位缓冲段长
*               CAN_BS1 : CAN_BS1_1tq - CAN_BS1_16tq
*               CAN_BS2 : CAN_BS2_1tq - CAN_BS2_8tq
*               CAN_Prescaler : 1 - 1024.
*
* Returns     : none
*
* Notes       : baud = 36 / (CAN_SJW + CAN_BS1 + CAN_BS2) / CAN_Prescaler.
***************************************************************************************************
*/

static void CAN_Baud_Process(unsigned int Baud, CAN_InitTypeDef *CAN_InitStructure)
{
    unsigned int i = 0;

    for(i = 0; i < CAN_BAUD_NUM; i ++) {
        if(Baud == CAN_baud_table[i][0]) {
            CAN_InitStructure->CAN_SJW = CAN_baud_table[i][1];
            CAN_InitStructure->CAN_BS1 = CAN_baud_table[i][2];
            CAN_InitStructure->CAN_BS2 = CAN_baud_table[i][3];
            CAN_InitStructure->CAN_Prescaler = CAN_baud_table[i][4];
            break;
        }
    }
}

/*
***************************************************************************************************
*                                          CAN1_RX0_IRQHandler()
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

void CAN1_RX0_IRQHandler(void)
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

            OSTimeDlyResume(&CommunicateTaskTCB,
                            &err);
            g_eCanMsgRxStatu = YES;
            CPU_CRITICAL_EXIT();
        }

        CanMsgRxStatu = NOT_STARTED;
//      g_eCAN_BusOnLineFlag = ON;
    } else {
        //否则未接收完，不予解析
    }
}

#endif

/******************* (C) COPYRIGHT 2015 Personal Electronics *****END OF FILE****/
