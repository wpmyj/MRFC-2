
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
#define CAN_BAUD_NUM 20  //�����ʸ���

#define SLAVE_ID 0x0001 //�ӻ�ID
/*
***************************************************************************************************
*                                         FUNCTION PROTOTYPES
***************************************************************************************************
*/
static uint8_t CAN1_Init(uint8_t i_u8CanMode);
static uint8_t CAN2_Init(uint8_t i_u8CanMode);

static void CAN_Baud_Process(unsigned int Baud, CAN_InitTypeDef *CAN_InitStructure);

static void CAN1_RX0_IRQHandler(void);
static void CAN2_RX0_IRQHandler(void);
/*
***************************************************************************************************
*                                           LOCAL VARIABLES
***************************************************************************************************
*/

//CAN���ߵĲ����� = PCLK1/((CAN_SJW +CAN_BS1 +  CAN_BS2) * CAN_Prescaler)
const unsigned int CAN_baud_table[CAN_BAUD_NUM][5] = {
    /*�����ʣ� CAN_SJW��    CAN_BS1��       CAN_BS2��  CAN_Prescaler */
    {5,     CAN_SJW_1tq,    CAN_BS1_13tq,   CAN_BS2_2tq,    450  },     //δͨ
    {10,    CAN_SJW_1tq,    CAN_BS1_6tq,    CAN_BS2_2tq,    400  },     //δͨ
    {15,    CAN_SJW_1tq,    CAN_BS1_13tq,   CAN_BS2_2tq,    150  },     //15K
    {20,    CAN_SJW_1tq,    CAN_BS1_6tq,    CAN_BS2_2tq,    200  },     //20k
    {25,    CAN_SJW_1tq,    CAN_BS1_13tq,   CAN_BS2_2tq,    90   },     //25k
    {40,    CAN_SJW_1tq,    CAN_BS1_6tq,    CAN_BS2_2tq,    100  },     //40k
    {50,    CAN_SJW_1tq,    CAN_BS1_13tq,   CAN_BS2_2tq,    45   },     //50k
    {62,    CAN_SJW_1tq,    CAN_BS1_13tq,   CAN_BS2_2tq,    36   },     //62.5k
    {80,    CAN_SJW_1tq,    CAN_BS1_6tq,    CAN_BS2_2tq,    50   },     //80k
    {100,   CAN_SJW_1tq,    CAN_BS1_5tq,    CAN_BS2_2tq,    45   },     //100K
    {125,   CAN_SJW_1tq,    CAN_BS1_13tq,   CAN_BS2_2tq,    18   },     //125K
    {200,   CAN_SJW_1tq,    CAN_BS1_6tq,    CAN_BS2_2tq,    20   },     //200K
    {250,   CAN_SJW_1tq,    CAN_BS1_13tq,   CAN_BS2_2tq,    9    },     //250k
    {400,   CAN_SJW_1tq,    CAN_BS1_6tq,    CAN_BS2_2tq,    10   },     //400K
    {500,   CAN_SJW_1tq,    CAN_BS1_5tq,    CAN_BS2_2tq,    9    },     //500K
    {666,   CAN_SJW_1tq,    CAN_BS1_5tq,    CAN_BS2_2tq,    8    },     //δͨ
    {800,   CAN_SJW_1tq,    CAN_BS1_6tq,    CAN_BS2_2tq,    5    },     //800K
    {1000,  CAN_SJW_1tq,    CAN_BS1_6tq,    CAN_BS2_2tq,    4    },     //1000K
};

/*
***************************************************************************************************
*                                           GLOBAL VARIABLES
***************************************************************************************************
*/
uint8_t CAN1_RX_STA = 0;
uint8_t CAN2_RX_STA = 0;

uint8_t CAN1_RX_DATA_BUF[8] = {0};
uint8_t CAN2_RX_DATA_BUF[8] = {0};

/*
***************************************************************************************************
*                                          CAN_Configuration()
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
        APP_TRACE_INFO(("CAN 1 init successed...\r\n"));
    }else{
        APP_TRACE_INFO(("CAN 1 init failed...\r\n"));
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
    //ʱ��ʹ��
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA, ENABLE);

    //CAN1 TX
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //��������
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    //CAN1 RX
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//��������
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /*CAN 1 ��ʼ��*/
    CAN_DeInit(CAN1);  //���Ĵ�������Ϊȱʡֵ
    CAN_StructInit(&CAN_InitStructure);

    /* CAN1 ��Ԫ���� */
    CAN_InitStructure.CAN_TTCM = DISABLE; /* ʱ�䴥����ֹ, ʱ�䴥����CANӲ�����ڲ���ʱ����������ұ����ڲ���ʱ��� */
    CAN_InitStructure.CAN_ABOM = DISABLE; /* �Զ����߽�ֹ���Զ����ߣ�һ��Ӳ����ص�128��11������λ�����Զ��˳�����״̬��������Ҫ����趨������˳� */
    CAN_InitStructure.CAN_AWUM = DISABLE; /* �Զ����ѽ�ֹ���б�������ʱ���Զ��˳�����   */
    CAN_InitStructure.CAN_NART = DISABLE; /* �����ش�, �������һֱ�����ɹ�ֹ������ֻ��һ�� */
    CAN_InitStructure.CAN_RFLM = DISABLE; /* ����FIFO����, 1--��������յ��µı���ժ��Ҫ��0--���յ��µı����򸲸�ǰһ����   */
    CAN_InitStructure.CAN_TXFP = ENABLE;  /* �������ȼ�  0---�ɱ�ʶ������  1---�ɷ�������˳�����   */
    CAN_InitStructure.CAN_Mode = i_u8CanMode; /*����ģʽ*/

    CAN_Baud_Process(500, &CAN_InitStructure); //������500K

    if(CAN_Init(CAN1, &CAN_InitStructure) == CANINITFAILED) {
        /* ��ʼ��ʱ������CAN_MCR�ĳ�ʼ��λ */
        /* Ȼ��鿴Ӳ���Ƿ����������CAN_MSR�ĳ�ʼ��λ��ȷ���Ƿ�����˳�ʼ��ģʽ  */
         return 1;
    }

    CAN_FilterInitStructure.CAN_FilterNumber = 0;     //����������
    CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask; //����������λģʽ
    CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit; //
    
    CAN_FilterInitStructure.CAN_FilterIdHigh =  ((u32)SLAVE_ID << 5) & 0xFFFF; //Ҫ���˵�ID��λ�����˵����Ƿ��͸�����������֡
    CAN_FilterInitStructure.CAN_FilterIdLow = (( CAN_ID_STD | CAN_RTR_DATA) & 0xFFFF); //ȷ���յ����Ǳ�׼����֡
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0xFFFF;         //����ƥ���λ(��ֻ�ж�nodeIDλ)
    CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0xFFFF;                 

//    CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;     //���ȼ�
//    CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
//    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;       //32λ��ʾ����λ
//    CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_FIFO0;  //������0������FIFO0
    CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;//���������0���������Զ��˳���ʼ��ģʽ
    CAN_FilterInit(&CAN_FilterInitStructure);

#if CAN1_RX0_INT_ENABLE
    NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;     // �����ȼ�Ϊ1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // �����ȼ�Ϊ0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);/* �Һ��ж�, �����жϺ��FIFO�ı��ĺ����ͷű������жϱ�־ */
    BSP_IntVectSet(BSP_INT_ID_CAN1_RX0, CAN1_RX0_IRQHandler);
    BSP_IntEn(BSP_INT_ID_CAN1_RX0);
#endif
    return 0;
}

/*
***************************************************************************************************
*                                 CANx_Send_Msg()
*
* Description : can����һ֡����(�̶���ʽ:��׼ID + ��չID + ��׼֡ + ����֡)
*
* Arguments   : CANx:CAN�ӿ�,CAN1��CAN2;len:���ݳ���(���Ϊ8);msg:����ָ��,���Ϊ8���ֽ�;ID:��ʾ��.
*
* Returns     : ����ֵ:0,�ɹ�;����,ʧ��;
*
* Caller(s)   : Application.
*
***************************************************************************************************
*/
u8 CANx_Send_Msg(CAN_TypeDef *CANx, u16 ID, u8 *i_u8TxMsg, u8 i_u8LenOfFrame)
{
    u8 u8Mbox = 0;
    u8 u8SendStatus = 0;
    u16 i = 0;
    CanTxMsg TxMessage;
    TxMessage.StdId = ID;                   // ��׼��ʶ��11λ,���������ȼ�
    TxMessage.ExtId = 0x00;                 // ������չ��ʾ��
    TxMessage.IDE = CAN_ID_STD;             // ʹ�ñ�׼��ʶ��
    TxMessage.RTR = CAN_RTR_DATA;           //��Ϣ����Ϊ����֡��һ֡8λ
    TxMessage.DLC = i_u8LenOfFrame;         // Ҫ���͵����ݳ���

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

/*
***************************************************************************************************
*                                   Can_Receive_Msg()
*
* Description : can�ڽ�������.
*
* Arguments   : CANx:CAN�ӿ�,CAN1��CAN2;i_u8RxBuf:���ݻ�����;.
*
* Returns     : ����ֵ:0,�����ݱ��յ�;����,���յ����ݳ���.
*
* Note        :
*
***************************************************************************************************
*/
u8 CANx_Receive_Msg(CAN_TypeDef *CANx, u8 *i_u8RxBuf)
{
    u32 i;
    CanRxMsg RxMessage;

    if(CAN_MessagePending(CANx, CAN_FIFO0) == 0) {
        return 0;    //û�н��յ�����,ֱ���˳�
    }

    CAN_Receive(CANx, CAN_FIFO0, &RxMessage);//��ȡ����

    for(i = 0; i < 8; i++) {
        i_u8RxBuf[i] = RxMessage.Data[i];
    }

    return RxMessage.DLC;
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

static void CAN1_RX0_IRQHandler(void)
{
    CanRxMsg RxMessage;
    unsigned char i;

    CAN_Receive(CAN1, CAN_FIFO0, &RxMessage); /* �˴���ȡ���ĺ���Զ�����ж�*/

    for(i = 0; i < 8; i ++) {
        CAN1_RX_DATA_BUF[i] = RxMessage.Data[i];
    }
    CAN1_RX_STA = 1;
}
#endif

/*
***************************************************************************************************
*                                          CAN_Baud_Process()
*
* Description : ����CAN�Ĳ�����.
*
* Arguments   : CAN_SJW : CAN_SJW_1tq - CAN_SJW_4tq   ���ܱ��κ�һ��λ����γ�
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



#if 0

#include "canfestival.h"
#include "GlobalVar.h"

extern      SWITCH_TYPE_VARIABLE_Typedef            g_eCAN_BusOnLineFlag;
CO_Data *CANOpenMasterObject = (CO_Data *) &ObjDict_Data;


unsigned char canSend(CAN_PORT CANx, Message *m)
{
//  OS_ERR err;
//  uint8_t u8SendErrCount = 0 ;
    uint8_t u8SendStatu = 0;

    unsigned char ret;
    unsigned char i;
    CanTxMsg TxMessage;

    TxMessage.StdId = (uint32_t)(m->cob_id);
    TxMessage.ExtId = 0x00;
    TxMessage.RTR = m->rtr;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.DLC = m->len;

    for(i = 0; i < m->len; i++) {
        TxMessage.Data[i] = m->data[i];
    }

    ret = CAN_Transmit(CAN1, &TxMessage);

    if(ret != CAN_TxStatus_NoMailBox) {
//      APP_TRACE_INFO(("MailBox Num: %d\r\n", ret));
        u8SendStatu = 0;
    } else {
//      APP_TRACE_INFO(("MailBox has no empty space!\r\n"));
        u8SendStatu = 1;
    }

    return u8SendStatu;
}


u8 SendMsgThroughCAN(u32 i_Msglen, u8 *msg, u8 i_NodeId)
{
    OS_ERR err;
    u32 i, j;
    u8  u8SendErrCount = 0;
    uint8_t LastTimeSendErrFlag = NO;
    Message stStdMsg;
    u8 ret;

    stStdMsg.cob_id = i_NodeId | CAN_MSG_TX_FLAG;
    stStdMsg.rtr = CAN_RTR_DATA;

    for(i = 0; i < (i_Msglen / 8 + ((i_Msglen % 8) ? 1 : 0)); i++) { //����Ϣ����ķְ���,8ΪCAN��Ϣ�ı�׼������
        if(LastTimeSendErrFlag == NO) {
            stStdMsg.len = ((i_Msglen - i * 8) > 8) ? 8 : (i_Msglen - i * 8);//ÿ���̰������ݳ��ȣ������һ����Ҫ�����⣬�����Ϊ8

            for(j = 0; j < stStdMsg.len; j++) {
                stStdMsg.data[j] = msg[i * 8 + j];
            }

            /*          if(stStdMsg.len < 8)//���жϵ������ݰ�β������β������NodeID�������������
                        {
                            stStdMsg.len++;
                            stStdMsg.data[stStdMsg.len - 1] = i_NodeId;
                        }
                        else
                        {}//������Ҫ���⴦��
            */
        } else {
        }//�ϴη��ͳ���ֱ�ӷ������鼴�ɣ�����Ҫ���¸���

        ret = canSend(CAN1, &stStdMsg);

        if(ret != 0) {
            LastTimeSendErrFlag = YES;
            i--;
            OSTimeDlyHMSM(0, 0, 0, 100,
                          OS_OPT_TIME_HMSM_STRICT,
                          &err);

            if(++u8SendErrCount >= 10) {
//              g_eCAN_BusOnLineFlag = OFF;
                APP_TRACE_INFO(("CAN���ߵ��ߣ����ͳ���\r\n"));
                break;
            }
        } else {
            LastTimeSendErrFlag = NO;//���ͳɹ�������ñ�־��������һ��ѭ�������͸�����
        }
    }

    return ret;
}

#define NOT_STARTED     0
#define COLLECTTING     1
#define READY           2


extern  OS_TCB      CommunicateTaskTCB;

uint8_t     g_u8CanRxMsg[PRGM_RX_BUFF_SIZE];//��ɻ��ܺ������

static uint8_t g_u8CanMsgRxBuff[PRGM_RX_BUFF_SIZE];//��CAN�ӿڽ��յ������ݵĻ��ܻ�����
static uint8_t  CanMsgRxStatu = NOT_STARTED;

WHETHER_TYPE_VARIABLE_Typedef g_eCanMsgRxStatu = NO;

void CAN1_RX0_IRQHandler(void)
{
    static   unsigned int i = 0;

    uint8_t j;
    OS_ERR err;
    CPU_SR_ALLOC();
    CanRxMsg stCAN1_Rx_Msg;                         //������·������ݰ�����
//  Message stRxMSG = Message_Initializer;          //���������ݰ�����

    CAN_Receive(CAN1, CAN_FIFO0, &(stCAN1_Rx_Msg));         //��CAN1 FIFO0����������·���CAN����


    if((stCAN1_Rx_Msg.DLC == 8) && (stCAN1_Rx_Msg.Data[0] == 0xFC) && (stCAN1_Rx_Msg.Data[1] == 0xFD) && (stCAN1_Rx_Msg.Data[2] == 0xFE)) { //�յ����Ǳ�ͷ
        for(i = 0; i < stCAN1_Rx_Msg.DLC; i++) {
            g_u8CanMsgRxBuff[i] = stCAN1_Rx_Msg.Data[i];
        }

        CanMsgRxStatu = COLLECTTING;
    } else { //�յ��Ĳ��Ǳ�ͷ
        if(CanMsgRxStatu == COLLECTTING) { //���򽫺������ֽڴ��뵽CAN��Ϣ������
            for(j = 0; j < stCAN1_Rx_Msg.DLC;) {
                g_u8CanMsgRxBuff[i] = stCAN1_Rx_Msg.Data[j];
                j++;
                i++;

                if(i >= PRGM_RX_BUFF_SIZE) {
//                  BSP_BuzzerOn();
                    CanMsgRxStatu = READY;
                    break;
                }
            }
        } else { //δ�ռ�����ʱ�յ���δ֪����
            CanMsgRxStatu = NOT_STARTED;
        }
    }

    if(CanMsgRxStatu == READY) {
        if(g_eCanMsgRxStatu == NO) {
            CPU_CRITICAL_ENTER();

            for(j = 0; j < PRGM_RX_BUFF_SIZE; j++) {
                g_u8CanRxMsg[j] = g_u8CanMsgRxBuff[j];
            }

            OSTimeDlyResume(&CommunicateTaskTCB,
                            &err);
            g_eCanMsgRxStatu = YES;
            CPU_CRITICAL_EXIT();
//          BSP_BuzzerOn();
        }

        CanMsgRxStatu = NOT_STARTED;
//      g_eCAN_BusOnLineFlag = ON;
    } else {
    }//����δ�����꣬�������

    //��Ϊ���յ�������֡�Ǳ�׼֡��ͨ�����洦������ת��Ϊ���������ݰ�����
    /*  stRxMSG.cob_id = (uint16_t)(stCAN1_Rx_Msg.StdId);
        stRxMSG.rtr = stCAN1_Rx_Msg.RTR;
        stRxMSG.len = stCAN1_Rx_Msg.DLC;
        for(i = 0; i < stRxMSG.len; i++)
        {
            stRxMSG.data[i] = stCAN1_Rx_Msg.Data[i];
        }
    */
//  canDispatch(CANOpenMasterObject, &(stRxMSG));           //������������������ݰ�
}

#endif

/******************* (C) COPYRIGHT 2015 Personal Electronics *****END OF FILE****/
