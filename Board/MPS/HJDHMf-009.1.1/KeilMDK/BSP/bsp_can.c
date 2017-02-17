
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
#include "app_system_run_cfg_parameters.h"
/*
***************************************************************************************************
*                                           MACRO DEFINITIONS
***************************************************************************************************
*/

//CAN�������ݷ���״̬
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
static uint8_t  g_u8CanMsgRxBuff[PRGM_RX_BUFF_SIZE];//��CAN�ӿڽ��յ��Ĵ����ܵ����ݵĻ�����
static uint8_t  CanMsgRxStatu = NOT_STARTED;       //CAN���߽���״̬
/*
***************************************************************************************************
*                                           GLOBAL VARIABLES
***************************************************************************************************
*/
WHETHER_TYPE_VARIABLE_Typedef       g_eCAN_BusOnLineFlag = YES;
WHETHER_TYPE_VARIABLE_Typedef       g_eCanMsgRxStatu = NO;//�Ƿ���յ���Ϣ

uint8_t     g_u8CanRxMsg[PRGM_RX_BUFF_SIZE];//���ܺ�CAN��������

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
    //ʱ��ʹ��
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOD, ENABLE);

    GPIO_PinRemapConfig(GPIO_Remap2_CAN1, ENABLE);  //����ӳ��
    
    //CAN1 TX
    GPIO_InitStructure.GPIO_Pin = BSP_GPIOD_PIN1_CAN1_TX_PORT_NMB;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //��������
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    //CAN1 RX
    GPIO_InitStructure.GPIO_Pin = BSP_GPIOD_PIN0_CAN1_RX_PORT_NMB;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//��������
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    /* CAN1 ��Ԫ���� */
    CAN_InitStructure.CAN_TTCM = DISABLE; /* ʱ�䴥����ֹ, ʱ�䴥����CANӲ�����ڲ���ʱ����������ұ����ڲ���ʱ��� */
    CAN_InitStructure.CAN_ABOM = DISABLE; /* �Զ�����,һ��Ӳ����ص�128��11������λ�����Զ��˳�����״̬��������Ҫ����趨������˳� */
    CAN_InitStructure.CAN_AWUM = ENABLE; /* �Զ�����,�б�������ʱ���Զ��˳�����   */
    CAN_InitStructure.CAN_NART = DISABLE; /* �����ش�, �������һֱ�����ɹ�ֹ������ֻ��һ�� */
    CAN_InitStructure.CAN_RFLM = DISABLE; /* ����FIFO����, 1--��������յ��µı���ժ��Ҫ��0--���յ��µı����򸲸�ǰһ����   */
    CAN_InitStructure.CAN_TXFP = ENABLE;  /* �������ȼ�  0---�ɱ�ʶ������  1---�ɷ�������˳�����   */
    CAN_InitStructure.CAN_Mode = CAN_Mode_Normal; /*����ģʽ*/ //CAN_Mode_LoopBack��CAN_Mode_Normal
    CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;		//BTR-SJW ����ͬ����Ծ��� 1��ʱ�䵥Ԫ
    CAN_InitStructure.CAN_BS1 = CAN_BS1_3tq;		//BTR-TS1 ʱ���1 ռ����12��ʱ�䵥Ԫ
    CAN_InitStructure.CAN_BS2 = CAN_BS2_2tq;		//BTR-TS1 ʱ���2 ռ����3��ʱ�䵥Ԫ
    CAN_InitStructure.CAN_Prescaler = 120;		   	//BTR-BRP �����ʷ�Ƶ��  ������ʱ�䵥Ԫ��ʱ�䳤�� 36MHz/(1 + 3 + 2) / 120 = 50KHZ
    CAN_Init(CAN1, &CAN_InitStructure);	

    CAN_FilterInitStructure.CAN_FilterNumber = 0;     //����������
    CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask; //����������λģʽ
    CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;                 
    
    CAN_FilterInitStructure.CAN_FilterIdHigh =  (g_u16GlobalNetWorkId & 0xFFFF); //Ҫ���˵�ID��λ�����˵����Ƿ��͸�����������֡
    CAN_FilterInitStructure.CAN_FilterIdLow = (( CAN_ID_STD | CAN_RTR_DATA) & 0xFFFF); //ȷ���յ����Ǳ�׼����֡
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0xFFFF;//���е�λȫ������ƥ��
    CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0xFFFF; 

//    CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;     //���ȼ�
//    CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
//    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000; //32λ��ʾ����λ
//    CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;

    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_FIFO0;  //������0������FIFO0
    CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;//���������0���������Զ��˳���ʼ��ģʽ
    CAN_FilterInit(&CAN_FilterInitStructure);

#if CAN1_RX0_INT_ENABLE
    NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;     // �����ȼ�Ϊ1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;            // �����ȼ�Ϊ0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);/* �Һ��ж�, �����жϺ��FIFO�ı��ĺ����ͷű������жϱ�־ */
    BSP_IntVectSet(BSP_INT_ID_CAN1_RX0, CAN1_RX0_IRQHandler);
    BSP_IntEn(BSP_INT_ID_CAN1_RX0);
#endif
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
uint8_t CANx_Send_Msg(CAN_TypeDef * CANx, Message *m)
{
    uint8_t u8SendStatu = 0;
    uint8_t ret;
    uint8_t i;
    CanTxMsg TxMessage;

    TxMessage.StdId = (uint32_t)(m->cob_id);// ��׼��ʶ��11λ,���������ȼ�
    TxMessage.ExtId = 0x00;         // ������չ��ʾ��
    TxMessage.RTR = m->rtr;         //��Ϣ����Ϊ����֡��һ֡8λ
    TxMessage.IDE = CAN_ID_STD;     // ʹ�ñ�׼��ʶ��
    TxMessage.DLC = m->len;         // Ҫ���͵����ݳ���

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

    ProcessedData.cob_id = i_NodeId ;//�ϲ����ID
    ProcessedData.rtr = CAN_RTR_DATA;

    for(i = 0; i < (i_Msglen / 8 + ((i_Msglen % 8) ? 1 : 0)); i++) { //����Ϣ����ķְ���,8ΪCAN��Ϣ�ı�׼������
        if(LastTimeSendErrFlag == NO) {
            ProcessedData.len = ((i_Msglen - i * 8) > 8) ? 8 : (i_Msglen - i * 8);//ÿ���̰������ݳ��ȣ������һ����Ҫ�����⣬�����Ϊ8

            for(j = 0; j < ProcessedData.len; j++) {
                ProcessedData.data[j] = msg[i * 8 + j];
            }        
        } else {
            //�ϴη��ͳ���ֱ�ӷ������鼴�ɣ�����Ҫ���¸���
        }

        ret = CANx_Send_Msg(CAN1, &ProcessedData);

        if(ret != 0) {
            LastTimeSendErrFlag = YES;
            i--;
            OSTimeDlyHMSM(0, 0, 0, 100,
                          OS_OPT_TIME_HMSM_STRICT,
                          &err);

            if(++u8SendErrCount >= 10) {
//                g_eCAN_BusOnLineFlag = NO;
                APP_TRACE_INFO(("CAN bus has dropped,send err..\r\n"));
                break;
            }
        } else {
            LastTimeSendErrFlag = NO;//���ͳɹ�������ñ�־��������һ��ѭ�������͸�����
        }   
    }
    return ret;
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
uint8_t CANx_Receive_Msg(CAN_TypeDef *CANx, uint8_t *i_u8RxBuf)
{
    uint32_t i;
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
    CanRxMsg CAN1_Rx_Msg; //������·������ݰ�����
    OS_ERR err;
    CPU_SR_ALLOC();
    
    CAN_Receive(CAN1, CAN_FIFO0, &(CAN1_Rx_Msg));//��CAN1 FIFO0����������·���CAN����,ע��˴���ȡ���ĺ���Զ�����ж�
    
    if((CAN1_Rx_Msg.DLC == 8) 
        && (CAN1_Rx_Msg.Data[0] == 0xFC) 
        && (CAN1_Rx_Msg.Data[1] == 0xFD) 
        && (CAN1_Rx_Msg.Data[2] == 0xFE)) { //�յ����Ǳ�ͷ֡����
            
        for(i = 0; i < CAN1_Rx_Msg.DLC; i++) {
            g_u8CanMsgRxBuff[i] = CAN1_Rx_Msg.Data[i];
        }

        CanMsgRxStatu = COLLECTTING;
    } else { //�յ����ǷǱ�ͷ�ĺ���֡����
        if(CanMsgRxStatu == COLLECTTING) { //���򽫺������ֽڴ��뵽CAN��Ϣ������
            for(j = 0; j < CAN1_Rx_Msg.DLC;) {
                g_u8CanMsgRxBuff[i] = CAN1_Rx_Msg.Data[j];
                j++;
                i++;

                if(i >= PRGM_RX_BUFF_SIZE) {
                    CanMsgRxStatu = READY;//�����ռ����
                    break;
                }
            }
        } else { //δ�ռ�����ʱ�յ���δ֪����
            CanMsgRxStatu = NOT_STARTED;
        }
    }

    if(CanMsgRxStatu == READY) {//���ܺ�����ݰ�������ɺ��ٷ��͵���λ��
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
        g_eCAN_BusOnLineFlag = YES;//ԭΪע��״̬
    } else {
        //����δ�����꣬�������
    }
}

#endif

/******************* (C) COPYRIGHT 2016 Guangdong ENECO *****END OF FILE****/
