/*
***************************************************************************************************
*                                              APPLICATION CODE
*
*                          (c) Copyright 2016; Guangdong ENECO Energy Science And Technology Co.,Ltd
*
*               All rights reserved.  Protected by international copyright laws.
*               Knowledge of the source code may NOT be used without authorization.
***************************************************************************************************
*/
/*
***************************************************************************************************
* Filename      : bsp_huawei_485_adjush.c
* Version       : V1.00
* Programmer(s) : Fanjun
*
***************************************************************************************************
*/
/*
***************************************************************************************************
*                                             INCLUDE FILES
***************************************************************************************************
*/
#include "bsp_dc_module_adjust.h"
/*
***************************************************************************************************
*                                           MACRO DEFINITIONS
***************************************************************************************************
*/
//ת������ѯ�����ʽ
//   �����   �������� ��������   �����ֽ�    ------��Ϣ��--------   У����
// {0x00,0x08,    0xC8,   0x01,   0x55,0x55,  0x00,0x00,  0x00,0x00,  0x00};
/*
***************************************************************************************************
*                                           LOCAL VARIABLES
***************************************************************************************************
*/


/*
***************************************************************************************************
*                                         OS-RELATED    VARIABLES
***************************************************************************************************
*/
OS_MUTEX        Rs485RxFiniehedMutex;
/*
***************************************************************************************************
*                                           GLOBAL VARIABLES
***************************************************************************************************
*/
uint16_t g_u16CommandAdress = 0x1C7;//Ĭ��Ϊת������
uint16_t RS485_RX_BUF[64] = {0};     //���ջ�����,���64���ֽ�
uint8_t RS485_RX_CNT = 0;           //���յ������ݳ���
uint8_t g_u8RS485TxDateType = 0;//485�������ݵ�����
/*
***************************************************************************************************
*                                         FUNCTION PROTOTYPES
***************************************************************************************************
*/
static void RS485Rx_IRQHandler(void);

/*
***************************************************************************************************
*                                     Bsp_DcModuleConmunicateInit()
*
* Description : UART5 Init.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Note(s)     : none.
***************************************************************************************************
*/
void Bsp_DcModuleConmunicateInit(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    //ʱ��ʹ��
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);   //ʹ��UART5ʱ��
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD | RCC_APB2Periph_AFIO, ENABLE);  //ʹ��GPIOC��D��ʱ�Ӻ�AFIO���ù���ģ��ʱ��

    //UART5_TX-PC12
    GPIO_InitStructure.GPIO_Pin = BSP_GPIOC_UART5_TX_PORT_NMB;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //�����������
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    //UART5_RX-PD2
    GPIO_InitStructure.GPIO_Pin = BSP_GPIOD_UART5_RX_PORT_NMB;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;   //��������
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    USART_DeInit(UART5);  //��λ����5

    USART_InitStructure.USART_BaudRate = 9600;
    USART_InitStructure.USART_WordLength = USART_WordLength_9b;//ע��������9λ,ͨѶЭ�����涨
    USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
    USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; //�շ�ģʽ
    USART_Init(UART5, &USART_InitStructure); //��ʼ������

    BSP_IntVectSet(BSP_INT_ID_USART5, RS485Rx_IRQHandler);
    BSP_IntEn(BSP_INT_ID_USART5);

    USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);
    USART_Cmd(UART5, ENABLE);

    OSMutexCreate(&Rs485RxFiniehedMutex,
                  "Rs485 rx finished mutex",
                  NULL);
}

/*
***************************************************************************************************
*                             RS485Rx_IRQHandler()
*
* Description : Serial ISR..
*
* Arguments   : none.
*
* Returns     : none.
*
* Notes       : none.
***************************************************************************************************
*/

static void RS485Rx_IRQHandler(void)
{
    uint16_t res;
    OS_ERR  err;

    if(USART_GetITStatus(UART5, USART_IT_RXNE) != RESET) {
        res = (USART_ReceiveData(UART5) & 0xFFFF);

        if(RS485_RX_CNT < 64) {
            RS485_RX_BUF[RS485_RX_CNT] = res;   //��¼���յ���ֵ
            RS485_RX_CNT++;

            if((TYPE_IS_ADRESS == GetRS485TxDateType()) || (TYPE_IS_TRANSPOND_CMD == GetRS485TxDateType())) {

                if((RS485_RX_BUF[0] == 0x7F) || (RS485_RX_BUF[0] == 0x07)) { //���ص�ַ��ת������У����ȷ
                    OSMutexPost(&Rs485RxFiniehedMutex,//������+�������Ž�����һ����������
                                OS_OPT_POST_1,
                                &err);
                }
            } else if(TYPE_IS_INQUIRY_CMD == GetRS485TxDateType()) {
                if(RS485_RX_CNT >= (RS485_RX_BUF[3] + 3)) { //��ѯ��������֡�������
                    OSMutexPost(&Rs485RxFiniehedMutex,//������+�������Ž�����һ����������
                                OS_OPT_POST_1,
                                &err);
                }
            }
        }

        USART_ClearITPendingBit(UART5, USART_IT_RXNE);
    }
}

/*
***************************************************************************************************
*                              Bsp_RS485_Send_Data()
*
* Description : RS485��buf��ַ����Len���ֽ�����.
*
* Arguments   : buf:�������׵�ַ;len:���͵��ֽ���.
*
* Returns     : none.
*
* Notes       : none.
*
***************************************************************************************************
*/
void Bsp_RS485_Send_Data(u16 *buf, u8 len)
{
    uint8_t i = 0;
    OS_ERR      err;

    //���ͱ���ȴ��������
    OSMutexPend(&Rs485RxFiniehedMutex,
                OS_CFG_TICK_RATE_HZ / 10, //���ͽ������̼���ʱ
                OS_OPT_PEND_BLOCKING,
                NULL,
                &err);

    if(OS_ERR_NONE == err) {
        for(i = 0; i < len; i++) { //ѭ����������
            while(USART_GetFlagStatus(UART5, USART_FLAG_TC) == RESET);

            USART_SendData(UART5, buf[i]);
//            APP_TRACE_INFO(("Rs485TxBuf[%d]: = %X...\r\n",i,buf[i]));
        }

        while(USART_GetFlagStatus(UART5, USART_FLAG_TC) == RESET);
    } else {
        APP_TRACE_INFO(("Rs485 Rx finiehed mutex pend errcode is %d...\r\n", err));
    }

    OSMutexPost(&Rs485RxFiniehedMutex,//������+�������Ž�����һ����������
                OS_OPT_POST_1,
                &err);
}


/*
***************************************************************************************************
*                              Bsp_RS485_Receive_Data()
*
* Description : RS485��������,������ݺ����ݳ���.
*
* Arguments   : buf:���ջ����׵�ַ;len:���������ݳ���.
*
* Returns     : none.
*
* Notes       : none.
*
***************************************************************************************************
*/

void Bsp_RS485_Receive_Data(u16 *buf, u8 *len)
{
    uint8_t rxlen = RS485_RX_CNT;
    uint16_t nullnum = 0;
    uint8_t i = 0;
    *len = 0; //Ĭ��Ϊ0

    for(nullnum = 0; nullnum < 1000; nullnum++); //��ʱ,��������10msû�н��յ�һ������,����Ϊ���ս���

    if(rxlen == RS485_RX_CNT && rxlen) { //���յ�������,�ҽ��������
        for(i = 0; i < rxlen; i++) {
            buf[i] = RS485_RX_BUF[i];
        }

        *len = RS485_RX_CNT; //��¼�������ݳ���
        RS485_RX_CNT = 0;   //����
    }
}



/*
***************************************************************************************************
*                           Bsp_SetAddressByDifferentCmdType()
*
* Description : ���ݲ�ͬ��������Ͳ�ͬ��Ѱַ��ַ.
*
* Arguments   : i_u8CmdType :INQUIRY_COMMAND��TRANSPOND_COMMAND��BROADCAST_COMMAND
*
* Returns     : none.
*
* Notes       : none.
***************************************************************************************************
*/
void Bsp_SendAddressByDifferentCmdType(uint8_t i_u8CmdType)
{
    switch((uint8_t)i_u8CmdType) {
        case INQUIRY_COMMAND:
            g_u16CommandAdress = INQUIRY_COMMAND_ADRESS  & 0xFFFF;
            break;

        case TRANSPOND_COMMAND:
            g_u16CommandAdress = TRANSPOND_COMMAND_ANDRESS & 0xFFFF;
            break;

        case BROADCAST_COMMAND:
            g_u16CommandAdress = BROADCAST_COMMAND_ADRESS & 0xFFFF;
            break;

        default:
            break;
    }

    SetRS485TxDateType(TYPE_IS_ADRESS);
    Bsp_RS485_Send_Data(&g_u16CommandAdress, 1); //����Ѱַ��ַ
}


/*
***************************************************************************************************
*                           Bsp_SendRequestCmdToDcModule()
*
* Description :����ת��ָ��.
*
* Arguments   :i_u8CmdType :CMD_REGISTER_RESPONSE��CMD_GET_ALARM_INFO��
*                           CMD_GET_CURRENT_VI_VALUE��CMD_GET_MODULE_VI_SET_PARA.
*
* Returns     : none.
*
* Notes       : none.
***************************************************************************************************
*/
void Bsp_SendTransportCmdToHuaWeiDC(uint8_t i_u8CmdType)
{
    uint8_t i = 0;
    uint8_t u8CheckSumValue = 0;
    uint16_t HuaWeiModuleTxBuf[7] = {0x00, 0x04, 0xC8, 0x00, 0x66, 0x66, 0x00};

    HuaWeiModuleTxBuf[3] = i_u8CmdType & 0xFF;

    for(i = 0; i < 7; i++) {
        u8CheckSumValue = u8CheckSumValue + HuaWeiModuleTxBuf[i];
    }

    HuaWeiModuleTxBuf[6] = u8CheckSumValue & 0xFF;//У���

    SetRS485TxDateType(TYPE_IS_TRANSPOND_CMD);
    Bsp_RS485_Send_Data(HuaWeiModuleTxBuf, 7);
}

/*
***************************************************************************************************
*                           Bsp_GetReportInformationAfterTransportCmd()
*
* Description :���Ͳ�ѯָ���ȡת��ָ����Ч����ϱ���Ϣ.
*
* Arguments   : none.
*
* Returns     : none.
*
* Notes       : none.
***************************************************************************************************
*/
void Bsp_GetReportInformationAfterTransportCmd(void)
{
    SetRS485TxDateType(TYPE_IS_INQUIRY_CMD);
    Bsp_SendAddressByDifferentCmdType(INQUIRY_COMMAND);

}
/*
***************************************************************************************************
*                           Bsp_SendRequestCmdToDcModule()
)
*
* Description : ����ָ���û�Ϊģ�鿪�ػ�.
*
* Arguments   : i_u8PowerStatus :0-����;1-�ػ�;i_u8StatusChangeDly:�ػ����Զ�������ʱʱ��.
*
* Returns     : none.
*
* Notes       : none.
***************************************************************************************************
*/
void Bsp_SendCmdControlDcModulePowerOnOrDown(uint8_t i_u8PowerStatus, uint8_t i_u8StatusChangeDly)
{
    uint8_t i = 0;
    uint8_t u8CheckSumValue = 0;
    uint16_t HuaWeiModuleTxBuf[9] = {0x00, 0x06, 0xC8, 0x29, 0x66, 0x66, 0x00, 0x00, 0x00};

    HuaWeiModuleTxBuf[6] = i_u8PowerStatus & 0xFF;
    HuaWeiModuleTxBuf[7] = i_u8StatusChangeDly & 0xFF;

    for(i = 0; i < 8; i++) {
        u8CheckSumValue = u8CheckSumValue + HuaWeiModuleTxBuf[i];
    }

    HuaWeiModuleTxBuf[8] = u8CheckSumValue & 0xFF;//У���

    Bsp_RS485_Send_Data(HuaWeiModuleTxBuf, 9);
}

/*
***************************************************************************************************
*                           Bsp_SetDcModuleOutPutVIvalue()
*
* Description : set hua wei module current voletage and current.
*
* Arguments   : i_fVvalue:set volatage value;i_fIvalue:set current value;Support two decimal places.
*
* Returns     : none.
*
* Notes       : none.
***************************************************************************************************
*/
void Bsp_SetDcModuleOutPutVIvalue(float i_fVvalue, float i_fIvalue)
{
    uint8_t i = 0;
    uint8_t u8CheckSumValue = 0;
    uint16_t Value_temp = 0, Ivalue_temp = 0;
    uint16_t HuaWeiModuleTxBuf[11] = {0x00, 0x08, 0xC8, 0x42, 0x55, 0x55, 0x00, 0x00, 0x00, 0x00, 0x00};

//    APP_TRACE_INFO(("Set DC module Out Put VI value...\n\r"));
    Value_temp = (uint16_t)(i_fVvalue * 100);
    Ivalue_temp = (uint16_t)(i_fIvalue * 100);

    HuaWeiModuleTxBuf[6] = (Value_temp & 0xFF00) >> 8;
    HuaWeiModuleTxBuf[7] = (Value_temp & 0x00FF);
    HuaWeiModuleTxBuf[8] = (Ivalue_temp & 0xFF00) >> 8;
    HuaWeiModuleTxBuf[9] = (Ivalue_temp & 0x00FF);

    for(i = 0; i < 10; i++) {
        u8CheckSumValue = u8CheckSumValue + HuaWeiModuleTxBuf[i];
    }

    HuaWeiModuleTxBuf[10] = u8CheckSumValue & 0xFF;//У���

    SetRS485TxDateType(TYPE_IS_TRANSPOND_CMD);
    Bsp_RS485_Send_Data(&HuaWeiModuleTxBuf[0], 11);
}

/*
***************************************************************************************************
*                           Bsp_SetDcModuleOutPutVIvalue()
*
* Description : set hua wei module current voletage and current.
*
* Arguments   : i_fVvalue:set volatage value;i_fIvalue:set current value;Support two decimal places.
*
* Returns     : none.
*
* Notes       : none.
***************************************************************************************************
*/

void SetRS485TxDateType(uint8_t i_u8Rs485TxType)
{
    g_u8RS485TxDateType = i_u8Rs485TxType;
}

uint8_t GetRS485TxDateType(void)
{
    return g_u8RS485TxDateType;
}
