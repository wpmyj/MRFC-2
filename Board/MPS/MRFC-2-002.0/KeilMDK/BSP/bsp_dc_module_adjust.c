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
#include "app_dc_module_communicate_task.h"
#include "bsp_crc16.h"
/*
***************************************************************************************************
*                                           MACRO DEFINITIONS
***************************************************************************************************
*/

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
uint16_t    RS485_RX_BUF[64] = {0};     //���ջ�����,���64���ֽ�
uint8_t     RS485_RX_CNT = 0;           //���յ������ݳ���
uint8_t     g_u8RS485TxDateType = 0;    //485�������ݵ�����

float       g_fDCInputVoltageA = 0;     //DC-A·�����ѹ  [10-11]
float       g_fDCTemp = 0;              //DC�ڲ������¶�  [18-19]
float       g_fDCOutputVoltage = 0;     //DC�����ѹ      [23-24]
float       g_fDCOutputCurrent = 0;     //DC�������      [25-26]
uint16_t    g_u16DCOutputPower = 0;       //DC�������      [27-28]


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
    GPIO_InitStructure.GPIO_Pin = BSP_GPIOD_PIN2_UART5_RX_PORT_NMB;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;   //��������
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    USART_DeInit(UART5);  //��λ����5

    USART_InitStructure.USART_BaudRate = 9600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;//ע��������9λ,ͨѶЭ�����涨
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

    if(USART_GetITStatus(UART5, USART_IT_RXNE) != RESET) {
        res = (USART_ReceiveData(UART5) & 0xFF);

        if(RS485_RX_CNT < 64) {
            RS485_RX_BUF[RS485_RX_CNT] = res;   //��¼���յ���ֵ
            RS485_RX_CNT++;
        } else {
            RS485_RX_CNT = 0;
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
void Bsp_RS485_Send_Data(u8 *buf, u8 len)
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

void Bsp_RS485_Receive_Data(u8 *buf, u8 *len)
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
*                            DCToStm32Message()
*
* Description:  DC to Stm32 message.
*
* Arguments  :  none
*
* Returns    :  none
***************************************************************************************************
*/
void DCToStm32Message()
{
    u16         u16DCInputVoltageA;
    u16         u16DCOutputCurrent;
//  u16         u16DCInputVoltageB;
//  float       DCInputVoltageB;     //DC-A·�����ѹ  [12-13]

    u16         u16DCTemp;

    u16         u16DCOutputVoltage;

    u16         u16DCOutputPower;
    uint8_t     DCAlarm_1;           //DC�澯1         [20] bit[7]Ϊ1��ʾA·�����ѹ�澯  bit[6]Ϊ1��ʾA·�����ѹ�澯
    uint8_t     DCAlarm_2;           //DC�澯2         [21] bit[3]Ϊ1��ʾ������־

    uint8_t u8RxLength = 0;
    uint8_t u8Rs485RxBuf[64] = {0};

    Bsp_RS485_Receive_Data(u8Rs485RxBuf, &u8RxLength);

    if((u8Rs485RxBuf[0] == 0xA0) &&
            (u8Rs485RxBuf[1] == 0xA0) &&
            (u8Rs485RxBuf[2] == 0xA0)) {
        u16DCInputVoltageA = (u8Rs485RxBuf[10] << 8) + u8Rs485RxBuf[11];
        g_fDCInputVoltageA = (float)u16DCInputVoltageA / 100;;              //DC-A·�����ѹ[10-11]

        u16DCTemp = (u8Rs485RxBuf[18] << 8) +  u8Rs485RxBuf[19];
        g_fDCTemp = (float)u16DCTemp / 100;                                 //DC�ڲ������¶�[18-19]

        u16DCOutputVoltage = (u8Rs485RxBuf[23] << 8) + u8Rs485RxBuf[24];
        g_fDCOutputVoltage = (float)u16DCOutputVoltage / 100;               //DC�����ѹ [23-24]

        u16DCOutputCurrent = (u8Rs485RxBuf[25] << 8) + u8Rs485RxBuf[26];
        g_fDCOutputCurrent = (float)u16DCOutputCurrent / 100;               //DC������� [25-26]

        u16DCOutputPower = (u8Rs485RxBuf[27] << 8) + u8Rs485RxBuf[28];
        g_u16DCOutputPower  = (uint16_t)u16DCOutputPower / 100;             //DC������� [27-28]

        DCAlarm_1 = u8Rs485RxBuf[20];                                       //DC�澯1 [20] bit[7]Ϊ1��ʾA·�����ѹ�澯  bit[6]Ϊ1��ʾA·�����ѹ�澯
        DCAlarm_2 = u8Rs485RxBuf[21];                                       //DC�澯2 [21] bit[3]Ϊ1��ʾ������־
    }
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
    uint16_t u16CheckSumValue = 0;
    uint16_t Value_temp = 0, Ivalue_temp = 0;
    uint8_t DCModuleTxBuf[13] = {0xA0, 0xA0, 0xA0, 0xFF, 0x08, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    Value_temp = (uint16_t)(i_fVvalue * 100);
    Ivalue_temp = (uint16_t)(i_fIvalue * 100);

    DCModuleTxBuf[7] = (Value_temp & 0xFF00) >> 8;
    DCModuleTxBuf[8] = (Value_temp & 0x00FF);
    DCModuleTxBuf[9] = (Ivalue_temp & 0xFF00) >> 8;
    DCModuleTxBuf[10] = (Ivalue_temp & 0x00FF);

    u16CheckSumValue = GetXmodemCrc16Code(&DCModuleTxBuf[4], 7);

    DCModuleTxBuf[11] = (u16CheckSumValue & 0xFF00) >> 8;    // CRC-16У��
    DCModuleTxBuf[12] = (u16CheckSumValue & 0x00FF);         // CRC-16У��

    Bsp_RS485_Send_Data(&DCModuleTxBuf[0], 13);


}



void Bsp_SendReqCmdToDcModule(void)
{
    uint16_t u16CheckSumValue = 0;
    //A0 A0 A0 F0 04 00 00 +У���

    uint8_t DCModuleTxBuf[9] = {0xA0, 0xA0, 0xA0, 0xFF, 0x04, 0x00, 0x00, 0x00, 0x00};

    //GetXmodemCrc16Code
    u16CheckSumValue = GetXmodemCrc16Code(&DCModuleTxBuf[4], 3); 

    DCModuleTxBuf[7] = (u16CheckSumValue & 0xFF00) >> 8;              // CRC-16У��
    DCModuleTxBuf[8] = (u16CheckSumValue & 0x00FF);                   // CRC-16У��

    Bsp_RS485_Send_Data(DCModuleTxBuf, 9);
}




/*
***************************************************************************************************
*                           Bsp_CmdDcModuleStartUp()
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
void Bsp_CmdDcModuleStartUp(void)
{
    uint16_t u16CheckSumValue = 0;
    uint8_t DCModuleTxBuf[10] = {0xA0, 0xA0, 0xA0, 0xFF, 0x05, 0x00, 0x02, 0x88, 0x00, 0x00};

    u16CheckSumValue = GetXmodemCrc16Code(&DCModuleTxBuf[4], 4);

    DCModuleTxBuf[8] = (u16CheckSumValue & 0xFF00) >> 8;// CRC-16У��
    DCModuleTxBuf[9] = (u16CheckSumValue & 0x00FF);// CRC-16У��

    Bsp_RS485_Send_Data(&DCModuleTxBuf[0], 10);
}



/*
***************************************************************************************************
*                           Bsp_CmdDcModuleShutDown()
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
void Bsp_CmdDcModuleShutDown(void)
{
    uint16_t u16CheckSumValue = 0;

    uint8_t DCModuleTxBuf[10] = {0xA0, 0xA0, 0xA0, 0xFF, 0x05, 0x00, 0x02, 0x00, 0x00, 0x00};


    u16CheckSumValue = GetXmodemCrc16Code(&DCModuleTxBuf[4], 4);

    DCModuleTxBuf[8] = (u16CheckSumValue & 0xFF00) >> 8;              // CRC-16У��
    DCModuleTxBuf[9] = (u16CheckSumValue & 0x00FF);                   // CRC-16У��

    Bsp_RS485_Send_Data(DCModuleTxBuf, 10);
}







/*
***************************************************************************************************
*                     SetDcModeOutPutNominalVoltageButDifferentCurrent()
*
* Description : set hua wei module current voletage and current.
*
* Arguments   : i_fIvalue:set current value.
*
* Returns     : none.
*
* Notes       : none.
***************************************************************************************************
*/
void SetDcModeOutPutNominalVoltageButDifferentCurrent(float i_fIvalue)
{
    g_fIvalueNow = i_fIvalue;//ˢ��ȫ��������
    Bsp_SetDcModuleOutPutVIvalue(VOLTAGE_LIMIT_MAX, i_fIvalue);
}

