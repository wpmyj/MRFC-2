/*
***************************************************************************************************
*                                         APPLICATION CODE
*
*                      (c) Copyright 2016; Guangdong ENECO Science And Technology Co.,Ltd
*
*               All rights reserved.  Protected by international copyright laws.
*               Knowledge of the source code may NOT be used without authorization.
***************************************************************************************************
*/
/*
***************************************************************************************************
* Filename      : bsp_spi.c
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
#include "bsp_spi.h"
/*
***************************************************************************************************
*                                           MACRO DEFINES
***************************************************************************************************
*/


/*
***************************************************************************************************
*                                         OS-RELATED    VARIABLES
***************************************************************************************************
*/


/*
***************************************************************************************************
*                                       LOCAL VARIABLES
***************************************************************************************************
*/

/*
***************************************************************************************************
*                                         FUNCTION PROTOTYPES
***************************************************************************************************
*/


/*
***************************************************************************************************
*                               SPI3_Init()
*
* Description : The funciton is to init the spi 3.
*
* Arguments   : none.
*
* Returns     : none.
*
* Notes       : none.
***************************************************************************************************
*/
void SPI3_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    SPI_InitTypeDef  SPI_InitStructure;

    /*ʱ��ʹ��*/
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE); 
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3,  ENABLE); 

    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //�����������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //SPI����Ϊ˫��˫��ȫ˫��
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;       //����SPI����ģʽ:����Ϊ��SPI
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;   //SPI���ͽ���8λ֡�ṹ
    /*�˴�ʹ��SPIģʽ0*/
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;          //����ͬ��ʱ�ӵĿ���״̬Ϊ�͵�ƽ SPI_CPOL_High+SPI_CPHA_2Edge:ģʽ��
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;        //����ͬ��ʱ�ӵĵ�һ��������(������)���ݱ�����
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;           //NSS�ź���Ӳ����NSS�ܽţ����������ʹ��SSIλ������:�ڲ�NSS�ź���SSIλ����
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;// 36/4=9MHz AT25оƬ��3.3V���������Ƶ��Ϊ10MHz     
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;  //���ݴ����MSBλ��ʼ
    SPI_InitStructure.SPI_CRCPolynomial = 7;            //CRCֵ����Ķ���ʽ
    SPI_Init(SPI3, &SPI_InitStructure);                 //��ʼ������SPIx�Ĵ���

    SPI_Cmd(SPI3, ENABLE); //ʹ��SPI����

}

/*
***************************************************************************************************
*                               Make_Vacuum_FunctionTask()
*
* Description : SPI3 read and write a byte data.
*
* Arguments   : TxData:Ҫд����ֽ�.
*
* Returns     : ��ȡ�����ֽ�.
*
* Notes       : none.
***************************************************************************************************
*/
uint16_t SPI3_ReadWriteByte(uint16_t TxData)
{
    uint8_t retry = 0;

    while(SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_TXE) == RESET) { //���ͻ���ձ�־λ
        retry++;

        if(retry > 200) {
            return 0;
        }
    }

    SPI_I2S_SendData(SPI3, TxData); //ͨ������SPIx����һ������
    retry = 0;

    while(SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_RXNE) == RESET) { //���ָ����SPI��־λ�������:���ܻ���ǿձ�־λ
        retry++;

        if(retry > 200) {
            return 0;
        }
    }

    return SPI_I2S_ReceiveData(SPI3); //����ͨ��SPIx������յ�����
}

