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

    /*时钟使能*/
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE); 
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3,  ENABLE); 

    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //复用推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //SPI设置为双线双向全双工
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;       //设置SPI工作模式:设置为主SPI
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;   //SPI发送接收8位帧结构
    /*此处使用SPI模式0*/
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;          //串行同步时钟的空闲状态为低电平 SPI_CPOL_High+SPI_CPHA_2Edge:模式三
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;        //串行同步时钟的第一个跳变沿(上升沿)数据被采样
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;           //NSS信号由硬件（NSS管脚）还是软件（使用SSI位）管理:内部NSS信号有SSI位控制
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;// 36/4=9MHz AT25芯片在3.3V供电下最高频率为10MHz     
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;  //数据传输从MSB位开始
    SPI_InitStructure.SPI_CRCPolynomial = 7;            //CRC值计算的多项式
    SPI_Init(SPI3, &SPI_InitStructure);                 //初始化外设SPIx寄存器

    SPI_Cmd(SPI3, ENABLE); //使能SPI外设

}

/*
***************************************************************************************************
*                               Make_Vacuum_FunctionTask()
*
* Description : SPI3 read and write a byte data.
*
* Arguments   : TxData:要写入的字节.
*
* Returns     : 读取到的字节.
*
* Notes       : none.
***************************************************************************************************
*/
uint16_t SPI3_ReadWriteByte(uint16_t TxData)
{
    uint8_t retry = 0;

    while(SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_TXE) == RESET) { //发送缓存空标志位
        retry++;

        if(retry > 200) {
            return 0;
        }
    }

    SPI_I2S_SendData(SPI3, TxData); //通过外设SPIx发送一个数据
    retry = 0;

    while(SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_RXNE) == RESET) { //检查指定的SPI标志位设置与否:接受缓存非空标志位
        retry++;

        if(retry > 200) {
            return 0;
        }
    }

    return SPI_I2S_ReceiveData(SPI3); //返回通过SPIx最近接收的数据
}

