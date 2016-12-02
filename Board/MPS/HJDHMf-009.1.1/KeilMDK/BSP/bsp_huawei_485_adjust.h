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

/********************************************************************************************************
* Filename      :  bsp_huawei_485_adjush.h
* Programmer(s) :  Fanjun
* Version       :  V1.0
* data          :  2016.11.12
* brief         :  This file contains all the functions prototypes for the system run
*                  config parameters firmware library.
*********************************************************************************************************/
#ifndef __BSP_HUA_WEI_485_COMMUNICATE_H_
#define __BSP_HUA_WEI_485_COMMUNICATE_H_
/*
***************************************************************************************************
*                                           INCLUDE FILES
***************************************************************************************************
*/
#include "includes.h"
/*
***************************************************************************************************
*                                           MACRO DEFINITIONS
***************************************************************************************************
*/
#if (EN_RS485_RX > 1)
#error Rs485 recive has enabled!!!
#endif

#define EN_RS485_RX    1           //0,不接收;1,接收.

//寻址地址格式:
//bit8地址/数据帧位,1:表示地址帧，0表示数据帧。
//bit7-bit5:表示各类命令类型
//bit4-bit0:表示模块监控地址，全为1表示广播命令,不回监控地址，紧跟数据帧就行了
#define  BROADCAST_COMMAND_ADRESS  0x1DF
#define  INQUIRY_COMMAND_ADRESS    0x137
#define  TRANSPOND_COMMAND_ANDRESS 0x1C7

//转发指令类型码
#define    CMD_REGISTER_RESPONSE        0x01
#define    CMD_GET_ALARM_INFO           0x23
#define    CMD_GET_CURRENT_VI_VALUE     0x24
#define    CMD_SET_OUTPUT_VI_VALUE      0x42
#define    CMD_GET_MODULE_VI_SET_PARA   0x43

//发送数据类型:地址或者命令
#define    TYPE_IS_ADRESS   0x01
#define    TYPE_IS_TRANSPOND_CMD   0x02
#define    TYPE_IS_INQUIRY_CMD   0x03
/*
***************************************************************************************************
*                                           EXPORTED GLOBAL VARIABLE DECLARATIONS
***************************************************************************************************
*/
extern uint16_t RS485_RX_BUF[64];         //接收缓冲,最大64个字节
extern uint8_t  RS485_RX_CNT;             //接收到的数据长度

/*
***************************************************************************************************
*                                           EXPORTED DATA TYPE
***************************************************************************************************
*/
typedef enum
{
    INQUIRY_COMMAND = 0,
    TRANSPOND_COMMAND,
    BROADCAST_COMMAND,
    
}CMD_TYPE_Typedef;


/*
***************************************************************************************************
*                                           EXPORTED FUNCTION
***************************************************************************************************
*/
void Bsp_RS485_Send_Data(u16 *buf, u8 len);
void Bsp_RS485_Receive_Data(u16 *buf, u8 *len);

void Bsp_SendAddressByDifferentCmdType(uint8_t i_u8CmdType);
void Bsp_GetReportInformationAfterTransportCmd(void);

void Bsp_HuaWeiDCConmunicateInit(void);
void Bsp_SendRequestCmdToHuaWeiDC(uint8_t i_u8CmdType);
 
void Bsp_SetHWmoduleOutPutVIvalue(float i_fVvalue, float i_fIvalue);
void Bsp_SendCmdControlHuaWeiDCPowerOnOrDown(uint8_t i_u8PowerStatus,uint8_t i_u8StatusChangeDly);

void SetRS485TxDateType(uint8_t i_u8Rs485TxType);
uint8_t GetRS485TxDateType(void);

#endif


