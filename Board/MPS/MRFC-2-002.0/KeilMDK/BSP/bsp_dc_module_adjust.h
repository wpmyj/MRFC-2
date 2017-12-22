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
* Filename      :  bsp_dc_module_485_adjust.h
* Programmer(s) :  JasonFan
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


/*
***************************************************************************************************
*                                           EXPORTED GLOBAL VARIABLE DECLARATIONS
***************************************************************************************************
*/
extern uint16_t RS485_RX_BUF[64];         //接收缓冲,最大64个字节
extern uint8_t  RS485_RX_CNT;             //接收到的数据长度

extern  float       g_fDCInputVoltageA;     //DC-A路输入电压  [10-11]
extern  float       g_fDCTemp;              //DC内部环境温度  [18-19]
extern  float       g_fDCOutputVoltage;     //DC输出电压      [23-24]
extern  float       g_fDCOutputCurrent;     //DC输出电流      [25-26]

extern  uint16_t       g_u16DCOutputPower;       //DC输出功率      [27-28]
extern  uint16_t       g_u16DCOutputCurrent;

/*
***************************************************************************************************
*                                           EXPORTED DATA TYPE
***************************************************************************************************
*/


/*
***************************************************************************************************
*                                           EXPORTED FUNCTION
***************************************************************************************************
*/
void Bsp_RS485_Send_Data(u8 *buf, u8 len);
void Bsp_RS485_Receive_Data(u8 *buf, u8 *len);

void DCToStm32Message(void);

void Bsp_CmdDcModuleShutDown(void);
void Bsp_CmdDcModuleStartUp(void);
void Bsp_SendReqCmdToDcModule(void);


void Bsp_SendAddressByDifferentCmdType(uint8_t i_u8CmdType);
void Bsp_GetReportInformationAfterTransportCmd(void);

void Bsp_DcModuleConmunicateInit(void);
void Bsp_SendRequestCmdToDcModule(uint8_t i_u8CmdType);

void    SetRS485TxDateType(uint8_t i_u8Rs485TxType);
uint8_t GetRS485TxDateType(void);

void SetDcOutPutCurrentLimitPoint(float i_fIvalue);

#endif


