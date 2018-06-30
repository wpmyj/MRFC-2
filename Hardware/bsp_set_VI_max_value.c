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
* Filename      : bsp_set_VI_max_value.c
* Version       : V1.00
* Programmer(s) : FanJun
*
***************************************************************************************************
*/
/*
***************************************************************************************************
*                                             INCLUDE FILES
***************************************************************************************************
*/
#include "bsp_set_VI_max_value.h"
#include "app_system_run_cfg_parameters.h"
#include "app_dc_module_communicate_task.h"
#include "bsp_crc16.h"
/*
***************************************************************************************************
*                                           MACRO DEFINITIONS
***************************************************************************************************
*/

/*
***************************************************************************************************
*                                            LOCAL DATATYPE
***************************************************************************************************
*/
#pragma pack(1)		/*指定按1字节对齐*/
#pragma anon_unions	//注意添加这句则可以不用给内嵌的结构体和联合体命名
typedef struct
{
	uint8_t head;
	uint8_t type;
	uint8_t length;
	union{
		struct{
			char     machineId[6];
			uint16_t currentMax;
			uint16_t voltageMax;
		};
	};

	uint16_t crc16;
	uint8_t tail;
} SettingCommand_t;
#pragma pack()	/*取消指定对齐，恢复缺省对齐*/
/*
***************************************************************************************************
*                                           LOCAL VARIABLES
***************************************************************************************************
*/
static uint8_t buff[16] = {0};
static uint8_t count = 0;
static uint8_t isProcessFinish = DEF_TRUE;
/*
***************************************************************************************************
*                                         FUNCTION PROTOTYPES
***************************************************************************************************
*/

/*
***************************************************************************************************
*                                          recLocalCfgData()
*
* Description : Receive and pack the local config data.
*
* Arguments   : none.
*
* Returns     : none.
*
* Notes       : none.
***************************************************************************************************
*/
void recLocalCfgData(const uint8_t data)
{
    static uint8_t isDetectHead = DEF_FALSE;

    if(isProcessFinish)
    {
        if(isDetectHead){
            buff[count++] = data;

            if(data == '>'){
                isDetectHead = DEF_FALSE;
                isProcessFinish = DEF_FALSE;
            }
        }else{
            if(data == '<'){
                isDetectHead = DEF_TRUE;
                count = 0;
                buff[count++] = data;
            }
        }
    }

}
/*
***************************************************************************************************
*                                          processData()
*
* Description : The use of the funciton is to keep the system warm.
*
* Arguments   : none.
*
* Returns     : none.
*
* Notes       : none.
***************************************************************************************************
*/
void processData(void)
{
    if(!isProcessFinish){
		
        const SettingCommand_t *cmd_ptr = (SettingCommand_t *)buff;//数组转结构体,
        const uint16_t crc16 = GetModbusCrc16Code((uint8_t *)&cmd_ptr->type, cmd_ptr->length - 2);
		
        if((cmd_ptr->length + 2 == count) && (crc16 == cmd_ptr->crc16)){
			
            if( cmd_ptr->type == 0x81 ){//收到写入指令

                g_stMachineIdPara.byte1 = cmd_ptr->machineId[0];
                g_stMachineIdPara.byte2 = cmd_ptr->machineId[1];
                g_stMachineIdPara.byte3 = cmd_ptr->machineId[2];
                g_stMachineIdPara.byte4 = cmd_ptr->machineId[3];
                g_stMachineIdPara.byte5 = cmd_ptr->machineId[4];
                g_stMachineIdPara.byte6 = cmd_ptr->machineId[5]; 
				//限定参数设置范围
				if(cmd_ptr->currentMax < 3500){
					g_stMaxVILimitPara.current_max =  cmd_ptr->currentMax;
				}else{
					g_stMaxVILimitPara.current_max = 3500;
				}
				if(cmd_ptr->voltageMax < 5700){
					g_stMaxVILimitPara.voltage_max =  cmd_ptr->voltageMax;
				}else{
					g_stMaxVILimitPara.voltage_max = 5700;
				}
				
                buff[13] = GetModbusCrc16Code( &buff[3],10) & 0xFF;
                buff[14] = GetModbusCrc16Code( &buff[3],10) >> 8;
				//存储参数到Flash中
				StoreRemoteMachineIdPara(&g_stMachineIdPara);
				StoreLimitVIMaxValue(&g_stMaxVILimitPara);
				//重新载入新的参数
				GetRemoteMachineIdFromFlash(&g_stMachineIdPara);
				GetLimitVIMaxValueFromFlash(&g_stMaxVILimitPara);
				UpdateVIParaToPrgm();
				Uart3SendData(buff,16);//发送更新参数到上位机
				
            }else if( cmd_ptr->type == 0x01){//收到查询指令

                buff[3] = g_stMachineIdPara.byte1;
                buff[4] = g_stMachineIdPara.byte2;
                buff[5] = g_stMachineIdPara.byte3;
                buff[6] = g_stMachineIdPara.byte4;
                buff[7] = g_stMachineIdPara.byte5;
                buff[8] = g_stMachineIdPara.byte6;
                buff[9] = g_stMaxVILimitPara.current_max & 0xFF ;//大端模式
                buff[10]= (g_stMaxVILimitPara.current_max >> 8) & 0xFF ;
                buff[11]= g_stMaxVILimitPara.voltage_max & 0xFF;
                buff[12]= (g_stMaxVILimitPara.voltage_max >> 8) & 0xFF;
                buff[13] = GetModbusCrc16Code( &buff[1],12) & 0xFF;
                buff[14] = GetModbusCrc16Code( &buff[1],12) >> 8;	
                Uart3SendData(buff,16);
            }
        }
        isProcessFinish = DEF_TRUE;
    }
}

