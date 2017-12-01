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
* Filename      :  app_wireness_communicate_task.h
* Programmer(s) :  Fanjun
* Version       :  V1.0
* data          :  2017.4.20
* brief         :  This file contains all the functions prototypes for the system run
*                  config parameters firmware library.
*
*********************************************************************************************************/
#ifndef __APP_WIRENESS_COMMUNICATE_TASK_H__
#define __APP_WIRENESS_COMMUNICATE_TASK_H__
/*
***************************************************************************************************
*                                           INCLUDE FILES
***************************************************************************************************
*/
#include "stdint.h"
#include "os.h"
/*
***************************************************************************************************
*                                           MACRO DEFINITIONS
***************************************************************************************************
*/


#define PRGM_TX_BUFF_SIZE       60
#define PRGM_RX_BUFF_SIZE       16

#define TX_MSG_SEND_QUEUE_SIZE          10

/* Send queue data status */
#define EN_FREE             0
#define EN_WAITTING         1
#define EN_SENDING          2
/* Data update status */
#define EN_LATEST           0
#define EN_HISTORY          1
/*
***************************************************************************************************
*                                    EXTERNAL OS VARIABLE DECLARATIONS
***************************************************************************************************
*/
extern  OS_SEM          g_stCommDataSendResponseSem;

extern  OS_TCB          CommTaskTCB;
extern  OS_TCB          CommDataSendTaskTCB;
/*
***************************************************************************************************
*                                           EXPORTED GLOBAL VARIABLE DECLARATIONS
***************************************************************************************************
*/

/*
***************************************************************************************************
*                                           EXPORTED DATA TYPE
***************************************************************************************************
*/

//typedef struct {

//    


//}HYDROGEN_PRODUCER_RUNNING_INFO_Typedef;

typedef enum {
    LOCAL_NETWORK_HOST = 0,             //本地组网主机
    LOCAL_NETWORK_SLAVE,                //本地组网从机
    DEBUG_APP_PORT,                     //调试APP
    CLIENT_APP_PORT,                    //客户APP
    OPERATE_APP_PORT,                   //运营APP
    COUNTER_PORT,                       //柜台端
    BACKGROUND_CONTROL_SERVER,          //后台服务器
    REPAIR_PORT,                        //维修端
    DEBUG_PC_PORT,                      //调试PC端
    RESERVED_PORT_1,
    RESERVED_PORT_2,
    RESERVED_PORT_3,
    RESERVED_PORT_4,
    RESERVED_PORT_5,
    RESERVED_PORT_6,
    RESERVED_PORT_7,

} CONTROL_TEERMINAL_TYPE_Typedef;


/* Send packets byte sequence definition */
typedef enum {
    HEAD_BYTE_ONE = 0,
    HEAD_BYTE_TWO = 1,
    HEAD_BYTE_THREE = 2,
    PRODUCTS_TYPE_ID_HIGH = 3,
    PRODUCTS_TYPE_ID_LOW,
    LOCAL_NETWORK_ID_CODE,              //本地组网ID

    INFORMATION_TYPE_CODE = 6,      //信息类型控制码

    DATA_IDENTIFY_TAG_INF_CODE_1 = 7,    //数据身份标签码
    DATA_IDENTIFY_TAG_INF_CODE_2,
    DATA_IDENTIFY_TAG_INF_CODE_3,
    DATA_IDENTIFY_TAG_INF_CODE_4,

    VALID_INFO_LEN_CTRL_CODE = 11,

    /*实时请求信息数据*/
    REQUEST_INFORMATION_TYPE = 12,
    LENGTH_OF_REQUEST_PARAMETERS,
    REQUEST_PARA_ONE,
    REQUEST_PARA_TWO,
    REQUEST_PARA_THREE,
    REQUEST_PARA_FOUR,

    /**实时辅助信息**/
    LEGAL_AUTHORIZATION_CODE = 12,

    SELF_CHECK_SENSOR_STATUS_CODE = 13,
    SELF_CHECK_CODE_GROUP_HYDRGEN_BYTE_4 = 14,
    SELF_CHECK_CODE_GROUP_HYDRGEN_BYTE_3,
    SELF_CHECK_CODE_GROUP_HYDRGEN_BYTE_2,
    SELF_CHECK_CODE_GROUP_HYDRGEN_BYTE_1,

    SELF_CHECK_CODE_GROUP_FUEL_CELL_BYTE_4 = 18,
    SELF_CHECK_CODE_GROUP_FUEL_CELL_BYTE_3,
    SELF_CHECK_CODE_GROUP_FUEL_CELL_BYTE_2,
    SELF_CHECK_CODE_GROUP_FUEL_CELL_BYTE_1,

    CTRL_AND_COMM_STATU_CODE_BYTE_H = 22,
    CTRL_AND_COMM_STATU_CODE_BYTE_L,

    /*实时运行信息段1--40个字节*/
    RUN_ALARM_CODE_BYTE_4 = 12,
    RUN_ALARM_CODE_BYTE_3,
    RUN_ALARM_CODE_BYTE_2,
    RUN_ALARM_CODE_BYTE_1,

    RUNNING_STATU_CODE_BYTE_4 = 16,
    RUNNING_STATU_CODE_BYTE_3,
    RUNNING_STATU_CODE_BYTE_2,
    RUNNING_STATU_CODE_BYTE_1,

    REFORMER_TEMP_HIGH = 20,
    REFORMER_TEMP_LOW,
    FIRE_OR_ROD_TEMP_HIGH,
    FIRE_OR_ROD_TEMP_LOW,

    LIQUID_PRESS_INTEGER_PART = 24,
    LIQUID_PRESS_DECIMAL_PART,

    HYDROGEN_FAN_SPD_CONTROL_HIGH = 26,
    HYDROGEN_FAN_SPD_CONTROL_LOW,
    HYDROGEN_FAN_SPD_FEEDBACK_HIGH,
    HYDROGEN_FAN_SPD_FEEDBACK_LOW,

    PUMP_SPD_CONTROL_HIGH = 30,
    PUMP_SPD_CONTROL_LOW,
    PUMP_SPD_FEEDBACK_HIGH,
    PUMP_SPD_FEEDBACK_LOW,

    HYDROGEN_PRODUCT_TIME_THIS_TIME_HOUR_HIGH = 34,
    HYDROGEN_PRODUCT_TIME_THIS_TIME_HOUR_LOW,
    HYDROGEN_PRODUCT_TIME_THIS_TIME_MINUTE,
    HYDROGEN_PRODUCT_TIME_THIS_TIME_SECOND,

    HYDROGEN_PRODUCT_TIME_TOTAL_HOUR_HIGH = 38,
    HYDROGEN_PRODUCT_TIME_TOTAL_HOUR_LOW,
    HYDROGEN_PRODUCT_TIME_TOTAL_MINUTE,
    HYDROGEN_PRODUCT_TIME_TOTAL_SECOND,

    HYDROGEN_PRODUCT_TOTAL_TIMES_HIGH = 42,
    HYDROGEN_PRODUCT_TOTAL_TIMES_LOW,

    LIQUID_LEVEL_INTEGER_PART = 44,
    LIQUID_LEVEL_DECIMAL_PART,

    FUEL_WEIGHT_INTEGER_PART_HIGH = 46,
    FUEL_WEIGHT_INTEGER_PART_MID,
    FUEL_WEIGHT_INTEGER_PART_LOW,
    FUEL_WEIGHT_DECIMAL_PART,

//    LIQUID_FEED_PER_MINUTE_MUL_10_INTEGER_PART = 50,
//    LIQUID_FEED_PER_MINUTE_MUL_10_DECIMAL_PART,
    
    VACUUM_NEGATIVE_PRESSURE_HIGH = 50,//真空负压值-移动到1-2组里面
    VACUUM_NEGATIVE_PRESSURE_LOW,

    HYDROGEN_EXPORT_PRESS_DIV100_VALUE_HIGH = 52,//出气口气压值
    HYDROGEN_EXPORT_PRESS_DIV100_VALUE_LOW,

    /*实时运行信息段1-2--28个字节*/
    /*前8个字节与实时信息段1-1相同*/
    EXTEND_TEMP_VALUE_1_HIGH = 20,
    EXTEND_TEMP_VALUE_1_LOW,
    
    EXTEND_TEMP_VALUE_2_HIGH = 22,
    EXTEND_TEMP_VALUE_2_LOW,
    
    EXTEND_TEMP_VALUE_3_HIGH = 24,
    EXTEND_TEMP_VALUE_3_LOW,
    
    EXTEND_TEMP_VALUE_4_HIGH = 26,
    EXTEND_TEMP_VALUE_4_LOW,
    
    EXTEND_TEMP_VALUE_5_HIGH = 28,
    EXTEND_TEMP_VALUE_5_LOW,
    
    EXTEND_TEMP_VALUE_6_HIGH = 30,
    EXTEND_TEMP_VALUE_6_LOW,
    
    PUMP_2_CTRL_SPD_HIGH = 32,
    PUMP_2_CTRL_SPD_LOW,
    
    PUMP_2_FEEDBACK_SPD_HIGH = 34,
    PUMP_2_FEEDBACK_SPD_LOW,
    
    REMAIND_HOUR_OF_RICH_HYDROGEN_ACTIVATION = 36,//富氢活化本阶段剩余时间
    REMAIND_MINUTE_OF_RICH_HYDROGEN_ACTIVATION,
    
    VACUUM_NEGATIVE_PRESSURE_B_HIGH = 38,
    VACUUM_NEGATIVE_PRESSURE_B_LOW,
    
//    HYDROGEN_ACTIVATION_CURRENT_STEP = 38,//新添加字节
    
    /*实时运行信息段2-1 — 45个字节,12~19字节与段1相同*/
    STACK_CURRENT_INTEGER_PART = 20, //Amp
    STACK_CURRENT_DECIMAL_PART,

    STACK_VOLTAGE_INTEGER_PART = 22, //Voltage
    STACK_VOLTAGE_DECIMAL_PART,

    STACK_TEMP_INTEGER_PART = 24,
    STACK_TEMP_DECIMAL_PART,

    STACK_HYDROGEN_PRESS_INTEGER_PART = 26,//KPa
    STACK_HYDROGEN_PRESS_DECIMAL_PART,

    STACK_FAN_SPD_CONTROL_HIGH = 28,
    STACK_FAN_SPD_CONTROL_LOW,
    STACK_FAN_PART_B_SPD_FEEDBACK_HIGH,
    STACK_FAN_PART_B_SPD_FEEDBACK_LOW,
    STACK_FAN_PART_A_SPD_FEEDBACK_HIGH,
    STACK_FAN_PART_A_SPD_FEEDBACK_LOW,

    STACK_WORK_TIME_THIS_TIME_HOUR_HIGH = 34,
    STACK_WORK_TIME_THIS_TIME_HOUR_LOW,
    STACK_WORK_TIME_THIS_TIME_MINUTE,
    STACK_WORK_TIME_THIS_TIME_SECOND,

    STACK_WORK_TIME_TOTAL_HOUR_HIGH = 38,
    STACK_WORK_TIME_TOTAL_HOUR_LOW,
    STACK_WORK_TIME_TOTAL_MINUTE,
    STACK_WORK_TIME_TOTAL_SECOND,

    STACK_WORK_TOTAL_TIMES_HIGH = 42,
    STACK_WORK_TOTAL_TIMES_LOW,

    //当前单机输出功率
    CURRENT_ISOLATED_POWER_INTEGER_PART_HIGH = 44,
    CURRENT_ISOLATED_POWER_INTEGER_PART_LOW,
    CURRENT_ISOLATED_POWER_DECIMAL_PART,

    //本次单机发电量
    ISOLATED_GENERATED_ENERGY_THIE_TIME_INTEGER_PART_HIGH = 47,
    ISOLATED_GENERATED_ENERGY_THIE_TIME_INTEGER_PART_LOW,
    ISOLATED_GENERATED_ENERGY_THIE_TIME_DECIMAL_PART_HIGH,
    ISOLATED_GENERATED_ENERGY_THIE_TIME_DECIMAL_PART_LOW,

    //单机累计发电量
    GENERATED_ENERGY_TOTAL_INTEGER_PART_HIGH = 51,
    GENERATED_ENERGY_TOTAL_INTEGER_PART_LOW,
    GENERATED_ENERGY_TOTAL_DECIMAL_PART_HIGH,
    GENERATED_ENERGY_TOTAL_DECIMAL_PART_LOW,

    HYDROGEN_YIELD_MATCHING_OFFSET_VALUE_INTEGER_PART_MUL100 = 55,
    HYDROGEN_YIELD_MATCHING_OFFSET_VALUE_DECIMAL_PART_MUL100_HIGH,

    SUB_MODULE_ID_OF_THE_MULTI_MODULE_TYPE = 57,

    END_BYTE_ONE = 58,
    END_BYTE_TWO,

    /*实时运行信息段2-2 — 6个字节*/
    STACK_DECOMPRESS_COUNT_PER_MINUTES_HIGH = 12,
    STACK_DECOMPRESS_COUNT_PER_MINUTES_LOW,

    BATTERY_VOLTAGE_INTEGER_PART = 14,
    BATTERY_VOLTAGE_IDECIMAL_PART,

    BATTERY_CURRENT_INTEGER_PART = 16,
    BATTERY_CURRENT_IDECIMAL_PART,

    /*固定辅助信息--产地、批次等*/

    /*备询、配置信息位定义*/
    HEAT_STATUS_PUMP_CONTROL_SPD_HIGH = 12,
    HEAT_STATUS_PUMP_CONTROL_SPD_LOW,
    HEAT_STATUS_HYDROGEN_FAN_CONTROL_SPD_HIGH = 18,
    HEAT_STATUS_HYDROGEN_FAN_CONTROL_SPD_LOW,

    FAST_HEAT_HOLD_SECONDS_VALUE_HIGH = 26,
    FAST_HEAT_HOLD_SECONDS_VALUE_LOW,

    RUNNING_STATUS_PUMP_SPD_CONTROL_HIGH = 30,
    RUNNING_STATUS_PUMP_SPD_CONTROL_LOW,
    
    RUNNING_STATUS_HYDROGEN_FAN_SPD_CONTROL_HIGH = 36,
    RUNNING_STATUS_HYDROGEN_FAN_SPD_CONTROL_LOW,
    RUNNING_STATUS_FAN_DLY_ADJUST_SEC_HIGH,
    RUNNING_STATUS_FAN_DLY_ADJUST_SEC_LOW,
    
    HYDROGEN_ACTIVATED_STEP1_PUMP_SPD_HIGH = 40,
    HYDROGEN_ACTIVATED_STEP1_PUMP_SPD_LOW,
    HYDROGEN_ACTIVATED_STEP1_HOLD_TIME_HOUR,
    HYDROGEN_ACTIVATED_STEP1_HOLD_TIME_MIN,
    HYDROGEN_ACTIVATED_STEP2_PUMP_SPD_HIGH = 44,
    HYDROGEN_ACTIVATED_STEP2_PUMP_SPD_LOW,
    HYDROGEN_ACTIVATED_STEP2_HOLD_TIME_HOUR,
    HYDROGEN_ACTIVATED_STEP2_HOLD_TIME_MIN,
    HYDROGEN_ACTIVATED_STEP3_PUMP_SPD_HIGH = 48,
    HYDROGEN_ACTIVATED_STEP3_PUMP_SPD_LOW,
    HYDROGEN_ACTIVATED_STEP3_HOLD_TIME_HOUR,
    HYDROGEN_ACTIVATED_STEP3_HOLD_TIME_MIN,
    HYDROGEN_ACTIVATED_STEP4_PUMP_SPD_HIGH = 52,
    HYDROGEN_ACTIVATED_STEP4_PUMP_SPD_LOW,
    HYDROGEN_ACTIVATED_STEP4_HOLD_TIME_HOUR,
    HYDROGEN_ACTIVATED_STEP4_HOLD_TIME_MIN,

} PRGM_TX_MSG_DATA_ADDR_Typedef;

/*
***************************************************************************************************
*                                    EXTERNAL VARIABLE DECLARATIONS
***************************************************************************************************
*/

/*
***************************************************************************************************
*                                           EXPORTED CONSTANTS
***************************************************************************************************
*/

/*
***************************************************************************************************
*                                           EXPORTED MACRO
***************************************************************************************************
*/

/*Send information type macro definition */
#define RT_RUNNING_INFO_A_PART_A      				0x0
#define RT_REQ_INFO               					0x01
#define RT_ASSIST_INFO                				0x02
#define CONSTANT_ASSIST_INFO                 		0x03
#define RT_RUNNING_INFO_B_PART_A      				0x04
#define FOR_QUERY_AND_CFG_INFO            		0x05//备询、配置信息
#define RT_RUNNING_INFO_B_PART_B      				0x06
#define RT_RUNNING_INFO_A_PART_B      				0x07
#define EN_SEND_DATA_TYPE_MAX                       0x08


/*All kinds of sending message length macro definition*/
#define RT_RUNNING_INFO_A_1_LEN                       42//对接协议为34
#define RT_RUNNING_INFO_A_2_LEN                       28
#define RT_RUNNING_INFO_B_1_LENGTH                     45//对接协议为43
#define RT_RUNNING_INFO_B_2_LENGTH                     8
#define RT_REQUEST_INFO_LENGTH                       6
#define RT_ASSIST_INFO_LENGTH                        12
#define CONSTANT_ASSIST_INFO_LEN                  0
#define FOR_QUERY_AND_CFG_INFO_LEN             42

/*Macro define about the real time qequest instructions*/
#define RT_REQUEST_INFO_REQUEST_ID_ALLOCATION                0x01
#define RT_REQUEST_INFO_REQUEST_SHUT_DOWN                    0x02
#define RT_REQUEST_INFO_REQUEST_CHOOSE_WORK_MODE             0x03

/*Macro define about the control conmmand type*/
#define CMD_TYPE_DBG                 0x00
#define CMD_TYPE_REMOTE_CTRL      0x01
#define CMD_TYPE_CFG                 0x02
#define CMD_TYPE_CLR                 0x03
#define CMD_TYPE_REQ                 0x04
#define CMD_TYPE_CONFIRM             0x05



/*Packet received byte sequence definition*/
#define REC_DATA_BYTE_HEAD_ONE                              0
#define REC_DATA_BYTE_HEAD_TWO                              1
#define REC_DATA_BYTE_HEAD_THREE                            2
#define REC_DATA_BYTE_HEAD_CMD_SOURCE                       3
#define REC_DATA_BYTE_HEAD_TARGET_LOCAL_NET_ID              4//受控机器在其组网网络中的 ID 号
#define REC_DATA_BYTE_CMD_TYPE                              5
#define REC_DATA_BYTE_CMD_CODE_VALUE                        6
#define REC_DATA_BYTE_CMD_PARAMETERS_LENGTH                 7
#define REC_DATA_BYTE_CMD_PARA_SECTION_1                       8
#define REC_DATA_BYTE_CMD_PARA_SECTION_2                       9
#define REC_DATA_BYTE_CMD_PARA_SECTION_3                       10
#define REC_DATA_BYTE_CMD_PARA_SECTION_4                       11
#define REC_DATA_BYTE_CMD_PARA_SECTION_5                       12
#define REC_DATA_BYTE_CMD_PARA_SECTION_6                       13
#define REC_DATA_BYTE_CMD_CHILD_MODULE_ID                       14
#define REC_DATA_BYTE_END_OF_DATA                  15


/*Macro define about the debug mode instructions*/
#define DBG_SELECT_WORK_MODE                                    0x02
#define DBG_SWITCH_OVER_CONTROL_MODE                            0x03
#define DBG_START_THE_MACHINE                                   0x04
#define DBG_AHEAD_RUNNING                                       0x05
#define DBG_SHUT_DOWN_THE_MACHINE                               0x06
#define DBG_CHANGE_THE_WORK_STATU_TO_STANDING_BY                0x07

#define DBG_PUMP1_SPEED_INC                                     0x20
#define DBG_PUMP1_SPEED_DEC                                     0x21
#define DBG_PUMP1_SPEED_SET_WITH_PARAMETERS                     0x22
#define DBG_HYDRG_SPEED_INC                                     0x23
#define DBG_HYDRG_SPEED_DEC                                     0x24
#define DBG_HYDRG_SPEED_SET_WHTI_PARAMETERS                     0x25
#define DBG_PUMP2_SPEED_PARAMETERS_SET                          0x26

#define DBG_OPEN_IGNITER                                        0x30
#define DBG_CLOSE_IGNITER                                       0x31

#define DBG_STACK_FAN_SPEED_INC                                 0x50
#define DBG_STACK_FAN_SPEED_DEC                                 0x51
#define DBG_STACK_FAN_SPEED_SET_WITH_PARAMTERS                  0x52

//查询、请求指令
#define REQUEST_DATA_RETRANSMIT                                 0x02
#define INQUIRE_HYDROGEN_RUNNING_PARAMETERS                     0x03


#define RESPONSE_ALLOCATE_ID_NMB_WITH_PARAMETERS                0x02
#define RESPONSE_SLAVE_SHUT_DOWN_CMD                            0x03

#define RESPONSE_SLAVE_SHUT_DOWN_CMD_RIGHT_NOW                  0x01
#define RESPONSE_SLAVE_SHUT_DOWN_CMD_DELAY                      0x02


/*Macro define about the config part*/
#define CONFIG_HYDROGEN_GROUP_RUNNING_PARA      0x02
#define CONFIG_FUEL_CELL_GROUP_RUNNING_PARA     0x03

/*Macro define about the config step*/
#define CONFIG_HEAT_STEP_PARA                   0x02
#define CONFIG_RUNNING_STATUS_PARA              0x03
#define CONFIG_SHUTING_DOWN_STEP_PARA           0x04
#define CONFIG_RICH_HYDROG_ACTIVE_PARA           0xFF

/*Macro define about config instruction*/
//Ignite first time step
#define CONFIG_HEAT_STEP_PUMP_SPD                           0x01
#define CONFIG_HEAT_PUMP_SPD_SMOOTHLY_CTRL_SEC              0x02
#define CONFIG_HEAT_STEP_FAN_SPD                            0x04
#define CONFIG_HEAT_STEP_FAN_SPD_SMOOTHLY_CTRL_SEC          0x05
#define CONFIG_HEAT_HOLD_TIME_BY_SEC                        0x08

//Ignite second time step
#define CONFIG_RUNNING_STEP_PUMP_SPD                          0x01
#define CONFIG_RUNNING_PUMP_SPD_SMOOTHLY_CTRL_SEC             0x02
#define CONFIG_RUNNING_STEP_FAN_SPD                           0x04
#define CONFIG_RUNNING_FAN_SPD_SMOOTHLY_CTRL_SEC              0x05
#define CONFIG_RUNNING_FAN_SPD_DLY_CTRL_MIN                   0x07
//rich hydrogen active para
#define CONFIG_RICH_HYDROG_ACTIVE_STEP                      0x00
#define CONFIG_RICH_HYDROG_ACTIVE_STEP_PUMP_SPD             0x01
#define CONFIG_RICH_HYDROG_ACTIVE_STEP_FAN_SPD              0x02
#define CONFIG_RICH_HYDROG_ACTIVE_STEP_HOLD_TIME            0x03
#define CONFIG_HYDROG_PRODUCER_WORK_MODE_NEXT_TIME          0x04

/*
***************************************************************************************************
*                                           EXPORTED FUNCTION
***************************************************************************************************
*/
void    CommTaskCreate(void);
void    CommDataSendTaskCreate(void);
void    CmdStart(void);
void    CmdShutDown(void);

void    SendShutDownRequest(void);
void    SendChooseWorkModeRequest(void);
void    SendRealTimeAssistInfo(void);
void    SendInquireOrConfigurationInfo(void);

uint8_t *GetPrgmRxBuffAddr(void);
uint8_t GetPrgmRxBuffLen(void);

#endif

