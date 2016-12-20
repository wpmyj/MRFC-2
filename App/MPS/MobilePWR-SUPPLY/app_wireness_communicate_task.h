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
* data          :  2016.12.10
* brief         :  This file contains all the functions prototypes for the system run
*                  config parameters firmware library.
*********************************************************************************************************/
#ifndef __APP_WIRENESS_COMMUNICATE_TASK_H__
#define __APP_WIRENESS_COMMUNICATE_TASK_H__
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
#define PRODUCT_MODEL_CODE      0x1100              //PS3
#define LOCAL_NETWORK_ID        0
#define PRGM_TX_BUFF_SIZE       60
#define PRGM_RX_BUFF_SIZE       16

#define TX_MSG_SEND_QUEUE_SIZE          5

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
extern  OS_TCB          CommunicateTaskTCB;
extern  OS_TCB          CommunicateDataSendTaskTCB;
extern  OS_TCB          CommunicateRequsetInfSendTaskTCB;
extern  OS_FLAG_GRP     ConfigParametersChangeState;
extern  OS_SEM          g_stCommunicateDataSendResponseSem;
/*
***************************************************************************************************
*                                           EXPORTED GLOBAL VARIABLE DECLARATIONS
***************************************************************************************************
*/
extern      uint8_t     g_u8SerRxMsgBuff[PRGM_RX_BUFF_SIZE];
/*
***************************************************************************************************
*                                           EXPORTED DATA TYPE
***************************************************************************************************
*/
typedef struct {
    uint8_t D_last;
    uint8_t D_next;
    uint8_t OccupyStatu;
    uint8_t Data[PRGM_TX_BUFF_SIZE];

} PRGM_TX_DATA_FRAME_Typedef;

typedef struct {
    uint8_t Q_length;
    uint8_t Q_Qhead;
    uint8_t Q_Qrear;
    PRGM_TX_DATA_FRAME_Typedef Queue[TX_MSG_SEND_QUEUE_SIZE];   //队列帧数据

} TX_MSG_SEND_BUFF_Typedef;


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
    INFORMATION_TYPE_CONTROL_CODE,      //信息类型控制码

//      DATA_IDENTIFY_TAG_INF_CODE_1 =7,            //数据身份标签码
//      DATA_IDENTIFY_TAG_INF_CODE_2,
//      DATA_IDENTIFY_TAG_INF_CODE_3,
//      DATA_IDENTIFY_TAG_INF_CODE_4,
    VALID_INFORMATION_LENGTH_CONTROL_CODE = 11,

    /*实时请求信息数据*/
    REQUEST_INFORMATION_TYPE = 12,
    LENGTH_OF_REQUEST_PARAMETERS,
    REQUEST_PARAMETER_ONE,
    REQUEST_PARAMETER_TWO,
    REQUEST_PARAMETER_THREE,
    REQUEST_PARAMETER_FOUR,

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

    CONTROL_AND_COMMUNICATE_STATU_CODE_BYTE_H = 22,
    CONTROL_AND_COMMUNICATE_STATU_CODE_BYTE_L,

    /*实时运行信息段1--34个字节*/
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

    FUEL_WEIGHT_INTEGER_PART_HIGH = 46, //燃料重量
    FUEL_WEIGHT_INTEGER_PART_MID,
    FUEL_WEIGHT_INTEGER_PART_LOW,
    FUEL_WEIGHT_DECIMAL_PART,

    /*实时运行信息段2--43个字节,12~19字节与段1相同*/
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



    /*固定辅助信息--产地、批次等*/

    /*备询、配置信息位定义*/
    IGNITE_FIRST_STEP_PUMP_SPD_HIGH = 12,
    IGNITE_FIRST_STEP_PUMP_SPD_LOW,

    IGNITE_FIRST_STEP_FAN_SPD_HIGH = 18,
    IGNITE_FIRST_STEP_FAN_SPD_LOW,

    IGNITE_FIRST_SUCCESSED_FAN_SPD_HIGH = 22,
    IGNITE_FIRST_SUCCESSED_FAN_SPD_LOW,

    IGNITE_SECOND_STEP_PUMP_SPD_HIGH = 26,
    IGNITE_SECOND_STEP_PUMP_SPD_LOW,

    IGNITE_SECOND_STEP_FAN_SPD_HIGH = 32,
    IGNITE_SECOND_STEP_FAN_SPD_LOW,

    IGNITE_SECOND_SUCCESSED_FAN_SPD_HIGH = 36,
    IGNITE_SECOND_SUCCESSED_FAN_SPD_LOW,

    RUNNING_STATUS_LIQUID_PRESS_EXCEED_4KG_PUMP_SPD_HIGH = 42,
    RUNNING_STATUS_LIQUID_PRESS_EXCEED_4KG_PUMP_SPD_LOW,

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
#define REAL_TIME_RUNNING_INFORMATION_A         0x0
#define REAL_TIME_REQUEST_INFORMATION           0x01
#define REAL_TIME_ASSIST_INFORMATION            0x02
#define CONSTANT_ASSIST_INFORMATION             0x03
#define REAL_TIME_RUNNING_INFORMATION_B         0x04
#define FOR_QUERY_AND_CONFIG_INFORMATION        0x05    //备询、配置信息
#define EN_SEND_DATA_TYPE_MAX                   0x06

/*All kinds of sending message length macro definition*/
#define REAL_TIME_RUNNING_INFO_A_LENGTH                     34
#define REAL_TIME_RUNNING_INFO_B_LENGTH                     43
#define REAL_TIME_REQUEST_INFO_LENGTH                       6
#define REAL_TIME_ASSIST_INFO_LENGTH                        12
#define CONSTANT_ASSIST_INFORMATION_LENGTH                  0
#define FOR_QUERY_AND_CONFIG_INFORMATION_LENGTH             30

/*Macro define about the real time qequest instructions*/
#define REAL_TIME_REQUEST_INFO_REQUEST_ID_ALLOCATION                0x01
#define REAL_TIME_REQUEST_INFO_REQUEST_SHUT_DOWN                    0x02
#define REAL_TIME_REQUEST_INFO_REQUEST_CHOOSE_WORK_MODE             0x03

/*Macro define about the control conmmand type*/
#define COMMAND_TYPE_DBG                0x00
#define COMMAND_TYPE_REMOTE_CONTRO      0x01
#define COMMAND_TYPE_CONFIGURATION      0x02
#define COMMAND_TYPE_CLEAR              0x03
#define COMMAND_TYPE_INQUIRE_REQUEST    0x04
#define COMMAND_TYPE_RESPONSE_CONFIRM   0x05



/*Packet received byte sequence definition*/
#define RECEIVE_DATA_BYTE_HEAD_ONE                              0
#define RECEIVE_DATA_BYTE_HEAD_TWO                              1
#define RECEIVE_DATA_BYTE_HEAD_THREE                            2
#define RECEIVE_DATA_BYTE_HEAD_CMD_SOURCE                       3
#define RECEIVE_DATA_BYTE_HEAD_TARGET_LOCAL_NET_ID              4
#define RECEIVE_DATA_BYTE_CMD_TYPE                              5
#define REDEIVE_DATA_BYTE_CMD_CODE_VALUE                        6
#define REDEIVE_DATA_BYTE_CMD_PARAMETERS_LENGTH                 7
#define REDEIVE_DATA_BYTE_CMD_PARAMETER_SECTION_ONE_1                       8
#define REDEIVE_DATA_BYTE_CMD_PARAMETER_SECTION_ONE_2                       9
#define REDEIVE_DATA_BYTE_CMD_PARAMETER_SECTION_ONE_3                       10
#define REDEIVE_DATA_BYTE_CMD_PARAMETER_SECTION_ONE_4                       11
#define REDEIVE_DATA_BYTE_CMD_PARAMETER_SECTION_ONE_5                       12
#define REDEIVE_DATA_BYTE_CMD_PARAMETER_SECTION_ONE_6                       13
#define REDEIVE_DATA_BYTE_CMD_CHILD_MODULE_ID                       14
#define REDEIVE_DATA_BYTE_END_OF_DATA                  15



/*Macro define about the debug mode instructions*/
#define DBG_SELECT_WORK_MODE                                    0x02
#define DBG_SWITCH_OVER_CONTROL_MODE                            0x03
#define DBG_START_THE_MACHINE                                   0x04
#define DBG_AHEAD_RUNNING                                       0x05
#define DBG_SHUT_DOWN_THE_MACHINE                               0x06
#define DBG_CHANGE_THE_WORK_STATU_TO_STANDING_BY                0x07

#define DBG_PUMP_SPEED_INC                                      0x20
#define DBG_PUMP_SPEED_DEC                                      0x21
#define DBG_PUMP_SPEED_SET_WITH_PARAMETERS                      0x22
#define DBG_HYDRG_SPEED_INC                                     0x23
#define DBG_HYDRG_SPEED_DEC                                     0x24
#define DBG_HYDRG_SPEED_SET_WHTI_PARAMETERS                     0x25

#define DBG_OPEN_IGNITER                                        0x30
#define DBG_CLOSE_IGNITER                                       0x31

#define DBG_STACK_FAN_SPEED_INC                                 0x50
#define DBG_STACK_FAN_SPEED_DEC                                 0x51
#define DBG_STACK_FAN_SPEED_SET_WITH_PARAMTERS                  0x52

#define QUERY_REQUEST_DATA_RETRANSMIT                           0x02

#define RESPONSE_ALLOCATE_ID_NMB_WITH_PARAMETERS                0x02
#define RESPONSE_SLAVE_SHUT_DOWN_CMD                            0x03

#define RESPONSE_SLAVE_SHUT_DOWN_CMD_RIGHT_NOW                  0x01
#define RESPONSE_SLAVE_SHUT_DOWN_CMD_DELAY                      0x02


/*Macro define about the config part*/
#define CONFIG_HYDROGEN_GROUP_RUNNING_PARA      0x02
#define CONFIG_FUEL_CELL_GROUP_RUNNING_PARA     0x03

/*Macro define about the config step*/
#define CONFIG_IGNITE_FIRST_STEP_PARA      0x02
#define CONFIG_IGNITE_SECOND_STEP_PARA     0x03
#define CONFIG_RUNNING_STATUS_PARA         0x04
#define CONFIG_SHUTING_DOWN_STEP_PARA      0x05

/*Macro define about config instruction*/
#define CONFIG_IGNITE_FIRST_STEP_PUMP_SPD                           0x01
#define CONFIG_IGNITE_FIRST_STEP_FAN_SPD                            0x04
#define CONFIG_IGNITE_FIRST_SUCCESSED_FAN_SPD                       0x05

#define CONFIG_IGNITE_SECOND_STEP_PUMP_SPD                          0x01
#define CONFIG_IGNITE_SECOND_STEP_FAN_SPD                           0x04
#define CONFIG_IGNITE_SECOND_SUCCESSED_FAN_SPD                      0x07

#define CONFIG_RUNNING_STATUS_LIQUID_PRESS_EXCEED_4KG_PUMP_SPD      0x01

/*Macro define about whether the parameters config success*/
#define CFG_IGNITE_FIRST_STEP_PUMP_SPD                           0x01
#define CFG_IGNITE_FIRST_STEP_FAN_SPD                            0x02
#define CFG_IGNITE_FIRST_SUCCESSED_FAN_SPD                       0x03

#define CFG_IGNITE_SECOND_STEP_PUMP_SPD                          0x0A
#define CFG_IGNITE_SECOND_STEP_FAN_SPD                           0x0B
#define CFG_IGNITE_SECOND_SUCCESSED_FAN_SPD                      0x0C

#define CFG_LIQUID_PRESS_EXCEED_4KG_PUMP_SPD                     0x0D

/*
***************************************************************************************************
*                                           EXPORTED FUNCTION
***************************************************************************************************
*/
void    CommunicateTaskCreate(void);
void    CmdStart(void);
void    CmdShutDown(void);

void    SendShutDownRequest(void);
void    SendChooseWorkModeRequest(void);
void    SendRealTimeAssistInfo(void);

#endif

