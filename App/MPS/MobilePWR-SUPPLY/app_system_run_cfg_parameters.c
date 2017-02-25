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
* Filename      : app_system_real_time_parameters.c
* Version       : V1.00
* Programmer(s) : FanJun
***************************************************************************************************
*/
/*
***************************************************************************************************
*                                           INCLUDE FILES
***************************************************************************************************
*/
#include "includes.h"
#include "app_system_run_cfg_parameters.h"
#include "app_system_real_time_parameters.h"
#include "bsp_ana_sensor.h"
/*
***************************************************************************************************
*                                           MACRO DEFINITIONS
***************************************************************************************************
*/
#define STM32_FLASH_SIZE            128             //����STM32��FLASH������С(��λΪK)
#define STM32_FLASH_WREN            1               //ʹ��FLASHд��(0:ʧ��;1:ʹ��)

#define STM32_FLASH_BASE            0x08000000      //STM32 FLASH����ʼ��ַ

#define SYSTEM_PARAMETER_STORE_SEGMENT_ADDR         0x0801FC00      //��ַ픶˞�0x08020000 ����FLASH �����ַ�������1K�ֹ�(����Ϊż��)

#define PARAMETERS_STORE_AREA_SIZE                      512 //�����������Ĵ�С���ֽڣ�

//����궨���ʾ�������е����������������������Ӧ�ò㹤�������ǲ����趨ֵ����Ĭ��ֵ�ı�־ֵ��ƫ�Ƶ�ַ
//�޷���16λ����������Ϊȫ0xFFFF
//bit[0]��ʾӦ�ò��ǲ���Ĭ��ֵ��������ֵ--0:����ֵ  1:Ĭ��ֵ
//bit[1]��ʾ�������������Ĭ��ֵ��������--0:����ֵ  1:Ĭ��ֵ
#define APP_PARAMETERS_SELECT_FLAG_CHECK                                0x0001
#define APP_PARAMETERS_SELECT_CFG_VALUE                                 0x0000
#define APP_PARAMETERS_SELECT_DEFAULT_VALUE                             0x0001

#define DRIVER_LAYER_PARAMETERS_SELECT_FLAG_CHECK                       0x0002
#define DRIVER_LAYER_PARAMETERS_SELECT_CFG_VALUE                        0x0000  //ʹ�����ù��Ĳ���
#define DRIVER_LAYER_PARAMETERS_SELECT_DEFAULT_VALUE                    0x0002  //ʹ��Ĭ�ϲ���

/*���º궨���ʾ�����洢����ַ*/
#define PARAMETERS_SELECT_FLAG_OFFSET_ADDR                              0   //����ѡ���־
#define SYSTEM_RUN_TIME_STORE_OFFSET_ADDR                               2   //ϵͳ���д���
#define GLOBAL_NET_WORK_ID_ADDR                                         4   //��������ID
#define SYSTEM_TOTAL_WORK_TIME_STORE_OFFSET_ADDR                        6   //ϵͳʱ��ʱ��4������4���ֽ�
#define RUN_REFORMER_CMP_TBL_STORE_OFFSET_ADDR                          30  //12����
#define RUN_LIQUIDPRESS_CMP_TBL_STORE_OFFSET_ADDR                       50  // Һѹ4����
#define RUN_LIQUID_HEIGHT_CMP_TBL_STORE_OFFSET_ADDR                     60  //Һλ����4��
#define RUN_HYDROGEN_PUMP_SPEED_PARA_STORE_OFFSET_ADDR                  70  //�ò���3��
#define RUN_HYDROGEN_FAN_SPEED_PARA_STORE_OFFSET_ADDR                   80  //����������4��
#define RUN_PURIFY_AMP_INTEGRAL_VALUE                                   90  //����ᴿĤ������1��
#define DRIVER_LAYER_CALIBRATED_LINEAR_ANA_SENSOR_PARAMETERS_OFFSET_ADDR  92  //���������Դ�����У׼����,28����
#define RUNNING_STATUS_DELAY_ADJUST_SPEED_PARAMETERS_ADDR                130 //���е����豸��ʱ���ڲ��� 
#define FIRST_TIME_HEAT_HOLD_SECONDS_PARA_ADDR                           140 //���ȳ���ʱ��

/*
***************************************************************************************************
*                                     GLOBAL  VARIABLES
***************************************************************************************************
*/
REFORMER_TEMP_CMP_LINES_Typedef                         g_stReformerTempCmpTbl;
LIQUID_PRESSURE_CMP_LINES_Typedef                       g_stLqdPressCmpTbl;
LIQUID_HEIGHT_CMP_LINES_Typedef                         g_stLqdHeightCmpTbl;
HYDROGEN_PUMP_SPEED_PARA_Typedef                        g_stStartHydrgPumpSpdPara;
HYDROGEN_FAN_SPEED_PARA_Typedef                         g_stStartHydrgFanSpdPara;
RUNNING_STATUS_DELAY_ADJUST_SPEED_PARAMETERS_Typedef    g_stRunningStatusDelayAdjustSpdPara;
uint16_t                                                g_u8FirstTimeHeatHoldSeconds;
uint16_t                                                g_u16RunPurifyAmpIntegralValue;

//��CAN�����еĽڵ�ID/��������ID,Ĭ��Ϊ255��,�������еĻ�������Ϊ0
uint16_t 		                            g_u16GlobalNetWorkId = 0xFF;
/*
***************************************************************************************************
*                                     LOCAL VARIABLES
***************************************************************************************************
*/
//static     uint16_t      RW_ParametersBuffer[PARAMETERS_STORE_AREA_SIZE / 2];

/*
***************************************************************************************************
*                                         FUNCTION PROTOTYPES
***************************************************************************************************
*/
static          uint16_t    STMFLASH_ReadHalfWord(u32 faddr);         //��������
static          void        STMFLASH_Write_NoCheck(u32 WriteAddr, u16 *pBuffer, u16 NumToWrite);

static          void        STMFLASH_Write(u32 WriteAddr, u16 *pBuffer, u16 NumToWrite);    //��ָ����ַдָ�����ȵ�����
static          void        STMFLASH_Read(u32 ReadAddr, u16 *pBuffer, u16 NumToRead);       //��ָ����ַ��ָ�����ȵ�����

static          void        GetParametersSelectFlag(uint16_t *o_pParametersSelectFlagAddr);
static          void        SetParametersSelectFlag(uint16_t *i_pParametersSelectFlagAddr);

static          void        BSP_GetStoredAnaSensorsParaFromFlash(ANALOG_SIGNAL_SERSOR_PARAMETERS_Typedef *, uint8_t);
static          void        BSP_StoreAnaSensorParaToFlash(ANALOG_SIGNAL_SERSOR_PARAMETERS_Typedef *, uint8_t);

static          void        GetSystemWorkTimesFromFlash(u16 *o_StoreAddr);
static          void        GetTotalWorkTimeFromFlash(SYSTEM_TIME_Typedef *o_StoreAddr);
static          void        StoreTotalWorkTime(SYSTEM_TIME_Typedef *i_stTotalTime);

static          void        GetReformerTempCmpTblFromFlash(REFORMER_TEMP_CMP_LINES_Typedef *);
static          void        GetDefaultReformerTempCmpTbl(REFORMER_TEMP_CMP_LINES_Typedef *);
static          void        StoreReformerTempCmpTbl(REFORMER_TEMP_CMP_LINES_Typedef *);

static          void        GetLqdPressCmpTblFromFlash(LIQUID_PRESSURE_CMP_LINES_Typedef *);
static          void        GetDefaultLqdPressCmpTbl(LIQUID_PRESSURE_CMP_LINES_Typedef *);
static          void        StoreLqdPressCmpTbl(LIQUID_PRESSURE_CMP_LINES_Typedef *);

static          void        GetLqdHeightCmpTblFromFlash(LIQUID_HEIGHT_CMP_LINES_Typedef *i_LqdHeightCmpTbl);
static          void        GetDefaultLqdHeightCmpTbl(LIQUID_HEIGHT_CMP_LINES_Typedef *i_LqdHeightCmpTbl);
static          void        StoreLqdHeightCmpTbl(LIQUID_HEIGHT_CMP_LINES_Typedef *i_LqdHeightCmpTbl);

static          void        GetDefaultStartHydrgPumpSpdPara(HYDROGEN_PUMP_SPEED_PARA_Typedef *i_StartPumpSpdPara);
static          void        GetDefaultStartHydrgFanSpdPara(HYDROGEN_FAN_SPEED_PARA_Typedef *i_StartHydrgFanSpdPara);

static          void        GetRunPurifyAmpIntegralValueFromFlash(u16 *);
static          void        GetDefaultRunPurifyAmpIntegralValue(u16 *);
static          void        StoreRunPurifyAmpIntegralValue(u16 *);

static          void        GetDefaultRunningStatusDelayAdjustSpdPara(RUNNING_STATUS_DELAY_ADJUST_SPEED_PARAMETERS_Typedef* i_DelayAndAdjustSpd);
static          void        StoreRunningStatusDelayAdjustSpdPara(RUNNING_STATUS_DELAY_ADJUST_SPEED_PARAMETERS_Typedef* i_DelayAndAdjustSpdPara);
static          void        GetRunningStageDelayAndAdjustSpdFromFlash(RUNNING_STATUS_DELAY_ADJUST_SPEED_PARAMETERS_Typedef * i_DelayAndAdjustSpd);

static          void        GetGlobalNetWorkIDFromFlash(u16* i_GlobalNetWorkID);
static          void        GetDefaultFirstTimeHeatHoldSeconds(u16 *i_FirstTimeHeatHoldSeconds);
static          void        GetFirstTimeHeatHoldSecondsFromFlash(u16* i_FirstTimeHeatHoldSeconds);

#if 1
/*
***************************************************************************************************
*                               LoadDriverLayerParameters()
*
* Description:  load the driver Layer sensors and others devices prameters.
*
* Arguments  :  none.
*
* Returns    :  none.
*
* Note(s)    :  none.
***************************************************************************************************
*/
void LoadDriverLayerParameters()
{
    uint16_t u16ParametersSelectFlag;
    ErrorStatus eCalibratedFlag;
    ANALOG_SIGNAL_SERSOR_PARAMETERS_Typedef stAnaSensorParameters[BSP_ANA_SENSORS_NMB];
    uint8_t u8DataLength = BSP_ANA_SENSORS_NMB;

    GetParametersSelectFlag(&u16ParametersSelectFlag);

    /* ʹ���Ѿ����úõĲ��� */
    if(DRIVER_LAYER_PARAMETERS_SELECT_CFG_VALUE == (DRIVER_LAYER_PARAMETERS_SELECT_FLAG_CHECK & u16ParametersSelectFlag)) {
        BSP_GetStoredAnaSensorsParaFromFlash(stAnaSensorParameters, u8DataLength);//��flash�ж�ȡ����
//      APP_TRACE_INFO(("%f , %f...\r\n", stAnaSensorParameters[0].AnaToDigRatio, stAnaSensorParameters[0].BaseDigValue));
        BSP_LoadAnaSensorParaToPrgm(stAnaSensorParameters, u8DataLength);//��������������
    } else { //�״�У׼����У׼ʧ��ʱ����У׼
        eCalibratedFlag = BSP_GetCalibratedAnaSensorPara(stAnaSensorParameters, u8DataLength);//��У׼������У���Ĳ���

//      APP_TRACE_INFO(("%f , %f...\r\n", stAnaSensorParameters[0].AnaToDigRatio, stAnaSensorParameters[0].BaseDigValue));
        if(eCalibratedFlag == SUCCESS) {
            /*У׼�ɹ����´�������������ֵ����*/
            u16ParametersSelectFlag &= ~DRIVER_LAYER_PARAMETERS_SELECT_DEFAULT_VALUE;//���ѡ��Ĭ�ϲ�����־λ
            SetParametersSelectFlag(&u16ParametersSelectFlag);
            BSP_StoreAnaSensorParaToFlash(stAnaSensorParameters, u8DataLength);//����õĲ������浽flash
        } else {
            //У׼ʧ�ܣ�����ʹ��Ĭ��ֵ
            //��������������ѡ���־�����޸ģ��´�������Ȼ��ҪУ׼
        }
    }
}

/*
***************************************************************************************************
*                                      LoadApplicationLayerParameters()
*
* Description:  load the application layer paramters .
*
* Arguments  :  none.
*
* Returns    :  none.
*
* Note(s)    :  none.
***************************************************************************************************
*/
void LoadApplicationLayerParameters()
{
    uint16_t u16WorkTimes;
    uint16_t u16ParametersSelectFlag;
    SYSTEM_TIME_Typedef     stTotalWorkTime;

    GetParametersSelectFlag(&u16ParametersSelectFlag);

    /*ʹ�����úõ�Ӧ�ò���*/
    if(APP_PARAMETERS_SELECT_CFG_VALUE == (APP_PARAMETERS_SELECT_FLAG_CHECK & u16ParametersSelectFlag)) {
        APP_TRACE_INFO(("The machine will work with the configured parameters!...\r\n"));
        GetSystemWorkTimesFromFlash(&u16WorkTimes);

        if(u16WorkTimes != 0xFFFF) {
            LoadSystemWorkTimesToPrgm(u16WorkTimes);//ʹ�ü�¼�Ĳ���
        } else {
            LoadSystemWorkTimesToPrgm(0);//��ʷ���д����Զ����㣬���ݲ����棬���������������һ��
        }

        APP_TRACE_INFO(("The %d time for the system to boot,loading the parameters..\r\n", u16WorkTimes + 1));

        GetTotalWorkTimeFromFlash(&stTotalWorkTime);
        LoadTotalWorkTimeToPrgm(stTotalWorkTime);   //����ʵʱ���в��������ļ���Ӧ�ĳ�����ȥ
        
        GetGlobalNetWorkIDFromFlash(&g_u16GlobalNetWorkId);
        APP_TRACE_INFO(("Load GLOBAL_NET_WORK_ID:%d...\n\r",g_u16GlobalNetWorkId));
        GetReformerTempCmpTblFromFlash(&g_stReformerTempCmpTbl);
        GetLqdPressCmpTblFromFlash(&g_stLqdPressCmpTbl);
        GetLqdHeightCmpTblFromFlash(&g_stLqdHeightCmpTbl);
        GetStartHydrgPumpSpdParaFromFlash(&g_stStartHydrgPumpSpdPara);               
        GetStartHydrgFanSpdParaFromFlash(&g_stStartHydrgFanSpdPara);
        GetRunPurifyAmpIntegralValueFromFlash(&g_u16RunPurifyAmpIntegralValue);
        GetRunningStageDelayAndAdjustSpdFromFlash(&g_stRunningStatusDelayAdjustSpdPara);
        GetFirstTimeHeatHoldSecondsFromFlash(&g_u8FirstTimeHeatHoldSeconds);
        
    } else { //����Ĭ�ϲ���
        APP_TRACE_INFO(("First time run,the machine will work with the default parameters!...\r\n"));

        u16ParametersSelectFlag &= ~APP_PARAMETERS_SELECT_DEFAULT_VALUE;//�´������������úõĲ���
        SetParametersSelectFlag(&u16ParametersSelectFlag);//�������

        u16WorkTimes = 0;//ϵͳ���д���Ϊ0
        LoadSystemWorkTimesToPrgm(u16WorkTimes);
        StoreSystemWorkTimes(u16WorkTimes);

        GetTotalWorkTimeFromFlash(&stTotalWorkTime);

        if((stTotalWorkTime.hour == 0xFFFF) && (stTotalWorkTime.minute == 0xFF) && (stTotalWorkTime.second == 0xFF)) {
            stTotalWorkTime.hour = 0;
            stTotalWorkTime.minute = 0;
            stTotalWorkTime.second = 0;
            StoreTotalWorkTime(&stTotalWorkTime);//�ۼ�����ʱ����㲢����
        }

        LoadTotalWorkTimeToPrgm(stTotalWorkTime);

        APP_TRACE_INFO(("This is the first time for the system to boot,capturing the parameters...\r\n"));

        /*��ȡĬ�ϲ������Ҵ浽Flash��*/
        StoreGlobalNetWorkID(&g_u16GlobalNetWorkId);
        APP_TRACE_INFO(("Default GLOBAL_NET_WORK_ID:%d...\n\r",g_u16GlobalNetWorkId));
        
        GetDefaultReformerTempCmpTbl(&g_stReformerTempCmpTbl);
        StoreReformerTempCmpTbl(&g_stReformerTempCmpTbl);
        
        GetDefaultLqdPressCmpTbl(&g_stLqdPressCmpTbl);
        StoreLqdPressCmpTbl(&g_stLqdPressCmpTbl);

        GetDefaultLqdHeightCmpTbl(&g_stLqdHeightCmpTbl);
        StoreLqdHeightCmpTbl(&g_stLqdHeightCmpTbl);

        GetDefaultStartHydrgPumpSpdPara(&g_stStartHydrgPumpSpdPara);
        StoreStartHydrgPumpSpdPara(&g_stStartHydrgPumpSpdPara);

        GetDefaultStartHydrgFanSpdPara(&g_stStartHydrgFanSpdPara);
        StoreStartHydrgFanSpdPara(&g_stStartHydrgFanSpdPara);

        GetDefaultRunPurifyAmpIntegralValue(&g_u16RunPurifyAmpIntegralValue);
        StoreRunPurifyAmpIntegralValue(&g_u16RunPurifyAmpIntegralValue);
        
        GetDefaultFirstTimeHeatHoldSeconds(&g_u8FirstTimeHeatHoldSeconds);
        StoreFirstTimeHeatHoldSeconds(&g_u8FirstTimeHeatHoldSeconds);
        
        GetDefaultRunningStatusDelayAdjustSpdPara(&g_stRunningStatusDelayAdjustSpdPara);
        StoreRunningStatusDelayAdjustSpdPara(&g_stRunningStatusDelayAdjustSpdPara);
    }
}

/*
***************************************************************************************************
*                           GetSystemWorkTimesFromFlash()
*
* Description:  Get the system work times that has stored in the flash
*
* Arguments  :  the avriable that store the work times.
*
* Returns    :  none.
*
* Note(s)    :  none.
***************************************************************************************************
*/
static void GetParametersSelectFlag(uint16_t *o_pParametersSelectFlagAddr)
{
    STMFLASH_Read(SYSTEM_PARAMETER_STORE_SEGMENT_ADDR + PARAMETERS_SELECT_FLAG_OFFSET_ADDR, o_pParametersSelectFlagAddr, 1);//��ȡ���в���ѡ���־
}

static void SetParametersSelectFlag(uint16_t *i_pParametersSelectFlagAddr)
{
    STMFLASH_Write(SYSTEM_PARAMETER_STORE_SEGMENT_ADDR + PARAMETERS_SELECT_FLAG_OFFSET_ADDR, i_pParametersSelectFlagAddr, 1);
}

/*
*********************************************************************************************************
*                                      StoreGlobalNetWorkID()
*
* Description:  save the paramters that related to the network ID.
*
* Arguments  :  the buff that store the parameters.
*
* Returns    :  none.
*
* Note(s)	 :	none.
*********************************************************************************************************
*/
static void GetGlobalNetWorkIDFromFlash(u16* i_GlobalNetWorkID)
{
	STMFLASH_Read(SYSTEM_PARAMETER_STORE_SEGMENT_ADDR + GLOBAL_NET_WORK_ID_ADDR, i_GlobalNetWorkID, 1);
}

void StoreGlobalNetWorkID(u16* i_GlobalNetWorkID)
{	
	STMFLASH_Write(SYSTEM_PARAMETER_STORE_SEGMENT_ADDR + GLOBAL_NET_WORK_ID_ADDR, i_GlobalNetWorkID, 1);
}
/*
***************************************************************************************************
*                           StoreSystemWorkTimes()
*
* Description:  Get the system work times that has stored in the flash
*
* Arguments  :  the avriable that store the work times.
*
* Returns    :  none.
*
* Note(s)    :  none.
***************************************************************************************************
*/
void StoreSystemWorkTimes(uint16_t i_u16WorkTimes)
{
    STMFLASH_Write(SYSTEM_PARAMETER_STORE_SEGMENT_ADDR + SYSTEM_RUN_TIME_STORE_OFFSET_ADDR, &i_u16WorkTimes, 1);
}

static void GetSystemWorkTimesFromFlash(u16 *o_StoreAddr)
{
    STMFLASH_Read(SYSTEM_PARAMETER_STORE_SEGMENT_ADDR + SYSTEM_RUN_TIME_STORE_OFFSET_ADDR, o_StoreAddr, 1);//��ȡ���д���
}


/*
***************************************************************************************************
*                                   BSP_GetStoredAnaSensorsParaFromFlash()
*
* Description:  Get the Analog sensors prameters from Flash.
*
* Arguments  :  none.
*
* Returns    :  none.
*
* Note(s)    :  none.
***************************************************************************************************
*/
void BSP_GetStoredAnaSensorsParaFromFlash(ANALOG_SIGNAL_SERSOR_PARAMETERS_Typedef *o_stAnaSensorParameters, uint8_t i_u8Length)
{
    STMFLASH_Read(SYSTEM_PARAMETER_STORE_SEGMENT_ADDR + DRIVER_LAYER_CALIBRATED_LINEAR_ANA_SENSOR_PARAMETERS_OFFSET_ADDR,
                  (uint16_t *)o_stAnaSensorParameters, i_u8Length * 4); //�ṹ��ĳ�����u16�������ݳ��ȵ�4��
}

void BSP_StoreAnaSensorParaToFlash(ANALOG_SIGNAL_SERSOR_PARAMETERS_Typedef *i_stAnaSensorParameters, uint8_t i_u8Length)
{
    STMFLASH_Write(SYSTEM_PARAMETER_STORE_SEGMENT_ADDR + DRIVER_LAYER_CALIBRATED_LINEAR_ANA_SENSOR_PARAMETERS_OFFSET_ADDR,
                   (uint16_t *)i_stAnaSensorParameters, i_u8Length * 4); //�ṹ��ĳ�����u16�������ݳ��ȵ�4��
}

/*
***************************************************************************************************
*                                      GetTotalWorkTimeFromFlash()
*
* Description:  load the paramters, that related to the reformer, that has stored in the flash.
*
* Arguments  :  the avriable that store the parameters.
*
* Returns    :  none.
*
* Note(s)    :  none.
***************************************************************************************************
*/
static void GetTotalWorkTimeFromFlash(SYSTEM_TIME_Typedef *o_StoreAddr)
{
    STMFLASH_Read(SYSTEM_PARAMETER_STORE_SEGMENT_ADDR + SYSTEM_TOTAL_WORK_TIME_STORE_OFFSET_ADDR, (uint16_t *)o_StoreAddr, 16);
}

void StoreTotalWorkTime(SYSTEM_TIME_Typedef *i_stTotalTime)
{
    STMFLASH_Write(SYSTEM_PARAMETER_STORE_SEGMENT_ADDR + SYSTEM_TOTAL_WORK_TIME_STORE_OFFSET_ADDR, (uint16_t *)i_stTotalTime, 16);
}

/*
***************************************************************************************************
*                                      GetReformerTempCmpTblFromFlash()
*
* Description:  load the paramters, that related to the reformer, that has stored in the flash.
*
* Arguments  :  the avriable that store the parameters.
*
* Returns    :  none.
*
* Note(s)    :  none.
***************************************************************************************************
*/
static void GetReformerTempCmpTblFromFlash(REFORMER_TEMP_CMP_LINES_Typedef *i_ReformerTemCmpTbl)
{    
    STMFLASH_Read(SYSTEM_PARAMETER_STORE_SEGMENT_ADDR + RUN_REFORMER_CMP_TBL_STORE_OFFSET_ADDR, (uint16_t*)i_ReformerTemCmpTbl, 12);
}

static void GetDefaultReformerTempCmpTbl(REFORMER_TEMP_CMP_LINES_Typedef *i_ReformerTemCmpTbl)
{
    i_ReformerTemCmpTbl->IgFstTimeFrtToBhdTmpPnt    = NULL;
    i_ReformerTemCmpTbl->IgFstTimeOverTmpPnt        = 230;
    i_ReformerTemCmpTbl->IgFstTimeWatiTimeMax1      = NULL;
    i_ReformerTemCmpTbl->IgFstTimeWatiTimeMax2      = NULL;
    i_ReformerTemCmpTbl->IgScdTimeFrtToBhdTmpPnt    = NULL;
    i_ReformerTemCmpTbl->IgScdTimeOverTmpPnt        = NULL;
    i_ReformerTemCmpTbl->IgScdTimeWatiTimeMax1      = NULL;
    i_ReformerTemCmpTbl->IgScdTimeWatiTimeMax2      = NULL;

    i_ReformerTemCmpTbl->AlarmLowerLimit            = 0;
    i_ReformerTemCmpTbl->AlarmUpperLimit            = 550;
    i_ReformerTemCmpTbl->ShutDownLowerLimit         = 0;
    i_ReformerTemCmpTbl->ShutDownUpperLimit         = 100;
}

static void StoreReformerTempCmpTbl(REFORMER_TEMP_CMP_LINES_Typedef *i_ReformerTemCmpTbl)
{
    STMFLASH_Write(SYSTEM_PARAMETER_STORE_SEGMENT_ADDR + RUN_REFORMER_CMP_TBL_STORE_OFFSET_ADDR, (uint16_t*)i_ReformerTemCmpTbl, 12);
}

/*
***************************************************************************************************
*                                      GetLqdPressCmpTblFromFlash()
*
* Description:  load the paramters, that related to the liquid press, that has stored in the flash.
*
* Arguments  :  the avriable that store the parameters.
*
* Returns    :  none.
*
* Note(s)    :  none.
***************************************************************************************************
*/
static void GetLqdPressCmpTblFromFlash(LIQUID_PRESSURE_CMP_LINES_Typedef *i_LqdPressCmpTbl)
{
    STMFLASH_Read(SYSTEM_PARAMETER_STORE_SEGMENT_ADDR + RUN_LIQUIDPRESS_CMP_TBL_STORE_OFFSET_ADDR, (uint16_t*)i_LqdPressCmpTbl, 4);
}

static void GetDefaultLqdPressCmpTbl(LIQUID_PRESSURE_CMP_LINES_Typedef *i_LqdPressCmpTbl)
{
    i_LqdPressCmpTbl->AlarmLowerLimit               = 1;
    i_LqdPressCmpTbl->AlarmUpperLimit               = 18;
    i_LqdPressCmpTbl->ShutDownLowerLimit            = 0;
    i_LqdPressCmpTbl->ShutDownUpperLimit            = 20;
}

static void StoreLqdPressCmpTbl(LIQUID_PRESSURE_CMP_LINES_Typedef *i_LqdPressCmpTbl)
{
    STMFLASH_Write(SYSTEM_PARAMETER_STORE_SEGMENT_ADDR + RUN_LIQUIDPRESS_CMP_TBL_STORE_OFFSET_ADDR, (uint16_t*)i_LqdPressCmpTbl, 4);
}
/*
***************************************************************************************************
*                                      GetLqdHeightCmpTblFromFlash()
*
* Description:  load the paramters, that related to the liquid press, that has stored in the flash.
*
* Arguments  :  the avriable that store the parameters.
*
* Returns    :  none.
*
* Note(s)    :  none.
***************************************************************************************************
*/
static void GetLqdHeightCmpTblFromFlash(LIQUID_HEIGHT_CMP_LINES_Typedef *i_LqdHeightCmpTbl)
{
    STMFLASH_Read(SYSTEM_PARAMETER_STORE_SEGMENT_ADDR + RUN_LIQUID_HEIGHT_CMP_TBL_STORE_OFFSET_ADDR, (uint16_t*)i_LqdHeightCmpTbl, 3);
}

static void GetDefaultLqdHeightCmpTbl(LIQUID_HEIGHT_CMP_LINES_Typedef *i_LqdHeightCmpTbl)
{
    i_LqdHeightCmpTbl->AlarmlowerLiquidLevellimit = 70;
    i_LqdHeightCmpTbl->OpenAutomaticliquidValue = 100;
    i_LqdHeightCmpTbl->CloseAutomaticliquidValue = 200;
}

static void StoreLqdHeightCmpTbl(LIQUID_HEIGHT_CMP_LINES_Typedef *i_LqdHeightCmpTbl)
{
    STMFLASH_Write(SYSTEM_PARAMETER_STORE_SEGMENT_ADDR + RUN_LIQUID_HEIGHT_CMP_TBL_STORE_OFFSET_ADDR, (uint16_t*)i_LqdHeightCmpTbl, 3);
}

/*
***************************************************************************************************
*                                      GetStartHydrgPumpSpdParaFromFlash()
*
* Description:  load the paramters, that related to the speed adjust device, that has stored in the flash.
*
* Arguments  :  the avriable that store the parameters.
*
* Returns    :  none.
*
* Note(s)    :  none.
***************************************************************************************************
*/
void GetStartHydrgPumpSpdParaFromFlash(HYDROGEN_PUMP_SPEED_PARA_Typedef *i_StartPumpSpdPara)
{
    STMFLASH_Read(SYSTEM_PARAMETER_STORE_SEGMENT_ADDR + RUN_HYDROGEN_PUMP_SPEED_PARA_STORE_OFFSET_ADDR, (uint16_t*)i_StartPumpSpdPara, 3);
}

static void GetDefaultStartHydrgPumpSpdPara(HYDROGEN_PUMP_SPEED_PARA_Typedef *i_StartPumpSpdPara)
{
    i_StartPumpSpdPara->PumpSpdIgniterFirstTime   = 190;
    i_StartPumpSpdPara->PumpSpdIgniterSecondTime  = 360;
    i_StartPumpSpdPara->PumpSpdAfterLiquidPressExceed4Kg  = 320;//�ȶ����еı���
}

void StoreStartHydrgPumpSpdPara(HYDROGEN_PUMP_SPEED_PARA_Typedef *i_StartPumpSpdPara)
{
    STMFLASH_Write(SYSTEM_PARAMETER_STORE_SEGMENT_ADDR + RUN_HYDROGEN_PUMP_SPEED_PARA_STORE_OFFSET_ADDR, (uint16_t*)i_StartPumpSpdPara, 3);
}

//����һ��ˮ�õĲ�������
void StoreStartHydrgPumpSpdParaBySingle(uint16_t *i_StartPumpSpdPara,u8 DataAddr)
{	
	STMFLASH_Write(SYSTEM_PARAMETER_STORE_SEGMENT_ADDR + RUN_HYDROGEN_PUMP_SPEED_PARA_STORE_OFFSET_ADDR + 2*DataAddr,i_StartPumpSpdPara, 1);
}
/*
***************************************************************************************************
*                                      GetStartHydrgFanSpdParaFromFlash()
*
* Description:  load the default paramters that related to the speed adjust device.
*
* Arguments  :  the avriable that store the parameters.
*
* Returns    :  none.
*
* Note(s)    :  none.
***************************************************************************************************
*/
void GetStartHydrgFanSpdParaFromFlash(HYDROGEN_FAN_SPEED_PARA_Typedef *i_StartHydrgFanSpdPara)
{
    STMFLASH_Read(SYSTEM_PARAMETER_STORE_SEGMENT_ADDR + RUN_HYDROGEN_FAN_SPEED_PARA_STORE_OFFSET_ADDR, (uint16_t*)i_StartHydrgFanSpdPara, 4);
}

static void GetDefaultStartHydrgFanSpdPara(HYDROGEN_FAN_SPEED_PARA_Typedef *i_StartHydrgFanSpdPara)
{
    i_StartHydrgFanSpdPara->FanSpdIgniterFirstTime   = 1000;
    i_StartHydrgFanSpdPara->FanSpdAfterIgniterFirstSuccessd = 2000;
    i_StartHydrgFanSpdPara->FanSpdIgniterSecondTime   = 1300;
    i_StartHydrgFanSpdPara->FanSpdAfterIgniterSecondSuccessd = 2000;
}

void StoreStartHydrgFanSpdPara(HYDROGEN_FAN_SPEED_PARA_Typedef *i_StartHydrgFanSpdPara)
{
    STMFLASH_Write(SYSTEM_PARAMETER_STORE_SEGMENT_ADDR + RUN_HYDROGEN_FAN_SPEED_PARA_STORE_OFFSET_ADDR, (uint16_t*)i_StartHydrgFanSpdPara, 4);
}

//����һ������Ĳ�������
void StoreStartHydrgFanSpdParaBySingle(uint16_t *i_StartHydrgFanSpdPara,u8 DataAddr)
{	
	STMFLASH_Write(SYSTEM_PARAMETER_STORE_SEGMENT_ADDR + RUN_HYDROGEN_FAN_SPEED_PARA_STORE_OFFSET_ADDR + 2*DataAddr,i_StartHydrgFanSpdPara, 1);
}

/*
*********************************************************************************************************
*                                      GetRunningStageDelayAndAdjustSpdFromFlash()
*
* Description:  load the paramters, that related to the speed adjust device, that has stored in the flash.
*
* Arguments  :  the avriable that store the parameters.
*
* Returns    :  none.
*
* Note(s)	 :	none.
*********************************************************************************************************
*/
static void GetRunningStageDelayAndAdjustSpdFromFlash(RUNNING_STATUS_DELAY_ADJUST_SPEED_PARAMETERS_Typedef * i_DelayAndAdjustSpd)
{
	STMFLASH_Read(SYSTEM_PARAMETER_STORE_SEGMENT_ADDR + RUNNING_STATUS_DELAY_ADJUST_SPEED_PARAMETERS_ADDR, (uint16_t*)i_DelayAndAdjustSpd, 6);
}

static void GetDefaultRunningStatusDelayAdjustSpdPara(RUNNING_STATUS_DELAY_ADJUST_SPEED_PARAMETERS_Typedef* i_DelayAndAdjustSpd)
{
	i_DelayAndAdjustSpd->FirstDelayTimeByMin       = 2;
	i_DelayAndAdjustSpd->FirstTimeAdjustPumpFlag   =  1;
	i_DelayAndAdjustSpd->FirstTimeAdjustPumpValue  = 20;
	i_DelayAndAdjustSpd->FirstTimeAdjustFanFlag    =  1;
	i_DelayAndAdjustSpd->FirstTimeAdjustFanValue   = 20;
    
	i_DelayAndAdjustSpd->SecondDelayTimeByMin      = 2;
	i_DelayAndAdjustSpd->SecondTimeAdjustPumpFlag  =  1;
	i_DelayAndAdjustSpd->SecondTimeAdjustPumpValue = 20;
	i_DelayAndAdjustSpd->SecondTimeAdjustFanFlag   =  1;
	i_DelayAndAdjustSpd->SecondTimeAdjustFanValue  = 20;
}

static void StoreRunningStatusDelayAdjustSpdPara(RUNNING_STATUS_DELAY_ADJUST_SPEED_PARAMETERS_Typedef* i_DelayAndAdjustSpdPara)
{	
	STMFLASH_Write(SYSTEM_PARAMETER_STORE_SEGMENT_ADDR + RUNNING_STATUS_DELAY_ADJUST_SPEED_PARAMETERS_ADDR, (u16 *)i_DelayAndAdjustSpdPara, 6);
}

//��ʱ���ٵ��������ֽڵ����ݱ���
void StoreRunningStatusDelayAdjustSpdParaBySingle(u16* i_DelayAndAdjustSpdPara,u8 DataAddr)
{	
	STMFLASH_Write(SYSTEM_PARAMETER_STORE_SEGMENT_ADDR + RUNNING_STATUS_DELAY_ADJUST_SPEED_PARAMETERS_ADDR + 2 * DataAddr, i_DelayAndAdjustSpdPara, 1);
}

/*
*********************************************************************************************************
*                                      GetFirstTimeHeatHoldSecondsFromFlash()
*
* Description:  load the paramters, that related to the fire heat time by the first ignite, that has stored in the flash.
*
* Arguments  :  the avriable that store the parameters.
*
* Returns    :  none.
*
* Note(s)	 :	none.
*********************************************************************************************************
*/
void GetFirstTimeHeatHoldSecondsFromFlash(u16* i_FirstTimeHeatHoldSeconds)
{
	STMFLASH_Read(SYSTEM_PARAMETER_STORE_SEGMENT_ADDR + FIRST_TIME_HEAT_HOLD_SECONDS_PARA_ADDR, i_FirstTimeHeatHoldSeconds, 1);
}

void StoreFirstTimeHeatHoldSeconds(u16* i_FirstTimeHeatHoldSeconds)
{	
	STMFLASH_Write(SYSTEM_PARAMETER_STORE_SEGMENT_ADDR + FIRST_TIME_HEAT_HOLD_SECONDS_PARA_ADDR, i_FirstTimeHeatHoldSeconds, 1);
}

static void GetDefaultFirstTimeHeatHoldSeconds(u16 *i_FirstTimeHeatHoldSeconds)
{
    *i_FirstTimeHeatHoldSeconds = 180;
}
/*
***************************************************************************************************
*                                GetRunPurifyAmpIntegralValueFromFlash()
*
* Description:  load the paramters that related to the hydrogen in and out cmp level.
*
* Arguments  :  the avriable that store the parameters.
*
* Returns    :  none.
*
* Note(s)    :  none.
***************************************************************************************************
*/
static void GetRunPurifyAmpIntegralValueFromFlash(u16 *i_RunPurifyAmpIntegralValue)
{
    STMFLASH_Read(SYSTEM_PARAMETER_STORE_SEGMENT_ADDR + RUN_PURIFY_AMP_INTEGRAL_VALUE, (uint16_t*)i_RunPurifyAmpIntegralValue, 1);
}

static void GetDefaultRunPurifyAmpIntegralValue(u16 *i_RunPurifyAmpIntegralValue)
{
    *i_RunPurifyAmpIntegralValue = 800;
}

static void StoreRunPurifyAmpIntegralValue(u16 *i_RunPurifyAmpIntegralValue)
{
    STMFLASH_Write(SYSTEM_PARAMETER_STORE_SEGMENT_ADDR + RUN_PURIFY_AMP_INTEGRAL_VALUE, (uint16_t*)i_RunPurifyAmpIntegralValue, 1);
}
/*
***************************************************************************************************
*                                      STMFLASH_ReadHalfWord()
*
* Description:  ��ȡָ����ַ�İ���(16λ����)
*
* Arguments  :  ����ַ(�˵�ַ����Ϊ2�ı���!!)
*
* Returns    :  ��Ӧ����.
*
* Note(s)    :  none.
***************************************************************************************************
*/
u16 STMFLASH_ReadHalfWord(u32 faddr)
{
    return *(vu16 *)faddr;
}

#if STM32_FLASH_WREN    //���ʹ����д����    
/*
***************************************************************************************************
*                                      STMFLASH_Write_NoCheck()
*
* Description:  ������д�����(16λ����)��ָ����ַ
*
* Arguments  :  д��ʼ��ַ(�˵�ַ����Ϊ2�ı���!!)
*               ��д����
*
* Returns    :  none.
*
* Note(s)    :  none.
***************************************************************************************************
*/
static void STMFLASH_Write_NoCheck(u32 WriteAddr, u16 *pBuffer, u16 NumToWrite)
{
    u16 i;

    for(i = 0; i < NumToWrite; i++) {
        FLASH_ProgramHalfWord(WriteAddr, pBuffer[i]);
        WriteAddr += 2;//��ַ����2.
    }
}

/*
***************************************************************************************************
*                                      STMFLASH_Write()
*
* Description:  ��ָ����ַ��ʼд��ָ�����ȵ�����
*
* Arguments  :  WriteAddr   -  д��ʼ��ַ(�˵�ַ����Ϊ2�ı���!!)
*               pBuffer     -  ��д����ָ��
*               NumToWrite  -  ��д����ָ��İ��֣�16λ����
*
* Returns    :  none.
*
* Note(s)    :  none.
***************************************************************************************************
*/
#if     STM32_FLASH_SIZE < 256
    #define STM_SECTOR_SIZE 1024    //�ֽ�
#else
    #define STM_SECTOR_SIZE 2048
#endif
static      u16     STMFLASH_BUF[PARAMETERS_STORE_AREA_SIZE];
static void STMFLASH_Write(u32 WriteAddr, u16 *pBuffer, u16 NumToWrite)
{
    u32 secpos;    //������ַ
    u16 secoff;    //������ƫ�Ƶ�ַ(16λ�ּ���)
    u16 secremain; //������ʣ���ַ(16λ�ּ���)
    u16 i;
    u32 offaddr;   //ȥ��0X08000000��ĵ�ַ

    if((WriteAddr >= STM32_FLASH_BASE) && (WriteAddr < (STM32_FLASH_BASE + 1024 * STM32_FLASH_SIZE))) {
        FLASH_Unlock();                                 //����
        offaddr = WriteAddr - STM32_FLASH_BASE;         //ʵ��ƫ�Ƶ�ַ.
        secpos = offaddr / STM_SECTOR_SIZE;             //������ַ  0~127 ��128����������STM32F105RB�У�ÿ������1K
        secoff = (offaddr % STM_SECTOR_SIZE) / 2;       //�������ڵ�ƫ��(2���ֽ�Ϊ������λ.)
        secremain = STM_SECTOR_SIZE / 2 - secoff;       //����ʣ��ռ��С

        if(NumToWrite <= secremain) {
            secremain = NumToWrite;//�����ڸ�������Χ   //���������������һ����������д
        }

        while(1) {
            STMFLASH_Read(secpos * STM_SECTOR_SIZE + STM32_FLASH_BASE, STMFLASH_BUF, STM_SECTOR_SIZE / 2);//������������������

            for(i = 0; i < secremain; i++) { //У������
                if(STMFLASH_BUF[secoff + i] != 0XFFFF) {
                    break;//��Ҫ����
                }
            }

            if(i < secremain) { //��Ҫ����
                FLASH_ErasePage(secpos * STM_SECTOR_SIZE + STM32_FLASH_BASE);//�����������

                for(i = 0; i < secremain; i++) { //����
                    STMFLASH_BUF[i + secoff] = pBuffer[i];
                }

                STMFLASH_Write_NoCheck(secpos * STM_SECTOR_SIZE + STM32_FLASH_BASE, STMFLASH_BUF, STM_SECTOR_SIZE / 2);//д����������
            } else {
                STMFLASH_Write_NoCheck(WriteAddr, pBuffer, secremain);//д�Ѿ������˵�,ֱ��д������ʣ������.
            }

            if(NumToWrite == secremain) {
                break;//д�������
            } else { //д��δ����
                secpos ++;              //������ַ��1
                secoff = 0;             //ƫ��λ��Ϊ0
                pBuffer += secremain;   //ָ��ƫ��
                WriteAddr += secremain; //д��ַƫ��
                NumToWrite -= secremain;    //�ֽ�(16λ)���ݼ�

                if(NumToWrite > (STM_SECTOR_SIZE / 2)) {
                    secremain = STM_SECTOR_SIZE / 2;//��һ����������д����
                } else {
                    secremain = NumToWrite;//��һ����������д����
                }
            }
        }

        FLASH_Lock();//����
    }
}
#endif

/*
***************************************************************************************************
*                                      STMFLASH_Write()
*
* Description:  ��ָ����ַ��ʼ����ָ�����ȵ�����
*
* Arguments  :  ReadAddr   -  ����ʼ��ַ(�˵�ַ����Ϊ2�ı���!!)
*               pBuffer     -  �н�����ָ��
*               NumToWrite  -  ��������ָ��İ��֣�16λ����
*
* Returns    :  none.
*
* Note(s)    :  none.
***************************************************************************************************
*/
void STMFLASH_Read(u32 ReadAddr, u16 *pBuffer, u16 NumToRead)
{
    u16 i;

    for(i = 0; i < NumToRead; i++) {
        pBuffer[i] = STMFLASH_ReadHalfWord(ReadAddr);//��ȡ2���ֽ�.
        ReadAddr += 2;//ƫ��2���ֽ�.
    }
}

#endif
/******************* (C) COPYRIGHT 2015 Guangdong Hydrogen *****END OF FILE****/

