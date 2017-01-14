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
#define SYSTEM_TOTAL_WORK_TIME_STORE_OFFSET_ADDR                        4   //ϵͳʱ��ʱ��4������4���ֽ�
#define RUN_REFORMER_CMP_TBL_STORE_OFFSET_ADDR                          20  //12����
#define RUN_LIQUIDPRESS_CMP_TBL_STORE_OFFSET_ADDR                       40  // Һѹ4����
#define RUN_LIQUID_HEIGHT_CMP_TBL_STORE_OFFSET_ADDR                     50  //Һλ����4��
#define RUN_HYDROGEN_PUMP_SPEED_PARA_STORE_OFFSET_ADDR                  60  //�ò���2��
#define RUN_HYDROGEN_FAN_SPEED_PARA_STORE_OFFSET_ADDR                   70  //����������4��
#define RUN_PURIFY_AMP_INTEGRAL_VALUE                                   80  //����ᴿĤ������1��
#define DRIVER_LAYER_CALIBRATED_LINEAR_ANA_SENSOR_PARAMETERS_OFFSET_ADDR  82  //���������Դ�����У׼����,28����


/*
***************************************************************************************************
*                                     GLOBAL  VARIABLES
***************************************************************************************************
*/
REFORMER_TEMP_CMP_LINES_Typedef             g_stReformerTempCmpTbl;
LIQUID_PRESSURE_CMP_LINES_Typedef           g_stLqdPressCmpTbl;
LIQUID_HEIGHT_CMP_LINES_Typedef             g_stLqdHeightCmpTbl;
HYDROGEN_PUMP_SPEED_PARA_Typedef            g_stStartHydrgPumpSpdPara;
HYDROGEN_FAN_SPEED_PARA_Typedef             g_stStartHydrgFanSpdPara;

uint16_t                                    g_u16RunPurifyAmpIntegralValue;
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
        
        GetReformerTempCmpTblFromFlash(&g_stReformerTempCmpTbl);
        GetLqdPressCmpTblFromFlash(&g_stLqdPressCmpTbl);
        GetLqdHeightCmpTblFromFlash(&g_stLqdHeightCmpTbl);
        GetStartHydrgPumpSpdParaFromFlash(&g_stStartHydrgPumpSpdPara);
        GetStartHydrgFanSpdParaFromFlash(&g_stStartHydrgFanSpdPara);
        GetRunPurifyAmpIntegralValueFromFlash(&g_u16RunPurifyAmpIntegralValue);
        
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
    STMFLASH_Write(SYSTEM_PARAMETER_STORE_SEGMENT_ADDR + SYSTEM_RUN_TIME_STORE_OFFSET_ADDR, &i_u16WorkTimes, 2);
}

static void GetSystemWorkTimesFromFlash(u16 *o_StoreAddr)
{
    STMFLASH_Read(SYSTEM_PARAMETER_STORE_SEGMENT_ADDR + SYSTEM_RUN_TIME_STORE_OFFSET_ADDR, o_StoreAddr, 2);//��ȡ���д���
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
    STMFLASH_Read(SYSTEM_PARAMETER_STORE_SEGMENT_ADDR + RUN_LIQUID_HEIGHT_CMP_TBL_STORE_OFFSET_ADDR, (uint16_t*)i_LqdHeightCmpTbl, 4);
}

static void GetDefaultLqdHeightCmpTbl(LIQUID_HEIGHT_CMP_LINES_Typedef *i_LqdHeightCmpTbl)
{
    i_LqdHeightCmpTbl->AlarmlowerLiquidLevellimit = 70;
    i_LqdHeightCmpTbl->OpenAutomaticliquidValue = 100;
    i_LqdHeightCmpTbl->CloseAutomaticliquidValue = 200;
    i_LqdHeightCmpTbl->AlarmUpperLiquidLevellimit = 230;//�ݶ�
}

static void StoreLqdHeightCmpTbl(LIQUID_HEIGHT_CMP_LINES_Typedef *i_LqdHeightCmpTbl)
{
    STMFLASH_Write(SYSTEM_PARAMETER_STORE_SEGMENT_ADDR + RUN_LIQUID_HEIGHT_CMP_TBL_STORE_OFFSET_ADDR, (uint16_t*)i_LqdHeightCmpTbl, 4);
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
    i_StartPumpSpdPara->PumpSpdIgniterSecondTime  = 250;
    i_StartPumpSpdPara->PumpSpdAfterLiquidPressExceed4Kg  = 320;
}

void StoreStartHydrgPumpSpdPara(HYDROGEN_PUMP_SPEED_PARA_Typedef *i_StartPumpSpdPara)
{
    STMFLASH_Write(SYSTEM_PARAMETER_STORE_SEGMENT_ADDR + RUN_HYDROGEN_PUMP_SPEED_PARA_STORE_OFFSET_ADDR, (uint16_t*)i_StartPumpSpdPara, 3);
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
    i_StartHydrgFanSpdPara->FanSpdIgniterSecondTime   = 1500;
    i_StartHydrgFanSpdPara->FanSpdAfterIgniterSecondSuccessd = 2000;
}

void StoreStartHydrgFanSpdPara(HYDROGEN_FAN_SPEED_PARA_Typedef *i_StartHydrgFanSpdPara)
{
    STMFLASH_Write(SYSTEM_PARAMETER_STORE_SEGMENT_ADDR + RUN_HYDROGEN_FAN_SPEED_PARA_STORE_OFFSET_ADDR, (uint16_t*)i_StartHydrgFanSpdPara, 4);
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

