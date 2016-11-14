/*
*********************************************************************************************************
*                                              APPLICATION CODE
*
*                          (c) Copyright 2015; Guangdong Hydrogen Energy Science And Technology Co.,Ltd
*
*               All rights reserved.  Protected by international copyright laws.
*               Knowledge of the source code may NOT be used without authorization.
*********************************************************************************************************
*/
/*
*********************************************************************************************************
* Filename      : bsp_ana_sensor.c
* Version       : V1.00
* Programmer(s) : SunKing.Yun
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/
#define  BSP_ANA_SENSOR_MODULE
#include "math.h"
#include <bsp.h>
#include "os_cfg_app.h"
#include <bsp_ana_sensor.h>
#include "app_system_real_time_parameters.h"

/*
*********************************************************************************************************
*                                           MACRO DEFINITIONS
*********************************************************************************************************
*/
#define BSP_ANA_SENSORS_NMB                      9u     //ģ���źŲ���ͨ����
#define NMB_OF_AVERAGE_WHEN_CALIBRATION          5u     //ƽ��У׼ʱ��
#define NMB_OF_AVERAGE_ANALOG_SIGNAL_SAMPLE      20u    //ƽ��ģ���ź���������

/*
*********************************************************************************************************
*                                            LOCAL DATA TYPES
*********************************************************************************************************
*/
typedef struct
{
    float    BaseDigValue;      //ԭʼ������
    float    AnaToDigRatio; //ģ��ת������
} ANALOG_SIGNAL_SERSOR_PARAMETERS_Typedef;

/*
*********************************************************************************************************
*                                         OS-RELATED    VARIABLES
*********************************************************************************************************
*/
extern      OS_SEM      g_stAnaSigConvertFinishSem;
/*
*********************************************************************************************************
*                                       LOCAL GLOBAL VARIABLES
*********************************************************************************************************
*/

extern      uint64_t    g_u64SysSelfCheckCode;

static      vu16        g_u16OriginalDigValue[BSP_ANA_SENSORS_NMB] = {0};//ԭʼ�źŴ洢λ��
static      uint32_t    g_u32DigValueFilter[BSP_ANA_SENSORS_NMB][NMB_OF_AVERAGE_ANALOG_SIGNAL_SAMPLE] = {0};//�˲�������
static      float       g_fDigValueSum[BSP_ANA_SENSORS_NMB] = {0};      //�˲��������ݺ�
static      uint8_t     g_u8FilterOperationCursor = 0;                  //�˲������ݸ��±��
static      float       g_fDigFilteredValue[BSP_ANA_SENSORS_NMB] = {0}; //�˲������ֵ

//��������Ĵ�������ز���
static ANALOG_SIGNAL_SERSOR_PARAMETERS_Typedef g_stAnaSigSensorParameter[BSP_ANA_SENSORS_NMB];
static ANALOG_SIGNAL_SERSOR_PARAMETERS_Typedef g_stAnaSigSensorDefaultParameter[BSP_ANA_SENSORS_NMB] =
                                                                                                    {
                                                                                                        {0, 0xFFFF},      //����¶�
                                                                                                        {819.2, 32.0},      //��ѹ
                                                                                                        {0, 3.921},      //����{2007.35, 16.059},
                                                                                                        {794.2, 127.06}, //Һѹ
                                                                                                        {0, 6.06},     // ��ѹ1{401.47, 45.165}
                                                                                                        {0, 1241.21},     // ��ѹ2{401.47, 45.165}
                                                                                                        {708.98, 3.1767}, // Һλ
                                                                                                        {0, 1241.21},     // Ԥ��1
                                                                                                        {0, 1241.21}      // Ԥ��2
                                                                                                    };

static      float       g_fAnaSigAnaValue[BSP_ANA_SENSORS_NMB] = {0};           //������ǰ�����źŵ�Դ�����ź�ֵ����׼��λ��

/*
*********************************************************************************************************
*                                                AnaSigSampleStart()
*
* Description : The function start the  analog signal to digital signal convert.
*
* Arguments   : none.
*
* Returns     : none.
*
* Caller(s)   : Application.
*
*********************************************************************************************************
*/
void AnaSigSampleStart(void)
{
    BSP_AnaSensorConvertStart(g_u16OriginalDigValue, BSP_ANA_SENSORS_NMB);
}
/*
*********************************************************************************************************
*                                                BSP_LoadAnaSensorParameters()
*
* Description : The function load the parameters that along to the sensor model and circuit from the rom flash.
*
* Arguments   : none.
*
* Returns     : none.
*
* Caller(s)   : Application.
*
*********************************************************************************************************
*/
void BSP_LoadAnaSensorParameters(float *i_NewParameters)
{
    g_stAnaSigSensorParameter[0].BaseDigValue       = *i_NewParameters;
    g_stAnaSigSensorParameter[0].AnaToDigRatio  = *(i_NewParameters + 1);

    g_stAnaSigSensorParameter[1].BaseDigValue       = *(i_NewParameters + 2);
    g_stAnaSigSensorParameter[1].AnaToDigRatio  = *(i_NewParameters + 3);

    g_stAnaSigSensorParameter[2].BaseDigValue       = *(i_NewParameters + 4);
    g_stAnaSigSensorParameter[2].AnaToDigRatio  = *(i_NewParameters + 5);

    g_stAnaSigSensorParameter[3].BaseDigValue       = *(i_NewParameters + 6);
    g_stAnaSigSensorParameter[3].AnaToDigRatio  = *(i_NewParameters + 7);

    g_stAnaSigSensorParameter[4].BaseDigValue       = *(i_NewParameters + 8);
    g_stAnaSigSensorParameter[4].AnaToDigRatio  = *(i_NewParameters + 9);

    g_stAnaSigSensorParameter[5].BaseDigValue       = *(i_NewParameters + 10);
    g_stAnaSigSensorParameter[5].AnaToDigRatio  = *(i_NewParameters + 11);

    g_stAnaSigSensorParameter[6].BaseDigValue       = *(i_NewParameters + 12);
    g_stAnaSigSensorParameter[6].AnaToDigRatio  = *(i_NewParameters + 13);

    g_stAnaSigSensorParameter[7].BaseDigValue       = *(i_NewParameters + 14);
    g_stAnaSigSensorParameter[7].AnaToDigRatio  = *(i_NewParameters + 15);

    g_stAnaSigSensorParameter[8].BaseDigValue       = *(i_NewParameters + 16);
    g_stAnaSigSensorParameter[8].AnaToDigRatio  = *(i_NewParameters + 17);


}

void BSP_SaveAnaSensorParameters(float *i_NewParameters)
{
    *i_NewParameters        = g_stAnaSigSensorParameter[0].BaseDigValue;
    *(i_NewParameters + 1)  = g_stAnaSigSensorParameter[0].AnaToDigRatio;

    *(i_NewParameters + 2)  = g_stAnaSigSensorParameter[1].BaseDigValue;
    *(i_NewParameters + 3)  = g_stAnaSigSensorParameter[1].AnaToDigRatio;

    *(i_NewParameters + 4)  = g_stAnaSigSensorParameter[2].BaseDigValue;
    *(i_NewParameters + 5)  = g_stAnaSigSensorParameter[2].AnaToDigRatio;

    *(i_NewParameters + 6)  = g_stAnaSigSensorParameter[3].BaseDigValue;
    *(i_NewParameters + 7)  = g_stAnaSigSensorParameter[3].AnaToDigRatio;

    *(i_NewParameters + 8)  = g_stAnaSigSensorParameter[4].BaseDigValue;
    *(i_NewParameters + 9)  = g_stAnaSigSensorParameter[4].AnaToDigRatio;

    *(i_NewParameters + 10) = g_stAnaSigSensorParameter[5].BaseDigValue;
    *(i_NewParameters + 11) = g_stAnaSigSensorParameter[5].AnaToDigRatio;

    *(i_NewParameters + 12) = g_stAnaSigSensorParameter[6].BaseDigValue;
    *(i_NewParameters + 13) = g_stAnaSigSensorParameter[6].AnaToDigRatio;

    *(i_NewParameters + 14) = g_stAnaSigSensorParameter[7].BaseDigValue;
    *(i_NewParameters + 15) = g_stAnaSigSensorParameter[7].AnaToDigRatio;

    *(i_NewParameters + 16) = g_stAnaSigSensorParameter[8].BaseDigValue;
    *(i_NewParameters + 17) = g_stAnaSigSensorParameter[8].AnaToDigRatio;
}

//�����׼��ģ�����������Ĳ���
void BSP_LoadCalibratedAnaSensorParameters(void)
{
    OS_ERR err;
    uint8_t i, j;
    float Temp;
    float fOriginalDigValueSum[BSP_ANA_SENSORS_NMB] = {0};

    OSTimeDlyHMSM(0, 0, 0, 10,                      //�ȴ��������ϵ���
                  OS_OPT_TIME_HMSM_STRICT,
                  &err);

    for(i = 0; i < NMB_OF_AVERAGE_WHEN_CALIBRATION; i++)
    {
        AnaSigSampleStart();    //ģ���źŲ�����ʼ1S����һ��
        OSSemPend(&g_stAnaSigConvertFinishSem,
                  OS_CFG_TICK_RATE_HZ,
                  OS_OPT_PEND_BLOCKING,
                  NULL,
                  &err);

        if(err == OS_ERR_NONE)
        {
            for(j = 0; j < BSP_ANA_SENSORS_NMB; j++)
            {
                fOriginalDigValueSum[j] += g_u16OriginalDigValue[j];
            }
        }
        else if(err == OS_ERR_TIMEOUT)
        {
            APP_TRACE_INFO(("The analog sensor's waitting the dma convert has been time out...\n\r"));
            break;
        }
        else
        {
            APP_TRACE_INFO(("the ana sensor init go to undefine err...\n\r"));
        }
    }

    //����У׼
    if(err == OS_ERR_NONE)
    {
        APP_TRACE_INFO(("Captur the analog sensors parameters successed...\n\r"));

        for(i = 1; i < BSP_ANA_SENSORS_NMB; i++)//��1��ͨ���ǵ���¶��źţ�����ҪУ��
        {
            //5s�ڲ����ܺ�/����ʱ��
            Temp = fOriginalDigValueSum[i] / NMB_OF_AVERAGE_WHEN_CALIBRATION;

            if((Temp - g_stAnaSigSensorDefaultParameter[i].BaseDigValue) <= 250
                    && (Temp - g_stAnaSigSensorDefaultParameter[i].BaseDigValue) >= -250)//����ǰֵ���������õ�ǰֵ��250Լ����0.2V�ź�ƫ��
            {
                g_stAnaSigSensorParameter[i].BaseDigValue = Temp;
            }
            else    //����ǰֵ������������Ĭ��ֵ
            {
                g_stAnaSigSensorParameter[i].BaseDigValue = g_stAnaSigSensorDefaultParameter[i].BaseDigValue;
            }
        }
    }
    else
    {
        APP_TRACE_INFO(("Captur the analog sensor parameters failed, use the default parameters...\n\r"));

        for(i = 1; i < BSP_ANA_SENSORS_NMB; i++)//��1��ͨ���ǵ���¶��źţ�����ҪУ��
        {
            g_stAnaSigSensorParameter[i].BaseDigValue = g_stAnaSigSensorDefaultParameter[i].BaseDigValue;
        }
    }

    for(i = 1; i < BSP_ANA_SENSORS_NMB; i++)//��1��ͨ���ǵ���¶��źţ�����ҪУ��
    {
        g_stAnaSigSensorParameter[i].AnaToDigRatio = g_stAnaSigSensorDefaultParameter[i].AnaToDigRatio;
        g_stAnaSigSensorParameter[i].BaseDigValue = g_stAnaSigSensorDefaultParameter[i].BaseDigValue; // test
    }
}


//ģ���źŴ������Լ�
void AnaSensorSelfCheck(void)
{
    float Temp;

    //����¶Ȳ���Ҫ���㴦��
    Temp = g_u16OriginalDigValue[0];

    if(Temp >= (3775.43))//�˿����ն�Ӧ��ֵ
    {
        SetSelfCheckCodeBit(SelfCheckCodeGrpFCAnaTempBit);
    }

    //��ѵ�ѹ
    Temp = g_u16OriginalDigValue[1] - g_stAnaSigSensorParameter[1].BaseDigValue;

    if(Temp >= 100 || Temp <= -100)
    {
        SetSelfCheckCodeBit(SelfCheckCodeGrpFCAnaVoltageBit);
    }

    //��ѵ���
    Temp = g_u16OriginalDigValue[2] - g_stAnaSigSensorParameter[2].BaseDigValue;

    if(Temp >= 100 || Temp <= -100)
    {
        SetSelfCheckCodeBit(SelfCheckCodeGrpFCAnaCurrentBit);
    }

    //Һѹ
    Temp = g_u16OriginalDigValue[3] - g_stAnaSigSensorParameter[3].BaseDigValue;

    if(Temp >= 100 || Temp <= -100)
    {
        SetSelfCheckCodeBit(SelfCheckCodeGrpHydrgLqdPressBit);
    }

    //��ѹ
    Temp = g_u16OriginalDigValue[4] - g_stAnaSigSensorParameter[4].BaseDigValue;

    if(Temp >= 100 || Temp <= -100)
    {
        SetSelfCheckCodeBit(SelfCheckCodeGrpFCAnaHydrgPressBit);
    }

    /*
        //��ѹ2
        Temp = g_u16OriginalDigValue[5] - g_stAnaSigSensorParameter[5].BaseDigValue;
        if(Temp >= 100 || Temp <= -100)
        {
            SetSelfCheckCodeBit(SelfCheckCodeGrpFCAnaRsvdBit_0);
        }

        //Ԥ��1
        Temp = g_u16OriginalDigValue[6] - g_stAnaSigSensorParameter[6].BaseDigValue;
        if(Temp >= 100 || Temp <= -100)
        {
            SetSelfCheckCodeBit(SelfCheckCodeGrpFCAnaRsvdBit_1);
        }

        //Ԥ��2
        Temp = g_u16OriginalDigValue[7] - g_stAnaSigSensorParameter[7].BaseDigValue;
        if(Temp >= 100 || Temp <= -100)
        {
            SetSelfCheckCodeBit(SelfCheckCodeGrpFCAnaRsvdBit_2);
        }

        //Ԥ��3
        Temp = g_u16OriginalDigValue[8] - g_stAnaSigSensorParameter[8].BaseDigValue;
        if(Temp >= 100 || Temp <= -100)
        {
            SetSelfCheckCodeBit(SelfCheckCodeGrpFCAnaRsvdBit_3);
        }
    */
}
/*
*********************************************************************************************************
*                                                BSP_StoreAnaSigParameters()
*
* Description : The function store the parameters that along to the sensor model and circuit from the rom flash.
*
* Arguments   : none.
*
* Returns     : none
*
* Caller(s)   : Application.
*
*********************************************************************************************************
*/
void BSP_StoreAnaSigParameters()
{

}
/*
*********************************************************************************************************
*                                                UpdateAnaSigDigValue()
*
* Description : The function can convert the digtal value of the analog sersor to natural signal.
*               ���²���ֵ
* Arguments   : none.
*
* Returns     : none.
*
* Caller(s)   : Application.
*
*********************************************************************************************************
*/

void UpdateAnaSigDigValue()
{
    uint8_t     i;

    if(g_u8FilterOperationCursor >= NMB_OF_AVERAGE_ANALOG_SIGNAL_SAMPLE)
    {
        g_u8FilterOperationCursor = 0;
    }

    //�����˲��㷨:�˲�ֵ=[(�ܺ�-�˲�ֵ)+ԭʼֵ]/��������
    for(i = 0; i < BSP_ANA_SENSORS_NMB; i++)
    {
        g_fDigValueSum[i] -= g_u32DigValueFilter[i][g_u8FilterOperationCursor];
        g_u32DigValueFilter[i][g_u8FilterOperationCursor] = g_u16OriginalDigValue[i];
        g_fDigValueSum[i] += g_u32DigValueFilter[i][g_u8FilterOperationCursor];
        g_fDigFilteredValue[i] = g_fDigValueSum[i] / NMB_OF_AVERAGE_ANALOG_SIGNAL_SAMPLE;
    }

    g_u8FilterOperationCursor ++;
}
/*
*********************************************************************************************************
*                                                BSP_GetSourceAnaSig()
*
* Description : The function can convert the digtal value of the analog sensor to natural signal.
*               ��ȡģ���ź�ֵ
* Arguments   : none.
*
* Returns     : none
*
* Caller(s)   : Application.
*
*********************************************************************************************************
*/

float GetSrcAnaSig(ANALOG_SIGNAL_KIND_Typedef i_eAnaSigKind)
{
    float fMidRtMul100K, fRtValue;
   

    if(i_eAnaSigKind != STACK_TEMP)
    {   
        if(i_eAnaSigKind == STACK_CURRENT)
        {
             g_fAnaSigAnaValue[i_eAnaSigKind] =  (g_fDigFilteredValue[i_eAnaSigKind] / 4096.0 * 3.3 * 1024 / 3.235 * 0.1);             
             if( g_fAnaSigAnaValue[i_eAnaSigKind] > 51.2 )
             {
                g_fAnaSigAnaValue[i_eAnaSigKind] = ( g_fAnaSigAnaValue[i_eAnaSigKind] - 51.2 ) * 2.477;         
                
             }
             else
             {
                g_fAnaSigAnaValue[i_eAnaSigKind] = 0 ;
             }
             g_fAnaSigAnaValue[i_eAnaSigKind] = g_fAnaSigAnaValue[i_eAnaSigKind];
        }
        else if( i_eAnaSigKind == HYDROGEN_PRESS_1 )
        {
            g_fAnaSigAnaValue[i_eAnaSigKind] =  (g_fDigFilteredValue[i_eAnaSigKind] / 4096.0 * 3.3 * 1024 / 5.0);
            if( g_fAnaSigAnaValue[i_eAnaSigKind] > 51.2 )
            {
                g_fAnaSigAnaValue[i_eAnaSigKind] = g_fAnaSigAnaValue[i_eAnaSigKind] - 51.2;
            }
            else
            {
                g_fAnaSigAnaValue[i_eAnaSigKind] = 0;
            }
            g_fAnaSigAnaValue[i_eAnaSigKind] = g_fAnaSigAnaValue[i_eAnaSigKind] / 460.8 * 80 * 1.25;         
            
        }
        else
        {
            g_fAnaSigAnaValue[i_eAnaSigKind] = (g_fDigFilteredValue[i_eAnaSigKind] - g_stAnaSigSensorParameter[i_eAnaSigKind].BaseDigValue) / g_stAnaSigSensorParameter[i_eAnaSigKind].AnaToDigRatio;
        }
    }
    /*else if(i_eAnaSigKind != STACK_CURRENT)
    {
        g_fAnaSigAnaValue[i_eAnaSigKind] = fabs((g_fDigFilteredValue[i_eAnaSigKind] - g_stAnaSigSensorParameter[i_eAnaSigKind].BaseDigValue)) / g_stAnaSigSensorParameter[i_eAnaSigKind].AnaToDigRatio;
    }*/
    else
    {
        if(g_fDigFilteredValue[i_eAnaSigKind] <= (3775.43))//�˿ڽ�1106K���ϵ���ʱ��ֵ����Ӧ-20�棩
        {
            fMidRtMul100K = 59000 * (3.3 * g_fDigFilteredValue[i_eAnaSigKind] / 5.0 / 4095) / (1 - 3.3 * g_fDigFilteredValue[i_eAnaSigKind] / 5.0 / 4095);
            fRtValue = fMidRtMul100K * 100000 / (100000 - fMidRtMul100K);
            g_fAnaSigAnaValue[i_eAnaSigKind] = GetSrcTemp(fRtValue);
        }
        else
        {
            g_fAnaSigAnaValue[i_eAnaSigKind] = -40;
        }       
    }

    return  g_fAnaSigAnaValue[i_eAnaSigKind];
}

/*
*
*********************************************************************************************************
*                                        BSP_STLM75_CfgSet()
*
* Description : Configures the STLM75 Tempeture sensor
*
* Argument(s) : p_stlm75_cfg    Pointer to the STLM75 configuration.
*
* Return(s)   : DEF_OK     If the STLM75 Tempeture Sensor configuration could be returned
*               DEF_FAIL   If the STLM75 Tempeture sensor configuration could not be returned
*
* Caller(s)   : Application
*
* Note(s)     : none.
*********************************************************************************************************
*/



/*
*********************************************************************************************************
*                                        BSP_STLM75_WrReg_08()
*
* Description : Write 8-bit value to STLM75 register.
*
* Argument(s) : reg        STLM75's register
*                              BSP_STLM75_REG_TEMP
*                              BSP_STLM75_REG_CONF
*                              BSP_STLM75_REG_T_HYST
*                              BSP_STLM75_REG_T_OS
*
* Return(s)   : DEF_OK     If the STLM75's register could be written.
*               DEF_FAIL   If the STLM75's register could not be written.
*
* Caller(s)   : Application
*
* Note(s)     : none.
*********************************************************************************************************
*/




/*
*********************************************************************************************************
*                                        BSP_STLM75_WrReg_16()
*
* Description : Write 16-bit value to STLM75 register.
*
* Argument(s) : reg        STLM75's register
*                              BSP_STLM75_REG_TEMP
*                              BSP_STLM75_REG_CONF
*                              BSP_STLM75_REG_T_HYST
*                              BSP_STLM75_REG_T_OS
*
* Return(s)   : DEF_OK     If the STLM75's register could be written.
*               DEF_FAIL   If the STLM75's register could not be written.
*
* Caller(s)   : Application
*
* Note(s)     : none.
*********************************************************************************************************
*/




/*
*********************************************************************************************************
*                                        BSP_STLM75_RdReg_16()
*
* Description : Read 16-bit register from STLM75 device
*
* Argument(s) : reg        STLM75's register
*                              BSP_STLM75_REG_TEMP
*                              BSP_STLM75_REG_CONF
*                              BSP_STLM75_REG_T_HYST
*                              BSP_STLM75_REG_T_OS
*
* Return(s)   : DEF_OK     If the STLM75's register could be read.
*               DEF_FAIL   If the STLM75's register could not be read.
*
* Caller(s)   : Application
*
* Note(s)     : none.
*********************************************************************************************************
*/




/*
*********************************************************************************************************
*                                        BSP_STLM75_TempGet()
*
* Description : Read the current temperature from the STLM75
*
* Argument(s) : temp_unit       Temperature unit:
*                                   BSP_STLM75_TEMP_UNIT_CELSIUS
*                                   BSP_STLM75_TEMP_UNIT_FAHRENHEIT
*                                   BSP_STLM75_TEMP_UNIT_KELVIN
*
*               p_temp_val      Pointer to the variable that will store the temperature.

*
* Return(s)   : DEF_OK     If the temperature could be read.
*               DEF_FAIL   If the temperature could not be read.
*
* Caller(s)   : Application
*
* Note(s)     : none.
*********************************************************************************************************
*/



