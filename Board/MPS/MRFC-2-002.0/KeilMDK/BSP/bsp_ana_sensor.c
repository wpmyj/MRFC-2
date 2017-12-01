/*
***************************************************************************************************
*                                              APPLICATION CODE
*
*                          (c) Copyright 2015; Guangdong Hydrogen Energy Science And Technology Co.,Ltd
*
*               All rights reserved.  Protected by international copyright laws.
*               Knowledge of the source code may NOT be used without authorization.
***************************************************************************************************
*/
/*
***************************************************************************************************
* Filename      : bsp_ana_sensor.c
* Version       : V1.00
* Programmer(s) : SunKing.Yun
***************************************************************************************************
*/

/*
***************************************************************************************************
*                                             INCLUDE FILES
***************************************************************************************************
*/
#define  BSP_ANA_SENSOR_MODULE
#include "math.h"
#include <bsp.h>
#include "os_cfg_app.h"
#include <bsp_ana_sensor.h>
#include "app_system_real_time_parameters.h"
#include "app_analog_signal_monitor_task.h"
/*
***************************************************************************************************
*                                           MACRO DEFINITIONS
***************************************************************************************************
*/
#define NMB_OF_AVERAGE_WHEN_CALIBRATION          5u     //����У׼��������
#define NMB_OF_AVERAGE_ANALOG_SIGNAL_SAMPLE      20u    //ƽ���˲�ģ���ź���������

//������������ز���λ����
//bit[0]�����Ƿ�����        0:��У��    1:�޷�У��
//bit[1]�����Ƿ�˫���Ӧ    0:����  1:˫��
#define     ANA_SENSOR_CALIBRATE_CHECK              0x01
#define     ANA_SENSOR_CALIBRATE_ENABLED            0x00
#define     ANA_SENSOR_CALIBRATE_DISABLED           0x01

#define     ANA_SNESOR_DIRECTION_CHECK              0x01
#define     ANA_SENSOR_UNIDIRECTION                 0x00
#define     ANA_SENSOR_BIDIRECTIONAL                0x01
/*
***************************************************************************************************
*                                            LOCAL DATA TYPES
***************************************************************************************************
*/


/*
***************************************************************************************************
*                                         OS-RELATED    VARIABLES
***************************************************************************************************
*/

/*
***************************************************************************************************
*                                       LOCAL GLOBAL VARIABLES
***************************************************************************************************
*/
static      uint16_t    g_u16OriginalDigValue[BSP_ANA_SENSORS_NMB] = {0};//ԭʼ�źŴ洢λ��
static      uint32_t    g_u32DigValueFilter[BSP_ANA_SENSORS_NMB][NMB_OF_AVERAGE_ANALOG_SIGNAL_SAMPLE] = {0};//�˲�������
static      float       g_fDigValueSum[BSP_ANA_SENSORS_NMB] = {0};      //�˲��������ݺ�
static      uint8_t     g_u8FilterOperationCursor = 0;                  //�˲������ݸ��±��
static      float       g_fDigFilteredValue[BSP_ANA_SENSORS_NMB] = {0}; //�˲������ֵ
static      uint8_t     g_u8AnaSensorTypeNmb[BSP_ANA_SENSORS_NMB] = {0};

//��������Ĵ�������ز���
static ANALOG_SIGNAL_SERSOR_PARAMETERS_Typedef g_stAnaSigSensorParameter[BSP_ANA_SENSORS_NMB];
static ANALOG_SIGNAL_SERSOR_PARAMETERS_Typedef g_stAnaSigSensorDefaultParameter[BSP_ANA_SENSORS_NMB] = {
    {0, 0xFFFF},       //����¶�
    {790, 17.3955},    //��ѹ1.48--4.783.294��(4ma- 12.9277ma)* 370��
    {2007.35, 16.059}, //����
    {794.2, 127.06},   //Һѹ
    {359.86, 33.504},  //��ѹ1:0.58 / 2 * 3.3 * 4095 ~ 4.9/2 *3.3 * 4095   359.86 - 3040.23 LSB,��Ӧ0.58 - 4.9����,��2��5.1K���轵ѹ��0.29-2.45V��������0.29V-3.3V��Ӧ0-135Kpa������Ӧ0-80KPa������Ϊ33.504LSB/KPa
    {359.86, 33.504},  //��ѹ2-->��ΪҺλ2
    {794.2, 3.1767},  // Һλ1
    {790, 17.3955},    //��ѹ1.48--4.783.294��(4ma- 12.9277ma)* 370��
    {0, 1241.21},     // ����Ũ��
	{0, 1241.21},	  //��ص���
	{0, 1241.21},	  //���ټ���������
	{1578.5, 10.592},  //��ѹ������794.375������-100Kpa-200Kpa,����ź���0.64-3.2V����Ӧ794.375-3971.881LSB��������0-300Kpa���������
	{0, 0xFFFF},	  // Ԥ�������ʹ�����1
	{0, 0xFFFF},	  // Ԥ�������ʹ�����2
};

static      float       g_fAnaSigAnaValue[BSP_ANA_SENSORS_NMB] = {0};//������ǰ�����źŵ�Դ�����ź�ֵ����׼��λ��

/*
***************************************************************************************************
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
***************************************************************************************************
*/
void AnaSigSampleStart(void)
{
    BSP_AnaSensorConvertStart(g_u16OriginalDigValue, BSP_ANA_SENSORS_NMB);
}

/*
***************************************************************************************************
*                                   BSP_GetCalibratedAnaSensorPara()
*
* Description : The function get the calibrated analog sensor parameters.
*
* Arguments   : none.
*
* Returns     : none.
*
* Caller(s)   : none.
*
***************************************************************************************************
*/
ErrorStatus BSP_GetCalibratedAnaSensorPara(ANALOG_SIGNAL_SERSOR_PARAMETERS_Typedef *o_stAnaSensorPara, uint8_t i_u8Length)
{
    OS_ERR err;
    uint8_t i, j;
    ErrorStatus eCalibratedFlag = SUCCESS;
    uint8_t u8ConvertSuccessCount = 0;
    float Temp;
    float fOriginalDigValueSum[BSP_ANA_SENSORS_NMB] = {0};

    OSTimeDlyHMSM(0, 0, 0, 100,    //�ȴ��������ϵ���
                  OS_OPT_TIME_HMSM_STRICT,
                  &err);

    for(i = 0; i < NMB_OF_AVERAGE_WHEN_CALIBRATION; i++) {

        AnaSigSampleStart();    //ģ���źŲ�����ʼ
        OSSemPend(&g_stAnaSigConvertFinishSem,
                  OS_CFG_TICK_RATE_HZ,
                  OS_OPT_PEND_BLOCKING,
                  NULL,
                  &err);

        if(err == OS_ERR_NONE) {
            for(j = 0; j < BSP_ANA_SENSORS_NMB; j++) {
                fOriginalDigValueSum[j] += g_u16OriginalDigValue[j];
            }
        } else if(err == OS_ERR_TIMEOUT) {
            APP_TRACE_INFO(("The analog sensor's waitting the dma convert has been time out...\r\n"));
            break;
        } else {
            APP_TRACE_INFO(("the ana sensor init go to undefine err...\r\n"));
        }
    }

    u8ConvertSuccessCount = i;//ת���ɹ��Ĵ���

    //��ת���ɹ���ͨ�����в���У׼
    if(err == OS_ERR_NONE) {
        APP_TRACE_INFO(("Capture the analog sensors parameters successed,begin calibrate ...\r\n"));

        for(i = 0; i < BSP_ANA_SENSORS_NMB; i++) {
            /*���Դ�����ͨ��������У��*/
            if((g_u8AnaSensorTypeNmb[i] & ANA_SENSOR_CALIBRATE_CHECK) == ANA_SENSOR_CALIBRATE_ENABLED) {
                Temp = fOriginalDigValueSum[i] / u8ConvertSuccessCount;//�����ܺ�/�����ɹ�����

                if((Temp - g_stAnaSigSensorDefaultParameter[i].BaseDigValue) <= 250  \
                        && (Temp - g_stAnaSigSensorDefaultParameter[i].BaseDigValue) >= -250) { //����ǰֵ���������õ�ǰֵ��250Լ����0.2V�ź�ƫ��
                    APP_TRACE_INFO(("The analog sensors of Channel %d is normal...\r\n", i));
                    g_stAnaSigSensorParameter[i].BaseDigValue = Temp;
                } else { //����ǰֵ������������Ĭ��ֵ
                    APP_TRACE_INFO(("The analog sensors of channel %d is Unnormal, use default parameters...\r\n", i));
                    g_stAnaSigSensorParameter[i].BaseDigValue = g_stAnaSigSensorDefaultParameter[i].BaseDigValue;
                    eCalibratedFlag = ERROR;
                }
            } else {} //����У��
        }
    } else {
        APP_TRACE_INFO(("Captur the analog sensor parameters failed, use the default parameters...\r\n"));

        for(i = 0; i < BSP_ANA_SENSORS_NMB; i++) {
            g_stAnaSigSensorParameter[i].BaseDigValue = g_stAnaSigSensorDefaultParameter[i].BaseDigValue;
        }

        eCalibratedFlag = ERROR;
    }

    /*����������ģת���Ȳ���Ҫ�ı�*/
    for(i = 0; i < BSP_ANA_SENSORS_NMB; i++) {
        /* ���Դ�����ͨ��������У�� */
        if((g_u8AnaSensorTypeNmb[i] & ANA_SENSOR_CALIBRATE_CHECK) == ANA_SENSOR_CALIBRATE_ENABLED) {
            g_stAnaSigSensorParameter[i].AnaToDigRatio = g_stAnaSigSensorDefaultParameter[i].AnaToDigRatio;
        }
    }

    /*����У׼���ֵ*/
    for(i = 0; i < i_u8Length; i++) {
        (o_stAnaSensorPara + i)->AnaToDigRatio = g_stAnaSigSensorParameter[i].AnaToDigRatio;
        (o_stAnaSensorPara + i)->BaseDigValue = g_stAnaSigSensorParameter[i].BaseDigValue;
        o_stAnaSensorPara = o_stAnaSensorPara;
    }

    return eCalibratedFlag;
}
/*
***************************************************************************************************
*                                                BSP_LoadAnaSensorParaToPrgm()
*
* Description : The function load the parameters that along to the sensor model and circuit from the rom flash.
*
* Arguments   : none.
*
* Returns     : none.
*
* Caller(s)   : Application.
*
***************************************************************************************************
*/
void BSP_LoadAnaSensorParaToPrgm(ANALOG_SIGNAL_SERSOR_PARAMETERS_Typedef *i_ParaGrps, uint8_t i_u8GrpLenth)
{
    int i;

    for(i = 0; i < i_u8GrpLenth; i++) {
        g_stAnaSigSensorParameter[i] = *(i_ParaGrps + i);
    }
}

void BSP_SaveAnaSensorParameters(float *i_NewParameters)
{
    int i;

    for(i = 0; i < BSP_ANA_SENSORS_NMB; i++) {
        *(i_NewParameters + i * 2) = g_stAnaSigSensorParameter[0].BaseDigValue;
        *(i_NewParameters + i * 2 + 1) = g_stAnaSigSensorParameter[0].AnaToDigRatio;
    }
}

/*
***************************************************************************************************
*                                                AnaSensorSelfCheck()
*
* Description : The function load the parameters that along to the sensor model and circuit from the rom flash.
*
* Arguments   : none.
*
* Returns     : none.
*
* Caller(s)   : Application.
*
***************************************************************************************************
*/
void AnaSensorSelfCheck(void)
{
    float Temp;

    //����¶Ȳ���Ҫ���㴦��
    Temp = g_u16OriginalDigValue[0];

    if(Temp >= (3775.43)) { //�˿����ն�Ӧ��ֵ
        SetMachinePartBSelfCheckCodeBit(SelfCheckCodeGrpFCAnaTempBit);
    } else {
        ResetMachinePartBSelfCheckCodeBit(SelfCheckCodeGrpFCAnaTempBit);
    }

    //��ѵ�ѹ
    Temp = g_u16OriginalDigValue[1] - g_stAnaSigSensorParameter[1].BaseDigValue;

    if(Temp >= 200 || Temp <= -200) {
        SetMachinePartBSelfCheckCodeBit(SelfCheckCodeGrpFCAnaVoltageBit);
    } else {
        ResetMachinePartBSelfCheckCodeBit(SelfCheckCodeGrpFCAnaVoltageBit);
    }

    //��ѵ���
    Temp = g_u16OriginalDigValue[2] - g_stAnaSigSensorParameter[2].BaseDigValue;

    if(Temp >= 200 || Temp <= -200) {
        SetMachinePartBSelfCheckCodeBit(SelfCheckCodeGrpFCAnaCurrentBit);
    } else {
        ResetMachinePartBSelfCheckCodeBit(SelfCheckCodeGrpFCAnaCurrentBit);
    }

    //Һѹ
    Temp = g_u16OriginalDigValue[3] - g_stAnaSigSensorParameter[3].BaseDigValue;

    if(Temp >= 200 || Temp <= -200) {
        SetMachinePartASelfCheckCodeBit(SelfCheckCodeGrpHydrgLqdPressBit);
    } else {
        ResetMachinePartASelfCheckCodeBit(SelfCheckCodeGrpHydrgLqdPressBit);
    }

    //��ѹ1
    Temp = g_u16OriginalDigValue[4] - g_stAnaSigSensorParameter[4].BaseDigValue;

    if(Temp >= 200 || Temp <= -200) {
        SetMachinePartBSelfCheckCodeBit(SelfCheckCodeGrpFCAnaHydrgPressBit);
    } else {
        ResetMachinePartBSelfCheckCodeBit(SelfCheckCodeGrpFCAnaHydrgPressBit);
    }
   
	//��ѹ2--�ĳɵ�ѹ2
	Temp = g_u16OriginalDigValue[5] - g_stAnaSigSensorParameter[5].BaseDigValue;
	if(Temp >= 200 || Temp <= -200)
	{
		SetMachinePartBSelfCheckCodeBit(SelfCheckCodeGrpFCAnaRsvdBit_0);
	}else {
        ResetMachinePartBSelfCheckCodeBit(SelfCheckCodeGrpFCAnaRsvdBit_0);
    }

	//Һλ
	Temp = g_u16OriginalDigValue[6] - g_stAnaSigSensorParameter[6].BaseDigValue;
	if(Temp >= 200 || Temp <= -200)
	{
		SetMachinePartBSelfCheckCodeBit(SelfCheckCodeGrpHydrgAnaRsvdBit_0);
	}else {
        ResetMachinePartBSelfCheckCodeBit(SelfCheckCodeGrpHydrgAnaRsvdBit_0);
    }

//	//����Ũ��
//	Temp = g_u16OriginalDigValue[7] - g_stAnaSigSensorParameter[7].BaseDigValue;
//	if(Temp >= 200 || Temp <= -200)
//	{
//		SetMachinePartBSelfCheckCodeBit(SelfCheckCodeGrpHydrgAnaRsvdBit_1);
//	}else {
//        ResetMachinePartBSelfCheckCodeBit(SelfCheckCodeGrpHydrgAnaRsvdBit_1);
//    }
	
//	//��ص���
//	Temp = g_u16OriginalDigValue[8] - g_stAnaSigSensorParameter[8].BaseDigValue;
//	if(Temp >= 200 || Temp <= -200)
//	{
//		SetMachinePartBSelfCheckCodeBit(SelfCheckCodeGrpFCAnaRsvdBit_1);
//	}else {
//        ResetMachinePartBSelfCheckCodeBit(SelfCheckCodeGrpFCAnaRsvdBit_1);
//    }
	
//	//���ټ���������
//	Temp = g_u16OriginalDigValue[9] - g_stAnaSigSensorParameter[9].BaseDigValue;
//	if(Temp >= 200 || Temp <= -200)
//	{
//		SetMachinePartBSelfCheckCodeBit(SelfCheckCodeGrpHydrgAnaRsvdBit_2);
//	}else {
//        ResetMachinePartBSelfCheckCodeBit(SelfCheckCodeGrpHydrgAnaRsvdBit_2);
//    }
	
	//��ѹ������
	Temp = g_u16OriginalDigValue[10] - g_stAnaSigSensorParameter[10].BaseDigValue;
	if(Temp >= 200 || Temp <= -200)
	{
		SetMachinePartBSelfCheckCodeBit(SelfCheckCodeGrpHydrgAnaRsvdBit_3);
	}else {
        ResetMachinePartBSelfCheckCodeBit(SelfCheckCodeGrpHydrgAnaRsvdBit_3);
    }

//	//Ԥ�������ʹ�����1
//	Temp = g_u16OriginalDigValue[11] - g_stAnaSigSensorParameter[11].BaseDigValue;
//	if(Temp >= 200 || Temp <= -200)
//	{
//		SetMachinePartBSelfCheckCodeBit(SelfCheckCodeGrpFCAnaRsvdBit_2);
//	}else {
//        ResetMachinePartBSelfCheckCodeBit(SelfCheckCodeGrpFCAnaRsvdBit_2);
//    }
   
}
/*
***************************************************************************************************
*                                                UpdateAnaSigDigValue()
*
* Description : The function can convert the digtal value of the analog sersor to natural signal.
*
* Arguments   : none.
*
* Returns     : none.
*
* Caller(s)   : Application.
*
***************************************************************************************************
*/

void UpdateAnaSigDigValue()
{
    uint8_t     i;

    if(g_u8FilterOperationCursor >= NMB_OF_AVERAGE_ANALOG_SIGNAL_SAMPLE) {
        g_u8FilterOperationCursor = 0;
    }

    //�����˲��㷨:�˲�ֵ=[(�ܺ�-�˲�ֵ)+ԭʼֵ]/��������
    for(i = 0; i < BSP_ANA_SENSORS_NMB; i++) {
        if(i != HYDROGEN_PRESS_1) { //�����ѹֵ���˲�
            g_fDigValueSum[i] -= g_u32DigValueFilter[i][g_u8FilterOperationCursor];
            g_u32DigValueFilter[i][g_u8FilterOperationCursor] = g_u16OriginalDigValue[i];
            g_fDigValueSum[i] += g_u32DigValueFilter[i][g_u8FilterOperationCursor];
            g_fDigFilteredValue[i] = g_fDigValueSum[i] / NMB_OF_AVERAGE_ANALOG_SIGNAL_SAMPLE;
        } else {
            g_fDigFilteredValue[i] = g_u16OriginalDigValue[i];
        }
    }

    g_u8FilterOperationCursor ++;
}
/*
***************************************************************************************************
*                                                BSP_GetSourceAnaSig()
*
* Description : The function can convert the digtal value of the analog sensor to natural signal.
*
* Arguments   : i_eAnaSigKind --��ȡ��ģ���ź�����.
*
* Returns     : g_fAnaSigAnaValue
*
* Caller(s)   : Application.
*
***************************************************************************************************
*/

float GetSrcAnaSig(ANALOG_SIGNAL_KIND_Typedef i_eAnaSigKind)
{
    float fMidRtMul100K, fRtValue;

    if(STACK_TEMP == i_eAnaSigKind) {
        fMidRtMul100K = 59000 * (3.3 * g_fDigFilteredValue[STACK_TEMP] / 5.0 / 4095) / (1 - 3.3 * g_fDigFilteredValue[STACK_TEMP] / 5.0 / 4095);
        fRtValue = fMidRtMul100K * 100000 / (100000 - fMidRtMul100K);
        g_fAnaSigAnaValue[STACK_TEMP] = (float)(GetSourceTemp(fRtValue));
//            APP_TRACE_INFO(("--> g_fDigFilteredValue[STACK_TEMP]: %f \r\n", g_fDigFilteredValue[STACK_TEMP]));
//            APP_TRACE_INFO(("--> g_fAnaSigAnaValue[STACK_TEMP]: %f \r\n\r\n", g_fAnaSigAnaValue[STACK_TEMP]));
    } else {
        if(STACK_VOLTAGE == i_eAnaSigKind) {
            if(g_fDigFilteredValue[STACK_VOLTAGE] <= g_stAnaSigSensorParameter[STACK_VOLTAGE].BaseDigValue) {
                g_fAnaSigAnaValue[STACK_VOLTAGE] = 0.0;
            } else {
                g_fAnaSigAnaValue[STACK_VOLTAGE] = (g_fDigFilteredValue[STACK_VOLTAGE] - g_stAnaSigSensorParameter[STACK_VOLTAGE].BaseDigValue) / g_stAnaSigSensorParameter[STACK_VOLTAGE].AnaToDigRatio;
            }
//            APP_TRACE_INFO(("--> g_fDigFilteredValue[STACK_VOLTAGE]: %f \r\n", g_fDigFilteredValue[STACK_VOLTAGE]));
//            APP_TRACE_INFO(("--> g_stAnaSigSensorParameter[STACK_VOLTAGE].BaseDigValue: %f \r\n", g_stAnaSigSensorParameter[STACK_VOLTAGE].BaseDigValue));
//            APP_TRACE_INFO(("--> g_fAnaSigAnaValue[STACK_VOLTAGE]: %f \r\n\r\n", g_fAnaSigAnaValue[STACK_VOLTAGE]));
		}else if(BATTERY_VOLTAGE == i_eAnaSigKind) {
			if(g_fDigFilteredValue[BATTERY_VOLTAGE] <= g_stAnaSigSensorParameter[BATTERY_VOLTAGE].BaseDigValue) {
				g_fAnaSigAnaValue[BATTERY_VOLTAGE] = 0.0;
			} else {
				g_fAnaSigAnaValue[BATTERY_VOLTAGE] = (g_fDigFilteredValue[BATTERY_VOLTAGE] - g_stAnaSigSensorParameter[BATTERY_VOLTAGE].BaseDigValue) / g_stAnaSigSensorParameter[BATTERY_VOLTAGE].AnaToDigRatio;
			}
        } else if(STACK_CURRENT == i_eAnaSigKind) {
            if(g_fDigFilteredValue[STACK_CURRENT] <= g_stAnaSigSensorParameter[STACK_CURRENT].BaseDigValue) {
                g_fAnaSigAnaValue[STACK_CURRENT] = 0.0;
            } else if(g_fDigFilteredValue[STACK_CURRENT] >= 4014.71) {
                g_fAnaSigAnaValue[STACK_CURRENT] = 100.0;
            } else {
                g_fAnaSigAnaValue[STACK_CURRENT] = (fabs((g_fDigFilteredValue[STACK_CURRENT] - g_stAnaSigSensorParameter[STACK_CURRENT].BaseDigValue) + 1) / g_stAnaSigSensorParameter[STACK_CURRENT].AnaToDigRatio);
            }

        } else if(LIQUID_PRESS == i_eAnaSigKind) {
            if(g_fDigFilteredValue[LIQUID_PRESS] <= g_stAnaSigSensorParameter[LIQUID_PRESS].BaseDigValue) {
                g_fAnaSigAnaValue[LIQUID_PRESS] = 0.0;
            } else if(g_fDigFilteredValue[LIQUID_PRESS] >= 3970.91) {
                g_fAnaSigAnaValue[LIQUID_PRESS] = 25.0;
            } else {
                g_fAnaSigAnaValue[LIQUID_PRESS] = (fabs((g_fDigFilteredValue[LIQUID_PRESS] - g_stAnaSigSensorParameter[LIQUID_PRESS].BaseDigValue) + 1) / g_stAnaSigSensorParameter[LIQUID_PRESS].AnaToDigRatio);
            }
		}else if(NEGATIVE_PRESSURE == i_eAnaSigKind) {
        
			if(g_fDigFilteredValue[NEGATIVE_PRESSURE] > g_stAnaSigSensorParameter[NEGATIVE_PRESSURE].BaseDigValue){
				g_fAnaSigAnaValue[NEGATIVE_PRESSURE] = 0.0;//��ѹ���ݲ���ʾ
			}else if(g_fDigFilteredValue[NEGATIVE_PRESSURE] < 794.375){//�쳣����
				g_fAnaSigAnaValue[NEGATIVE_PRESSURE] = 0.0;
			}else{
				//��0-300��ֵת��Ϊ-100��200��ֵ,���ֻ��ʾ��ѹ
				g_fAnaSigAnaValue[NEGATIVE_PRESSURE] =(g_stAnaSigSensorParameter[NEGATIVE_PRESSURE].BaseDigValue - g_fDigFilteredValue[NEGATIVE_PRESSURE]) / (g_stAnaSigSensorParameter[NEGATIVE_PRESSURE].AnaToDigRatio);
			}
//        APP_TRACE_INFO(("--> g_fDigFilteredValue[NEGATIVE_PRESSURE]: %f \r\n", g_fDigFilteredValue[NEGATIVE_PRESSURE]));
//        APP_TRACE_INFO(("--> g_stAnaSigSensorParameter[NEGATIVE_PRESSURE].BaseDigValue: %f \r\n", g_stAnaSigSensorParameter[NEGATIVE_PRESSURE].BaseDigValue));
//        APP_TRACE_INFO(("--> g_fAnaSigAnaValue[NEGATIVE_PRESSURE]: %f \r\n\r\n", g_fAnaSigAnaValue[NEGATIVE_PRESSURE]));
        } else if(HYDROGEN_PRESS_1 == i_eAnaSigKind) {

            if(g_fDigFilteredValue[HYDROGEN_PRESS_1] <= g_stAnaSigSensorParameter[HYDROGEN_PRESS_1].BaseDigValue) {
                g_fAnaSigAnaValue[HYDROGEN_PRESS_1] = 0.0;
            } else if(g_fDigFilteredValue[HYDROGEN_PRESS_1] >= 3040.23) {
                g_fAnaSigAnaValue[HYDROGEN_PRESS_1] = 80.0;
            } else {
                g_fAnaSigAnaValue[HYDROGEN_PRESS_1] = (fabs((g_fDigFilteredValue[HYDROGEN_PRESS_1] - g_stAnaSigSensorParameter[HYDROGEN_PRESS_1].BaseDigValue) + 1) / g_stAnaSigSensorParameter[HYDROGEN_PRESS_1].AnaToDigRatio);
            }

//            APP_TRACE_INFO(("--> g_fDigFilteredValue[HYDROGEN_PRESS_1]: %f \r\n", g_fDigFilteredValue[HYDROGEN_PRESS_1]));
//            APP_TRACE_INFO(("--> g_stAnaSigSensorParameter[HYDROGEN_PRESS_1].BaseDigValue: %f \r\n", g_stAnaSigSensorParameter[HYDROGEN_PRESS_1].BaseDigValue));
//            APP_TRACE_INFO(("--> g_fAnaSigAnaValue[HYDROGEN_PRESS_1]: %f \r\n\r\n", g_fAnaSigAnaValue[HYDROGEN_PRESS_1]));
        } else if(LIQUID_LEVEL == i_eAnaSigKind) {
            if(g_fDigFilteredValue[LIQUID_LEVEL] <= g_stAnaSigSensorParameter[LIQUID_LEVEL].BaseDigValue) {
                g_fAnaSigAnaValue[LIQUID_LEVEL] = 0.0;
            } else {
                g_fAnaSigAnaValue[LIQUID_LEVEL] = (g_fDigFilteredValue[LIQUID_LEVEL] - g_stAnaSigSensorParameter[LIQUID_LEVEL].BaseDigValue) / g_stAnaSigSensorParameter[LIQUID_LEVEL].AnaToDigRatio;
            }

        } else {}
    }

    return  g_fAnaSigAnaValue[i_eAnaSigKind];
}
