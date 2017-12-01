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
#define NMB_OF_AVERAGE_WHEN_CALIBRATION          5u     //参数校准样本数量
#define NMB_OF_AVERAGE_ANALOG_SIGNAL_SAMPLE      20u    //平均滤波模拟信号样本数量

//传感器类型相关参数位定义
//bit[0]代表是否线性        0:可校零    1:无法校零
//bit[1]代表是否双向感应    0:单向  1:双向
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
static      uint16_t    g_u16OriginalDigValue[BSP_ANA_SENSORS_NMB] = {0};//原始信号存储位置
static      uint32_t    g_u32DigValueFilter[BSP_ANA_SENSORS_NMB][NMB_OF_AVERAGE_ANALOG_SIGNAL_SAMPLE] = {0};//滤波器数组
static      float       g_fDigValueSum[BSP_ANA_SENSORS_NMB] = {0};      //滤波器内数据和
static      uint8_t     g_u8FilterOperationCursor = 0;                  //滤波器数据更新标记
static      float       g_fDigFilteredValue[BSP_ANA_SENSORS_NMB] = {0}; //滤波后输出值
static      uint8_t     g_u8AnaSensorTypeNmb[BSP_ANA_SENSORS_NMB] = {0};

//线性输出的传感器相关参数
static ANALOG_SIGNAL_SERSOR_PARAMETERS_Typedef g_stAnaSigSensorParameter[BSP_ANA_SENSORS_NMB];
static ANALOG_SIGNAL_SERSOR_PARAMETERS_Typedef g_stAnaSigSensorDefaultParameter[BSP_ANA_SENSORS_NMB] = {
    {0, 0xFFFF},       //电堆温度
    {790, 17.3955},    //电压1.48--4.783.294，(4ma- 12.9277ma)* 370Ω
    {2007.35, 16.059}, //电流
    {794.2, 127.06},   //液压
    {359.86, 33.504},  //气压1:0.58 / 2 * 3.3 * 4095 ~ 4.9/2 *3.3 * 4095   359.86 - 3040.23 LSB,对应0.58 - 4.9输入,经2个5.1K电阻降压至0.29-2.45V（满量程0.29V-3.3V对应0-135Kpa），对应0-80KPa，比例为33.504LSB/KPa
    {359.86, 33.504},  //气压2-->改为液位2
    {794.2, 3.1767},  // 液位1
    {790, 17.3955},    //电压1.48--4.783.294，(4ma- 12.9277ma)* 370Ω
    {0, 1241.21},     // 氢气浓度
	{0, 1241.21},	  //电池电流
	{0, 1241.21},	  //快速加热器电流
	{1578.5, 10.592},  //负压传感器794.375，量程-100Kpa-200Kpa,输出信号是0.64-3.2V，对应794.375-3971.881LSB，量程是0-300Kpa，算出比率
	{0, 0xFFFF},	  // 预留电流型传感器1
	{0, 0xFFFF},	  // 预留电流型传感器2
};

static      float       g_fAnaSigAnaValue[BSP_ANA_SENSORS_NMB] = {0};//产生当前数字信号的源物理信号值（标准单位）

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

    OSTimeDlyHMSM(0, 0, 0, 100,    //等待传感器上电充分
                  OS_OPT_TIME_HMSM_STRICT,
                  &err);

    for(i = 0; i < NMB_OF_AVERAGE_WHEN_CALIBRATION; i++) {

        AnaSigSampleStart();    //模拟信号采样开始
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

    u8ConvertSuccessCount = i;//转换成功的次数

    //对转换成功的通道进行参数校准
    if(err == OS_ERR_NONE) {
        APP_TRACE_INFO(("Capture the analog sensors parameters successed,begin calibrate ...\r\n"));

        for(i = 0; i < BSP_ANA_SENSORS_NMB; i++) {
            /*线性传感器通道，可以校零*/
            if((g_u8AnaSensorTypeNmb[i] & ANA_SENSOR_CALIBRATE_CHECK) == ANA_SENSOR_CALIBRATE_ENABLED) {
                Temp = fOriginalDigValueSum[i] / u8ConvertSuccessCount;//采样总和/采样成功次数

                if((Temp - g_stAnaSigSensorDefaultParameter[i].BaseDigValue) <= 250  \
                        && (Temp - g_stAnaSigSensorDefaultParameter[i].BaseDigValue) >= -250) { //若当前值正常，则用当前值，250约代表0.2V信号偏差
                    APP_TRACE_INFO(("The analog sensors of Channel %d is normal...\r\n", i));
                    g_stAnaSigSensorParameter[i].BaseDigValue = Temp;
                } else { //若当前值不正常，则用默认值
                    APP_TRACE_INFO(("The analog sensors of channel %d is Unnormal, use default parameters...\r\n", i));
                    g_stAnaSigSensorParameter[i].BaseDigValue = g_stAnaSigSensorDefaultParameter[i].BaseDigValue;
                    eCalibratedFlag = ERROR;
                }
            } else {} //无需校零
        }
    } else {
        APP_TRACE_INFO(("Captur the analog sensor parameters failed, use the default parameters...\r\n"));

        for(i = 0; i < BSP_ANA_SENSORS_NMB; i++) {
            g_stAnaSigSensorParameter[i].BaseDigValue = g_stAnaSigSensorDefaultParameter[i].BaseDigValue;
        }

        eCalibratedFlag = ERROR;
    }

    /*传感器的数模转换比不需要改变*/
    for(i = 0; i < BSP_ANA_SENSORS_NMB; i++) {
        /* 线性传感器通道，可以校零 */
        if((g_u8AnaSensorTypeNmb[i] & ANA_SENSOR_CALIBRATE_CHECK) == ANA_SENSOR_CALIBRATE_ENABLED) {
            g_stAnaSigSensorParameter[i].AnaToDigRatio = g_stAnaSigSensorDefaultParameter[i].AnaToDigRatio;
        }
    }

    /*保存校准后的值*/
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

    //电堆温度不需要调零处理
    Temp = g_u16OriginalDigValue[0];

    if(Temp >= (3775.43)) { //端口悬空对应的值
        SetMachinePartBSelfCheckCodeBit(SelfCheckCodeGrpFCAnaTempBit);
    } else {
        ResetMachinePartBSelfCheckCodeBit(SelfCheckCodeGrpFCAnaTempBit);
    }

    //电堆电压
    Temp = g_u16OriginalDigValue[1] - g_stAnaSigSensorParameter[1].BaseDigValue;

    if(Temp >= 200 || Temp <= -200) {
        SetMachinePartBSelfCheckCodeBit(SelfCheckCodeGrpFCAnaVoltageBit);
    } else {
        ResetMachinePartBSelfCheckCodeBit(SelfCheckCodeGrpFCAnaVoltageBit);
    }

    //电堆电流
    Temp = g_u16OriginalDigValue[2] - g_stAnaSigSensorParameter[2].BaseDigValue;

    if(Temp >= 200 || Temp <= -200) {
        SetMachinePartBSelfCheckCodeBit(SelfCheckCodeGrpFCAnaCurrentBit);
    } else {
        ResetMachinePartBSelfCheckCodeBit(SelfCheckCodeGrpFCAnaCurrentBit);
    }

    //液压
    Temp = g_u16OriginalDigValue[3] - g_stAnaSigSensorParameter[3].BaseDigValue;

    if(Temp >= 200 || Temp <= -200) {
        SetMachinePartASelfCheckCodeBit(SelfCheckCodeGrpHydrgLqdPressBit);
    } else {
        ResetMachinePartASelfCheckCodeBit(SelfCheckCodeGrpHydrgLqdPressBit);
    }

    //气压1
    Temp = g_u16OriginalDigValue[4] - g_stAnaSigSensorParameter[4].BaseDigValue;

    if(Temp >= 200 || Temp <= -200) {
        SetMachinePartBSelfCheckCodeBit(SelfCheckCodeGrpFCAnaHydrgPressBit);
    } else {
        ResetMachinePartBSelfCheckCodeBit(SelfCheckCodeGrpFCAnaHydrgPressBit);
    }
   
	//气压2--改成电压2
	Temp = g_u16OriginalDigValue[5] - g_stAnaSigSensorParameter[5].BaseDigValue;
	if(Temp >= 200 || Temp <= -200)
	{
		SetMachinePartBSelfCheckCodeBit(SelfCheckCodeGrpFCAnaRsvdBit_0);
	}else {
        ResetMachinePartBSelfCheckCodeBit(SelfCheckCodeGrpFCAnaRsvdBit_0);
    }

	//液位
	Temp = g_u16OriginalDigValue[6] - g_stAnaSigSensorParameter[6].BaseDigValue;
	if(Temp >= 200 || Temp <= -200)
	{
		SetMachinePartBSelfCheckCodeBit(SelfCheckCodeGrpHydrgAnaRsvdBit_0);
	}else {
        ResetMachinePartBSelfCheckCodeBit(SelfCheckCodeGrpHydrgAnaRsvdBit_0);
    }

//	//氢气浓度
//	Temp = g_u16OriginalDigValue[7] - g_stAnaSigSensorParameter[7].BaseDigValue;
//	if(Temp >= 200 || Temp <= -200)
//	{
//		SetMachinePartBSelfCheckCodeBit(SelfCheckCodeGrpHydrgAnaRsvdBit_1);
//	}else {
//        ResetMachinePartBSelfCheckCodeBit(SelfCheckCodeGrpHydrgAnaRsvdBit_1);
//    }
	
//	//电池电流
//	Temp = g_u16OriginalDigValue[8] - g_stAnaSigSensorParameter[8].BaseDigValue;
//	if(Temp >= 200 || Temp <= -200)
//	{
//		SetMachinePartBSelfCheckCodeBit(SelfCheckCodeGrpFCAnaRsvdBit_1);
//	}else {
//        ResetMachinePartBSelfCheckCodeBit(SelfCheckCodeGrpFCAnaRsvdBit_1);
//    }
	
//	//快速加热器电流
//	Temp = g_u16OriginalDigValue[9] - g_stAnaSigSensorParameter[9].BaseDigValue;
//	if(Temp >= 200 || Temp <= -200)
//	{
//		SetMachinePartBSelfCheckCodeBit(SelfCheckCodeGrpHydrgAnaRsvdBit_2);
//	}else {
//        ResetMachinePartBSelfCheckCodeBit(SelfCheckCodeGrpHydrgAnaRsvdBit_2);
//    }
	
	//负压传感器
	Temp = g_u16OriginalDigValue[10] - g_stAnaSigSensorParameter[10].BaseDigValue;
	if(Temp >= 200 || Temp <= -200)
	{
		SetMachinePartBSelfCheckCodeBit(SelfCheckCodeGrpHydrgAnaRsvdBit_3);
	}else {
        ResetMachinePartBSelfCheckCodeBit(SelfCheckCodeGrpHydrgAnaRsvdBit_3);
    }

//	//预留电流型传感器1
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

    //数据滤波算法:滤波值=[(总和-滤波值)+原始值]/采样总数
    for(i = 0; i < BSP_ANA_SENSORS_NMB; i++) {
        if(i != HYDROGEN_PRESS_1) { //电堆气压值不滤波
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
* Arguments   : i_eAnaSigKind --获取的模拟信号类型.
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
				g_fAnaSigAnaValue[NEGATIVE_PRESSURE] = 0.0;//正压数据不显示
			}else if(g_fDigFilteredValue[NEGATIVE_PRESSURE] < 794.375){//异常数据
				g_fAnaSigAnaValue[NEGATIVE_PRESSURE] = 0.0;
			}else{
				//将0-300的值转换为-100到200的值,结果只显示负压
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
