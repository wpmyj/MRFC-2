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
* Filename      : app_analog_signal_monitor_task.c
* Version       : V1.00
* Programmer(s) : JasonFan
*
***************************************************************************************************
*/
/*
***************************************************************************************************
*                                             INCLUDE FILES
***************************************************************************************************/
#define  BSP_MODULE
#include <bsp.h>
#include "app_system_real_time_parameters.h"
#include "app_analog_signal_monitor_task.h"
#include "app_wireness_communicate_task.h"
#include "bsp_speed_adjust_device.h"
#include "app_stack_manager.h"
#include "app_top_task.h"
#include "app_system_run_cfg_parameters.h"
#include "bsp_can.h"
#include "bsp_delay_task_timer.h"
#include "bsp_set_VI_max_value.h"
/*
***************************************************************************************************
*                                           MACRO DEFINITIONS
***************************************************************************************************
*/
#define MX6675_Bits_NUM                 16u

#define ANALOG_TEMP_TO_MAX6675_LSB_RATIO   4.0 //MAX6675的LSB数值与温度的比例常数

#define MAX6675_DATA_CLK_DELAY_SYSTEM_CYCLES    (BSP_CPU_ClkFreq()/10000000 + 1)    //读MAX6675数据时，读取时钟需要保持的系统时钟周期数
#define MAX6675_DATA_READ_DELAY_SYSTEM_CYCLES   ((MAX6675_DATA_CLK_DELAY_SYSTEM_CYCLES + 1) / 2) //读MAX6675数据时，读取时钟就绪后，为保证准确而将读取动作延后的系统时钟数

#define MAX6675_CS_YES   GPIO_ResetBits(GPIOD, BSP_GPIOD_PIN10_MAX6675_CHIPS_SELECT_PORT_NMB);   //MAX6675片选选中
#define MAX6675_CS_NO    GPIO_SetBits(GPIOD, BSP_GPIOD_PIN10_MAX6675_CHIPS_SELECT_PORT_NMB);
#define MAX6675_SCK_UP   GPIO_SetBits(GPIOD, BSP_GPIOD_PIN11_MAX6675_CHIPS_SCLK_PORT_NMB);
#define MAX6675_SCK_DOWN GPIO_ResetBits(GPIOD, BSP_GPIOD_PIN11_MAX6675_CHIPS_SCLK_PORT_NMB);

#define GET_REFORMER_TEMP_DATA_BY_BIT   GPIO_ReadInputDataBit(GPIOD,BSP_GPIOD_PIN12_MAX6675_CHIP_ONE_DIG_SIGNAL_PORT_NMB)
#define GET_FIRE_TEMP_DATA_BY_BIT       GPIO_ReadInputDataBit(GPIOD,BSP_GPIOD_PIN13_MAX6675_CHIP_TWO_DIG_SIGNAL_PORT_NMB)

#define NUMBER_OF_THE_PUMP_SPEED_GRADES     (2000u)
#define DAC_DELT_LSB_PER_PUMP_SPEED         (4095 / ((float)NUMBER_OF_THE_PUMP_SPEED_GRADES))      //    4095/ 200

#define NUMBER_OF_THE_HYDROGEN_FAN_SPEED_GRADES     (2000u)
#define DAC_DELT_LSB_PER_HYDROGEN_FAN_SPEED         (4095 / ((float)NUMBER_OF_THE_HYDROGEN_FAN_SPEED_GRADES))

#define NUMBER_OF_THE_STACK_FAN_SPEED_GRADES        (2000u)
#define TIMER_UPDATE_NUMBER                         (999)

#define TIMER_CMP_DELT_NUMBER_PER_STACK_FAN_SPEED   ((TIMER_UPDATE_NUMBER + 1) / ((float)NUMBER_OF_THE_STACK_FAN_SPEED_GRADES))
#define TIMER_CMP_DELT_NUMBER_PER_HYDROGEN_FAN_SPEED   ((TIMER_UPDATE_NUMBER + 1) / ((float)NUMBER_OF_THE_HYDROGEN_FAN_SPEED_GRADES ))

#define ADC1_DR_Address     ((uint32_t)ADC1_BASE + 0x4C)
#define TIM_CCR1_Address    ((uint32_t)TIM3_BASE + 0x34)
#define TIM_CCR2_Address    ((uint32_t)TIM3_BASE + 0x38)
#define TIM_CCR3_Address    ((uint32_t)TIM3_BASE + 0x3C)
#define TIM_CCR4_Address    ((uint32_t)TIM3_BASE + 0x40)
/*
***************************************************************************************************
*                                         OS-RELATED    VARIABLES
***************************************************************************************************
*/


/*
***************************************************************************************************
*                                           GLOBAL VARIABLES
***************************************************************************************************
*/

/*
***************************************************************************************************
*                                           LOCAL VARIABLES
***************************************************************************************************
*/
CPU_INT32U      BSP_CPU_ClkFreq_MHz;

static		uint8_t 	g_u8PassiveDecompressCnt = 0;
/*
***************************************************************************************************
*                                      LOCAL FUNCTION PROTOTYPES
***************************************************************************************************
*/

static  void  BSP_AnaSensorInit(void);
static  void  BSP_AnaSensorPortsInit(void);
static  void  BSP_AnaSensorADCNVIC_Init(void);
static  void  BSP_AdcNormalConvertStart(vu16 *i_pBuffAddr, uint8_t i_u8ChUsedNmb);

static  void  BSP_AnaSigConvertFinishHandler(void);

static  void  BSP_DigSensorInit(void);

static  void  BSP_SpdCtrlDevicesInit(void);

static  void  BSP_PumpCtrlInit(void);
static  void  BSP_PumpPwrOn(void);
static  void  BSP_PumpPwrOff(void);

static  void  BSP_HydrgFanCtrInit(void);
static  void  BSP_HydrgFanPwrOn(void);
static  void  BSP_HydrgFanPwrOff(void);

static void EXTI15_10_StatusCheck_IRQHandler(void);

static  void  BSP_SwTypePwrDeviceStatuInit(void);
static void BSP_ImpulseInputPortInit(void);

static  void  BSP_StackFanCtrInit(void);
static  void  BSP_StackFanPwrOn(void);
static  void  BSP_StackFanPwrOff(void);

static void   BSP_DeviceSpdCheckPortInit(u16 arr, u16 psc);
static void   BSP_VentingTimeRecordHandler(void);

//static  void  BSP_CheckSwTypeWorkStatuInit(void);
/*
***************************************************************************************************
*                                             REGISTERS
***************************************************************************************************
*/
#define  DWT_CR      *(CPU_REG32 *)0xE0001000
#define  DWT_CYCCNT  *(CPU_REG32 *)0xE0001004
#define  DEM_CR      *(CPU_REG32 *)0xE000EDFC
#define  DBGMCU_CR   *(CPU_REG32 *)0xE0042004

/*
***************************************************************************************************
*                                            REGISTER BITS
***************************************************************************************************
*/
#define  DBGMCU_CR_TRACE_IOEN_MASK       0x10
#define  DBGMCU_CR_TRACE_MODE_ASYNC      0x00
#define  DBGMCU_CR_TRACE_MODE_SYNC_01    0x40
#define  DBGMCU_CR_TRACE_MODE_SYNC_02    0x80
#define  DBGMCU_CR_TRACE_MODE_SYNC_04    0xC0
#define  DBGMCU_CR_TRACE_MODE_MASK       0xC0

#define  DEM_CR_TRCENA                   (1 << 24)

#define  DWT_CR_CYCCNTENA                (1 <<  0)

/*
***************************************************************************************************
*                                     LOCAL CONFIGURATION ERRORS
***************************************************************************************************
*/
/*
#if ((CPU_CFG_TS_TMR_EN          != DEF_ENABLED) && \
     (APP_CFG_PROBE_OS_PLUGIN_EN == DEF_ENABLED) && \
     (OS_PROBE_HOOKS_EN          >  0u))
#error  "CPU_CFG_TS_EN                  illegally #define'd in 'cpu.h'"
#error  "                              [MUST be  DEF_ENABLED] when    "
#error  "                               using uC/Probe COM modules    "
#endif

*/
/*
***************************************************************************************************
*                                               BSP_Init()
*
* Description : Initialize the Board Support Package (BSP).
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : Application.
*
* Note(s)     : (1) This function SHOULD be called before any other BSP function is called.
*
*               (2) CPU instruction / data tracing requires the use of the following pins :
*                   (a) (1) Aysynchronous     :  PB[3]
*                       (2) Synchronous 1-bit :  PE[3:2]
*                       (3) Synchronous 2-bit :  PE[4:2]
*                       (4) Synchronous 4-bit :  PE[6:2]
*
*                   (b) The uC-Eval board MAY utilize the following pins depending on the application :
*                       (1) PE[5], MII_INT
*                       (1) PE[6], SDCard_Detection
*
*                   (c) The application may wish to adjust the trace bus width depending on I/O
*                       requirements.
***************************************************************************************************
*/

void  BSP_Init(void)
{
    ErrorStatus HSEStartUpStatus;

    BSP_IntInit();

    RCC_DeInit();
    RCC_HSEConfig(RCC_HSE_ON);
    HSEStartUpStatus = RCC_WaitForHSEStartUp();

    //HSE起振成功
    if(HSEStartUpStatus == SUCCESS) {

        RCC_PREDIV2Config(RCC_PREDIV2_Div2);                        /* Fprediv2 = HSE      /  2 =  4MHz.                    */
        RCC_PLL2Config(RCC_PLL2Mul_10);                             /* PLL2     = Fprediv2 *  10 = 40MHz.                    */
        RCC_PLL2Cmd(ENABLE);
        RCC_PLL3Config(RCC_PLL3Mul_10);                             /* PLL3     = Fprediv2 * 10 = 40MHz.                    */
        RCC_PLL3Cmd(ENABLE);

        RCC_HCLKConfig(RCC_SYSCLK_Div1);                            /* HCLK    = AHBCLK  = PLL / AHBPRES(1) = 72MHz.       */
        RCC_PCLK2Config(RCC_HCLK_Div1);                             /* APB2CLK = AHBCLK  / APB2DIV(1)  = 72MHz.             */
        RCC_PCLK1Config(RCC_HCLK_Div2);                             /* APB1CLK = AHBCLK  / APB1DIV(2)  = 36MHz (max).       */
        RCC_ADCCLKConfig(RCC_PCLK2_Div6);                           /* ADCCLK  = AHBCLK  / APB2DIV / 6 = 12MHz.             */
        RCC_OTGFSCLKConfig(RCC_OTGFSCLKSource_PLLVCO_Div3);        /* OTGCLK  = PLLVCO / USBPRES(3)  = 144MHz / 3 = 48MHz */

        FLASH_SetLatency(FLASH_Latency_2);                          /* 设置FLASH延时周期数为2,when HCLK > 48MHz.*/
        FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);        /*使能预取缓存*/

        while(RCC_GetFlagStatus(RCC_FLAG_PLL2RDY) == RESET) {       /* Wait for PLL2 to lock.                               */
            ;
        }

        while(RCC_GetFlagStatus(RCC_FLAG_PLL3RDY) == RESET) {       /* Wait for PLL3 to lock.                               */
            ;
        }

        /* Fprediv1 = PLL2 / 5 =  8MHz.                         */
        RCC_PREDIV1Config(RCC_PREDIV1_Source_PLL2, RCC_PREDIV1_Div5);
        RCC_PLLConfig(RCC_PLLSource_PREDIV1, RCC_PLLMul_9);      /* PLL1 = Fprediv1 * 9 = 72Mhz.                         */
        RCC_PLLCmd(ENABLE);

        while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET) {       /* Wait for PLL1 to lock.                               */
            ;
        }

        RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);                 /* HCLK = SYSCLK = PLL = 72MHz.                        */

        while(RCC_GetSYSCLKSource() != 0x08) {
            ;
        }
    }

    BSP_CPU_ClkFreq_MHz = BSP_CPU_ClkFreq() / (CPU_INT32U)1000000;
    BSP_CPU_ClkFreq_MHz = BSP_CPU_ClkFreq_MHz;                  /* Surpress compiler warning BSP_CPU_ClkFreq_MHz    ... */
    /* ... set and not used.                                */

    BSP_AnaSensorInit();            // 模拟传感器初始化
    BSP_DigSensorInit();            // 数字传感器初始化——K型热电偶适配的MAX6675
    BSP_SpdCtrlDevicesInit();       // 调速设备
    BSP_DeviceSpdCheckPortInit(5000 - 1, 7200 - 1); //测速引脚初始化
    BSP_SwTypePwrDeviceStatuInit(); // 开关型输出设备
    BSP_CmdButtonInit();            // 硬件按钮
    BSP_ImpulseInputPortInit();     //外部脉冲输入引脚初始化
    CAN1_Init();                    //CAN总线配置，波特率50K
	DelayQueueTimerInit();
	Bsp_Usart3Init();				//初始化本地限流端口
//    AT25256B_Init();              //外部EEPROM初始化

    BSP_VentingIntervalRecordTimerInit();//电堆排气时间参数定时器初始化

#ifdef TRACE_EN                                                 /* See project / compiler preprocessor options.         */
    DBGMCU_CR |=  DBGMCU_CR_TRACE_IOEN_MASK;                    /* Enable tracing (see Note #2).                        */
    DBGMCU_CR &= ~DBGMCU_CR_TRACE_MODE_MASK;                    /* Clr trace mode sel bits.                             */
    DBGMCU_CR |=  DBGMCU_CR_TRACE_MODE_SYNC_04;                 /* Cfg trace mode to synch 4-bit.                       */
#endif
}


/*
***************************************************************************************************
*                                            BSP_CPU_ClkFreq()
*
* Description : Read CPU registers to determine the CPU clock frequency of the chip.
*
* Argument(s) : none.
*
* Return(s)   : The CPU clock frequency, in Hz.
*
* Caller(s)   : Application.
*
* Note(s)     : none.
***************************************************************************************************
*/

CPU_INT32U  BSP_CPU_ClkFreq(void)
{
    RCC_ClocksTypeDef  rcc_clocks;


    RCC_GetClocksFreq(&rcc_clocks);

    return ((CPU_INT32U)rcc_clocks.HCLK_Frequency);
}

/*
***************************************************************************************************
***************************************************************************************************
*                                      ANALOG INPUT SERSOR FUNCTIONS
***************************************************************************************************
***************************************************************************************************
*/
/*
***************************************************************************************************
*                                        BSP_AnaSensorInit ()
*
* Description : Initialize the sensor.
*
* Argument(s) : none.
*
* Return(s)   :
*
* Caller(s)   : Application
*
* Note(s)     : none.
***************************************************************************************************
*/
static void  BSP_AnaSensorInit(void)
{
    BSP_AnaSensorPortsInit();       //模拟量采集引脚初始化
    BSP_AnaSensorADCNVIC_Init();    // ADC DMA1channel1中断优先级配置
}

/*
***************************************************************************************************
*                                             BSP_AnaSensorPortsInit()
*
* Description : Initialize the signal Port(s) for the analog sersor(s).
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : BSP_Init().
*
* Note(s)     : none.
***************************************************************************************************
*/
static  void  BSP_AnaSensorPortsInit(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE);//时钟使能
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_ADC1, ENABLE);

    GPIO_InitStructure.GPIO_Pin   = BSP_GPIOA_PIN2_STACK_TEMP_ANA_SIGNAL_PORT_NMB;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin  = BSP_GPIOB_STACK_VOLETAGE_ANA_SIGNAL_PORT_NMB;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AIN;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin   = BSP_GPIOA_PIN3_STACK_CURRENT_ANA_SIGNAL_PORT_NMB;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin   = BSP_GPIOA_PIN1_LIQUID_PRESS_ANA_SIGNAL_PORT_NMB;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin   = BSP_GPIOB_STACK_HYDROGEN_PRESS_ONE_ANA_SIGNAL_PORT_NMB;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AIN;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin   = BSP_GPIOA_PIN7_STACK_HYDROGEN_PRESS_TWO_ANA_SIGNAL_PORT_NMB;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin   = BSP_GPIOA_PIN0_LIQUID_LEVEL_ANA_SIGNAL_PORT_NMB;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin   = BSP_GPIOA_PIN6_BATTERY_VOLETAGE_ANA_SIGNAL_PORT_NMB;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin   = BSP_GPIOC_HYDROGRN_CONCENTRATION_DETECTION_PORT_NMB;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AIN;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin   = BSP_GPIOC_BATTERY_CURRENT_ANA_SIGNAL_PORT_NMB;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AIN;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin   = BSP_GPIOC_RAPID_HEATER_CURRETN_ANA_SIGNAL_PORT_NMB;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AIN;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin   = BSP_GPIOC_NEGATIVE_PRESSURE_ANA_SIGNAL_PORT_NMB;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AIN;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin   = BSP_GPIOC_RSVD1_ANA_SIGNAL_PORT_NMB;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AIN;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin   = BSP_GPIOC_RSVD2_ANA_SIGNAL_PORT_NMB;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AIN;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}

void BSP_AnaSensorADCNVIC_Init(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Configure and enable DMA1_Channel1 interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}
/*
***************************************************************************************************
*                                             BSP_AnaSensorConvertStart()()
*
* Description : Start the conversion of the analog sensor(s).
*
* Argument(s) : *i_pBuffAddr is the head address of the array that to store the value after analog to digital convert.
*               i_u8ChUsedNmb is the number of the channels that has been used in the project.
*
* Return(s)   : none.
*
* Caller(s)   : Application.
*
* Note(s)     : none.
***************************************************************************************************
*/
void BSP_AnaSensorConvertStart(vu16 *i_pBuffAddr, uint8_t i_u8ChUsedNmb)
{
    BSP_AdcNormalConvertStart(i_pBuffAddr, i_u8ChUsedNmb);
}
/*
***************************************************************************************************
*                                             BSP_AdcNormalConvertStart()()
*
* Description : Start the work of ADC.
*
* Argument(s) : *i_pBuffAddr is the head address of the array that to store the value after analog to digital convert.
*               i_u8ChUsedNmb: the number of the adc conver channels .
*
* Return(s)   : none.
*
* Caller(s)   : BSP_AnaSensorConvertStart().
*
* Note(s)     : none.
***************************************************************************************************
*/
static void BSP_AdcNormalConvertStart(vu16 *i_pBuffAddr, uint8_t i_u8ChUsedNmb)
{
    ADC_InitTypeDef ADC_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);  //使能DMA1时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_ADC1, ENABLE);
    RCC_ADCCLKConfig(RCC_PCLK2_Div6);   //设置ADC分频因子6 72M/6=12,ADC时钟最大频率不能超过14M

    /* DMA1 channel1 configuration ----------------------------------------------*/
    DMA_DeInit(DMA1_Channel1);
    DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)i_pBuffAddr;   //DMA外设基地址(_u16OriginalDigValue)
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;              //外设作为数据传输来源
    DMA_InitStructure.DMA_BufferSize = i_u8ChUsedNmb;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;//数据宽度为16位
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel1, &DMA_InitStructure);
    /* Enable DMA1 channel1 */
    DMA_Cmd(DMA1_Channel1, ENABLE);
    DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);
    BSP_IntVectSet(BSP_INT_ID_DMA1_CH1, BSP_AnaSigConvertFinishHandler);//AD装换完成信号量发送
    BSP_IntEn(BSP_INT_ID_DMA1_CH1);

    /* ADC1 configuration ------------------------------------------------------*/
    ADC_DeInit(ADC1);
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;          //独立工作模式
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;                //允许扫描——多通道
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;         //非连续转换模式
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;//转换由软件而不是外部触发启动
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;      //数据右对齐
    ADC_InitStructure.ADC_NbrOfChannel = i_u8ChUsedNmb;
    ADC_Init(ADC1, &ADC_InitStructure);
    /*设置规则组通道、采样顺序、采样时间 */
    ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 1, ADC_SampleTime_239Cycles5);    // 电堆温度
    ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 2, ADC_SampleTime_239Cycles5);    // 电堆电压
    ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 3, ADC_SampleTime_239Cycles5);    // 电堆电流
    ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 4, ADC_SampleTime_239Cycles5);    // 液压
    ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 5, ADC_SampleTime_239Cycles5);    // 气压1
    ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 6, ADC_SampleTime_239Cycles5);    // 气压2
    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 7, ADC_SampleTime_239Cycles5);    // 液位
    ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 8, ADC_SampleTime_239Cycles5);    // 电池电压
    ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 9, ADC_SampleTime_239Cycles5);    // 氢气浓度
    ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 10, ADC_SampleTime_239Cycles5);    // 电池电流
    ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 11, ADC_SampleTime_239Cycles5);    // 快速加热器电流
    ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 12, ADC_SampleTime_239Cycles5);    // 负压传感器
    ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 13, ADC_SampleTime_239Cycles5);    // 预留电流型传感器
    ADC_RegularChannelConfig(ADC1, ADC_Channel_15, 14, ADC_SampleTime_239Cycles5);    // 预留电流型传感器

    /* Enable ADC1 DMA */
    ADC_DMACmd(ADC1, ENABLE);

    /* Enable ADC1 */
    ADC_Cmd(ADC1, ENABLE);

    /* Enable ADC1 reset calibration register */
    ADC_ResetCalibration(ADC1);

    /* get the status of ADC1 reset calibration register */
    while(ADC_GetResetCalibrationStatus(ADC1));

    /* Start ADC1 calibration */
    ADC_StartCalibration(ADC1);

    /* Check the end of ADC1 calibration */
    while(ADC_GetCalibrationStatus(ADC1));  //等待校准结果

    ADC_SoftwareStartConvCmd(ADC1, ENABLE); //使能软件转换启动功能
}
/*
***************************************************************************************************
*                                        BSP_AnaSigConvertFinishHandlerDummy()
*
* Description : Send post that the adc conversion is finish to the task.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : BSP_IntHandler().
*
* Note(s)     : none.
***************************************************************************************************
*/
void BSP_AnaSigConvertFinishHandler(void)
{
    OS_ERR      err;

    if(DMA_GetITStatus(DMA1_IT_TC1) != RESET) {
        //发送信号量，告知ADC的DMA转换已经完成
        OSSemPost(&g_stAnaSigConvertFinishSem,
                  OS_OPT_POST_1,
                  &err);
        DMA_ClearITPendingBit(DMA1_IT_TC1);
    }
}
/*
***************************************************************************************************
***************************************************************************************************
*                                      DIGITAL INPUT SERSOR FUNCTIONS
***************************************************************************************************
***************************************************************************************************
*/
/*
***************************************************************************************************
*                                        BSP_DigSensorInit ()
*
* Description : Initialize the ports of the MCU that connect to MAX6675 which is between the MCU and the K-type thermocouple sensor(s).
*               初始化芯片与热电偶连接引脚
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : BSP_Init().
*
* Note(s)     : none.
***************************************************************************************************
*/

static void  BSP_DigSensorInit(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOB , ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);    //使能功能复用IO
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);    //SWJ调试接口完全失能

    /******配置GPIOB口引脚*********/
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Pin = BSP_GPIOD_PIN13_MAX6675_CHIP_TWO_DIG_SIGNAL_PORT_NMB | BSP_GPIOD_PIN12_MAX6675_CHIP_ONE_DIG_SIGNAL_PORT_NMB;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin = BSP_GPIOD_PIN10_MAX6675_CHIPS_SELECT_PORT_NMB | BSP_GPIOD_PIN11_MAX6675_CHIPS_SCLK_PORT_NMB;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOD, BSP_GPIOD_PIN10_MAX6675_CHIPS_SELECT_PORT_NMB | BSP_GPIOD_PIN11_MAX6675_CHIPS_SCLK_PORT_NMB);
}
/*
***************************************************************************************************
*                                             DigTempSensorConvertStart()
*
* Description : Start the conversion of the Digital temperature sensor(s).
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : Application.
*
* Note(s)     : none.
***************************************************************************************************
*/
void DigTempSensorConvertStart()
{
    BSP_MAX6675ConvertStart();
}
/*
***************************************************************************************************
*                                             BSP_MAX6675ConvertStart()()
*
* Description : Start the conversion of the MAX6675.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : DigTempSensorConvertStart().
*
* Note(s)     : none.
***************************************************************************************************
*/
void BSP_MAX6675ConvertStart(void)
{
    MAX6675_CS_NO;
}

/*
***************************************************************************************************
*                                             BSP_MAX6675_Temp_Read()()
*
* Description : Get the convert values of the MAX6675.
*
* Argument(s) :  *i_fTemp   存放温度数据的数组
*                *err       存放错误标志的数组
* Return(s)   : unsigned int16.
*
* Caller(s)   : Applition.
*
* Note(s)     : none.
***************************************************************************************************
*/
void BSP_MAX6675_Temp_Read(float *i_fTemp, uint8_t *err)
{
    int i, j;
    uint16_t m_temp[2] = {0};

    MAX6675_CS_YES;

    for(i = 0; i < MAX6675_DATA_CLK_DELAY_SYSTEM_CYCLES; i++);      // 由于片选选中后，输入的MX6675的时钟需要滞后100ns，所以在此空延时以满足时间要求

    for(j = 0; j < MX6675_Bits_NUM; j++) {
        MAX6675_SCK_UP;

        for(i = 0; i < MAX6675_DATA_READ_DELAY_SYSTEM_CYCLES; i++); // 因MAX6675时钟电平维持时间不得短于100ns，所以在读取动作前后各空延时几个周期，保证数据准确性

        m_temp[0] = (m_temp[0] << 1) | GET_REFORMER_TEMP_DATA_BY_BIT;
        m_temp[1] = (m_temp[1] << 1) | GET_FIRE_TEMP_DATA_BY_BIT;

        for(i = 0; i < MAX6675_DATA_READ_DELAY_SYSTEM_CYCLES; i++);

        MAX6675_SCK_DOWN;

        for(i = 0; i < MAX6675_DATA_CLK_DELAY_SYSTEM_CYCLES; i++);
    }

    MAX6675_CS_NO;

    if((m_temp[0] & (1 << 15 | 4)) == 0) { //数据正确且热电偶均未断线
        m_temp[0] &= 0x7FFF;
        m_temp[0] >>= 3;
        *err = 0;
    } else {
        m_temp[0] = 0xF9C;      //3996
        *err = 1;
    }

    if((m_temp[1] & (1 << 15 | 4)) == 0) { //数据正确且热电偶均未断线
        m_temp[1] &= 0x7FFF;
        m_temp[1] >>= 3;
        *(err + 1) = 0;
    } else {
        m_temp[1] = 0xF9C;
        *(err + 1) = 1;
    }

    *i_fTemp = ((float) m_temp[0] / ANALOG_TEMP_TO_MAX6675_LSB_RATIO);
    *(i_fTemp + 1) = ((float) m_temp[1] / ANALOG_TEMP_TO_MAX6675_LSB_RATIO);
}
/*
***************************************************************************************************
***************************************************************************************************
*                                      SWITCH OUTPUT DEVICE FUNCTIONS
***************************************************************************************************
***************************************************************************************************
*/
/*
***************************************************************************************************
*                                             BSP_BuzzerOn()
*
* Description : Turn ON the buzzer on the board.
*
* Argument(s) : none.
*
* Note(s)     : none.
***************************************************************************************************
*/

void  BSP_BuzzerOn()
{
    GPIO_SetBits(GPIOB, BSP_GPIOB_BUZZER_CTRL_PORT_NMB);
}

void  BSP_BuzzerOff()
{
    GPIO_ResetBits(GPIOB, BSP_GPIOB_BUZZER_CTRL_PORT_NMB);
}
//蜂鸣器反转
void  BSP_BuzzerTurnover()
{
    GPIO_ReadOutputDataBit(GPIOB, BSP_GPIOB_BUZZER_CTRL_PORT_NMB)\
    ? GPIO_ResetBits(GPIOB, BSP_GPIOB_BUZZER_CTRL_PORT_NMB)\
    : GPIO_SetBits(GPIOB, BSP_GPIOB_BUZZER_CTRL_PORT_NMB);
}

/*
***************************************************************************************************
*                                              BSP_SwTypePwrDeviceStatuInit()
*
*Description : Initialize the switch-type power device control port(s)
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : BSP_Init()
*
* Note(s)     : none.
***************************************************************************************************
*/
static void BSP_SwTypePwrDeviceStatuInit(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA
                           | RCC_APB2Periph_GPIOB
                           | RCC_APB2Periph_GPIOC
                           | RCC_APB2Periph_GPIOD
                           | RCC_APB2Periph_GPIOE
                           , ENABLE);//使能所有GPIO口时钟

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);//部分引脚占用了SW调试口或JTAG口，需要将其先关闭

    GPIO_InitStructure.GPIO_Pin   = BSP_GPIOB_BUZZER_CTRL_PORT_NMB;//蜂鸣器
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOB, BSP_GPIOB_BUZZER_CTRL_PORT_NMB);

    GPIO_InitStructure.GPIO_Pin   = BSP_GPIOB_LIQUID_INPUT_VALVE_ONE_PWR_CTRL_PORT_NMB;  // 进液电磁阀1控制引脚
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOB, BSP_GPIOB_LIQUID_INPUT_VALVE_ONE_PWR_CTRL_PORT_NMB);

    GPIO_InitStructure.GPIO_Pin   = BSP_GPIOC_LIQUID_INPUT_VALVE_TWO_PWR_CTRL_PORT_NMB;  // 进液电磁阀2控制引脚
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOC, BSP_GPIOC_LIQUID_INPUT_VALVE_TWO_PWR_CTRL_PORT_NMB);

    GPIO_InitStructure.GPIO_Pin   = BSP_GPIOC_IGNITER_PWR_CTRL_PORT_NMB;             // 点火器控制引脚
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOC, BSP_GPIOC_IGNITER_PWR_CTRL_PORT_NMB);

    GPIO_InitStructure.GPIO_Pin   = BSP_GPIOC_FAST_HEATER_PWR_CTRL_PORT_NMB;              // 快速加热器控制引脚
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOC, BSP_GPIOC_FAST_HEATER_PWR_CTRL_PORT_NMB);

    GPIO_InitStructure.GPIO_Pin   = BSP_GPIOC_HYDROGEN_INTO_STACK_VALVE_PWR_CTRL_PORT_NMB; // 电堆进气阀控制引脚
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOC, BSP_GPIOC_HYDROGEN_INTO_STACK_VALVE_PWR_CTRL_PORT_NMB);

    GPIO_InitStructure.GPIO_Pin   = BSP_GPIOC_HYDROGEN_OUTOF_STACK_VALVE_PWR_CTRL_PORT_NMB;// 电堆出气阀控制引脚
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOC, BSP_GPIOC_HYDROGEN_OUTOF_STACK_VALVE_PWR_CTRL_PORT_NMB);

    GPIO_InitStructure.GPIO_Pin   = BSP_GPIOB_DC_CONNECTER_PWR_CTRL_PORT_NMB;             // 直流接触器控制引脚
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOB, BSP_GPIOB_DC_CONNECTER_PWR_CTRL_PORT_NMB);

    PWR_BackupAccessCmd(ENABLE);//允许修改RTC 和后备寄存器
    RCC_LSEConfig(RCC_LSE_OFF);//关闭外部低速外部时钟信号功能 后，PC13 PC14 PC15 才可以当普通IO用。
    BKP_TamperPinCmd(DISABLE);//关闭入侵检测功能，也就是 PC13，也可以当普通IO 使用
    PWR_BackupAccessCmd(DISABLE);//禁止修改后备寄存器

    GPIO_InitStructure.GPIO_Pin   = BSP_GPIOC_RSVD_OUTPUT_PWR_CTRL_PORT_NMB;             // 自动加液外置甲醇泵
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOC, BSP_GPIOC_RSVD_OUTPUT_PWR_CTRL_PORT_NMB);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin =  BSP_GPIOD_PIN7_RSVD8_OUTPUT_PWR_CTRL_PORT_NMB ;       //预留输出8
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOD,  BSP_GPIOD_PIN7_RSVD8_OUTPUT_PWR_CTRL_PORT_NMB);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin =  BSP_GPIOD_PIN14_RSVD6_OUTPUT_PWR_CTRL_PORT_NMB ;      //预留输出6
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOD,  BSP_GPIOD_PIN14_RSVD6_OUTPUT_PWR_CTRL_PORT_NMB);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = BSP_GPIOD_PIN15_RSVD7_OUTPUT_PWR_CTRL_PORT_NMB  ; //预留输出7
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOD,  BSP_GPIOD_PIN15_RSVD7_OUTPUT_PWR_CTRL_PORT_NMB);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin =  BSP_GPIOE_PIN14_RSVD5_OUTPUT_PWR_CTRL_PORT_NMB;    //预留输出5
    GPIO_Init(GPIOE, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOE, BSP_GPIOE_PIN14_RSVD5_OUTPUT_PWR_CTRL_PORT_NMB);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin =  BSP_GPIOE_PIN15_RSVD4_OUTPUT_PWR_CTRL_PORT_NMB;    //预留输出4
    GPIO_Init(GPIOE, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOE, BSP_GPIOE_PIN15_RSVD4_OUTPUT_PWR_CTRL_PORT_NMB);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin =  BSP_GPIOE_PIN8_RSVD3_OUTPUT_PWR_CTRL_PORT_NMB;    //预留输出3
    GPIO_Init(GPIOE, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOE, BSP_GPIOE_PIN8_RSVD3_OUTPUT_PWR_CTRL_PORT_NMB);

    GPIO_InitStructure.GPIO_Pin   = BSP_GPIOB_RSVD2_OUTPUT_PWR_CTRL_PORT_NMB; // 预留2控制引脚
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOB, BSP_GPIOB_RSVD2_OUTPUT_PWR_CTRL_PORT_NMB);
}
/*
***************************************************************************************************
*                                            BSP_LqdValve1_PwrOn()
*
* Description : Open the power switch of the liquid valve one.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Note(s)     : none.
***************************************************************************************************
*/
void  BSP_LqdValve1_PwrOn(void)
{
    GPIO_SetBits(GPIOB, BSP_GPIOB_LIQUID_INPUT_VALVE_ONE_PWR_CTRL_PORT_NMB);
}

void  BSP_LqdValve1_PwrOff(void)
{
    GPIO_ResetBits(GPIOB, BSP_GPIOB_LIQUID_INPUT_VALVE_ONE_PWR_CTRL_PORT_NMB);
}
/*
***************************************************************************************************
*                                            BSP_LqdValve2_PwrOn()
*
* Description : Open the power switch of the liquid valve two.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Note(s)     : none.
***************************************************************************************************
*/
void  BSP_LqdValve2_PwrOn(void)
{
    GPIO_SetBits(GPIOC, BSP_GPIOC_LIQUID_INPUT_VALVE_TWO_PWR_CTRL_PORT_NMB);
}

void  BSP_LqdValve2_PwrOff(void)
{
    GPIO_ResetBits(GPIOC, BSP_GPIOC_LIQUID_INPUT_VALVE_TWO_PWR_CTRL_PORT_NMB);
}
/*
***************************************************************************************************
*                                            BSP_IgniterPwrOn()
*
* Description : Open the power switch of the igniter.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : application.
*
* Note(s)     : none.
***************************************************************************************************
*/
void  BSP_IgniterPwrOn(void)
{
    GPIO_SetBits(GPIOC, BSP_GPIOC_IGNITER_PWR_CTRL_PORT_NMB);
    SetSystemRunningStatuCodeBit(RuningStatuCodeGrpHydrgOut2_Bit);
}

void  BSP_IgniterPwrOff(void)
{
    GPIO_ResetBits(GPIOC, BSP_GPIOC_IGNITER_PWR_CTRL_PORT_NMB);
    ResetSystemRunningStatuCodeBit(RuningStatuCodeGrpHydrgOut2_Bit);
}

/*
***************************************************************************************************
*                                            BSP_FastHeaterPwrOn()
*
* Description : Open the power switch of the heater.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Note(s)     : none.
***************************************************************************************************
*/
void  BSP_FastHeaterPwrOn(void)
{
    SetSystemRunningStatuCodeBit(RuningStatuCodeGrpHydrgOut1_Bit);
    GPIO_SetBits(GPIOC, BSP_GPIOC_FAST_HEATER_PWR_CTRL_PORT_NMB);
}

void  BSP_FastHeaterPwrOff(void)
{
    GPIO_ResetBits(GPIOC, BSP_GPIOC_FAST_HEATER_PWR_CTRL_PORT_NMB);
    ResetSystemRunningStatuCodeBit(RuningStatuCodeGrpHydrgOut1_Bit);
}

/*
***************************************************************************************************
*                                            BSP_HydrgInValvePwrOn()
*
* Description : Open the power switch of the hydrogen input valve.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Note(s)     : none.
***************************************************************************************************
*/
void  BSP_HydrgInValvePwrOn(void)
{
    GPIO_SetBits(GPIOC, BSP_GPIOC_HYDROGEN_INTO_STACK_VALVE_PWR_CTRL_PORT_NMB);
}

void  BSP_HydrgInValvePwrOff(void)
{
    GPIO_ResetBits(GPIOC, BSP_GPIOC_HYDROGEN_INTO_STACK_VALVE_PWR_CTRL_PORT_NMB);
}
/*
***************************************************************************************************
*                                            BSP_HydrgOutValvePwrOn()
*
* Description : Open the power switch of the hydrogen output valve.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Note(s)     : none.
***************************************************************************************************
*/
void  BSP_HydrgOutValvePwrOn(void)
{
    GPIO_SetBits(GPIOC, BSP_GPIOC_HYDROGEN_OUTOF_STACK_VALVE_PWR_CTRL_PORT_NMB);
}

void  BSP_HydrgOutValvePwrOff(void)
{
    GPIO_ResetBits(GPIOC, BSP_GPIOC_HYDROGEN_OUTOF_STACK_VALVE_PWR_CTRL_PORT_NMB);
}
/*
***************************************************************************************************
*                                            BSP_DCConnectValvePwrOn()
*
* Description : Open the power switch of the DC connect.

* Argument(s) : none.
*
* Return(s)   : none.
*
* Note(s)     : none.
***************************************************************************************************
*/
void  BSP_DCConnectValvePwrOn(void)
{
    APP_TRACE_INFO(("Dc connect power on...\n\r"));
    GPIO_SetBits(GPIOB, BSP_GPIOB_DC_CONNECTER_PWR_CTRL_PORT_NMB);
}

void  BSP_DCConnectValvePwrOff(void)
{
    APP_TRACE_INFO(("Dc connect power off...\n\r"));
    GPIO_ResetBits(GPIOB, BSP_GPIOB_DC_CONNECTER_PWR_CTRL_PORT_NMB);
}

/*
***************************************************************************************************
*                                            BSP_OutsidePumpPwrOn()
*
* Description : Open the power switch of the outside pump.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : application.
*
* Note(s)     : none.
***************************************************************************************************
*/
void  BSP_OutsidePumpPwrOn(void)
{

    GPIO_SetBits(GPIOC, BSP_GPIOC_RSVD_OUTPUT_PWR_CTRL_PORT_NMB);
}

void  BSP_OutsidePumpPwrOff(void)
{

    GPIO_ResetBits(GPIOC, BSP_GPIOC_RSVD_OUTPUT_PWR_CTRL_PORT_NMB);
}

/*
***************************************************************************************************
*                                          电堆短路活化控制引脚
*
* Description :
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : application.
*
* Note(s)     : none.
***************************************************************************************************
*/
void  BSP_StackShortCircuitActivationOn(void)
{
    GPIO_SetBits(GPIOE, BSP_GPIOE_PIN15_RSVD4_OUTPUT_PWR_CTRL_PORT_NMB);
    APP_TRACE_INFO(("Stack Short Circuit Activation On...\n\r"));
}

void  BSP_StackShortCircuitActivationOff(void)
{
    GPIO_ResetBits(GPIOE, BSP_GPIOE_PIN15_RSVD4_OUTPUT_PWR_CTRL_PORT_NMB);
    APP_TRACE_INFO(("Stack Short Circuit Activation Off...\n\r"));
}
/*
***************************************************************************************************
*                                           预留5/6/7/8口开关
*
* Description :
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : application.
*
* Note(s)     : none.
***************************************************************************************************
*/

void  BSP_TailGasOutValvePwrOn(void)
{
    GPIO_SetBits(GPIOE, BSP_GPIOE_PIN8_RSVD3_OUTPUT_PWR_CTRL_PORT_NMB);
    APP_TRACE_INFO(("Tail Gas Out Valve power on...\n\r"));
}

void  BSP_TailGasOutValvePwrOff(void)
{
    GPIO_ResetBits(GPIOE, BSP_GPIOE_PIN8_RSVD3_OUTPUT_PWR_CTRL_PORT_NMB);
    APP_TRACE_INFO(("Tail Gas Out Valve power off...\n\r"));
}

void  BSP_PureHydrogenGasOutValvePwrOn(void)
{
    GPIO_SetBits(GPIOB, BSP_GPIOB_RSVD2_OUTPUT_PWR_CTRL_PORT_NMB);
    APP_TRACE_INFO(("Pure Hydrogen Gas Out Valve power on...\n\r"));
}
void  BSP_PureHydrogenGasOutValvePwrOff(void)
{
    GPIO_ResetBits(GPIOB, BSP_GPIOB_RSVD2_OUTPUT_PWR_CTRL_PORT_NMB);
    APP_TRACE_INFO(("Pure Hydrogen Gas Out Valve power off...\n\r"));
}

void  BSP_ShortProtectValvePwrOn(void)
{
    GPIO_SetBits(GPIOE, BSP_GPIOE_PIN14_RSVD5_OUTPUT_PWR_CTRL_PORT_NMB);
    APP_TRACE_INFO(("Bsp Short Protect Valve power on...\n\r"));
}
void  BSP_ShortProtectValvePwrOff(void)
{
    GPIO_ResetBits(GPIOE, BSP_GPIOE_PIN14_RSVD5_OUTPUT_PWR_CTRL_PORT_NMB);
    APP_TRACE_INFO(("Bsp Short Protect Valve power off...\n\r"));
}

//void  BSP_MakeVavuumValve2PwrOn(void)
//{
//    GPIO_SetBits(GPIOE, BSP_GPIOE_PIN14_RSVD5_OUTPUT_PWR_CTRL_PORT_NMB);
//    APP_TRACE_INFO(("Bsp Make Vavuum Valve2 power on...\n\r"));
//}
//void  BSP_MakeVavuumValve2PwrOff(void)
//{
//    GPIO_ResetBits(GPIOE, BSP_GPIOE_PIN14_RSVD5_OUTPUT_PWR_CTRL_PORT_NMB);
//    APP_TRACE_INFO(("Bsp Make Vavuum Valve2 power off...\n\r"));
//}

void  BSP_MakeVavuumValve3PwrOn(void)
{
    GPIO_SetBits(GPIOD, BSP_GPIOD_PIN14_RSVD6_OUTPUT_PWR_CTRL_PORT_NMB);
    APP_TRACE_INFO(("Bsp Make Vavuum Valve3 power on...\n\r"));
}
void  BSP_MakeVavuumValve3PwrOff(void)
{
    GPIO_ResetBits(GPIOD, BSP_GPIOD_PIN14_RSVD6_OUTPUT_PWR_CTRL_PORT_NMB);
    APP_TRACE_INFO(("Bsp Make Vavuum Valve3 power off...\n\r"));
}

void  BSP_DcOutPutValvePwrOn(void)
{
    GPIO_SetBits(GPIOD, BSP_GPIOD_PIN15_RSVD7_OUTPUT_PWR_CTRL_PORT_NMB);
    APP_TRACE_INFO(("Dc Out Put Valve power on...\n\r"));
}

void  BSP_DcOutPutValvePwrOff(void)
{
    GPIO_ResetBits(GPIOD, BSP_GPIOD_PIN15_RSVD7_OUTPUT_PWR_CTRL_PORT_NMB);
    APP_TRACE_INFO(("Dc Out Put Valve power off...\n\r"));
}

//void  BSP_MakeVavuumValve4PwrOn(void)
//{
//    GPIO_SetBits(GPIOD, BSP_GPIOD_PIN15_RSVD7_OUTPUT_PWR_CTRL_PORT_NMB);
//    APP_TRACE_INFO(("Bsp Make Vavuum Valve4 power on...\n\r"));
//}

//void  BSP_MakeVavuumValve4PwrOff(void)
//{
//    GPIO_ResetBits(GPIOD, BSP_GPIOD_PIN15_RSVD7_OUTPUT_PWR_CTRL_PORT_NMB);
//    APP_TRACE_INFO(("Bsp Make Vavuum Valve4 power off...\n\r"));
//}

void  BSP_MakeVavuumPumpPwrOn(void)
{
    GPIO_SetBits(GPIOD, BSP_GPIOD_PIN7_RSVD8_OUTPUT_PWR_CTRL_PORT_NMB);
    APP_TRACE_INFO(("Bsp Make Vavuum pump power on...\n\r"));
}

void  BSP_MakeVavuumPumpPwrOff(void)
{
    GPIO_ResetBits(GPIOD, BSP_GPIOD_PIN7_RSVD8_OUTPUT_PWR_CTRL_PORT_NMB);
    APP_TRACE_INFO(("Bsp Make Vavuum pump power off...\n\r"));
}

/*
***************************************************************************************************
*                                            BSP_VentingIntervalRecordTimerInit()
*
* Description : 初始化电堆排气时间记录硬件定时器.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Note(s)     : none.
***************************************************************************************************
*/
void BSP_VentingIntervalRecordTimerInit(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

    TIM_DeInit(TIM5);
    /* 时基参数配置*/
    TIM_TimeBaseStructure.TIM_Period = 10 - 1;     //重载值:当定时器从0计数到10，1ms产生一次更新事件
    TIM_TimeBaseStructure.TIM_Prescaler = 7199;     //设置预分频系数 72MHz/7200 = 10KHz
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //向上计数模式

    TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);

    TIM_ARRPreloadConfig(TIM5, ENABLE);                 // 使能TIM5重载寄存器
    TIM_GenerateEvent(TIM5, TIM_EventSource_Update);    // 定时器事件由软件更新事件产生，立即更新数据
    TIM_ClearFlag(TIM5, TIM_FLAG_Update);               //清除标志位。定时器一打开便产生更新事件，若不清除，将会直接进入中断
    TIM_ITConfig(TIM5, TIM_IT_Update, DISABLE);         //禁能更新中断

    BSP_IntVectSet(BSP_INT_ID_TIM5, BSP_VentingTimeRecordHandler);//更新中断产生进入中断服务函数
    BSP_IntEn(BSP_INT_ID_TIM5);
	
	TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);          //使能更新中断
    TIM_Cmd(TIM5, ENABLE);
}


/*
***************************************************************************************************
*                                            PassiveDecompressCurrentNmbInc()
*
* Description : The Passive decompress time record.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Note(s)     : none.
***************************************************************************************************
*/
static void BSP_VentingTimeRecordHandler(void)
{
    if(TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET) {

					//更新排气次数
        TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
    }
}


/*
***************************************************************************************************
***************************************************************************************************
*                                      SPEED CONTROL OUTPUT DEVICE FUNCTIONS
***************************************************************************************************
***************************************************************************************************
*/
/*
***************************************************************************************************
*                                            BSP_DeviceSpdCheckPortInit()
*
* Description : Initialize the all Device speed check port
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : BSP_SpdCheckPortInit()
*
* Note(s)     : none.
***************************************************************************************************
*/
static  void  BSP_DeviceSpdCheckPortInit(u16 arr, u16 psc)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    TIM_ICInitTypeDef  TIM_ICInitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

    /* 时钟使能 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE | RCC_APB2Periph_TIM1 | RCC_APB2Periph_AFIO, ENABLE);

    GPIO_PinRemapConfig(GPIO_FullRemap_TIM1, ENABLE); //定时器一完全重映射
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);  
    /*初始化IO口*/
    //初始化水泵速度检测引脚-CH1
    GPIO_InitStructure.GPIO_Pin   = BSP_GPIOE_PIN9_PUMP_SPEED_CHECK_PORT_NMB;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPD;    //输入下拉，原来为GPIO_Mode_IPU
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOE, &GPIO_InitStructure);
    //初始化制氢风机速度检测引脚-CH2
    GPIO_InitStructure.GPIO_Pin   = BSP_GPIOE_PIN11_HYDRG_FAN_SPEED_CHECK_PORT_NMB;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOE, &GPIO_InitStructure);
    //初始化电堆风扇速度检测引脚-CH3
    GPIO_InitStructure.GPIO_Pin   = BSP_GPIOE_PIN13_STACK_FAN_SPEED_CHECK_PORT_NMB;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOE, &GPIO_InitStructure);

    /*初始化定时器*/
    TIM_DeInit(TIM1);
    TIM_TimeBaseStructure.TIM_Period = arr; //自动重载值,当定时器从0计数到4999，即为5000次为一个定时周期500ms
    TIM_TimeBaseStructure.TIM_Prescaler = psc;//TIMx_CLK= 72MHZ/7200 = 1MHz
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;     //输入时钟不分频
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //向上计数
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

    //3通道输入捕获
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;                //捕获通道
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;     //捕获上升
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //捕获中断
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;           //捕获不分频
    TIM_ICInitStructure.TIM_ICFilter = 0x0;                         //捕获输入不滤波
    TIM_ICInit(TIM1, &TIM_ICInitStructure);

    TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = 0x0;
    TIM_ICInit(TIM1, &TIM_ICInitStructure);

    TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = 0x0;
    TIM_ICInit(TIM1, &TIM_ICInitStructure);

    TIM_ARRPreloadConfig(TIM1, ENABLE);     		   //使能ARR预装载缓冲器(影子功能)
    TIM_GenerateEvent(TIM1, TIM_EventSource_Update);   // 产生软件更新事件，立即更新数据

    TIM_ClearFlag(TIM1, TIM_IT_CC1
                  | TIM_IT_CC2
                  | TIM_IT_CC3); //清除标志位

    TIM_ITConfig(TIM1, TIM_IT_CC1 //使能捕获通道
                 | TIM_IT_CC2
                 | TIM_IT_CC3, ENABLE);

    TIM_Cmd(TIM1, ENABLE);
    BSP_IntVectSet(BSP_INT_ID_TIM1_CC, BSP_DevSpdCaptureFinishedHandler);//捕获完成中断产生进入中断
    BSP_IntEn(BSP_INT_ID_TIM1_CC);

}

/*
***************************************************************************************************
*                                            BSP_SpdCtrlDeviceInit()
*
* Description : Initialize the speed control device(s)
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : BSP_Init()
*
* Note(s)     : none.
***************************************************************************************************
*/
static  void  BSP_SpdCtrlDevicesInit(void)
{
    BSP_PumpCtrlInit();     //泵速调节DAC引脚初始化
    BSP_HydrgFanCtrInit();  //制氢风机DAC调速引脚初始化
    BSP_StackFanCtrInit();  //电堆风扇PWM调速引脚初始化
}
/*
***************************************************************************************************
*                                            BSP_PumpCtrlInit()
*
* Description : Initialize the pump control I/O ports and DAC port.
*                               泵速控制引脚初始化
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : BSP_SpdCtrlDevicesInit()
*
* Note(s)     : none.
***************************************************************************************************
*/
static void  BSP_PumpCtrlInit(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    DAC_InitTypeDef DAC_InitType;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD
                           | RCC_APB2Periph_AFIO , ENABLE);//使能所有GPIO口时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);       //使能DAC通道时钟
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

    GPIO_InitStructure.GPIO_Pin   = BSP_GPIOA_PIN4_PUMP_SPD_ANA_SIGNAL_CTRL_PORT_NMB;   //水泵速度控制引脚
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;        //水泵供电开关控制引脚
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin = BSP_GPIOB_PUMP_PWR_CTRL_PORT_NMB;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

//  GPIO_PinLockConfig(BSP_GPIOA_PUMP_PWR_CTRL_PORT, BSP_GPIOA_PUMP_PWR_CTRL_PORT_NMB);
    GPIO_ResetBits(GPIOB, BSP_GPIOB_PUMP_PWR_CTRL_PORT_NMB);

    DAC_InitType.DAC_Trigger = DAC_Trigger_None;    //不使用触发功能 TEN1=0
    DAC_InitType.DAC_WaveGeneration = DAC_WaveGeneration_None;//不使用波形发生
    DAC_InitType.DAC_LFSRUnmask_TriangleAmplitude = DAC_LFSRUnmask_Bit0;//屏蔽、幅值设置
    DAC_InitType.DAC_OutputBuffer = DAC_OutputBuffer_Enable ;   //DAC输出缓存关闭
    DAC_Init(DAC_Channel_1, &DAC_InitType);  //初始化DAC通道1
//  DAC_Init(DAC_Channel_2,&DAC_InitType);   //初始化DAC通道2

    DAC_Cmd(DAC_Channel_1, ENABLE);
    DAC_SoftwareTriggerCmd(DAC_Channel_1, ENABLE);      //DAC channel 1软件触发使能
//  DAC_Cmd(DAC_Channel_2, ENABLE);
//  DAC_SoftwareTriggerCmd(DAC_Channel_2,ENABLE);
    DAC_SetChannel1Data(DAC_Align_12b_R, 0);            //12位右对齐数据格式设置DAC值
    DAC_SoftwareTriggerCmd(DAC_Channel_1, ENABLE);
//  DAC_SetChannel2Data(DAC_Align_12b_R, 0);            //12位右对齐数据格式设置DAC值
//  DAC_SoftwareTriggerCmd(DAC_Channel_2,ENABLE);
}
/*
***************************************************************************************************
*                                            BSP_PumpPwrOn()
*
* Description : Open the power switch of the pump.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : BSP_SetPumpSpd().
*
* Note(s)     : none.
***************************************************************************************************
*/
static void  BSP_PumpPwrOn(void)
{
    GPIO_SetBits(GPIOB, BSP_GPIOB_PUMP_PWR_CTRL_PORT_NMB);
}

static void  BSP_PumpPwrOff(void)
{
    GPIO_ResetBits(GPIOB, BSP_GPIOB_PUMP_PWR_CTRL_PORT_NMB);
}
/*
***************************************************************************************************
*                                            BSP_SetPumpSpd()
*
* Description : Control the speed of the pump.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : Application.
*
* Note(s)     : none.
***************************************************************************************************
*/
void BSP_SetPumpSpd(u16 i_u16PumpSpdValue)
{
    uint16_t u16DAC1_Value;

    if(i_u16PumpSpdValue >= NUMBER_OF_THE_PUMP_SPEED_GRADES) {
        i_u16PumpSpdValue = NUMBER_OF_THE_PUMP_SPEED_GRADES;
    }

    if(i_u16PumpSpdValue > 0) {
        BSP_PumpPwrOn();
        SetSpdMonitorSwitch(PUMP_SPD_MONITOR);//开启对应的速度监测通道
    } else {
        BSP_PumpPwrOff();
        ResetSpdMonitorSwitch(PUMP_SPD_MONITOR);
    }

    u16DAC1_Value = (uint16_t)(i_u16PumpSpdValue * DAC_DELT_LSB_PER_PUMP_SPEED);

    if(u16DAC1_Value >= 4095) {
        u16DAC1_Value = 4095;
    }

    DAC_SetChannel1Data(DAC_Align_12b_R, u16DAC1_Value);
}
/*
***************************************************************************************************
*                                            BSP_HydrgFanCtrInit()
*
* Description : Initialize the Hydrg Fan control I/O ports and timer ports.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : BSP_SpdCtrlDevicesInit()
*
* Note(s)     : none.
***************************************************************************************************
*/
static void  BSP_HydrgFanCtrInit(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;

    /*时钟使能*/
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

    /*GPIO使能*/
    GPIO_InitStructure.GPIO_Pin   = BSP_GPIOA_PIN5_HYDROGEN_FAN_SPD_ANA_SIGNAL_CTRL_PORT_NMB;//制氢风机速度控制引脚
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;                           //制氢风机电源控制引脚
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin = BSP_GPIOB_HYDROGEN_FAN_PWR_CTRL_PORT_NMB;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_ResetBits(GPIOB, BSP_GPIOB_HYDROGEN_FAN_PWR_CTRL_PORT_NMB);

    DAC_InitTypeDef DAC_InitType;
    DAC_InitType.DAC_Trigger = DAC_Trigger_None;                         //不使用触发功能 TEN1=0
    DAC_InitType.DAC_WaveGeneration = DAC_WaveGeneration_None;           //不使用波形发生
    DAC_InitType.DAC_LFSRUnmask_TriangleAmplitude = DAC_LFSRUnmask_Bit0; //屏蔽、幅值设置
    DAC_InitType.DAC_OutputBuffer = DAC_OutputBuffer_Disable ;           //DAC1输出缓存关闭 BOFF1=1
    DAC_Init(DAC_Channel_2, &DAC_InitType);                              //初始化DAC通道2
    DAC_Cmd(DAC_Channel_2, ENABLE);                                      //使能DAC2

    DAC_SetChannel2Data(DAC_Align_12b_R, 0);  //12位右对齐数据格式设置DAC值

}
/*
***************************************************************************************************
*                                            BSP_HydrgFanPwrOn()
*
* Description : Open the power switch of the hydrogen fan.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Note(s)     : none.
***************************************************************************************************
*/
static void  BSP_HydrgFanPwrOn(void)
{
    GPIO_SetBits(GPIOB, BSP_GPIOB_HYDROGEN_FAN_PWR_CTRL_PORT_NMB);
}

static void  BSP_HydrgFanPwrOff(void)
{
    GPIO_ResetBits(GPIOB, BSP_GPIOB_HYDROGEN_FAN_PWR_CTRL_PORT_NMB);
}
/*
***************************************************************************************************
*                                            BSP_SetHydrgFanSpd()
*
* Description : Control the speed of the hydrogen fan.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : Application.
*
* Note(s)     : none.
***************************************************************************************************
*/
void BSP_SetHydrgFanSpd(u16 i_u16HydrgFanSpdValue)
{

    uint16_t u16DAC2_Value;

    if(i_u16HydrgFanSpdValue >= NUMBER_OF_THE_HYDROGEN_FAN_SPEED_GRADES) {
        i_u16HydrgFanSpdValue = NUMBER_OF_THE_HYDROGEN_FAN_SPEED_GRADES;
    }

    if(i_u16HydrgFanSpdValue > 0) {
        BSP_HydrgFanPwrOn();
        SetSpdMonitorSwitch(HYDROGEN_FAN_SPD_MONITOR);//开启对应的速度监测通道
    } else {
        BSP_HydrgFanPwrOff();
        ResetSpdMonitorSwitch(HYDROGEN_FAN_SPD_MONITOR);
    }

    u16DAC2_Value = (uint16_t)(i_u16HydrgFanSpdValue * DAC_DELT_LSB_PER_HYDROGEN_FAN_SPEED);

    if(u16DAC2_Value >= 4095) {
        u16DAC2_Value = 4095;
    }

    DAC_SetChannel2Data(DAC_Align_12b_R, u16DAC2_Value);
}

/*
***************************************************************************************************
*                                            BSP_StackFanCtrInit()
*
* Description : Initialize the stack Fan control I/O ports and timer ports.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : BSP_SpdCtrlDevicesInit()
*
* Note(s)     : none.
***************************************************************************************************
*/
static void  BSP_StackFanCtrInit(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;

    //时钟使能
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);

    GPIO_InitStructure.GPIO_Pin = BSP_GPIOB_STACK_FAN_SPD_CTRL_PORT_NMB;//TIM4_CH1，控制电堆风机速度的引脚
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //复用推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure); //初始化GPIO

    GPIO_InitStructure.GPIO_Pin = BSP_GPIOC_STACK_FAN_PWR_CTRL_PORT_NMB;//控制电堆风机电源的引脚
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  //推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);//初始化GPIO
    GPIO_ResetBits(GPIOC, BSP_GPIOC_STACK_FAN_PWR_CTRL_PORT_NMB);

    TIM_DeInit(TIM4);
    /* Time base configuration */
    TIM_TimeBaseStructure.TIM_Period = TIMER_UPDATE_NUMBER; //当定时器从0计数到999，即为1000次，为一个定时周期50ms
    TIM_TimeBaseStructure.TIM_Prescaler = 71;     //1KHZ,设置预分频：72MHz/7200 = 10KHz
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分频系数：不分频
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //向上计数模式

    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

    TIM_Cmd(TIM4, DISABLE);  //失能TIM4
    /* Channel 1 PWM1 Mode configuration */
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //脉冲宽度调制模式1，计数值小于比较值为有效电平，即低电平
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //有效电平为低
    TIM_OCInitStructure.TIM_Pulse = 0; //设置待装入输出比较寄存器的脉冲值

    TIM_OC1Init(TIM4, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable); //使能TIM4在CCR1上的预装载寄存器,即TIM4_CCR1的预装载值在更新事件到来时才能被传送至当前寄存器中。

    TIM_ARRPreloadConfig(TIM4, ENABLE);// 使能TIM4重载寄存器ARR
    TIM_GenerateEvent(TIM4, TIM_EventSource_Update);   // 产生软件更新事件，立即更新数据,使重载寄存器中的数据立即生效
    TIM_ClearFlag(TIM4, TIM_FLAG_Update);             //清除标志位。定时器一打开便产生更新事件，若不清除，将会进入中断
    TIM_ITConfig(TIM4, TIM_IT_Update | TIM_IT_CC1, DISABLE); //允许更新中断

    TIM_Cmd(TIM4, ENABLE);  //使能TIM4
}


/*
***************************************************************************************************
*                                            BSP_StackFanPwrOn()
*
* Description : Open the power switch of the stack fan.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : BSP_SetStackFanSpd().
*
* Note(s)     : none.
***************************************************************************************************
*/

static void  BSP_StackFanPwrOn(void)
{
    GPIO_SetBits(GPIOC, BSP_GPIOC_STACK_FAN_PWR_CTRL_PORT_NMB);
}

static void  BSP_StackFanPwrOff(void)
{
    GPIO_ResetBits(GPIOC, BSP_GPIOC_STACK_FAN_PWR_CTRL_PORT_NMB);
}
/*
***************************************************************************************************
*                                            BSP_SetStackFanSpd()
*
* Description : Control the speed of the hydrogen fan.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Note(s)     : none.
***************************************************************************************************
*/

void BSP_SetStackFanSpd(u16 i_u16StackFanSpdValue)
{
    uint16_t u16TIM4_CMP_Value;

    if(i_u16StackFanSpdValue >= NUMBER_OF_THE_STACK_FAN_SPEED_GRADES) {
        i_u16StackFanSpdValue = NUMBER_OF_THE_STACK_FAN_SPEED_GRADES;
    }

    if(i_u16StackFanSpdValue > 0) {
        BSP_StackFanPwrOn();
        SetSpdMonitorSwitch(STACK_FAN_SPD_MONITOR);//开启对应的速度监测通道
    } else {
        BSP_StackFanPwrOff();
        ResetSpdMonitorSwitch(STACK_FAN_SPD_MONITOR);
    }

    u16TIM4_CMP_Value = (uint16_t)(i_u16StackFanSpdValue * TIMER_CMP_DELT_NUMBER_PER_STACK_FAN_SPEED);

    TIM_SetCompare1(TIM4, u16TIM4_CMP_Value);//电堆风扇设置PWM占空比
}


/*
***************************************************************************************************
***************************************************************************************************
*                                            EXTI  INIT
***************************************************************************************************
***************************************************************************************************
*/
/*
***************************************************************************************************
*                                            BSP_CmdButtonInit()
*
* Description : 外部按键中断初始化.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Note(s)     : none.
***************************************************************************************************
*/
void BSP_CmdButtonInit(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOB, ENABLE); //使能复用功能时钟

    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;   //上拉输入
    GPIO_InitStructure.GPIO_Pin = BSP_GPIOB_EXTERNEL_SWITCH_INPUT_PORT_NMB;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource13);
    EXTI_InitStructure.EXTI_Line = EXTI_Line13;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    BSP_IntVectSet(BSP_INT_ID_EXTI15_10, EXTI15_10_StatusCheck_IRQHandler);
    BSP_IntEn(BSP_INT_ID_EXTI15_10);
}


/*
***************************************************************************************************
*                                            BSP_ImpulseInputPortInit()
*
* Description : 泄压脉冲输入监测引脚初始化.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Note(s)     : none.
***************************************************************************************************
*/
static void BSP_ImpulseInputPortInit(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE , ENABLE);

    //松下泄压阀脉冲输入引脚1
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;   //下拉输入
    GPIO_InitStructure.GPIO_Pin = BSP_GPIOE_PIN10_PD_PULSE1_PORT_NMB;
    GPIO_Init(GPIOE, &GPIO_InitStructure);

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOE, GPIO_PinSource10);
    EXTI_InitStructure.EXTI_Line = EXTI_Line10;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);  //根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器

    //脉冲输入引脚2
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;   //下拉输入
    GPIO_InitStructure.GPIO_Pin = BSP_GPIOE_PIN12_PD_PULSE2_PORT_NMB;
    GPIO_Init(GPIOE, &GPIO_InitStructure);

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOE, GPIO_PinSource12);
    EXTI_InitStructure.EXTI_Line = EXTI_Line12;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    BSP_IntVectSet(BSP_INT_ID_EXTI15_10, EXTI15_10_StatusCheck_IRQHandler);
    BSP_IntEn(BSP_INT_ID_EXTI15_10);
}

/*
***************************************************************************************************
*                            EXTI15_10_StatusCheck_IRQHandler()
*
* Description : 外部中断线10-15输入状态检测.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Note(s)     : none.
***************************************************************************************************
*/
static void EXTI15_10_StatusCheck_IRQHandler()
{

    if(EXTI_GetITStatus(EXTI_Line10) != RESET) {//PDPulse1-电堆前端的泄压阀状态

        EXTI_ClearITPendingBit(EXTI_Line10);
    }

    if(EXTI_GetITStatus(EXTI_Line12) != RESET) {//PDPulse2-电堆后端的泄压阀状态
        if(1 == GPIO_ReadInputDataBit(GPIOE, BSP_GPIOE_PIN12_PD_PULSE2_PORT_NMB)) {
            
			g_u8PassiveDecompressCnt++;
		}
        EXTI_ClearITPendingBit(EXTI_Line12);
    }

    if(EXTI_GetITStatus(EXTI_Line13) != RESET) {//开关按键

		CmdButtonFuncDisable();   
        if(EN_WAIT_CMD == GetSystemWorkStatu()) {
//            StartTimerDelayTask(START_UP_SWITCH_CHECK_DELAY_1S, 1000); //定时1s后进行开机动作检测（取消此功能）
        } else {
            StartTimerDelayTask(SHUT_DOWN_SWITCH_CHECK_DELAY_3S, 3000);; //关机动作检测
        }
        EXTI_ClearITPendingBit(EXTI_Line13);  //清除LINE13上的中断标志位
    }
}

/*
***************************************************************************************************
*                            GetPassiveDecompressCntEver30Sec()
*
* Description : get the passive decompress count.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Note(s)     : none.
***************************************************************************************************
*/
uint8_t GetPassiveDecompressCntEver30Sec(void)
{
	return g_u8PassiveDecompressCnt;
}

void ResetPassiveDecompressCnt(void)
{
	g_u8PassiveDecompressCnt = 0;
}
/*
***************************************************************************************************
*                                            CmdButtonFuncDisable()
*
* Description : Disable the button function.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Note(s)     : none.
***************************************************************************************************
*/
void CmdButtonFuncDisable(void)
{
    EXTI_InitTypeDef EXTI_InitStructure;

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource13);
    EXTI_InitStructure.EXTI_Line = EXTI_Line13; //外部控制键
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = DISABLE;
    EXTI_Init(&EXTI_InitStructure);     //根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器
}

void CmdButtonFuncEnable(void)
{
    EXTI_InitTypeDef EXTI_InitStructure;

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource13);
    EXTI_InitStructure.EXTI_Line = EXTI_Line13; //外部控制键
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
}
/*
***************************************************************************************************
*                                            CmdButtonStatuCheck()
*
* Description : Check the button .
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : none.
*
* Note(s)     : none.
***************************************************************************************************
*/

void CmdButtonStatuCheck(void)
{
    SYS_WORK_STATU_Typedef eSysRunningStatu;
    
    if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_13) == 0) {
        eSysRunningStatu = GetSystemWorkStatu();
        if((EN_WAIT_CMD == eSysRunningStatu) || (eSysRunningStatu == EN_ALARMING)) {    
//            CmdStart();  //取消硬件开机功能
        } else {
            CmdShutDown();
        }               
    }
    CmdButtonFuncEnable();
}



//开关设备工作状态查询函数
WHETHER_TYPE_VARIABLE_Typedef   Check_Swtype_Work_Status(SwType_WORK_STATUS_Typedef Is_Swtype_Work)
{
    WHETHER_TYPE_VARIABLE_Typedef WHETHER_TYPE;

    switch(Is_Swtype_Work) {
        case IS_FIRE_STATUS_WORK:
            WHETHER_TYPE = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_5) ? YES : NO;
            break;

        case IS_PUMP_AND_VALUE1_WORK:
            WHETHER_TYPE = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13) ? YES : NO;
            break;

        case IS_VALUE2_AND_IGNITER_WORK:
            WHETHER_TYPE = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_14) ? YES : NO;
            break;

        case IS_FAN_WORK:
            WHETHER_TYPE = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_15) ? YES : NO;
            break;

        default:
            WHETHER_TYPE = NO;
            break;

    }

    return WHETHER_TYPE;
}

/*
***************************************************************************************************
*                                           OSProbe_TmrInit()
*
* Description : Select & initialize a timer for use with the uC/Probe Plug-In for uC/OS-II.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : OSProbe_Init().
*
* Note(s)     : none.
***************************************************************************************************
*/

#if ((APP_CFG_PROBE_OS_PLUGIN_EN == DEF_ENABLED) && \
     (OS_PROBE_HOOKS_EN          == 1))
void  OSProbe_TmrInit(void)
{
}
#endif


/*
***************************************************************************************************
*                                            OSProbe_TmrRd()
*
* Description : Read the current counts of a free running timer.
*
* Argument(s) : none.
*
* Return(s)   : The 32-bit timer counts.
*
* Caller(s)   : OSProbe_TimeGetCycles().
*
* Note(s)     : none.
***************************************************************************************************
*/

#if ((APP_CFG_PROBE_OS_PLUGIN_EN == DEF_ENABLED) && \
     (OS_PROBE_HOOKS_EN          == 1))
CPU_INT32U  OSProbe_TmrRd(void)
{
    return ((CPU_INT32U)DWT_CYCCNT);
}
#endif


/*$PAGE*/
/*
***************************************************************************************************
*                                          CPU_TS_TmrInit()
*
* Description : Initialize & start CPU timestamp timer.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : CPU_TS_Init().
*
*               This function is an INTERNAL CPU module function & MUST be implemented by application/
*               BSP function(s) [see Note #1] but MUST NOT be called by application function(s).
*
* Note(s)     : (1) CPU_TS_TmrInit() is an application/BSP function that MUST be defined by the developer
*                   if either of the following CPU features is enabled :
*
*                   (a) CPU timestamps
*                   (b) CPU interrupts disabled time measurements
*
*                   See 'cpu_cfg.h  CPU TIMESTAMP CONFIGURATION  Note #1'
*                     & 'cpu_cfg.h  CPU INTERRUPTS DISABLED TIME MEASUREMENT CONFIGURATION  Note #1a'.
*
*               (2) (a) Timer count values MUST be returned via word-size-configurable 'CPU_TS_TMR'
*                       data type.
*
*                       (1) If timer has more bits, truncate timer values' higher-order bits greater
*                           than the configured 'CPU_TS_TMR' timestamp timer data type word size.
*
*                       (2) Since the timer MUST NOT have less bits than the configured 'CPU_TS_TMR'
*                           timestamp timer data type word size; 'CPU_CFG_TS_TMR_SIZE' MUST be
*                           configured so that ALL bits in 'CPU_TS_TMR' data type are significant.
*
*                           In other words, if timer size is not a binary-multiple of 8-bit octets
*                           (e.g. 20-bits or even 24-bits), then the next lower, binary-multiple
*                           octet word size SHOULD be configured (e.g. to 16-bits).  However, the
*                           minimum supported word size for CPU timestamp timers is 8-bits.
*
*                       See also 'cpu_cfg.h   CPU TIMESTAMP CONFIGURATION  Note #2'
*                              & 'cpu_core.h  CPU TIMESTAMP DATA TYPES     Note #1'.
*
*                   (b) Timer SHOULD be an 'up'  counter whose values increase with each time count.
*
*                   (c) When applicable, timer period SHOULD be less than the typical measured time
*                       but MUST be less than the maximum measured time; otherwise, timer resolution
*                       inadequate to measure desired times.
*
*                   See also 'CPU_TS_TmrRd()  Note #2'.
***************************************************************************************************
*/

#if (CPU_CFG_TS_TMR_EN == DEF_ENABLED)
void  CPU_TS_TmrInit(void)
{
    CPU_INT32U  cpu_clk_freq_hz;


    DEM_CR         |= (CPU_INT32U)DEM_CR_TRCENA;                /* Enable Cortex-M3's DWT CYCCNT reg.                   */
    DWT_CYCCNT      = (CPU_INT32U)0u;
    DWT_CR         |= (CPU_INT32U)DWT_CR_CYCCNTENA;

    cpu_clk_freq_hz = BSP_CPU_ClkFreq();
    CPU_TS_TmrFreqSet(cpu_clk_freq_hz);
}
#endif


/*$PAGE*/
/*
***************************************************************************************************
*                                           CPU_TS_TmrRd()
*
* Description : Get current CPU timestamp timer count value.
*
* Argument(s) : none.
*
* Return(s)   : Timestamp timer count (see Notes #2a & #2b).
*
* Caller(s)   : CPU_TS_Init(),
*               CPU_TS_Get32(),
*               CPU_TS_Get64(),
*               CPU_IntDisMeasStart(),
*               CPU_IntDisMeasStop().
*
*               This function is an INTERNAL CPU module function & MUST be implemented by application/
*               BSP function(s) [see Note #1] but SHOULD NOT be called by application function(s).
*
* Note(s)     : (1) CPU_TS_TmrRd() is an application/BSP function that MUST be defined by the developer
*                   if either of the following CPU features is enabled :
*
*                   (a) CPU timestamps
*                   (b) CPU interrupts disabled time measurements
*
*                   See 'cpu_cfg.h  CPU TIMESTAMP CONFIGURATION  Note #1'
*                     & 'cpu_cfg.h  CPU INTERRUPTS DISABLED TIME MEASUREMENT CONFIGURATION  Note #1a'.
*
*               (2) (a) Timer count values MUST be returned via word-size-configurable 'CPU_TS_TMR'
*                       data type.
*
*                       (1) If timer has more bits, truncate timer values' higher-order bits greater
*                           than the configured 'CPU_TS_TMR' timestamp timer data type word size.
*
*                       (2) Since the timer MUST NOT have less bits than the configured 'CPU_TS_TMR'
*                           timestamp timer data type word size; 'CPU_CFG_TS_TMR_SIZE' MUST be
*                           configured so that ALL bits in 'CPU_TS_TMR' data type are significant.
*
*                           In other words, if timer size is not a binary-multiple of 8-bit octets
*                           (e.g. 20-bits or even 24-bits), then the next lower, binary-multiple
*                           octet word size SHOULD be configured (e.g. to 16-bits).  However, the
*                           minimum supported word size for CPU timestamp timers is 8-bits.
*
*                       See also 'cpu_cfg.h   CPU TIMESTAMP CONFIGURATION  Note #2'
*                              & 'cpu_core.h  CPU TIMESTAMP DATA TYPES     Note #1'.
*
*                   (b) Timer SHOULD be an 'up'  counter whose values increase with each time count.
*
*                       (1) If timer is a 'down' counter whose values decrease with each time count,
*                           then the returned timer value MUST be ones-complemented.
*
*                   (c) (1) When applicable, the amount of time measured by CPU timestamps is
*                           calculated by either of the following equations :
*
*                           (A) Time measured  =  Number timer counts  *  Timer period
*
*                                   where
*
*                                       Number timer counts     Number of timer counts measured
*                                       Timer period            Timer's period in some units of
*                                                                   (fractional) seconds
*                                       Time measured           Amount of time measured, in same
*                                                                   units of (fractional) seconds
*                                                                   as the Timer period
*
*                                                  Number timer counts
*                           (B) Time measured  =  ---------------------
*                                                    Timer frequency
*
*                                   where
*
*                                       Number timer counts     Number of timer counts measured
*                                       Timer frequency         Timer's frequency in some units
*                                                                   of counts per second
*                                       Time measured           Amount of time measured, in seconds
*
*                       (2) Timer period SHOULD be less than the typical measured time but MUST be less
*                           than the maximum measured time; otherwise, timer resolution inadequate to
*                           measure desired times.
***************************************************************************************************
*/

#if (CPU_CFG_TS_TMR_EN == DEF_ENABLED)
CPU_TS_TMR  CPU_TS_TmrRd(void)
{
    return ((CPU_TS_TMR)DWT_CYCCNT);
}
#endif

/******************* (C) COPYRIGHT 2016 Guangdong ENECO POWER *****END OF FILE****/
