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
* Programmer(s) : Fanjun
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

/*
***************************************************************************************************
*                                           MACRO DEFINITIONS
***************************************************************************************************
*/
#define MX6675_Bits_NUM                 16u

#define ANALOG_TEMP_TO_MAX6675_LSB_RATIO   4.0 //MAX6675��LSB��ֵ���¶ȵı�������

#define MAX6675_DATA_CLK_DELAY_SYSTEM_CYCLES    (BSP_CPU_ClkFreq()/10000000 + 1)    //��MAX6675����ʱ����ȡʱ����Ҫ���ֵ�ϵͳʱ��������
#define MAX6675_DATA_READ_DELAY_SYSTEM_CYCLES   ((MAX6675_DATA_CLK_DELAY_SYSTEM_CYCLES + 1) / 2) //��MAX6675����ʱ����ȡʱ�Ӿ�����Ϊ��֤׼ȷ������ȡ�����Ӻ��ϵͳʱ����

#define MAX6675_CS_YES   GPIO_ResetBits(GPIOD, BSP_GPIOD_MAX6675_CHIPS_SELECT_PORT_NMB);   //MAX6675Ƭѡѡ��
#define MAX6675_CS_NO    GPIO_SetBits(GPIOD, BSP_GPIOD_MAX6675_CHIPS_SELECT_PORT_NMB);
#define MAX6675_SCK_UP   GPIO_SetBits(GPIOD, BSP_GPIOD_MAX6675_CHIPS_SCLK_PORT_NMB);
#define MAX6675_SCK_DOWN GPIO_ResetBits(GPIOD, BSP_GPIOD_MAX6675_CHIPS_SCLK_PORT_NMB);

#define GET_REFORMER_TEMP_DATA_BY_BIT   GPIO_ReadInputDataBit(GPIOD,BSP_GPIOD_MAX6675_CHIP_ONE_DIG_SIGNAL_PORT_NMB)
#define GET_FIRE_TEMP_DATA_BY_BIT       GPIO_ReadInputDataBit(GPIOD,BSP_GPIOD_MAX6675_CHIP_TWO_DIG_SIGNAL_PORT_NMB)

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

static void ButtonStatusCheck_IRQHandler(void);
static void PDPulseStatusCheck_IRQHandler(void);

static  void  BSP_SwTypePwrDeviceStatuInit(void);

static  void  BSP_StackFanCtrInit(void);
static  void  BSP_StackFanPwrOn(void);
static  void  BSP_StackFanPwrOff(void);

static void BSP_DeviceSpdCheckPortInit(u16 arr, u16 psc);
static void BSP_VentingTimeRecordHandler(void);
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

    //HSE����ɹ�
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

        FLASH_SetLatency(FLASH_Latency_2);                          /* ����FLASH��ʱ������Ϊ2,when HCLK > 48MHz.*/
        FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);        /*ʹ��Ԥȡ����*/

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


    BSP_AnaSensorInit();            // ģ�⴫������ʼ��
    BSP_DigSensorInit();            // ���ִ�������ʼ������K���ȵ�ż�����MAX6675
    BSP_SpdCtrlDevicesInit();       // �����豸
    BSP_DeviceSpdCheckPortInit(5000 - 1, 7200 - 1); //�������ų�ʼ��
    BSP_SwTypePwrDeviceStatuInit(); // ����������豸
    BSP_CmdButtonInit();            // Ӳ����ť

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
    BSP_AnaSensorPortsInit();       //ģ�����ɼ����ų�ʼ��
    BSP_AnaSensorADCNVIC_Init();    // ADC DMA1channel1�ж����ȼ�����
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

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE);//ʱ��ʹ��
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_ADC1, ENABLE);

    GPIO_InitStructure.GPIO_Pin   = BSP_GPIOA_PIN_2_STACK_TEMP_ANA_SIGNAL_PORT_NMB;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin  = BSP_GPIOB_STACK_VOLETAGE_ANA_SIGNAL_PORT_NMB;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AIN;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin   = BSP_GPIOA_PIN_3_STACK_CURRENT_ANA_SIGNAL_PORT_NMB;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin   = BSP_GPIOA_PIN_1_LIQUID_PRESS_ANA_SIGNAL_PORT_NMB;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin   = BSP_GPIOB_STACK_HYDROGEN_PRESS_ONE_ANA_SIGNAL_PORT_NMB;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AIN;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin   = BSP_GPIOA_PIN_7_STACK_HYDROGEN_PRESS_TWO_ANA_SIGNAL_PORT_NMB;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin   = BSP_GPIOA_PIN_0_LIQUID_LEVEL_ANA_SIGNAL_PORT_NMB;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin   = BSP_GPIOA_PIN_6_BATTERY_VOLETAGE_ANA_SIGNAL_PORT_NMB;
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

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);  //ʹ��DMA1ʱ��
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_ADC1, ENABLE);
    RCC_ADCCLKConfig(RCC_PCLK2_Div6);   //����ADC��Ƶ����6 72M/6=12,ADCʱ�����Ƶ�ʲ��ܳ���14M

    /* DMA1 channel1 configuration ----------------------------------------------*/
    DMA_DeInit(DMA1_Channel1);
    DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)i_pBuffAddr;   //DMA�������ַ(_u16OriginalDigValue)
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;              //������Ϊ���ݴ�����Դ
    DMA_InitStructure.DMA_BufferSize = i_u8ChUsedNmb;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;//���ݿ��Ϊ16λ
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel1, &DMA_InitStructure);
    /* Enable DMA1 channel1 */
    DMA_Cmd(DMA1_Channel1, ENABLE);
    DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);
    BSP_IntVectSet(BSP_INT_ID_DMA1_CH1, BSP_AnaSigConvertFinishHandler);//ADװ������ź�������
    BSP_IntEn(BSP_INT_ID_DMA1_CH1);

    /* ADC1 configuration ------------------------------------------------------*/
    ADC_DeInit(ADC1);
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;          //��������ģʽ
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;                //����ɨ�衪����ͨ��
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;         //������ת��ģʽ
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;//ת��������������ⲿ��������
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;      //�����Ҷ���
    ADC_InitStructure.ADC_NbrOfChannel = i_u8ChUsedNmb;
    ADC_Init(ADC1, &ADC_InitStructure);
    /*���ù�����ͨ��������˳�򡢲���ʱ�� */
    ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 1, ADC_SampleTime_239Cycles5);    // ����¶�
    ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 2, ADC_SampleTime_239Cycles5);    // ��ѵ�ѹ
    ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 3, ADC_SampleTime_239Cycles5);    // ��ѵ���
    ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 4, ADC_SampleTime_239Cycles5);    // Һѹ
    ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 5, ADC_SampleTime_239Cycles5);    // ��ѹ1
    ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 6, ADC_SampleTime_239Cycles5);    // ��ѹ2
    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 7, ADC_SampleTime_239Cycles5);    // Һλ
//    ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 8, ADC_SampleTime_239Cycles5);    // ��ص�ѹ
//    ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 9, ADC_SampleTime_239Cycles5);    // ����Ũ��
//    ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 10, ADC_SampleTime_239Cycles5);    // ��ص���
//    ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 11, ADC_SampleTime_239Cycles5);    // ���ټ���������
//    ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 12, ADC_SampleTime_239Cycles5);    // ��ѹ������
//    ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 13, ADC_SampleTime_239Cycles5);    // Ԥ�������ʹ�����
//    ADC_RegularChannelConfig(ADC1, ADC_Channel_15, 14, ADC_SampleTime_239Cycles5);    // Ԥ�������ʹ�����

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
    while(ADC_GetCalibrationStatus(ADC1));  //�ȴ�У׼���

    ADC_SoftwareStartConvCmd(ADC1, ENABLE); //ʹ�����ת����������
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
        //�����ź�������֪ADC��DMAת���Ѿ����
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
*               ��ʼ��оƬ���ȵ�ż��������
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
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);    //ʹ�ܹ��ܸ���IO
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);    //SWJ���Խӿ���ȫʧ��

    /******����GPIOB������*********/
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Pin = BSP_GPIOD_MAX6675_CHIP_TWO_DIG_SIGNAL_PORT_NMB | BSP_GPIOD_MAX6675_CHIP_ONE_DIG_SIGNAL_PORT_NMB;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin = BSP_GPIOD_MAX6675_CHIPS_SELECT_PORT_NMB | BSP_GPIOD_MAX6675_CHIPS_SCLK_PORT_NMB;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOD, BSP_GPIOD_MAX6675_CHIPS_SELECT_PORT_NMB | BSP_GPIOD_MAX6675_CHIPS_SCLK_PORT_NMB);
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
* Argument(s) :  *i_fTemp   ����¶����ݵ�����
*                *err       ��Ŵ����־������
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

    for(i = 0; i < MAX6675_DATA_CLK_DELAY_SYSTEM_CYCLES; i++);      // ����Ƭѡѡ�к������MX6675��ʱ����Ҫ�ͺ�100ns�������ڴ˿���ʱ������ʱ��Ҫ��

    for(j = 0; j < MX6675_Bits_NUM; j++) {
        MAX6675_SCK_UP;

        for(i = 0; i < MAX6675_DATA_READ_DELAY_SYSTEM_CYCLES; i++); // ��MAX6675ʱ�ӵ�ƽά��ʱ�䲻�ö���100ns�������ڶ�ȡ����ǰ�������ʱ�������ڣ���֤����׼ȷ��

        m_temp[0] = (m_temp[0] << 1) | GET_REFORMER_TEMP_DATA_BY_BIT;
        m_temp[1] = (m_temp[1] << 1) | GET_FIRE_TEMP_DATA_BY_BIT;

        for(i = 0; i < MAX6675_DATA_READ_DELAY_SYSTEM_CYCLES; i++);

        MAX6675_SCK_DOWN;

        for(i = 0; i < MAX6675_DATA_CLK_DELAY_SYSTEM_CYCLES; i++);
    }

    MAX6675_CS_NO;

    if((m_temp[0] & (1 << 15 | 4)) == 0) { //������ȷ���ȵ�ż��δ����
        m_temp[0] &= 0x7FFF;
        m_temp[0] >>= 3;
        *err = 0;
    } else {
        m_temp[0] = 0xF9C;      //3996
        *err = 1;
    }

    if((m_temp[1] & (1 << 15 | 4)) == 0) { //������ȷ���ȵ�ż��δ����
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
*                                             BSP_BuzzerInit()()
*
* Description : Initialize the I/O for the buzzer.
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

void  BSP_BuzzerInit(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA
                           | RCC_APB2Periph_GPIOB
                           | RCC_APB2Periph_GPIOC
                           | RCC_APB2Periph_GPIOD
                           , ENABLE);//ʹ������GPIO��ʱ��

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);

    PWR_BackupAccessCmd(ENABLE);//�����޸�RTC �ͺ󱸼Ĵ���
    RCC_LSEConfig(RCC_LSE_OFF);//�ر��ⲿ�����ⲿʱ���źŹ��� ��PC13 PC14 PC15 �ſ��Ե���ͨIO�á�
    BKP_TamperPinCmd(DISABLE);//�ر����ּ�⹦�ܣ�Ҳ���� PC13��Ҳ���Ե���ͨIO ʹ��
    PWR_BackupAccessCmd(DISABLE);//��ֹ�޸ĺ󱸼Ĵ���


    GPIO_InitStructure.GPIO_Pin   = BSP_GPIOB_BUZZER_CTRL_PORT_NMB;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOB, BSP_GPIOB_BUZZER_CTRL_PORT_NMB);

}
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
//��������ת
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
                           , ENABLE);//ʹ������GPIO��ʱ��

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);//��������ռ����SW���Կڻ�JTAG�ڣ���Ҫ�����ȹر�

    GPIO_InitStructure.GPIO_Pin   = BSP_GPIOB_BUZZER_CTRL_PORT_NMB;//������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_ResetBits(GPIOB, BSP_GPIOB_BUZZER_CTRL_PORT_NMB);
    GPIO_InitStructure.GPIO_Pin   = BSP_GPIOB_LIQUID_INPUT_VALVE_ONE_PWR_CTRL_PORT_NMB;  // ��Һ��ŷ�1��������
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOB, BSP_GPIOB_LIQUID_INPUT_VALVE_ONE_PWR_CTRL_PORT_NMB);

    GPIO_InitStructure.GPIO_Pin   = BSP_GPIOC_LIQUID_INPUT_VALVE_TWO_PWR_CTRL_PORT_NMB;  // ��Һ��ŷ�2��������
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOC, BSP_GPIOC_LIQUID_INPUT_VALVE_TWO_PWR_CTRL_PORT_NMB);

    GPIO_InitStructure.GPIO_Pin   = BSP_GPIOC_IGNITER_PWR_CTRL_PORT_NMB;             // �������������
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOC, BSP_GPIOC_IGNITER_PWR_CTRL_PORT_NMB);

    GPIO_InitStructure.GPIO_Pin   = BSP_GPIOB_KEEPWARM_HEATER_PWR_CTRL_PORT_NMB;              // ���¼�������������
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOB, BSP_GPIOB_KEEPWARM_HEATER_PWR_CTRL_PORT_NMB);

    GPIO_InitStructure.GPIO_Pin   = BSP_GPIOC_FAST_HEATER_PWR_CTRL_PORT_NMB;              // ���ټ�������������
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOC, BSP_GPIOC_FAST_HEATER_PWR_CTRL_PORT_NMB);

    GPIO_InitStructure.GPIO_Pin   = BSP_GPIOC_HYDROGEN_INTO_STACK_VALVE_PWR_CTRL_PORT_NMB; // ��ѽ�������������
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOC, BSP_GPIOC_HYDROGEN_INTO_STACK_VALVE_PWR_CTRL_PORT_NMB);

    GPIO_InitStructure.GPIO_Pin   = BSP_GPIOC_HYDROGEN_OUTOF_STACK_VALVE_PWR_CTRL_PORT_NMB;// ��ѳ�������������
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOC, BSP_GPIOC_HYDROGEN_OUTOF_STACK_VALVE_PWR_CTRL_PORT_NMB);

    GPIO_InitStructure.GPIO_Pin   = BSP_GPIOB_DC_CONNECTER_PWR_CTRL_PORT_NMB;             // ֱ���Ӵ�����������
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOB, BSP_GPIOB_DC_CONNECTER_PWR_CTRL_PORT_NMB);

    PWR_BackupAccessCmd(ENABLE);//�����޸�RTC �ͺ󱸼Ĵ���
    RCC_LSEConfig(RCC_LSE_OFF);//�ر��ⲿ�����ⲿʱ���źŹ��� ��PC13 PC14 PC15 �ſ��Ե���ͨIO�á�
    BKP_TamperPinCmd(DISABLE);//�ر����ּ�⹦�ܣ�Ҳ���� PC13��Ҳ���Ե���ͨIO ʹ��
    PWR_BackupAccessCmd(DISABLE);//��ֹ�޸ĺ󱸼Ĵ���

    GPIO_InitStructure.GPIO_Pin   = BSP_GPIOC_RSVD_OUTPUT_PWR_CTRL_PORT_NMB;             // �Զ���Һ���ü״���
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOC, BSP_GPIOC_RSVD_OUTPUT_PWR_CTRL_PORT_NMB);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin =  BSP_GPIOD_RSVD8_OUTPUT_PWR_CTRL_PORT_NMB ;       //Ԥ�����8
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOD,  BSP_GPIOD_RSVD8_OUTPUT_PWR_CTRL_PORT_NMB);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin =  BSP_GPIOD_RSVD6_OUTPUT_PWR_CTRL_PORT_NMB ;      //Ԥ�����6
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOD,  BSP_GPIOD_RSVD6_OUTPUT_PWR_CTRL_PORT_NMB);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = BSP_GPIOD_RSVD7_OUTPUT_PWR_CTRL_PORT_NMB  ; //Ԥ�����7
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOD,  BSP_GPIOD_RSVD7_OUTPUT_PWR_CTRL_PORT_NMB);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin =  BSP_GPIOE_RSVD5_OUTPUT_PWR_CTRL_PORT_NMB;    //Ԥ�����5
    GPIO_Init(GPIOE, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOE, BSP_GPIOE_RSVD5_OUTPUT_PWR_CTRL_PORT_NMB);

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
}

void  BSP_IgniterPwrOff(void)
{
    GPIO_ResetBits(GPIOC, BSP_GPIOC_IGNITER_PWR_CTRL_PORT_NMB);
}
/*
***************************************************************************************************
*                                            BSP_KeepWarmHeaterPwrOn()
*
* Description : ����ż�����.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Note(s)     : none.
***************************************************************************************************
*/
void  BSP_KeepWarmHeaterPwrOn(void)
{
    GPIO_SetBits(GPIOB, BSP_GPIOB_KEEPWARM_HEATER_PWR_CTRL_PORT_NMB);
}

void  BSP_KeepWarmHeaterPwrOff(void)
{
    GPIO_ResetBits(GPIOB, BSP_GPIOB_KEEPWARM_HEATER_PWR_CTRL_PORT_NMB);
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
    GPIO_SetBits(GPIOC, BSP_GPIOC_FAST_HEATER_PWR_CTRL_PORT_NMB);
}

void  BSP_FastHeaterPwrOff(void)
{
    GPIO_ResetBits(GPIOC, BSP_GPIOC_FAST_HEATER_PWR_CTRL_PORT_NMB);
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
    GPIO_SetBits(GPIOB, BSP_GPIOB_DC_CONNECTER_PWR_CTRL_PORT_NMB);
}

void  BSP_DCConnectValvePwrOff(void)
{
    GPIO_ResetBits(GPIOB, BSP_GPIOB_DC_CONNECTER_PWR_CTRL_PORT_NMB);
}

/*
***************************************************************************************************
*                                            BSP_OutsidePumpPwrOn()
*
* Description : Open the power switch of the outside pump.
*               �Զ���Һˮ�ÿ���
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
*                                           Ԥ��5/6/7/8�ڿ���
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
void  BSP_RSVD5PwrOn(void)
{
    GPIO_SetBits(GPIOE, BSP_GPIOE_RSVD5_OUTPUT_PWR_CTRL_PORT_NMB);
}
void  BSP_RSVD5PwrOff(void)
{
    GPIO_ResetBits(GPIOE, BSP_GPIOE_RSVD5_OUTPUT_PWR_CTRL_PORT_NMB);
}

void  BSP_RSVD6PwrOn(void)
{
    GPIO_SetBits(GPIOD, BSP_GPIOD_RSVD6_OUTPUT_PWR_CTRL_PORT_NMB);
}
void  BSP_RSVD6PwrOff(void)
{
    GPIO_ResetBits(GPIOD, BSP_GPIOD_RSVD6_OUTPUT_PWR_CTRL_PORT_NMB);
}

void  BSP_RSVD7PwrOn(void)
{
    GPIO_SetBits(GPIOD, BSP_GPIOD_RSVD7_OUTPUT_PWR_CTRL_PORT_NMB);
}

void  BSP_RSVD7PwrOff(void)
{
    GPIO_ResetBits(GPIOD, BSP_GPIOD_RSVD7_OUTPUT_PWR_CTRL_PORT_NMB);
}

void  BSP_RSVD8PwrOn(void)
{
    GPIO_SetBits(GPIOD, BSP_GPIOD_RSVD8_OUTPUT_PWR_CTRL_PORT_NMB);
}

void  BSP_RSVD8PwrOff(void)
{
    GPIO_ResetBits(GPIOD, BSP_GPIOD_RSVD8_OUTPUT_PWR_CTRL_PORT_NMB);
}
/*
***************************************************************************************************
*                                            BSP_GetPassiveGpioStatu()
*
* Description : ��ȡ���β��йѹ����������״̬.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Note(s)     : none.
***************************************************************************************************
*/
uint8_t BSP_GetPassiveGpioStatu(void)
{
    return GPIO_ReadInputDataBit(GPIOE, BSP_GPIOE_PD_PULSE2_PORT_NMB);
}
/*
***************************************************************************************************
*                                            BSP_VentingIntervalRecordTimerInit()
*
* Description : ��ʼ���������ʱ���¼Ӳ����ʱ��.
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
    /* ʱ����������*/
    TIM_TimeBaseStructure.TIM_Period = 10 - 1;     //����ֵ:����ʱ����0������10��1ms����һ�θ����¼�
    TIM_TimeBaseStructure.TIM_Prescaler = 7199;     //����Ԥ��Ƶϵ�� 72MHz/7200 = 10KHz
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //���ϼ���ģʽ

    TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);

    TIM_ARRPreloadConfig(TIM5, ENABLE);                                 // ʹ��TIM5���ؼĴ���
    TIM_GenerateEvent(TIM5, TIM_EventSource_Update);    // ��ʱ���¼�����������¼�������������������
    TIM_ClearFlag(TIM5, TIM_FLAG_Update);               //�����־λ����ʱ��һ�򿪱���������¼����������������ֱ�ӽ����ж�
    TIM_ITConfig(TIM5, TIM_IT_Update, DISABLE);                 //���ܸ����ж�

    BSP_IntVectSet(BSP_INT_ID_TIM5, BSP_VentingTimeRecordHandler);//�����жϲ��������жϷ�����
    BSP_IntEn(BSP_INT_ID_TIM5);
}

/*
***************************************************************************************************
*                                            BSP_StartRunningVentingTimeRecord()
*
* Description : ��ʼ���β������ʱ�������¼.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Note(s)     : none.
***************************************************************************************************
*/
void BSP_StartRunningVentingTimeRecord()
{
    TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);                  //ʹ�ܸ����ж�
    TIM_Cmd(TIM5, ENABLE);                                      //ʹ�ܶ�ʱ��5
}

void BSP_StopRunningVentingTimeRecord()
{
    TIM_ITConfig(TIM5, TIM_IT_Update, DISABLE); //���ܸ����ж�
    TIM_Cmd(TIM5, DISABLE);                   //���ܶ�ʱ��5
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

        StackVentAirTimeParameter.u32_TimeRecordNum++;    //����ʱ���������,��һ�μ�0.001s
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

    /* ʱ��ʹ�� */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE | RCC_APB2Periph_TIM1 | RCC_APB2Periph_AFIO, ENABLE);

    GPIO_PinRemapConfig(GPIO_FullRemap_TIM1, ENABLE); //��ʱ��һ��ȫ��ӳ��
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);  //SWJ��ȫʧ��
    /*��ʼ��IO��*/
    //��ʼ��ˮ���ٶȼ������
    GPIO_InitStructure.GPIO_Pin   = BSP_GPIOE_PUMP_SPEED_CHECK_PORT_NMB;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;    //�������������ݵ�·ͼ;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOE, &GPIO_InitStructure);
    //��ʼ���������ٶȼ������
    GPIO_InitStructure.GPIO_Pin   = BSP_GPIOE_HYDRG_FAN_SPEED_CHECK_PORT_NMB;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOE, &GPIO_InitStructure);
    //��ʼ����ѷ����ٶȼ������
    GPIO_InitStructure.GPIO_Pin   = BSP_GPIOE_STACK_FAN_SPEED_CHECK_PORT_NMB;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOE, &GPIO_InitStructure);

    /*��ʼ����ʱ��*/
    TIM_DeInit(TIM1);
    TIM_TimeBaseStructure.TIM_Period = arr; //�Զ�����ֵ TIMX->ARR,����ʱ����0������999����Ϊ1000��Ϊһ����ʱ����250ms
    TIM_TimeBaseStructure.TIM_Prescaler = psc;//��Ԥ��Ƶ, ʹTIMx_CLK= 72MHZ/17999 = 4KHz
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;     //����ʱ�Ӳ���Ƶ
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //���ϼ���
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

    //3ͨ�����벶��
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;                //����ͨ��
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;     //��������
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //�����ж�
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;           //���񲻷�Ƶ
    TIM_ICInitStructure.TIM_ICFilter = 0x0;                         //�������벻�˲�
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

    TIM_ARRPreloadConfig(TIM1, ENABLE);     //ʹ��ARRԤװ�ػ�����(Ӱ�ӹ���)
    TIM_GenerateEvent(TIM1, TIM_EventSource_Update);   // ������������¼���������������

    TIM_ClearFlag(TIM1, TIM_IT_CC1
                  | TIM_IT_CC2
                  | TIM_IT_CC3
                  | TIM_FLAG_Update); //�����־λ����ʱ��һ�򿪱���������¼��������������������ж�

    TIM_ITConfig(TIM1, TIM_IT_CC1       //�ڸ����ж���һ��һ��ͨ������ʹ�ܣ�����ֻʹ��һ��ͨ��
                 | TIM_IT_CC2
                 | TIM_IT_CC3
                 | TIM_IT_Update, ENABLE);

    TIM_Cmd(TIM1, ENABLE);
    BSP_IntVectSet(BSP_INT_ID_TIM1_CC, BSP_DevSpdCaptureFinishedHandler);//��������жϲ��������ж�
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
    BSP_PumpCtrlInit();     //���ٵ���DAC���ų�ʼ��
    BSP_HydrgFanCtrInit();  //������DAC�������ų�ʼ��
    BSP_StackFanCtrInit();  //��ѷ���PWM�������ų�ʼ��
}
/*
***************************************************************************************************
*                                            BSP_PumpCtrlInit()
*
* Description : Initialize the pump control I/O ports and DAC port.
*                               ���ٿ������ų�ʼ��
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
                           | RCC_APB2Periph_AFIO , ENABLE);//ʹ������GPIO��ʱ��
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);       //ʹ��DACͨ��ʱ��
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);

    GPIO_InitStructure.GPIO_Pin   = BSP_GPIOA_PIN_4_PUMP_SPD_ANA_SIGNAL_CTRL_PORT_NMB;   //ˮ���ٶȿ�������
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;        //ˮ�ù��翪�ؿ�������
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin = BSP_GPIOB_PUMP_PWR_CTRL_PORT_NMB;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

//  GPIO_PinLockConfig(BSP_GPIOA_PUMP_PWR_CTRL_PORT, BSP_GPIOA_PUMP_PWR_CTRL_PORT_NMB);
    GPIO_ResetBits(GPIOB, BSP_GPIOB_PUMP_PWR_CTRL_PORT_NMB);

    DAC_InitType.DAC_Trigger = DAC_Trigger_None;    //��ʹ�ô������� TEN1=0
    DAC_InitType.DAC_WaveGeneration = DAC_WaveGeneration_None;//��ʹ�ò��η���
    DAC_InitType.DAC_LFSRUnmask_TriangleAmplitude = DAC_LFSRUnmask_Bit0;//���Ρ���ֵ����
    DAC_InitType.DAC_OutputBuffer = DAC_OutputBuffer_Enable ;   //DAC�������ر�
    DAC_Init(DAC_Channel_1, &DAC_InitType);  //��ʼ��DACͨ��1
//  DAC_Init(DAC_Channel_2,&DAC_InitType);   //��ʼ��DACͨ��2

    DAC_Cmd(DAC_Channel_1, ENABLE);
    DAC_SoftwareTriggerCmd(DAC_Channel_1, ENABLE);      //DAC channel 1�������ʹ��
//  DAC_Cmd(DAC_Channel_2, ENABLE);
//  DAC_SoftwareTriggerCmd(DAC_Channel_2,ENABLE);
    DAC_SetChannel1Data(DAC_Align_12b_R, 0);            //12λ�Ҷ������ݸ�ʽ����DACֵ
    DAC_SoftwareTriggerCmd(DAC_Channel_1, ENABLE);
//  DAC_SetChannel2Data(DAC_Align_12b_R, 0);            //12λ�Ҷ������ݸ�ʽ����DACֵ
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
        SetSpdMonitorSwitch(PUMP_SPD_MONITOR);//������Ӧ���ٶȼ��ͨ��
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

    /*ʱ��ʹ��*/
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);

    /*GPIOʹ��*/
    GPIO_InitStructure.GPIO_Pin   = BSP_GPIOA_PIN_5_HYDROGEN_FAN_SPD_ANA_SIGNAL_CTRL_PORT_NMB;//�������ٶȿ�������
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;                           //��������Դ��������
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin = BSP_GPIOB_HYDROGEN_FAN_PWR_CTRL_PORT_NMB;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_ResetBits(GPIOB, BSP_GPIOB_HYDROGEN_FAN_PWR_CTRL_PORT_NMB);

    DAC_InitTypeDef DAC_InitType;
    DAC_InitType.DAC_Trigger = DAC_Trigger_None;                         //��ʹ�ô������� TEN1=0
    DAC_InitType.DAC_WaveGeneration = DAC_WaveGeneration_None;           //��ʹ�ò��η���
    DAC_InitType.DAC_LFSRUnmask_TriangleAmplitude = DAC_LFSRUnmask_Bit0; //���Ρ���ֵ����
    DAC_InitType.DAC_OutputBuffer = DAC_OutputBuffer_Disable ;           //DAC1�������ر� BOFF1=1
    DAC_Init(DAC_Channel_2, &DAC_InitType);                              //��ʼ��DACͨ��2
    DAC_Cmd(DAC_Channel_2, ENABLE);                                      //ʹ��DAC2

    DAC_SetChannel2Data(DAC_Align_12b_R, 0);  //12λ�Ҷ������ݸ�ʽ����DACֵ

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
        SetSpdMonitorSwitch(HYDROGEN_FAN_SPD_MONITOR);//������Ӧ���ٶȼ��ͨ��
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

    //ʱ��ʹ��
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);

    GPIO_InitStructure.GPIO_Pin = BSP_GPIOB_STACK_FAN_SPD_CTRL_PORT_NMB;//TIM4_CH1�����Ƶ�ѷ���ٶȵ�����
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //�����������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure); //��ʼ��GPIO

    GPIO_InitStructure.GPIO_Pin = BSP_GPIOC_STACK_FAN_PWR_CTRL_PORT_NMB;//���Ƶ�ѷ����Դ������
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  //�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);//��ʼ��GPIO
    GPIO_ResetBits(GPIOC, BSP_GPIOC_STACK_FAN_PWR_CTRL_PORT_NMB);

    TIM_DeInit(TIM4);
    /* Time base configuration */
    TIM_TimeBaseStructure.TIM_Period = TIMER_UPDATE_NUMBER; //����ʱ����0������999����Ϊ1000�Σ�Ϊһ����ʱ����50ms
    TIM_TimeBaseStructure.TIM_Prescaler = 3599;     //����Ԥ��Ƶ��72MHz/3600 = 20KHz
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //����ʱ�ӷ�Ƶϵ��������Ƶ
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //���ϼ���ģʽ

    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

    TIM_Cmd(TIM4, DISABLE);  //ʧ��TIM4
    /* Channel 1 PWM1 Mode configuration */
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //�����ȵ���ģʽ1������ֵС�ڱȽ�ֵΪ��Ч��ƽ�����͵�ƽ
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //��Ч��ƽΪ��
    TIM_OCInitStructure.TIM_Pulse = 0; //���ô�װ������ȽϼĴ���������ֵ

    TIM_OC1Init(TIM4, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable); //ʹ��TIM4��CCR1�ϵ�Ԥװ�ؼĴ���,��TIM4_CCR1��Ԥװ��ֵ�ڸ����¼�����ʱ���ܱ���������ǰ�Ĵ����С�

    TIM_ARRPreloadConfig(TIM4, ENABLE);          // ʹ��TIM4���ؼĴ���ARR
    TIM_GenerateEvent(TIM4, TIM_EventSource_Update);   // ������������¼���������������,ʹ���ؼĴ����е�����������Ч
    TIM_ClearFlag(TIM4, TIM_FLAG_Update);             //�����־λ����ʱ��һ�򿪱���������¼��������������������ж�
    TIM_ITConfig(TIM4, TIM_IT_Update | TIM_IT_CC1, DISABLE); //��������ж�

    TIM_Cmd(TIM4, ENABLE);  //ʹ��TIM4
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
        SetSpdMonitorSwitch(STACK_FAN_SPD_MONITOR);//������Ӧ���ٶȼ��ͨ��
    } else {
        BSP_StackFanPwrOff();
        ResetSpdMonitorSwitch(STACK_FAN_SPD_MONITOR);
    }

    u16TIM4_CMP_Value = (uint16_t)(i_u16StackFanSpdValue * TIMER_CMP_DELT_NUMBER_PER_STACK_FAN_SPEED);

    TIM_SetCompare1(TIM4, u16TIM4_CMP_Value);//��ѷ�������PWMռ�ձ�
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
* Description : �ⲿ�����жϳ�ʼ��.
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
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOB, ENABLE); //ʹ�ܸ��ù���ʱ��

    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;   //��������
    GPIO_InitStructure.GPIO_Pin = BSP_GPIOB_EXTERNEL_SWITCH_INPUT_PORT_NMB;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource13);
    EXTI_InitStructure.EXTI_Line = EXTI_Line13;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);     //����EXTI_InitStruct��ָ���Ĳ�����ʼ������EXTI�Ĵ���

    BSP_IntVectSet(BSP_INT_ID_EXTI15_10, ButtonStatusCheck_IRQHandler);
    BSP_IntEn(BSP_INT_ID_EXTI15_10);
}

static void ButtonStatusCheck_IRQHandler()
{
    if(EXTI_GetITStatus(EXTI_Line13) != RESET) {//���ذ���
        CmdButtonFuncDisable();
        StartCmdButtonActionCheckDly(); //��ʱ����ʱ0.5����ж����жϰ�ť���£�Ȼ��ִ����Ӧ����
        EXTI_ClearITPendingBit(EXTI_Line13);  //���LINE10�ϵ��жϱ�־λ
    }
}
/*
***************************************************************************************************
*                                            BSP_CmdButtonInit()
*
* Description : йѹ�������������ų�ʼ��.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Note(s)     : none.
***************************************************************************************************
*/
void BSP_ImpulseInputPortInit(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOE, ENABLE); //ʹ�ܸ��ù���ʱ��

    //����йѹ��������������1
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;   //��������
    GPIO_InitStructure.GPIO_Pin = BSP_GPIOE_PD_PULSE1_PORT_NMB;
    GPIO_Init(GPIOE, &GPIO_InitStructure);

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOE, GPIO_PinSource10);
    EXTI_InitStructure.EXTI_Line = EXTI_Line10;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);  //����EXTI_InitStruct��ָ���Ĳ�����ʼ������EXTI�Ĵ���

    //������������2
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;   //��������
    GPIO_InitStructure.GPIO_Pin = BSP_GPIOE_PD_PULSE2_PORT_NMB;
    GPIO_Init(GPIOE, &GPIO_InitStructure);

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOE, GPIO_PinSource12);
    EXTI_InitStructure.EXTI_Line = EXTI_Line12;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    BSP_IntVectSet(BSP_INT_ID_EXTI15_10, PDPulseStatusCheck_IRQHandler);
    BSP_IntEn(BSP_INT_ID_EXTI15_10);
}

/*
***************************************************************************************************
*                                  PDPulseStatusCheck_IRQHandler()
*
* Description : �����������Ų����жϺ���.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Note(s)     : none.
***************************************************************************************************
*/
static void PDPulseStatusCheck_IRQHandler()
{
    OS_ERR      err;
    static uint8_t u8VentAirTimeIntervalRecordFlag = NO;
    static uint8_t u8DecompressVentTimeRecordFlag = NO;

    if(EXTI_GetITStatus(EXTI_Line10) != RESET) {//PDPulse1-���ǰ�˵�йѹ��״̬

        EXTI_ClearITPendingBit(EXTI_Line10);
    }

    if(EXTI_GetITStatus(EXTI_Line12) != RESET) {//PDPulse2-��Ѻ�˵�йѹ��״̬
        if(0 == BSP_GetPassiveGpioStatu()) {
            DecompressCountPerMinuteInc();
            BSP_StartRunningVentingTimeRecord(); //Start recording the exhaust time parameter
            StackVentAirTimeParameter.fVentAirTimeIntervalValue = StackVentAirTimeParameter.u32_TimeRecordNum;//��¼�������ʱ��
            StackVentAirTimeParameter.u32_TimeRecordNum = 0;//reset time record num
            u8VentAirTimeIntervalRecordFlag = YES;
        } else {
            StackVentAirTimeParameter.fDecompressVentTimeValue = StackVentAirTimeParameter.u32_TimeRecordNum;//��¼йѹʱ��
            StackVentAirTimeParameter.u32_TimeRecordNum = 0;//reset time record num
            u8DecompressVentTimeRecordFlag = YES;
        }

        //����������¼�귢�͵�ƥ��ƫ�����������
        if((u8VentAirTimeIntervalRecordFlag == YES) && (u8DecompressVentTimeRecordFlag == YES)) {
            OSTaskSemPost(&StackHydrogenYieldMatchingOffsetValueMonitorTaskTCB,
                          OS_OPT_POST_NO_SCHED,
                          &err);
            u8VentAirTimeIntervalRecordFlag = NO;
            u8DecompressVentTimeRecordFlag = NO;
        }

        EXTI_ClearITPendingBit(EXTI_Line12);
    }
}
/*
***************************************************************************************************
*                                            BSP_CmdButtonInit()
*
* Description : �����������ų�ʼ��.
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
    EXTI_InitStructure.EXTI_Line = EXTI_Line13; //�ⲿ���Ƽ�
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = DISABLE;
    EXTI_Init(&EXTI_InitStructure);     //����EXTI_InitStruct��ָ���Ĳ�����ʼ������EXTI�Ĵ���
}

void CmdButtonFuncEnable(void)
{
    EXTI_InitTypeDef EXTI_InitStructure;

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource13);
    EXTI_InitStructure.EXTI_Line = EXTI_Line13; //�ⲿ���Ƽ�
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
}

void StartCmdButtonActionCheckDly(void)
{
    TIM7_DlyMilSecondsInit(500);
}
void TIM7_DlyMilSecondsInit(u16 i_u16DlyMilSeconds)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);    //ʹ��TIM7ʱ�ӣ�APB1=72MHz

    //��ʼ����ʱ��7
    TIM_DeInit(TIM7);
    TIM_TimeBaseStructure.TIM_Period = (i_u16DlyMilSeconds * 2 - 1); //�趨�������Զ���װֵ,0.5�������ж�
    TIM_TimeBaseStructure.TIM_Prescaler = 35999;                     //Ԥ��Ƶ��,��72MHz��Ϊ2KHz
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;          //����ʱ�ӷָ�:TDTS = Tck_tim,����TIM6��7��Ч
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;      //TIM���ϼ���ģʽ
    TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);                  //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ

    //�жϷ����ʼ��
    NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;             //TIM7�ж�
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;   //��ռ���ȼ�2��
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;          //�����ȼ�0��
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;             //IRQͨ����ʹ��
    NVIC_Init(&NVIC_InitStructure);                             //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���

    TIM_GenerateEvent(TIM7, TIM_EventSource_Update);   // ������������¼���������������
    TIM_ClearFlag(TIM7, TIM_FLAG_Update);             //�����־λ����ʱ��һ�򿪱���������¼��������������������ж�

    TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);

    BSP_IntVectSet(BSP_INT_ID_TIM7, CmdButtonStatuCheck);
    BSP_IntEn(BSP_INT_ID_DMA2_CH3);

    TIM_Cmd(TIM7, ENABLE);  //ʹ�ܶ�ʱ��7
}

void CmdButtonStatuCheck(void)
{
    SYSTEM_WORK_STATU_Typedef eSysRunningStatu;

    if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_13) == 0) {
        eSysRunningStatu = GetSystemWorkStatu();

        if((EN_WAITTING_COMMAND == eSysRunningStatu) || (eSysRunningStatu == EN_ALARMING)) {
            CmdStart();
        } else {
            CmdShutDown();
        }
    }

    CmdButtonFuncEnable();
    TIM_ITConfig(TIM7, TIM_IT_Update, DISABLE);
    TIM_ClearITPendingBit(TIM7, TIM_IT_Update); //����жϱ�־λ
}


/*
***************************************************************************************************
*                                            BSP_CmdButtonInit()
*
* Description : �������豸�Լ����̣����й��̲��ܱ����.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Note(s)     : none.
***************************************************************************************************
*/
void SwitchTypeDevicesSelfCheck()
{
    OS_ERR      err;
    BSP_LqdValve1_PwrOn();
    OSTimeDlyHMSM(0, 0, 0, 500, OS_OPT_TIME_HMSM_STRICT, &err);
    BSP_LqdValve1_PwrOff();

    BSP_LqdValve2_PwrOn();
    OSTimeDlyHMSM(0, 0, 0, 500, OS_OPT_TIME_HMSM_STRICT, &err);
    BSP_LqdValve2_PwrOff();

    BSP_IgniterPwrOn();
    OSTimeDlyHMSM(0, 0, 0, 500, OS_OPT_TIME_HMSM_STRICT, &err);
    BSP_IgniterPwrOff();

    BSP_HydrgInValvePwrOn();
    OSTimeDlyHMSM(0, 0, 0, 500, OS_OPT_TIME_HMSM_STRICT, &err);
    BSP_HydrgInValvePwrOff();

    BSP_HydrgOutValvePwrOn();
    OSTimeDlyHMSM(0, 0, 0, 500, OS_OPT_TIME_HMSM_STRICT, &err);
    BSP_HydrgOutValvePwrOff();

    BSP_DCConnectValvePwrOn();
    OSTimeDlyHMSM(0, 0, 0, 500, OS_OPT_TIME_HMSM_STRICT, &err);
    BSP_DCConnectValvePwrOff();
}

/*
***************************************************************************************************
*                                            BSP_CmdButtonInit()
*
* Description : �����������ų�ʼ��.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Note(s)     : none.
***************************************************************************************************
*/
void DiagnosticFeedBack_0_IRQHandler(void);
void DiagnosticFeedBack_1_IRQHandler(void);
void DiagnosticFeedBack_2_IRQHandler(void);
void DiagnosticFeedBack_3_IRQHandler(void);
void DiagnosticFeedBack_4_IRQHandler(void);
void DiagnosticFeedBack_9_5_IRQHandler(void);


//��ϼ�����ų�ʼ��
void BSP_DiagnosticFeedBackPortInit(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOE, ENABLE); //ʹ�ܸ��ù���ʱ��

    //�������͵������ϼ��
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;   //��������
    GPIO_InitStructure.GPIO_Pin = BSP_GPIOE_HEATER_AND_IGNITER_DIAGNOSTIC_FEEDBACK_PORT_NMB;
    GPIO_Init(GPIOE, &GPIO_InitStructure);

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOE, GPIO_PinSource0);
    EXTI_InitStructure.EXTI_Line = EXTI_Line0;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;

    //Ԥ��1��ֱ���Ӵ������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;   //��������
    GPIO_InitStructure.GPIO_Pin = BSP_GPIOE_RVD1_AND_DC_OUTPUT_DIAGNOSTIC_FEEDBACK_PORT_NMB;
    GPIO_Init(GPIOE, &GPIO_InitStructure);

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOE, GPIO_PinSource1);
    EXTI_InitStructure.EXTI_Line = EXTI_Line1;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;

    //������ŷ��ͳ�����ŷ���ϼ������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;   //��������
    GPIO_InitStructure.GPIO_Pin = BSP_GPIOE_HYDROGEN_OUTPUT_AND_INPUT_VALVE_DIAGNOSTIC_FEEDBACK_PORT_NMB;
    GPIO_Init(GPIOE, &GPIO_InitStructure);

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOE, GPIO_PinSource2);
    EXTI_InitStructure.EXTI_Line = EXTI_Line2;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;

    //Ԥ�����Ƶ�4��Ԥ�����Ƶ�3��ϼ������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;   //��������
    GPIO_InitStructure.GPIO_Pin = BSP_GPIOE_RVD4_CTRL_AND_RVD3_CTRL_DIAGNOSTIC_FEEDBACK_PORT_NMB;
    GPIO_Init(GPIOE, &GPIO_InitStructure);

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOE, GPIO_PinSource3);
    EXTI_InitStructure.EXTI_Line = EXTI_Line3;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;

    //Ԥ�����Ƶ�2�ͽ�Һ��ŷ�2��ϼ������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;   //��������
    GPIO_InitStructure.GPIO_Pin = BSP_GPIOE_RVD2_CTRL_AND_WATER_INPUT_VALVE2_DIAGNOSTIC_FEEDBACK_PORT_NMB;
    GPIO_Init(GPIOE, &GPIO_InitStructure);

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOE, GPIO_PinSource4);
    EXTI_InitStructure.EXTI_Line = EXTI_Line4;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;

    //Ԥ�����Ƶ�7��Ԥ�����Ƶ�8��ϼ������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;   //��������
    GPIO_InitStructure.GPIO_Pin = BSP_GPIOE_RVD7_CTRL_AND_RVD8_CTRL_DIAGNOSTIC_FEEDBACK_PORT_NMB;
    GPIO_Init(GPIOE, &GPIO_InitStructure);

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOE, GPIO_PinSource5);
    EXTI_InitStructure.EXTI_Line = EXTI_Line5;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;

    //Ԥ�����Ƶ�5��Ԥ�����Ƶ�6��ϼ������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;   //��������
    GPIO_InitStructure.GPIO_Pin = BSP_GPIOE_RVD5_CTRL_AND_RVD6_CTRL_DIAGNOSTIC_FEEDBACK_PORT_NMB;
    GPIO_Init(GPIOE, &GPIO_InitStructure);

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOE, GPIO_PinSource6);
    EXTI_InitStructure.EXTI_Line = EXTI_Line6;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;

    //���״̬�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;   //��������
    GPIO_InitStructure.GPIO_Pin = BSP_GPIOE_FIRE_STATUS_FEEDBACK_PORT_NMB;
    GPIO_Init(GPIOE, &GPIO_InitStructure);

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOE, GPIO_PinSource7);
    EXTI_InitStructure.EXTI_Line = EXTI_Line7;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    //�úͽ�Һ��ŷ�1��ϼ��
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;   //��������
    GPIO_InitStructure.GPIO_Pin = BSP_GPIOB_PUMP_AND_WATER_INPUT_VALVE_ONE_DIAGNOSTIC_FEEDBACK_PORT_NMB;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource9);
    EXTI_InitStructure.EXTI_Line = EXTI_Line9;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;

    BSP_IntVectSet(BSP_INT_ID_EXTI0, DiagnosticFeedBack_0_IRQHandler);
    BSP_IntEn(BSP_INT_ID_EXTI0);

    BSP_IntVectSet(BSP_INT_ID_EXTI1, DiagnosticFeedBack_1_IRQHandler);
    BSP_IntEn(BSP_INT_ID_EXTI1);

    BSP_IntVectSet(BSP_INT_ID_EXTI2, DiagnosticFeedBack_2_IRQHandler);
    BSP_IntEn(BSP_INT_ID_EXTI2);

    BSP_IntVectSet(BSP_INT_ID_EXTI3, DiagnosticFeedBack_3_IRQHandler);
    BSP_IntEn(BSP_INT_ID_EXTI3);

    BSP_IntVectSet(BSP_INT_ID_EXTI4, DiagnosticFeedBack_4_IRQHandler);
    BSP_IntEn(BSP_INT_ID_EXTI4);

    BSP_IntVectSet(BSP_INT_ID_EXTI9_5, DiagnosticFeedBack_9_5_IRQHandler);
    BSP_IntEn(BSP_INT_ID_EXTI9_5);

}


void DiagnosticFeedBack_0_IRQHandler(void)
{
    if((GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_13) == 0)) {
        if(1 == GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_7)) { //������ƽ�Ϊ��

        } else {

        }

        if(1 == GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_7)) {

        } else {

        }
    }

    EXTI_ClearITPendingBit(EXTI_Line0);
}

void DiagnosticFeedBack_1_IRQHandler(void)
{
    if((GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_13) == 0)) {
        if(1 == GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_7)) { //������ƽ�Ϊ��

        } else {

        }

        if(1 == GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_7)) {

        }
    }

    EXTI_ClearITPendingBit(EXTI_Line1);
}

void DiagnosticFeedBack_2_IRQHandler(void)
{

    EXTI_ClearITPendingBit(EXTI_Line2);
}

void DiagnosticFeedBack_3_IRQHandler(void)
{

    EXTI_ClearITPendingBit(EXTI_Line3);
}

void DiagnosticFeedBack_4_IRQHandler(void)
{

    EXTI_ClearITPendingBit(EXTI_Line4);
}

void DiagnosticFeedBack_9_5_IRQHandler(void)
{

    EXTI_ClearITPendingBit(EXTI_Line5);

    EXTI_ClearITPendingBit(EXTI_Line5);

    EXTI_ClearITPendingBit(EXTI_Line5);

    EXTI_ClearITPendingBit(EXTI_Line5);

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
