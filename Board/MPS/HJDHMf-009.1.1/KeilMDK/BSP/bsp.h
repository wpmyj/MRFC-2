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
*
*                                        BOARD SUPPORT PACKAGE
*
*                                     ST Microelectronics STM32
*                                              on the
*
*                                         ENECO MRFC-2-001.0 board
***************************************************************************************************
*/
/********************************************************************************
  * @file    bsp_ans_senor.h
  * @author  Fanjun
  * @version V1.0
  * @date    6-December-2016
  * @brief   This file contains all the functions prototypes for the analog sensor
  *          firmware library.
*********************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef  BSP_PRESENT
#define  BSP_PRESENT
/*
***************************************************************************************************
* Note(s) : 定时器使用情况
*   1)定时器1：调速设备速度监测
*   2)定时器2：
*   3)定时器3：调速设备速度监测
*   4)定时器4：电堆风机速度控制
*   5)定时器5：电堆排气时间参数计时
*   6)定时器6:
*   7)定时器7：硬件按钮按下时间检测
*   8)
***************************************************************************************************
*/
/*
***************************************************************************************************
*                                                 MODULE
*
* Note(s) : (1) This header file is protected from multiple pre-processor inclusion through use of the
*               BSP present pre-processor macro definition.
***************************************************************************************************
*/

/*
***************************************************************************************************
*                                                 EXTERNS
***************************************************************************************************
*/

#ifdef   BSP_MODULE
    #define  BSP_EXT
#else
    #define  BSP_EXT  extern
#endif


/*
***************************************************************************************************
*                                              INCLUDE FILES
***************************************************************************************************
*/

#include  <stdarg.h>
#include  <stdio.h>

#include  <cpu.h>
#include  <cpu_core.h>

#include  <lib_ascii.h>
#include  <lib_def.h>
#include  <lib_mem.h>
#include  <lib_str.h>

//#include  <stm32f10x_lib.h>

#include  <stm32f10x_conf.h>

#include  <app_cfg.h>

#include  <bsp_os.h>
#include  <bsp_ser.h>
#include  <bsp_i2c.h>
#include  <bsp_ntc.h>
#include  <bsp_ana_sensor.h>


/*
***************************************************************************************************
*                                          GPIO PIN DEFINITIONS
***************************************************************************************************
*/
//管脚定义分为:组别+管脚号
/* ------------------------------------------ GPIOA PINS ---------------------------------------- */
#define  BSP_GPIOA_PIN0_LIQUID_LEVEL_ANA_SIGNAL_PORT_NMB                        DEF_BIT_00//液位传感器
#define  BSP_GPIOA_PIN1_LIQUID_PRESS_ANA_SIGNAL_PORT_NMB                        DEF_BIT_01//液压传感器
#define  BSP_GPIOA_PIN2_STACK_TEMP_ANA_SIGNAL_PORT_NMB                          DEF_BIT_02//电堆温度传感器
#define  BSP_GPIOA_PIN3_STACK_CURRENT_ANA_SIGNAL_PORT_NMB                       DEF_BIT_03//电堆电流传感器
#define  BSP_GPIOA_PIN4_PUMP_SPD_ANA_SIGNAL_CTRL_PORT_NMB                       DEF_BIT_04//泵速控制引脚
#define  BSP_GPIOA_PIN5_HYDROGEN_FAN_SPD_ANA_SIGNAL_CTRL_PORT_NMB               DEF_BIT_05//制氢风机速度控制引脚
#define  BSP_GPIOA_PIN6_BATTERY_VOLETAGE_ANA_SIGNAL_PORT_NMB                    DEF_BIT_06//电池电压传感器
#define  BSP_GPIOA_PIN7_STACK_HYDROGEN_PRESS_TWO_ANA_SIGNAL_PORT_NMB            DEF_BIT_07//气压传感器2
#define  BSP_GPIOA_PIN8_STACK_220_VOLATAGE_MONITOR_PORT_NMB                     DEF_BIT_08//交流220V电压监测
#define  BSP_GPIOA_PIN9_USART1_TX_PORT_NMB                                      DEF_BIT_09
#define  BSP_GPIOA_PIN10_USART1_RX_PORT_NMB                                     DEF_BIT_10
#define  BSP_GPIOA_PIN11_USB_DM_PORT_NMB                                        DEF_BIT_11
#define  BSP_GPIOA_PIN12_USB_DP_PORT_NMB                                        DEF_BIT_12
#define  BSP_GPIOA_PIN15_STM_SPI_NSS_PORT_NMB                                   DEF_BIT_15

/* ------------------------------------------ GPIOB PINS ---------------------------------------- */

#define  BSP_GPIOB_STACK_HYDROGEN_PRESS_ONE_ANA_SIGNAL_PORT_NMB                 DEF_BIT_00//气压传感器1
#define  BSP_GPIOB_STACK_VOLETAGE_ANA_SIGNAL_PORT_NMB                           DEF_BIT_01//电堆电压传感器
#define  BSP_GPIOB_BUZZER_CTRL_PORT_NMB                                         DEF_BIT_02
#define  BSP_GPIOB_STM_SPI_CLK_PORT_NMB                                         DEF_BIT_03
#define  BSP_GPIOB_STM_SPI_MISO_PORT_NMB                                        DEF_BIT_04
#define  BSP_GPIOB_STM_SPI_MOSI_PORT_NMB                                        DEF_BIT_05
#define  BSP_GPIOB_STACK_FAN_SPD_CTRL_PORT_NMB                                  DEF_BIT_06//电堆风机速度控制引脚
#define  BSP_GPIOB_LIQUID_INPUT_VALVE_ONE_PWR_CTRL_PORT_NMB                     DEF_BIT_07//进液电磁阀一电源控制引脚
#define  BSP_GPIOB_RSVD2_OUTPUT_PWR_CTRL_PORT_NMB                               DEF_BIT_08//预留2电源控制引脚--纯氢出口电磁阀
#define  BSP_GPIOB_PUMP_AND_WATER_INPUT_VALVE_ONE_DIAGNOSTIC_FEEDBACK_PORT_NMB  DEF_BIT_09//泵和进液电磁阀1诊断监测引脚
#define  BSP_GPIOB_PUMP_PWR_CTRL_PORT_NMB                                       DEF_BIT_10//泵电源开关引脚
#define  BSP_GPIOB_HYDROGEN_FAN_PWR_CTRL_PORT_NMB                               DEF_BIT_11//制氢机风机电源开关引脚
#define  BSP_GPIOB_DC_CONNECTER_PWR_CTRL_PORT_NMB                               DEF_BIT_12//直流接触器控制引脚
#define  BSP_GPIOB_EXTERNEL_SWITCH_INPUT_PORT_NMB                               DEF_BIT_13//外部开关机按钮
#define  BSP_GPIOB_USB_BUS_PWR_EN_CTRL_PORT_NMB                                 DEF_BIT_14//USB PWR EN
#define  BSP_GPIOB_USB_BUS_PWR_OVRCR_CTRL_PORT_NMB                              DEF_BIT_15//USB BUS PWR OVRCR

/* ------------------------------------------ GPIOC PINS ---------------------------------------- */

#define  BSP_GPIOC_HYDROGRN_CONCENTRATION_DETECTION_PORT_NMB                    DEF_BIT_00//氢气浓度监测引脚
#define  BSP_GPIOC_BATTERY_CURRENT_ANA_SIGNAL_PORT_NMB                          DEF_BIT_01//电池电流
#define  BSP_GPIOC_RAPID_HEATER_CURRETN_ANA_SIGNAL_PORT_NMB                     DEF_BIT_02//快速加热器电流
#define  BSP_GPIOC_NEGATIVE_PRESSURE_ANA_SIGNAL_PORT_NMB                        DEF_BIT_03//负压传感器
#define  BSP_GPIOC_RSVD1_ANA_SIGNAL_PORT_NMB                                    DEF_BIT_04//预留模拟信号传感器1
#define  BSP_GPIOC_RSVD2_ANA_SIGNAL_PORT_NMB                                    DEF_BIT_05//预留模拟信号传感器2
#define  BSP_GPIOC_RSVD_OUTPUT_PWR_CTRL_PORT_NMB                                DEF_BIT_06//水泵自动加液引脚控制开关
#define  BSP_GPIOC_HYDROGEN_INTO_STACK_VALVE_PWR_CTRL_PORT_NMB                  DEF_BIT_07//电堆进气控制阀
#define  BSP_GPIOC_HYDROGEN_OUTOF_STACK_VALVE_PWR_CTRL_PORT_NMB                 DEF_BIT_08//电堆出气控制阀
#define  BSP_GPIOC_STACK_FAN_PWR_CTRL_PORT_NMB                                  DEF_BIT_09//电堆风机开关引脚控制
#define  BSP_GPIOC_UART4_TX_PORT_NMB                                            DEF_BIT_10
#define  BSP_GPIOC_UART4_RX_PORT_NMB                                            DEF_BIT_11
#define  BSP_GPIOC_UART5_TX_PORT_NMB                                            DEF_BIT_12
#define  BSP_GPIOC_FAST_HEATER_PWR_CTRL_PORT_NMB                                DEF_BIT_13//快速加热器控制引脚
#define  BSP_GPIOC_LIQUID_INPUT_VALVE_TWO_PWR_CTRL_PORT_NMB                     DEF_BIT_14//进液电磁阀二控制引脚
#define  BSP_GPIOC_IGNITER_PWR_CTRL_PORT_NMB                                    DEF_BIT_15//点火器控制引脚


/* ------------------------------------------ GPIOD PINS ---------------------------------------- */
#define  BSP_GPIOD_PIN0_CAN1_RX_PORT_NMB                                        DEF_BIT_00
#define  BSP_GPIOD_PIN1_CAN1_TX_PORT_NMB                                        DEF_BIT_01
#define  BSP_GPIOD_PIN2_UART5_RX_PORT_NMB                                       DEF_BIT_02
#define  BSP_GPIOD_PIN3_FLASH_HOLD_PORT_NMB                                     DEF_BIT_03
#define  BSP_GPIOD_PIN4_FLASH_WP_PORT_NMB                                       DEF_BIT_04
#define  BSP_GPIOD_PIN5_UART2_TX_PORT_NMB                                       DEF_BIT_05
#define  BSP_GPIOD_PIN6_UART2_RX_PORT_NMB                                       DEF_BIT_06
#define  BSP_GPIOD_PIN7_RSVD8_OUTPUT_PWR_CTRL_PORT_NMB                          DEF_BIT_07
#define  BSP_GPIOD_PIN10_MAX6675_CHIPS_SELECT_PORT_NMB                          DEF_BIT_10
#define  BSP_GPIOD_PIN11_MAX6675_CHIPS_SCLK_PORT_NMB                            DEF_BIT_11
#define  BSP_GPIOD_PIN12_MAX6675_CHIP_ONE_DIG_SIGNAL_PORT_NMB                   DEF_BIT_12
#define  BSP_GPIOD_PIN13_MAX6675_CHIP_TWO_DIG_SIGNAL_PORT_NMB                   DEF_BIT_13//热电偶相关管脚                 
#define  BSP_GPIOD_PIN14_RSVD6_OUTPUT_PWR_CTRL_PORT_NMB                         DEF_BIT_14
#define  BSP_GPIOD_PIN15_RSVD7_OUTPUT_PWR_CTRL_PORT_NMB                         DEF_BIT_15


/* ------------------------------------------ GPIOE PINS ---------------------------------------- */
/*诊断检测引脚*/
#define  BSP_GPIOE_PIN0_HEATER_AND_IGNITER_DIAGNOSTIC_FEEDBACK_PORT_NMB                  DEF_BIT_00//加热器和点火器诊断监测引脚
#define  BSP_GPIOE_PIN1_RVD1_AND_DC_OUTPUT_DIAGNOSTIC_FEEDBACK_PORT_NMB                  DEF_BIT_01//预留1和直流接触器诊断
#define  BSP_GPIOE_PIN2_HYDROGEN_OUTPUT_AND_INPUT_VALVE_DIAGNOSTIC_FEEDBACK_PORT_NMB     DEF_BIT_02//进气电磁阀和出气电磁阀诊断监测引脚
#define  BSP_GPIOE_PIN3_RVD4_AND_RVD3_CTRL_DIAGNOSTIC_FEEDBACK_PORT_NMB                  DEF_BIT_03//预留控制点4和预留控制点3诊断监测引脚
#define  BSP_GPIOE_PIN4_RVD2_AND_WATER_INPUT_VALVE2_DIAGNOSTIC_FEEDBACK_PORT_NMB         DEF_BIT_04//预留控制点2和进液电磁阀2诊断监测引脚
#define  BSP_GPIOE_PIN5_RVD7_AND_RVD8_CTRL_DIAGNOSTIC_FEEDBACK_PORT_NMB                  DEF_BIT_05
#define  BSP_GPIOE_PIN6_RVD5_AND_RVD6_CTRL_DIAGNOSTIC_FEEDBACK_PORT_NMB                  DEF_BIT_06
#define  BSP_GPIOE_PIN7_FIRE_STATUS_FEEDBACK_PORT_NMB                                    DEF_BIT_07//点火状态监测引脚
#define  BSP_GPIOE_PIN8_RSVD3_OUTPUT_PWR_CTRL_PORT_NMB                                   DEF_BIT_08//尾气电磁阀开关引脚
#define  BSP_GPIOE_PIN9_PUMP_SPEED_CHECK_PORT_NMB                                        DEF_BIT_09//泵速检测引脚
#define  BSP_GPIOE_PIN10_PD_PULSE1_PORT_NMB                                              DEF_BIT_10//外部电位脉冲1
#define  BSP_GPIOE_PIN11_HYDRG_FAN_SPEED_CHECK_PORT_NMB                                  DEF_BIT_11//制氢风机速度检测引脚
#define  BSP_GPIOE_PIN12_PD_PULSE2_PORT_NMB                                              DEF_BIT_12//外部电位脉冲2
#define  BSP_GPIOE_PIN13_STACK_FAN_SPEED_CHECK_PORT_NMB                                  DEF_BIT_13//电堆风机速度检测引脚
#define  BSP_GPIOE_PIN14_RSVD5_OUTPUT_PWR_CTRL_PORT_NMB                                  DEF_BIT_14
#define  BSP_GPIOE_PIN15_RSVD4_OUTPUT_PWR_CTRL_PORT_NMB                                  DEF_BIT_15//预留口5/6/7/8 (抽真空电磁阀2/3/4/5)

/*
***************************************************************************************************
*                                               INT DEFINES
***************************************************************************************************
*/
//板级中断向量编号
#define  BSP_INT_ID_WWDG                                   0    /* Window WatchDog Interrupt                            */
#define  BSP_INT_ID_PVD                                    1    /* PVD through EXTI Line detection Interrupt            */
#define  BSP_INT_ID_TAMPER                                 2    /* Tamper Interrupt                                     */
#define  BSP_INT_ID_RTC                                    3    /* RTC global Interrupt                                 */
#define  BSP_INT_ID_FLASH                                  4    /* FLASH global Interrupt                               */
#define  BSP_INT_ID_RCC                                    5    /* RCC global Interrupt                                 */
#define  BSP_INT_ID_EXTI0                                  6    /* EXTI Line0 Interrupt                                 */
#define  BSP_INT_ID_EXTI1                                  7    /* EXTI Line1 Interrupt                                 */
#define  BSP_INT_ID_EXTI2                                  8    /* EXTI Line2 Interrupt                                 */
#define  BSP_INT_ID_EXTI3                                  9    /* EXTI Line3 Interrupt                                 */
#define  BSP_INT_ID_EXTI4                                 10    /* EXTI Line4 Interrupt                                 */
#define  BSP_INT_ID_DMA1_CH1                              11    /* DMA1 Channel 1 global Interrupt                      */
#define  BSP_INT_ID_DMA1_CH2                              12    /* DMA1 Channel 2 global Interrupt                      */
#define  BSP_INT_ID_DMA1_CH3                              13    /* DMA1 Channel 3 global Interrupt                      */
#define  BSP_INT_ID_DMA1_CH4                              14    /* DMA1 Channel 4 global Interrupt                      */
#define  BSP_INT_ID_DMA1_CH5                              15    /* DMA1 Channel 5 global Interrupt                      */
#define  BSP_INT_ID_DMA1_CH6                              16    /* DMA1 Channel 6 global Interrupt                      */
#define  BSP_INT_ID_DMA1_CH7                              17    /* DMA1 Channel 7 global Interrupt                      */
#define  BSP_INT_ID_ADC1_2                                18    /* ADC1 et ADC2 global Interrupt                        */
#define  BSP_INT_ID_CAN1_TX                               19    /* CAN1 TX Interrupts                                   */
#define  BSP_INT_ID_CAN1_RX0                              20    /* CAN1 RX0 Interrupts                                  */
#define  BSP_INT_ID_CAN1_RX1                              21    /* CAN1 RX1 Interrupt                                   */
#define  BSP_INT_ID_CAN1_SCE                              22    /* CAN1 SCE Interrupt                                   */
#define  BSP_INT_ID_EXTI9_5                               23    /* External Line[9:5] Interrupts                        */
#define  BSP_INT_ID_TIM1_BRK                              24    /* TIM1 Break Interrupt                                 */
#define  BSP_INT_ID_TIM1_UP                               25    /* TIM1 Update Interrupt                                */
#define  BSP_INT_ID_TIM1_TRG_COM                          26    /* TIM1 Trigger and Commutation Interrupt               */
#define  BSP_INT_ID_TIM1_CC                               27    /* TIM1 Capture Compare Interrupt                       */
#define  BSP_INT_ID_TIM2                                  28    /* TIM2 global Interrupt                                */
#define  BSP_INT_ID_TIM3                                  29    /* TIM3 global Interrupt                                */
#define  BSP_INT_ID_TIM4                                  30    /* TIM4 global Interrupt                                */
#define  BSP_INT_ID_I2C1_EV                               31    /* I2C1 Event Interrupt                                 */
#define  BSP_INT_ID_I2C1_ER                               32    /* I2C1 Error Interrupt                                 */
#define  BSP_INT_ID_I2C2_EV                               33    /* I2C2 Event Interrupt                                 */
#define  BSP_INT_ID_I2C2_ER                               34    /* I2C2 Error Interrupt                                 */
#define  BSP_INT_ID_SPI1                                  35    /* SPI1 global Interrupt                                */
#define  BSP_INT_ID_SPI2                                  36    /* SPI2 global Interrupt                                */
#define  BSP_INT_ID_USART1                                37    /* USART1 global Interrupt                              */
#define  BSP_INT_ID_USART2                                38    /* USART2 global Interrupt                              */
#define  BSP_INT_ID_USART3                                39    /* USART3 global Interrupt                              */
#define  BSP_INT_ID_EXTI15_10                             40    /* External Line[15:10] Interrupts                      */
#define  BSP_INT_ID_RTC_ALARM                             41    /* RTC Alarm through EXTI Line Interrupt                */
#define  BSP_INT_ID_OTG_FS_WKUP                           42    /* USB WakeUp from suspend through EXTI Line Interrupt  */

#define  BSP_INT_ID_TIM5                                  50    /* TIM5 global Interrupt                                */
#define  BSP_INT_ID_SPI3                                  51    /* SPI3 global Interrupt                                */
#define  BSP_INT_ID_USART4                                52    /* USART4 global Interrupt                              */
#define  BSP_INT_ID_USART5                                53    /* USRT5 global Interrupt                               */
#define  BSP_INT_ID_TIM6                                  54    /* TIM6 global Interrupt                                */
#define  BSP_INT_ID_TIM7                                  55    /* TIM7 global Interrupt                                */
#define  BSP_INT_ID_DMA2_CH1                              56    /* DMA2 Channel 1 global Interrupt                      */
#define  BSP_INT_ID_DMA2_CH2                              57    /* DMA2 Channel 2 global Interrupt                      */
#define  BSP_INT_ID_DMA2_CH3                              58    /* DMA2 Channel 3 global Interrupt                      */
#define  BSP_INT_ID_DMA2_CH4                              59    /* DMA2 Channel 4 global Interrupt                      */
#define  BSP_INT_ID_DMA2_CH5                              60    /* DMA2 Channel 5 global Interrupt                      */

#define  BSP_INT_ID_ETH                                   61    /* ETH  global Interrupt                                */
#define  BSP_INT_ID_ETH_WKUP                              62    /* ETH  WakeUp from EXTI line interrupt                 */
#define  BSP_INT_ID_CAN2_TX                               63    /* CAN2 TX Interrupts                                   */
#define  BSP_INT_ID_CAN2_RX0                              64    /* CAN2 RX0 Interrupts                                  */
#define  BSP_INT_ID_CAN2_RX1                              65    /* CAN2 RX1 Interrupt                                   */
#define  BSP_INT_ID_CAN2_SCE                              66    /* CAN2 SCE Interrupt                                   */
#define  BSP_INT_ID_OTG_FS                                67    /* OTG global Interrupt                                 */


/*
***************************************************************************************************
*                                             PERIPH DEFINES
***************************************************************************************************
*/
//外部设备ID
#define  BSP_PERIPH_ID_DMA1                                0
#define  BSP_PERIPH_ID_DMA2                                1
#define  BSP_PERIPH_ID_SRAM                                2
#define  BSP_PERIPH_ID_FLITF                               4
#define  BSP_PERIPH_ID_CRC                                 6
#define  BSP_PERIPH_ID_OTGFS                              12
#define  BSP_PERIPH_ID_ETHMAC                             14
#define  BSP_PERIPH_ID_ETHMACTX                           15

#define  BSP_PERIPH_ID_AFIO                               32
#define  BSP_PERIPH_ID_IOPA                               34
#define  BSP_PERIPH_ID_IOPB                               35
#define  BSP_PERIPH_ID_IOPC                               36
#define  BSP_PERIPH_ID_IOPD                               37
#define  BSP_PERIPH_ID_IOPE                               38
#define  BSP_PERIPH_ID_ADC1                               41
#define  BSP_PERIPH_ID_ADC2                               42
#define  BSP_PERIPH_ID_TIM1                               43
#define  BSP_PERIPH_ID_SPI1                               44
#define  BSP_PERIPH_ID_USART1                             46

#define  BSP_PERIPH_ID_TIM2                               64
#define  BSP_PERIPH_ID_TIM3                               65
#define  BSP_PERIPH_ID_TIM4                               66
#define  BSP_PERIPH_ID_TIM5                               67
#define  BSP_PERIPH_ID_TIM6                               68
#define  BSP_PERIPH_ID_TIM7                               69
#define  BSP_PERIPH_ID_WWDG                               75
#define  BSP_PERIPH_ID_SPI2                               78
#define  BSP_PERIPH_ID_SPI3                               79
#define  BSP_PERIPH_ID_USART2                             81
#define  BSP_PERIPH_ID_USART3                             82
#define  BSP_PERIPH_ID_USART4                             83
#define  BSP_PERIPH_ID_USART5                             84
#define  BSP_PERIPH_ID_I2C1                               85
#define  BSP_PERIPH_ID_I2C2                               86
#define  BSP_PERIPH_ID_CAN1                               89
#define  BSP_PERIPH_ID_CAN2                               90
#define  BSP_PERIPH_ID_BKP                                91
#define  BSP_PERIPH_ID_PWR                                92
#define  BSP_PERIPH_ID_DAC                                93


/*
***************************************************************************************************
*                                               DATA TYPES
***************************************************************************************************
*/


/*
***************************************************************************************************
*                                            GLOBAL VARIABLES
***************************************************************************************************
*/


/*
***************************************************************************************************
*                                                 MACRO'S
***************************************************************************************************
*/


/*
***************************************************************************************************
*                                           FUNCTION PROTOTYPES
***************************************************************************************************
*/

void         BSP_Init(void);

void         BSP_IntDisAll(void);

CPU_INT32U   BSP_CPU_ClkFreq(void);


/*
***************************************************************************************************
*                                           INTERRUPT SERVICES
***************************************************************************************************
*/

void         BSP_IntInit(void);

void         BSP_IntEn(CPU_DATA       int_id);

void         BSP_IntDis(CPU_DATA       int_id);

void         BSP_IntClr(CPU_DATA       int_id);

void         BSP_IntVectSet(CPU_DATA       int_id,
                            CPU_FNCT_VOID  isr);

void         BSP_IntPrioSet(CPU_DATA       int_id,
                            CPU_INT08U     prio);

void         BSP_IntHandlerWWDG(void);
void         BSP_IntHandlerPVD(void);
void         BSP_IntHandlerTAMPER(void);
void         BSP_IntHandlerRTC(void);
void         BSP_IntHandlerFLASH(void);
void         BSP_IntHandlerRCC(void);
void         BSP_IntHandlerEXTI0(void);
void         BSP_IntHandlerEXTI1(void);
void         BSP_IntHandlerEXTI2(void);
void         BSP_IntHandlerEXTI3(void);
void         BSP_IntHandlerEXTI4(void);
void         BSP_IntHandlerDMA1_CH1(void);
void         BSP_IntHandlerDMA1_CH2(void);
void         BSP_IntHandlerDMA1_CH3(void);
void         BSP_IntHandlerDMA1_CH4(void);
void         BSP_IntHandlerDMA1_CH5(void);

void         BSP_IntHandlerDMA1_CH6(void);
void         BSP_IntHandlerDMA1_CH7(void);
void         BSP_IntHandlerADC1_2(void);
void         BSP_IntHandlerCAN1_TX(void);
void         BSP_IntHandlerCAN1_RX0(void);
void         BSP_IntHandlerCAN1_RX1(void);
void         BSP_IntHandlerCAN1_SCE(void);
void         BSP_IntHandlerEXTI9_5(void);
void         BSP_IntHandlerTIM1_BRK(void);
void         BSP_IntHandlerTIM1_UP(void);
void         BSP_IntHandlerTIM1_TRG_COM(void);
void         BSP_IntHandlerTIM1_CC(void);
void         BSP_IntHandlerTIM2(void);
void         BSP_IntHandlerTIM3(void);
void         BSP_IntHandlerTIM4(void);
void         BSP_IntHandlerI2C1_EV(void);

void         BSP_IntHandlerI2C1_ER(void);
void         BSP_IntHandlerI2C2_EV(void);
void         BSP_IntHandlerI2C2_ER(void);
void         BSP_IntHandlerSPI1(void);
void         BSP_IntHandlerSPI2(void);
void         BSP_IntHandlerUSART1(void);
void         BSP_IntHandlerUSART2(void);
void         BSP_IntHandlerUSART3(void);
void         BSP_IntHandlerEXTI15_10(void);
void         BSP_IntHandlerRTCAlarm(void);
void         BSP_IntHandlerUSBWakeUp(void);

void         BSP_IntHandlerTIM5(void);
void         BSP_IntHandlerSPI3(void);
void         BSP_IntHandlerUSART4(void);
void         BSP_IntHandlerUSART5(void);
void         BSP_IntHandlerTIM6(void);
void         BSP_IntHandlerTIM7(void);
void         BSP_IntHandlerDMA2_CH1(void);
void         BSP_IntHandlerDMA2_CH2(void);
void         BSP_IntHandlerDMA2_CH3(void);
void         BSP_IntHandlerDMA2_CH4(void);
void         BSP_IntHandlerDMA2_CH5(void);
void         BSP_IntHandlerETH(void);
void         BSP_IntHandlerETHWakeup(void);
void         BSP_IntHandlerCAN2_TX(void);
void         BSP_IntHandlerCAN2_RX0(void);
void         BSP_IntHandlerCAN2_RX1(void);
void         BSP_IntHandlerCAN2_SCE(void);
void         BSP_IntHandlerOTG(void);


/*
***************************************************************************************************
*                                     PERIPHERAL POWER/CLOCK SERVICES
***************************************************************************************************
*/

CPU_INT32U   BSP_PeriphClkFreqGet(CPU_DATA       pwr_clk_id);

void         BSP_PeriphEn(CPU_DATA       pwr_clk_id);

void         BSP_PeriphDis(CPU_DATA       pwr_clk_id);


/*
***************************************************************************************************
*                                              LED SERVICES
***************************************************************************************************
*/

void         BSP_LED_On(CPU_INT08U     led);

void         BSP_LED_Off(CPU_INT08U     led);

void         BSP_LED_Toggle(CPU_INT08U     led);

/*
***************************************************************************************************
*                                              ANALOG SENSOR SERVICES
***************************************************************************************************
*/
void  BSP_AnaSensorConvertStart(vu16 *, uint8_t);
/*
***************************************************************************************************
*                                              DIGLOG SENSOR SERVICES
***************************************************************************************************
*/
void  DigTempSensorConvertStart(void);
void  BSP_MAX6675ConvertStart(void);
void  BSP_MAX6675_Temp_Read(float *, uint8_t *);

/*
***************************************************************************************************
*                                              SWITCH-TYPE DEVICE SERVICES
***************************************************************************************************
*/

void  BSP_BuzzerOn(void);

void  BSP_BuzzerOff(void);

void  BSP_BuzzerTurnover(void);

void  BSP_LqdValve1_PwrOn(void);
void  BSP_LqdValve1_PwrOff(void);

void  BSP_LqdValve2_PwrOn(void);
void  BSP_LqdValve2_PwrOff(void);

void  BSP_IgniterPwrOn(void);
void  BSP_IgniterPwrOff(void);

void  BSP_KeepWarmHeaterPwrOn(void);
void  BSP_KeepWarmHeaterPwrOff(void);

void  BSP_FastHeaterPwrOn(void);
void  BSP_FastHeaterPwrOff(void);

void  BSP_FastHeaterPwrOn(void);
void  BSP_FastHeaterPwrOff(void);

void  BSP_HydrgInValvePwrOn(void);
void  BSP_HydrgInValvePwrOff(void);

void  BSP_HydrgOutValvePwrOn(void);
void  BSP_HydrgOutValvePwrOff(void);

void  BSP_DCConnectValvePwrOn(void);
void  BSP_DCConnectValvePwrOff(void);

void  BSP_OutsidePumpPwrOn(void);
void  BSP_OutsidePumpPwrOff(void);

void  BSP_StackShortCircuitActivationOn(void);
void  BSP_StackShortCircuitActivationOff(void);

void  BSP_PureHydrogenGasOutValvePwrOn(void);
void  BSP_PureHydrogenGasOutValvePwrOff(void);

void  BSP_TailGasOutValvePwrOn(void);
void  BSP_TailGasOutValvePwrOff(void);
    
void  BSP_MakeVavuumValve2PwrOn(void);
void  BSP_MakeVavuumValve3PwrOn(void);
void  BSP_MakeVavuumValve4PwrOn(void);
void  BSP_MakeVavuumPumpPwrOn(void);

void  BSP_MakeVavuumValve2PwrOff(void);
void  BSP_MakeVavuumValve3PwrOff(void);
void  BSP_MakeVavuumValve4PwrOff(void);
void  BSP_MakeVavuumPumpPwrOff(void);

/*
***************************************************************************************************
*                                              SPEED CONTROL DEVICE SERVICES
***************************************************************************************************
*/
void  BSP_SetPumpSpd(uint16_t);
void  BSP_SetHydrgFanSpd(uint16_t);
void  BSP_SetStackFanSpd(uint16_t);
/*
***************************************************************************************************
*                                              Running Venting SERVICES
***************************************************************************************************
*/

void BSP_VentingIntervalRecordTimerInit(void);
void BSP_StartRunningVentingTimeRecord(void);
void BSP_StopRunningVentingTimeRecord(void);

/*
***************************************************************************************************
*                                              Others
***************************************************************************************************
*/
void  BSP_BuzzerInit(void);

void  BSP_CmdButtonInit(void);
void  EXTI15_10_IRQHandler(void);
void  CmdButtonFuncDisable(void);
void  CmdButtonFuncEnable(void);
void  TIM7_DlyMilSecondsInit(u16 i_u16DlyMilSeconds);
void  CmdButtonStatuCheck(void);
void  StartCmdButtonActionCheckDly(void);
void  TIM7_DlyMilSecondsInit(u16 i_u16DlyMilSeconds);
/*
***************************************************************************************************
*                                              STATUS INPUTS
***************************************************************************************************
*/

CPU_BOOLEAN  BSP_StatusRd(CPU_INT08U  id);

/*
***************************************************************************************************
*                                             MODULE END
***************************************************************************************************
*/


#endif                                                          /* End of module include.                               */

