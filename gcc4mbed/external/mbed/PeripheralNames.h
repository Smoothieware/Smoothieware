/* mbed Microcontroller Library - PeripheralNames
 * Copyright (C) 2008-2011 ARM Limited. All rights reserved.
 *
 * Provides the mappings for peripherals
 */

#ifndef MBED_PERIPHERALNAMES_H
#define MBED_PERIPHERALNAMES_H

#include "cmsis.h"

#ifdef __cplusplus
extern "C" {
#endif 

#if defined(TARGET_LPC1768) || defined(TARGET_LPC2368)

enum UARTName {
    UART_0 = (int)LPC_UART0_BASE
    , UART_1 = (int)LPC_UART1_BASE
    , UART_2 = (int)LPC_UART2_BASE
    , UART_3 = (int)LPC_UART3_BASE
};
typedef enum UARTName UARTName;

enum ADCName {
    ADC0_0 = 0
    , ADC0_1
    , ADC0_2
    , ADC0_3
    , ADC0_4
    , ADC0_5
    , ADC0_6
    , ADC0_7
};
typedef enum ADCName ADCName;

enum DACName {
    DAC_0 = 0
};
typedef enum DACName DACName;

enum SPIName {
    SPI_0 = (int)LPC_SSP0_BASE
    , SPI_1 = (int)LPC_SSP1_BASE
};
typedef enum SPIName SPIName;

enum I2CName {
    I2C_0 = (int)LPC_I2C0_BASE
    , I2C_1 = (int)LPC_I2C1_BASE
    , I2C_2 = (int)LPC_I2C2_BASE
};
typedef enum I2CName I2CName;

enum PWMName {
    PWM_1 = 1
    , PWM_2 
    , PWM_3 
    , PWM_4 
    , PWM_5 
    , PWM_6 
};
typedef enum PWMName PWMName;

enum TimerName {
    TIMER_0 = (int)LPC_TIM0_BASE
    , TIMER_1 = (int)LPC_TIM1_BASE
    , TIMER_2 = (int)LPC_TIM2_BASE
    , TIMER_3 = (int)LPC_TIM3_BASE
};
typedef enum TimerName TimerName;

enum CANName { 
     CAN_1 = (int)LPC_CAN1_BASE,
     CAN_2 = (int)LPC_CAN2_BASE 
}; 
typedef enum CANName CANName;

#define US_TICKER_TIMER TIMER_3
#define US_TICKER_TIMER_IRQn TIMER3_IRQn

#elif defined(TARGET_LPC11U24)

enum UARTName {
    UART_0 = (int)LPC_USART_BASE
};
typedef enum UARTName UARTName;

enum I2CName {
    I2C_0 = (int)LPC_I2C_BASE
};
typedef enum I2CName I2CName;

enum TimerName {
    TIMER_0 = (int)LPC_CT32B0_BASE
    , TIMER_1 = (int)LPC_CT32B1_BASE
};
typedef enum TimerName TimerName;

enum ADCName {
    ADC0_0 = 0
    , ADC0_1
    , ADC0_2
    , ADC0_3
    , ADC0_4
    , ADC0_5
    , ADC0_6
    , ADC0_7
};
typedef enum ADCName ADCName;

enum SPIName {
    SPI_0 = (int)LPC_SSP0_BASE
    , SPI_1 = (int)LPC_SSP1_BASE
};
typedef enum SPIName SPIName;

#define US_TICKER_TIMER TIMER_1
#define US_TICKER_TIMER_IRQn     TIMER_32_1_IRQn 

typedef enum PWMName {
    PWM_1 = 0
    , PWM_2
    , PWM_3
    , PWM_4
    , PWM_5
    , PWM_6
    , PWM_7
    , PWM_8
    , PWM_9
    , PWM_10
    , PWM_11
} PWMName;

#endif

#define STDIO_UART_TX     USBTX
#define STDIO_UART_RX     USBRX
#define STDIO_UART        UART_0

#ifdef __cplusplus
}
#endif 

#endif 
