/* mbed Microcontroller Library - PeripheralNames
 * Copyright (C) 2008-2009 ARM Limited. All rights reserved.
 *
 * Provides the mappings for peripherals
 * Implementation specific to the LPC1768/LPC2368
 * sford
 */

#ifndef MBED_PERIPHERALNAMES_H
#define MBED_PERIPHERALNAMES_H

#include "cmsis.h"

#ifdef __cplusplus
extern "C" {
#endif 

typedef enum UARTName UARTName;
enum UARTName {
    UART_0 = (int)LPC_UART0_BASE
    , UART_1 = (int)LPC_UART1_BASE
    , UART_2 = (int)LPC_UART2_BASE
    , UART_3 = (int)LPC_UART3_BASE
};

typedef enum ADCName ADCName;
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

typedef enum DACName DACName;
enum DACName {
    DAC_0 = 0
};

typedef enum SPIName SPIName;
enum SPIName {
    SPI_0 = (int)LPC_SSP0_BASE
    , SPI_1 = (int)LPC_SSP1_BASE
};

typedef enum I2CName I2CName;
enum I2CName {
    I2C_0 = (int)LPC_I2C0_BASE
    , I2C_1 = (int)LPC_I2C1_BASE
    , I2C_2 = (int)LPC_I2C2_BASE
};

typedef enum PWMName PWMName;
enum PWMName {
    PWM_1 = 1
    , PWM_2 
    , PWM_3 
    , PWM_4 
    , PWM_5 
    , PWM_6 
};

typedef enum TimerName TimerName;
enum TimerName {
    TIMER_0 = (int)LPC_TIM0_BASE
    , TIMER_1 = (int)LPC_TIM1_BASE
    , TIMER_2 = (int)LPC_TIM2_BASE
    , TIMER_3 = (int)LPC_TIM3_BASE
};

typedef enum CANName CANName;
enum CANName { 
     CAN_1 = (int)LPC_CAN1_BASE,
     CAN_2 = (int)LPC_CAN2_BASE 
}; 

#define STDIO_UART_TX     USBTX
#define STDIO_UART_RX     USBRX
#define STDIO_UART        UART_0
#define US_TICKER_TIMER TIMER_3
#define US_TICKER_TIMER_IRQn TIMER3_IRQn

#ifdef __cplusplus
}
#endif 

#endif 
