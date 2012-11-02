/* mbed Microcontroller Library - PinNames
 * Copyright (C) 2008-2011 ARM Limited. All rights reserved.
 *
 * Provides the mapping of mbed DIP and LPC Pin Names
 */

#ifndef MBED_PINNAMES_H
#define MBED_PINNAMES_H

#include "cmsis.h"

#ifdef __cplusplus
extern "C" {
#endif 

#if defined(TARGET_LPC1768) || defined(TARGET_LPC2368)

enum PinName {

    // LPC Pin Names
    P0_0 = LPC_GPIO0_BASE, P0_1, P0_2, P0_3, P0_4, P0_5, P0_6, P0_7
      , P0_8, P0_9, P0_10, P0_11, P0_12, P0_13, P0_14, P0_15
      , P0_16, P0_17, P0_18, P0_19, P0_20, P0_21, P0_22, P0_23
      , P0_24, P0_25, P0_26, P0_27, P0_28, P0_29, P0_30, P0_31
      , P1_0, P1_1, P1_2, P1_3, P1_4, P1_5, P1_6, P1_7
      , P1_8, P1_9, P1_10, P1_11, P1_12, P1_13, P1_14, P1_15
      , P1_16, P1_17, P1_18, P1_19, P1_20, P1_21, P1_22, P1_23
      , P1_24, P1_25, P1_26, P1_27, P1_28, P1_29, P1_30, P1_31
      , P2_0, P2_1, P2_2, P2_3, P2_4, P2_5, P2_6, P2_7
      , P2_8, P2_9, P2_10, P2_11, P2_12, P2_13, P2_14, P2_15
      , P2_16, P2_17, P2_18, P2_19, P2_20, P2_21, P2_22, P2_23
      , P2_24, P2_25, P2_26, P2_27, P2_28, P2_29, P2_30, P2_31
      , P3_0, P3_1, P3_2, P3_3, P3_4, P3_5, P3_6, P3_7
      , P3_8, P3_9, P3_10, P3_11, P3_12, P3_13, P3_14, P3_15
      , P3_16, P3_17, P3_18, P3_19, P3_20, P3_21, P3_22, P3_23
      , P3_24, P3_25, P3_26, P3_27, P3_28, P3_29, P3_30, P3_31
      , P4_0, P4_1, P4_2, P4_3, P4_4, P4_5, P4_6, P4_7
      , P4_8, P4_9, P4_10, P4_11, P4_12, P4_13, P4_14, P4_15
      , P4_16, P4_17, P4_18, P4_19, P4_20, P4_21, P4_22, P4_23
      , P4_24, P4_25, P4_26, P4_27, P4_28, P4_29, P4_30, P4_31

    // mbed DIP Pin Names
      , p5 = P0_9 
      , p6 = P0_8
      , p7 = P0_7
      , p8 = P0_6
      , p9 = P0_0
     , p10 = P0_1
      , p11 = P0_18
      , p12 = P0_17
     , p13 = P0_15
      , p14 = P0_16
      , p15 = P0_23
      , p16 = P0_24
      , p17 = P0_25
      , p18 = P0_26
      , p19 = P1_30
      , p20 = P1_31
      , p21 = P2_5
      , p22 = P2_4
      , p23 = P2_3
      , p24 = P2_2
      , p25 = P2_1
      , p26 = P2_0
      , p27 = P0_11
      , p28 = P0_10
      , p29 = P0_5
      , p30 = P0_4

    // Other mbed Pin Names
#ifdef MCB1700
      , LED1 = P1_28
      , LED2 = P1_29
      , LED3 = P1_31
      , LED4 = P2_2
#else 
      , LED1 = P1_18
      , LED2 = P1_20
      , LED3 = P1_21
      , LED4 = P1_23
#endif
      , USBTX = P0_2
      , USBRX = P0_3

      // Not connected
    , NC = (int)0xFFFFFFFF

};
typedef enum PinName PinName;

enum PinMode {
    PullUp = 0
    , PullDown = 3
    , PullNone = 2
    , OpenDrain = 4
};
typedef enum PinMode PinMode;

// version of PINCON_TypeDef using register arrays
typedef struct {
  __IO uint32_t PINSEL[11];
       uint32_t RESERVED0[5];
  __IO uint32_t PINMODE[10];
#ifndef TARGET_LPC2368
// Open drain mode is not available on LPC2368
  __IO uint32_t PINMODE_OD[5];
#endif
} PINCONARRAY_TypeDef;

#define PINCONARRAY ((PINCONARRAY_TypeDef *)LPC_PINCON_BASE)


#elif defined(TARGET_LPC11U24)

enum PinName {

    // LPC11U Pin Names
   P0_0 = 0
  , P0_1 = 1
  , P0_2 = 2
  , P0_3 = 3
  , P0_4 = 4
  , P0_5 = 5
  , P0_6 = 6
  , P0_7 = 7
  , P0_8 = 8
  , P0_9 = 9
  , P0_10 = 10
  , P0_11 = 11
  , P0_12 = 12
  , P0_13 = 13
  , P0_14 = 14
  , P0_15 = 15
  , P0_16 = 16
  , P0_17 = 17
  , P0_18 = 18
  , P0_19 = 19
  , P0_20 = 20
  , P0_21 = 21
  , P0_22 = 22
  , P0_23 = 23
  , P0_24 = 24
  , P0_25 = 25
  , P0_26 = 26
  , P0_27 = 27

  , P1_0 = 32
  , P1_1 = 33
  , P1_2 = 34
  , P1_3 = 35
  , P1_4 = 36
  , P1_5 = 37
  , P1_6 = 38
  , P1_7 = 39
  , P1_8 = 40
  , P1_9 = 41
  , P1_10 = 42
  , P1_11 = 43
  , P1_12 = 44
  , P1_13 = 45
  , P1_14 = 46
  , P1_15 = 47
  , P1_16 = 48
  , P1_17 = 49
  , P1_18 = 50
  , P1_19 = 51
  , P1_20 = 52
  , P1_21 = 53
  , P1_22 = 54
  , P1_23 = 55
  , P1_24 = 56
  , P1_25 = 57
  , P1_26 = 58
  , P1_27 = 59
  , P1_28 = 60
  , P1_29 = 61

  , P1_31 = 63

    // mbed DIP Pin Names
      , p5  = P0_9
      , p6  = P0_8
      , p7  = P1_29
      , p8  = P0_2
      , p9  = P1_27
      , p10 = P1_26
      , p11 = P1_22
      , p12 = P1_21
      , p13 = P1_20
      , p14 = P1_23
      , p15 = P0_11
      , p16 = P0_12
      , p17 = P0_13
      , p18 = P0_14
      , p19 = P0_16
      , p20 = P0_22
      , p21 = P0_7
      , p22 = P0_17
      , p23 = P1_17
      , p24 = P1_18
      , p25 = P1_24
      , p26 = P1_25
      , p27 = P0_4
      , p28 = P0_5
      , p29 = P1_5
      , p30 = P1_2

      , p33 = P0_3
      , p34 = P1_15
      , p35 = P0_20
      , p36 = P0_21

    // Other mbed Pin Names
      , LED1 = P1_8
      , LED2 = P1_9
      , LED3 = P1_10
      , LED4 = P1_11

      , USBTX = P0_19
      , USBRX = P0_18

      // Not connected
    , NC = (int)0xFFFFFFFF

};
typedef enum PinName PinName;

typedef enum {
    CHANNEL0=FLEX_INT0_IRQn,
    CHANNEL1=FLEX_INT1_IRQn,
    CHANNEL2=FLEX_INT2_IRQn,
    CHANNEL3=FLEX_INT3_IRQn,
    CHANNEL4=FLEX_INT4_IRQn,
    CHANNEL5=FLEX_INT5_IRQn,
    CHANNEL6=FLEX_INT6_IRQn,
    CHANNEL7=FLEX_INT7_IRQn
} Channel;

enum PinMode {
    PullUp = 2
    , PullDown = 1
    , PullNone = 0
    , Repeater = 3
    , OpenDrain = 4
};
typedef enum PinMode PinMode;
#endif


#ifdef __cplusplus
}
#endif 

#endif 
