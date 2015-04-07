#ifndef _SPI_HAL_H
#define _SPI_HAL_H

#include "lpc17xx_ssp.h"

#ifdef __LPC17XX__
#include <PinNames.h>
    typedef struct {
        uint8_t port;
        uint8_t pin;
    } Pin_t;

    typedef LPC_SSP_TypeDef SPI_REG;
    typedef LPC_GPDMACH_TypeDef DMA_REG;

    #define N_SPI_INTERRUPT_ROUTINES 2
#endif

#endif /* _SPI_HAL_H */
