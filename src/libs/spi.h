#ifndef _SPI_H
#define _SPI_H

#include <stdint.h>

#include "spi_hal.h"

class SPI {
public:
    SPI(PinName mosi, PinName miso, PinName sclk);
    ~SPI();

    void frequency(uint32_t);
    uint8_t write(uint8_t);

//     int writeblock(uint8_t *, int);

    bool can_DMA();
    int setup_DMA_rx(DMA_REG *);
    int setup_DMA_tx(DMA_REG *);

    void irq(void);

    static SPI* isr_dispatch[N_SPI_INTERRUPT_ROUTINES];

protected:
    uint32_t delay;
    Pin_t miso;
    Pin_t mosi;
    Pin_t sclk;
    SPI_REG *sspr;
};

#endif /* _SPI_H */
