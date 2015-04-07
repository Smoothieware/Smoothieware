#include "spi.h"

#include "lpc17xx_clkpwr.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_ssp.h"
#include "lpc17xx_gpio.h"

#include <stdio.h>

SPI* SPI::isr_dispatch[N_SPI_INTERRUPT_ROUTINES];

class DMA;

SPI::SPI(PinName mosi, PinName miso, PinName sclk)
{
    this->mosi.port = (mosi >> 5) & 7;
    this->mosi.pin = mosi & 0x1F;

    this->miso.port = (miso >> 5) & 7;
    this->miso.pin = miso & 0x1F;

    this->sclk.port = (sclk >> 5) & 7;
    this->sclk.pin = sclk & 0x1F;

    FIO_SetDir(this->mosi.port, 1UL << this->mosi.pin, 1);
    FIO_SetDir(this->miso.port, 1UL << this->miso.pin, 0);
    FIO_SetDir(this->sclk.port, 1UL << this->sclk.pin, 1);

    if (mosi == P0_9 && miso == P0_8 && sclk == P0_7)
    {
//         iprintf("SPI: using 0.7,0.8,0.9 with SSP1\n");
        // SSP1 on 0.7,0.8,0.9
        sspr = LPC_SSP1;
        isr_dispatch[1] = this;

        LPC_PINCON->PINSEL0 &= ~((3 << (7*2)) | (3 << (8*2)) | (3 << (9*2)));
        LPC_PINCON->PINSEL0 |=  ((2 << (7*2)) | (2 << (8*2)) | (2 << (9*2)));

        LPC_SC->PCLKSEL0 &= 0xFFCFFFFF;
        LPC_SC->PCLKSEL0 |= 0x00100000;

        LPC_SC->PCONP |= CLKPWR_PCONP_PCSSP1;
    }
    else if (mosi == P0_18 && miso == P0_17 && sclk == P0_15)
    {
//         iprintf("SPI: using 0.15,0.17,0.18 with SSP0\n");
        // SSP0 on 0.15,0.16,0.17,0.18
        sspr = LPC_SSP0;
        isr_dispatch[0] = this;

        LPC_PINCON->PINSEL0 &= ~(3 << (15*2));
        LPC_PINCON->PINSEL0 |=  (2 << (15*2));
        LPC_PINCON->PINSEL1 &= ~( (3 << ((17*2)&30)) | (3 << ((18*2)&30)) );
        LPC_PINCON->PINSEL1 |=  ( (2 << ((17*2)&30)) | (2 << ((18*2)&30)) );

        LPC_SC->PCLKSEL1 &= 0xFFFFF3FF;
        LPC_SC->PCLKSEL1 |= 0x00000400;

        LPC_SC->PCONP |= CLKPWR_PCONP_PCSSP0;
    }
    else if (mosi == P1_24 && miso == P1_23 && sclk == P1_20)
    {
//         iprintf("SPI: using 1.20,1.23,1.24 with SSP0\n");
        // SSP0 on 1.20,1.23,1.24
        sspr = LPC_SSP0;
        isr_dispatch[0] = this;

// //         LPC_PINCON->PINSEL3 &= 0xFFFC3CFF;
//         LPC_PINCON->PINSEL3 |= 0x0003C300;

//         LPC_PINCON->PINSEL3 &= ~( (3 << ((20*2)&30)) | (3 << ((23*2)&30)) | (3 << ((24*2)&30)) );
        LPC_PINCON->PINSEL3 |=  ( (3 << ((20*2)&30)) | (3 << ((23*2)&30)) | (3 << ((24*2)&30)) );

        LPC_SC->PCLKSEL1 &= 0xFFFFF3FF;
        LPC_SC->PCLKSEL1 |= 0x00000400;

        LPC_SC->PCONP |= CLKPWR_PCONP_PCSSP0;
    }
    else
    {
//         iprintf("SPI: using soft-SPI\n");
        sspr = (LPC_SSP_TypeDef *) 0;
    }

    if (sspr) {
        sspr->CR0 = SSP_DATABIT_8 |
                    SSP_FRAME_SPI;
        sspr->CR1 = SSP_MASTER_MODE;
        frequency(10000);
        sspr->CR1 |= SSP_CR1_SSP_EN;
    }
}

SPI::~SPI()
{
    if (sspr == LPC_SSP0)
        LPC_SC->PCONP &= CLKPWR_PCONP_PCSSP0;
    else if (sspr == LPC_SSP1)
        LPC_SC->PCONP &= CLKPWR_PCONP_PCSSP1;
}

void SPI::frequency(uint32_t f)
{
    // CCLK = 25MHz
    // CPSR = 2 to 254, even only
    // CR0[8:15] (SCR, 0..255) is a further prescale

//     iprintf("SPI: frequency %lu:", f);
    delay = 25000000 / f;
    // f = 25MHz / (CPSR . [SCR + 1])
    // CPSR . (SCR + 1) = 25MHz / f
    // min freq is 25MHz / (254 * 256)
    if (sspr) {
        if (f < 385) {
            sspr->CPSR = 254;
            sspr->CR0 &= 0x00FF;
            sspr->CR0 |= 255 << 8;
        }
        // max freq is 25MHz / (2 * 1)
        else if (f > 12500000) {
            sspr->CPSR = 2;
            sspr->CR0 &= 0x00FF;
        }
        else {
            sspr->CPSR = delay & 0xFE;
            // CPSR . (SCR + 1) = f;
            // (SCR + 1) = f / CPSR;
            // SCR = (f / CPSR) - 1
            sspr->CR0 &= 0x00FF;
            sspr->CR0 |= (((delay / sspr->CPSR) - 1) & 0xFF) << 8;
        }
//         iprintf(" CPSR=%lu, CR0=%lu", sspr->CPSR, sspr->CR0);
    }
//    iprintf("\n");
}

void _delay(uint32_t ticks) {
    for (;ticks;ticks--)
        asm volatile("nop\n\t");
}

uint8_t SPI::write(uint8_t data)
{
//     _cs = 1;
    uint8_t r = 0;
//     iprintf("SPI: >0x%02X", data);
    if (sspr) {
        while ((sspr->SR & SSP_SR_TNF) == 0);
        sspr->DR = data;
        while ((sspr->SR & SSP_SR_RNE) == 0);
        r = sspr->DR & 255;
    }
    else {
        for (int i = 0; i < 8; i++) {
            FIO_ClearValue(sclk.port, 1UL << sclk.pin);         // clock LOW

            if (data & 0x80)                                    // WRITE
                FIO_SetValue(mosi.port, 1UL << mosi.pin);
            else
                FIO_ClearValue(mosi.port, 1UL << mosi.pin);
            data <<= 1;

            _delay(delay >> 1);                                 // DELAY

            FIO_SetValue(sclk.port, 1UL << sclk.pin);           // clock HIGH

            _delay(delay >> 1);                                 // DELAY

            r <<= 1;
            if (FIO_ReadValue(miso.port) & (1UL << miso.pin))   // READ
                r |= 1;
        }
        FIO_ClearValue(sclk.port, 1UL << sclk.pin);
    }
//     iprintf(" <0x%02X\n", r);
    return r;
}

// TODO: timer feeds DMA feeds 0xFFs to card then we listen for responses using our interrupt
// allow me to do something like:
// disk.start_multi_write(int blocks, int blocksize, void *buffer);
// enable_usb_isr();
// [...]
// usb_isr() {
//    if (disk.buffer_in_use(void *buffer))
//        return;
//    usb_ep_read(buffer);
//    if (buffer_full)
//        disk.validate_buffer(buffer);
//    if (disk.finished_transfer())
//        disk.end_multi_write();
// };

bool SPI::can_DMA()
{
    return (sspr != NULL);
}

// int SPI::setup_DMA_rx(DMA_REG *dma)
// {
//     if (!sspr)
//         return -1;
//
//     dma->DMACCControl = 0;
//     dma->DMACCConfiguration = 0;
//     if (sspr == LPC_SSP0)
//         dma->DMACCConfiguration |= (GPDMA_CONN_SSP0_Rx << 6);
//     if (sspr == LPC_SSP1)
//         dma->DMACCConfiguration |= (GPDMA_CONN_SSP1_Rx << 6);
//
//     dma->DMACCConfiguration |= GPDMA_TRANSFERTYPE_M2P << 11;
//     return 0;
// }
//
// int SPI::start_DMA_rx(DMA_REG *dma)
// {
//     dma->DMACCConfiguration |=
// }

// int SPI::writeblock(uint8_t *block, int blocklen)
// {
//     static DMA *d = new DMA();
//     d.sourceaddr(block);
//     d.transferlength(blocklen);
//     d.destinationperipheral(sspr);
//     d.start();
//     while (d.active());
//     return blocklen;
//     return 0;
// }

void SPI::irq()
{
}

void SSP0_IRQHandler(void) {
    if (SPI::isr_dispatch[0])
        SPI::isr_dispatch[0]->irq();
}

void SSP1_IRQHandler(void) {
    if (SPI::isr_dispatch[1])
        (SPI::isr_dispatch[1])->irq();
}
