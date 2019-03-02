/* mbed Library - ADC
 * Copyright (c) 2010, sblandford
 * released under MIT license http://mbed.org/licence/mit
 */
#include "mbed.h"
#include "adc.h"

using namespace mbed;

ADC *ADC::instance;

ADC::ADC(int sample_rate, int cclk_div)
    {

    int i, adc_clk_freq, pclk, clock_div, max_div=1;

    //Work out CCLK
    adc_clk_freq=CLKS_PER_SAMPLE*sample_rate;
    int m = (LPC_SC->PLL0CFG & 0xFFFF) + 1;
    int n = (LPC_SC->PLL0CFG >> 16) + 1;
    int cclkdiv = LPC_SC->CCLKCFG + 1;
    int Fcco = (2 * m * XTAL_FREQ) / n;
    int cclk = Fcco / cclkdiv;

    //Power up the ADC
    LPC_SC->PCONP |= (1 << 12);
    //Set clock at cclk / 1.
    LPC_SC->PCLKSEL0 &= ~(0x3 << 24);
    switch (cclk_div) {
        case 1:
            LPC_SC->PCLKSEL0 |= 0x1 << 24;
            break;
        case 2:
            LPC_SC->PCLKSEL0 |= 0x2 << 24;
            break;
        case 4:
            LPC_SC->PCLKSEL0 |= 0x0 << 24;
            break;
        case 8:
            LPC_SC->PCLKSEL0 |= 0x3 << 24;
            break;
        default:
            printf("ADC Warning: ADC CCLK clock divider must be 1, 2, 4 or 8. %u supplied.\n",
                cclk_div);
            printf("Defaulting to 1.\n");
            LPC_SC->PCLKSEL0 |= 0x1 << 24;
            break;
    }
    pclk = cclk / cclk_div;
    clock_div=pclk / adc_clk_freq;

    if (clock_div > 0xFF) {
        printf("ADC Warning: Clock division is %u which is above 255 limit. Re-Setting at limit.\n", clock_div);
        clock_div=0xFF;
    }
    if (clock_div == 0) {
        printf("ADC Warning: Clock division is 0. Re-Setting to 1.\n");
        clock_div=1;
    }

    _adc_clk_freq=pclk / clock_div;
    if (_adc_clk_freq > MAX_ADC_CLOCK) {
        printf("ADC Warning: Actual ADC sample rate of %u which is above %u limit\n",
            _adc_clk_freq / CLKS_PER_SAMPLE, MAX_ADC_CLOCK / CLKS_PER_SAMPLE);
        while ((pclk / max_div) > MAX_ADC_CLOCK) max_div++;
        printf("ADC Warning: Maximum recommended sample rate is %u\n", (pclk / max_div) / CLKS_PER_SAMPLE);
    }

    LPC_ADC->ADCR =
        ((clock_div - 1 ) << 8 ) |    //Clkdiv
        ( 1 << 21 );                  //A/D operational

    //Default no channels enabled
    LPC_ADC->ADCR &= ~0xFF;
    //Default NULL global custom isr
    _adc_g_isr = NULL;
    //Initialize arrays
    for (i=7; i>=0; i--) {
        _adc_data[i] = 0;
        _adc_isr[i] = NULL;
    }


    //* Attach IRQ
    instance = this;
    NVIC_SetVector(ADC_IRQn, (uint32_t)&_adcisr);

    //Disable global interrupt
    LPC_ADC->ADINTEN &= ~0x100;

};

void ADC::_adcisr(void)
{
    instance->adcisr();
}


void ADC::adcisr(void)
{
    uint32_t stat;
    int chan;

    // Read status
    stat = LPC_ADC->ADSTAT;
    //Scan channels for over-run or done and update array
    if (stat & 0x0101) _adc_data[0] = LPC_ADC->ADDR0;
    if (stat & 0x0202) _adc_data[1] = LPC_ADC->ADDR1;
    if (stat & 0x0404) _adc_data[2] = LPC_ADC->ADDR2;
    if (stat & 0x0808) _adc_data[3] = LPC_ADC->ADDR3;
    if (stat & 0x1010) _adc_data[4] = LPC_ADC->ADDR4;
    if (stat & 0x2020) _adc_data[5] = LPC_ADC->ADDR5;
    if (stat & 0x4040) _adc_data[6] = LPC_ADC->ADDR6;
    if (stat & 0x8080) _adc_data[7] = LPC_ADC->ADDR7;

    // Channel that triggered interrupt
    chan = (LPC_ADC->ADGDR >> 24) & 0x07;
    //User defined interrupt handlers
    if (_adc_isr[chan] != NULL)
        _adc_isr[chan](_adc_data[chan]);
    if (_adc_g_isr != NULL)
        _adc_g_isr(chan, _adc_data[chan]);
    return;
}

int ADC::_pin_to_channel(PinName pin) {
    int chan;
    switch (pin) {
        case p15://=p0.23 of LPC1768
        default:
            chan=0;
            break;
        case p16://=p0.24 of LPC1768
            chan=1;
            break;
        case p17://=p0.25 of LPC1768
            chan=2;
            break;
        case p18://=p0.26 of LPC1768
            chan=3;
            break;
        case p19://=p1.30 of LPC1768
            chan=4;
            break;
        case p20://=p1.31 of LPC1768
            chan=5;
            break;
    }
    return(chan);
}

PinName ADC::channel_to_pin(int chan) {
    const PinName pin[8]={p15, p16, p17, p18, p19, p20, p15, p15};

    if ((chan < 0) || (chan > 5))
        fprintf(stderr, "ADC channel %u is outside range available to MBED pins.\n", chan);
    return(pin[chan & 0x07]);
}


int ADC::channel_to_pin_number(int chan) {
    const int pin[8]={15, 16, 17, 18, 19, 20, 0, 0};

    if ((chan < 0) || (chan > 5))
        fprintf(stderr, "ADC channel %u is outside range available to MBED pins.\n", chan);
    return(pin[chan & 0x07]);
}


uint32_t ADC::_data_of_pin(PinName pin) {
    //If in burst mode and at least one interrupt enabled then
    //take all values from _adc_data
    if (burst() && (LPC_ADC->ADINTEN & 0x3F)) {
        return(_adc_data[_pin_to_channel(pin)]);
    } else {
        //Return current register value or last value from interrupt
        switch (pin) {
            case p15://=p0.23 of LPC1768
            default:
                return(LPC_ADC->ADINTEN & 0x01?_adc_data[0]:LPC_ADC->ADDR0);
            case p16://=p0.24 of LPC1768
                return(LPC_ADC->ADINTEN & 0x02?_adc_data[1]:LPC_ADC->ADDR1);
            case p17://=p0.25 of LPC1768
                return(LPC_ADC->ADINTEN & 0x04?_adc_data[2]:LPC_ADC->ADDR2);
            case p18://=p0.26 of LPC1768:
                return(LPC_ADC->ADINTEN & 0x08?_adc_data[3]:LPC_ADC->ADDR3);
            case p19://=p1.30 of LPC1768
                return(LPC_ADC->ADINTEN & 0x10?_adc_data[4]:LPC_ADC->ADDR4);
            case p20://=p1.31 of LPC1768
                return(LPC_ADC->ADINTEN & 0x20?_adc_data[5]:LPC_ADC->ADDR5);
        }
    }
}

//Enable or disable an ADC pin
void ADC::setup(PinName pin, int state) {
    int chan;
    chan=_pin_to_channel(pin);
    if ((state & 1) == 1) {
        switch(pin) {
            case p15://=p0.23 of LPC1768
            default:
                LPC_PINCON->PINSEL1 &= ~((unsigned int)0x3 << 14);
                LPC_PINCON->PINSEL1 |= (unsigned int)0x1 << 14;
                LPC_PINCON->PINMODE1 &= ~((unsigned int)0x3 << 14);
                LPC_PINCON->PINMODE1 |= (unsigned int)0x2 << 14;
                break;
            case p16://=p0.24 of LPC1768
                LPC_PINCON->PINSEL1 &= ~((unsigned int)0x3 << 16);
                LPC_PINCON->PINSEL1 |= (unsigned int)0x1 << 16;
                LPC_PINCON->PINMODE1 &= ~((unsigned int)0x3 << 16);
                LPC_PINCON->PINMODE1 |= (unsigned int)0x2 << 16;
                break;
            case p17://=p0.25 of LPC1768
                LPC_PINCON->PINSEL1 &= ~((unsigned int)0x3 << 18);
                LPC_PINCON->PINSEL1 |= (unsigned int)0x1 << 18;
                LPC_PINCON->PINMODE1 &= ~((unsigned int)0x3 << 18);
                LPC_PINCON->PINMODE1 |= (unsigned int)0x2 << 18;
                break;
            case p18://=p0.26 of LPC1768:
                LPC_PINCON->PINSEL1 &= ~((unsigned int)0x3 << 20);
                LPC_PINCON->PINSEL1 |= (unsigned int)0x1 << 20;
                LPC_PINCON->PINMODE1 &= ~((unsigned int)0x3 << 20);
                LPC_PINCON->PINMODE1 |= (unsigned int)0x2 << 20;
                break;
            case p19://=p1.30 of LPC1768
                LPC_PINCON->PINSEL3 &= ~((unsigned int)0x3 << 28);
                LPC_PINCON->PINSEL3 |= (unsigned int)0x3 << 28;
                LPC_PINCON->PINMODE3 &= ~((unsigned int)0x3 << 28);
                LPC_PINCON->PINMODE3 |= (unsigned int)0x2 << 28;
                break;
            case p20://=p1.31 of LPC1768
                LPC_PINCON->PINSEL3 &= ~((unsigned int)0x3 << 30);
                LPC_PINCON->PINSEL3 |= (unsigned int)0x3 << 30;
                LPC_PINCON->PINMODE3 &= ~((unsigned int)0x3 << 30);
                LPC_PINCON->PINMODE3 |= (unsigned int)0x2 << 30;
               break;
        }
        //Only one channel can be selected at a time if not in burst mode
        if (!burst()) LPC_ADC->ADCR &= ~0xFF;
        //Select channel
        LPC_ADC->ADCR |= (1 << chan);
    }
    else {
        switch(pin) {
            case p15://=p0.23 of LPC1768
            default:
                LPC_PINCON->PINSEL1 &= ~((unsigned int)0x3 << 14);
                LPC_PINCON->PINMODE1 &= ~((unsigned int)0x3 << 14);
                break;
            case p16://=p0.24 of LPC1768
                LPC_PINCON->PINSEL1 &= ~((unsigned int)0x3 << 16);
                LPC_PINCON->PINMODE1 &= ~((unsigned int)0x3 << 16);
                break;
            case p17://=p0.25 of LPC1768
                LPC_PINCON->PINSEL1 &= ~((unsigned int)0x3 << 18);
                LPC_PINCON->PINMODE1 &= ~((unsigned int)0x3 << 18);
                break;
            case p18://=p0.26 of LPC1768:
                LPC_PINCON->PINSEL1 &= ~((unsigned int)0x3 << 20);
                LPC_PINCON->PINMODE1 &= ~((unsigned int)0x3 << 20);
                break;
            case p19://=p1.30 of LPC1768
                LPC_PINCON->PINSEL3 &= ~((unsigned int)0x3 << 28);
                LPC_PINCON->PINMODE3 &= ~((unsigned int)0x3 << 28);
                break;
            case p20://=p1.31 of LPC1768
                LPC_PINCON->PINSEL3 &= ~((unsigned int)0x3 << 30);
                LPC_PINCON->PINMODE3 &= ~((unsigned int)0x3 << 30);
                break;
        }
        LPC_ADC->ADCR &= ~(1 << chan);
    }
}
//Return channel enabled/disabled state
int ADC::setup(PinName pin) {
    int chan;

    chan = _pin_to_channel(pin);
    return((LPC_ADC->ADCR & (1 << chan)) >> chan);
}

//Select channel already setup
void ADC::select(PinName pin) {
    int chan;

    //Only one channel can be selected at a time if not in burst mode
    if (!burst()) LPC_ADC->ADCR &= ~0xFF;
    //Select channel
    chan = _pin_to_channel(pin);
    LPC_ADC->ADCR |= (1 << chan);
}

//Enable or disable burst mode
void ADC::burst(int state) {
    if ((state & 1) == 1) {
        if (startmode(0) != 0)
            fprintf(stderr, "ADC Warning. startmode is %u. Must be 0 for burst mode.\n", startmode(0));
        LPC_ADC->ADCR |= (1 << 16);
    }
    else
        LPC_ADC->ADCR &= ~(1 << 16);
}
//Return burst mode state
int  ADC::burst(void) {
    return((LPC_ADC->ADCR & (1 << 16)) >> 16);
}

//Set startmode and edge
void ADC::startmode(int mode, int edge) {
    int lpc_adc_temp;

    //Reset start mode and edge bit,
    lpc_adc_temp = LPC_ADC->ADCR & ~(0x0F << 24);
    //Write with new values
    lpc_adc_temp |= ((mode & 7) << 24) | ((edge & 1) << 27);
    LPC_ADC->ADCR = lpc_adc_temp;
}

//Return startmode state according to mode_edge=0: mode and mode_edge=1: edge
int ADC::startmode(int mode_edge){
    switch (mode_edge) {
        case 0:
        default:
            return((LPC_ADC->ADCR >> 24) & 0x07);
        case 1:
            return((LPC_ADC->ADCR >> 27) & 0x01);
    }
}

//Start ADC conversion
void ADC::start(void) {
    startmode(1,0);
}


//Set interrupt enable/disable for pin to state
void ADC::interrupt_state(PinName pin, int state) {
    int chan;

    chan = _pin_to_channel(pin);
    if (state == 1) {
        LPC_ADC->ADINTEN &= ~0x100;
        LPC_ADC->ADINTEN |= 1 << chan;
        /* Enable the ADC Interrupt */
        NVIC_EnableIRQ(ADC_IRQn);
    } else {
        LPC_ADC->ADINTEN &= ~( 1 << chan );
        //Disable interrrupt if no active pins left
        if ((LPC_ADC->ADINTEN & 0xFF) == 0)
            NVIC_DisableIRQ(ADC_IRQn);
    }
}

//Return enable/disable state of interrupt for pin
int ADC::interrupt_state(PinName pin) {
    int chan;

    chan = _pin_to_channel(pin);
    return((LPC_ADC->ADINTEN >> chan) & 0x01);
}


//Attach custom interrupt handler replacing default
void ADC::attach(void(*fptr)(void)) {
    //* Attach IRQ
    NVIC_SetVector(ADC_IRQn, (uint32_t)fptr);
}

//Restore default interrupt handler
void ADC::detach(void) {
    //* Attach IRQ
    instance = this;
    NVIC_SetVector(ADC_IRQn, (uint32_t)&_adcisr);
}


//Append interrupt handler for pin to function isr
void ADC::append(PinName pin, void(*fptr)(uint32_t value)) {
    int chan;

    chan = _pin_to_channel(pin);
    _adc_isr[chan] = fptr;
}

//Append interrupt handler for pin to function isr
void ADC::unappend(PinName pin) {
    int chan;

    chan = _pin_to_channel(pin);
    _adc_isr[chan] = NULL;
}

//Unappend global interrupt handler to function isr
void ADC::append(void(*fptr)(int chan, uint32_t value)) {
    _adc_g_isr = fptr;
}

//Detach global interrupt handler to function isr
void ADC::unappend() {
    _adc_g_isr = NULL;
}

//Set ADC offset
void ADC::offset(int offset) {
    LPC_ADC->ADTRM &= ~(0x07 << 4);
    LPC_ADC->ADTRM |= (offset & 0x07) << 4;
}

//Return current ADC offset
int ADC::offset(void) {
    return((LPC_ADC->ADTRM >> 4) & 0x07);
}

//Return value of ADC on pin
int ADC::read(PinName pin) {
    //Reset DONE and OVERRUN flags of interrupt handled ADC data
    _adc_data[_pin_to_channel(pin)] &= ~(((uint32_t)0x01 << 31) | ((uint32_t)0x01 << 30));
    //Return value
    return((_data_of_pin(pin) >> 4) & 0xFFF);
}

//Return DONE flag of ADC on pin
int ADC::done(PinName pin) {
    return((_data_of_pin(pin) >> 31) & 0x01);
}

//Return OVERRUN flag of ADC on pin
int ADC::overrun(PinName pin) {
    return((_data_of_pin(pin) >> 30) & 0x01);
}

int ADC::actual_adc_clock(void) {
    return(_adc_clk_freq);
}

int ADC::actual_sample_rate(void) {
    return(_adc_clk_freq / CLKS_PER_SAMPLE);
}
