/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "SerialConsole.h"

#include "libs/Module.h"
#include "libs/Kernel.h"
#include "libs/nuts_bolts.h"
#include "libs/SerialMessage.h"
#include "libs/StreamOutput.h"
#include "libs/StreamOutputPool.h"
#include "libs/Config.h"
#include "checksumm.h"
#include "ConfigValue.h"
#include "utils.h"

#include <string>
#include <stdarg.h>

#include "lpc17xx_uart.h"
#include "lpc17xx_libcfg_default.h"
#include "lpc17xx_pinsel.h"
#include "cmsis_nvic.h"

#define baud_rate_setting_checksum CHECKSUM("baud_rate")
#define uart0_checksum             CHECKSUM("uart0")

static SerialConsole* instance = 0;
static void *LPC_UART;

// Serial reading module
// Treats every received line as a command and passes it ( via event call ) to the command dispatcher.
// The command dispatcher will then ask other modules if they can do something with it
SerialConsole::SerialConsole(int uart_ch)
{
    switch(uart_ch) {
        case 0: LPC_UART = LPC_UART0; break;
        case 1: LPC_UART = LPC_UART1; break;
        case 2: LPC_UART = LPC_UART2; break;
        case 3: LPC_UART = LPC_UART3; break;
    }
    uartn = uart_ch;
    instance = this;
}

SerialConsole::~SerialConsole()
{
    switch(uartn) {
        case 0: NVIC_DisableIRQ(UART0_IRQn); break;
        case 1: NVIC_DisableIRQ(UART1_IRQn); break;
        case 2: NVIC_DisableIRQ(UART2_IRQn); break;
        case 3: NVIC_DisableIRQ(UART3_IRQn); break;
    }
    // DeInitialize UART0 peripheral
    UART_DeInit((LPC_UART_TypeDef *)LPC_UART);
    instance = nullptr;
}

/*----------------- INTERRUPT SERVICE ROUTINES --------------------------*/
/*********************************************************************/
static void UART_IRQHandler(void)
{
    uint32_t intsrc, tmp;

    /* Determine the interrupt source */
    intsrc = UART_GetIntId((LPC_UART_TypeDef *)LPC_UART);
    tmp = intsrc & UART_IIR_INTID_MASK;

    // Receive Line Status
    if (tmp == UART_IIR_INTID_RLS) {
        // Check line status
        uint32_t tmp1 = UART_GetLineStatus((LPC_UART_TypeDef *)LPC_UART);
        // Mask out the Receive Ready and Transmit Holding empty status
        tmp1 &= (UART_LSR_OE | UART_LSR_PE | UART_LSR_FE | UART_LSR_BI | UART_LSR_RXFE);
        // If any error exist
        if (tmp1) {
            // UART_IntErr(&uart0, tmp1);
        }
    }

    // Receive Data Available or Character time-out
    if ((tmp == UART_IIR_INTID_RDA) || (tmp == UART_IIR_INTID_CTI)) {
        uint8_t tmpc;
        uint32_t rLen;

        while(1) {
            // Call UART read function in UART driver
            rLen = UART_Receive((LPC_UART_TypeDef *)LPC_UART, &tmpc, 1, NONE_BLOCKING);
            // If data received
            if (rLen) {
                if(instance != nullptr) {
                    instance->on_serial_char_received(tmpc);
                }
            } else {
                // no more data
                break;
            }
        }
    }
}

void SerialConsole::init_uart(int baud_rate)
{
    // UART Configuration structure variable
    UART_CFG_Type UARTConfigStruct;
    // UART FIFO configuration Struct variable
    UART_FIFO_CFG_Type UARTFIFOConfigStruct;
    // Pin configuration for UART0
    PINSEL_CFG_Type PinCfg;

    /*
     * Initialize UARTn pin connect
     */
    int fnc, pinnum, portnum;
    switch(uartn) {
        case 0: fnc= 1; pinnum= 2; portnum= 0; break;
        case 1: fnc= 2; pinnum= 0; portnum= 2; break;
        case 2: fnc= 1; pinnum= 10; portnum= 0; break;
        case 3: fnc= 2; pinnum= 0; portnum= 0; break;
    }
    PinCfg.Funcnum = fnc;
    PinCfg.OpenDrain = 0;
    PinCfg.Pinmode = 0;
    PinCfg.Pinnum = pinnum;
    PinCfg.Portnum = portnum;
    PINSEL_ConfigPin(&PinCfg);
    PinCfg.Pinnum = pinnum+1;
    PINSEL_ConfigPin(&PinCfg);

    /* Initialize UART Configuration parameter structure to default state:
     * Baudrate = 9600bps
     * 8 data bit
     * 1 Stop bit
     * None parity
     */
    UART_ConfigStructInit(&UARTConfigStruct);

    UARTConfigStruct.Baud_rate = baud_rate;

    // Initialize UART0 peripheral with given to corresponding parameter
    UART_Init((LPC_UART_TypeDef *)LPC_UART, &UARTConfigStruct);


    /* Initialize FIFOConfigStruct to default state:
     *                              - FIFO_DMAMode = DISABLE
     *                              - FIFO_Level = UART_FIFO_TRGLEV0
     *                              - FIFO_ResetRxBuf = ENABLE
     *                              - FIFO_ResetTxBuf = ENABLE
     *                              - FIFO_State = ENABLE
     */
    UART_FIFOConfigStructInit(&UARTFIFOConfigStruct);

    // Initialize FIFO for UART0 peripheral
    UART_FIFOConfig((LPC_UART_TypeDef *)LPC_UART, &UARTFIFOConfigStruct);

    // Enable UART Transmit
    UART_TxCmd((LPC_UART_TypeDef *)LPC_UART, ENABLE);

    /* Enable UART Rx interrupt */
    UART_IntConfig((LPC_UART_TypeDef *)LPC_UART, UART_INTCFG_RBR, ENABLE);
    /* Disablee UART line status interrupt */
    UART_IntConfig((LPC_UART_TypeDef *)LPC_UART, UART_INTCFG_RLS, DISABLE);
    UART_IntConfig((LPC_UART_TypeDef *)LPC_UART, UART_INTCFG_THRE, DISABLE);


    /* Enable Interrupt for UART channel */
    switch(uartn) {
        case 0: NVIC_SetVector(UART0_IRQn, (uint32_t)&UART_IRQHandler); NVIC_EnableIRQ(UART0_IRQn); break;
        case 1: NVIC_SetVector(UART1_IRQn, (uint32_t)&UART_IRQHandler); NVIC_EnableIRQ(UART1_IRQn); break;
        case 2: NVIC_SetVector(UART2_IRQn, (uint32_t)&UART_IRQHandler); NVIC_EnableIRQ(UART2_IRQn); break;
        case 3: NVIC_SetVector(UART3_IRQn, (uint32_t)&UART_IRQHandler); NVIC_EnableIRQ(UART3_IRQn); break;
    }
}

// Called when the module has just been loaded
void SerialConsole::on_module_loaded()
{
    int baud = THEKERNEL->config->value(uart0_checksum, baud_rate_setting_checksum)->by_default(DEFAULT_SERIAL_BAUD_RATE)->as_number();
    init_uart(baud);

    query_flag = false;
    halt_flag = false;
    lf_count = 0;
    last_char_was_cr = false;

    // We only call the command dispatcher in the main loop, nowhere else
    this->register_for_event(ON_MAIN_LOOP);
    this->register_for_event(ON_IDLE);

    // Add to the pack of streams kernel can call to, for example for broadcasting
    THEKERNEL->streams->append_stream(this);
}

// Called on interrupt, meaning we have received a char
void SerialConsole::on_serial_char_received(char received)
{
    if(received == '?') {
        query_flag = true;
        return;
    }
    if(received == 'X' - 'A' + 1) { // ^X
        halt_flag = true;
        return;
    }
    if(received == 'Y' - 'A' + 1) { // ^Y
        THEKERNEL->set_stop_request(true); // generic stop what you are doing request
        return;
    }
    if(received == '\n' && last_char_was_cr) {
        // ignore the \n of a \r\n pair
        last_char_was_cr = false;
        return;
    }
    last_char_was_cr = (received == '\r');

    // convert CR to NL (for host OSs that don't send NL)
    if( received == '\r' ) { received = '\n'; }
    if(this->buffer.put(received)) {
        if(received == '\n') ++lf_count;
    }
}

void SerialConsole::on_idle(void * argument)
{
    if(query_flag) {
        query_flag = false;
        puts(THEKERNEL->get_query_string().c_str());
    }
    if(halt_flag) {
        halt_flag = false;
        THEKERNEL->call_event(ON_HALT, nullptr);
        char c;
        while(this->buffer.get(c)) ; // flush the recieve buffer, hopefully upstream has stopped sending
        lf_count= 0;

        if(THEKERNEL->is_grbl_mode()) {
            puts("ALARM: Abort during cycle\r\n");
        } else {
            puts("HALTED, M999 or $X to exit HALT state\r\n");
        }
    }
}

// Actual event calling must happen in the main loop because if it happens in the interrupt we will loose data
void SerialConsole::on_main_loop(void * argument)
{
    if(lf_count > 0) {
        string received;
        received.reserve(20);
        while(1) {
            char c;
            if(!this->buffer.get(c)) {
                lf_count= 0; // be safe as this should not happen anyway
                return;
            }
            if(c == '\n') {
                --lf_count;
                struct SerialMessage message;
                message.message = received;
                message.stream = this;
                THEKERNEL->call_event(ON_CONSOLE_LINE_RECEIVED, &message );
                return;
            } else {
                received += c;
            }
        }
    }
}

#pragma GCC diagnostic ignored "-Wcast-qual"
int SerialConsole::puts(const char* s)
{
    const uint8_t *p = (const uint8_t*)s;
    return UART_Send((LPC_UART_TypeDef *)LPC_UART, (uint8_t*)p,  strlen(s), BLOCKING);
}

int SerialConsole::_putc(int c)
{
    UART_SendByte((LPC_UART_TypeDef *)LPC_UART, c);
    return 1;
}

bool SerialConsole::ready()
{
    return !this->buffer.empty();
}

int SerialConsole::_getc()
{
    while(1) {
        char c;
        if(this->buffer.get(c)) {
            if(c == '\n') --lf_count;
            return c;
        }
        safe_delay_ms(1);
    }
}

extern "C" int UART_Write(const unsigned char*, unsigned int);

int UART_Write(const unsigned char *buf, unsigned int len)
{
    return UART_Send((LPC_UART_TypeDef *)LPC_UART, (uint8_t*)buf, len, BLOCKING);
}

extern "C" int UART_ReadChar();

int UART_ReadChar()
{
    if(instance == nullptr) return -1;
    return instance->_getc();
}
