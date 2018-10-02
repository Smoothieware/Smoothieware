/* Copyright (c) 2010-2011 mbed.org, MIT License
*
* Permission is hereby granted, free of charge, to any person obtaining a copy of this software
* and associated documentation files (the "Software"), to deal in the Software without
* restriction, including without limitation the rights to use, copy, modify, merge, publish,
* distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all copies or
* substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
* BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
* NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
* DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <cstdint>
#include <cstdio>

#include "USBSerial.h"

#include "libs/Kernel.h"
#include "libs/SerialMessage.h"
#include "StreamOutputPool.h"

#include "mbed.h"

// extern void setled(int, bool);
#define setled(a, b) do {} while (0)

#define iprintf(...) do { } while (0)

USBSerial::USBSerial(USB *u): USBCDC(u), rxbuf(256 + 8), txbuf(128 + 8)
{
    usb = u;
    nl_in_rx = 0;
    attach = attached = false;
    flush_to_nl = false;
    halt_flag = false;
    query_flag = false;
    last_char_was_dollar = false;
}

bool USBSerial::ensure_tx_space(int space)
{
    // we need some kind of timeout here or it will hang if upstream stalls
    uint32_t start = us_ticker_read();
    while (txbuf.free() < space) {
        if((us_ticker_read() - start) > 1000000) {
            // 1 second timeout
            return false;
        }
        usb->endpointSetInterrupt(CDC_BulkIn.bEndpointAddress, true);
        usb->usbisr();
    }
    return true;
}

int USBSerial::_putc(int c)
{
    if (!attached)
        return 1;
    if(ensure_tx_space(1)) {
        txbuf.queue(c);
    }

    usb->endpointSetInterrupt(CDC_BulkIn.bEndpointAddress, true);
    return 1;
}

int USBSerial::_getc()
{
    if (!attached)
        return 0;
    uint8_t c = 0;
    setled(4, 1); while (rxbuf.isEmpty()); setled(4, 0);
    rxbuf.dequeue(&c);
    if (rxbuf.free() == MAX_PACKET_SIZE_EPBULK) {
        usb->endpointSetInterrupt(CDC_BulkOut.bEndpointAddress, true);
        iprintf("rxbuf has room for another packet, interrupt enabled\n");
    } else if ((rxbuf.free() < MAX_PACKET_SIZE_EPBULK) && (nl_in_rx == 0)) {
        // handle potential deadlock where a short line, and the beginning of a very long line are bundled in one usb packet
        rxbuf.flush();
        flush_to_nl = true;

        usb->endpointSetInterrupt(CDC_BulkOut.bEndpointAddress, true);
        iprintf("rxbuf has room for another packet, interrupt enabled\n");
    }
    if (nl_in_rx > 0)
        if (c == '\n' || c == '\r')
            nl_in_rx--;

    return c;
}

int USBSerial::puts(const char *str)
{
    if (!attached)
        return strlen(str);
    int i = 0;
    while (*str) {
        if(!ensure_tx_space(1)) break;
        txbuf.queue(*str);
        if ((txbuf.available() % 64) == 0)
            usb->endpointSetInterrupt(CDC_BulkIn.bEndpointAddress, true);
        i++;
        str++;
    }
    usb->endpointSetInterrupt(CDC_BulkIn.bEndpointAddress, true);
    return i;
}

uint16_t USBSerial::writeBlock(const uint8_t * buf, uint16_t size)
{
    if (!attached)
        return size;
    if (size > txbuf.free()) {
        size = txbuf.free();
    }
    if (size > 0) {
        for (uint8_t i = 0; i < size; i++) {
            txbuf.queue(buf[i]);
        }
        usb->endpointSetInterrupt(CDC_BulkIn.bEndpointAddress, true);
    }
    return size;
}

bool USBSerial::USBEvent_EPIn(uint8_t bEP, uint8_t bEPStatus)
{
    /*
     * Called in ISR context
     */

//     static bool needToSendNull = false;

    bool r = true;

    if (bEP != CDC_BulkIn.bEndpointAddress)
        return false;

    iprintf("USBSerial:EpIn: 0x%02X\n", bEPStatus);

    uint8_t b[MAX_PACKET_SIZE_EPBULK];

    int l = txbuf.available();
    if (l > 0) {
        // Use MAX_PACKET_SIZE_EPBULK-1 below instead of MAX_PACKET_SIZE_EPBULK
        // to work around a problem sending packets that are exactly MAX_PACKET_SIZE_EPBULK
        // bytes in length. The problem is that these packets don't flush properly.
        if (l > MAX_PACKET_SIZE_EPBULK-1)
            l = MAX_PACKET_SIZE_EPBULK-1;
        int i;
        for (i = 0; i < l; i++) {
            txbuf.dequeue(&b[i]);
        }
        send(b, l);
        if (txbuf.available() == 0)
            r = false;
    } else {
        r = false;
    }
    iprintf("USBSerial:EpIn Complete\n");
    return r;
}

bool USBSerial::USBEvent_EPOut(uint8_t bEP, uint8_t bEPStatus)
{
    /*
     * Called in ISR context
     */

    bool r = true;

    iprintf("USBSerial:EpOut\n");
    if (bEP != CDC_BulkOut.bEndpointAddress)
        return false;

    if (rxbuf.free() < MAX_PACKET_SIZE_EPBULK) {
//         usb->endpointSetInterrupt(bEP, false);
        return false;
    }

    uint8_t c[MAX_PACKET_SIZE_EPBULK];
    uint32_t size = 64;

    //we read the packet received and put it on the circular buffer
    readEP(c, &size);
    iprintf("Read %ld bytes:\n\t", size);
    for (uint8_t i = 0; i < size; i++) {

        // handle backspace and delete by deleting the last character in the buffer if there is one
        if(c[i] == 0x08 || c[i] == 0x7F) {
            if(!rxbuf.isEmpty()) rxbuf.pop();
            continue;
        }

        if(c[i] == 'X' - 'A' + 1) { // ^X
            //THEKERNEL->set_feed_hold(false); // required to free stuff up
            halt_flag = true;
            continue;
        }

        if(c[i] == '?') { // ?
            query_flag = true;
            continue;
        }

        if(THEKERNEL->is_grbl_mode() || THEKERNEL->is_feed_hold_enabled()) {
            if(c[i] == '!') { // safe pause
                THEKERNEL->set_feed_hold(true);
                continue;
            }

            if(c[i] == '~') { // safe resume
                THEKERNEL->set_feed_hold(false);
                continue;
            }
        }

        last_char_was_dollar = (c[i] == '$');

        if (flush_to_nl == false)
            rxbuf.queue(c[i]);

        // if (c[i] >= 32 && c[i] < 128)
        // {
        //     iprintf("%c", c[i]);
        // }
        // else
        // {
        //     iprintf("\\x%02X", c[i]);
        // }

        if (c[i] == '\n' || c[i] == '\r') {
            if (flush_to_nl)
                flush_to_nl = false;
            else
                nl_in_rx++;
        } else if (rxbuf.isFull() && (nl_in_rx == 0)) {
            // to avoid a deadlock with very long lines, we must dump the buffer
            // and continue flushing to the next newline
            rxbuf.flush();
            flush_to_nl = true;
        }
    }
    iprintf("\nQueued, %d empty\n", rxbuf.free());

    if (rxbuf.free() < MAX_PACKET_SIZE_EPBULK) {
        // if buffer is full, stall endpoint, do not accept more data
        r = false;

        if (nl_in_rx == 0) {
            // we have to check for long line deadlock here too
            flush_to_nl = true;
            rxbuf.flush();

            // and since our buffer is empty, we can accept more data
            r = true;
        }
    }

    usb->readStart(CDC_BulkOut.bEndpointAddress, MAX_PACKET_SIZE_EPBULK);
    iprintf("USBSerial:EpOut Complete\n");
    return r;
}

uint8_t USBSerial::available()
{
    return rxbuf.available();
}

bool USBSerial::ready()
{
    return rxbuf.available();
}

void USBSerial::on_module_loaded()
{
    this->register_for_event(ON_MAIN_LOOP);
    this->register_for_event(ON_IDLE);
}

void USBSerial::on_idle(void *argument)
{
    if(halt_flag) {
        halt_flag = false;
        THEKERNEL->call_event(ON_HALT, nullptr);
        if(THEKERNEL->is_grbl_mode()) {
            puts("ALARM: Abort during cycle\r\n");
        } else {
            puts("HALTED, M999 or $X to exit HALT state\r\n");
        }
        rxbuf.flush(); // flush the recieve buffer, hopefully upstream has stopped sending
        nl_in_rx = 0;
    }

    if(query_flag) {
        query_flag = false;
        puts(THEKERNEL->get_query_string().c_str());
    }

}

void USBSerial::on_main_loop(void *argument)
{
    // apparently some OSes don't assert DTR when a program opens the port
    if (available() && !attach)
        attach = true;

    if (attach != attached) {
        if (attach) {
            attached = true;
            THEKERNEL->streams->append_stream(this);
            puts("Smoothie\r\nok\r\n");
        } else {
            attached = false;
            THEKERNEL->streams->remove_stream(this);
            txbuf.flush();
            rxbuf.flush();
            nl_in_rx = 0;
        }
    }

    // if we are in feed hold we do not process anything
    //if(THEKERNEL->get_feed_hold()) return;

    if (nl_in_rx) {
        string received;
        while (available()) {
            char c = _getc();
            if( c == '\n' || c == '\r') {
                struct SerialMessage message;
                message.message = received;
                message.stream = this;
                iprintf("USBSerial Received: %s\n", message.message.c_str());
                THEKERNEL->call_event(ON_CONSOLE_LINE_RECEIVED, &message );
                return;
            } else {
                received += c;
            }
        }
    }
}

void USBSerial::on_attach()
{
    attach = true;
}

void USBSerial::on_detach()
{
    attach = false;
}
