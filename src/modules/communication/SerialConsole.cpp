/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include <string>
#include <stdarg.h>
using std::string;
#include "libs/Module.h"
#include "libs/Kernel.h"
#include "libs/nuts_bolts.h"
#include "SerialConsole.h"
#include "libs/RingBuffer.h"
#include "libs/SerialMessage.h"
#include "libs/StreamOutput.h"

// Serial reading module
// Treats every received line as a command and passes it ( via event call ) to the command dispatcher.
// The command dispatcher will then ask other modules if they can do something with it
SerialConsole::SerialConsole( PinName rx_pin, PinName tx_pin, int baud_rate ){
    this->serial = new mbed::Serial( rx_pin, tx_pin );
    this->serial->baud(baud_rate);
}

// Called when the module has just been loaded
void SerialConsole::on_module_loaded() {
    // We want to be called every time a new char is received
    this->serial->attach(this, &SerialConsole::on_serial_char_received, mbed::Serial::RxIrq);

    // Add to the pack of streams kernel can call to, for example for broadcasting
    this->kernel->streams->append_stream(this);
}

// Called on Serial::RxIrq interrupt, meaning we have received a char
void SerialConsole::on_serial_char_received(){
    while(this->serial->readable()){
        this->serial->getc();
    }
}

int SerialConsole::puts(const char* s)
{
    return fwrite(s, strlen(s), 1, this->serial->_file);
}

int SerialConsole::_putc(int c)
{
    return this->serial->putc(c);
}

int SerialConsole::_getc()
{
    return this->serial->getc();
}
