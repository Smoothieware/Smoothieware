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

    // We only call the command dispatcher in the main loop, nowhere else
    this->register_for_event(ON_MAIN_LOOP);
}
        
// Called on Serial::RxIrq interrupt, meaning we have received a char
void SerialConsole::on_serial_char_received(){
    if(this->serial->readable()){
        char received = this->serial->getc();
        //On newline, we have received a line, else concatenate in buffer
        if( received == '\r' ){ return; }
        this->buffer.push_back(received); 
    }
}
        
// Actual event calling must happen in the main loop because if it happens in the interrupt we will loose data
void SerialConsole::on_main_loop(void * argument){
    if( this->has_char('\n') ){
        int index = 0;
        string received;
        while(1){
           char c;
           this->buffer.pop_front(c);
           if( c == '\n' ){
                struct SerialMessage message; 
                message.message = received;
                message.stream = this;
                this->kernel->call_event(ON_CONSOLE_LINE_RECEIVED, &message ); 
                return;
            }else{
                received += c;
            }
        }
    }
}


int SerialConsole::printf(const char* format, ...){
    va_list args;
    int result; 
    va_start (args, format);
    result = vfprintf( this->serial->_file, format, args);
    va_end (args);
    return result;
}


bool SerialConsole::has_char(char letter){
    int index = this->buffer.head;
    while( index != this->buffer.tail ){
        if( this->buffer.buffer[index] == letter ){
            return true;
        }
        index = this->buffer.next_block_index(index);
    }
    return false;
}
