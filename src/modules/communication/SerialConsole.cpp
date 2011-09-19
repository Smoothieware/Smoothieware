/*  
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>. 
*/

#include <string>
using std::string;
#include "mbed.h"
#include "libs/Module.h"
#include "libs/Kernel.h"
#include "libs/nuts_bolts.h"
#include "SerialConsole.h"

// Serial reading module
// Treats every received line as a command and passes it ( via event call ) to the command dispatcher. 
// The command dispatcher will then ask other modules if they can do something with it
SerialConsole::SerialConsole( PinName rx_pin, PinName tx_pin, int baud_rate )  : Serial( rx_pin, tx_pin ){ //We extend Serial, it is convenient
    this->baud(baud_rate);
}  


// Called when the module has just been loaded
void SerialConsole::on_module_loaded() {
    // We want to be called every time a new char is received
    this->attach(this, &SerialConsole::on_serial_char_received, Serial::RxIrq);

    // We only call the command dispatcher in the main loop, nowhere else
    this->register_for_event(ON_MAIN_LOOP);
}
        
// Called on Serial::RxIrq interrupt, meaning we have received a char
void SerialConsole::on_serial_char_received(){
    if(this->readable()){
        char received = this->getc();
        //On newline, we have received a line, else concatenate in buffer
        if( received == '\r' ){ return; }
        if( received == '\n' ){
            this->line_received();
            receive_buffer = "";
        }else{
            this->receive_buffer += received;                
        }
   }
}
        
// Call event when newline received, for other modules to read the line
inline void SerialConsole::line_received(){
  string new_string = this->receive_buffer;
  this->received_lines.insert(this->received_lines.begin(),new_string);
}

// Actual event calling must happen in the main loop because if it happens in the interrupt we will loose data
void SerialConsole::on_main_loop(void * argument){
  if( this->received_lines.size() < 1 ){ return; }
  this->kernel->call_event(ON_CONSOLE_LINE_RECEIVED, &this->received_lines.back() ); 
  this->received_lines.pop_back();
}


