/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef SERIALCONSOLE_H
#define SERIALCONSOLE_H

#include "libs/Module.h"
#include "Serial.h" // mbed.h lib
#include "libs/Kernel.h"
#include <vector>
#include <string>
using std::string;
#include "libs/RingBuffer.h"
#include "libs/StreamOutput.h"


#define baud_rate_setting_checksum CHECKSUM("baud_rate")

class SerialConsole : public Module, public StreamOutput {
    public:
        SerialConsole( PinName rx_pin, PinName tx_pin, int baud_rate );

        void on_module_loaded();
        void on_serial_char_received();
        void on_main_loop(void * argument);
        bool has_char(char letter);

        int _putc(int c);
        int _getc(void);
        int puts(const char*);

        //string receive_buffer;                 // Received chars are stored here until a newline character is received
        //vector<std::string> received_lines;    // Received lines are stored here until they are requested
        RingBuffer<char,256> buffer;             // Receive buffer
        mbed::Serial* serial;
};

#endif
