/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef SERIALCONSOLE_H
#define SERIALCONSOLE_H

#include "libs/Module.h"
#include "libs/Kernel.h"
#include <vector>
#include <string>

#include "libs/TSRingBuffer.h"
#include "libs/StreamOutput.h"

class SerialConsole : public Module, public StreamOutput {
    public:
        SerialConsole(int ch);
        virtual ~SerialConsole();

        void on_module_loaded();
        void on_serial_char_received(char c);
        void on_main_loop(void * argument);
        void on_idle(void * argument);
        void init_uart(int baud_rate);
        int _putc(int c);
        int _getc(void);
        bool ready();
        int puts(const char*);

        TSRingBuffer<char, 256> buffer;   // Receive buffer

        struct {
          bool query_flag:1;
          bool halt_flag:1;
          bool last_char_was_cr:1;
          uint8_t uartn:2;
          uint8_t lf_count:8;
        };
};

#endif
