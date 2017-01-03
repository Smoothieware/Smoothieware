/*
    This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
    Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
    Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
    You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef MODBUS_H
#define MODBUS_H

#include "libs/Module.h"
#include <vector>

class BufferedSoftSerial;
class GPIO;

class Modbus : public Module {
    public:
        Modbus( PinName rx_pin, PinName tx_pin, PinName dir_pin);
        Modbus( PinName rx_pin, PinName tx_pin, PinName dir_pin, int baud_rate);
        Modbus( PinName rx_pin, PinName tx_pin, PinName dir_pin, int baud_rate, const char *format);

        void on_module_loaded();
        void on_serial_char_received();

        void read_coil(int slave_addr, int coil_addr, int n_coils);
        void read_holding_register(int slave_addr, int reg_addr, int n_regs);
        void write_coil(int slave_addr, int coil_addr, bool data);
        void write_holding_register(int slave_addr, int reg_addr, int data);
        void diagnostic(int slave_addr, int test_sub_code, int data);
        void write_multiple_coils(int slave_addr, int coil_addr, int n_coils, int data);
        void write_multiple_registers(int slave_addr, int start_addr, int data);
        void read_write_multiple_holding_registers(int slave_addr, int read_addr, int n_read, int write_addr, int data);
        void calculate_delay(int baudrate, int bits, int parity, int stop);
        void delay(unsigned int);
        unsigned int crc16(char *data, unsigned int len); 

        GPIO *dir_output;

        BufferedSoftSerial* serial;
        std::vector<int> buffer;

        float delay_time;        
};

#endif
