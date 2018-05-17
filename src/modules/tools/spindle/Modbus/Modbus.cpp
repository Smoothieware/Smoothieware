/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "libs/Module.h"
#include "libs/Kernel.h"
#include "libs/nuts_bolts.h"
#include "libs/gpio.h"
#include "BufferedSoftSerial.h"
#include "Modbus.h"

Modbus::Modbus( PinName tx_pin, PinName rx_pin, PinName dir_pin){
    serial = new BufferedSoftSerial( tx_pin, rx_pin );
    serial->baud(9600);
    serial->format(8,serial->Parity::None,1);
    calculate_delay(9600, 8, 0, 1);
    dir_output = new GPIO(dir_pin);
    dir_output->output();
    dir_output->clear();
}

Modbus::Modbus( PinName tx_pin, PinName rx_pin, PinName dir_pin, int baud_rate){
    serial = new BufferedSoftSerial( tx_pin, rx_pin );
    serial->baud(baud_rate);
    serial->format(8,serial->Parity::None,1);
    calculate_delay(baud_rate, 8, 0, 1);
    dir_output = new GPIO(dir_pin);
    dir_output->output();
    dir_output->clear();
}

Modbus::Modbus( PinName tx_pin, PinName rx_pin, PinName dir_pin, int baud_rate, const char *format){
    serial = new BufferedSoftSerial( tx_pin, rx_pin );
    serial->baud(baud_rate);
    
    if(strncmp(format, "8N1", 3) == 0){
        serial->format(8,serial->Parity::None,1);
        calculate_delay(baud_rate, 8, 0, 1);
    } else if(strncmp(format, "8O1", 3) == 0){
        serial->format(8,serial->Parity::Odd,1);
        calculate_delay(baud_rate, 8, 1, 1);
    } else if(strncmp(format, "8E1", 3) == 0){
        serial->format(8,serial->Parity::Even,1);
        calculate_delay(baud_rate, 8, 1, 1);
    } else if(strncmp(format, "8N2", 3) == 0){
        serial->format(8,serial->Parity::None,2);
        calculate_delay(baud_rate, 8, 0, 2);
    } else {
        serial->format(8,serial->Parity::None,1);
        calculate_delay(baud_rate, 8, 0, 1);
    }
    dir_output = new GPIO(dir_pin);
    dir_output->output();
    dir_output->clear();
}

// Called when the module has just been loaded
void Modbus::on_module_loaded() {
    // We want to be called every time a new char is received
    serial->attach(this, &Modbus::on_serial_char_received, BufferedSoftSerial::RxIrq);
}

// Called on Serial::RxIrq interrupt, meaning we have received a char
void Modbus::on_serial_char_received(){
    buffer.push_back(serial->getc());
}

void Modbus::read_coil(int slave_addr, int coil_addr, int n_coils){
    char telegram[8];
    unsigned int crc;
    telegram[0] = slave_addr;       // Slave address
    telegram[1] = 0x01;             // Function code
    telegram[2] = (coil_addr >> 8); // Coil address MSB
    telegram[3] = coil_addr & 0xFF; // Coil address LSB
    telegram[4] = (n_coils >> 8);   // number of coils to read MSB
    telegram[5] = n_coils & 0xFF;   // number of coils to read LSB
    crc = crc16(telegram, 6);       
    telegram[6] = crc;              // CRC LSB
    telegram[7] = (crc >> 8);       // CRC MSB
    dir_output->set();
    serial->write(telegram,8);
    delay((int) ceil(50 + 8 * delay_time));
    dir_output->clear();
    // TODO: read reply from buffer and return it
}

void Modbus::read_holding_register(int slave_addr, int reg_addr, int n_regs){

    // TODO: implement this
}

void Modbus::write_coil(int slave_addr, int coil_addr, bool data){
    char telegram[8];
    unsigned int crc;
    telegram[0] = slave_addr;       // Slave address
    telegram[1] = 0x05;             // Function code
    telegram[2] = (coil_addr >> 8); // Coil address MSB
    telegram[3] = coil_addr & 0xFF; // Coil address LSB
    telegram[4] = 0x00;             // Data MSB
    telegram[5] = (data == true) ? 0xFF : 0x00; // Data LSB
    crc = crc16(telegram, 6);       
    telegram[6] = crc;              // CRC LSB
    telegram[7] = (crc >> 8);       // CRC MSB
    dir_output->set();
    serial->write(telegram,8);
    delay((int) ceil(50 + 8 * delay_time));
    dir_output->clear();
}


void Modbus::write_holding_register(int slave_addr, int reg_addr, int data){
    char telegram[8];
    unsigned int crc;
    telegram[0] = slave_addr;       // Slave address
    telegram[1] = 0x06;             // Function code
    telegram[2] = (reg_addr >> 8);  // Register address MSB
    telegram[3] = reg_addr;         // Register address LSB
    telegram[4] = (data >> 8);      // Data MSB
    telegram[5] = data;             // Data LSB
    crc = crc16(telegram, 6);       
    telegram[6] = crc;              // CRC LSB
    telegram[7] = (crc >> 8);       // CRC MSB
    dir_output->set();
    serial->write(telegram,8);
    delay((int) ceil(50 + 8 * delay_time));
    dir_output->clear();
}

void Modbus::diagnostic(int slave_addr, int test_sub_code, int data){
    // TODO: implement this
}

void Modbus::write_multiple_coils(int slave_addr, int coil_addr, int n_coils, int data){
    // TODO: implement this
}

void Modbus::write_multiple_registers(int slave_addr, int start_addr, int data){
    // TODO: implement this
}

void Modbus::read_write_multiple_holding_registers(int slave_addr, int read_addr, int n_read, int write_addr, int data){
    // TODO: implement this
}

void Modbus::calculate_delay(int baudrate, int bits, int parity, int stop) {

    float bittime = 1000.0 / baudrate;
    // here we calculate how long a byte with all surrounding bits take
    // startbit + number of bits + parity bit + stop bit
    delay_time = bittime * (1 + bits + parity + 1);
}

void Modbus::delay(unsigned int value) {
    
    uint32_t start = us_ticker_read(); // mbed call
    while ((us_ticker_read() - start) < value*1000) {
        THEKERNEL->call_event(ON_IDLE, this);
    }

}

unsigned int Modbus::crc16(char *data, unsigned int len) {
    
    static const unsigned short crc_table[] = {
    0X0000, 0XC0C1, 0XC181, 0X0140, 0XC301, 0X03C0, 0X0280, 0XC241,
    0XC601, 0X06C0, 0X0780, 0XC741, 0X0500, 0XC5C1, 0XC481, 0X0440,
    0XCC01, 0X0CC0, 0X0D80, 0XCD41, 0X0F00, 0XCFC1, 0XCE81, 0X0E40,
    0X0A00, 0XCAC1, 0XCB81, 0X0B40, 0XC901, 0X09C0, 0X0880, 0XC841,
    0XD801, 0X18C0, 0X1980, 0XD941, 0X1B00, 0XDBC1, 0XDA81, 0X1A40,
    0X1E00, 0XDEC1, 0XDF81, 0X1F40, 0XDD01, 0X1DC0, 0X1C80, 0XDC41,
    0X1400, 0XD4C1, 0XD581, 0X1540, 0XD701, 0X17C0, 0X1680, 0XD641,
    0XD201, 0X12C0, 0X1380, 0XD341, 0X1100, 0XD1C1, 0XD081, 0X1040,
    0XF001, 0X30C0, 0X3180, 0XF141, 0X3300, 0XF3C1, 0XF281, 0X3240,
    0X3600, 0XF6C1, 0XF781, 0X3740, 0XF501, 0X35C0, 0X3480, 0XF441,
    0X3C00, 0XFCC1, 0XFD81, 0X3D40, 0XFF01, 0X3FC0, 0X3E80, 0XFE41,
    0XFA01, 0X3AC0, 0X3B80, 0XFB41, 0X3900, 0XF9C1, 0XF881, 0X3840,
    0X2800, 0XE8C1, 0XE981, 0X2940, 0XEB01, 0X2BC0, 0X2A80, 0XEA41,
    0XEE01, 0X2EC0, 0X2F80, 0XEF41, 0X2D00, 0XEDC1, 0XEC81, 0X2C40,
    0XE401, 0X24C0, 0X2580, 0XE541, 0X2700, 0XE7C1, 0XE681, 0X2640,
    0X2200, 0XE2C1, 0XE381, 0X2340, 0XE101, 0X21C0, 0X2080, 0XE041,
    0XA001, 0X60C0, 0X6180, 0XA141, 0X6300, 0XA3C1, 0XA281, 0X6240,
    0X6600, 0XA6C1, 0XA781, 0X6740, 0XA501, 0X65C0, 0X6480, 0XA441,
    0X6C00, 0XACC1, 0XAD81, 0X6D40, 0XAF01, 0X6FC0, 0X6E80, 0XAE41,
    0XAA01, 0X6AC0, 0X6B80, 0XAB41, 0X6900, 0XA9C1, 0XA881, 0X6840,
    0X7800, 0XB8C1, 0XB981, 0X7940, 0XBB01, 0X7BC0, 0X7A80, 0XBA41,
    0XBE01, 0X7EC0, 0X7F80, 0XBF41, 0X7D00, 0XBDC1, 0XBC81, 0X7C40,
    0XB401, 0X74C0, 0X7580, 0XB541, 0X7700, 0XB7C1, 0XB681, 0X7640,
    0X7200, 0XB2C1, 0XB381, 0X7340, 0XB101, 0X71C0, 0X7080, 0XB041,
    0X5000, 0X90C1, 0X9181, 0X5140, 0X9301, 0X53C0, 0X5280, 0X9241,
    0X9601, 0X56C0, 0X5780, 0X9741, 0X5500, 0X95C1, 0X9481, 0X5440,
    0X9C01, 0X5CC0, 0X5D80, 0X9D41, 0X5F00, 0X9FC1, 0X9E81, 0X5E40,
    0X5A00, 0X9AC1, 0X9B81, 0X5B40, 0X9901, 0X59C0, 0X5880, 0X9841,
    0X8801, 0X48C0, 0X4980, 0X8941, 0X4B00, 0X8BC1, 0X8A81, 0X4A40,
    0X4E00, 0X8EC1, 0X8F81, 0X4F40, 0X8D01, 0X4DC0, 0X4C80, 0X8C41,
    0X4400, 0X84C1, 0X8581, 0X4540, 0X8701, 0X47C0, 0X4680, 0X8641,
    0X8201, 0X42C0, 0X4380, 0X8341, 0X4100, 0X81C1, 0X8081, 0X4040
    };

    unsigned char tmp;
    unsigned short crc = 0xFFFF;

    while(len--) {
        tmp = *data++ ^ crc;
        crc = crc >> 8;
        crc ^= crc_table[tmp];
    }

    return crc;

}
