/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

/* 
    Hunayang RS485 communication protocol

    originally found on cnczone.nl, posted by Rikkepic:
    http://cnczone.nl/viewtopic.php?f=35&t=11605

    Parameters

    PD001   2   RS485 Control of run commands
    PD002   2   RS485 Control of operating frequency
    PD023   1   Reverse run enabled
    PD163   1   RS485 Address: 1
    PD164   1   RS485 Baud rate: 9600
    PD165   3   RS485 Mode: RTU, 8N1

    == Function Read == 

    ADDR    CMD     LEN     PAR     DATA        CRC
    0x01    0x01    0x03    0xA5    0x00 0x00   0x2C 0x6D       Read PD165 (165=0xA5)

    == Function Write ==

    ADDR    CMD     LEN     PAR     DATA        CRC
    0x01    0x02    0x03    0x03    0x09 0xC4   0x8F 0x8D       Write PD003 (0x9C4 = 2500 = 25.00Hz)

    == Control Write ==

    ADDR    CMD     LEN     DATA    CRC
    0x01    0x03    0x01    0x01    0x31 0x88                   Start spindle clockwise

    ADDR    CMD     LEN     DATA    CRC
    0x01    0x03    0x01    0x08    0xF1 0x8E                   Stop spindle

    ADDR    CMD     LEN     DATA    CRC
    0x01    0x03    0x01    0x11    0x30 0x44                   Start spindle counter-clockwise

    == Control Read ==

    ADDR    CMD     LEN     PAR     DATA        CRC
    0x01    0x04    0x03    0x00    0x00 0x00   0xF0 0x4E       Read Frequency

    ADDR    CMD     LEN     PAR     DATA        CRC
    0x01    0x04    0x03    0x02    0x00 0x00   0x51 0x8E       Read Output Current

    ADDR    CMD     LEN     PAR     DATA        CRC
    0x01    0x04    0x03    0x03    0x00 0x00   0x00 0x4E       Read Rotation

    ADDR    CMD     LEN     PAR     DATA        CRC
    0x01    0x04    0x03    0x04    0x00 0x00   0xB1 0x8F       Read DC Volatge

    ADDR    CMD     LEN     PAR     DATA        CRC
    0x01    0x04    0x03    0x05    0x00 0x00   0xE0 0x4F       Read AC Voltage

    ADDR    CMD     LEN     PAR     DATA        CRC
    0x01    0x04    0x03    0x07    0x00 0x00   0x41 0x8F       Read Temperature

    == Control Read ==

    ADDR    CMD     LEN     DATA        CRC
    0x01    0x05    0x02    0x09 0xC4   0xBF 0x0F               Write Frequency (0x9C4 = 2500 = 25.00HZ)

*/

#include "libs/Kernel.h"
#include "StreamOutputPool.h"
#include "BufferedSoftSerial.h"
#include "ModbusSpindleControl.h"
#include "HuanyangSpindleControl.h"
#include "gpio.h"
#include "Modbus.h"

void HuanyangSpindleControl::turn_on() 
{
    // prepare data for the spindle off command
    char turn_on_msg[6] = { 0x01, 0x03, 0x01, 0x01, 0x00, 0x00 };
    // calculate CRC16 checksum
    unsigned int crc = modbus->crc16(turn_on_msg, sizeof(turn_on_msg)-2);
    turn_on_msg[4] = crc & 0xFF;
    turn_on_msg[5] = (crc >> 8);

    // enable transmitter
    modbus->dir_output->set();
    modbus->delay(1);
    // send the actual message
    modbus->serial->write(turn_on_msg, sizeof(turn_on_msg));
    // wait a calculated time for the data to be sent 
    modbus->delay((int) ceil(sizeof(turn_on_msg) * modbus->delay_time));
    // disable transmitter
    modbus->dir_output->clear();
    // wait 50ms, required by the Modbus standard
    modbus->delay(50);
    spindle_on = true;

}

void HuanyangSpindleControl::turn_off() 
{
    // prepare data for the spindle off command
    char turn_off_msg[6] = { 0x01, 0x03, 0x01, 0x08, 0x00, 0x00 };
    // calculate CRC16 checksum
    unsigned int crc = modbus->crc16(turn_off_msg, sizeof(turn_off_msg)-2);
    turn_off_msg[4] = crc & 0xFF;
    turn_off_msg[5] = (crc >> 8);

    // enable transmitter
    modbus->dir_output->set();
    modbus->delay(1);
    // send the actual message
    modbus->serial->write(turn_off_msg, sizeof(turn_off_msg));
    // wait a calculated time for the data to be sent 
    modbus->delay((int) ceil(sizeof(turn_off_msg) * modbus->delay_time));
    // disable transmitter
    modbus->dir_output->clear();
    // wait 50ms, required by the Modbus standard
    modbus->delay(50);
    spindle_on = false;

}

void HuanyangSpindleControl::set_speed(int target_rpm) 
{

    // prepare data for the set speed command
    char set_speed_msg[7] = { 0x01, 0x05, 0x02, 0x00, 0x00, 0x00, 0x00 };
    // convert RPM into Hz
    unsigned int hz = target_rpm / 60 * 100; 
    set_speed_msg[3] = (hz >> 8);
    set_speed_msg[4] = hz & 0xFF;
    // calculate CRC16 checksum
    unsigned int crc = modbus->crc16(set_speed_msg, sizeof(set_speed_msg)-2);
    set_speed_msg[5] = crc & 0xFF;
    set_speed_msg[6] = (crc >> 8);

    // enable transmitter
    modbus->dir_output->set();
    modbus->delay(1);
    // send the actual message
    modbus->serial->write(set_speed_msg, sizeof(set_speed_msg));
    // wait a calculated time for the data to be sent 
    modbus->delay((int) ceil(sizeof(set_speed_msg) * modbus->delay_time));
    // disable transmitter
    modbus->dir_output->clear();
    // wait 50ms, required by the Modbus standard
    modbus->delay(50);

}

void HuanyangSpindleControl::report_speed() 
{
    // clear RX buffer before start
    while(modbus->serial->readable()){
        modbus->serial->getc();
    }

    // prepare data for the get speed command
    char get_speed_msg[8] = { 0x01, 0x04, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00 };
    // calculate CRC16 checksum
    unsigned int crc = modbus->crc16(get_speed_msg, sizeof(get_speed_msg)-2);
    get_speed_msg[6] = crc & 0xFF;
    get_speed_msg[7] = (crc >> 8);

    // enable transmitter
    modbus->dir_output->set();
    modbus->delay(1);
    // send the actual message
    modbus->serial->write(get_speed_msg, sizeof(get_speed_msg));
    // wait a calculated time for the data to be sent 
    modbus->delay((int) ceil(sizeof(get_speed_msg) * modbus->delay_time));
    // disable transmitter
    modbus->dir_output->clear();
    // wait 50ms, required by the Modbus standard
    modbus->delay(50);

    // wait for the complete message to be received
    modbus->delay((int) ceil(8 * modbus->delay_time));
    // prepare an array for the answer
    char speed[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
   
    // read the answer into the buffer
    for(int i=0; i<8; i++) {
        speed[i] = modbus->serial->getc();
    }
    // get the Hz value from trhe answer and convert it into an RPM value
    unsigned int hz = (speed[4] << 8) | speed[5];
    unsigned int rpm = hz / 100 * 60;

    // report the current RPM value
    THEKERNEL->streams->printf("Current RPM: %d\n", rpm);
}
