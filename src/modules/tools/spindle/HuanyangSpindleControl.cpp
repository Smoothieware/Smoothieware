/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

/*
     Sako SKI780 RS485 communication protocol

    Got the Sako SKI780 RS485 communication protocol form the manufacture
    Posted it on: https://github.com/Rickishier/SKI780-485-communication-protocol-English-Edition

    Parameters

    P0-02   2      Command source selection
    P0-03   9      Main frequenty source 9 = comunication settings
    PD-00   6005   Baud rate   
    PD-01   3      Modbus data format, 8N1
    PD-02   1      Local address: 1
    PD-05   31     Modbus protocol selection

    == Control Write ==

    ADDR    CMD     LEN     		DATA		CRC
    0x01    0x06    0x20 0x00	0x00 0x01    0x43 0xCA                   Start spindle clockwise

    ADDR    CMD     LEN				DATA		CRC
    0x01    0x06    0x20 0x00	0x00 0x06    0x02 0x08                   Stop spindle

    ADDR    CMD     LEN           	DATA    	CRC
    0x01    0x06    0x20 0x00    0x00 0x02	0x30 0x44              Start spindle counter-clockwise
0
    == Control Read ==

    ADDR    CMD     LEN     PAR     DATA        CRC
    0x01    0x03    0x10    0x01    0x00 0x01   0xD1 0x0A       Read running Frequency
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
    char turn_on_msg[8] = { 0x01, 0x06, 0x20, 0x00, 0x00, 0x01, 0x43, 0xCA};
    // calculate CRC16 checksum
    //unsigned int crc = modbus->crc16(turn_on_msg, sizeof(turn_on_msg)-2);
    //turn_on_msg[6] = crc & 0xFF;
    //turn_on_msg[7] = (crc >> 8);

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

// FIXME this cannot call delay() as it calls on_idle and this can be called from on_idle so recursion
void HuanyangSpindleControl::turn_off()
{
    // prepare data for the spindle off command
    char turn_off_msg[8] = { 0x01, 0x06, 0x20, 0x00, 0x00, 0x06, 0x02, 0x08};
    // calculate CRC16 checksum
    //unsigned int crc = modbus->crc16(turn_off_msg, sizeof(turn_off_msg)-2);
    //turn_off_msg[6] = crc & 0xFF;
    //turn_off_msg[7] = (crc >> 8);

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
    char set_speed_msg[8] = { 0x01, 0x06, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00};
    // convert RPM into Hz
    unsigned int hz = target_rpm / 240 * 100;
    set_speed_msg[4] = (hz >> 8);
    set_speed_msg[5] = hz & 0xFF;
    // calculate CRC16 checksum
    unsigned int crc = modbus->crc16(set_speed_msg, sizeof(set_speed_msg)-2);
    set_speed_msg[7] = (crc >> 8);
	set_speed_msg[6] = crc & 0xFF;

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
    char get_speed_msg[8] = { 0x01, 0x03, 0x10, 0x01, 0x00, 0x01, 0xD1, 0x0A };
    // calculate CRC16 checksum  CBF0
    //unsigned int crc = modbus->crc16(get_speed_msg, sizeof(get_speed_msg)-2);
    //get_speed_msg[6] = crc & 0xFF;
    //get_speed_msg[7] = (crc >> 8);

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
    char speed[7] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    // read the answer into the buffer
    for(int i=0; i<8; i++) {
        speed[i] = modbus->serial->getc();
    }
    // get the Hz value from trhe answer and convert it into an RPM value
    unsigned int hz = (speed[3] << 8) | speed[4];
    unsigned int rpm = hz / 100 * 60;

    // report the current RPM value
    THEKERNEL->streams->printf("Current RPM: %d\n", rpm);
}
