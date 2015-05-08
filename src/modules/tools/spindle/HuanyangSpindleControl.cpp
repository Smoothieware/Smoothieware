/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "ModbusSpindleControl.h"
#include "HuanyangSpindleControl.h"
#include "gpio.h"
#include "Modbus.h"

// Examples:
// https://github.com/bebro/linuxcnc-huanyang-vfd/blob/emc2/hy_modbus.c

void HuanyangSpindleControl::turn_on() 
{

    char turn_on_msg[6] = { 0x01, 0x03, 0x01, 0x01, 0x00, 0x00 };
    unsigned int crc = modbus->crc16(turn_on_msg, 4);
    turn_on_msg[4] = crc & 0xFF;
    turn_on_msg[5] = (crc >> 8);

    // Send cippled Modbus telegram for spindle ON
    modbus->dir_output->set();
    modbus->delay(1);
    modbus->serial->write(turn_on_msg, 6);
    modbus->delay((int) ceil(6 * modbus->delay_time));
    modbus->dir_output->clear();
    modbus->delay(50);
    spindle_on = true;

}

void HuanyangSpindleControl::turn_off() 
{

    char turn_off_msg[6] = { 0x01, 0x03, 0x01, 0x08, 0x00, 0x00 };
    unsigned int crc = modbus->crc16(turn_off_msg, 4);
    turn_off_msg[4] = crc & 0xFF;
    turn_off_msg[5] = (crc >> 8);

    // Send Modbus telegram for spindle OFF
    modbus->dir_output->set();
    modbus->delay(1);
    modbus->serial->write(turn_off_msg, 6);
    modbus->delay((int) ceil(6 * modbus->delay_time));
    modbus->dir_output->clear();
    modbus->delay(50);
    spindle_on = false;

}

void HuanyangSpindleControl::set_speed(int target_rpm) 
{

    // sizeof(set_speed_msg) verwenden
    //char set_speed_msg[8] = { 0x01, 0x02, 0x03, 0x03, 0x00, 0x00, 0x00, 0x00 };
    char set_speed_msg[7] = { 0x01, 0x05, 0x02, 0x00, 0x00, 0x00, 0x00 };
    // convert RPM into Hz
    unsigned int hz = target_rpm / 60 * 100; 
    set_speed_msg[3] = (hz >> 8);
    set_speed_msg[4] = hz & 0xFF;
    
    unsigned int crc = modbus->crc16(set_speed_msg, sizeof(set_speed_msg)-2);
    set_speed_msg[5] = crc & 0xFF;
    set_speed_msg[6] = (crc >> 8);

    // Send Modbus telegram for spindle speed change
    modbus->dir_output->set();
    modbus->delay(1);
    modbus->serial->write(set_speed_msg, sizeof(set_speed_msg));
    modbus->delay((int) ceil(sizeof(set_speed_msg) * modbus->delay_time));
    modbus->dir_output->clear();
    modbus->delay(50);

}

void HuanyangSpindleControl::report_speed() 
{

    // TODO: implement this

}
