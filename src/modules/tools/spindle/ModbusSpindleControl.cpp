/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "libs/Module.h"
#include "libs/Kernel.h"
#include "libs/Pin.h"
#include "mbed.h"
#include "Modbus.h"
#include "Config.h"
#include "checksumm.h"
#include "ConfigValue.h"
#include "ModbusSpindleControl.h"

#define spindle_checksum                    CHECKSUM("spindle")
#define spindle_rx_pin_checksum             CHECKSUM("rx_pin")
#define spindle_tx_pin_checksum             CHECKSUM("tx_pin")
#define spindle_dir_pin_checksum            CHECKSUM("dir_pin")

void ModbusSpindleControl::on_module_loaded()
{

    spindle_on = false;
    PinName rx_pin;
    PinName tx_pin;
    PinName dir_pin;
    
    // preparing PinName objects from the config string
    {
        Pin *smoothie_pin = new Pin();
        smoothie_pin->from_string(THEKERNEL->config->value(spindle_checksum, spindle_rx_pin_checksum)->by_default("nc")->as_string());
        smoothie_pin->as_input();
        rx_pin = port_pin((PortName)smoothie_pin->port_number, smoothie_pin->pin);
        
        smoothie_pin->from_string(THEKERNEL->config->value(spindle_checksum, spindle_tx_pin_checksum)->by_default("nc")->as_string());
        smoothie_pin->as_input();
        tx_pin = port_pin((PortName)smoothie_pin->port_number, smoothie_pin->pin);
        
        smoothie_pin->from_string(THEKERNEL->config->value(spindle_checksum, spindle_dir_pin_checksum)->by_default("nc")->as_string());
        smoothie_pin->as_input();
        dir_pin = port_pin((PortName)smoothie_pin->port_number, smoothie_pin->pin);

        delete smoothie_pin;
    }

    // setup the Modbus interface
    modbus = new Modbus(tx_pin, rx_pin, dir_pin);
}

