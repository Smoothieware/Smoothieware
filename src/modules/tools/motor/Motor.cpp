/*
    This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
    Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
    Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
    You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "Motor.h"
#include "Module.h"
#include "Kernel.h"
#include "nuts_bolts.h"
#include "Config.h"
#include "ConfigValue.h"
#include "utils.h"
#include "checksumm.h"

#include "Pin.h"
#include "Gcode.h"

#define motor_checksum                           CHECKSUM("motor")
#define enable_checksum                          CHECKSUM("enable")
#define tick_pin_checksum                        CHECKSUM("tick_pin")
#define home_pin_checksum                        CHECKSUM("home_pin")
#define clockwise_pin_checksum                   CHECKSUM("clockwise_pin")
#define counter_clockwise_pin_checksum           CHECKSUM("counter_clockwise_pin")

Motor::Motor(){

}

void Motor::on_module_loaded(){

  // Read module configuration
  // If the module is disabled -> do nothing
  if(!THEKERNEL->config->value( motor_checksum, enable_checksum )->by_default(false)->as_bool()) {
      // As this module is not needed free up the resource
      delete this;
      return;
  }

  // Tick pin is used to count each time the motor moves by one unit of movement
  this->tick_pin.from_string( THEKERNEL->config->value(motor_checksum, tick_pin_checksum)->by_default("nc" )->as_string())->as_input();

  // Home pin is used to detect/seek the origin of the motor movement, so we can go back to a known position in absolute space
  this->home_pin.from_string( THEKERNEL->config->value(motor_checksum, home_pin_checksum)->by_default("nc" )->as_string())->as_input();

  // Clockwise pin is used to instruct the motor to move in the clockwise direction
  this->clockwise_pin.from_string( THEKERNEL->config->value(motor_checksum, clockwise_pin_checksum)->by_default("nc" )->as_string())->as_output();

  // Counter clockwise pin is used to instruct the motor to move in the counter clockwise direction
  this->counter_clockwise_pin.from_string( THEKERNEL->config->value(motor_checksum, counter_clockwise_pin_checksum)->by_default("nc" )->as_string())->as_output();


}

void Motor::on_gcode_received(void *argument){
    Gcode *gcode = static_cast<Gcode *>(argument);

    // M codes execute immediately
    if (gcode->has_m && 0) {

    }
}
