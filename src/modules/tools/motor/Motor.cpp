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
#include "SlowTicker.h"


#include "Pin.h"
#include "Gcode.h"

#define motor_checksum                           CHECKSUM("motor")
#define enable_checksum                          CHECKSUM("enable")
#define tick_pin_checksum                        CHECKSUM("tick_pin")
#define home_pin_checksum                        CHECKSUM("home_pin")
#define clockwise_pin_checksum                   CHECKSUM("clockwise_pin")
#define counter_clockwise_pin_checksum           CHECKSUM("counter_clockwise_pin")


Motor::Motor(){
  // Set the status to the default value
  this->status = NONE;

  // When we start, the current position is 0
  this->position = 0;

}

void Motor::on_module_loaded(){

  // Read module configuration
  // If the module is disabled -> do nothing
  if(!THEKERNEL->config->value( motor_checksum, enable_checksum )->by_default(false)->as_bool()) {
      // As this module is not needed free up the resource
      delete this;
      return;
  }

  // Register for events we want to listen on
  register_for_event(ON_GCODE_RECEIVED);

  // Tick pin is used to count each time the motor moves by one unit of movement
  this->tick_pin.from_string( THEKERNEL->config->value(motor_checksum, tick_pin_checksum)->by_default("nc" )->as_string())->as_input();

  // Home pin is used to detect/seek the origin of the motor movement, so we can go back to a known position in absolute space
  this->home_pin.from_string( THEKERNEL->config->value(motor_checksum, home_pin_checksum)->by_default("nc" )->as_string())->as_input();

  // Clockwise pin is used to instruct the motor to move in the clockwise direction
  this->clockwise_pin.from_string( THEKERNEL->config->value(motor_checksum, clockwise_pin_checksum)->by_default("nc" )->as_string())->as_output();

  // Counter clockwise pin is used to instruct the motor to move in the counter clockwise direction
  this->counter_clockwise_pin.from_string( THEKERNEL->config->value(motor_checksum, counter_clockwise_pin_checksum)->by_default("nc" )->as_string())->as_output();

  // Set up a regular tick to read pins and do things
  THEKERNEL->slow_ticker->attach(1000, this, &Motor::tick);

}

void Motor::on_gcode_received(void *argument){
    Gcode *gcode = static_cast<Gcode *>(argument);

    // M codes execute immediately

    // M570 is used to find the origin by moving counter-clockwise until the home pin gets to a high value
    // When the home pin is hit, the absolute position (count) of the tick pin is reset
    if (gcode->has_m && gcode->m == 570 && this->status == NONE ){
      // Only if we actually have a counterclockwise pin
      if( this->counter_clockwise_pin.connected() ){
        // Set the current status to 'homing'
        this->status = HOMING;

        // Start moving towards the home button, counterclockwise
        this->counter_clockwise_pin.set(HIGH);
      }
    }
    // M571 is used to move the motor to a specific position (count) of the tick pin
    if (gcode->has_m && gcode->m == 571 && this->status == NONE ){
      // Read the target position, as either absolute or relative

      // A is for Absolute positionning
      if( gcode->has_letter('A') ){
        this->target_position = gcode->get_value('A');

      // R is for Relative positionning
      }else if( gcode->has_letter('R') ){
        this->target_position = this->position + gcode->get_value('R');
      }

      // Set the current status to MOVING_TO
      if( this->position != this->target_position){
        // Change status to say we are currently moving to a new position
        this->status = MOVING_TO;

        // Start moving in the right direction
        if( this->target_position > this->position ){
          // If moving to a positive position relative to the current one, start moving clockwise
          this->clockwise_pin.set(HIGH);
        }else{
          // Otherwise, move counter clockwise
          this->counter_clockwise_pin.set(HIGH);
        }
      }
    }
    // M572 is used to set the motor moving in one direction or another ( or to stop it )
    // M579 is used to read the current status ( active action, absolute position ) of this module
}

// Executed 1000 times a second, to check the various pins, and do actions as needed
uint32_t Motor::tick(uint32_t dummy){

  // If we are homing, and the value of the home pin changes
  if( this->status == HOMING && this->current_home_pin_value != this->home_pin.get() ){
    // We have succesfully homed
    // Change the status to not homing
    this->status = NONE;

    // Stop the motor movement
    this->counter_clockwise_pin.set(LOW);

    // The positon is now 0, as we are at the beginning of the axis
    this->position = 0;
  }

  // If we are moving, and the value of the tick pin changes from LOW to HIGH
  if( this->status == MOVING_TO && this->current_tick_pin_value != this->tick_pin.get() && this->current_tick_pin_value == LOW ){
    // We have hit a tick, a marker along the way, act depending on the direction we are moving towards
    // If moving clockwise
    if( this->target_position > this->position ){
      this->position++;
    // If moving counter clockwise
    }else{
      this->position--;
    }

    // If we have arrived
    if( this->position == this->target_position ){
      this->status = NONE;
      this->clockwise_pin.set(LOW);
      this->counter_clockwise_pin.set(LOW);
    }
  }

  // Remember the home pin current value so we can check if it changed or not
  this->current_home_pin_value = this->home_pin.get();

  // Remember the tick pin current value so we can check if it changed or not
  this->current_tick_pin_value = this->tick_pin.get();

  return dummy;
}













// End of file
