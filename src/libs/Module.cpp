/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "libs/Module.h"
#include "libs/Kernel.h"

Module::Module(){ }

void Module::on_module_loaded(){
}

void Module::register_for_event(int event_id){
    this->kernel->register_for_event(event_id, this);
}

void Module::on_main_loop(             void * argument){}
void Module::on_console_line_received( void * argument){}
void Module::on_gcode_received(        void * argument){}
void Module::on_stepper_wake_up(       void * argument){}
void Module::on_gcode_execute(         void * argument){}
void Module::on_speed_change(          void * argument){}
void Module::on_block_begin(           void * argument){}
void Module::on_block_end(             void * argument){}
void Module::on_config_reload(         void * argument){}
void Module::on_play(                  void * argument){}
void Module::on_pause(                 void * argument){}
void Module::on_idle(                  void * argument){}
