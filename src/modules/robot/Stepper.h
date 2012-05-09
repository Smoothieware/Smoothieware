/*  
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>. 
*/

#ifndef STEPPER_H
#define STEPPER_H
#include "libs/Module.h"
#include "libs/Kernel.h"
#include "Planner.h"
#include "Block.h"

#define microseconds_per_step_pulse_checksum        42333
#define acceleration_ticks_per_second_checksum      25075
#define minimum_steps_per_minute_checksum           9003
#define base_stepping_frequency_checksum            21918
#define alpha_step_pin_checksum                     11468
#define beta_step_pin_checksum                      22114
#define gamma_step_pin_checksum                     1225
#define alpha_dir_pin_checksum                      55887
#define beta_dir_pin_checksum                       28644
#define gamma_dir_pin_checksum                      46412
#define alpha_en_pin_checksum                       35042  
#define beta_en_pin_checksum                        34680 
#define gamma_en_pin_checksum                       26335 


class Stepper : public Module {
    public:
        Stepper();
        void on_module_loaded();
        void on_config_reload(void* argument);
        void on_block_begin(void* argument);
        void on_block_end(void* argument);
        void on_gcode_execute(void* argument);
        void on_play(void* argument);
        void on_pause(void* argument);
        uint32_t main_interrupt(uint32_t dummy);
        void trapezoid_generator_reset();
        void set_step_events_per_minute(double steps_per_minute);
        uint32_t trapezoid_generator_tick(uint32_t dummy);
        uint32_t reset_step_pins(uint32_t dummy);
        void update_offsets();
        int config_step_timer( int cycles );

        Block* current_block;
        int counters[3];
        int stepped[3]; 
        int offsets[3]; 
        float counter_alpha;
        float counter_beta;
        float counter_gamma;
        int step_events_completed; 
        unsigned int out_bits; 
        double trapezoid_adjusted_rate;
        int trapezoid_tick_cycle_counter;
        int cycles_per_step_event; 
        bool trapezoid_generator_busy;
        int microseconds_per_step_pulse; 
        int acceleration_ticks_per_second;
        int divider;
        int minimum_steps_per_minute;
        int base_stepping_frequency;
        Pin* alpha_step_pin;
        Pin* beta_step_pin;
        Pin* gamma_step_pin;
        Pin* alpha_dir_pin;
        Pin* beta_dir_pin;
        Pin* gamma_dir_pin;
        Pin* alpha_en_pin;
        Pin* beta_en_pin;
        Pin* gamma_en_pin;
        unsigned short step_bits[3];
        int counter_increment;
        bool paused;
};




#endif
