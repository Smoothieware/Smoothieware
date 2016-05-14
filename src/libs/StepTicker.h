/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/



#pragma once

#include <stdint.h>
#include <array>
#include <bitset>
#include <functional>
#include <atomic>

#include "ActuatorCoordinates.h"

class StepperMotor;
class Block;

class StepTicker{
    public:
        StepTicker();
        ~StepTicker();
        void set_frequency( float frequency );
        void signal_a_move_finished();
        void set_unstep_time( float microseconds );
        int register_motor(StepperMotor* motor);
        float get_frequency() const { return frequency; }
        void unstep_tick();

        void TIMER0_IRQHandler (void);
        void PendSV_IRQHandler (void);

        void start();
        void copy_block(Block *block);
        void set_next_block(Block *block) { next_block= block; }
        bool is_next_block() const { return next_block != nullptr; }

        // whatever setup the block should register this to know when it is done
        std::function<void()> finished_fnc{nullptr};

        static StepTicker *getInstance() { return instance; }

    private:
        static StepTicker *instance;

        float frequency;
        uint32_t period;
        std::array<StepperMotor*, k_max_actuators> motor;
        std::atomic_uchar do_move_finished;
        std::bitset<k_max_actuators> unstep;

        // this is tick info needed for this block. applies to all motors
        struct block_info_t {
            uint32_t accelerate_until;
            uint32_t decelerate_after;
            uint32_t maximum_rate;
            uint32_t deceleration_per_tick;
            uint32_t total_move_ticks;
        };
        block_info_t block_info;
        Block *next_block{nullptr};

        // this is the data needed to determine when each motor needs to be issued a step
        struct tickinfo_t {
            float steps_per_tick; // 2.30 fixed point
            float counter; // 2.30 fixed point
            float acceleration_change; // 1.30 fixed point signed
            float axis_ratio;
            uint32_t steps_to_move;
            uint32_t step_count;
            uint32_t next_accel_event;
        };
        std::array<tickinfo_t, k_max_actuators> tick_info;

        struct {
            volatile bool move_issued:1;
            uint8_t num_motors:4;
        };
};
