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
#include "TSRingBuffer.h"

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
        void step_tick (void);
        void handle_finish (void);
        float get_total_time() const { return total_move_time/frequency; }
        void start();
        bool add_job(const Block *block) { return push_block(block); }
        bool is_jobq_full() const { return jobq.full(); }
        bool is_jobq_empty() const { return jobq.empty(); }

        // whatever setup the block should register this to know when it is done
        std::function<void()> finished_fnc{nullptr};

        static StepTicker *getInstance() { return instance; }

    private:
        static StepTicker *instance;

        bool push_block(const Block*);
        bool pop_next_job();

        float frequency;
        uint32_t period;
        std::array<StepperMotor*, k_max_actuators> motor;
        std::bitset<k_max_actuators> unstep;
        uint32_t total_move_time{0};

        // this is tick info needed for this block. applies to all motors
        using block_info_t = struct {
            uint32_t accelerate_until;
            uint32_t decelerate_after;
            uint32_t total_move_ticks;
            std::bitset<k_max_actuators> direction_bits;     // Direction for each axis in bit form, relative to the direction port's mask
        };

        // this is the data needed to determine when each motor needs to be issued a step
        using tickinfo_t= struct {
            int32_t steps_per_tick; // 2.30 fixed point
            int32_t counter; // 2.30 fixed point
            int32_t acceleration_change; // 2.30 fixed point signed
            int32_t deceleration_change; // 2.30 fixed point
            int32_t plateau_rate; // 2.30 fixed point
            uint32_t steps_to_move;
            uint32_t step_count;
            uint32_t next_accel_event;
        };

        using job_entry_t= struct {
            block_info_t block_info;
            std::array<tickinfo_t, k_max_actuators> tick_info;
        };

        // Thread safe for single consumer and single provider
        #define jobq_size 32
        TSRingBuffer<job_entry_t, jobq_size> jobq;
        job_entry_t current_job;

        struct {
            volatile bool running:1;
            uint8_t num_motors:4;
        };
};
