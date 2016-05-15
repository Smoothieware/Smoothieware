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
        bool is_moving(int i) const { return tick_info[i].steps_to_move != 0; }
        uint32_t get_stepped(int i) const { return tick_info[i].step_count; }
        void unstep_tick();
        void step_tick (void);
        void handle_finish (void);

        void start();
        bool add_job(Block *block) { return jobq.put(block); }
        bool is_jobq_full() const { return jobq.full(); }

        // whatever setup the block should register this to know when it is done
        std::function<void()> finished_fnc{nullptr};

        static StepTicker *getInstance() { return instance; }

    private:
        static StepTicker *instance;

            //
            //  Simple fixed size ring buffer.
            //  Manage objects by value.
            //  Thread safe for single Producer and single Consumer.
            //  By Dennis Lang http://home.comcast.net/~lang.dennis/code/ring/ring.html
            //  Slightly modified for naming

            template <class T, size_t RingSize>
            class TSRingBuffer
            {
            public:
                TSRingBuffer()
                    : m_size(RingSize), m_buffer(new T[RingSize]), m_rIndex(0), m_wIndex(0)
                { }

                ~TSRingBuffer()
                {
                    delete [] m_buffer;
                };

                size_t next(size_t n) const
                {
                    return (n + 1) % m_size;
                }

                bool empty() const
                {
                    return (m_rIndex == m_wIndex);
                }

                bool full() const
                {
                    return (next(m_wIndex) == m_rIndex);
                }

                bool put(const T &value)
                {
                    if (full())
                        return false;
                    m_buffer[m_wIndex] = value;
                    m_wIndex = next(m_wIndex);
                    return true;
                }

                bool get(T &value)
                {
                    if (empty())
                        return false;
                    value = m_buffer[m_rIndex];
                    m_rIndex = next(m_rIndex);
                    return true;
                }

            private:
                size_t          m_size;
                T              *m_buffer;

                // volatile is only used to keep compiler from placing values in registers.
                // volatile does NOT make the index thread safe.
                volatile size_t m_rIndex;
                volatile size_t m_wIndex;
            };

        void copy_block(Block *block);

        float frequency;
        uint32_t period;
        std::array<StepperMotor*, k_max_actuators> motor;
        std::bitset<k_max_actuators> unstep;

        // this is tick info needed for this block. applies to all motors
        struct block_info_t {
            uint32_t accelerate_until;
            uint32_t decelerate_after;
            uint32_t total_move_ticks;
        };
        block_info_t block_info;

        TSRingBuffer<Block*, 4> jobq;

        // this is the data needed to determine when each motor needs to be issued a step
        struct tickinfo_t {
            int32_t steps_per_tick; // 2.30 fixed point
            int32_t counter; // 2.30 fixed point
            int32_t acceleration_change; // 2.30 fixed point signed
            int32_t deceleration_change; // 2.30 fixed point
            int32_t plateau_rate; // 2.30 fixed point
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
