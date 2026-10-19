/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include <bitset>
#include <stddef.h>
#include <stdint.h>
#include "ActuatorCoordinates.h"

class Block {
    public:
        Block();

        static void init(uint8_t);

        // The description of what one motor does in this block that every moving motor needs (40 bytes)
        using tickinfo_t= struct {
            int64_t steps_per_tick; // 2.62 fixed point
            int64_t acceleration_change; // 2.62 fixed point signed
            int64_t deceleration_change; // 2.62 fixed point
            int64_t plateau_rate; // 2.62 fixed point
            uint32_t steps_to_move;
            uint32_t next_accel_event;
        };

        // What only a compensated motor needs (32 bytes), stored after the axis records and only for the motors in
        // ext_mask: a motor that filament advance reshapes, or that has backlash slack to cross. Every other motor
        // reads the defaults of k_no_ext, so the common move pays nothing for either feature.
        using tickext_t= struct {
            int64_t decel_start_rate_delta; // 2.62 fixed point signed: added to steps_per_tick when the deceleration begins, 0 = none
            int64_t takeup_rate; // 2.62 fixed point, backlash: extra steps per tick added to the rate until takeup_end_tick, 0 = none
            int32_t position_correction_steps; // backlash: added to the motor's step counter when the block is started from the queue (never on a re-feed), 0 = none
            uint32_t reversal_tick; // tick at which the motor reverses direction, NO_EVENT = never
            uint32_t takeup_end_tick; // backlash: first tick without the extra rate, NO_EVENT = never
            uint32_t reserved;      // keeps the record a multiple of 8 bytes
        };

        // A block is stored in the queue arena as this header, then the small per-motor arrays for every registered motor
        // (steps, shaper_state), then one axis record (tick_info) for each motor that really moves in this block, in motor
        // order, then one extension record (ext_info) for each of those that is compensated. bytes_needed() is the largest size of that record (every motor moves) and is what the queue reserves for a
        // block that is being filled; bytes_used() is the size once set_active() knows which motors move, and the queue
        // keeps only that much. Both are multiples of 8. bind() is called once, on freshly placed storage.
        static size_t bytes_needed();
        // the size of a block in which every motor moves and none is compensated
        static size_t bytes_dense();
        size_t bytes_used() const;
        void bind();

        // Call once the steps[] of the block are final: allocates and initialises one axis record for each motor with
        // steps, and from then on axis(m) is valid for exactly those motors. Before this call no motor has an axis record.
        // correction[m] is the backlash slack correction of motor m (0 = none); shaped_motors must already be set.
        void set_active(const int32_t *correction);
        bool is_active(uint8_t m) const { return (active_mask >> m) & 1u; }
        // The axis record of an active motor (is_active(m) must be true)
        tickinfo_t *axis(uint8_t m);
        const tickinfo_t *axis(uint8_t m) const;
        // The backlash slack correction of a motor, 0 for a motor that does not move in this block
        int32_t correction_steps(uint8_t m) const { return ext_of(m).position_correction_steps; }

        static const tickext_t k_no_ext;
        bool has_ext(uint8_t m) const { return (ext_mask >> m) & 1u; }
        // The extension record of a motor with has_ext(m)
        tickext_t *ext(uint8_t m);
        // The extension record of a motor, or the defaults (no compensation) when it has none
        const tickext_t &ext_of(uint8_t m) const;

        // previous is the block that runs just before this one (nullptr if none is queued). A compensation that reshapes
        // a motor's rate (filament advance) needs it: it tells where the previous block left that motor.
        void calculate_trapezoid( float entry_speed, float exit_speed, const Block *previous = nullptr );

        float reverse_pass(float exit_speed);
        float forward_pass(float next_entry_speed);
        float max_exit_speed();
        void debug() const;
        void ready() { is_ready= true; }
        void clear();

    private:
        float max_allowable_speed( float acceleration, float target_velocity, float distance);
        void prepare(float acceleration_in_steps, float deceleration_in_steps, const Block *previous);

        static double fp_scale; // optimize to store this as it does not change

    public:
        uint32_t *steps;             // Number of steps for each axis for this block, n_actuators entries (trailing storage)
        uint32_t steps_event_count;  // Steps for the longest axis
        float nominal_rate;       // Nominal rate in steps per second
        float nominal_speed;      // Nominal speed in mm per second
        float millimeters;        // Distance for this move
        float entry_speed;
        float exit_speed;
        float acceleration;       // the acceleration for this block
        float initial_rate;       // Initial rate in steps per second
        float maximum_rate;

        float max_entry_speed;

        // this is tick info needed for this block. applies to all motors
        uint32_t accelerate_until;
        uint32_t decelerate_after;
        uint32_t total_move_ticks;
        std::bitset<k_max_actuators> direction_bits;     // Direction for each axis in bit form, relative to the direction port's mask

        // Motors whose step rate a compensation reshapes in this block (filament advance, see MotionCompensation.h).
        // Set by the Planner when the block is appended.
        std::bitset<k_max_actuators> shaped_motors;
        // Set by prepare(): per motor, the compensation's carry-over to the next block (for filament advance: how many
        // steps ahead of the nominal position the motor is at the end of this block). 0 for every motor that is not shaped.
        int32_t *shaper_state;   // n_actuators entries (trailing storage)

        static const uint32_t NO_EVENT = 0xFFFFFFFFu; // a tick that never comes

        // The read-only description of what each motor does in this block: the step interrupt copies it into its own
        // run state when the block starts and never writes it. steps_per_tick, acceleration_change and next_accel_event
        // are the values at tick 0.
        // one record for each motor that moves in this block, in motor order: active_count entries (trailing storage)
        tickinfo_t *tick_info;
        uint32_t active_mask;   // bit m set = motor m has an axis record
        uint8_t active_count;
        uint32_t ext_mask;      // bit m set = motor m (also in active_mask) has an extension record
        uint8_t ext_count;
        tickext_t *ext_info;    // ext_count records, trailing storage after the axis records

        static uint8_t n_actuators;

        struct {
            bool recalculate_flag:1;             // Planner flag to recalculate trapezoids on entry junction
            bool nominal_length_flag:1;          // Planner flag for nominal speed always reached
            bool is_ready:1;
            bool primary_axis:1;                 // set if this move is a primary axis
            bool is_g123:1;                      // set if this is a G1, G2 or G3
            volatile bool is_ticking:1;          // set when this block is being actively ticked by the stepticker
            volatile bool locked:1;              // set to true when the critical data is being updated, stepticker will have to skip if this is set
            uint16_t s_value:12;                 // for laser 1.11 Fixed point
        };
};
