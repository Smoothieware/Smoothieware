/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include <stdint.h>

// THE switch for the whole filament advance feature. Every piece of it (this class, the extra Block and
// StepTicker data, the Planner and StepTicker code, StepperMotor::lead_time, and the config key, M900 and M500
// code in Extruder.cpp) is wrapped in
// "#ifdef FILAMENT_ADVANCE". CNC builds (make CNC=1) have no extruders, so they get none of it: no code,
// no extra RAM per block, no extra work in the step ticker interrupt.
#ifndef CNC
#define FILAMENT_ADVANCE
#endif

#ifdef FILAMENT_ADVANCE

/*
 * FILAMENT ADVANCE (a.k.a. pressure advance / linear advance)
 * ===========================================================
 *
 * The problem: molten plastic is slightly compressible and the nozzle is a restriction, so the pressure
 * in the nozzle (and therefore the flow out of it) lags behind the extruder motor. When the printhead speeds
 * up, the line starts thin; when it slows down, the line ends fat (blobs in corners, bulging ends).
 *
 * The model: to a good approximation, nozzle pressure is proportional to the flow rate. So we push a little
 * harder while the flow is increasing and a little less while it is decreasing. We command the extruder
 * motor to be "ahead" of the nominal filament position by a fixed amount of TIME, K (seconds):
 *
 *      E_commanded(t) = E_nominal(t) + K * dE_nominal/dt
 *
 * For one planner block that has a trapezoidal speed profile (accelerate, cruise, decelerate), the extruder
 * step rate is therefore the nominal rate plus a constant OFFSET in each phase:
 *
 *      accelerating:  + K * a      (a = the extruder's share of the block acceleration, steps/s²)
 *      cruising:        0
 *      decelerating:  - K * d      (d = the same for the deceleration)
 *
 * Extra steps pushed during acceleration are taken back during deceleration, so over a chain of blocks that
 * begins and ends at rest the extruder ends up exactly where it would have been without this feature.
 *
 * If the nominal rate during a deceleration drops below K*d, the commanded rate crosses zero and the extruder
 * must run BACKWARDS to relieve the pressure (this is the same thing as a tiny retract). With a short ramp (K is
 * often longer than the time the head needs to slow down) that happens from the very start of the deceleration,
 * and a short block can end with the extruder slightly behind where it began (a negative net step count).
 *
 * This file only does the arithmetic for ONE extruder in ONE block. It knows nothing about the kernel, the
 * step ticker or the hardware, so it is easy to read and to unit test on a PC. Block::prepare() calls it and
 * converts the result to the fixed point numbers that the step ticker interrupt consumes.
 *
 * Rules the rest of the firmware follows (see Planner.cpp and Block.cpp):
 *   - Only blocks that move at least one of X/Y/Z and extrude forwards use it. Retracts, primes and pure
 *     extruder moves do not (the nozzle is not carrying a flow that needs correcting).
 *   - A junction between a block that uses it and one that does not is taken at the planner's minimum
 *     speed, so the extra "lead" steps are paid back before the printhead moves on (exactly, when minimum_planner_speed
 *     is 0, the default; otherwise the few steps of lead at that speed are lost at the junction).
 *   - Everything is measured in whole steps ("lead", below) at block boundaries, so rounding errors can never
 *     accumulate: every block starts with exactly the lead that the previous block ended with.
 */
class FilamentAdvance
{
    public:
        // Everything the calculation needs to know about one extruder motor in one block.
        // Rates are in steps/second of that motor, accelerations in steps/second², times in ticks.
        struct Input {
            float    lead_time;          // K in seconds, from M900 or the config file
            float    min_reversal_steps; // reversals shallower than this many steps are skipped; 0 = never skip (see below)
            float    max_rate;           // the motor's configured maximum, steps/second (limits K*a)

            float    entry_rate;         // nominal rate at the start of the block
            float    plateau_rate;       // nominal rate at the end of the acceleration (the cruise rate)
            float    exit_rate;          // nominal rate at the end of the block
            float    acceleration;       // acceleration ramp slope, >= 0, 0 if there is no acceleration phase
            float    deceleration;       // deceleration ramp slope, >= 0, 0 if there is no deceleration phase

            uint32_t accelerate_until;   // same meaning as in Block: last tick of the acceleration ramp (0 = none)
            uint32_t decelerate_after;   // tick where the deceleration ramp begins (== total_move_ticks = none)
            uint32_t total_move_ticks;   // length of the block
            float    tick_frequency;     // step ticker frequency in Hz, to convert seconds to ticks

            uint32_t steps;              // number of steps this motor makes in the block without advance, > 0
            int32_t  lead_in;            // lead (see Output::lead_out) the previous block left behind, 0 if none
        };

        // The shaped motion, in the same terms as Input.
        struct Output {
            float    initial_rate;       // rate to start the block with (steps/second)
            float    plateau_rate;       // rate to use for the cruise phase (steps/second)
            float    rate_scale;         // multiply the acceleration and deceleration slopes by this (see below)
            float    decel_start_adjust; // add this to the rate at the instant the deceleration begins (steps/s)
            uint32_t flip_tick;          // tick at which the motor must reverse direction, NO_FLIP = never
            int32_t  net_steps;          // net number of steps (forward minus backward) the motor must make, never 0; negative = the block ends behind where it started
            int32_t  lead_out;           // extra steps this block leaves "in the nozzle" at its end
        };

        // Value of Output::flip_tick (and of the tick info) for a block in which the motor never reverses. It is not 0
        // because a block that starts decelerating can have to reverse at its very first tick.
        static const uint32_t NO_FLIP = 0xFFFFFFFFu;

        // Default for Input::min_reversal_steps (config key pressure_advance_min_reversal, M900 R). A reversal that
        // would take fewer backward steps than this is not worth the direction change: the pulse timing around it
        // would be worse than the error it removes. 0 switches the skipping off, so every reversal is made.
        static constexpr float DEFAULT_MIN_REVERSAL_STEPS = 2.5F;

        // Computes the shaped motion for one extruder in one block.
        static Output plan(const Input &in);
};

#endif // FILAMENT_ADVANCE
