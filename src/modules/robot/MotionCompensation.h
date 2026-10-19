/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include "BacklashCompensator.h"

#include <stddef.h>

class Block;

/*
    The planner's single point of contact with the compensations that add or reshape steps of a motor
    (backlash today, filament advance once it is migrated behind the same hooks, see
    docs/research/2026-10-18-motion-compensation-and-compact-queue-implementation-plan.md).
    The planner calls these hooks per block and per motor; it does not know which compensation answers.
*/
class MotionCompensation
{
public:
    // Hook 1 (pure read, called by Robot inside its rate-limit loop and by the planner): extra steps this motor needs
    uint32_t extra_steps(uint8_t motor, float steps_per_mm, int32_t nominal_steps) const { return backlash.slack_steps(motor, steps_per_mm, nominal_steps); }

    // Hook 2 (called by the planner when it appends the block): commits the chain state, returns the extra steps
    uint32_t commit(uint8_t motor, float steps_per_mm, int32_t nominal_steps) { return backlash.commit(motor, steps_per_mm, nominal_steps); }

    // Hook 5: halt or queue flush
    void on_flush(StepperMotor *const *motors, uint8_t n_motors) { backlash.on_flush(motors, n_motors); }

    // Hook 3 (planner, once the block's steps are known): marks the motors whose step rate this block reshapes
    // (filament advance); does nothing in builds without a compensation that reshapes.
    void begin_block(Block *block, size_t n_motors) const;

    // Hook 4 (planner, junction with the block before it): true when the junction must be taken at minimum speed
    // because the reshaped motors differ between the two blocks.
    bool needs_stop_junction(const Block *previous, const Block *block) const;

    // Hook 4b (planner, trapezoid recalculation): calculates the block's trapezoid; "predecessor" is the block that
    // runs before it, or nullptr, which the reshaping compensation needs to know where the previous block left off.
    void calculate_trapezoid(Block *block, float entry_speed, float exit_speed, const Block *predecessor) const;

    // Hook 4c (called from Block::prepare for every motor in the block's shaped_motors): overwrites the motor's tick_info
    // with the reshaped rates and records the carry-over for the next block in block->shaper_state[motor].
    // "acceleration_change" and "deceleration_per_tick" are the block's plain per-tick values (2.62 fixed point, before
    // scaling by the motor's share "aratio").
    void shape_motor(Block *block, uint8_t motor, const Block *previous, float aratio, float acceleration_in_steps,
                     float deceleration_in_steps, double acceleration_change, double deceleration_per_tick) const;

    // Hook 4d (called from Block::prepare for a motor that carries backlash slack steps): front-loads them. The motor's
    // planned profile is scaled to its nominal steps and the slack steps ride on top of it as an extra rate that lasts
    // until they are all issued, so the load starts moving at once instead of after the slack share of the block.
    void takeup_motor(Block *block, uint8_t motor) const;

    BacklashCompensator backlash;
};
