/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include <stdint.h>
#include "ActuatorCoordinates.h"

class StepperMotor;

/*
    Backlash compensation: when a motor reverses direction its first steps only take up the slack in the drive
    train, so the tool does not move. The compensator adds those take-up steps ("slack steps") to the first block
    after a reversal.

    The slack steps are part of the block's step count, so they are inside the block's time, and Robot adds them
    to the motor distance of its limit loop (known gaps: see the PR text, "Not verified"). They are NOT part of the
    position: the planner's nominal milestones, distance and junction speeds ignore them, and the step counter is
    corrected at the start of the block (Block::tick_info.position_correction_steps). Until the slack is crossed the
    counter is therefore ahead of the ideal load position by up to the slack (a halt in that window rebuilds the
    position from it).

    The chain state is the direction of the last move that was appended to the queue for each motor.
    All functions except on_flush() run from the main loop (planner); on_flush() is called from
    Conveyor::get_next_block(), i.e. from the step interrupt context, and only touches the direction chain.
*/
class BacklashCompensator
{
public:
    BacklashCompensator();

    // Slack of a motor in mm, 0 = off. The step count is derived from the motor's steps per mm when it is used.
    void set_backlash_mm(uint8_t motor, float mm);
    float get_backlash_mm(uint8_t motor) const { return motor < MAX_ROBOT_ACTUATORS ? backlash_mm[motor] : 0.0F; }
    // M425 P0/P1: master switch that keeps the configured values; while off no slack steps are added
    void set_enabled(bool on) { enabled = on; }
    bool is_enabled() const;
    // Speed of the take-up in mm/s of motor travel (config backlash_takeup_rate): the first steps of a reversal block are
    // issued at this extra rate until the slack is gone, then the motor rejoins the planned profile. < 0 (the default)
    // = the motor's maximum rate, 0 = no take-up phase (the slack steps are spread over the whole block).
    void set_takeup_rate(float mm_s) { takeup_mm_s = (mm_s == mm_s && mm_s >= 0.0F) ? mm_s : -1.0F; }
    float get_takeup_rate() const { return takeup_mm_s; }
    // G92/G92.4/M92: the position was redefined by hand, so the next move of every motor gets no slack steps
    void forget_directions();

    // Hook 1, a pure read: number of slack steps the next move of this motor needs, given its signed nominal step count.
    uint32_t slack_steps(uint8_t motor, float steps_per_mm, int32_t nominal_steps) const;

    // Hook 2, commits the chain state when the block is appended. Returns the same value slack_steps() returned.
    uint32_t commit(uint8_t motor, float steps_per_mm, int32_t nominal_steps);

    // Halt or queue flush: the queued moves are gone, so the direction chain restarts from the direction the motors
    // really have (a motor whose direction is still unknown stays unknown, and its next move gets no slack steps).
    void on_flush(StepperMotor *const *motors, uint8_t n_motors);

private:
    bool enabled{true};
    float takeup_mm_s{-1.0F};
    float backlash_mm[MAX_ROBOT_ACTUATORS];
    int8_t last_direction[MAX_ROBOT_ACTUATORS]; // +1, -1, or 0 = not known yet
};
