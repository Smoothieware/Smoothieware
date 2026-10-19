/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "FilamentAdvance.h"

#ifdef FILAMENT_ADVANCE

#include <math.h>
#include <algorithm>

// See FilamentAdvance.h for the model. Here is how one block is turned into numbers.
//
// The step ticker already knows how to run a trapezoid: it starts at some rate, adds a fixed amount per tick
// while accelerating, jumps to the plateau rate, then subtracts a fixed amount per tick while decelerating.
// Filament advance only needs to change a few of those numbers:
//
//   rate
//    ^                                                   nominal                 shaped
//    |           plateau_rate  ........______           ........               ........
//    |                       /                \         /        \             /        \          (the shaped
//    |                     /                    \     /            \         /            \         curve is the
//    |  initial_rate  ___/                        \_ /                \_____/                \__    nominal one,
//    |                                                                                              moved up by K*a
//    +----------------------------------------------------> time                                    while speeding
//          accelerate_until      decelerate_after                                                    up and down by
//                                                                                                    K*d while slowing)
//
//   1. initial_rate      = entry rate + K*a            (the offset of the phase we start in)
//   2. plateau_rate      = cruise rate                 (no offset while cruising; the ticker already jumps to it)
//   3. decel_start_adjust= -K*d (- K*a if we go straight from accelerating to decelerating, because the +K*a
//                           offset is then still in the rate and must be removed too)
//   4. if the rate would cross zero while decelerating, flip_tick says when to reverse the motor
//
// The extra distance the offsets add up to is K*(exit_rate - entry_rate). We do not trust the floating point
// version of that number, we compute an integer instead (see "lead" below) and then stretch the whole
// profile by a tiny factor, rate_scale, so that the ticker produces exactly the integer number of steps.
FilamentAdvance::Output FilamentAdvance::plan(const Input &in)
{
    const bool has_accel         = in.accelerate_until > 0;
    const bool has_decel         = in.decelerate_after < in.total_move_ticks && in.deceleration > 0.0F;
    const bool starts_decelerating = in.decelerate_after == 0;
    const bool goes_straight_from_accel_to_decel = has_accel && in.decelerate_after == in.accelerate_until;

    // Nominal rate at the instant the deceleration starts
    const float rate_at_decel_start = starts_decelerating ? in.entry_rate : in.plateau_rate;

    // ---- Step 1: limit K for this block ------------------------------------------------------------------
    // The only limit is the motor itself: the offset K*a is a rate the motor has to reach, so it must not exceed the
    // motor's configured maximum. (K itself is NOT limited by the length of the ramps: when K is longer than the
    // time the printhead takes to slow down, the extruder simply starts running backwards as soon as the
    // deceleration begins, or even in the first tick of a block that starts decelerating. That is what the model
    // asks for, and step 4 and StepTicker handle it.)
    float k = in.lead_time;
    const float steepest_ramp = std::max(in.acceleration, in.deceleration);
    if (steepest_ramp > 0.0F && in.max_rate > 0.0F) k = std::min(k, in.max_rate / steepest_ramp);
    if (k < 0.0F) k = 0.0F;

    // ---- Step 2: the offsets of each phase ---------------------------------------------------------------
    const float accel_offset = has_accel ? k * in.acceleration : 0.0F;
    const float decel_offset = has_decel ? k * in.deceleration : 0.0F;

    // The offset of the phase the block starts in
    float start_offset = 0.0F;
    if (has_accel) start_offset = accel_offset;
    else if (starts_decelerating && has_decel) start_offset = -decel_offset;

    // What has to be added when the deceleration begins to go from the offset in effect to -K*d. The ticker has
    // just replaced the rate by the plain plateau rate if there was a cruise phase (offset 0). It has not if the
    // acceleration runs straight into the deceleration, then the +K*a offset is still in the rate.
    float decel_start_adjust = 0.0F;
    if (has_decel && !starts_decelerating) {
        decel_start_adjust = -decel_offset - (goes_straight_from_accel_to_decel ? accel_offset : 0.0F);
    }

    // ---- Step 3: lead, the exact number of extra steps -------------------------------------------------
    // "lead" is how many steps ahead of the nominal position the extruder is at the end of a block: K times
    // the exit rate, rounded to a whole step. A block must make (lead_out - lead_in) more steps than the
    // nominal motion, because it begins where the previous block ended. Because lead_in is always exactly the
    // previous block's lead_out, the extras of a whole print add up to exactly zero when it comes to rest.
    int32_t lead_out = (int32_t)lroundf(k * in.exit_rate);
    int32_t net_steps = (int32_t)in.steps + lead_out - in.lead_in;

    // A short block that decelerates from a high rate can have to repay more lead than it has nominal steps: the
    // net is then zero or NEGATIVE, the extruder ends the block behind where it started (a small retraction).
    // That is exactly what the model asks for. Only a block without deceleration cannot reverse, so it must keep
    // at least one forward step; any lead it cannot repay is handed on to the next block as lead_out.
    if (!has_decel && net_steps < 1) {
        lead_out += 1 - net_steps;
        net_steps = 1;
    }

    // The offsets above add up to k*(exit-entry) extra steps. We need net_steps - steps. Stretch the whole
    // profile (every rate and ramp slope) by the ratio so that they agree; it is within a few percent. The
    // ticker stops on the step COUNT, so the scale only has to be roughly right: it is ignored (1.0) when the
    // profile is so small or of a different sign that the ratio would be meaningless.
    const float profile_net_steps = (float)in.steps + k * (in.exit_rate - in.entry_rate);
    float rate_scale = 1.0F;
    if (profile_net_steps * (float)net_steps > 0.0F) {
        rate_scale = std::min(2.0F, std::max(0.5F, (float)net_steps / profile_net_steps));
    }

    // ---- Step 4: reversing during a deceleration ---------------------------------------------------------
    // During the deceleration the shaped rate is  R0 - K*d - d*t  (R0 = nominal rate when the ramp starts, t = time
    // since then). It reaches zero at t0 = (R0 - K*d) / d, and the motor runs backwards from there to the end
    // of the ramp. If R0 < K*d then t0 is negative: the extruder has to reverse the moment the ramp begins
    // (flip at tick 0 of the ramp) and its backward speed first has to grow from K*d - R0.
    uint32_t flip_tick = NO_FLIP;
    if (has_decel) {
        // How many steps the motor would make backwards: the area under the backward-speed curve.
        const float back_speed_at_start = std::max(0.0F, decel_offset - rate_at_decel_start);
        const float back_speed_at_end   = decel_offset - in.exit_rate;
        const float backward_steps = (back_speed_at_end > 0.0F)
            ? (back_speed_at_end * back_speed_at_end - back_speed_at_start * back_speed_at_start) / (2.0F * in.deceleration)
            : 0.0F;

        // A negative net means the block must reverse whatever the rounding says. A tiny reversal is not worth
        // a direction change (the pulse timing would be worse than the error): the block just ends at rate ~0
        // and the step COUNT, which is exact, takes care of the position.
        const bool must_reverse = net_steps < 0;
        if (must_reverse || backward_steps > 0.0F && backward_steps >= in.min_reversal_steps) {
            const float seconds_until_zero = (rate_at_decel_start - decel_offset) / in.deceleration;
            const uint32_t ticks_until_zero = (seconds_until_zero > 0.0F) ? (uint32_t)lroundf(seconds_until_zero * in.tick_frequency) : 0;
            const uint32_t tick = in.decelerate_after + ticks_until_zero;
            if (tick < in.total_move_ticks) flip_tick = tick;
            else if (must_reverse) flip_tick = in.decelerate_after; // rounding pushed it past the end
        }
    }

    Output out;
    out.initial_rate       = (in.entry_rate + start_offset) * rate_scale;
    out.plateau_rate       = in.plateau_rate * rate_scale;
    out.rate_scale         = rate_scale;
    out.decel_start_adjust = decel_start_adjust * rate_scale;
    out.flip_tick          = flip_tick;
    // The NET number of steps (forward minus backward). If the motor reverses it makes F forward steps and then
    // F - net backward ones. The ticker does not guess F: at the flip it knows how many forward steps it really
    // made (see StepTicker::step_tick), so rounding cannot add up.
    out.net_steps          = net_steps;
    out.lead_out           = lead_out;
    return out;
}

#endif // FILAMENT_ADVANCE
