/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "MotionCompensation.h"
#include "Block.h"
#include "Robot.h"
#include "StepperMotor.h"
#include "Kernel.h"
#include "StepTicker.h"
#include <math.h>
#include <algorithm>

// The filament advance parts of the hooks live here, behind one compile switch, so the planner, the block queue and
// the step interrupt hold none of it: they only know the generic fields (Block::shaped_motors, shaper_state,
// tickext_t::decel_start_rate_delta and reversal_tick).

void MotionCompensation::begin_block(Block *block, size_t n_motors) const
{
#ifdef FILAMENT_ADVANCE
    // Find the extruders whose step rate will be shaped in this block (see FilamentAdvance.h).
    // Only moves that also move X, Y, Z (or another primary axis) qualify, and only when extruding forwards, so
    // retracts, primes and pure extruder moves keep their plain trapezoid.
    block->shaped_motors.reset();
    for (size_t i = 0; i < Block::n_actuators; i++) block->shaper_state[i] = 0;
    if(block->primary_axis) {
        for (size_t i = 0; i < n_motors; i++) {
            StepperMotor *motor = THEROBOT->actuators[i];
            if(motor->is_extruder() && motor->get_lead_time() > 0.0F && block->steps[i] != 0 && !block->direction_bits[i]) {
                block->shaped_motors.set(i);
            }
        }
    }
#else
    (void)block;
    (void)n_motors;
#endif
}

bool MotionCompensation::needs_stop_junction(const Block *previous, const Block *block) const
{
    // Backlash: a block that carries slack steps runs its motor at a different step rate than a neighbour that does
    // not, so the junctions on both sides of it are taken at the minimum speed, as if the machine had stopped.
    for (size_t i = 0; i < Block::n_actuators; i++) {
        if (previous->correction_steps(i) != 0 || block->correction_steps(i) != 0) return true;
    }
#ifdef FILAMENT_ADVANCE
    // The extruder is "ahead" of the nominal position while a shaped block runs, and it only pays that back when the
    // block slows down. So a junction between a block that is shaped and one that is not (the start or end of a
    // printed line) is taken at the minimum speed, as if the machine had stopped. The same is done when the two
    // blocks shape different extruders, because the lead is tracked per extruder and the one that is no longer
    // shaped would carry its lead into the next block.
    return previous->shaped_motors != block->shaped_motors;
#else
    (void)previous;
    (void)block;
    return false;
#endif
}

void MotionCompensation::calculate_trapezoid(Block *block, float entry_speed, float exit_speed, const Block *predecessor) const
{
    block->calculate_trapezoid(entry_speed, exit_speed, predecessor);
}

void MotionCompensation::shape_motor(Block *block, uint8_t m, const Block *previous, float aratio, float acceleration_in_steps,
                                     float deceleration_in_steps, double acceleration_change, double deceleration_per_tick) const
{
#ifdef FILAMENT_ADVANCE
    const float tick_frequency = THEKERNEL->step_ticker->get_frequency(); // float on purpose: the divisions below must round exactly as before
    const uint32_t steps = block->steps[m];
    StepperMotor *extruder = THEROBOT->actuators[m];
    // FilamentAdvance works in steps/sec for this one motor, so the block's rates are scaled by the motor's share (aratio)
    FilamentAdvance::Input in;
    in.lead_time        = extruder->get_lead_time();
    in.min_reversal_steps = extruder->get_min_reversal();
    in.max_rate         = extruder->get_max_rate() * extruder->get_steps_per_mm();
    in.entry_rate       = (double)block->initial_rate * aratio;
    in.plateau_rate     = (double)block->maximum_rate * aratio;
    in.exit_rate        = (block->nominal_speed > 0.0F) ? (double)block->nominal_rate * (block->exit_speed / block->nominal_speed) * aratio : 0.0;
    in.acceleration     = acceleration_in_steps * aratio;
    in.deceleration     = deceleration_in_steps * aratio;
    // A tiny block can get a slightly negative acceleration time, which calculate_trapezoid floors to -1 tick and
    // stores as 0xFFFFFFFF in accelerate_until: that means "no acceleration phase", the same as 0. Both the plan and
    // the reversal event below must see it as 0.
    const bool has_no_accel_phase = block->accelerate_until == 0 || block->accelerate_until > block->total_move_ticks;
    in.accelerate_until = has_no_accel_phase ? 0 : block->accelerate_until;
    in.decelerate_after = block->decelerate_after;
    in.total_move_ticks = block->total_move_ticks;
    in.tick_frequency   = tick_frequency;
    in.steps            = steps;
    // the lead the previous block left in the nozzle, 0 if there is none
    in.lead_in          = previous != nullptr ? previous->shaper_state[m] : 0;

    FilamentAdvance::Output out = FilamentAdvance::plan(in);
    Block::tickinfo_t &ti = *block->axis(m);
    // The NET number of steps, possibly negative (stored as two's complement; the ticker sorts out the reversal)
    ti.steps_to_move = (uint32_t)out.net_steps;
    ti.steps_per_tick = (int64_t)round((out.initial_rate / tick_frequency) * STEPTICKER_FPSCALE);
    ti.plateau_rate = (int64_t)round((out.plateau_rate / tick_frequency) * STEPTICKER_FPSCALE);
    // the ramps are stretched by rate_scale so that the ticker produces exactly steps_to_move steps
    ti.acceleration_change = (int64_t)round(acceleration_change * aratio * out.rate_scale);
    ti.deceleration_change = -(int64_t)round(deceleration_per_tick * aratio * out.rate_scale);
    Block::tickext_t &ex = *block->ext(m); // every shaped motor has one
    ex.decel_start_rate_delta = (int64_t)round((out.decel_start_adjust / tick_frequency) * STEPTICKER_FPSCALE);
    ex.reversal_tick = out.flip_tick;
    // A block that starts decelerating has no "start of deceleration" event, so the reversal is its only event
    if(out.flip_tick != Block::NO_EVENT && has_no_accel_phase && block->decelerate_after == 0) {
        ti.next_accel_event = out.flip_tick;
    }
    block->shaper_state[m] = out.lead_out;
#else
    (void)block; (void)m; (void)previous; (void)aratio; (void)acceleration_in_steps; (void)deceleration_in_steps;
    (void)acceleration_change; (void)deceleration_per_tick;
#endif
}

void MotionCompensation::takeup_motor(Block *block, uint8_t m) const
{
    const float configured = backlash.get_takeup_rate();
    if (configured == 0.0F) return; // take-up phase switched off: the slack steps stay spread over the whole block

    Block::tickinfo_t &ti = *block->axis(m);
    Block::tickext_t &ex = *block->ext(m); // a motor with slack to cross always has one
    const uint32_t slack = (uint32_t)(ex.position_correction_steps < 0 ? -ex.position_correction_steps : ex.position_correction_steps);
    const uint32_t all_steps = block->steps[m];
    if (slack == 0 || slack >= all_steps || block->total_move_ticks == 0 || block->steps_event_count == 0) return;
    const uint32_t nominal = all_steps - slack;

    // the planned profile only has to deliver the nominal steps
    const double scale = (double)nominal / (double)all_steps;
    ti.steps_per_tick = (int64_t)round(ti.steps_per_tick * scale);
    ti.acceleration_change = (int64_t)round(ti.acceleration_change * scale);
    ti.deceleration_change = (int64_t)round(ti.deceleration_change * scale);
    ti.plateau_rate = (int64_t)round(ti.plateau_rate * scale);

    // extra rate in steps per tick: as fast as asked, but no faster than the motor's maximum on top of the cruise rate,
    // and never slower than what spreads the slack over the whole block (which is what the rate limits were checked for)
    const double frequency = THEKERNEL->step_ticker->get_frequency();
    StepperMotor *motor = THEROBOT->actuators[m];
    const double max_rate = (double)motor->get_max_rate() * motor->get_steps_per_mm(); // steps/s
    double wanted = (configured < 0.0F ? motor->get_max_rate() : configured) * (double)motor->get_steps_per_mm(); // steps/s
    const double cruise = (double)block->maximum_rate * ((double)nominal / (double)block->steps_event_count); // steps/s
    wanted = std::min(wanted, max_rate - cruise);
    double rate = wanted / frequency; // steps per tick
    const double slowest = (double)slack / (double)block->total_move_ticks;
    if (rate < slowest) rate = slowest;

    ex.takeup_rate = (int64_t)round(rate * STEPTICKER_FPSCALE);
    if (ex.takeup_rate <= 0) { ex.takeup_rate = 0; return; }
    uint32_t end = (uint32_t)round((double)slack / rate);
    if (end > block->total_move_ticks) end = block->total_move_ticks;
    ex.takeup_end_tick = end == 0 ? 1 : end;
}
