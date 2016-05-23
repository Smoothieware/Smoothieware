/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl) with additions from Sungeun K. Jeon (https://github.com/chamnit/grbl)
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "libs/nuts_bolts.h"
#include "libs/RingBuffer.h"
#include "../communication/utils/Gcode.h"
#include "libs/Module.h"
#include "libs/Kernel.h"
#include "Timer.h" // mbed.h lib
#include "wait_api.h" // mbed.h lib
#include "Block.h"
#include "Conveyor.h"
#include "Planner.h"
#include "mri.h"
#include "checksumm.h"
#include "Config.h"
#include "libs/StreamOutputPool.h"
#include "ConfigValue.h"
#include "StepTicker.h"

#include <functional>
#include <vector>

#include "mbed.h"

#define planner_queue_size_checksum CHECKSUM("planner_queue_size")

/*
The conveyor holds the queue of blocks, takes care of creating them, and moving them to the job  queue when ready

A Block is created by the planner in Planner::append_block() and stuck on the
head of the queue.

The Conveyor is always checking (in on_main) when blocks on the queue become
fully planned (ie recalculate flag gets cleared). When this happens the block
is pushed onto the job queue in stepticker and removed off the tail of the block queue.
If the job queue is full then it will get pushed when the job queue has room.

If the block queue has entries that are not yet fully planned, and a certain
time has elapsed it gives up waiting and forces the tail of the block queue
onto the job queue, the time needs to be configurable, but it needs to be long
enough to allow gcodes to be sent from a host, or read from sdcard, so that
even very short moves will get some planning, otherwise a stream of very short
moves will be very jerky as they will always decelerate to zero. However the
delay cannot be so long that there is a noticable lag for jog commands.

*/

Conveyor::Conveyor()
{
    //gc_pending = queue.tail_i;
    running = true;
    halted = false;
}

void Conveyor::on_module_loaded()
{
    //register_for_event(ON_IDLE);
    register_for_event(ON_MAIN_LOOP);
    register_for_event(ON_HALT);

    // Attach to the end_of_move stepper event
    //THEKERNEL->step_ticker->finished_fnc = std::bind( &Conveyor::all_moves_finished, this);
    queue.resize(THEKERNEL->config->value(planner_queue_size_checksum)->by_default(32)->as_number());
}

void Conveyor::on_halt(void* argument)
{
    if(argument == nullptr) {
        halted = true;
        flush_queue();
    } else {
        halted = false;
    }
}

// // Delete blocks here, because they can't be deleted in interrupt context ( see Block.cpp:release )
// // note that blocks get cleaned as they come off the tail, so head ALWAYS points to a cleaned block.
// void Conveyor::on_idle(void* argument)
// {
//     if (queue.tail_i != gc_pending) {
//         if (queue.is_empty()) {
//             __debugbreak();
//         } else {
//             // Cleanly delete block
//             Block* block = queue.tail_ref();
// //             block->debug();
//             block->clear();
//             queue.consume_tail();
//         }
//     }
// }

/*
 * In on_main_loop, we check whether the queue should be running, but isn't.
 * We also check if there are any blocks on the queue that are ready to be removed and moved to the job queue where they will be stepped
 * Doing it this way the block queue is handled entirely in on_main_loop so no race conditions are possible.
 * We can safely delete any blocks moved to the job queue here
 */

void Conveyor::on_main_loop(void*)
{
    if (running) {
        if(queue.is_empty() || THEKERNEL->step_ticker->is_jobq_full()) return;

        check_queue();
    }
}

// void Conveyor::append_gcode(Gcode* gcode)
// {
//     queue.head_ref()->append_gcode(gcode);
// }

// When all moves in a block have finished this is called by step ticker (in the pendsv ISR)
// void Conveyor::all_moves_finished()
// {
//     // TODO ideally we should pass in the pointer to the block that just completed, but stepticker doesn't really know what it is,
//     // but it has to be the tail of the block queue
//     // we mark the block as deleted here (release it), which will call on_block_end() below
//     this->queue.item_ref(gc_pending)->release();
// }

// // A new block is popped from the queue
// void Conveyor::on_block_begin(Block *block)
// {
//     // setup stepticker to execute this block
//     // if it returns false the job queue was full
//     THEKERNEL->step_ticker->add_job(block);
// }

// // Process a new block in the queue
// // gets called when the block is released
// void Conveyor::on_block_end(Block *block)
// {
//     if (queue.is_empty())
//         __debugbreak();

//     gc_pending = queue.next(gc_pending);

//     // mark entire queue for GC if flush flag is asserted
//     if (flush) {
//         while (gc_pending != queue.head_i) {
//             gc_pending = queue.next(gc_pending);
//         }
//     }

//     // Return if queue is empty
//     if (gc_pending == queue.head_i) {
//         running = false;
//         return;
//     }

// //     // Get a new block
// //     Block* next = this->queue.item_ref(gc_pending);
// //     next->begin(); // causes on_block_begin() (above) to be called

// }

// Wait for the queue to be empty and for all the jobs to finish in step ticker
void Conveyor::wait_for_empty_queue()
{
    // wait for the job queue to empty, this means cycling everything on the block queue into the job queue
    // forcing them to be ready
    while (!queue.is_empty()) {
        check_queue(true); // forces all blocks to be moved to the step ticker job queue
        THEKERNEL->call_event(ON_IDLE, this);
    }

    // now we wait for stepticker to finish all the jobs
    while(!THEKERNEL->step_ticker->is_jobq_empty()){
        THEKERNEL->call_event(ON_IDLE, this);
    }

    // returning now means that everything has totally finished
}

/*
 * push the pre-prepared head block onto the queue
 */
void Conveyor::queue_head_block()
{
    if(halted) {
        // we do not want to stick more stuff on the queue if we are in halt state
        // clear and release the block on the head
        queue.head_ref()->clear();
        return;
    }

    // upstream caller will block on this until there is room in the queue
    while (queue.is_full()) {
        check_queue();
        THEKERNEL->call_event(ON_IDLE, this);
    }

    queue.head_ref()->ready();
    queue.produce_head();
}

// if the queue is not empty see if we can stick something on the stepticker job queue.
// we have to walk back and find blocks where recalculate_flag is clear.
// otherwise if the jobq is empty... (don't force anyhting if we have at least one thing on the job queue)
// see if we have reached the time limit, otherwise give it some more time to finish planning some entries
// if we have reached the time limit force the next thing on the queue into the jobq, fully planned or not
// as this is only called in on)main_loop() it is safe to delete blocks and stuff
void Conveyor::check_queue(bool force)
{
    static uint32_t last_time_check = us_ticker_read();

    if(queue.is_empty()) return;

    // if we have been checking for more than the required waiting time, we force
    if((us_ticker_read() - last_time_check) >= (queue_delay_time_ms*1000))
        force= true;

    Block* block = queue.tail_ref();
    if(force || !block->recalculate_flag) {
        last_time_check = us_ticker_read(); // reset timer
        // setup stepticker to execute this block
        // if it returns false the job queue was full
        if(!THEKERNEL->step_ticker->add_job(block)) return;
        THEKERNEL->streams->printf("%lu > ", last_time_check);
        block->debug();

        // remove from tail
        // TODO need to set the exit speed so it can be used by the planner

        block->release();
        block->clear();
        queue.consume_tail();
    }
}

/*
    In most cases this will not totally flush the queue, as when streaming
    gcode there is one stalled waiting for space in the queue, in
    queue_head_block() so after this flush, once main_loop runs again one more
    gcode gets stuck in the queue, this is bad. Current work around is to call
    this when the queue in not full and streaming has stopped
*/
void Conveyor::flush_queue()
{
    while (!queue.is_empty()) {
        Block* block = queue.tail_ref();
        block->clear();
        queue.consume_tail();
    }

    // TODO force deceleration of last block

    // now wait until the job queue has finished too
    wait_for_empty_queue();
}

// Debug function
void Conveyor::dump_queue()
{
    for (unsigned int index = queue.tail_i, i = 0; true; index = queue.next(index), i++ ) {
        THEKERNEL->streams->printf("block %03d > ", i);
        queue.item_ref(index)->debug();

        if (index == queue.head_i)
            break;
    }
}

// feels hacky, but apparently the way to do it
#include "HeapRing.cpp"
template class HeapRing<Block>;
