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

#define planner_queue_size_checksum CHECKSUM("planner_queue_size")

/*
 * The conveyor holds the queue of blocks, takes care of creating them, and starting the executing chain of blocks
 *
 * The Queue is implemented as a ringbuffer- with a twist
 *
 * Since delete() is not thread-safe, we must marshall deletable items out of ISR context
 *
 * To do this, we have implmented a *double* ringbuffer- two ringbuffers sharing the same ring, and one index pointer
 *
 * as in regular ringbuffers, HEAD always points to a clean, free block. We are free to prepare it as we see fit, at our leisure.
 * When the block is fully prepared, we increment the head pointer, and from that point we must not touch it anymore.
 *
 * also, as in regular ringbuffers, we can 'use' the TAIL block, and increment tail pointer when we're finished with it
 *
 * Both of these are implemented here- see queue_head_block() (where head is pushed) and on_idle() (where tail is consumed)
 *
 * The double ring is implemented by adding a third index pointer that lives in between head and tail. We call it gc_pending which describes its function rather than its operation
 *
 * in ISR context, we use HEAD as the head pointer, and gc_pending as the tail pointer.
 * As HEAD increments, ISR context can consume the new blocks which appear, and when we're finished with a block, we increment gc_pending to signal that they're finishd, and ready to be cleaned
 *
 * in IDLE context, we use gc_pending as the head pointer, and TAIL as the tail pointer.
 * When gc_pending != tail, we clean up the tail block (performing ISR-unsafe delete operations) and consume it (increment tail pointer), returning it to the pool of clean, unused blocks which HEAD is allowed to prepare for queueing
 *
 * Thus, our two ringbuffers exist sharing the one ring of blocks, and we safely marshall used blocks from ISR context to IDLE context for safe cleanup.
 */


/*

A Block is created by the planner in Planner::append_block() and stuck on the
head of the queue.

The Conveyor is always checking (in on_main) when blocks on the queue become
fully planned (ie recalculate flag gets cleared). When this happens a pointer
to the block is pushed onto the job queue in stepticker, but also left in the
block queue. If the job queue is full then it will get pushed when the job
queue has room. (TODO should actually copy block not pointer so that is not done in ISR).

Once the job has finished being executed the block is
removed from the block queue (at least marked to be removed as explained
above).

If the block queue has entries that are not yet fully planned, and a certain
time has elapsed it gives up waiting and forces the tail of the block queue
onto the job queue, the time needs to be configurable, but it needs to be long
enough to allow gcodes to be sent from a host, or read from sdcard, so that
even very short moves will get some planning, otherwise a stream of very short
moves will be very jerky as they will always decelerate to zero. However the
delay cannot be so long that there is a noticable lag for jog commands.

TODO an optimization is to remove the block from the block queue and put it
into the job queue, as this happens in on_idle it should be safe to do without
the double  pointer stuff. as the job queue has nothing that needs deleting
this works out.

*/

Conveyor::Conveyor()
{
    gc_pending = queue.tail_i;
    running = false;
    flush = false;
    halted = false;
}

void Conveyor::on_module_loaded()
{
    register_for_event(ON_IDLE);
    register_for_event(ON_MAIN_LOOP);
    register_for_event(ON_HALT);

    // Attach to the end_of_move stepper event
    THEKERNEL->step_ticker->finished_fnc = std::bind( &Conveyor::all_moves_finished, this);
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

// Delete blocks here, because they can't be deleted in interrupt context ( see Block.cpp:release )
// note that blocks get cleaned as they come off the tail, so head ALWAYS points to a cleaned block.
void Conveyor::on_idle(void* argument)
{
    if (queue.tail_i != gc_pending) {
        if (queue.is_empty()) {
            __debugbreak();
        } else {
            // Cleanly delete block
            Block* block = queue.tail_ref();
//             block->debug();
            block->clear();
            queue.consume_tail();
        }
    }
}

/*
 * In on_main_loop, we check whether the queue should be running, but isn't.
 *
 * The main trigger for this event is other pieces of code adding gcode to a block, but not pushing it. This occurs frequently with gcodes that must be executed at the correct point in the queue, but take zero time to execute.
 * Smoothie will happily attach many of such gcodes onto a single block, to save room in the queue.
 *
 * Any gcode which can potentially take time to execute, or might like to halt the queue MUST push the head block, otherwise gcodes that arrive later may get executed at the same time, and gcode execution order strictness would be violated.
 *
 * If we get back to main loop context and the block has gcode but isn't pushed, then we can safely push it and start the queue.
 *
 *
 * It's also theoretically possible that a race condition could occur where we pop the final block and stop the queue, while at the same time main loop is pushing head but thinks the queue is running and thus does not start it.
 *
 * In this case, we start the queue again when execution returns to main loop.
 * No stuttering or other visible effects could be caused by this event, as the planner will have set the last block to decelerate to zero, and the new block to accelerate from zero.
 *
 */

void Conveyor::on_main_loop(void*)
{
    if (running) {
        if(THEKERNEL->step_ticker->is_jobq_full()) return;

        // see if there are any fully planned things on the queue we can give to step ticker
        // TODO....

        return;
    }

    // not currently running, see if we can find something to do
    if (queue.is_empty()) {
        // nothing to do

        // if (queue.head_ref()->gcodes.size())
        // {
        //     queue_head_block();
        //     ensure_running();
        // }
    } else {
        // queue not empty so see if we can stick something on the stepticker job queue
        // we have to walk back and find blocks where recalculate_flag is clear.
        // otherwise if the jobq is empty... (don't force anyhting if we have at least one thing on the job queue)
        // see if we have reached the time limit, otherwise give it some more time to finish planning some entries
        // if we have reached the time limit force the next thing on the queue into the jobq, fully planned or not
        ensure_running();
    }
}

// void Conveyor::append_gcode(Gcode* gcode)
// {
//     queue.head_ref()->append_gcode(gcode);
// }

// When all moves in a block have finished this is called by step ticker (in the pendsv ISR)
void Conveyor::all_moves_finished()
{
    // TODO ideally we should pass in the pointer to the block that just completed, but stepticker doesn't really know what it is,
    // but it has to be the tail of the block queue
    // we mark the block as deleted here (release it), which will call on_block_end() below
    this->queue.item_ref(gc_pending)->release();
}

// A new block is popped from the queue
void Conveyor::on_block_begin(Block *block)
{
    // setup stepticker to execute this block
    // if it returns false the job queue was full
    THEKERNEL->step_ticker->add_job(block);
}

// Process a new block in the queue
// gets called when the block is released
void Conveyor::on_block_end(Block *block)
{
    if (queue.is_empty())
        __debugbreak();

    gc_pending = queue.next(gc_pending);

    // mark entire queue for GC if flush flag is asserted
    if (flush) {
        while (gc_pending != queue.head_i) {
            gc_pending = queue.next(gc_pending);
        }
    }

    // Return if queue is empty
    if (gc_pending == queue.head_i) {
        running = false;
        return;
    }

//     // Get a new block
//     Block* next = this->queue.item_ref(gc_pending);
//     next->begin(); // causes on_block_begin() (above) to be called

}

// Wait for the queue to be empty
void Conveyor::wait_for_empty_queue()
{
    while (!queue.is_empty()) {
        ensure_running();
        THEKERNEL->call_event(ON_IDLE, this);
    }
}

/*
 * push the pre-prepared head block onto the queue
 */
void Conveyor::queue_head_block()
{
    // upstream caller will block on this until there is room in the queue
    while (queue.is_full()) {
        ensure_running();
        THEKERNEL->call_event(ON_IDLE, this);
    }

    if(halted) {
        // we do not want to stick more stuff on the queue if we are in halt state
        // clear and release the block on the head
        queue.head_ref()->clear();

    } else {
        queue.head_ref()->ready();
        queue.produce_head();
    }
}

void Conveyor::ensure_running()
{
    // if we are already running or the block queue is empty do nothing
    if (running || gc_pending == queue.head_i) return;

    // if the job queue is empty we can force the next block into it
    if(THEKERNEL->step_ticker->is_jobq_empty()) {
        // force the next block if not already forced
        if(queue.item_ref(gc_pending)->is_job) return;
        running = true;
        queue.item_ref(gc_pending)->begin(); // on_block_begin() above gets called
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
    flush = true;
    wait_for_empty_queue();
    flush = false;
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
