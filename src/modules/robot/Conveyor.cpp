/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl) with additions from Sungeun K. Jeon (https://github.com/chamnit/grbl)
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

using namespace std;
#include <vector>
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

Conveyor::Conveyor(){
    gc_pending = queue.tail_i;
    running = false;
}

void Conveyor::on_module_loaded(){
    register_for_event(ON_IDLE);
    register_for_event(ON_MAIN_LOOP);

    on_config_reload(this);
}

// Delete blocks here, because they can't be deleted in interrupt context ( see Block.cpp:release )
// note that blocks get cleaned as they come off the tail, so head ALWAYS points to a cleaned block.
void Conveyor::on_idle(void* argument){
    if (queue.tail_i != gc_pending)
    {
        if (queue.is_empty())
            __debugbreak();
        else
        {
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
    if (running)
        return;

    if (queue.is_empty())
    {
        if (queue.head_ref()->gcodes.size())
        {
            queue_head_block();
            ensure_running();
        }
    }
    else
        // queue not empty
        ensure_running();
}

void Conveyor::on_config_reload(void* argument)
{
    queue.resize(THEKERNEL->config->value(planner_queue_size_checksum)->by_default(32)->as_number());
}

void Conveyor::append_gcode(Gcode* gcode)
{
    gcode->mark_as_taken();
    queue.head_ref()->append_gcode(gcode);
}

// Process a new block in the queue
void Conveyor::on_block_end(void* block)
{
    if (queue.is_empty())
        __debugbreak();

    gc_pending = queue.next(gc_pending);

    // Return if queue is empty
    if (gc_pending == queue.head_i)
    {
        running = false;
        return;
    }

    // Get a new block
    Block* next = this->queue.item_ref(gc_pending);

    next->begin();
}

// Wait for the queue to be empty
void Conveyor::wait_for_empty_queue()
{
    while (!queue.is_empty())
    {
        ensure_running();
        THEKERNEL->call_event(ON_IDLE, this);
    }
}

/*
 * push the pre-prepared head block onto the queue
 */
void Conveyor::queue_head_block()
{
    while (queue.is_full())
    {
        ensure_running();
        THEKERNEL->call_event(ON_IDLE, this);
    }

    queue.head_ref()->ready();
    queue.produce_head();
}

void Conveyor::ensure_running()
{
    if (!running)
    {
        if (gc_pending == queue.head_i)
            return;

        running = true;
        queue.item_ref(gc_pending)->begin();
    }
}

// Debug function
void Conveyor::dump_queue()
{
    for (unsigned int index = queue.tail_i, i = 0; true; index = queue.next(index), i++ )
    {
        THEKERNEL->streams->printf("block %03d > ", i);
        queue.item_ref(index)->debug();

        if (index == queue.head_i)
            break;
    }
}

// feels hacky, but apparently the way to do it
#include "HeapRing.cpp"
template class HeapRing<Block>;
