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

#define planner_queue_size_checksum CHECKSUM("planner_queue_size")

// The conveyor holds the queue of blocks, takes care of creating them, and starting the executing chain of blocks

Conveyor::Conveyor(){
    gc_pending = queue.tail_i;
    running = false;
}

void Conveyor::on_module_loaded(){
    register_for_event(ON_IDLE);
    register_for_event(ON_MAIN_LOOP);
    register_for_event(ON_CONFIG_RELOAD);

    on_config_reload(this);
}

// Delete blocks here, because they can't be deleted in interrupt context ( see Block.cpp:release )
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

// Wait for the queue to have a given number of free blocks
void Conveyor::wait_for_queue(int free_blocks)
{
    while (queue.is_full())
    {
        ensure_running();
        THEKERNEL->call_event(ON_IDLE);
    }
}

// Wait for the queue to be empty
void Conveyor::wait_for_empty_queue()
{
    while (!queue.is_empty())
    {
        ensure_running();
        THEKERNEL->call_event(ON_IDLE);
    }
}

// Return true if the queue is empty
bool Conveyor::is_queue_empty()
{
    return queue.is_empty();
}

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
