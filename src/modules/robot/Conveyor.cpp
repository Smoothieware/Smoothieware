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

// The conveyor holds the queue of blocks, takes care of creating them, and starting the executing chain of blocks

Conveyor::Conveyor(){
    this->current_block = NULL;
    this->looking_for_new_block = false;
    flush_blocks = 0;
}

void Conveyor::on_module_loaded(){
    register_for_event(ON_IDLE);
}

// Delete blocks here, because they can't be deleted in interrupt context ( see Block.cpp:release )
void Conveyor::on_idle(void* argument){
    if (flush_blocks){
        // Cleanly delete block 
        Block* block = queue.get_head_ref();
        block->gcodes.clear(); 
        queue.delete_first();
        __disable_irq();
        flush_blocks--;
        __enable_irq();
    }
}

// Append a block to the list
Block* Conveyor::new_block(){

    // Take the next untaken block on the queue ( the one after the last one )
    Block* block = this->queue.get_head_ref();
    // Then clean it up
    if( block->conveyor == this ){
        block->gcodes.clear();
    }

    // Create a new virgin Block in the queue
    this->queue.push_back(Block());
    block = this->queue.get_ref( this->queue.size()-1 );
    while( block == NULL ){
        block = this->queue.get_ref( this->queue.size()-1 );
    }
    block->is_ready = false;
    block->initial_rate = -2;
    block->final_rate = -2;
    block->conveyor = this;
    
    return block;
}

// Used by blocks to signal when they are ready to be used by the system
void Conveyor::new_block_added(){
    if( this->current_block == NULL ){
        this->pop_and_process_new_block(33);
    }
}

// Process a new block in the queue
void Conveyor::pop_and_process_new_block(int debug){
    if( this->looking_for_new_block ){ return; }
    this->looking_for_new_block = true;

    if( this->current_block != NULL ){ this->looking_for_new_block = false; return; }

    // Return if queue is empty
    if( this->queue.size() == 0 ){
        this->current_block = NULL;
        // TODO : ON_QUEUE_EMPTY event
        this->looking_for_new_block = false;
        return;
    }

    // Get a new block
    this->current_block = this->queue.get_ref(0);

    // Tell all modules about it
    THEKERNEL->call_event(ON_BLOCK_BEGIN, this->current_block);

	// In case the module was not taken
    if( this->current_block->times_taken < 1 ){
        Block* temp = this->current_block; 
        this->current_block = NULL; // It seems this was missing and adding it fixes things, if something breaks, this may be a suspect 
        temp->take(); 
        temp->release();
    }

    this->looking_for_new_block = false;

}

// Wait for the queue to have a given number of free blocks
void Conveyor::wait_for_queue(int free_blocks){
    while( this->queue.size() >= this->queue.capacity()-free_blocks ){
        THEKERNEL->call_event(ON_IDLE);
    }
}

// Wait for the queue to be empty
void Conveyor::wait_for_empty_queue(){
    while( this->queue.size() > 0){
        THEKERNEL->call_event(ON_IDLE);
    }
}

// Return true if the queue is empty
bool Conveyor::is_queue_empty(){
    return (this->queue.size() == 0);
}

