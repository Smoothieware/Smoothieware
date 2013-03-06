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

Conveyor::Conveyor(){
    this->current_block = NULL;
    this->looking_for_new_block = false;
    flush_blocks = 0;
}

void Conveyor::on_module_loaded(){
    register_for_event(ON_IDLE);
}

void Conveyor::on_idle(void* argument){
    if (flush_blocks){

        Block* block = queue.get_tail_ref();
        while (block->gcodes.size()){
            Gcode* gcode = block->gcodes.back();
            block->gcodes.pop_back();
        
            // Do we just delete this gcode from this vector, or do we also actually delete the gcode ?
            // This is pretty complex and explained in detail in Gcode.h 
            if( gcode->passed_thru_all_blocks_added_context_without_deleting == true ){
                delete gcode;
            }else{
                gcode->passed_thru_new_block_context_without_deleting = true;
            }
        
        }
        queue.delete_first();

        __disable_irq();
        flush_blocks--;
        __enable_irq();
    }
}

// Append a block to the list
Block* Conveyor::new_block(){

    // Clean up the vector of commands in the block we are about to replace
    // It is quite strange to do this here, we really should do it inside Block->pop_and_execute_gcode
    // but that function is called inside an interrupt and thus can break everything if the interrupt was trigerred during a memory access

    // Take the next untaken block on the queue ( the one after the last one )
    Block* block = this->queue.get_tail_ref();
    // Then clean it up
    if( block->conveyor == this ){
        for(; block->gcodes.size(); ){
            Gcode* gcode = block->gcodes.back();
            block->gcodes.pop_back();
        
            // Do we just delete this gcode from this vector, or do we also actually delete the gcode ?
            // This is pretty complex and explained in detail in Gcode.h 
            if( gcode->passed_thru_all_blocks_added_context_without_deleting == true ){
                delete gcode;
            }else{
                gcode->passed_thru_new_block_context_without_deleting = true;
            }
       
        }
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
    this->kernel->call_event(ON_BLOCK_BEGIN, this->current_block);

    // In case the module was not taken
    if( this->current_block->times_taken < 1 ){
        Block* temp = this->current_block; 
        this->current_block = NULL; // It seems this was missing and adding it fixes things, if something breaks, this may be a suspect 
        temp->take(); 
        temp->release();
    }

    this->looking_for_new_block = false;

}

void Conveyor::wait_for_queue(int free_blocks)
{
    while( this->queue.size() >= this->queue.capacity()-free_blocks ){
        this->kernel->call_event(ON_IDLE);
    }
}

void Conveyor::wait_for_empty_queue(){
    while( this->queue.size() > 0){
        this->kernel->call_event(ON_IDLE);
    }
}

