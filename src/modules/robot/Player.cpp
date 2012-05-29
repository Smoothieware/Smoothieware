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
#include "Player.h"
#include "Planner.h"

Player::Player(){
    this->current_block = NULL;
    this->looking_for_new_block = false;
}

// Append a block to the list
Block* Player::new_block(){

    // Clean up the vector of commands in the block we are about to replace
    // It is quite strange to do this here, we really should do it inside Block->pop_and_execute_gcode
    // but that function is called inside an interrupt and thus can break everything if the interrupt was trigerred during a memory access
    //Block* block = this->queue.get_ref( this->queue.size()-1 );
    Block* block = this->queue.get_ref( this->queue.size() );
    if( block->player == this ){
        for(short index=0; index<block->gcodes.size(); index++){
            block->gcodes.pop_back(); 
        }     
    }
    
    // Create a new virgin Block in the queue 
    this->queue.push_back(Block());
    block = this->queue.get_ref( this->queue.size()-1 );
    block->is_ready = false;
    block->initial_rate = -2;
    block->final_rate = -2;
    block->player = this;
    
    return block;
}

// Used by blocks to signal when they are ready to be used by the system
void Player::new_block_added(){
    if( this->current_block == NULL ){
        this->pop_and_process_new_block(33);
    }
}

// Process a new block in the queue
void Player::pop_and_process_new_block(int debug){
    if( this->looking_for_new_block ){ return; }
    this->looking_for_new_block = true;

    if( this->current_block != NULL ){ this->looking_for_new_block = false; return; }

    // Return if queue is empty 
    if( this->queue.size() == 0 ){
        this->current_block = NULL; 
        // TODO : ON_QUEUE_EMPTY event 
        this->looking_for_new_block = false;
        return; 
    }
    
    // Get a new block
    this->current_block = this->queue.get_ref(0);

    // Tell all modules about it
    this->kernel->call_event(ON_BLOCK_BEGIN, this->current_block);
    
    // In case the module was not taken
    if( this->current_block->times_taken < 1 ){
        this->looking_for_new_block = false;
        this->current_block->release();
    }

    this->looking_for_new_block = false;

}

void Player::wait_for_queue(int free_blocks){
    mbed::Timer t;
    while( this->queue.size() >= this->queue.capacity()-free_blocks ){
        t.reset();
        t.start();
        this->kernel->call_event(ON_IDLE);
        t.stop();
        if(t.read_us() < 500)
            wait_us(500 - t.read_us());
    }
}
