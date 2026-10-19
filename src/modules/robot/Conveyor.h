/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include "libs/Module.h"
#include "BlockQueue.h"

class Block;

class Conveyor : public Module
{
public:
    Conveyor();
    void start(uint8_t n_actuators);

    void on_module_loaded(void);
    void on_idle(void *);
    void on_halt(void *);

    void wait_for_idle(bool wait_for_motors=true);
    bool is_queue_empty() { return queue.is_empty(); };
    bool is_queue_full() { return queue.is_full(); };
    bool is_idle() const;

    // returns next available block writes it to block and returns true
    bool get_next_block(Block **block);
    void block_finished();
    // true when the block last returned by get_next_block() is the continuous-mode block being fed again, not a block taken from the queue
    bool is_refeed() const { return refeed; }

    void dump_queue(void);
    void flush_queue(void);
    float get_current_feedrate() const { return current_feedrate; }
    void force_queue() { check_queue(true); }
    bool set_continuous_mode(bool f);
    void set_hold(bool f) { hold_queue= f; }
    unsigned int get_queue_used_bytes() const { return queue.used_bytes(); }
    unsigned int get_queue_arena_bytes() const { return queue.get_arena_bytes(); }
    unsigned int get_queue_length() const { return queue.get_length(); }

    friend class Planner; // for queue

private:
    void check_queue(bool force= false);
    void queue_head_block(void);

    using  Queue_t= BlockQueue;
    Queue_t queue;  // Queue of Blocks

    uint32_t queue_delay_time_ms;
    size_t queue_size;
    size_t queue_bytes;
    float current_feedrate{0}; // actual nominal feedrate that current block is running at in mm/sec
    const Block *last_fed{nullptr}; // block returned by the previous get_next_block(), only touched by the ISR

    volatile struct {
        volatile bool running:1;
        volatile bool allow_fetch:1;
        volatile bool flush:1;
        volatile bool hold_queue:1;
        volatile uint8_t continuous_mode:2;
        volatile bool refeed:1; // set by get_next_block(), see is_refeed()
    };

};
