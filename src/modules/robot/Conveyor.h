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
    void on_suspend(void *);

    void wait_for_idle(bool wait_for_motors=true);
    bool is_queue_empty() { return queue.is_empty(); };
    bool is_queue_full() { return queue.is_full(); };
    bool is_idle() const;

    // returns next available block writes it to block and returns true
    bool get_next_block(Block **block);
    void block_finished();

    void dump_queue(void);
    void flush_queue(void);
    float get_current_feedrate() const { return current_feedrate; }
    void force_queue() { check_queue(true); }

    friend class Planner; // for queue

private:
    void check_queue(bool force= false);
    void queue_head_block(void);

    using  Queue_t= BlockQueue;
    Queue_t queue;  // Queue of Blocks

    uint32_t queue_delay_time_ms;
    size_t queue_size;
    float current_feedrate{0}; // actual nominal feedrate that current block is running at in mm/sec

    struct {
        volatile bool running:1;
        volatile bool allow_fetch:1;
        bool flush:1;
    };

};
