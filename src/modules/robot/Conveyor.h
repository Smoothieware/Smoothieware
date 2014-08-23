/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef CONVEYOR_H
#define CONVEYOR_H

#include "libs/Module.h"
#include "HeapRing.h"

using namespace std;
#include <string>
#include <vector>

class Gcode;
class Block;

class Conveyor : public Module
{
    typedef HeapRing<Block> Queue_t;

public:
    typedef Queue_t::size_type  size_type;
    typedef Queue_t::value_type value_type;

    Conveyor();

    void on_module_loaded(void);
    void on_idle(void *);
    void on_main_loop(void *);
    void on_block_end(void *);
    void on_config_reload(void *);

    void notify_block_finished(Block *);

    void wait_for_empty_queue();
    bool is_queue_empty() { return queue.is_empty(); };

    void ensure_running(void);

    void append_gcode(Gcode *);
    void queue_head_block(void);

    void dump_queue(void);

    size_type   after(size_type i)  { return queue.next(i); }
    value_type* at(unsigned i)      { return queue.item_ref(i); }
    size_type   begin()             { return queue.head_i; }
    size_type   before(size_type i) { return queue.prev(i); }
    size_type   end()               { return queue.tail_i; }
    value_type* front()             { return queue.head_ref(); }
    value_type* previous()          { return at(queue.prev(queue.head_i)); }

private:
    Queue_t queue;  // Queue of Blocks

    volatile bool running;

    volatile unsigned int gc_pending;
};

#endif // CONVEYOR_H
