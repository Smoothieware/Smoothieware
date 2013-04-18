/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef CONVEYOR_H
#define CONVEYOR_H

#include "libs/Module.h"
#include "libs/Kernel.h"
using namespace std;
#include <string>
#include <vector>

#include "Action.h"

typedef RingBuffer<Action,16> ActionQueue;

class Conveyor : public Module {
    public:
        Conveyor();

        void on_module_loaded(void);
        void on_idle(void*);

        Action* commit_action();

        void start_next_action(void);

        // first ensures we have an available action, then returns a pointer to it
        Action* next_action(void);

        void wait_for_queue(int free_blocks);
        void wait_for_empty_queue();

        ActionQueue queue;  // Queue of Blocks

        // the next FREE action. Data is added to this action and it's placed in the queue when someone calls commit_action()
        Action* _next_action;

        // the CURRENTLY EXECUTING action. This is the tail of the queue for interrupt context.
        // in idle context, we advance the ring's tail to the action before this one, cleaning the completed actions as we go.
        Action* current_action;
        int current_action_index;
};

#endif // CONVEYOR_H
