/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef GCODE_H
#define GCODE_H
#include <string>
using std::string;
#include "libs/StreamOutput.h"
// Object to represent a Gcode command
#include <stdlib.h>

class Gcode {
    public:
        Gcode(const string&, StreamOutput*);

        bool   has_letter ( char letter );

        double get_value  ( char letter );

        double get_double ( char letter );
        int    get_int    ( char letter );

        int    get_num_args();
        void   prepare_cached_values();

        const string command;
        double millimeters_of_travel;
        bool call_on_gcode_execute_event_immediatly;
        bool on_gcode_execute_event_called;

        bool has_m;
        bool has_g;
        unsigned int m;
        unsigned int g;

        bool add_nl;
        StreamOutput* stream;
        
        // This requires a long explanation, unfortunately. We have the hard job to delete the gcode at the right momont.
        // The problem is that we can not delete the gcode before no modules wants to use/read it anymore. This gets complicated because there are actually two possible end moments 
        // of the life of the gcode, and they can happen in either order. So if one deletes it first, the other can not access it. The last context to use it is the one that must delete it, 
        // but it's not always the same.
        // Here is a resume of the life of a Gcode object : 
        // * Gets created inside GcodeDispatch
        // * Is passed along with the on_gcode_received event to all modules. We'll ignore all other modules and just concentrate on Robot
        // * Robot computes the distance for this gcode, if there is none, it just returns, otherwise : 
        // * If the queue is empty, we execute the gcode right away ( on_gcode_execute ), it is not attached to the queue, then we add blocks to the queue and they start getting executed
        // * If the queue is not empty however, we attach the gcode to the last block on the queue, then start adding blocks to the queue. This does the same as the line above, but with the queue already running, things happen later instead of immediately.
        // * Now we return to GcodeDispatch. 
        // The problem when we get there is this : if we attached the Gcode to a block in the Queue, We don't know now if it has been executed or not. Usually, it hasn't : we just added it to the queue and the queue has not gotton to it yet.
        // In that case, we can obviously not delete it right here : the queue still needs it to execute it. So why not delete it when it gets executed ?
        // Because there is one case with which we can get here, and the gcode *has* been executed already. That's in the case this gcode string appened so many blocks to the queue, that in order to make room for the later blocks
        // we waited for the earlier ones to get consumed. In that case, the gcode will get executed when we get to it doing that.
        // So, we can't *always* delete the gcode when it gets executed, and we can't *always* delete the gcode when we get back to GcodeDispatch.
        // So, there are two places we can delete the Gcode, let's call them : 
        // * NEW BLOCK CONTEXT ( when the queue has executed this gcode and can delete it ( actually two places, one in on_idle, the other when recycling a block for re-use, see Conveyor.cpp )
        // * ALL BLOCKS ADDED CONTEXT ( when we return to GcodeDispatch after the on_gcode_received call, we know all blocks that would be added for this gcode have been )
        // Now the trick here is basically : In any given context, we want to delete the gcode 
        // * If we reach the current context and the other context was not supposed to happen
        // * If we reach the current context and the other context did happen
        // In either of these cases, we want delete the gcode, otherwise, we want to mark the gcode as having passed thru the current context so that when we reach that other context, the gcode gets deleted
        // There is a complication/simplification here : the ALL_BLOCKS_ADDED context will always occur for a given gcode. That's a condition that will always be true, and a flag we don't need to keep track of.
        // The other one however, will only occur if the block is added to the queue. It will not otherwise.
        // So we use a flag to keep track of that
        bool will_go_thru_new_block_context;               // Defaults to false
        // Now for each context, we need a flag ( defaulting to false ) that is set to true when we go thru that context without deleting the Gcode.
        bool passed_thru_new_block_context_without_deleting;
        bool passed_thru_all_blocks_added_context_without_deleting;





};
#endif
