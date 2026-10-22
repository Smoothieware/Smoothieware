/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "RealtimeFilter.h"

#include "libs/Kernel.h"

bool RealtimeFilter::handle(char b)
{
    switch(b) {
        case '?':
            query = true;
            return true;

        case 'X' - 'A' + 1: // ^X
            halt = true;
            reset(); // the line being received is abandoned
            return true;

        case 'Y' - 'A' + 1: // ^Y
            THEKERNEL->set_stop_request(true); // generic stop what you are doing request
            return true;

        case '!': // safe pause
            if(at_line_start && THEKERNEL->is_feed_hold_enabled()) {
                THEKERNEL->set_feed_hold(true);
                return true;
            }
            break;

        case '~': // safe resume
            if(at_line_start && THEKERNEL->is_feed_hold_enabled()) {
                THEKERNEL->set_feed_hold(false);
                return true;
            }
            break;

        case '\n':
            if(last_char_was_cr) {
                // handle \r\n as a single line terminator
                last_char_was_cr = false;
                return true;
            }
            break;
    }

    last_char_was_cr = (b == '\r');
    at_line_start = (b == '\n' || b == '\r');
    return false;
}
