/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef REALTIMEFILTER_H
#define REALTIMEFILTER_H

#include <stdint.h>

// Decides what the receive interrupt of a serial stream does with each byte, so that every stream
// (USB serial, UART) follows the same rules:
//
//   ?   status query            anywhere in the stream
//   ^X  halt                    anywhere in the stream
//   ^Y  stop request            anywhere in the stream
//   !   feed hold               only as the first character of a line (and only if enable_feed_hold is set)
//   ~   feed resume             idem
//
// Anywhere else ! and ~ are part of the line, for example the inverted pin in "config-set sd alpha_step_pin 2.1!"
// or the short file name in "play /sd/LONGFI~1.GCO".
// ? ^X and ^Y are not characters of the line, so a status poll between the end of a line and a ! does not
// stop that ! from being recognised.
//
// The interrupt only records ? and ^X, the main loop answers them with take_query() and take_halt().
// The line state and the two requests each live in their own byte, so the interrupt and the main loop
// never share a read-modify-write.
class RealtimeFilter {
    public:
        RealtimeFilter() { reset(); query = halt = false; }

        // called from the receive interrupt for each received character.
        // Returns true if the character was a realtime command (or the \n of a \r\n pair) and must not be queued.
        // Feed hold, resume and stop requests are carried out right away.
        bool handle(char b);

        // called from the main loop, return true once per request
        bool take_query() { bool f = query; query = false; return f; }
        bool take_halt()  { bool f = halt;  halt = false;  return f; }

        // the next character starts a new line
        void reset() { at_line_start = true; last_char_was_cr = false; }

    private:
        // only written from the receive interrupt (and reset())
        bool at_line_start:1;
        bool last_char_was_cr:1;

        volatile bool query;
        volatile bool halt;
};

#endif
