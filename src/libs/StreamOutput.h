/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef STREAMOUTPUT_H
#define STREAMOUTPUT_H

#include <Stream.h>
#include <cstdarg>

class StreamOutput : public mbed::Stream {
    public:
        StreamOutput(){}
//         virtual int puts(const char *str) = 0;
        virtual int printf(const char* format, ...) __attribute__ ((format(printf, 2, 3))) {
            char *buffer;
            // Make the message
            va_list args;
            va_start(args, format);

            int size = vsnprintf(NULL, 0, format, args)
                + 1; // we add one to take into account space for the terminating \0

            buffer = new char[size];
            vsnprintf(buffer, size, format, args);
            va_end(args);

            puts(buffer);

            delete[] buffer;
            return size - 1;
        }
        virtual int _putc(int c) { return 0; }
        virtual int _getc(void) { return 0; }
        virtual int puts(const char*) { return 0; };
};

#endif
