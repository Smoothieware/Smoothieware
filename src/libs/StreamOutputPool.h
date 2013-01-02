/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef STREAMOUTPUTPOOL_H
#define STREAMOUTPUTPOOL_H

using namespace std;
#include <set>
#include <string>
#include <cstdio>
#include <cstdarg>

#include "libs/StreamOutput.h"

#define PRINTF_DEFAULT_BUFFER_SIZE 32

class StreamOutputPool {
private:
    char *printf_default_buffer;
public:
    StreamOutputPool(){
        printf_default_buffer = new char[PRINTF_DEFAULT_BUFFER_SIZE];
    }
    int printf(const std::string format, ...){
        char *buffer = printf_default_buffer;
        // Make the message
        va_list args;
        va_start(args, format);

        int size = vsnprintf(buffer, PRINTF_DEFAULT_BUFFER_SIZE, format.c_str(), args)
            + 1; // we add one to take into account space for the terminating \0

        if (size > PRINTF_DEFAULT_BUFFER_SIZE) {
            buffer = new char[size];
            vsnprintf(buffer, size, format.c_str(), args);
        }

        va_end(args);

        // Dispatch to all
        for(set<StreamOutput*>::iterator i = this->streams.begin(); i != this->streams.end(); i++)
        {
            (*i)->puts(buffer);
        }

        if (buffer != printf_default_buffer)
            delete buffer;

        return size - 1;
    }

    void append_stream(StreamOutput* stream)
    {
        this->streams.insert(stream);
    }

    void remove_stream(StreamOutput* stream)
    {
        this->streams.erase(stream);
    }

    set<StreamOutput*> streams;
};

#endif
