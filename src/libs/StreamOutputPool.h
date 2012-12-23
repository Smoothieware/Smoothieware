/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef STREAMOUTPUTPOOL_H
#define STREAMOUTPUTPOOL_H

using namespace std;
#include <vector>
#include <string>
#include <cstdio>
#include <cstdarg>

#include "libs/StreamOutput.h"



class StreamOutputPool {
    public:
       StreamOutputPool(){}
       int printf(const std::string format, ...){
            // Make the message
            va_list args;
            va_start(args, format);
            int size = format.size() * 2;
            char* buffer = new char[size];
            int len = vsnprintf(buffer, size, format.c_str(), args);
            if (len >= size) {
                delete[] buffer;
                size = len+1;
                buffer = new char[size];
                len = vsnprintf(buffer, size, format.c_str(), args);
            }
            va_end(args);

            // Dispatch to all
            for(unsigned int i=0; i < this->streams.size(); i++){
                this->streams.at(i)->printf(buffer);
            }
            delete[] buffer;
            return len;

       }

       void append_stream(StreamOutput* stream){
           this->streams.push_back(stream);
       }


       vector<StreamOutput*> streams;
};









#endif
