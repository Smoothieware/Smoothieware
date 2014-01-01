/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef STREAMOUTPUTPOOL_H
#define STREAMOUTPUTPOOL_H

using namespace std;
#include <forward_list>
#include <string>
#include <cstdio>
#include <cstdarg>

#include "libs/Channel.h"

class ChannelPool : public Channel {

public:
    ChannelPool(){
    }

    int puts(const char* s)
    {
        int r = 0;
        for(auto i = channels.begin(); i != channels.end(); i++)
        {
            int k = (*i)->puts(s);
            if (k > r)
                r = k;
        }
        return r;
    }

    void append_channel(Channel* channel)
    {
        channels.push_front(channel);
    }

    void remove_channel(Channel* channel)
    {
        channels.remove(channel);
    }

    bool on_receive_line()
    {
        for (auto it = channels.begin(); it != channels.end(); ++it)
        {
            if ((*it)->on_receive_line())
                return true;
        }
        return false;
    }

private:
    forward_list<Channel*> channels;
};

#endif
