/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.

      With chucks taken from http://en.wikipedia.org/wiki/Circular_buffer, see licence there also
*/

#ifndef RINGBUFFER_H
#define RINGBUFFER_H


template<class kind, int length> class RingBuffer
{
    public:
        RingBuffer();
        int          size();
        int          capacity();
        int          next_block_index(int index);
        int          prev_block_index(int index);
        kind*        next_head(void);
        kind*        consume_head(void);
        kind*        consume_tail(void);
        void         push_back(kind object);
        void         pop_front(kind &object);
        kind*        get_tail_ref();
        void         get( int index, kind &object);
        kind*        get_ref( int index);
        kind*        delete_first();

        kind         buffer[length];
        int          head;
        int          tail;
};


template<class kind, int length> RingBuffer<kind, length>::RingBuffer()
{
    tail = head = 0;
}

template<class kind, int length>  int RingBuffer<kind, length>::capacity()
{
    return length - 1;
}

template<class kind, int length>  int RingBuffer<kind, length>::size()
{
    return head - tail + ((tail > head)?length:0);
}

template<class kind, int length> int RingBuffer<kind, length>::next_block_index(int index)
{
    return ((++index >= length)?0:index);
}

template<class kind, int length> int RingBuffer<kind, length>::prev_block_index(int index)
{
    return (index?index:length) - 1;
}

template<class kind, int length> kind* RingBuffer<kind, length>::next_head()
{
    return &(buffer[head]);
}

template<class kind, int length> kind* RingBuffer<kind, length>::consume_head()
{
    kind* r = &(buffer[head]);
    head = next_block_index(head);
    return r;
}

template<class kind, int length> kind* RingBuffer<kind, length>::consume_tail()
{
    kind* r = &(buffer[tail]);
    tail = next_block_index(tail);
    return r;
}

template<class kind, int length> kind* RingBuffer<kind, length>::get_tail_ref(){
    return &(buffer[tail]);
}

template<class kind, int length> void RingBuffer<kind, length>::push_back(kind object)
{
    buffer[head] = object;
    head = next_block_index(head);
}

template<class kind, int length> void RingBuffer<kind, length>::get(int index, kind &object)
{
    int j= 0;
    int k= tail;
    while (k != head){
        if (j == index)
            break;
        j++;
        k = next_block_index(k);
    }
    object = buffer[k];
}


template<class kind, int length> kind* RingBuffer<kind, length>::get_ref(int index)
{
    int j = 0;
    int k = tail;
    while (k != head){
        if (j == index)
            break;
        j++;
        k= next_block_index(k);
    }
    // TODO : this checks wether we are asked a value out of range
    if (k == head){
        return NULL;
    }
    return &(buffer[k]);
}

template<class kind, int length> void RingBuffer<kind, length>::pop_front(kind &object)
{
    object = buffer[tail];
    tail = next_block_index(tail);
}

template<class kind, int length> kind* RingBuffer<kind, length>::delete_first()
{
    kind* r = &(buffer[tail]);
    tail = next_block_index(tail);
    return r;
}


#endif
