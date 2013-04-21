/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.

      With chucks taken from http://en.wikipedia.org/wiki/Circular_buffer, see licence there also
*/

#ifndef RINGBUFFER_H
#define RINGBUFFER_H

#include "sLPC17xx.h" // for __disable_irq and __enable_irq

template<class kind, int length> class RingBuffer {
    public:
        RingBuffer();
        int         size();
        int         capacity();

        bool        full();
        bool        empty();

        int         next_block_index(int index);
        int         prev_block_index(int index);

        kind*       get_tail_ref(void);
        kind*       get_head_ref(void);
        kind*       get_ref(int index);

        void        push(kind& item);
        void        pop(kind& item);

        kind*       produce_head(void);
        kind*       consume_tail(void);

        void        push_back(kind);
        kind        pop_front(kind);

        kind        buffer[length];
        int         head;
        int         tail;
};


template<class kind, int length> RingBuffer<kind, length>::RingBuffer()
{
    head = tail = 0;
}

template<class kind, int length>  int RingBuffer<kind, length>::capacity()
{
    return length-1;
}

template<class kind, int length>  int RingBuffer<kind, length>::size()
{
    int size;

    __disable_irq();
    size = ((head > tail)?length:0) + tail - head;
    __enable_irq();

    return size;
}

template<class kind, int length>  bool RingBuffer<kind, length>::full()
{
    return (next_block_index(head) == tail);
}

template<class kind, int length>  bool RingBuffer<kind, length>::empty()
{
    return (head == tail);
}

// get index + 1, obeying ring wrap
template<class kind, int length> int RingBuffer<kind, length>::next_block_index(int index)
{
    index++;
    if (index == length)
        index = 0;
    return index;
}

// get index - 1, obeying ring wrap
template<class kind, int length> int RingBuffer<kind, length>::prev_block_index(int index)
{
    if (index == 0)
        index = length;
    index--;
    return index;
}

// get reference to tail (oldest item in queue)
template<class kind, int length> kind* RingBuffer<kind, length>::get_tail_ref()
{
    return &(buffer[tail]);
}

// get reference to next head (item that will be added by produce_head)
template<class kind, int length> kind* RingBuffer<kind, length>::get_head_ref()
{
    return &(buffer[head]);
}

// get reference to a specific item.
// positive numbers (including 0) count up from the tail
// negative numbers count backwards from head
template<class kind, int length> kind* RingBuffer<kind, length>::get_ref(int index)
{
    if (index >= 0)
    {
        index += tail;
        while (index >= length)
            index -= length;

        return &buffer[index];
    }
    else
    {
        index = head - index;
        while (index < 0)
            index += length;

        return &buffer[head];
    }
}

// add an item to the queue
// WARNING: busy loops while queue is full. Check size before calling if you don't want this!
template<class kind, int length> void RingBuffer<kind, length>::push(kind& item)
{
    // busy loop while queue is full
    while (next_block_index(head) == tail);

    buffer[next_block_index(head)] = item;
    head = next_block_index(head);
}

// pop last item from queue
template<class kind, int length> void RingBuffer<kind, length>::pop(kind& item)
{
    // get last item
    item = buffer[tail];

    // advance tail if queue is not empty
    if (tail != head)
        tail = next_block_index(tail);
}

// enqueue newest item
// get reference with get_head_ref(), then call this function when it's ready
// WARNING: busy loops while queue is full. Check queue size before calling if you don't want this!
template<class kind, int length> kind* RingBuffer<kind, length>::produce_head()
{
    // busy loop while queue is full
    while (next_block_index(head) == tail);

    // increment head
    head = next_block_index(head);

    // return ref to new head
    return &(buffer[head]);
}

// remove oldest item from queue
// get reference with get_tail_ref(), call this function when you're finished with it
template<class kind, int length> kind* RingBuffer<kind, length>::consume_tail()
{
    // increment tail if queue is not empty, leave it alone if queue is empty
    if (tail == head)
        return NULL;

    tail = next_block_index(tail);

    // return pointer to new tail
    return &(buffer[tail]);
}

#endif
