/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
      
      With chucks taken from http://en.wikipedia.org/wiki/Circular_buffer, see licence there also
*/

#ifndef RINGBUFFER_H
#define RINGBUFFER_H


template<class kind, int length> class RingBuffer {
    public:
        RingBuffer();
        int          size();
        int          capacity();
        int          next_block_index(int index);
        int          prev_block_index(int index);
        void         push_back(kind object);
        void         pop_front(kind &object);
        kind*        get_tail_ref();
        void         get( int index, kind &object);
        kind*        get_ref( int index);
        void         delete_first();

        kind         buffer[length];
        volatile int          head;
        volatile int          tail;
};

#include "sLPC17xx.h"

template<class kind, int length> RingBuffer<kind, length>::RingBuffer(){
    this->head = this->tail = 0;
}

template<class kind, int length>  int RingBuffer<kind, length>::capacity(){
    return length-1;
}

template<class kind, int length>  int RingBuffer<kind, length>::size(){
	//return((this->head>this->tail)?length:0)+this->tail-head;
	__disable_irq();
	int i = tail - head + ((head > tail)?length:0);
	__enable_irq();
	return i;
}

template<class kind, int length> int RingBuffer<kind, length>::next_block_index(int index){
    index++;
    if (index == length) { index = 0; }
    return(index);
}

template<class kind, int length> int RingBuffer<kind, length>::prev_block_index(int index){
    if (index == 0) { index = length; }
    index--;
    return(index);
}

template<class kind, int length> void RingBuffer<kind, length>::push_back(kind object){
    this->buffer[this->tail] = object;
    this->tail = (tail+1)&(length-1);
}

template<class kind, int length> kind* RingBuffer<kind, length>::get_tail_ref(){
    return &(buffer[tail]);
}

template<class kind, int length> void RingBuffer<kind, length>::get(int index, kind &object){
    int j= 0;
    int k= this->head;
    while (k != this->tail){
        if (j == index) break;
        j++;
        k= (k + 1) & (length - 1);
    }
    // TODO : this checks wether we are asked a value out of range
    //if (k == this->tail){
    //    return NULL;
    //}
    object = this->buffer[k];
}


template<class kind, int length> kind* RingBuffer<kind, length>::get_ref(int index){
    int j= 0;
    int k= this->head;
    while (k != this->tail){
        if (j == index) break;
        j++;
        k= (k + 1) & (length - 1);
    }
    // TODO : this checks wether we are asked a value out of range
    if (k == this->tail){
        return NULL;
    }
    return &(this->buffer[k]);
}

template<class kind, int length> void RingBuffer<kind, length>::pop_front(kind &object){
    object = this->buffer[this->head];
    this->head = (this->head+1)&(length-1);
}

template<class kind, int length> void RingBuffer<kind, length>::delete_first(){
    //kind dummy;
    //this->pop_front(dummy);
    this->head = (this->head+1)&(length-1);
}


#endif
