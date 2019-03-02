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
        kind*        get_head_ref();
        kind*        get_tail_ref();
        void         get( int index, kind &object);
        kind*        get_ref( int index);
        void         delete_tail();

        kind         buffer[length];
        volatile int          tail;
        volatile int          head;
};

#include "sLPC17xx.h"

template<class kind, int length> RingBuffer<kind, length>::RingBuffer(){
    this->tail = this->head = 0;
}

template<class kind, int length>  int RingBuffer<kind, length>::capacity(){
    return length-1;
}

template<class kind, int length>  int RingBuffer<kind, length>::size(){
	//return((this->tail>this->head)?length:0)+this->head-tail;
	__disable_irq();
	int i = head - tail + ((tail > head)?length:0);
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
    this->buffer[this->head] = object;
    this->head = (head+1)&(length-1);
}

template<class kind, int length> kind* RingBuffer<kind, length>::get_head_ref(){
    return &(buffer[head]);
}

template<class kind, int length> kind* RingBuffer<kind, length>::get_tail_ref(){
    return &(buffer[tail]);
}

template<class kind, int length> void RingBuffer<kind, length>::get(int index, kind &object){
    int j= 0;
    int k= this->tail;
    while (k != this->head){
        if (j == index) break;
        j++;
        k= (k + 1) & (length - 1);
    }
    // TODO : this checks wether we are asked a value out of range
    //if (k == this->head){
    //    return NULL;
    //}
    object = this->buffer[k];
}


template<class kind, int length> kind* RingBuffer<kind, length>::get_ref(int index){
    int j= 0;
    int k= this->tail;
    while (k != this->head){
        if (j == index) break;
        j++;
        k= (k + 1) & (length - 1);
    }
    // TODO : this checks wether we are asked a value out of range
    if (k == this->head){
        return 0;
    }
    return &(this->buffer[k]);
}

template<class kind, int length> void RingBuffer<kind, length>::pop_front(kind &object){
    object = this->buffer[this->tail];
    this->tail = (this->tail+1)&(length-1);
}

template<class kind, int length> void RingBuffer<kind, length>::delete_tail(){
    //kind dummy;
    //this->pop_front(dummy);
    this->tail = (this->tail+1)&(length-1);
}


#endif
