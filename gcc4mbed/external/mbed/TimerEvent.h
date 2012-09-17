/* mbed Microcontroller Library - TimerEvent
 * Copyright (c) 2007-2009 ARM Limited. All rights reserved.
 */ 
 
#ifndef MBED_TIMEREVENT_H
#define MBED_TIMEREVENT_H

namespace mbed {

// Base abstraction for timer interrupts
class TimerEvent {

public:

    TimerEvent();
    
    // The handler registered with the underlying timer interrupt
    static void irq();

    // Destruction removes it...    
    virtual ~TimerEvent();

protected:

    // The handler called to service the timer event of the derived class
    virtual void handler() = 0;
    
    // insert in to linked list
    void insert(unsigned int timestamp);
    
    // remove from linked list, if in it
    void remove();
    
    // Get the current usec timestamp
    static unsigned int timestamp();

    static TimerEvent *_head;   // The head of the list of the events, NULL if none
    TimerEvent *_next;          // Pointer to the next in the list, NULL if last
    unsigned int _timestamp;    // The timestamp at which the even should be triggered

};

} // namespace mbed

#endif
