#include "CallbackStream.h"
#include "Kernel.h"
#include <stdio.h>

#include "SerialConsole.h"
#define DEBUG_PRINTF THEKERNEL->serial->printf

CallbackStream::CallbackStream(cb_t cb, void *u)
{
    DEBUG_PRINTF("Callbackstream ctor: %p\n", this);
    callback= cb;
    user= u;
    closed= false;
    use_count= 0;
}

CallbackStream::~CallbackStream()
{
    DEBUG_PRINTF("Callbackstream dtor: %p\n", this);
}

int CallbackStream::puts(const char *s)
{
    if(closed) return 0;

    if(s == NULL) return (*callback)(NULL, user);

    int len = strlen(s);
    int n;
    do {
        // call this streams result callback
        n= (*callback)(s, user);

        // if closed just pretend we sent it
        if(n == -1) {
            closed= true;
            return len;

        }else if(n == 0) {
            // if output queue is full
            // call idle until we can output more
            THEKERNEL->call_event(ON_IDLE);
        }
    } while(n == 0);

    return len;
}

void CallbackStream::mark_closed()
{
    closed= true;
    if(use_count <= 0) delete this;
}
void CallbackStream::dec()
{
    use_count--;
    if(closed && use_count <= 0) delete this;
}

extern "C" void *new_callback_stream(cb_t cb, void *u)
{
    return new CallbackStream(cb, u);
}

extern "C" void delete_callback_stream(void *p)
{
    // we don't delete it in case it is still on the command queue
    ((CallbackStream*)p)->mark_closed();
}
