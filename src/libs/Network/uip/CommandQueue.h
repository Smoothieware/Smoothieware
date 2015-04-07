#ifndef _COMMANDQUEUE_H_
#define _COMMANDQUEUE_H_

#ifdef __cplusplus

#include "fifo.h"
#include <string>

class StreamOutput;

class CommandQueue
{
public:
    CommandQueue();
    ~CommandQueue();
    bool pop();
    int add(const char* cmd, StreamOutput *pstream);
    int size() {return q.size();}
    static CommandQueue* getInstance();

private:
    typedef struct {char* str; StreamOutput *pstream; } cmd_t;
    Fifo<cmd_t> q;
    static CommandQueue *instance;
    StreamOutput *null_stream;
};

#else

extern int network_add_command(const char * cmd, void *pstream);
#endif

#endif
