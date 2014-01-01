#ifndef _COMMANDQUEUE_H_
#define _COMMANDQUEUE_H_

#ifdef __cplusplus

#include "fifo.h"
#include <string>

#include "Channel.h"

class CommandQueue
{
public:
    CommandQueue();
    ~CommandQueue();
    bool pop();
    int add(const char* cmd, Channel *pstream);
    int size() {return q.size();}
    static CommandQueue* getInstance();

private:
    typedef struct {char* str; Channel *pstream; } cmd_t;
    Fifo<cmd_t> q;
    static CommandQueue *instance;
    Channel *null_stream;
};

#else

extern int network_add_command(const char * cmd, void *pstream);
#endif

#endif
