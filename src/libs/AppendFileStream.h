#ifndef _APPENDFILESTREAM_H_
#define _APPENDFILESTREAM_H_

#include "Channel.h"
#include "string.h"
#include "stdlib.h"

class AppendFileStream : public Channel {
    public:
        AppendFileStream(const char *filename) { fn= strdup(filename); }
        virtual ~AppendFileStream(){ free(fn); }
        int puts(const char*);

    private:
        char *fn;
};

#endif
