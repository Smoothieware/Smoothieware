#ifndef _APPENDFILESTREAM_H_
#define _APPENDFILESTREAM_H_

#include "StreamOutput.h"
#include "string.h"
#include "stdlib.h"

class AppendFileStream : public StreamOutput {
    public:
        AppendFileStream(const char *filename) { fn= strdup(filename); }
        virtual ~AppendFileStream(){ free(fn); }
        int puts(const char*);

    private:
        char *fn;
};

#endif
