#ifndef _APPENDFILESTREAM_H_
#define _APPENDFILESTREAM_H_

#include <string>
#include "StreamOutput.h"

class AppendFileStream : public StreamOutput {
    public:
        AppendFileStream(const char *filename){ fn= filename; }
        int puts(const char*);

    private:
        std::string fn;
};

#endif
