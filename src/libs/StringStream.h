#ifndef _STRINGSTREAM_H_
#define _STRINGSTREAM_H_

#include "StreamOutput.h"

#include <string>

class StringStream : public StreamOutput {
    public:
        StringStream() {}
        int puts(const char *str){ output.append(str); return strlen(str); }
        void clear() { output.clear(); }
        std::string getOutput() const { return output; }

    private:
        std::string output;
};

#endif
