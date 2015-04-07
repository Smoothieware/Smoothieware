#include "StreamOutput.h"

NullStreamOutput StreamOutput::NullStream;

int StreamOutput::printf(const char *format, ...)
{
    char b[64];
    char *buffer;
    // Make the message
    va_list args;
    va_start(args, format);

    int size = vsnprintf(b, 64, format, args) + 1; // we add one to take into account space for the terminating \0

    if (size < 64) {
        buffer = b;
    } else {
        buffer = new char[size];
        vsnprintf(buffer, size, format, args);
    }
    va_end(args);

    puts(buffer);

    if (buffer != b)
        delete[] buffer;

    return size - 1;
}
