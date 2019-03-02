/* mbed Microcontroller Library - FATFileHandle
 * Copyright (c) 2008, sford
 */

#ifndef MBED_FATFILEHANDLE_H
#define MBED_FATFILEHANDLE_H

#include "FileHandle.h"
#include "ff.h"

namespace mbed {

class FATFileHandle : public FileHandle {
public:

    FATFileHandle(FIL_t fh);
    virtual int close();
    virtual ssize_t write(const void* buffer, size_t length);
    virtual ssize_t read(void* buffer, size_t length);
    virtual int isatty();
    virtual off_t lseek(off_t position, int whence);
    virtual int fsync();
    virtual off_t flen();

protected:

    FIL_t _fh;

};

}

#endif
