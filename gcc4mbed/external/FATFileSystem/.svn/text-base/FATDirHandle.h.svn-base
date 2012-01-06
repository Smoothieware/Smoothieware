/* Copyright 2008 ARM Limited. All rights reserved. */

#ifndef MBED_FATDIRHANDLE_H
#define MBED_FATDIRHANDLE_H

#include "DirHandle.h"
#include "ff.h"

namespace mbed {

class FATDirHandle : public DirHandle {

 public:
    FATDirHandle(const FATFS_DIR &the_dir);
    virtual int closedir();
    virtual struct dirent *readdir();
    virtual void rewinddir();
    virtual off_t telldir();
    virtual void seekdir(off_t location);

 private:
    FATFS_DIR dir;
    struct dirent cur_entry;

};

}

#endif
