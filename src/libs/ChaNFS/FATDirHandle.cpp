/* mbed Microcontroller Library - FATDirHandle
 * Copyright (c) 2008, sford
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ff.h"
#include "FATDirHandle.h"
#include "FATFileSystem.h"

namespace mbed {

FATDirHandle::FATDirHandle(const DIR_t &the_dir) {
    dir = the_dir;
}

int FATDirHandle::closedir() {
    delete this;
    return 0;
}

struct dirent *FATDirHandle::readdir() {
    FILINFO finfo;
#if _USE_LFN
    static char lfn[_MAX_LFN * (_LFN_UNICODE ? 2 : 1) + 1];
    finfo.lfname = lfn;
    finfo.lfsize = sizeof(lfn);
#endif
    FRESULT res = f_readdir(&dir, &finfo);
    if(res != 0 || finfo.fname[0]==0) {
        return NULL;
    } else {
        char* fn;
        int stringSize = 0;
#if _USE_LFN
        fn = *finfo.lfname ? finfo.lfname : finfo.fname;
        stringSize = *finfo.lfname ? finfo.lfsize : sizeof(finfo.fname);
#else
        fn = fno.fname;
        stringSize =  sizeof(finfo.fname);
#endif
        memcpy(cur_entry.d_name, fn, stringSize);
        cur_entry.d_isdir= (finfo.fattrib & AM_DIR);
        cur_entry.d_fsize= finfo.fsize;
        return &cur_entry;
    }
}

void FATDirHandle::rewinddir() {
    dir.index = 0;
}

off_t FATDirHandle::telldir() {
    return dir.index;
}

void FATDirHandle::seekdir(off_t location) {
    dir.index = location;
}

}

