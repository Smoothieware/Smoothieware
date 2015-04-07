/* mbed Microcontroller Library - FATFileHandle
 * Copyright (c) 2008, sford
 */

#include "FATFileHandle.h"

#include <stdio.h>
#include <stdlib.h>
#include "ff.h"
#include "FATFileSystem.h"

namespace mbed {

#if FFSDEBUG_ENABLED
static const char *FR_ERRORS[] = {
    "FR_OK = 0",
    "FR_NOT_READY",
    "FR_NO_FILE",
    "FR_NO_PATH",
    "FR_INVALID_NAME",
    "FR_INVALID_DRIVE",
    "FR_DENIED",
    "FR_EXIST",
    "FR_RW_ERROR",
    "FR_WRITE_PROTECTED",
    "FR_NOT_ENABLED",
    "FR_NO_FILESYSTEM",
    "FR_INVALID_OBJECT",
    "FR_MKFS_ABORTED"
};
#endif

FATFileHandle::FATFileHandle(FIL_t fh) {
    _fh = fh;
}
    
int FATFileHandle::close() {
    FFSDEBUG("close\n");
    int retval = f_close(&_fh);
    delete this;
    return retval;
}

ssize_t FATFileHandle::write(const void* buffer, size_t length) {
    FFSDEBUG("write(%d)\n", length);
    UINT n;
    FRESULT res = f_write(&_fh, buffer, length, &n);
    if(res) {
        FFSDEBUG("f_write() failed (%d, %s)", res, FR_ERRORS[res]);
        return -1;
    }
    return n;
}
        
ssize_t FATFileHandle::read(void* buffer, size_t length) {
    FFSDEBUG("read(%d)\n", length);
    UINT n;
    FRESULT res = f_read(&_fh, buffer, length, &n);
    if(res) {
        FFSDEBUG("f_read() failed (%d, %s)\n", res, FR_ERRORS[res]);
        return -1;
    }
    return n;
}
        
int FATFileHandle::isatty() {
    return 0;
}
        
off_t FATFileHandle::lseek(off_t position, int whence) {
    FFSDEBUG("lseek(%i,%i)\n",position,whence);
    if(whence == SEEK_END) {
        position += _fh.fsize;
    } else if(whence==SEEK_CUR) {
        position += _fh.fptr;
    }
    FRESULT res = f_lseek(&_fh, position);
    if(res) {
        FFSDEBUG("lseek failed (%d, %s)\n", res, FR_ERRORS[res]);
        return -1;
    } else {
        FFSDEBUG("lseek OK, returning %i\n", _fh.fptr);
        return _fh.fptr;
    }
}
        
int FATFileHandle::fsync() {
    FFSDEBUG("fsync()\n");
    FRESULT res = f_sync(&_fh);
    if (res) {
        FFSDEBUG("f_sync() failed (%d, %s)\n", res, FR_ERRORS[res]);
        return -1;
    }
    return 0;
}

off_t FATFileHandle::flen() {
    FFSDEBUG("flen\n");
    return _fh.fsize;
}

} // namespace mbed
