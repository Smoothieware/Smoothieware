/* mbed Microcontroller Library - FATFileSystem
 * Copyright (c) 2008, sford
 */

#include "FATFileSystem.h"

#include "mbed.h"

#include "FileSystemLike.h"
#include "FATFileHandle.h"
#include "FATDirHandle.h"
#include "ff.h"
//#include "Debug.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

DWORD get_fattime (void) {
    return 999;
}

// Longest path built on the stack (as before, "<drive>:/" plus a short name). Longer paths go to the heap.
#define FATFS_PATH_STACK 64
// Sanity cap so a corrupt or hostile name cannot eat the heap: "<drive>:/" plus a long file name (_MAX_LFN is 255) and some directories
#define FATFS_PATH_LIMIT 1024

namespace mbed {

// FatFs uses drive 0 when the path has no "<digit>:" prefix (_FS_RPATH is 0), so for drive 0 the name can be handed over as is.
// A name that itself starts with "<digit>:" would be read as a drive number, so that one still gets the prefix (and fails as before).
static bool needs_prefix(int fsid, const char *name) {
    return fsid != 0 || (name[0] >= '0' && name[0] <= '9' && name[1] == ':');
}

// Calls fn with the path to give to FatFs and returns its result, or NULL if the path cannot be built.
// The buffer lives in this function only, so the common case (drive 0) uses no stack for the path at all.
template <typename F>
static __attribute__((noinline)) void *with_prefixed_path(int fsid, const char *name, F fn) {
    char small[FATFS_PATH_STACK];
    size_t need = strlen(name) + 4; // "N:/" and the terminator
    if(need > FATFS_PATH_LIMIT) return NULL;
    char *buf = small;
    if(need > sizeof(small)) {
        buf = (char *)malloc(need);
        if(buf == NULL) return NULL;
    }
    snprintf(buf, need, "%d:/%s", fsid, name);
    void *r = fn(buf);
    if(buf != small) free(buf);
    return r;
}

template <typename F>
static void *with_path(int fsid, const char *name, F fn) {
    if(needs_prefix(fsid, name)) return with_prefixed_path(fsid, name, fn);
    return fn(name[0] == '\0' ? "/" : name);
}

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

FATFileSystem *FATFileSystem::_ffs[_DRIVES] = {0};

FATFileSystem::FATFileSystem(const char* n) : FileSystemLike(n) {
    FFSDEBUG("FATFileSystem(%s)\n", n);
    for(int i=0; i<_DRIVES; i++) {
        if(_ffs[i] == 0) {
            _ffs[i] = this;
            _fsid = i;
            FFSDEBUG("Mounting [%s] on ffs drive [%d]\n", _name, _fsid);
            f_mount(i, &_fs);
            return;
        }
    }
    error("Couldn't create %s in FATFileSystem::FATFileSystem\n",n);
}

FATFileSystem::~FATFileSystem() {
    for(int i=0; i<_DRIVES; i++) {
        if(_ffs[i] == this) {
            _ffs[i] = 0;
            f_mount(i, NULL);
        }
    }
}

FileHandle *FATFileSystem::open(const char* name, int flags) {
    FFSDEBUG("open(%s) on filesystem [%s], drv [%d]\n", name, _name, _fsid);
    /* POSIX flags -> FatFS open mode */
    BYTE openmode;
    if(flags & O_RDWR) {
        openmode = FA_READ|FA_WRITE;
    } else if(flags & O_WRONLY) {
        openmode = FA_WRITE;
    } else {
        openmode = FA_READ;
    }
    if(flags & O_CREAT) {
        if(flags & O_TRUNC) {
            openmode |= FA_CREATE_ALWAYS;
        } else {
            openmode |= FA_OPEN_ALWAYS;
        }
    }

    return (FileHandle *)with_path(_fsid, name, [=](const char *n) -> void * {
        FIL_t fh;
        FRESULT res = f_open(&fh, n, openmode);
        if(res) {
            FFSDEBUG("f_open('w') failed (%d, %s)\n", res, FR_ERRORS[res]);
            return NULL;
        }
        if(flags & O_APPEND) {
            f_lseek(&fh, fh.fsize);
        }
        return new FATFileHandle(fh);
    });
}

int FATFileSystem::remove(const char *filename) {
    FRESULT res = f_unlink(filename);
    if(res) {
        FFSDEBUG("f_unlink() failed (%d, %s)\n", res, FR_ERRORS[res]);
        return -1;
    }
    return 0;
}

int FATFileSystem::rename(const char *filename1, const char *filename2) {
    FRESULT res = f_rename(filename1, filename2);
    if(res) {
        FFSDEBUG("f_rename() failed (%d, %s)\n", res, FR_ERRORS[res]);
        return -1;
    }
    return 0;
}

int FATFileSystem::format() {
    FFSDEBUG("format()\n");
    FRESULT res = f_mkfs(_fsid, 0, 512); // Logical drive number, Partitioning rule, Allocation unit size (bytes per cluster)
    if(res) {
        FFSDEBUG("f_mkfs() failed (%d, %s)\n", res, FR_ERRORS[res]);
        return -1;
    }
    return 0;
}

DirHandle *FATFileSystem::opendir(const char *name) {
    return (DirHandle *)with_path(_fsid, name, [](const char *n) -> void * {
        DIR_t dir;
        FRESULT res = f_opendir(&dir, n);
        if(res != 0) {
            return NULL;
        }
        return new FATDirHandle(dir);
    });
}

int FATFileSystem::mkdir(const char *name, mode_t mode) {
    FRESULT res = f_mkdir(name);
    return res == 0 ? 0 : -1;
}

} // namespace mbed
