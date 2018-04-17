/* mbed Microcontroller Library - FATFileSystem
 * Copyright (c) 2008, sford
 */

/* Library: FATFileSystem.h
 * A library of stuff to make a fat filesystem on top of a block device
 */

#ifndef MBED_FATFILESYSTEM_H
#define MBED_FATFILESYSTEM_H

#ifndef FFSDEBUG_ENABLED
#define FFSDEBUG_ENABLED 0
#endif

#if FFSDEBUG_ENABLED
#define FFSDEBUG(FMT, ...) printf(FMT, ##__VA_ARGS__)
#else
#define FFSDEBUG(FMT, ...)
#endif

#include "FileSystemLike.h"
#include "FileHandle.h"
#include "ff.h"
#include "diskio.h"

namespace mbed {
/* Class: FATFileSystem
 * The class itself
 */
class FATFileSystem : public FileSystemLike {
public:

    FATFileSystem(const char* n);
    virtual ~FATFileSystem();

    /* Function: open
       * open a file on the filesystem. never called directly
       */
    virtual FileHandle *open(const char* name, int flags);
    virtual int remove(const char *filename);
    virtual int rename(const char *filename1, const char *filename2);
    virtual int format();
    virtual DirHandle *opendir(const char *name);
    virtual int mkdir(const char *name, mode_t mode);

    FATFS _fs;                                // Work area (file system object) for logical drive
    static FATFileSystem *_ffs[_DRIVES];    // FATFileSystem objects, as parallel to FatFs drives array
    int _fsid;

    virtual int disk_initialize() { return 0; }
    virtual int disk_status() { return 0; }
    virtual int disk_read(char *buffer, int sector) = 0;
    virtual int disk_write(const char *buffer, int sector) = 0;
    virtual int disk_sync() { return 0; }
    virtual int disk_sectors() = 0;

};

}

#endif
