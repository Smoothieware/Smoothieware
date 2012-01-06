/* mbed Microcontroller Library - DirHandler
 * Copyright (c) 2008-2009 ARM Limited. All rights reserved.
 * sford
 */ 
 
#ifndef MBED_DIRHANDLE_H
#define MBED_DIRHANDLE_H

#ifdef __ARMCC_VERSION
# define NAME_MAX 255
typedef int mode_t;
#else
# include <sys/syslimits.h>
#endif
#include "FileHandle.h"

struct dirent {
    char d_name[NAME_MAX+1];
};

namespace mbed {

/* Class DirHandle
 *  Represents a directory stream. Objects of this type are returned
 *  by a FileSystemLike's opendir method. Implementations must define
 *  at least closedir, readdir and rewinddir.
 *
 *  If a FileSystemLike class defines the opendir method, then the
 *  directories of an object of that type can be accessed by 
 *  DIR *d = opendir("/example/directory") (or opendir("/example") 
 *  to open the root of the filesystem), and then using readdir(d) etc.
 *
 *  The root directory is considered to contain all FileLike and
 *  FileSystemLike objects, so the DIR* returned by opendir("/") will
 *  reflect this.
 */
class DirHandle {

 public:
    /* Function closedir
     *  Closes the directory.
     *
     * Variables
     *  returns - 0 on success, or -1 on error.
     */
    virtual int closedir()=0;

    /* Function readdir
     *  Return the directory entry at the current position, and
     *  advances the position to the next entry.
     *
     * Returns
     *  A pointer to a dirent structure representing the
     *  directory entry at the current position, or NULL on reaching
     *  end of directory or error.
     */
    virtual struct dirent *readdir()=0;

    /* Function rewinddir
     *  Resets the position to the beginning of the directory.
     */
    virtual void rewinddir()=0;

    /* Function telldir
     *  Returns the current position of the DirHandle.
     *
     * Returns
     *  The current position, or -1 on error.
     */
    virtual off_t telldir() { return -1; }

    /* Function seekdir
     *  Sets the position of the DirHandle.
     *
     * Variables
     *  location - The location to seek to. Must be a value returned
     *   by telldir.
     */
    virtual void seekdir(off_t location) { }

};

} // namespace mbed

typedef mbed::DirHandle DIR;

extern "C" {
    DIR *opendir(const char*);
    struct dirent *readdir(DIR *);
    int closedir(DIR*);
    void rewinddir(DIR*);
    long telldir(DIR*);
    void seekdir(DIR*, long);
    int mkdir(const char *name, mode_t n);
};

#endif /* MBED_DIRHANDLE_H */
