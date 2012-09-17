/* mbed Microcontroller Library - FileHandler
 * Copyright (c) 2007-2009 ARM Limited. All rights reserved.
 */ 
 
#ifndef MBED_FILEHANDLE_H
#define MBED_FILEHANDLE_H

typedef int FILEHANDLE;

#include <stdio.h>
#ifdef __ARMCC_VERSION
typedef int ssize_t;
typedef long off_t;
#else
#include <sys/types.h>
#endif

namespace mbed { 

/* Class FileHandle
 *  An OO equivalent of the internal FILEHANDLE variable
 *  and associated _sys_* functions
 *
 * FileHandle is an abstract class, needing at least sys_write and
 * sys_read to be implmented for a simple interactive device
 *
 * No one ever directly tals to/instanciates a FileHandle - it gets 
 * created by FileSystem, and wrapped up by stdio
 */
class FileHandle {

public:

    /* Function write
     *  Write the contents of a buffer to the file
     *
     * Parameters
     *  buffer - the buffer to write from
     *  length - the number of characters to write
     *
     * Returns
     *  The number of characters written (possibly 0) on success, -1 on error.
     */
    virtual ssize_t write(const void* buffer, size_t length) = 0;

    /* Function close
     *  Close the file
     *
     * Returns
     *  Zero on success, -1 on error.
     */
    virtual int close() = 0;

    /* Function read
     *  Reads the contents of the file into a buffer
     *
     * Parameters
     *  buffer - the buffer to read in to
     *  length - the number of characters to read
     *
     * Returns
     *  The number of characters read (zero at end of file) on success, -1 on error.
     */
    virtual ssize_t read(void* buffer, size_t length) = 0;

    /* Function isatty
     *  Check if the handle is for a interactive terminal device 
     *
     * If so, line buffered behaviour is used by default
     *
     * Returns
     *  1 if it is a terminal, 0 otherwise
     */
    virtual int isatty() = 0 ;

    /* Function lseek
     *  Move the file position to a given offset from a given location.
     *
     * Parameters
     *  offset - The offset from whence to move to
     *  whence - SEEK_SET for the start of the file, SEEK_CUR for the
     *   current file position, or SEEK_END for the end of the file.
     *
     * Returns
     *  New file position on success, -1 on failure or unsupported
     */
    virtual off_t lseek(off_t offset, int whence) = 0;

    /* Function fsync
     *  Flush any buffers associated with the FileHandle, ensuring it
     *  is up to date on disk
     *
     * Returns
     *  0 on success or un-needed, -1 on error
     */
    virtual int fsync() = 0;

    virtual off_t flen() {
        /* remember our current position */
        off_t pos = lseek(0, SEEK_CUR);
        if(pos == -1) return -1;
        /* seek to the end to get the file length */
        off_t res = lseek(0, SEEK_END);
        /* return to our old position */
        lseek(pos, SEEK_SET);
        return res;
    }

};

} // namespace mbed

#endif

