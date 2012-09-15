/* mbed Microcontroller Library - FileLike
 * Copyright (c) 2008-2009 ARM Limited. All rights reserved.
 */ 
 
#ifndef MBED_FILELIKE_H
#define MBED_FILELIKE_H

#include "Base.h"
#include "FileHandle.h"

namespace mbed {

/* Class FileLike
 *  A file-like object is one that can be opened with fopen by
 *  fopen("/name", mode). It is intersection of the classes Base and
 *  FileHandle.
 */ 
class FileLike : public Base, public FileHandle {
    
 public:
    /* Constructor FileLike
     *
     * Variables
     *  name - The name to use to open the file.
     */
    FileLike(const char *name) : Base(name) { }
    virtual ~FileLike();

};

} // namespace mbed

#endif
