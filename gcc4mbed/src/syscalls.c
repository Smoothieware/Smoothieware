/* Copyright 2012 Adam Green (http://mbed.org/users/AdamGreen/)

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
*/
/* System calls called out as overrideable in newlib documentation which weren't implemented follow in this list
   with reason:
   
    environ         Not required.  getenv() and setenv() can be used without providing an override as there is a default
    _execve         Standard C APIs implemented by newlib don't end up calling this function.  Implemented _system
                    instead.
    _fork           Standard C APIs implemented by newlib don't end up calling this function.  Implemented _system
                    instead.
    _link           This is a POSIX routine and while newlib will call it for the standard C rename() function, this
                    particular routine is overridden in mbed.ar where it just returns -1.
    _stat           This is a system API and not part of the C standard.  It doesn't appear to be required for mbed.
    _times          Appears to be a system API and not part of the C standard.  Not called from standard APIs.
    _unlink         This is a POSIX routine.  newlib will call it for standard C remove(), this particular API is
                    already overidden by mbed.ar
*/
#include <reent.h>
#include <errno.h>
#include <stdlib.h> /* abort */
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/_default_fcntl.h>
#include "LPC17xx.h" /* for _get_*SP() from core_cm3.h*/
#include "mbedsys.h" /* for _sys_*() functions implemented in mbed/capi.ar */
#include "error.h"   /* for error() panic routine */
#include "mri.h"


#undef errno
extern int errno;


/* Flag used to track whether the stdin/stdout/stderr handles have been opened yet */
static int g_StandardHandlesOpened = 0;


/* Open the stdin/stdout/stderr handles */
extern "C" void __GCC4MBEDOpenStandardHandles(void)
{
    /* Open stdin/stdout/stderr */
    _sys_open("/stdin", OPENMODE_R);
    _sys_open("/stdout", OPENMODE_W);
    _sys_open("/stderr", OPENMODE_W);
    
    g_StandardHandlesOpened = 1;
}    


extern "C" int _kill(int pid, int sig)
{
    /* Avoid warnings */
    (void)pid;
    (void)sig;

	errno = EINVAL;
	return -1;
}

extern "C" void _exit(int status)
{
    /* Call the mbed version of exit found in capi.ar */
    exit(status);
}

extern "C" int _getpid(void)
{
	return 1;
}


extern char _end; /* Defined by the linker */
static char *heap_end;

static char* get_stack_top(void)
{
	return (char*) __get_MSP();
}

extern "C" caddr_t _sbrk(int incr)
{
	char *prev_heap_end;
	if (heap_end == 0) {
		heap_end = &_end;
	}
	prev_heap_end = heap_end;
	if (heap_end + incr > get_stack_top()) {
		
		abort();
	}
	heap_end += incr;
	return (caddr_t) prev_heap_end;
}

extern "C" int _open(const char *name, int flags, int mode) 
{
    int OpenMode = 0;
    
    (void)mode;
    
    /* Convert from newlib flags to mbed openmode */
    if (flags & O_RDWR)
    {
        OpenMode |= OPENMODE_PLUS;
    }
    if (flags & O_TRUNC)
    {
        OpenMode |= OPENMODE_W;
    }
    if (flags & O_APPEND)
    {
        OpenMode |= OPENMODE_A;
    }

    return _sys_open(name, OpenMode);
}

extern "C" int _close(int file)
{
	return _sys_close(file);
}

extern "C" int _isatty(int file)
{
    /* Hardcoding the stdin/stdout/stderr handles to be interactive tty devices, unlike mbed.ar */
    if (file < 3)
    {
        return 1;
    }

    return _sys_istty(file);
}

/* TODO: Remove this later when _lseek no longer requires it */
static unsigned short _crc16(const unsigned char* pBuffer, size_t BufferSizeInBytes)
{
    unsigned short  CRC = 0;
    
    while (BufferSizeInBytes--)
    {
        unsigned short  Byte = (unsigned short)*pBuffer++;
        int             i;

        for (i = 0 ; i < 8 ; i++)
        {
            unsigned short Value = (Byte ^ CRC) & 1;
            CRC >>= 1;
            if (Value)
            {
                CRC ^= 0xA001;
            }
            Byte >>= 1;
        }
    }
    
    return ~CRC;
}
    
extern "C" int _lseek(int file, int ptr, int dir) 
{
    /* Function pointer type for the FileHandle::lseek() method */
    typedef int (*lseekMethod)(void* pThis, int ptr, int dir);
    static const unsigned short ExpectedSysSeekCRC = 0xDFC0;
    static lseekMethod***       pppFileHandles = NULL;
    lseekMethod**               ppFileHandle;
    lseekMethod*                pFileHandleVTable;
    lseekMethod                 FileHandle_lseekMethod;
    
    /* When file is set to 0, 1, or 2 then the file is actually a standard I/O stream so just return 0 since they
       don't really support seeking and this is what _sys_seek() does as well */
    if (file < 3)
    {
        return 0;
    }
    
    /* Find the location of the filehandles array if we haven't already */
    if (!pppFileHandles)
    {
        /* TODO: This code is pretty hacky but it is the only way I can get things to work with mbed.ar at this time.
                 If the mbed crew could make a small change to their library such that they exposed something equivalent
                 to _sys_lseek in addition to their _sys_seek then both this _lseek function and their _sys_seek could 
                 call it to perform seeks using any seek direction (whence) value */
        unsigned short*       pSysSeek;
        unsigned short        CRC;
    
        /* Grab a pointer to where the _sys_seek code ends up being loaded for this program */
        pSysSeek = (unsigned short*)((unsigned int)_sys_seek & ~1);
    
        /* Perform a CRC over the first 40 bytes which contain PC-relative code for the _sys_seek function to
           make sure that it hasn't changed from when I figured out the location of the pointer used for
           dereferencing into the 16 element filehandles static array from stdio.cpp */
        CRC = _crc16((unsigned char*)pSysSeek, 40);
        if (ExpectedSysSeekCRC != CRC)
        {
            error("RIP: _sys_seek() in mbed.ar has been modified\r\n"
                  "     and _lseek in gcc4mbed/syscalls.c needs to be updated\r\n");
        }
        
        /* I know that the pointer for the filehandles array is stored as a literal 20 shorts after the beginning of the 
           _sys_seek() function */
        pppFileHandles = (lseekMethod***)*(void**)(void*)(pSysSeek + 20);
    }

    
    /* Find the pointer to the FileHandle object representing this file */
    ppFileHandle = pppFileHandles[file - 3];
    if (!ppFileHandle)
    {
        /* The file handle isn't valid */
        return -1;
    }
    
    /* The vtable for this FileHandle object will be stored in the first 4-byte word of the object */
    pFileHandleVTable = *ppFileHandle;
    
    /* The fifth element of this vtable is the pointer to the lseek method */
    FileHandle_lseekMethod = pFileHandleVTable[4];
    
    /* Finally call into the file handle object's seek method */
    return FileHandle_lseekMethod(ppFileHandle, ptr, dir);
}

extern "C" int _read(int file, char *ptr, int len)
{
    int BytesNotRead;
    
    /* Open stdin/stdout/stderr if needed */
    if (MRI_SEMIHOST_STDIO && file < 3)
    {
        return __mriNewlib_SemihostRead(file, ptr, len);
    }
    else if (!g_StandardHandlesOpened && file < 3)
    {
        __GCC4MBEDOpenStandardHandles();
    }

    /* Call the function in mbed.ar and let it handle the read */
    BytesNotRead = _sys_read(file, (unsigned char*)ptr, len, 0);
    
    /* EOF is indicated by setting the most significant bit so mask it off */
    BytesNotRead &= 0x7FFFFFFF;

    /* The mbed version of the function returns bytes not read and newlib wants bytes read count */
    return len - BytesNotRead;
}

extern "C" int _write(int file, char *ptr, int len)
{
    int BytesNotWritten;
    
    /* Open stdin/stdout/stderr if needed */
    if (MRI_SEMIHOST_STDIO && file < 3)
    {
        return __mriNewlib_SemihostWrite(file, ptr, len);
    }
    else if (!g_StandardHandlesOpened && file < 3)
    {
        __GCC4MBEDOpenStandardHandles();
    }

    /* Call the function in mbed.ar and let it handle the writes */
    BytesNotWritten = _sys_write(file, (const unsigned char*)ptr, len, 0);
    
    /* The mbed version of the function returns bytes not written and newlib wants byte written count */
    if (BytesNotWritten >= 0)
    {
        return len - BytesNotWritten;
    }
    else
    {
        /* In this case it did return an error */
        return BytesNotWritten;
    }
}

extern "C" int _fstat(int file, struct stat *st)
{
	(void)file;

	st->st_mode = S_IFCHR;
	return 0;
}

extern "C" int _system(const char* pCommand)
{
    if (NULL == pCommand)
    {
        /* There is no command interpreter on the mbed */
        return 0;
    }
    else
    {
        /* Indicate that command couldn't be executed since there is no command interpreter */
        errno = ENOENT;
        return -1;
    }
}
