/* Copyright 2012 Adam Green (http://mbed.org/users/AdamGreen/)

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU Lesser General Public License as published
   by the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU Lesser General Public License for more details.

   You should have received a copy of the GNU Lesser General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.   
*/
/* Provide routines which hook the MRI debug monitor into GCC4MBED projects. */
#include <string.h>
#include <sys/types.h>
#include <errno.h>
#include <mri.h>
#include <cmsis.h>

extern unsigned int __bss_start__;
extern unsigned int __bss_end__;
extern "C" int  main(void);
extern "C" void __libc_init_array(void);
extern "C" void exit(int ErrorCode);
extern "C" void _start(void)
{
    int bssSize = (int)&__bss_end__ - (int)&__bss_start__;
    int mainReturnValue;
    
    memset(&__bss_start__, 0, bssSize);
    
    if (MRI_ENABLE)
    {
        __mriInit(MRI_INIT_PARAMETERS);
        if (MRI_BREAK_ON_INIT)
            __debugbreak();
    }
    
    __libc_init_array();
    mainReturnValue = main();
    exit(mainReturnValue);
}


extern "C" int __real__read(int file, char *ptr, int len);
extern "C" int __wrap__read(int file, char *ptr, int len)
{
    if (MRI_SEMIHOST_STDIO && file < 3)
        return __mriNewlib_SemihostRead(file, ptr, len);
     return __real__read(file, ptr, len);
}


extern "C" int __real__write(int file, char *ptr, int len);
extern "C" int __wrap__write(int file, char *ptr, int len)
{
    if (MRI_SEMIHOST_STDIO && file < 3)
        return __mriNewlib_SemihostWrite(file, ptr, len);
    return __real__write(file, ptr, len);
}


extern "C" int __real__isatty(int file);
extern "C" int __wrap__isatty(int file)
{
    /* Hardcoding the stdin/stdout/stderr handles to be interactive tty devices, unlike mbed.ar */
    if (file < 3)
        return 1;
    return __real__isatty(file);
}


extern "C" int __wrap_semihost_connected(void)
{
    /* MRI makes it look like there is no mbed interface attached since it disables the JTAG portion but MRI does
       support some of the mbed semihost calls when it is running so force it to return -1, indicating that the
       interface is attached. */
    return -1;
}



extern "C" void abort(void)
{
    if (MRI_ENABLE)
        __debugbreak();
        
    exit(1);
}


extern "C" void __cxa_pure_virtual(void)
{
    abort();
}


extern "C" int __aeabi_unwind_cpp_pr0(int state, void* controlBlock, void* context)
{
    abort();
}


extern "C" int __aeabi_unwind_cpp_pr1(int state, void* controlBlock, void* context)
{
    abort();
}


extern "C" int __aeabi_unwind_cpp_pr2(int state, void* controlBlock, void* context)
{
    abort();
}

/* Trap calls to malloc/free/realloc in ISR. */
extern "C" void __malloc_lock(void)
{
    if (__get_IPSR() != 0)
        __debugbreak();
}

extern "C" void __malloc_unlock(void)
{
}


/* Linker defined symbol to be used by sbrk for where dynamically heap should start. */
extern "C" int __HeapBase;

/* Turn off the errno macro and use actual external global variable instead. */
#undef errno
extern int errno;

/* Dynamic memory allocation related syscalls. */
extern "C" caddr_t _sbrk(int incr) 
{
    static unsigned char* heap = (unsigned char*)&__HeapBase;
    unsigned char*        prev_heap = heap;
    unsigned char*        new_heap = heap + incr;

    if (new_heap >= (unsigned char*)__get_MSP()) {
        errno = ENOMEM;
        return (caddr_t)-1;
    }
    
    heap = new_heap;
    return (caddr_t) prev_heap;
}
