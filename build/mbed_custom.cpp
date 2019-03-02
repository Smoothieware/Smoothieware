/* Copyright 2013 Adam Green (http://mbed.org/users/AdamGreen/)

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
/* Provide routines which hook the MRI debug monitor into GCC4MBED projects. */
#include <string.h>
#include <sys/types.h>
#include <errno.h>
#include <mri.h>
#include <cmsis.h>
#include "mpu.h"

#include "platform_memory.h"

unsigned int g_maximumHeapAddress;

static void fillUnusedRAM(void);
static void configureStackSizeLimit(unsigned int stackSizeLimit);
static unsigned int alignTo32Bytes(unsigned int value);
static void configureMpuToCatchStackOverflowIntoHeap(unsigned int maximumHeapAddress);
static void configureMpuRegionToAccessAllMemoryWithNoCaching(void);


/* Symbols exposed from the linker script. */
extern unsigned int     __bss_start__;
extern unsigned int     __bss_end__;
extern unsigned int     __StackTop;
extern "C" unsigned int __end__;

extern "C" int  main(void);
extern "C" void __libc_init_array(void);
// extern "C" void exit(int ErrorCode);
extern "C" void _start(void)
{
    int bssSize = (int)&__bss_end__ - (int)&__bss_start__;
    int mainReturnValue;

    memset(&__bss_start__, 0, bssSize);
    fillUnusedRAM();

    if (STACK_SIZE) {
        configureStackSizeLimit(STACK_SIZE);
    }
    if (WRITE_BUFFER_DISABLE) {
        disableMPU();
        configureMpuRegionToAccessAllMemoryWithNoCaching();
        enableMPU();
    }
    if (MRI_ENABLE) {
        __mriInit(MRI_INIT_PARAMETERS);
        if (MRI_BREAK_ON_INIT)
            __debugbreak();
    }


    // MemoryPool stuff - needs to be initialised before __libc_init_array
    // so static ctors can use them
    extern uint8_t __AHB0_block_start;
    extern uint8_t __AHB0_dyn_start;
    extern uint8_t __AHB0_end;
    extern uint8_t __AHB1_block_start;
    extern uint8_t __AHB1_dyn_start;
    extern uint8_t __AHB1_end;

    // zero the data sections in AHB0 and AHB1
    memset(&__AHB0_block_start, 0, &__AHB0_dyn_start - &__AHB0_block_start);
    memset(&__AHB1_block_start, 0, &__AHB1_dyn_start - &__AHB1_block_start);

    MemoryPool _AHB0_stack(&__AHB0_dyn_start, &__AHB0_end - &__AHB0_dyn_start);
    MemoryPool _AHB1_stack(&__AHB1_dyn_start, &__AHB1_end - &__AHB1_dyn_start);


    _AHB0 = &_AHB0_stack;
    _AHB1 = &_AHB1_stack;
    // MemoryPool init done

    __libc_init_array();
    mainReturnValue = main();
    exit(mainReturnValue);
}

static __attribute__((naked)) void fillUnusedRAM(void)
{
    __asm (
        ".syntax unified\n"
        ".thumb\n"
        // Fill 2 words (8 bytes) at a time with 0xdeadbeef.
        " ldr   r2, =__FillStart\n"
        " movw  r0, #0xbeef\n"
        " movt  r0, #0xdead\n"
        " mov   r1, r0\n"
        // Don't fill past current stack pointer value.
        " mov   r3, sp\n"
        " bics  r3, r3, #7\n"
        "1$:\n"
        " strd  r0, r1, [r2], #8\n"
        " cmp   r2, r3\n"
        " blo   1$\n"
        " bx    lr\n"
    );
}

static void configureStackSizeLimit(unsigned int stackSizeLimit)
{
    // Note: 32 bytes are reserved to fall between top of heap and top of stack for minimum MPU guard region.
    g_maximumHeapAddress = alignTo32Bytes((unsigned int)&__StackTop - stackSizeLimit - 32);
    configureMpuToCatchStackOverflowIntoHeap(g_maximumHeapAddress);
}

static unsigned int alignTo32Bytes(unsigned int value)
{
    return (value + 31) & ~31;
}

static void configureMpuToCatchStackOverflowIntoHeap(unsigned int maximumHeapAddress)
{
#define MPU_REGION_SIZE_OF_32_BYTES ((5-1) << MPU_RASR_SIZE_SHIFT)  // 2^5 = 32 bytes.

    prepareToAccessMPURegion(getHighestMPUDataRegionIndex());
    setMPURegionAddress(maximumHeapAddress);
    setMPURegionAttributeAndSize(MPU_REGION_SIZE_OF_32_BYTES | MPU_RASR_ENABLE);
    enableMPUWithDefaultMemoryMap();
}

static void configureMpuRegionToAccessAllMemoryWithNoCaching(void)
{
    static const uint32_t regionToStartAtAddress0 = 0U;
    static const uint32_t regionReadWrite = 1  << MPU_RASR_AP_SHIFT;
    static const uint32_t regionSizeAt4GB = 31 << MPU_RASR_SIZE_SHIFT; /* 4GB = 2^(31+1) */
    static const uint32_t regionEnable    = MPU_RASR_ENABLE;
    static const uint32_t regionSizeAndAttributes = regionReadWrite | regionSizeAt4GB | regionEnable;
    uint32_t regionIndex = STACK_SIZE ? getHighestMPUDataRegionIndex() - 1 : getHighestMPUDataRegionIndex();

    prepareToAccessMPURegion(regionIndex);
    setMPURegionAddress(regionToStartAtAddress0);
    setMPURegionAttributeAndSize(regionSizeAndAttributes);
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


/* Trap calls to malloc/free/realloc in ISR. */
extern "C" void __malloc_lock(void)
{
    if (__get_IPSR() != 0)
        __debugbreak();
}

extern "C" void __malloc_unlock(void)
{
}


/* Turn off the errno macro and use actual external global variable instead. */
#undef errno
extern int errno;

static int doesHeapCollideWithStack(unsigned int newHeap);

/* Dynamic memory allocation related syscalls. */
extern "C" caddr_t _sbrk(int incr)
{
    static unsigned char *heap = (unsigned char *)&__end__;
    unsigned char        *prev_heap = heap;
    unsigned char        *new_heap = heap + incr;

    if (doesHeapCollideWithStack((unsigned int)new_heap)) {
        errno = ENOMEM;
        return (caddr_t) - 1;
    }

    heap = new_heap;
    return (caddr_t) prev_heap;
}

static int doesHeapCollideWithStack(unsigned int newHeap)
{
    return ((newHeap >= __get_MSP()) ||
            (STACK_SIZE && newHeap >= g_maximumHeapAddress));
}


/* Optional functionality which will tag each heap allocation with the caller's return address. */
#ifdef HEAP_TAGS

const unsigned int *__smoothieHeapBase = &__end__;

extern "C" void *__real_malloc(size_t size);
extern "C" void *__real_realloc(void *ptr, size_t size);
extern "C" void  __real_free(void *ptr);

static void setTag(void *pv, unsigned int tag);
static unsigned int *footerForChunk(void *pv);
static unsigned int *headerForChunk(void *pv);
static unsigned int sizeOfChunk(unsigned int *pHeader);
static int isChunkInUse(void *pv);

extern "C" __attribute__((naked)) void __wrap_malloc(size_t size)
{
    __asm (
        ".syntax unified\n"
        ".thumb\n"
        "mov r1,lr\n"
        "b mallocWithTag\n"
    );
}

extern "C" void *mallocWithTag(size_t size, unsigned int tag)
{
    void *p = __real_malloc(size + sizeof(tag));
    if (!p && __smoothieHeapBase)
        return p;
    setTag(p, tag);
    return p;
}

static void setTag(void *pv, unsigned int tag)
{
    unsigned int *pFooter = footerForChunk(pv);
    *pFooter = tag;
}

static unsigned int *footerForChunk(void *pv)
{
    unsigned int *pHeader = headerForChunk(pv);
    unsigned int  size = sizeOfChunk(pHeader);
    return (unsigned int *)(void *)((char *)pHeader + size);
}

static unsigned int *headerForChunk(void *pv)
{
    // Header is allocated two words (8 bytes) before the publicly returned allocation chunk address.
    unsigned int *p = (unsigned int *)pv;
    return &p[-2];
}

static unsigned int sizeOfChunk(unsigned int *pHeader)
{
    /* Remove previous chunk in use flag. */
    return pHeader[1] & ~1;
}

extern "C" __attribute__((naked)) void __wrap_realloc(void *ptr, size_t size)
{
    __asm (
        ".syntax unified\n"
        ".thumb\n"
        "mov r2,lr\n"
        "b reallocWithTag\n"
    );
}

extern "C" void *reallocWithTag(void *ptr, size_t size, unsigned int tag)
{
    void *p = __real_realloc(ptr, size + sizeof(tag));
    if (!p)
        return p;
    setTag(p, tag);
    return p;
}

extern "C" void __wrap_free(void *ptr)
{
    if (!isChunkInUse(ptr))
        __debugbreak();
    __real_free(ptr);
}

static int isChunkInUse(void *pv)
{
    unsigned int *pFooter = footerForChunk(pv);
    return pFooter[1] & 1;
}

__attribute__((naked)) void *operator new(size_t size)
{
    __asm (
        ".syntax unified\n"
        ".thumb\n"
        "push {r4,lr}\n"
        "mov r1,lr\n"
        "bl mallocWithTag\n"
        "cbnz r0, 1$\n"
        "bl abort\n"
        "1$:\n"
        "pop {r4,pc}\n"
    );
    // This line never executes but silences no return value warning from compiler.
    return (void *)1;
}

#else

/* Wrap memory allocation routines to make sure that they aren't being called from interrupt handler. */
static void breakOnHeapOpFromInterruptHandler(void)
{
    if (__get_IPSR() != 0)
        __debugbreak();
}

extern "C" void *__real_malloc(size_t size);
extern "C" void *__wrap_malloc(size_t size)
{
    breakOnHeapOpFromInterruptHandler();
    return __real_malloc(size);
}


extern "C" void *__real_realloc(void *ptr, size_t size);
extern "C" void *__wrap_realloc(void *ptr, size_t size)
{
    breakOnHeapOpFromInterruptHandler();
    return __real_realloc(ptr, size);
}


extern "C" void __real_free(void *ptr);
extern "C" void __wrap_free(void *ptr)
{
    breakOnHeapOpFromInterruptHandler();
    __real_free(ptr);
}

#endif // HEAP_TAGS
