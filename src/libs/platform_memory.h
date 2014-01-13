#ifndef _PLATFORM_MEMORY_H
#define _PLATFORM_MEMORY_H

#include "MemoryPool.h"

#define AHB0 (*_AHB0)
#define AHB1 (*_AHB1)

extern MemoryPool* _AHB0;
extern MemoryPool* _AHB1;

#endif /* _PLATFORM_MEMORY_H */
