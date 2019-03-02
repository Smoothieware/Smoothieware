#include "ahbmalloc.h"

#include <cstring>

#include <cstdint>
// #include <stdio.h>

#include "platform_memory.h"

void* ahbmalloc(size_t size, BANK bank)
{
	switch(bank)
	{
		case AHB_BANK_0:
			return AHB0.alloc(size);
		case AHB_BANK_1:
			return AHB1.alloc(size);
		default:
			return NULL;
	}
}

void ahbfree(void* ptr, size_t size)
{
	MemoryPool* m = MemoryPool::first;
	while (m)
	{
		if (m->has(ptr))
		{
			m->dealloc(ptr);
			return;
		}
		m = m->next;
	}
}
