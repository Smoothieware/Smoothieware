#include "MemoryPool.h"

#include "StreamOutput.h"

#include <mri.h>
#include <cstdio>

#define offset(x) (((uint8_t*) x) - ((uint8_t*) this->base))

typedef struct __attribute__ ((packed))
{
    uint32_t next :31;
    uint32_t used :1;

    uint8_t data[];
} _poolregion;

MemoryPool* MemoryPool::first = NULL;

MemoryPool::MemoryPool(void* base, uint16_t size)
{
    this->base = base;
    this->size = size;

    ((_poolregion*) base)->used = 0;
    ((_poolregion*) base)->next = size;

    // insert ourselves into head of LL
    next = first;
    first = this;
}

MemoryPool::~MemoryPool()
{
    MDEBUG("Pool %p destroyed: region %p (%d)\n", this, base, size);

    // remove ourselves from the LL
    if (first == this)
    {   // special case: we're first
        first = this->next;
        return;
    }

    // otherwise search the LL for the previous pool
    MemoryPool* m = first;
    while (m)
    {
        if (m->next == this)
        {
            m->next = next;
            return;
        }
        m = m->next;
    }
}

void* MemoryPool::alloc(size_t nbytes)
{
    // nbytes = ceil(nbytes / 4) * 4
    if (nbytes & 3)
        nbytes += 4 - (nbytes & 3);

    // start at the start
    _poolregion* p = ((_poolregion*) base);

    // find the allocation size including our metadata
    uint16_t nsize = nbytes + sizeof(_poolregion);

    MDEBUG("\tallocate %d bytes from %p\n", nsize, base);

    // now we walk the list, looking for a sufficiently large free block
    do {
        MDEBUG("\t\tchecking %p (%s, %db)\n", p, (p->used?"used":"free"), p->next);
        if ((p->used == 0) && (p->next >= nsize))
        {   // we found a free space that's big enough
            MDEBUG("\t\tFOUND free block at %p (%+d) with %d bytes\n", p, offset(p), p->next);
            // mark it as used
            p->used = 1;

            // if there's free space at the end of this block
            if (p->next > nsize)
            {
                // q = p->next
                _poolregion* q = (_poolregion*) (((uint8_t*) p) + nsize);

                MDEBUG("\t\twriting header to %p (%+d) (%d)\n", q, offset(q), p->next - nsize);
                // write a new block header into q
                q->used = 0;
                q->next = p->next - nsize;

                // set our next to point to it
                p->next = nsize;

                // sanity check
                if (offset(q) >= size)
                {
                    // captain, we have a problem!
                    // this can only happen if something has corrupted our heap, since we should simply fail to find a free block if it's full
                    __debugbreak();
                }
            }

            // then return the data region for the block
            return &p->data;
        }

        // p = p->next
        p = (_poolregion*) (((uint8_t*) p) + p->next);

        // make sure we don't walk off the end
    } while (p <= (_poolregion*) (((uint8_t*)base) + size));

    // fell off the end of the region!
    return NULL;
}

void MemoryPool::dealloc(void* d)
{
    _poolregion* p = (_poolregion*) (((uint8_t*) d) - sizeof(_poolregion));
    p->used = 0;

    MDEBUG("\tdeallocating %p (%+d, %db)\n", p, offset(p), p->next);

    // combine next block if it's free
    _poolregion* q = (_poolregion*) (((uint8_t*) p) + p->next);
    if (q->used == 0)
    {
        MDEBUG("\t\tCombining with next free region at %p, new size is %d\n", q, p->next + q->next);

        // sanity check
        if (offset(q) > size)
        {
            // captain, we have a problem!
            // this can only happen if something has corrupted our heap, since we should simply fail to find a free block if it's full
            __debugbreak();
        }

        p->next += q->next;
    }

    // walk the list to find previous block
    q = (_poolregion*) base;
    do {
        // check if q is the previous block
        if ((((uint8_t*) q) + q->next) == (uint8_t*) p) {
            // q is the previous block.
            if (q->used == 0)
            { // if q is free
                MDEBUG("\t\tCombining with previous free region at %p, new size is %d\n", q, p->next + q->next);

                // combine!
                q->next += p->next;

                // sanity check
                if ((offset(p) + p->next) >= size)
                {
                    // captain, we have a problem!
                    // this can only happen if something has corrupted our heap, since we should simply fail to find a free block if it's full
                    __debugbreak();
                }
            }

            // we found previous block, return
            return;
        }

        // return if last block
        if (offset(q) + q->next >= size)
            return;

        // q = q->next
        q = (_poolregion*) (((uint8_t*) q) + q->next);

        // if some idiot deallocates our memory region while we're using it, strange things can happen.
        // avoid an infinite loop in that case, however we'll still leak memory and may corrupt things
        if (q->next == 0)
            return;

        // make sure we don't walk off the end
    } while (q < (_poolregion*) (((uint8_t*) base) + size));
}

void MemoryPool::debug(StreamOutput* str)
{
    _poolregion* p = (_poolregion*) base;
    uint32_t tot = 0;
    uint32_t free = 0;
    str->printf("Start: %ub MemoryPool at %p\n", size, p);
    do {
        str->printf("\tChunk at %p (%+4d): %s, %lu bytes\n", p, offset(p), (p->used?"used":"free"), p->next);
        tot += p->next;
        if (p->used == 0)
            free += p->next;
        if ((offset(p) + p->next >= size) || (p->next <= sizeof(_poolregion)))
        {
            str->printf("End: total %lub, free: %lub\n", tot, free);
            return;
        }
        p = (_poolregion*) (((uint8_t*) p) + p->next);
    } while (1);
}

bool MemoryPool::has(void* p)
{
    return ((p >= base) && (p < (void*) (((uint8_t*) base) + size)));
}

uint32_t MemoryPool::free()
{
    uint32_t free = 0;

    _poolregion* p = (_poolregion*) base;

    do {
        if (p->used == 0)
            free += p->next;
        if (offset(p) + p->next >= size)
            return free;
        if (p->next <= sizeof(_poolregion))
            return free;
        p = (_poolregion*) (((uint8_t*) p) + p->next);
    } while (1);
}
