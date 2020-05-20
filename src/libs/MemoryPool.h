#ifndef _MEMORYPOOL_H
#define _MEMORYPOOL_H

#include <cstdint>
// #include <cstdio>
#include <cstdlib>

#ifdef MEMDEBUG
    #define MDEBUG(...) printf(__VA_ARGS__)
#else
    #define MDEBUG(...) do {} while (0)
#endif

class StreamOutput;

/*
 * with MUCH thanks to http://www.parashift.com/c++-faq-lite/memory-pools.html
 *
 * test framework at https://gist.github.com/triffid/5563987
 */

class MemoryPool
{
public:
    MemoryPool(void* base, uint16_t size);
    ~MemoryPool();

    void* alloc(size_t);
    void  dealloc(void* p);

    void  debug(StreamOutput*);

    bool  has(void*);

    uint32_t free(void);

    MemoryPool* next;

    static MemoryPool* first;

private:
    void* base;
    uint16_t size;
};

// this overloads "placement new"
inline void* operator new(size_t nbytes, MemoryPool& pool)
{
    return pool.alloc(nbytes);
}

// this allows placement new to free memory if the constructor fails
inline void  operator delete(void* p, MemoryPool& pool)
{
    pool.dealloc(p);
}

#endif /* _MEMORYPOOL_H */
