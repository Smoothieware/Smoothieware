#ifndef _AHBMALLOC_H
#define _AHBMALLOC_H

#include <stdint.h>
#include <stdlib.h>

typedef enum {
    AHB_BANK_0,
    AHB_BANK_1,
    AHB_NUM_BANKS
} BANK;

void* ahbmalloc(size_t size, BANK bank) __attribute__ ((warning("deprecated, please use new (AHB0) blah(); or blah = AHB0.alloc(size);")));
void ahbfree(void* ptr, size_t size);

#endif /* _AHBMALLOC_H */
